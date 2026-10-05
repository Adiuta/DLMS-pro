#include "dlms_ws.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include <Arduino.h>
#include "cJSON.h"
#include "dlms_state.h"
#include "pins.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "dlms_ws";

extern "C" void mark_ui_dirty();
extern "C" void mark_audio_dirty();
extern bool storage_save_preset(int slot);
extern bool storage_load_preset(int slot);
extern void storage_save_system(void);
extern void storage_save_audio_state(void);

#define WS_MAX_CLIENTS 8
#define WS_MAX_RX_BYTES 1024

static httpd_handle_t s_server = NULL;
static int s_ws_fds[WS_MAX_CLIENTS];
static size_t s_ws_count = 0;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

/* ── client list (touched by httpd task AND by meter/other tasks) ─ */

static void ws_add_client(int fd) {
  bool added = false;
  taskENTER_CRITICAL(&s_mux);
  bool found = false;
  for (size_t i = 0; i < s_ws_count; ++i)
    if (s_ws_fds[i] == fd) {
      found = true;
      break;
    }
  if (!found && s_ws_count < WS_MAX_CLIENTS) {
    s_ws_fds[s_ws_count++] = fd;
    added = true;
  }
  taskEXIT_CRITICAL(&s_mux);
  if (added)
    ESP_LOGI(TAG, "Studio WebSocket connected fd=%d", fd);
}

static void ws_remove_client(int fd) {
  bool removed = false;
  taskENTER_CRITICAL(&s_mux);
  for (size_t i = 0; i < s_ws_count; ++i) {
    if (s_ws_fds[i] == fd) {
      s_ws_fds[i] = s_ws_fds[s_ws_count - 1];
      --s_ws_count;
      removed = true;
      break;
    }
  }
  taskEXIT_CRITICAL(&s_mux);
  if (removed)
    ESP_LOGI(TAG, "Studio WebSocket removed fd=%d", fd);
}

static size_t ws_snapshot(int out[WS_MAX_CLIENTS]) {
  taskENTER_CRITICAL(&s_mux);
  size_t n = s_ws_count;
  memcpy(out, s_ws_fds, n * sizeof(int));
  taskEXIT_CRITICAL(&s_mux);
  return n;
}

size_t dlms_ws_client_count(void) {
  taskENTER_CRITICAL(&s_mux);
  size_t n = s_ws_count;
  taskEXIT_CRITICAL(&s_mux);
  return n;
}

static esp_err_t ws_send_text(int fd, const char *text) {
  if (!s_server || !text)
    return ESP_ERR_INVALID_ARG;

  httpd_ws_frame_t frame = {
      .final = true,
      .fragmented = false,
      .type = HTTPD_WS_TYPE_TEXT,
      .payload = (uint8_t *)text,
      .len = strlen(text),
  };
  return httpd_ws_send_frame_async(s_server, fd, &frame);
}

/* Send to every live client. Dead sockets are dropped here; before, the fd
 * list was never cleaned, so after 8 connects no new client could register
 * and stale/reused fds kept receiving frames. */
static void ws_broadcast_text(const char *text) {
  if (!s_server || !text)
    return;
  int fds[WS_MAX_CLIENTS];
  size_t n = ws_snapshot(fds);
  for (size_t i = 0; i < n; ++i) {
    if (httpd_ws_get_fd_info(s_server, fds[i]) != HTTPD_WS_CLIENT_WEBSOCKET) {
      ws_remove_client(fds[i]);
      continue;
    }
    ws_send_text(fds[i], text);
  }
}

static void ws_broadcast_json(cJSON *root) {
  char *json = cJSON_PrintUnformatted(root);
  if (json) {
    ws_broadcast_text(json);
    free(json);
  }
}

static inline float round1(float v) { return roundf(v * 10.0f) / 10.0f; }



static void send_hello(int fd) {
  extern cJSON *api_config_build_json(void);
  cJSON *root = api_config_build_json();
  if (root) {
    cJSON_AddStringToObject(root, "type", "hello");
    char *json = cJSON_PrintUnformatted(root);
    if (json) {
      ws_send_text(fd, json);
      free(json);
    }
    cJSON_Delete(root);
  }
}

static esp_err_t ws_handler(httpd_req_t *req) {
  int fd = httpd_req_to_sockfd(req);

  if (req->method == HTTP_GET) {
    ws_add_client(fd);
    send_hello(fd);
    return ESP_OK;
  }

  httpd_ws_frame_t frame = {0};
  frame.type = HTTPD_WS_TYPE_TEXT;

  esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);
  if (err != ESP_OK)
    return err;

  if (frame.type == HTTPD_WS_TYPE_CLOSE) {
    ws_remove_client(fd);
    return ESP_OK;
  }
  if (frame.type != HTTPD_WS_TYPE_TEXT || frame.len == 0)
    return ESP_OK;

  /* A client could announce a huge frame and exhaust the heap. */
  if (frame.len > WS_MAX_RX_BYTES)
    return ESP_ERR_INVALID_SIZE;

  uint8_t *buf = (uint8_t*)calloc(1, frame.len + 1);
  if (!buf)
    return ESP_ERR_NO_MEM;

  frame.payload = buf;
  err = httpd_ws_recv_frame(req, &frame, frame.len);
  if (err == ESP_OK) {
    cJSON *root = cJSON_Parse((char *)buf);
    if (root) {
      const cJSON *type = cJSON_GetObjectItem(root, "type");

      if (cJSON_IsString(type) && strcmp(type->valuestring, "get_state") == 0) {
        dlms_ws_broadcast_state();
      } else if (cJSON_IsString(type) &&
                 strcmp(type->valuestring, "set") == 0) {
        const cJSON *path = cJSON_GetObjectItem(root, "path");
        const cJSON *value = cJSON_GetObjectItem(root, "value");

        if (cJSON_IsString(path) && cJSON_IsNumber(value)) {
          float v = (float)value->valuedouble;
          /* Only echo a change that was actually accepted. */
          if (dlms_state_set_path_number(path->valuestring, v) == ESP_OK) {
            dlms_ws_broadcast_change(path->valuestring, v);
            mark_ui_dirty();
            if (strcmp(path->valuestring, "system.brightness") == 0) {
              analogWrite(PIN_TFT_BL, g_dlms.brightness);
              storage_save_system();
            } else if (strncmp(path->valuestring, "system.", 7) == 0) {
              storage_save_system();
            } else {
              mark_audio_dirty();
            }
          }
        }
      } else if (cJSON_IsString(type) && strcmp(type->valuestring, "preset_save") == 0) {
        const cJSON *slot = cJSON_GetObjectItem(root, "slot");
        if (cJSON_IsNumber(slot)) {
          if (storage_save_preset((int)slot->valuedouble)) {
            dlms_ws_broadcast_state();
          }
        }
      } else if (cJSON_IsString(type) && strcmp(type->valuestring, "preset_load") == 0) {
        const cJSON *slot = cJSON_GetObjectItem(root, "slot");
        if (cJSON_IsNumber(slot)) {
          if (storage_load_preset((int)slot->valuedouble)) {
            dlms_ws_broadcast_state();
            mark_ui_dirty();
          }
        }
      } else if (cJSON_IsString(type) && strcmp(type->valuestring, "copy_channel") == 0) {
        const cJSON *src = cJSON_GetObjectItem(root, "src");
        const cJSON *dst = cJSON_GetObjectItem(root, "dst");
        if (cJSON_IsNumber(src) && cJSON_IsNumber(dst)) {
          int s = (int)src->valuedouble;
          int d = (int)dst->valuedouble;
          xSemaphoreTake(g_state_mutex, portMAX_DELAY);
          if (s < 2 && d < 2) {
            char tmp[20]; strncpy(tmp, g_dlms.in[d].name, sizeof(tmp));
            g_dlms.in[d] = g_dlms.in[s];
            strncpy(g_dlms.in[d].name, tmp, sizeof(g_dlms.in[d].name));
          } else if (s >= 2 && d >= 2) {
            int so = s - 2, do_ch = d - 2;
            char tmp[20]; strncpy(tmp, g_dlms.out[do_ch].name, sizeof(tmp));
            g_dlms.out[do_ch] = g_dlms.out[so];
            strncpy(g_dlms.out[do_ch].name, tmp, sizeof(g_dlms.out[do_ch].name));
          }
          xSemaphoreGive(g_state_mutex);
          storage_save_audio_state();
          dlms_ws_broadcast_state();
          mark_ui_dirty();
        }
      } else if (cJSON_IsString(type) && strcmp(type->valuestring, "reset") == 0) {
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        dlms_state_init();
        xSemaphoreGive(g_state_mutex);
        storage_save_audio_state();
        dlms_ws_broadcast_state();
        mark_ui_dirty();
      }
      cJSON_Delete(root);
    }
  }

  free(buf);
  return err;
}

esp_err_t dlms_ws_register_uri(httpd_handle_t server) {
  if (!server)
    return ESP_ERR_INVALID_ARG;

  httpd_uri_t ws_uri = {
      .uri = "/ws",
      .method = HTTP_GET,
      .handler = ws_handler,
      .is_websocket = true,
      /* false: the server answers PING/CLOSE itself. With true the handler
       * must reply to them, which this handler did not do. */
      .handle_ws_control_frames = false,
  };
  return httpd_register_uri_handler(server, &ws_uri);
}

esp_err_t dlms_ws_init(void) {
  /* The HTTP server is started by wifi_web.c. */
  return ESP_OK;
}

esp_err_t dlms_ws_attach_server(httpd_handle_t server) {
  s_server = server;
  return ESP_OK;
}

esp_err_t dlms_ws_broadcast_state(void) {
  if (!s_server)
    return ESP_ERR_INVALID_STATE;
  if (dlms_ws_client_count() == 0)
    return ESP_OK;

  extern cJSON *api_config_build_json(void);
  cJSON *root = api_config_build_json();
  if (root) {
    cJSON_AddStringToObject(root, "type", "full_state");
    ws_broadcast_json(root);
    cJSON_Delete(root);
  }
  return ESP_OK;
}

esp_err_t dlms_ws_broadcast_meter(const float vu_db[6]) {
  if (!s_server || !vu_db)
    return ESP_ERR_INVALID_ARG;
  if (dlms_ws_client_count() == 0)
    return ESP_OK;

  cJSON *root = cJSON_CreateObject();
  cJSON_AddStringToObject(root, "type", "meter");
  cJSON *arr = cJSON_CreateArray();
  for (int i = 0; i < 6; ++i)
    cJSON_AddItemToArray(arr, cJSON_CreateNumber(round1(vu_db[i])));
  cJSON_AddItemToObject(root, "vu", arr);

  ws_broadcast_json(root);
  cJSON_Delete(root);
  return ESP_OK;
}

esp_err_t dlms_ws_broadcast_change(const char *path, float value) {
  if (!s_server || !path)
    return ESP_ERR_INVALID_ARG;
  if (dlms_ws_client_count() == 0)
    return ESP_OK;

  cJSON *root = cJSON_CreateObject();
  cJSON_AddStringToObject(root, "type", "change");
  cJSON_AddStringToObject(root, "path", path);
  cJSON_AddNumberToObject(root, "value", value);

  ws_broadcast_json(root);
  cJSON_Delete(root);
  return ESP_OK;
}

esp_err_t dlms_ws_broadcast_meter_rta(const float vu_db[6], const float *rta_db,
                                      size_t rta_count) {
  if (!s_server || !vu_db || !rta_db)
    return ESP_ERR_INVALID_ARG;
  if (dlms_ws_client_count() == 0)
    return ESP_OK;

  cJSON *root = cJSON_CreateObject();
  cJSON_AddStringToObject(root, "type", "meter");

  cJSON *vu = cJSON_CreateArray();
  for (int i = 0; i < 6; i++)
    cJSON_AddItemToArray(vu, cJSON_CreateNumber(round1(vu_db[i])));
  cJSON_AddItemToObject(root, "vu", vu);

  cJSON *gr = cJSON_CreateArray();
  cJSON_AddItemToArray(gr, cJSON_CreateNumber(round1(g_dlms.in_gr[0])));
  cJSON_AddItemToArray(gr, cJSON_CreateNumber(round1(g_dlms.in_gr[1])));
  cJSON_AddItemToArray(gr, cJSON_CreateNumber(round1(g_dlms.out_gr[0])));
  cJSON_AddItemToArray(gr, cJSON_CreateNumber(round1(g_dlms.out_gr[1])));
  cJSON_AddItemToArray(gr, cJSON_CreateNumber(round1(g_dlms.out_gr[2])));
  cJSON_AddItemToArray(gr, cJSON_CreateNumber(round1(g_dlms.out_gr[3])));
  cJSON_AddItemToObject(root, "gr", gr);

  cJSON *pre = cJSON_CreateArray();
  cJSON_AddItemToArray(pre, cJSON_CreateNumber(round1(g_dlms.in_peak[0])));
  cJSON_AddItemToArray(pre, cJSON_CreateNumber(round1(g_dlms.in_peak[1])));
  cJSON_AddItemToArray(pre, cJSON_CreateNumber(round1(g_dlms.out_peak_pre[0])));
  cJSON_AddItemToArray(pre, cJSON_CreateNumber(round1(g_dlms.out_peak_pre[1])));
  cJSON_AddItemToArray(pre, cJSON_CreateNumber(round1(g_dlms.out_peak_pre[2])));
  cJSON_AddItemToArray(pre, cJSON_CreateNumber(round1(g_dlms.out_peak_pre[3])));
  cJSON_AddItemToObject(root, "pre", pre);

  cJSON *rta = cJSON_CreateArray();
  for (size_t i = 0; i < rta_count; i++)
    cJSON_AddItemToArray(rta, cJSON_CreateNumber(round1(rta_db[i])));
  cJSON_AddItemToObject(root, "rta", rta);

  ws_broadcast_json(root);
  cJSON_Delete(root);
  return ESP_OK;
}