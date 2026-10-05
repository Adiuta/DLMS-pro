/* DLMS WIFI MODES: AP = fixed DLMS_WIFI_AP_IP (WPA2 if password >= 8 chars,
 * otherwise open); CLIENT = normal STA/DHCP. */
#include "dlms_ws.h"

#include "app_config.h"
#include "cJSON.h"
#include "dlms_state.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "wifi_web.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern "C" void mark_ui_dirty();
extern void storage_save_system(void);
extern void storage_save_audio_state(void);

static const char *TAG = "wifi_web";
static httpd_handle_t s_server = NULL;
static esp_netif_t *s_ap = NULL;
static esp_netif_t *s_sta = NULL;
static volatile bool s_wifi_reapply_pending = false;

static void update_ip_strings(void) {
  xSemaphoreTake(g_state_mutex, portMAX_DELAY);

  if (g_dlms.wifi_mode == RACK_LCD_WIFI_AP) {
    snprintf(g_dlms.ip, sizeof(g_dlms.ip), "%s", DLMS_WIFI_AP_IP);
    snprintf(g_dlms.ap_ip, sizeof(g_dlms.ap_ip), "%s", DLMS_WIFI_AP_IP);
  } else {
    esp_netif_ip_info_t info;
    if (s_sta && esp_netif_get_ip_info(s_sta, &info) == ESP_OK) {
      snprintf(g_dlms.ip, sizeof(g_dlms.ip), IPSTR, IP2STR(&info.ip));
    }
  }

  xSemaphoreGive(g_state_mutex);
}

/* Takes ownership of `root` (the old version leaked it on every request). */
static esp_err_t json_response(httpd_req_t *req, cJSON *root) {
  char *s = cJSON_PrintUnformatted(root);
  cJSON_Delete(root);
  if (!s)
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                               "no memory");
  httpd_resp_set_type(req, "application/json");
  esp_err_t e = httpd_resp_send(req, s, HTTPD_RESP_USE_STRLEN);
  cJSON_free(s);
  return e;
}

static void band_to_json(cJSON *a, const dlms_band_t *b) {
  cJSON *o = cJSON_CreateObject();
  cJSON_AddBoolToObject(o, "on", b->on);
  cJSON_AddNumberToObject(o, "f", b->f);
  cJSON_AddNumberToObject(o, "g", b->g);
  cJSON_AddNumberToObject(o, "q", b->q);
  cJSON_AddNumberToObject(o, "type", b->type);
  cJSON_AddItemToArray(a, o);
}

static cJSON *channel_json(const dlms_channel_t *c, bool input) {
  cJSON *o = cJSON_CreateObject();
  cJSON_AddStringToObject(o, "name", c->name);
  cJSON_AddBoolToObject(o, "mute", c->mute);
  cJSON_AddNumberToObject(o, "gain", c->gain);
  cJSON_AddBoolToObject(o, "invert", c->invert);
  cJSON_AddNumberToObject(o, "delay", c->delay_ms);
  cJSON_AddNumberToObject(o, "subsonic", c->subsonic_freq);
  cJSON_AddBoolToObject(o, "peqOn", c->peq_on);

  cJSON *bands = cJSON_CreateArray();
  int n = input ? DLMS_IN_BANDS : DLMS_OUT_BANDS;
  for (int i = 0; i < n; i++)
    band_to_json(bands, &c->peq[i]);
  cJSON_AddItemToObject(o, "peq", bands);

  cJSON *gate = cJSON_CreateObject();
  cJSON_AddBoolToObject(gate, "on", c->gate.on);
  cJSON_AddNumberToObject(gate, "thr", c->gate.thr);
  cJSON_AddNumberToObject(gate, "depth", c->gate.depth);
  cJSON_AddNumberToObject(gate, "attack", c->gate.attack);
  cJSON_AddNumberToObject(gate, "hold", c->gate.hold);
  cJSON_AddNumberToObject(gate, "release", c->gate.release);
  cJSON_AddItemToObject(o, "gate", gate);

  cJSON *lim = cJSON_CreateObject();
  cJSON_AddBoolToObject(lim, "on", c->limiter.on);
  cJSON_AddNumberToObject(lim, "thr", c->limiter.thr);
  cJSON_AddNumberToObject(lim, "ratio", c->limiter.ratio);
  cJSON_AddNumberToObject(lim, "attack", c->limiter.attack);
  cJSON_AddNumberToObject(lim, "release", c->limiter.release);
  cJSON_AddNumberToObject(lim, "knee", c->limiter.knee);
  cJSON_AddItemToObject(o, "lim", lim);

  if (!input) {
    cJSON_AddBoolToObject(o, "srcIn1", c->src_in1);
    cJSON_AddBoolToObject(o, "srcIn2", c->src_in2);
    cJSON_AddNumberToObject(o, "srcGain1", c->src_gain1);
    cJSON_AddNumberToObject(o, "srcGain2", c->src_gain2);

    cJSON *xo = cJSON_CreateObject();
    cJSON *hp = cJSON_CreateObject();
    cJSON_AddBoolToObject(hp, "on", c->xover.hp.on);
    cJSON_AddNumberToObject(hp, "f", c->xover.hp.f);
    cJSON_AddNumberToObject(hp, "type", c->xover.hp.type);
    cJSON_AddNumberToObject(hp, "slope", c->xover.hp.slope);
    cJSON_AddItemToObject(xo, "hp", hp);
    cJSON *lp = cJSON_CreateObject();
    cJSON_AddBoolToObject(lp, "on", c->xover.lp.on);
    cJSON_AddNumberToObject(lp, "f", c->xover.lp.f);
    cJSON_AddNumberToObject(lp, "type", c->xover.lp.type);
    cJSON_AddNumberToObject(lp, "slope", c->xover.lp.slope);
    cJSON_AddItemToObject(xo, "lp", lp);
    cJSON_AddItemToObject(o, "xover", xo);

    cJSON *fir = cJSON_CreateObject();
    cJSON_AddBoolToObject(fir, "on", c->fir.on);
    cJSON_AddNumberToObject(fir, "gain", c->fir.gain);
    cJSON_AddNumberToObject(fir, "taps", c->fir.taps);
    cJSON_AddNumberToObject(fir, "mode", c->fir.mode);
    cJSON_AddItemToObject(o, "fir", fir);
  }

  return o;
}

cJSON *api_config_build_json(void) {
  /* Copy the state under the mutex (microseconds) and build the JSON
   * outside it. Before, all of the cJSON work ran with g_state_mutex held,
   * and the DSP task takes that mutex every audio block -> dropouts. Heap
   * copy because the httpd task stack is small. */
  dlms_state_t *st = (dlms_state_t*)malloc(sizeof(*st));
  if (!st) return NULL;

  xSemaphoreTake(g_state_mutex, portMAX_DELAY);
  memcpy(st, &g_dlms, sizeof(*st));
  xSemaphoreGive(g_state_mutex);

  cJSON *root = cJSON_CreateObject();
  cJSON_AddStringToObject(root, "device", st->device_name);
  cJSON_AddStringToObject(root, "firmware", st->firmware_version);
  cJSON_AddStringToObject(root, "ip", st->ip);
  cJSON_AddStringToObject(root, "apIp", st->ap_ip);
  cJSON_AddNumberToObject(root, "sampleRate", st->fs);
  cJSON_AddNumberToObject(root, "brightness", st->brightness);
  cJSON_AddNumberToObject(root, "lcdTimeout", st->lcd_timeout_min);
  cJSON_AddBoolToObject(root, "wifiOn", st->wifi_on);
  cJSON_AddStringToObject(root, "wifiMode",
                          st->wifi_mode == RACK_LCD_WIFI_AP ? "AP" : "CLIENT");
  cJSON_AddStringToObject(root, "wifiPassword", st->wifi_password);

  cJSON *ins = cJSON_CreateArray();
  for (int i = 0; i < 2; i++)
    cJSON_AddItemToArray(ins, channel_json(&st->in[i], true));
  cJSON_AddItemToObject(root, "ins", ins);

  cJSON *outs = cJSON_CreateArray();
  for (int i = 0; i < 4; i++)
    cJSON_AddItemToArray(outs, channel_json(&st->out[i], false));
  cJSON_AddItemToObject(root, "outs", outs);

  cJSON *meters = cJSON_CreateObject();
  cJSON *in = cJSON_CreateArray();
  cJSON *out = cJSON_CreateArray();
  for (int i = 0; i < 2; i++)
    cJSON_AddItemToArray(in, cJSON_CreateNumber(st->in_peak[i]));
  for (int i = 0; i < 4; i++)
    cJSON_AddItemToArray(out, cJSON_CreateNumber(st->out_peak[i]));
  cJSON_AddItemToObject(meters, "in", in);
  cJSON_AddItemToObject(meters, "out", out);
  cJSON_AddItemToObject(root, "meters", meters);

  free(st);
  return root;
}

static esp_err_t api_get(httpd_req_t *req) {
  cJSON *root = api_config_build_json();
  if (!root) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no mem");
  return json_response(req, root);
}

static char *read_body(httpd_req_t *req) {
  if (req->content_len <= 0 || req->content_len > 16384)
    return NULL;
  char *b = (char*)calloc(1, req->content_len + 1);
  if (!b)
    return NULL;
  int off = 0;
  while (off < req->content_len) {
    int n = httpd_req_recv(req, b + off, req->content_len - off);
    if (n <= 0) {
      free(b);
      return NULL;
    }
    off += n;
  }
  return b;
}

/* ── validated JSON field readers ─────────────────────────────
 * Wrong type or NaN/Inf is ignored; numbers are clamped. Before, anything
 * from the network went straight into g_dlms: a gain of 1e30 becomes inf in
 * powf() and NaN samples in the DAC path. */
static bool get_f(const cJSON *o, const char *k, float *dst, float lo,
                  float hi) {
  const cJSON *v = cJSON_GetObjectItem(o, k);
  if (!cJSON_IsNumber(v) || !isfinite(v->valuedouble))
    return false;
  float f = (float)v->valuedouble;
  *dst = f < lo ? lo : (f > hi ? hi : f);
  return true;
}

static bool get_b(const cJSON *o, const char *k, bool *dst) {
  const cJSON *v = cJSON_GetObjectItem(o, k);
  if (!cJSON_IsBool(v))
    return false;
  *dst = cJSON_IsTrue(v);
  return true;
}

static bool get_u8(const cJSON *o, const char *k, uint8_t *dst, int lo,
                   int hi) {
  float f;
  if (!get_f(o, k, &f, (float)lo, (float)hi))
    return false;
  *dst = (uint8_t)lroundf(f);
  return true;
}

static bool get_snapped(const cJSON *o, const char *k, const int *tab, int n,
                        int *out) {
  float f;
  if (!get_f(o, k, &f, (float)tab[0], (float)tab[n - 1]))
    return false;
  int best = 0;
  for (int i = 1; i < n; i++)
    if (fabsf(f - (float)tab[i]) < fabsf(f - (float)tab[best]))
      best = i;
  *out = tab[best];
  return true;
}

static const int k_slopes[] = {6, 12, 18, 24, 36, 48};
static const int k_taps[] = {128, 256, 512, 1024};

static void apply_xover_sec(bool *on, float *f, uint8_t *type, uint8_t *slope,
                            const cJSON *sec) {
  int t;
  get_b(sec, "on", on);
  get_f(sec, "f", f, 20, 20000);
  get_u8(sec, "type", type, 0, 2);
  if (get_snapped(sec, "slope", k_slopes, 6, &t))
    *slope = (uint8_t)t;
}

static void json_apply_channel(dlms_channel_t *c, const cJSON *o, bool input) {
  const cJSON *v;
  get_b(o, "mute", &c->mute);
  get_f(o, "gain", &c->gain, -80, 12);
  get_b(o, "invert", &c->invert);
  get_f(o, "delay", &c->delay_ms, 0, 300);
  float sub = 0;
  if (get_f(o, "subsonic", &sub, 0, 200))
    c->subsonic_freq = (int)sub;
  get_b(o, "peqOn", &c->peq_on);

  v = cJSON_GetObjectItem(o, "peq");
  if (cJSON_IsArray(v)) {
    int n = input ? DLMS_IN_BANDS : DLMS_OUT_BANDS;
    for (int i = 0; i < n; i++) {
      const cJSON *b = cJSON_GetArrayItem(v, i);
      if (!b)
        continue;
      get_b(b, "on", &c->peq[i].on);
      get_f(b, "f", &c->peq[i].f, 20, 20000);
      get_f(b, "g", &c->peq[i].g, -24, 24);
      get_f(b, "q", &c->peq[i].q, 0.1f, 20);
      get_u8(b, "type", &c->peq[i].type, 0, 6);
    }
  }

  v = cJSON_GetObjectItem(o, "gate");
  if (v) {
    get_b(v, "on", &c->gate.on);
    get_f(v, "thr", &c->gate.thr, -90, 0);
    get_f(v, "depth", &c->gate.depth, -90, 0);
    get_f(v, "attack", &c->gate.attack, 0.1f, 500);
    get_f(v, "hold", &c->gate.hold, 0, 2000);
    get_f(v, "release", &c->gate.release, 1, 5000);
  }

  v = cJSON_GetObjectItem(o, "lim");
  if (v) {
    get_b(v, "on", &c->limiter.on);
    get_f(v, "thr", &c->limiter.thr, -40, 12);
    get_f(v, "ratio", &c->limiter.ratio, 1, 100);
    get_f(v, "attack", &c->limiter.attack, 0.1f, 500);
    get_f(v, "release", &c->limiter.release, 1, 5000);
    get_f(v, "knee", &c->limiter.knee, 0, 24);
  }

  if (!input) {
    get_b(o, "srcIn1", &c->src_in1);
    get_b(o, "srcIn2", &c->src_in2);
    get_f(o, "srcGain1", &c->src_gain1, -80, 12);
    get_f(o, "srcGain2", &c->src_gain2, -80, 12);

    v = cJSON_GetObjectItem(o, "xover");
    if (v) {
      const cJSON *sec = cJSON_GetObjectItem(v, "hp");
      if (sec)
        apply_xover_sec(&c->xover.hp.on, &c->xover.hp.f, &c->xover.hp.type,
                        &c->xover.hp.slope, sec);
      sec = cJSON_GetObjectItem(v, "lp");
      if (sec)
        apply_xover_sec(&c->xover.lp.on, &c->xover.lp.f, &c->xover.lp.type,
                        &c->xover.lp.slope, sec);
    }
    v = cJSON_GetObjectItem(o, "fir");
    if (v) {
      int t;
      get_b(v, "on", &c->fir.on);
      get_f(v, "gain", &c->fir.gain, -24, 24);
      if (get_snapped(v, "taps", k_taps, 4, &t))
        c->fir.taps = (uint16_t)t;
      get_u8(v, "mode", &c->fir.mode, 0, 2);
    }
  }
}

/* Re-applies Wi-Fi a moment after the HTTP response went out: switching
 * mode/password drops the very connection the request came in on. */
static void wifi_configure(bool on, uint8_t mode,
                           const char *password);

static void wifi_reapply_task(void *arg) {
  (void)arg;
  vTaskDelay(pdMS_TO_TICKS(500));

  xSemaphoreTake(g_state_mutex, portMAX_DELAY);
  bool on = g_dlms.wifi_on;
  uint8_t mode = g_dlms.wifi_mode;
  char pw[33];
  snprintf(pw, sizeof(pw), "%s", g_dlms.wifi_password);
  xSemaphoreGive(g_state_mutex);

  wifi_configure(on, mode, pw);
  s_wifi_reapply_pending = false;
  vTaskDelete(NULL);
}

static esp_err_t api_put(httpd_req_t *req) {
  char *body = read_body(req);
  if (!body)
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");

  cJSON *root = cJSON_Parse(body);
  free(body);
  if (!root)
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid json");

  bool system_changed = false, wifi_changed = false, audio_changed = false;

  xSemaphoreTake(g_state_mutex, portMAX_DELAY);

  cJSON *v = cJSON_GetObjectItem(root, "deviceName");
  if (cJSON_IsString(v) && v->valuestring[0])
    snprintf(g_dlms.device_name, sizeof(g_dlms.device_name), "%s",
             v->valuestring);

  uint8_t b;
  if (get_u8(root, "brightness", &b, 10, 100)) {
    if (b != g_dlms.brightness) {
      g_dlms.brightness = b;
      system_changed = true;
    }
  }

  uint8_t lt;
  if (get_u8(root, "lcdTimeout", &lt, 0, 30)) {
    if (lt != g_dlms.lcd_timeout_min) {
      g_dlms.lcd_timeout_min = lt;
      system_changed = true;
    }
  }

  bool on;
  if (get_b(root, "wifiOn", &on) && on != g_dlms.wifi_on) {
    g_dlms.wifi_on = on;
    system_changed = wifi_changed = true;
  }

  v = cJSON_GetObjectItem(root, "wifiMode");
  if (cJSON_IsString(v)) {
    uint8_t m = (strcmp(v->valuestring, "CLIENT") == 0)
                                 ? RACK_LCD_WIFI_CLIENT
                                 : RACK_LCD_WIFI_AP;
    if (m != g_dlms.wifi_mode) {
      g_dlms.wifi_mode = m;
      system_changed = wifi_changed = true;
    }
  }

  v = cJSON_GetObjectItem(root, "wifiPassword");
  if (cJSON_IsString(v) && strcmp(v->valuestring, g_dlms.wifi_password) != 0) {
    snprintf(g_dlms.wifi_password, sizeof(g_dlms.wifi_password), "%s",
             v->valuestring);
    system_changed = wifi_changed = true;
  }

  v = cJSON_GetObjectItem(root, "ins");
  if (cJSON_IsArray(v)) {
    audio_changed = true;
    for (int i = 0; i < 2; i++) {
      cJSON *o = cJSON_GetArrayItem(v, i);
      if (o)
        json_apply_channel(&g_dlms.in[i], o, true);
    }
  }

  v = cJSON_GetObjectItem(root, "outs");
  if (cJSON_IsArray(v)) {
    audio_changed = true;
    for (int i = 0; i < 4; i++) {
      cJSON *o = cJSON_GetArrayItem(v, i);
      if (o)
        json_apply_channel(&g_dlms.out[i], o, false);
    }
  }

  xSemaphoreGive(g_state_mutex);
  cJSON_Delete(root);

  /* The physical panel kept showing the old values after a web change. */
  mark_ui_dirty();
  
  dlms_ws_broadcast_state();
  if (system_changed)
    storage_save_system();
  if (audio_changed)
    storage_save_audio_state();

  httpd_resp_set_type(req, "application/json");
  esp_err_t e = httpd_resp_sendstr(req, "{\"ok\":true}");

  if (wifi_changed && !s_wifi_reapply_pending) {
    s_wifi_reapply_pending = true;
    if (xTaskCreatePinnedToCore(wifi_reapply_task, "wifi_apply", 4096, NULL, 4,
                            NULL, 1) !=
        pdPASS)
      s_wifi_reapply_pending = false;
  }
  return e;
}

/* index.html disisipkan sebagai array oleh src/CMakeLists.txt (index_html_gen.cpp) */
extern "C" const uint8_t index_html_data[];
extern "C" const uint32_t index_html_len;
#define index_html_start index_html_data
#define index_html_end (index_html_data + index_html_len)

static esp_err_t api_root(httpd_req_t *req) {
  size_t len = index_html_end - index_html_start;
  if (len && index_html_start[len - 1] == 0)
    len--; /* EMBED_TXTFILES appends a NUL */
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, (const char *)index_html_start, len);
}

static void start_http(void) {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 80;
  cfg.stack_size = 8192;       /* default 4 KB is tight for cJSON + WS */
  cfg.core_id = 1;             /* keep Core 0 for the DSP */
  cfg.lru_purge_enable = true; /* drop the oldest socket when full: browsers
                                  reconnect a lot and leave dead ones */
  if (httpd_start(&s_server, &cfg) != ESP_OK) {
    s_server = NULL;
    ESP_LOGE(TAG, "httpd_start failed");
    return;
  }

  dlms_ws_register_uri(s_server);
  dlms_ws_attach_server(s_server);

  httpd_uri_t root = {.uri = "/", .method = HTTP_GET, .handler = api_root};
  httpd_uri_t get = {
      .uri = "/api/state", .method = HTTP_GET, .handler = api_get};
  httpd_uri_t put = {
      .uri = "/api/state", .method = HTTP_PUT, .handler = api_put};

  httpd_register_uri_handler(s_server, &root);
  httpd_register_uri_handler(s_server, &get);
  httpd_register_uri_handler(s_server, &put);
}

/* IP_EVENT_STA_GOT_IP (id 0) used to be tested inside `base==WIFI_EVENT`, where
 * id 0 is WIFI_EVENT_WIFI_READY: GOT_IP was never handled in CLIENT mode. */
static void wifi_event(void *arg, esp_event_base_t base, int32_t id,
                       void *data) {
  (void)arg;
  (void)data;

  if (base == WIFI_EVENT) {
    switch (id) {
    case WIFI_EVENT_STA_START:
      esp_wifi_connect();
      break;
    case WIFI_EVENT_STA_DISCONNECTED:
      if (g_dlms.wifi_on && g_dlms.wifi_mode == RACK_LCD_WIFI_CLIENT)
        esp_wifi_connect();
      break;
    case WIFI_EVENT_AP_START:
      update_ip_strings();
      if (!s_server)
        start_http();
      
      break;
    default:
      break;
    }
  } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
    update_ip_strings();
    if (!s_server)
      start_http();
    
  }
}

static void wifi_configure(bool on, uint8_t mode,
                           const char *password) {
  esp_wifi_stop();
  if (!on)
    return;

  wifi_config_t cfg = {0};

  if (mode == RACK_LCD_WIFI_AP) {
    snprintf((char *)cfg.ap.ssid, sizeof(cfg.ap.ssid), "%s", DLMS_WIFI_AP_SSID);
    snprintf((char *)cfg.ap.password, sizeof(cfg.ap.password), "%s", password);
    cfg.ap.ssid_len = strlen((char *)cfg.ap.ssid);
    cfg.ap.channel = 1;
    cfg.ap.max_connection = 4;
    cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    if (strlen(password) < 8)
      cfg.ap.authmode = WIFI_AUTH_OPEN; /* WPA2 needs >= 8 */
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &cfg);
  } else {
    /* NOTE: there is no setting for the router's SSID. This still joins a
     * network named DLMS_WIFI_AP_SSID; add a stored STA SSID + UI/API. */
    snprintf((char *)cfg.sta.ssid, sizeof(cfg.sta.ssid), "%s",
             DLMS_WIFI_AP_SSID);
    snprintf((char *)cfg.sta.password, sizeof(cfg.sta.password), "%s",
             password);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &cfg);
  }

  esp_wifi_start();
  esp_wifi_set_ps(
      WIFI_PS_NONE); /* power-save adds tens of ms to the live meters */
  update_ip_strings();
}

void wifi_apply_from_lcd(bool on, uint8_t mode,
                         const char *password) {
  xSemaphoreTake(g_state_mutex, portMAX_DELAY);
  g_dlms.wifi_on = on;
  g_dlms.wifi_mode = mode;
  snprintf(g_dlms.wifi_password, sizeof(g_dlms.wifi_password), "%s",
           password ? password : "");
  xSemaphoreGive(g_state_mutex);

  wifi_configure(on, mode, password ? password : "");
}

/* Make the AP really answer on DLMS_WIFI_AP_IP. The default AP netif is
 * 192.168.4.1, while update_ip_strings() reported DLMS_WIFI_AP_IP on the LCD
 * and in /api/state. */
static void configure_ap_netif(void) {
  esp_netif_ip_info_t ip = {0};
  ip.ip.addr = esp_ip4addr_aton(DLMS_WIFI_AP_IP);
  ip.gw.addr = ip.ip.addr;
  ip.netmask.addr = esp_ip4addr_aton("255.255.255.0");
  esp_netif_dhcps_stop(s_ap);
  esp_netif_set_ip_info(s_ap, &ip);
  esp_netif_dhcps_start(s_ap);
}

void wifi_web_start(void) {
  esp_err_t netif_err = esp_netif_init();
  if (netif_err != ESP_OK && netif_err != ESP_ERR_INVALID_STATE)
    ESP_ERROR_CHECK(netif_err);
  esp_err_t e = esp_event_loop_create_default();
  if (e != ESP_OK && e != ESP_ERR_INVALID_STATE)
    ESP_ERROR_CHECK(e);

  s_ap = esp_netif_create_default_wifi_ap();
  s_sta = esp_netif_create_default_wifi_sta();
  configure_ap_netif();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  esp_wifi_set_storage(
      WIFI_STORAGE_RAM); /* do not rewrite NVS on every set_config */

  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                             &wifi_event, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                             &wifi_event, NULL));

  xSemaphoreTake(g_state_mutex, portMAX_DELAY);
  bool on = g_dlms.wifi_on;
  uint8_t mode = g_dlms.wifi_mode;
  char pw[33];
  snprintf(pw, sizeof(pw), "%s", g_dlms.wifi_password);
  xSemaphoreGive(g_state_mutex);

  wifi_configure(on, mode, pw);
}