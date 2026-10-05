#include "dlms_state.h"
#include "dsp.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "storage";

/* Called once from app_main before any other task runs. */
void storage_load(void) {
  nvs_handle_t h;
  if (nvs_open("dlms", NVS_READWRITE, &h) != ESP_OK) {
    ESP_LOGW(TAG, "nvs_open failed, using defaults");
    return;
  }

  uint8_t b;
  if (nvs_get_u8(h, "bright", &b) == ESP_OK)
    /* 0% would leave the backlight (and the menu) off */
    g_dlms.brightness = (b < 10) ? 10 : (b > 100 ? 100 : b);

  uint8_t lt;
  if (nvs_get_u8(h, "lcd_to", &lt) == ESP_OK)
    g_dlms.lcd_timeout_min = (lt > 30) ? 30 : lt;

  uint8_t w;
  if (nvs_get_u8(h, "wifi_on", &w) == ESP_OK)
    g_dlms.wifi_on = (w != 0);

  uint8_t m;
  if (nvs_get_u8(h, "wifi_mode", &m) == ESP_OK)
    g_dlms.wifi_mode =
        (m == 1) ? 1 : 0;

  char pw[33] = {0};
  size_t n = sizeof(pw);
  if (nvs_get_str(h, "wifi_pw", pw, &n) == ESP_OK && pw[0])
    snprintf(g_dlms.wifi_password, sizeof(g_dlms.wifi_password), "%s", pw);

  nvs_close(h);
}

/* Safe to call from any task. Copies the fields under the mutex first, so it
 * never reads g_dlms while another task writes it, and never holds the mutex
 * during the (slow) flash write. NVS skips values that did not change.
 * Note: a flash commit stalls cache access for a few ms; a short audio glitch
 * while saving is possible. */
void storage_save_system(void) {
  uint8_t bright, wifi_on, wifi_mode, lcd_to;
  char pw[33];

  xSemaphoreTake(g_state_mutex, portMAX_DELAY);
  bright = g_dlms.brightness;
  lcd_to = g_dlms.lcd_timeout_min;
  wifi_on = g_dlms.wifi_on ? 1 : 0;
  wifi_mode = (uint8_t)g_dlms.wifi_mode;
  snprintf(pw, sizeof(pw), "%s", g_dlms.wifi_password);
  xSemaphoreGive(g_state_mutex);

  nvs_handle_t h;
  if (nvs_open("dlms", NVS_READWRITE, &h) != ESP_OK) {
    ESP_LOGW(TAG, "nvs_open failed, settings not saved");
    return;
  }
  dsp_quiet_begin();
  esp_err_t e = nvs_set_u8(h, "bright", bright);
  if (e == ESP_OK)
    e = nvs_set_u8(h, "wifi_on", wifi_on);
  if (e == ESP_OK)
    e = nvs_set_u8(h, "wifi_mode", wifi_mode);
  if (e == ESP_OK)
    e = nvs_set_str(h, "wifi_pw", pw);
  if (e == ESP_OK)
    e = nvs_set_u8(h, "lcd_to", lcd_to);
  if (e == ESP_OK)
    e = nvs_commit(h);
  dsp_quiet_end();
  if (e != ESP_OK)
    ESP_LOGW(TAG, "save failed: %s", esp_err_to_name(e));
  nvs_close(h);
}

typedef struct {
  dlms_channel_t in[2];
  dlms_channel_t out[4];
} audio_state_blob_t;

void storage_load_audio_state(void) {
  nvs_handle_t h;
  if (nvs_open("dlms", NVS_READWRITE, &h) != ESP_OK) return;

  audio_state_blob_t blob;
  size_t len = sizeof(blob);
  if (nvs_get_blob(h, "audio_st", &blob, &len) == ESP_OK && len == sizeof(blob)) {
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    memcpy(g_dlms.in, blob.in, sizeof(g_dlms.in));
    memcpy(g_dlms.out, blob.out, sizeof(g_dlms.out));
    xSemaphoreGive(g_state_mutex);
    ESP_LOGI(TAG, "Audio state loaded");
  }
  nvs_close(h);
}

void storage_save_audio_state(void) {
  audio_state_blob_t blob;
  xSemaphoreTake(g_state_mutex, portMAX_DELAY);
  memcpy(blob.in, g_dlms.in, sizeof(blob.in));
  memcpy(blob.out, g_dlms.out, sizeof(blob.out));
  xSemaphoreGive(g_state_mutex);

  /* Lewati tulis kalau isi sama: tiap tulis blob ~2 KB hampir selalu
   * memicu erase halaman NVS (stall flash puluhan ms). */
  static audio_state_blob_t last;
  static bool have_last = false;
  if (have_last && memcmp(&last, &blob, sizeof(blob)) == 0)
    return;

  nvs_handle_t h;
  if (nvs_open("dlms", NVS_READWRITE, &h) == ESP_OK) {
    dsp_quiet_begin();
    esp_err_t e = nvs_set_blob(h, "audio_st", &blob, sizeof(blob));
    if (e == ESP_OK)
      e = nvs_commit(h);
    dsp_quiet_end();
    nvs_close(h);
    if (e == ESP_OK) {
      last = blob;
      have_last = true;
    }
    ESP_LOGI(TAG, "Audio state saved");
  }
}

bool storage_save_preset(int slot) {
  audio_state_blob_t blob;
  xSemaphoreTake(g_state_mutex, portMAX_DELAY);
  memcpy(blob.in, g_dlms.in, sizeof(blob.in));
  memcpy(blob.out, g_dlms.out, sizeof(blob.out));
  xSemaphoreGive(g_state_mutex);

  nvs_handle_t h;
  if (nvs_open("dlms", NVS_READWRITE, &h) == ESP_OK) {
    char key[16];
    snprintf(key, sizeof(key), "preset_%d", slot);
    dsp_quiet_begin();
    esp_err_t e = nvs_set_blob(h, key, &blob, sizeof(blob));
    if (e == ESP_OK) e = nvs_commit(h);
    dsp_quiet_end();
    nvs_close(h);
    return e == ESP_OK;
  }
  return false;
}

bool storage_load_preset(int slot) {
  nvs_handle_t h;
  if (nvs_open("dlms", NVS_READWRITE, &h) == ESP_OK) {
    audio_state_blob_t blob;
    size_t len = sizeof(blob);
    char key[16];
    snprintf(key, sizeof(key), "preset_%d", slot);
    if (nvs_get_blob(h, key, &blob, &len) == ESP_OK && len == sizeof(blob)) {
      xSemaphoreTake(g_state_mutex, portMAX_DELAY);
      memcpy(g_dlms.in, blob.in, sizeof(g_dlms.in));
      memcpy(g_dlms.out, blob.out, sizeof(g_dlms.out));
      xSemaphoreGive(g_state_mutex);
      nvs_close(h);
      return true;
    }
    nvs_close(h);
  }
  return false;
}

bool storage_preset_exists(int slot) {
  nvs_handle_t h;
  if (nvs_open("dlms", NVS_READWRITE, &h) == ESP_OK) {
    size_t len = 0;
    char key[16];
    snprintf(key, sizeof(key), "preset_%d", slot);
    esp_err_t e = nvs_get_blob(h, key, NULL, &len);
    nvs_close(h);
    return e == ESP_OK;
  }
  return false;
}