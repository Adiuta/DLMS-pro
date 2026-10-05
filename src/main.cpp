#include <Arduino.h>
#include "config.h"
#include "encoder.h"
#include "ui.h"
#include "dlms_state.h"
#include "wifi_web.h"
#include "nvs_flash.h"
#include "esp_system.h"

#include "dsp.h"
#include "audio.h"

extern void storage_load(void);
extern void storage_load_audio_state(void);

static Encoder encoder;

extern "C" void mark_ui_dirty() {
  ui::set_dirty();
}

extern "C" void mark_audio_dirty() {
  ui::schedule_autosave();
}

static const char *reset_reason_str(esp_reset_reason_t r) {
  switch (r) {
  case ESP_RST_POWERON:   return "POWERON";
  case ESP_RST_SW:        return "SW (esp_restart)";
  case ESP_RST_PANIC:     return "PANIC (crash/abort)";
  case ESP_RST_INT_WDT:   return "INT_WDT";
  case ESP_RST_TASK_WDT:  return "TASK_WDT";
  case ESP_RST_WDT:       return "WDT lain";
  case ESP_RST_BROWNOUT:  return "BROWNOUT (catu daya drop!)";
  case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
  default:                return "lainnya";
  }
}

void setup() {
  pinMode(PIN_TFT_BL, OUTPUT);
  analogWriteFrequency(25000); // 25kHz PWM → di atas batas dengar, hilangkan noise "nginggg"
  analogWrite(PIN_TFT_BL, 0); // Matikan layar secepat mungkin untuk menghindari flash putih

  Serial.begin(115200);
  Serial.println("\n" FW_NAME " - BOOTING...");
  Serial.printf("Reset reason: %s\n", reset_reason_str(esp_reset_reason()));

  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  dlms_state_init();
  storage_load();
  storage_load_audio_state();

  Serial.println("Starting UI (LCD)...");
  ui::begin();
  Serial.println("LCD OK! Starting encoder...");
  encoder.begin();
  Serial.println("Encoder OK!");

  Serial.println("Starting wifi_web...");
  wifi_web_start();
  Serial.println("WiFi OK!");


  Serial.println("Starting DSP (core 0, termasuk init I2S)...");
  esp_err_t dspErr = dsp_start();
  if (dspErr != ESP_OK) Serial.printf("dsp_start err: %d\n", dspErr);

  Serial.println("Setup done!");
}

void loop() {
  Input in = encoder.poll();
  if (in.any()) Serial.printf("enc raw=%d accel=%d click=%d long=%d\n", in.raw, in.accel, in.click, in.longPress);

  ui::handle(in);
  ui::render();
  delay(1);
}
