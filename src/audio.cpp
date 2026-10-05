#include <Arduino.h>
/*
 * Audio I/O — driver I2S legacy (ESP-IDF 4.4 / Arduino core 2.0.x).
 *
 *  I2S0 master full duplex : PCM1801 ADC (RX) + PCM5102 #1 (TX), MCLK 256fs
 *                            ke SCKI PCM1801. Pin BCK/LRCK = clock untuk semua.
 *  I2S1 SLAVE TX saja      : PCM5102 #2; BCK/LRCK diambil dari pin yang sama
 *                            lewat GPIO matrix (pin tetap disetir I2S0).
 *
 * Perbaikan vs versi lama:
 *  - mck_io_num tidak diisi (=0) -> MCLK keluar di GPIO0, ADC tanpa SCKI.
 *  - I2S1 menyetir pin BCK/LRCK yang sama dengan I2S0 -> pin direbut.
 *  - init gagal separuh jalan tidak dibersihkan.
 */
#include "app_config.h"
#include "audio.h"
#include "config.h"
#include "driver/i2s.h"
#include "esp_err.h"
#include "esp_rom_gpio.h"
#include "freertos/FreeRTOS.h"
#include "soc/gpio_periph.h"
#include "soc/i2s_periph.h"
#include "soc/io_mux_reg.h"
#include <stddef.h>
#include "driver/gpio.h"

/* Kekuatan drive pin clock/data I2S. Default ESP32 = CAP_2 (~20 mA). Karena
 * Anda sudah memasang resistor seri 47 ohm di hardware, kita kembalikan
 * drive strength ke CAP_2 agar sinyal clock cukup kuat untuk DAC. Kombinasi
 * CAP_0 dan 47 ohm bisa membuat sinyal terlalu lemah/lambat sehingga DAC
 * (PCM5102) kehilangan sinkronisasi (suara terputus/mute sementara). */
#define I2S_PIN_DRIVE GPIO_DRIVE_CAP_2

static bool s_i2s0 = false, s_i2s1 = false;

static esp_err_t init_i2s0(void) {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX);
  cfg.sample_rate = DLMS_I2S_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 16;
  cfg.dma_buf_len = DLMS_BLOCK_FRAMES;
  cfg.use_apll = false; /* ESP32-S3 tidak punya APLL (flag diabaikan) */
  cfg.tx_desc_auto_clear = true;
  cfg.fixed_mclk = 0;
  cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;

  i2s_pin_config_t pins = {};
  pins.mck_io_num = PIN_ADC_SCK;
  pins.bck_io_num = PIN_ADC_BCK;
  pins.ws_io_num = PIN_ADC_LRC;
  pins.data_out_num = PIN_DAC1_DIN;
  pins.data_in_num = PIN_ADC_DOUT;

  esp_err_t e = i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
  if (e != ESP_OK)
    return e;
  e = i2s_set_pin(I2S_NUM_0, &pins);
  if (e != ESP_OK) {
    i2s_driver_uninstall(I2S_NUM_0);
    return e;
  }
  s_i2s0 = true;
  return ESP_OK;
}

static esp_err_t init_i2s1(void) {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_SLAVE | I2S_MODE_TX);
  cfg.sample_rate = DLMS_I2S_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 16;
  cfg.dma_buf_len = DLMS_BLOCK_FRAMES;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = true;

  /* BCK/WS sengaja NO_CHANGE: pin-nya milik I2S0 (master). */
  i2s_pin_config_t pins = {};
  pins.mck_io_num = I2S_PIN_NO_CHANGE;
  pins.bck_io_num = I2S_PIN_NO_CHANGE;
  pins.ws_io_num = I2S_PIN_NO_CHANGE;
  pins.data_out_num = PIN_DAC2_DIN;
  pins.data_in_num = I2S_PIN_NO_CHANGE;

  esp_err_t e = i2s_driver_install(I2S_NUM_1, &cfg, 0, NULL);
  if (e != ESP_OK)
    return e;
  e = i2s_set_pin(I2S_NUM_1, &pins);
  if (e != ESP_OK) {
    i2s_driver_uninstall(I2S_NUM_1);
    return e;
  }

  /* Umpan pin BCK/LRCK (yang disetir I2S0) ke input clock I2S1.
   * Hanya input-enable + matrix-in; output I2S0 tidak disentuh. */
  PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[PIN_ADC_BCK]);
  PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[PIN_ADC_LRC]);
  esp_rom_gpio_connect_in_signal(
      PIN_ADC_BCK, i2s_periph_signal[I2S_NUM_1].s_tx_bck_sig, false);
  esp_rom_gpio_connect_in_signal(
      PIN_ADC_LRC, i2s_periph_signal[I2S_NUM_1].s_tx_ws_sig, false);
  s_i2s1 = true;
  return ESP_OK;
}

esp_err_t audio_init(void) {
  Serial.println("audio_init: I2S0 (ADC+DAC1)...");
  esp_err_t e = init_i2s0();
  if (e != ESP_OK) {
    Serial.printf("audio_init: I2S0 gagal: %d\n", e);
    return e;
  }
  Serial.println("audio_init: I2S1 slave (DAC2)...");
  e = init_i2s1();
  if (e != ESP_OK) {
    Serial.printf("audio_init: I2S1 gagal: %d\n", e);
    i2s_driver_uninstall(I2S_NUM_0);
    s_i2s0 = false;
    return e;
  }
  /* Setelah i2s_set_pin (yang mengatur ulang pin) */
  gpio_set_drive_capability((gpio_num_t)PIN_ADC_SCK, I2S_PIN_DRIVE);
  gpio_set_drive_capability((gpio_num_t)PIN_ADC_BCK, I2S_PIN_DRIVE);
  gpio_set_drive_capability((gpio_num_t)PIN_ADC_LRC, I2S_PIN_DRIVE);
  gpio_set_drive_capability((gpio_num_t)PIN_DAC1_DIN, I2S_PIN_DRIVE);
  gpio_set_drive_capability((gpio_num_t)PIN_DAC2_DIN, I2S_PIN_DRIVE);

  /* Mulai serempak: slave siap dulu, baru master mengeluarkan clock.
   * Supaya DAC1 & DAC2 tidak bergeser satu blok. */
  i2s_stop(I2S_NUM_0);
  i2s_stop(I2S_NUM_1);
  i2s_zero_dma_buffer(I2S_NUM_0);
  i2s_zero_dma_buffer(I2S_NUM_1);
  i2s_start(I2S_NUM_1);
  i2s_start(I2S_NUM_0);
  Serial.println("audio_init: OK");
  return ESP_OK;
}

esp_err_t audio_read(int32_t *buf, size_t bytes, size_t *got,
                     TickType_t timeout) {
  if (got)
    *got = 0;
  if (!s_i2s0)
    return ESP_ERR_INVALID_STATE;
  return i2s_read(I2S_NUM_0, buf, bytes, got, timeout);
}

esp_err_t audio_write_dac1(const int32_t *buf, size_t bytes, size_t *sent,
                           TickType_t timeout) {
  if (sent)
    *sent = 0;
  if (!s_i2s0)
    return ESP_ERR_INVALID_STATE;
  return i2s_write(I2S_NUM_0, buf, bytes, sent, timeout);
}

esp_err_t audio_write_dac2(const int32_t *buf, size_t bytes, size_t *sent,
                           TickType_t timeout) {
  if (sent)
    *sent = 0;
  if (!s_i2s1)
    return ESP_ERR_INVALID_STATE;
  return i2s_write(I2S_NUM_1, buf, bytes, sent, timeout);
}