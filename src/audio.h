#pragma once
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include <stddef.h>
#include <stdint.h>

esp_err_t audio_init(void);
esp_err_t audio_read(int32_t *buf, size_t bytes, size_t *got, TickType_t timeout);
esp_err_t audio_write_dac1(const int32_t *buf, size_t bytes, size_t *sent, TickType_t timeout);
esp_err_t audio_write_dac2(const int32_t *buf, size_t bytes, size_t *sent, TickType_t timeout);
