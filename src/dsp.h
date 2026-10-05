
#pragma once
#include <stdint.h>
#include "esp_err.h"

esp_err_t dsp_start(void);

/* Soft-mute untuk operasi yang menghentikan cache flash (tulis NVS).
 * dsp_quiet_begin(): fade-out output, tunggu sampai sunyi benar-benar sampai
 * di DAC (blokir ~80 ms). dsp_quiet_end(): fade-in lagi setelah penyangga
 * audio bersih. Boleh bersarang. Panggil dari task core 1 (UI / web). */
void dsp_quiet_begin(void);
void dsp_quiet_end(void);
