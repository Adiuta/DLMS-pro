#include "esp_err.h"

#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "app_config.h"

#define RACK_LCD_WIFI_AP 0
#define RACK_LCD_WIFI_CLIENT 1

typedef struct {
    bool on;
    float f;
    float g;
    float q;
    uint8_t type;
} dlms_band_t;

typedef struct {
    bool on;
    float thr;
    float depth;
    float attack;
    float hold;
    float release;
} dlms_gate_t;

typedef struct {
    bool on;
    float thr;
    float ratio;
    float attack;
    float release;
    float knee;
} dlms_limiter_t;

typedef struct {
    bool on;
    float f;
    uint8_t type;
    uint8_t slope;
} dlms_xover_sec_t;

typedef struct {
    dlms_xover_sec_t hp;
    dlms_xover_sec_t lp;
} dlms_xover_t;

typedef struct {
    bool on;
    float gain;
    uint16_t taps;
    uint8_t mode;
} dlms_fir_t;

typedef struct {
    char name[20];
    bool mute;
    float gain;
    bool invert;
    float delay_ms;
    int subsonic_freq;      // HPF low-cut frequency (0=off, 20-200 Hz)
    dlms_band_t peq[DLMS_OUT_BANDS];
    bool peq_on;
    dlms_gate_t gate;
    dlms_limiter_t limiter;
    dlms_xover_t xover;
    dlms_fir_t fir;
    bool src_in1;
    bool src_in2;
    float src_gain1;
    float src_gain2;
} dlms_channel_t;

typedef struct {
    char device_name[21];
    char firmware_version[16];
    char ip[16];
    char ap_ip[16];
    uint8_t brightness;
    uint8_t lcd_timeout_min;  // 0=always on, 1-30 min auto-off
    bool wifi_on;
    uint8_t wifi_mode;
    char wifi_password[33];

    uint32_t fs;
    float in_peak[2];
    float in_peak_post[2];
    float in_gr[2];
    float out_peak_pre[4];
    float out_peak[4];
    float out_gr[4];

    dlms_channel_t in[2];
    dlms_channel_t out[4];
} dlms_state_t;

extern dlms_state_t g_dlms;
extern SemaphoreHandle_t g_state_mutex;

void dlms_state_init(void);
void dlms_state_publish_vu(void);


/* WebSocket transport helpers. */
const dlms_state_t *dlms_state_get(void);
esp_err_t dlms_state_set_path_number(const char *path, float value);
