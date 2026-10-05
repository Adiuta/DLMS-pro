
#pragma once

#define DLMS_FW_VERSION "2.4.1"
#define DLMS_DEVICE_NAME "DLMS 2x4"

#define DLMS_FS 48000
#define DLMS_BLOCK_FRAMES 128

#define DLMS_CORE_DSP 0
#define DLMS_CORE_UI  1

/* 24 = tertinggi yang valid (configMAX_PRIORITIES=25). Task WiFi Arduino (prioritas 23)
   tidak bisa dipindah ke core 1 pada build framework=arduino, jadi DSP harus
   di atasnya supaya tidak pernah disela WiFi. */
#define DLMS_DSP_PRIORITY 24

/* Blok diam yang diisi ke DAC sebelum loop: penyangga TX terhadap stall singkat
   (tulis flash NVS, dll). Tiap blok = 2,67 ms latensi. */
#define DLMS_TX_PREFILL_BLOCKS 4
#define DLMS_UI_PRIORITY  8
#define DLMS_WEB_PRIORITY 5

#define DLMS_CHANNELS_IN  2
#define DLMS_CHANNELS_OUT 4

#define DLMS_IN_BANDS  7
#define DLMS_OUT_BANDS 15

#define DLMS_WIFI_DEFAULT_PASSWORD "12345678"
#define DLMS_WIFI_AP_IP "192.168.240.1"
#define DLMS_WIFI_AP_GW "192.168.240.1"
#define DLMS_WIFI_AP_MASK "255.255.255.0"
#define DLMS_WIFI_AP_SSID "DLMS 240 Pro"

#define DLMS_I2S_BITS 32

#define DLMS_WIFI_STA_SSID ""
#define DLMS_WIFI_STA_PASSWORD ""

#define DLMS_WIFI_AP_PASSWORD ""
