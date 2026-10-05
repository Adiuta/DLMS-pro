#include "esp_err.h"
#include "dlms_state.h"
#include <string.h>
#include <stdio.h>
#include <math.h>


dlms_state_t g_dlms;
SemaphoreHandle_t g_state_mutex;

static const float in_freq[DLMS_IN_BANDS] =
    {160,400,1000,2500,6300};

static const float out_freq[DLMS_OUT_BANDS] =
    {31.5f,50,80,125,200,315,500,800,1250,2000,3150,5000,8000,12500,16000};

static void init_channel(dlms_channel_t *c, const char *name, bool input, int idx)
{
    memset(c,0,sizeof(*c));
    snprintf(c->name,sizeof(c->name),"%s",name);
    c->peq_on=false;
    c->limiter.on=false;
    c->limiter.thr=0.0f;
    c->limiter.ratio=100.0f;  // INF:1 default
    c->limiter.attack=5.0f;
    c->limiter.release=150.0f;
    c->limiter.knee=2.0f;
    c->gate.on=false;
    c->gate.thr=-80.0f;
    c->gate.depth=-30.0f;
    c->gate.attack=3.0f;
    c->gate.hold=40.0f;
    c->gate.release=250.0f;
    c->delay_ms=0;
    c->subsonic_freq=0;  // 0 = off

    int n=input ? DLMS_IN_BANDS : DLMS_OUT_BANDS;
    for(int i=0;i<n;i++){
        c->peq[i].on=false;
        c->peq[i].f=input ? in_freq[i] : out_freq[i];
        c->peq[i].g=0;
        c->peq[i].q=1.4f;
        c->peq[i].type=0;
    }
    if(!input){
        c->src_in1=(idx % 2 == 0); // OUT1,3 = L
        c->src_in2=(idx % 2 == 1); // OUT2,4 = R
        c->src_gain1 = c->src_in1 ? 1.0f : 0.0f;
        c->src_gain2 = c->src_in2 ? 1.0f : 0.0f;
        c->xover.hp.on=false;
        c->xover.lp.on=false;
        c->xover.hp.f=20;
        c->xover.lp.f=20000;
        c->xover.hp.type=0; c->xover.lp.type=0;
        c->xover.hp.slope=24; c->xover.lp.slope=24;
        c->fir.taps=512;
        c->fir.mode=0;
    }
}

void dlms_state_init(void)
{
    memset(&g_dlms,0,sizeof(g_dlms));
    snprintf(g_dlms.device_name,sizeof(g_dlms.device_name),"%s",DLMS_DEVICE_NAME);
    snprintf(g_dlms.firmware_version,sizeof(g_dlms.firmware_version),"%s",DLMS_FW_VERSION);
    snprintf(g_dlms.ap_ip,sizeof(g_dlms.ap_ip),"%s",DLMS_WIFI_AP_IP);
    snprintf(g_dlms.wifi_password,sizeof(g_dlms.wifi_password),"%s",DLMS_WIFI_DEFAULT_PASSWORD);
    g_dlms.brightness=80;
    g_dlms.lcd_timeout_min=2;  // default 2 menit auto-off
    g_dlms.wifi_on=true;
    g_dlms.wifi_mode=RACK_LCD_WIFI_AP;
    g_dlms.fs=DLMS_FS;

    init_channel(&g_dlms.in[0],"IN 1",true,0);
    init_channel(&g_dlms.in[1],"IN 2",true,1);
    init_channel(&g_dlms.out[0],"OUT 1",false,0);
    init_channel(&g_dlms.out[1],"OUT 2",false,1);
    init_channel(&g_dlms.out[2],"OUT 3",false,2);
    init_channel(&g_dlms.out[3],"OUT 4",false,3);

    g_state_mutex=xSemaphoreCreateMutex();
}




void dlms_state_publish_vu(void)
{
    /* UI is updated by input_ui.c after reading the protected state. */
}


const dlms_state_t *dlms_state_get(void)
{
    return &g_dlms;
}

esp_err_t dlms_state_set_path_number(const char *path, float value)
{
    if (!path) return ESP_ERR_INVALID_ARG;

    int ch;
    char prop[32];
    esp_err_t r = ESP_ERR_NOT_FOUND;

    /* Dipanggil dari task httpd; DSP/UI membaca g_dlms di task lain. */
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    if (sscanf(path, "outs.%d.%31s", &ch, prop) == 2 && ch >= 0 && ch < 4) {
        r = ESP_OK;
        if (strcmp(prop, "gain") == 0) g_dlms.out[ch].gain = value;
        else if (strcmp(prop, "srcIn1") == 0) g_dlms.out[ch].src_in1 = (value > 0.5f);
        else if (strcmp(prop, "srcIn2") == 0) g_dlms.out[ch].src_in2 = (value > 0.5f);
        else r = ESP_ERR_NOT_FOUND;
    } else if (sscanf(path, "ins.%d.%31s", &ch, prop) == 2 && ch >= 0 && ch < 2) {
        r = ESP_OK;
        if (strcmp(prop, "gain") == 0) g_dlms.in[ch].gain = value;
        else r = ESP_ERR_NOT_FOUND;
    }
    xSemaphoreGive(g_state_mutex);
    return r;
}
