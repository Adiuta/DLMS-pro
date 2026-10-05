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

    esp_err_t r = ESP_ERR_NOT_FOUND;

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);

    if (strncmp(path, "system.", 7) == 0) {
        r = ESP_OK;
        const char *p = path + 7;
        if (strcmp(p, "brightness") == 0) {
            uint8_t b = (uint8_t)value;
            if (b < 10) b = 10;
            g_dlms.brightness = b;
        } else if (strcmp(p, "lcdTimeout") == 0) {
            g_dlms.lcd_timeout_min = (uint8_t)value;
        } else {
            r = ESP_ERR_NOT_FOUND;
        }
    } else {
        bool is_in = (strncmp(path, "ins.", 4) == 0);
        bool is_out = (strncmp(path, "outs.", 5) == 0);
        if (is_in || is_out) {
            const char *rest = is_in ? (path + 4) : (path + 5);
            int ch = atoi(rest);
            const char *dot = strchr(rest, '.');
            if (dot && ((is_in && ch >= 0 && ch < 2) || (is_out && ch >= 0 && ch < 4))) {
                dlms_channel_t *c = is_in ? &g_dlms.in[ch] : &g_dlms.out[ch];
                const char *p = dot + 1;
                r = ESP_OK;

                if (strcmp(p, "gain") == 0) c->gain = value;
                else if (strcmp(p, "mute") == 0) c->mute = (value > 0.5f);
                else if (strcmp(p, "invert") == 0) c->invert = (value > 0.5f);
                else if (strcmp(p, "delay") == 0) c->delay_ms = value;
                else if (strcmp(p, "subsonic") == 0) c->subsonic_freq = (int)value;
                else if (strcmp(p, "peqOn") == 0) c->peq_on = (value > 0.5f);
                else if (strncmp(p, "peq.", 4) == 0) {
                    int b_idx = atoi(p + 4);
                    const char *b_dot = strchr(p + 4, '.');
                    int max_bands = is_in ? DLMS_IN_BANDS : DLMS_OUT_BANDS;
                    if (b_dot && b_idx >= 0 && b_idx < max_bands) {
                        const char *bp = b_dot + 1;
                        if (strcmp(bp, "on") == 0) {
                            c->peq[b_idx].on = (value > 0.5f);
                            if (c->peq[b_idx].on) c->peq_on = true;
                        }
                        else if (strcmp(bp, "f") == 0) c->peq[b_idx].f = value;
                        else if (strcmp(bp, "g") == 0) {
                            c->peq[b_idx].g = value;
                            if (value != 0.0f) {
                                c->peq[b_idx].on = true;
                                c->peq_on = true;
                            }
                        }
                        else if (strcmp(bp, "q") == 0) c->peq[b_idx].q = value;
                        else if (strcmp(bp, "type") == 0) c->peq[b_idx].type = (uint8_t)value;
                        else r = ESP_ERR_NOT_FOUND;
                    } else r = ESP_ERR_NOT_FOUND;
                }
                else if (strncmp(p, "gate.", 5) == 0) {
                    const char *gp = p + 5;
                    if (strcmp(gp, "on") == 0) c->gate.on = (value > 0.5f);
                    else if (strcmp(gp, "thr") == 0) c->gate.thr = value;
                    else if (strcmp(gp, "depth") == 0) c->gate.depth = value;
                    else if (strcmp(gp, "attack") == 0) c->gate.attack = value;
                    else if (strcmp(gp, "hold") == 0) c->gate.hold = value;
                    else if (strcmp(gp, "release") == 0) c->gate.release = value;
                    else r = ESP_ERR_NOT_FOUND;
                }
                else if (strncmp(p, "lim.", 4) == 0) {
                    const char *lp = p + 4;
                    if (strcmp(lp, "on") == 0) c->limiter.on = (value > 0.5f);
                    else if (strcmp(lp, "thr") == 0) {
                        c->limiter.thr = value;
                        c->limiter.on = (value < 0.0f || c->limiter.on);
                    }
                    else if (strcmp(lp, "ratio") == 0) c->limiter.ratio = value;
                    else if (strcmp(lp, "attack") == 0) c->limiter.attack = value;
                    else if (strcmp(lp, "release") == 0) c->limiter.release = value;
                    else if (strcmp(lp, "knee") == 0) c->limiter.knee = value;
                    else r = ESP_ERR_NOT_FOUND;
                }
                else if (is_out && strncmp(p, "xover.", 6) == 0) {
                    const char *xp = p + 6;
                    if (strncmp(xp, "hp.", 3) == 0) {
                        const char *hpp = xp + 3;
                        if (strcmp(hpp, "on") == 0) c->xover.hp.on = (value > 0.5f);
                        else if (strcmp(hpp, "f") == 0) c->xover.hp.f = value;
                        else if (strcmp(hpp, "type") == 0) {
                            c->xover.hp.type = (uint8_t)value;
                            c->xover.hp.on = (c->xover.hp.type != 0);
                        }
                        else if (strcmp(hpp, "slope") == 0) c->xover.hp.slope = (uint8_t)value;
                        else r = ESP_ERR_NOT_FOUND;
                    } else if (strncmp(xp, "lp.", 3) == 0) {
                        const char *lpp = xp + 3;
                        if (strcmp(lpp, "on") == 0) c->xover.lp.on = (value > 0.5f);
                        else if (strcmp(lpp, "f") == 0) c->xover.lp.f = value;
                        else if (strcmp(lpp, "type") == 0) {
                            c->xover.lp.type = (uint8_t)value;
                            c->xover.lp.on = (c->xover.lp.type != 0);
                        }
                        else if (strcmp(lpp, "slope") == 0) c->xover.lp.slope = (uint8_t)value;
                        else r = ESP_ERR_NOT_FOUND;
                    } else r = ESP_ERR_NOT_FOUND;
                }
                else if (is_out && strcmp(p, "srcIn1") == 0) {
                    c->src_in1 = (value > 0.5f);
                    c->src_gain1 = c->src_in1 ? 1.0f : 0.0f;
                }
                else if (is_out && strcmp(p, "srcIn2") == 0) {
                    c->src_in2 = (value > 0.5f);
                    c->src_gain2 = c->src_in2 ? 1.0f : 0.0f;
                }
                else if (is_out && strcmp(p, "srcGain1") == 0) c->src_gain1 = value;
                else if (is_out && strcmp(p, "srcGain2") == 0) c->src_gain2 = value;
                else r = ESP_ERR_NOT_FOUND;
            }
        }
    }

    xSemaphoreGive(g_state_mutex);
    return r;
}

