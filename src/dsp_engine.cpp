/*
 * DLMS 240 Pro — DSP Engine Implementation
 *
 * Biquad coefficients (Audio EQ Cookbook — Robert Bristow-Johnson),
 * crossover filter chains (BW / LR / Bessel), dynamics, delay.
 */
#include "dsp_engine.h"
#include "app_config.h"
#include "esp_heap_caps.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ══════════════════════════════════════════════════════════════
 *  Butterworth / Bessel Q-factor tables
 *  Index: [order-1][section].  Q == 0 → 1st-order section.
 * ══════════════════════════════════════════════════════════════ */
static const float bw_q[8][4] = {
    /* 1 */ {0},
    /* 2 */ {0.70711f},
    /* 3 */ {1.00000f, 0},
    /* 4 */ {0.54120f, 1.30656f},
    /* 5 */ {0.61803f, 1.61803f, 0},
    /* 6 */ {0.51764f, 0.70711f, 1.93185f},
    /* 7 */ {0.55496f, 0.80194f, 2.24698f, 0},
    /* 8 */ {0.50980f, 0.60134f, 0.89997f, 2.56292f},
};

static const float bes_q[8][4] = {
    /* 1 */ {0},
    /* 2 */ {0.57735f},
    /* 3 */ {0.69130f, 0},
    /* 4 */ {0.52190f, 0.80554f},
    /* 5 */ {0.56380f, 0.91656f, 0},
    /* 6 */ {0.51032f, 0.61121f, 1.02332f},
    /* 7 */ {0.53200f, 0.66091f, 1.12625f, 0},
    /* 8 */ {0.50600f, 0.55960f, 0.71107f, 1.22566f},
};

/* ── helpers ─────────────────────────────────────────────────── */

void dsp_biquad_reset(dsp_biquad_t *bq)
{
    bq->s1 = 0;
    bq->s2 = 0;
}

static void biquad_passthrough(dsp_biquad_t *bq)
{
    bq->b0 = 1.0f; bq->b1 = 0; bq->b2 = 0;
    bq->a1 = 0;    bq->a2 = 0;
}

void dsp_dc_block_init(dsp_dc_block_t *dc, float freq, float fs)
{
    dc->r = 1.0f - (2.0f * (float)M_PI * freq / fs);
    dc->x1 = 0;
    dc->y1 = 0;
}

void dsp_ch_ctx_init(dsp_ch_ctx_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    dsp_dc_block_init(&ctx->dc_block, 5.0f, (float)DLMS_FS);
    ctx->gate.gain_lin    = 1.0f;
    ctx->limiter.gain_lin = 1.0f;
    /* Set all biquads to pass-through */
    for (int i = 0; i < DSP_MAX_PEQ_BANDS; i++)
        biquad_passthrough(&ctx->peq[i]);
    for (int i = 0; i < DSP_XOVER_MAX_STAGES; i++) {
        biquad_passthrough(&ctx->xover_hp[i]);
        biquad_passthrough(&ctx->xover_lp[i]);
    }
    dsp_delay_init(&ctx->delay);
}

/* ── 1st-order filters (represented as biquad with b2=a2=0) ── */

static void calc_lp1(dsp_biquad_t *bq, float freq, float fs)
{
    float wc   = tanf((float)M_PI * freq / fs);
    float norm = 1.0f / (1.0f + wc);
    bq->b0 = wc * norm;
    bq->b1 = bq->b0;
    bq->b2 = 0;
    bq->a1 = (wc - 1.0f) * norm;
    bq->a2 = 0;
}

static void calc_hp1(dsp_biquad_t *bq, float freq, float fs)
{
    float wc   = tanf((float)M_PI * freq / fs);
    float norm = 1.0f / (1.0f + wc);
    bq->b0 =  norm;
    bq->b1 = -norm;
    bq->b2 = 0;
    bq->a1 = (wc - 1.0f) * norm;
    bq->a2 = 0;
}

/* ── 2nd-order LP / HP with given Q ──────────────────────────── */

static void calc_lp2(dsp_biquad_t *bq, float freq, float q, float fs)
{
    float w0    = 2.0f * (float)M_PI * freq / fs;
    float cw    = cosf(w0);
    float alpha = sinf(w0) / (2.0f * q);
    float inv   = 1.0f / (1.0f + alpha);
    bq->b0 = ((1.0f - cw) * 0.5f) * inv;
    bq->b1 = ( 1.0f - cw)         * inv;
    bq->b2 = bq->b0;
    bq->a1 = (-2.0f * cw)         * inv;
    bq->a2 = ( 1.0f - alpha)      * inv;
}

static void calc_hp2(dsp_biquad_t *bq, float freq, float q, float fs)
{
    float w0    = 2.0f * (float)M_PI * freq / fs;
    float cw    = cosf(w0);
    float alpha = sinf(w0) / (2.0f * q);
    float inv   = 1.0f / (1.0f + alpha);
    bq->b0 = ((1.0f + cw) * 0.5f) * inv;
    bq->b1 = (-(1.0f + cw))       * inv;
    bq->b2 = bq->b0;
    bq->a1 = (-2.0f * cw)         * inv;
    bq->a2 = ( 1.0f - alpha)      * inv;
}

/* ══════════════════════════════════════════════════════════════
 *  PEQ coefficient calculation  (Audio EQ Cookbook)
 * ══════════════════════════════════════════════════════════════ */
void dsp_calc_peq(dsp_biquad_t *bq, uint8_t type,
                  float freq, float gain_db, float q, float fs)
{
    if (freq <= 0 || freq >= fs * 0.499f || q <= 0.01f) {
        biquad_passthrough(bq);
        return;
    }

    float w0    = 2.0f * (float)M_PI * freq / fs;
    float cw    = cosf(w0);
    float sw    = sinf(w0);
    float alpha = sw / (2.0f * q);
    float A     = powf(10.0f, gain_db / 40.0f);

    float b0, b1, b2, a0, a1, a2;

    switch (type) {
    default:
    case DSP_PEQ_BELL:
        b0 = 1.0f + alpha * A;
        b1 = -2.0f * cw;
        b2 = 1.0f - alpha * A;
        a0 = 1.0f + alpha / A;
        a1 = -2.0f * cw;
        a2 = 1.0f - alpha / A;
        break;

    case DSP_PEQ_LOSHELF: {
        float sqA = sqrtf(A);
        float t   = 2.0f * sqA * alpha;
        b0 = A * ((A+1) - (A-1)*cw + t);
        b1 = 2.0f * A * ((A-1) - (A+1)*cw);
        b2 = A * ((A+1) - (A-1)*cw - t);
        a0 = (A+1) + (A-1)*cw + t;
        a1 = -2.0f * ((A-1) + (A+1)*cw);
        a2 = (A+1) + (A-1)*cw - t;
        break;
    }
    case DSP_PEQ_HISHELF: {
        float sqA = sqrtf(A);
        float t   = 2.0f * sqA * alpha;
        b0 = A * ((A+1) + (A-1)*cw + t);
        b1 = -2.0f * A * ((A-1) + (A+1)*cw);
        b2 = A * ((A+1) + (A-1)*cw - t);
        a0 = (A+1) - (A-1)*cw + t;
        a1 = 2.0f * ((A-1) - (A+1)*cw);
        a2 = (A+1) - (A-1)*cw - t;
        break;
    }
    case DSP_PEQ_NOTCH:
        b0 = 1.0f;   b1 = -2.0f*cw;   b2 = 1.0f;
        a0 = 1.0f+alpha; a1 = -2.0f*cw; a2 = 1.0f-alpha;
        break;

    case DSP_PEQ_ALLPASS:
        b0 = 1.0f-alpha; b1 = -2.0f*cw; b2 = 1.0f+alpha;
        a0 = 1.0f+alpha; a1 = -2.0f*cw; a2 = 1.0f-alpha;
        break;

    case DSP_PEQ_LOWPASS:
        calc_lp2(bq, freq, q, fs);
        return;

    case DSP_PEQ_HIGHPASS:
        calc_hp2(bq, freq, q, fs);
        return;
    }

    float inv = 1.0f / a0;
    bq->b0 = b0 * inv;  bq->b1 = b1 * inv;  bq->b2 = b2 * inv;
    bq->a1 = a1 * inv;  bq->a2 = a2 * inv;
}

/* ══════════════════════════════════════════════════════════════
 *  CROSSOVER coefficient calculation
 * ══════════════════════════════════════════════════════════════ */
static uint8_t calc_xover_chain(dsp_biquad_t bq[], bool hp,
                                const float qtab[][4],
                                float freq, uint8_t order, float fs)
{
    if (order < 1) order = 1;
    if (order > 8) order = 8;

    uint8_t n = (order + 1) / 2;  /* number of sections */
    if (n > DSP_XOVER_MAX_STAGES) n = DSP_XOVER_MAX_STAGES;

    for (uint8_t i = 0; i < n; i++) {
        float q = qtab[order - 1][i];
        if (q < 0.001f) {
            /* 1st-order section (odd order, last section) */
            if (hp) calc_hp1(&bq[i], freq, fs);
            else    calc_lp1(&bq[i], freq, fs);
        } else {
            /* 2nd-order biquad section */
            if (hp) calc_hp2(&bq[i], freq, q, fs);
            else    calc_lp2(&bq[i], freq, q, fs);
        }
    }
    return n;
}

uint8_t dsp_calc_xover(dsp_biquad_t bq[], bool highpass,
                       uint8_t type, float freq, uint8_t slope, float fs)
{
    if (freq <= 0 || slope == 0)
        return 0;
        
    /* If LP freq is very high, just pass-through (no filtering needed) */
    if (!highpass && freq >= 20000.0f)
        return 0;

    if (freq >= fs * 0.49f) freq = fs * 0.49f;

    uint8_t order = slope / 6;
    if (order < 1) order = 1;
    if (order > 8) order = 8;

    switch (type) {
    case DSP_XOVER_LR: {
        /* Linkwitz-Riley 2N = cascade of two identical BW-N filters */
        uint8_t half = order / 2;
        if (half < 1) half = 1;
        uint8_t n1 = calc_xover_chain(bq,      highpass, bw_q, freq, half, fs);
        uint8_t n2 = calc_xover_chain(bq + n1, highpass, bw_q, freq, half, fs);
        uint8_t total = n1 + n2;
        return (total > DSP_XOVER_MAX_STAGES) ? DSP_XOVER_MAX_STAGES : total;
    }
    case DSP_XOVER_BES:
        return calc_xover_chain(bq, highpass, bes_q, freq, order, fs);

    default: /* DSP_XOVER_BW */
        return calc_xover_chain(bq, highpass, bw_q, freq, order, fs);
    }
}

/* ══════════════════════════════════════════════════════════════
 *  DYNAMICS — setup (pre-compute linear thresholds)
 * ══════════════════════════════════════════════════════════════ */
static void dynamics_set_coeff(dsp_dynamics_t *d,
                               float attack_ms, float release_ms, float fs)
{
    if (attack_ms  < 0.01f) attack_ms  = 0.01f;
    if (release_ms < 0.01f) release_ms = 0.01f;
    d->att_coeff = expf(-1.0f / (fs * attack_ms  * 0.001f));
    d->rel_coeff = expf(-1.0f / (fs * release_ms * 0.001f));
}

void dsp_gate_setup(dsp_dynamics_t *g, float thr_db, float depth_db,
                    float hold_ms, float attack_ms, float release_ms, float fs)
{
    dynamics_set_coeff(g, attack_ms, release_ms, fs);
    g->thr_lin   = powf(10.0f, thr_db   / 20.0f);
    g->depth_lin = powf(10.0f, depth_db  / 20.0f);
    g->hold_samp = (uint32_t)(hold_ms * fs * 0.001f);
}

void dsp_limiter_setup(dsp_dynamics_t *l, float thr_db, float ratio, float knee_db,
                       float attack_ms, float release_ms, float fs)
{
    dynamics_set_coeff(l, attack_ms, release_ms, fs);
    l->thr_lin = powf(10.0f, thr_db / 20.0f);
    if (ratio >= 100.0f) {
        l->ratio_slope = 1.0f;
    } else if (ratio <= 1.0f) {
        l->ratio_slope = 0.0f;
    } else {
        l->ratio_slope = 1.0f - (1.0f / ratio);
    }

    if (knee_db < 0.1f) knee_db = 0.1f;
    l->knee_lo = l->thr_lin * powf(10.0f, -knee_db * 0.5f / 20.0f);
    l->knee_hi = l->thr_lin * powf(10.0f,  knee_db * 0.5f / 20.0f);
}

/* ══════════════════════════════════════════════════════════════
 *  DELAY & STATE RESET — setup
 * ══════════════════════════════════════════════════════════════ */
void dsp_delay_init(dsp_delay_t *d)
{
    if (!d) return;
    d->len = 0;
    d->pos = 0;
    d->max_samp = 0;
    d->buf = NULL;

    /* Coba alokasi 300 ms (14400 sampel = 57,6 KB) di PSRAM */
    d->buf = (float *)heap_caps_malloc(DSP_MAX_DELAY_SAMP_PSRAM * sizeof(float), MALLOC_CAP_SPIRAM);
    if (d->buf) {
        d->max_samp = DSP_MAX_DELAY_SAMP_PSRAM;
        memset(d->buf, 0, DSP_MAX_DELAY_SAMP_PSRAM * sizeof(float));
        return;
    }

    /* Fallback ke internal RAM jika PSRAM tidak tersedia (960 sampel = 20 ms) */
    d->buf = (float *)heap_caps_malloc(DSP_MAX_DELAY_SAMP_INTERNAL * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (d->buf) {
        d->max_samp = DSP_MAX_DELAY_SAMP_INTERNAL;
        memset(d->buf, 0, DSP_MAX_DELAY_SAMP_INTERNAL * sizeof(float));
    }
}

void dsp_delay_reset(dsp_delay_t *d)
{
    if (d && d->buf && d->max_samp > 0) {
        memset(d->buf, 0, d->max_samp * sizeof(float));
        d->pos = 0;
    }
}

void dsp_ch_ctx_reset_states(dsp_ch_ctx_t *ctx)
{
    if (!ctx) return;
    ctx->dc_block.x1 = 0;
    ctx->dc_block.y1 = 0;
    for (int i = 0; i < DSP_MAX_PEQ_BANDS; i++) {
        dsp_biquad_reset(&ctx->peq[i]);
    }
    for (int i = 0; i < DSP_XOVER_MAX_STAGES; i++) {
        dsp_biquad_reset(&ctx->xover_hp[i]);
        dsp_biquad_reset(&ctx->xover_lp[i]);
    }
    ctx->gate.env = 0.0f;
    ctx->gate.gain_lin = 1.0f;
    ctx->limiter.env = 0.0f;
    ctx->limiter.gain_lin = 1.0f;
    dsp_delay_reset(&ctx->delay);
}

void dsp_delay_set(dsp_delay_t *d, float ms, float fs)
{
    if (!d || !d->buf || d->max_samp == 0) {
        if (d) d->len = 0;
        return;
    }
    if (ms <= 0.0f) {
        d->len = 0;
        return;
    }
    uint32_t samp = (uint32_t)(ms * fs * 0.001f + 0.5f);
    if (samp >= d->max_samp) samp = d->max_samp - 1;

    uint16_t new_len = (uint16_t)samp;
    if (new_len > d->len) {
        /* Bersihkan sampel masa lalu yang baru tercakup agar tidak memutar isi buffer lama (Item 14) */
        uint16_t diff = new_len - d->len;
        for (uint16_t i = 1; i <= diff; i++) {
            int idx = (int)d->pos - (int)d->len - (int)i;
            while (idx < 0) idx += d->max_samp;
            if (idx < d->max_samp) d->buf[idx] = 0.0f;
        }
    }
    d->len = new_len;
}

