/*
 * DLMS 240 Pro — DSP Engine
 *
 * Biquad filters, dynamics (gate/limiter), delay lines.
 * All per-sample functions are inline for real-time performance.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#define DSP_MAX_PEQ_BANDS    15
#define DSP_XOVER_MAX_STAGES  4

/* ── Biquad Filter (Direct Form II Transposed) ───────────────── */
typedef struct {
    float b0, b1, b2, a1, a2;   /* coefficients (a0 normalised to 1) */
    float s1, s2;                /* state variables */
} dsp_biquad_t;

/* ── Dynamics Processor (gate / limiter) ─────────────────────── */
typedef struct {
    float    env;          /* envelope (linear) */
    float    gain_lin;     /* current applied gain (linear 0..1) */
    uint32_t hold_cnt;     /* gate hold counter (samples) */
    float    att_coeff;    /* attack smoothing coefficient */
    float    rel_coeff;    /* release smoothing coefficient */
    /* Pre-computed per-parameter-change (no transcendentals per sample) */
    float    thr_lin;      /* threshold (linear) */
    float    depth_lin;    /* gate depth (linear, e.g. 0.032 for -30 dB) */
    uint32_t hold_samp;    /* gate hold time (samples) */
    float    knee_lo;      /* limiter soft-knee lower bound (linear) */
    float    knee_hi;      /* limiter soft-knee upper bound (linear) */
    float    ratio_slope;  /* limiter ratio slope (1.0 - 1.0/R) */
} dsp_dynamics_t;

/* ── DC Blocker (1st order High-Pass) ────────────────────────── */
typedef struct {
    float x1;
    float y1;
    float r;
} dsp_dc_block_t;

#define DSP_MAX_DELAY_SAMP_INTERNAL  960   /* 20 ms @ 48 kHz (internal RAM) */
#define DSP_MAX_DELAY_SAMP_PSRAM    14400  /* 300 ms @ 48 kHz (PSRAM) */
#define DSP_MAX_DELAY_SAMP          DSP_MAX_DELAY_SAMP_PSRAM

/* ── Delay Line ──────────────────────────────────────────────── */
typedef struct {
    float   *buf;          /* delay buffer pointer (internal RAM or PSRAM) */
    uint16_t len;          /* delay length (samples) */
    uint16_t max_samp;     /* maximum buffer capacity */
    uint16_t pos;          /* write position */
} dsp_delay_t;

/* ── Per-channel Processing Context ──────────────────────────── */
typedef struct {
    dsp_dc_block_t dc_block;
    
    dsp_biquad_t   peq[DSP_MAX_PEQ_BANDS];
    uint8_t        peq_count;

    dsp_biquad_t   xover_hp[DSP_XOVER_MAX_STAGES];
    uint8_t        xover_hp_n;

    dsp_biquad_t   xover_lp[DSP_XOVER_MAX_STAGES];
    uint8_t        xover_lp_n;

    dsp_dynamics_t gate;
    dsp_dynamics_t limiter;
    dsp_delay_t    delay;
} dsp_ch_ctx_t;

/* ── PEQ filter types (match rack_lcd_ui.h) ── */
#define DSP_PEQ_BELL     0
#define DSP_PEQ_LOSHELF  1
#define DSP_PEQ_HISHELF  2
#define DSP_PEQ_NOTCH    3
#define DSP_PEQ_ALLPASS  4
#define DSP_PEQ_LOWPASS  5
#define DSP_PEQ_HIGHPASS 6

/* ── Crossover types ── */
#define DSP_XOVER_LR   0   /* Linkwitz-Riley */
#define DSP_XOVER_BW   1   /* Butterworth */
#define DSP_XOVER_BES  2   /* Bessel */

/* ══════════════════════════════════════════════════════════════
 *  COEFFICIENT CALCULATION (called when parameters change)
 * ══════════════════════════════════════════════════════════════ */

void dsp_biquad_reset(dsp_biquad_t *bq);
void dsp_dc_block_init(dsp_dc_block_t *dc, float freq, float fs);
void dsp_ch_ctx_init(dsp_ch_ctx_t *ctx);

void dsp_calc_peq(dsp_biquad_t *bq, uint8_t type,
                  float freq, float gain_db, float q, float fs);

/* Returns the number of biquad stages written to bq[]. */
uint8_t dsp_calc_xover(dsp_biquad_t bq[], bool highpass,
                       uint8_t type, float freq, uint8_t slope, float fs);

void dsp_gate_setup(dsp_dynamics_t *g, float thr_db, float depth_db,
                    float hold_ms, float attack_ms, float release_ms, float fs);

void dsp_limiter_setup(dsp_dynamics_t *l, float thr_db, float ratio, float knee_db,
                       float attack_ms, float release_ms, float fs);

void dsp_delay_init(dsp_delay_t *d);
void dsp_delay_reset(dsp_delay_t *d);
void dsp_ch_ctx_reset_states(dsp_ch_ctx_t *ctx);
void dsp_delay_set(dsp_delay_t *d, float ms, float fs);

/* ══════════════════════════════════════════════════════════════
 *  PER-SAMPLE PROCESSING (inlined for real-time performance)
 * ══════════════════════════════════════════════════════════════ */

static inline float dsp_biquad_tick(dsp_biquad_t *bq, float x)
{
    float y  = bq->b0 * x + bq->s1;
    bq->s1   = bq->b1 * x - bq->a1 * y + bq->s2;
    bq->s2   = bq->b2 * x - bq->a2 * y;
    return y;
}

static inline float dsp_biquad_chain_tick(dsp_biquad_t *bq, uint8_t n, float x)
{
    for (uint8_t i = 0; i < n; i++)
        x = dsp_biquad_tick(&bq[i], x);
    return x;
}

/* DC Blocker */
static inline float dsp_dc_block_tick(dsp_dc_block_t *dc, float x)
{
    float y = x - dc->x1 + dc->r * dc->y1;
    dc->x1 = x;
    dc->y1 = y;
    return y;
}

/* Gate — no transcendentals per sample */
static inline float dsp_gate_tick(dsp_dynamics_t *g, float x)
{
    float x_abs = fabsf(x);
    float target;

    if (x_abs >= g->thr_lin) {
        target = 1.0f;
        g->hold_cnt = g->hold_samp;
    } else if (g->hold_cnt > 0) {
        target = 1.0f;
        g->hold_cnt--;
    } else {
        target = g->depth_lin;
    }

    float coeff = (target < g->gain_lin) ? g->rel_coeff : g->att_coeff;
    g->gain_lin = coeff * g->gain_lin + (1.0f - coeff) * target;
    return x * g->gain_lin;
}

/* Limiter — no transcendentals per sample */
static inline float dsp_limiter_tick(dsp_dynamics_t *l, float x)
{
    float x_abs = fabsf(x);

    /* Peak envelope follower */
    float ec = (x_abs > l->env) ? l->att_coeff : l->rel_coeff;
    l->env = ec * l->env + (1.0f - ec) * x_abs;

    /* Gain computation with soft knee (all linear domain) */
    float gain_target;
    if (l->env > l->knee_hi) {
        float raw_gain = l->thr_lin / (l->env + 1e-12f);
        if (l->ratio_slope >= 0.99f) gain_target = raw_gain;
        else gain_target = powf(raw_gain, l->ratio_slope);
    } else if (l->env > l->knee_lo) {
        float t = (l->env - l->knee_lo) / (l->knee_hi - l->knee_lo + 1e-12f);
        t = t * t;   /* quadratic knee transition */
        float raw_gain = l->thr_lin / (l->env + 1e-12f);
        float full = (l->ratio_slope >= 0.99f) ? raw_gain : powf(raw_gain, l->ratio_slope);
        gain_target = 1.0f + t * (full - 1.0f);
    } else {
        gain_target = 1.0f;
    }

    /* Smooth gain transition */
    float gc = (gain_target < l->gain_lin) ? l->att_coeff : l->rel_coeff;
    l->gain_lin = gc * l->gain_lin + (1.0f - gc) * gain_target;
    return x * l->gain_lin;
}

/* Delay line */
static inline float dsp_delay_tick(dsp_delay_t *d, float x)
{
    if (d->len == 0 || !d->buf || d->max_samp == 0) return x;
    int rd = (int)d->pos - (int)d->len;
    if (rd < 0) rd += d->max_samp;
    float out = d->buf[rd];
    d->buf[d->pos] = x;
    if (++d->pos >= d->max_samp) d->pos = 0;
    return out;
}
