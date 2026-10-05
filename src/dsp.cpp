/*
 * DLMS 240 Pro — Real-time DSP Core
 *
 * Signal chain:
 *   Input:  ADC → PEQ → Delay → Gate → Limiter → Gain/Mute/Invert
 *   Route:  src_in1 × gain1 + src_in2 × gain2
 *   Output: Crossover HP/LP → PEQ → Delay → Gate → Limiter
 *           → Gain/Mute/Invert → DAC
 *
 * DSP task: Core 0, high priority. Audio path only.
 * Meter task: Core 1, low priority. Builds JSON / RTA and talks to the
 * WebSocket, so the audio task never allocates memory or touches sockets.
 */
#include "dsp.h"
#include "app_config.h"
#include "audio.h"
#include "dlms_state.h"
#include "dlms_ws.h"
#include "dsp_engine.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Audio I/O (audio.c) */
extern esp_err_t audio_read(int32_t *buf, size_t bytes, size_t *got,
                            TickType_t timeout);
extern esp_err_t audio_write_dac1(const int32_t *buf, size_t bytes,
                                  size_t *sent, TickType_t timeout);
extern esp_err_t audio_write_dac2(const int32_t *buf, size_t bytes,
                                  size_t *sent, TickType_t timeout);
/* dlms_ws.c — add this prototype to dlms_ws.h */
extern size_t dlms_ws_client_count(void);

/* 1 = cetak waktu proses DSP ke Serial tiap 1 detik (untuk cek beban CPU) */
#define DSP_LOAD_LOG 1

/* TES: 1 = paksa semua output full-range tanpa crossover:
 *      OUT1/OUT3 = IN1, OUT2/OUT4 = IN2 (stereo lurus lewat DSP).
 *      Pakai untuk memeriksa apakah suara penuh keluar. Setelah selesai
 *      set ke 0 supaya crossover/routing dari web UI dipakai lagi. */
#define DSP_FORCE_FULLRANGE 0

/* TES ADC: 1 = tiap 1 detik cetak frekuensi nada, level, dan SINAD dari
 * sinyal ADC mentah. Beri nada sinus murni (sama di L dan R). Butuh
 * DSP_LOAD_LOG 1. */
#define DSP_SINAD 0

/* Level puncak sejak cetakan terakhir (dBFS): IN1 IN2 OUT1..OUT4 */
static volatile float s_vu_hold[6] = {-90, -90, -90, -90, -90, -90};

/* Daftar band PEQ yang benar-benar aktif per channel. Band dengan gain 0 dB
 * (bell/shelf) tidak mengubah sinyal, jadi dilewati: ini memangkas puluhan
 * biquad per sampel dan menurunkan beban CPU. */
typedef struct {
  uint8_t idx[DSP_MAX_PEQ_BANDS];
  uint8_t n;
  uint16_t mask;
} peq_list_t;

static peq_list_t s_in_peq[DLMS_CHANNELS_IN];
static peq_list_t s_out_peq[DLMS_CHANNELS_OUT];

/* Statistik waktu proses per blok (dibaca meter task) */
static volatile uint32_t s_proc_sum = 0, s_proc_cnt = 0, s_proc_max = 0;
/* Penghitung error I/O (dibaca meter task): RX gagal, TX DAC1, TX DAC2, mutex miss */
static volatile uint32_t s_err_rx = 0, s_err_tx1 = 0, s_err_tx2 = 0, s_mtx_miss = 0;
/* Stall: jeda antar blok > 3x periode blok (cache flash mati, dsb).
 * Kalau pop terdengar dan angka ini naik -> penyebabnya stall, bukan analog. */
static volatile uint32_t s_stall_cnt = 0, s_stall_max_us = 0;
/* Spike 1-sampel dari ADC yang diperbaiki (bit slip / gangguan BCK) */
#define DSP_GLITCH_REPAIR 1
static volatile uint32_t s_glitch_cnt = 0;

/* ── Soft-mute (dipakai saat tulis NVS) ─────────────────────── */
static volatile int s_quiet_req = 0;       /* jumlah permintaan aktif */
static volatile bool s_quiet_silent = false; /* DSP: fade sudah 0 */
#define QUIET_RAMP_SAMPLES 240              /* 5 ms */
#define QUIET_HOLD_BLOCKS 24                /* ~64 ms sunyi setelah selesai */
#define TX_LATENCY_MS 50                    /* 16 buffer DMA x 2,67 ms + margin */

/* ── Static DSP context (too large for stack) ───────────────── */
static dsp_ch_ctx_t s_in_ctx[DLMS_CHANNELS_IN];
static dsp_ch_ctx_t s_out_ctx[DLMS_CHANNELS_OUT];
static dsp_biquad_t s_subsonic[DLMS_CHANNELS_IN]; // HPF low-cut per input
static bool s_subsonic_on[DLMS_CHANNELS_IN];

/* ── Snapshots for change detection ──────────────────────────── */
static dlms_channel_t s_prev_in[DLMS_CHANNELS_IN];
static dlms_channel_t s_prev_out[DLMS_CHANNELS_OUT];
static bool s_first_update = true;

/* ── Processed input buffer (per-channel, mono samples) ──── */
static float s_proc_in[DLMS_CHANNELS_IN][DLMS_BLOCK_FRAMES];
static float s_proc_out[DLMS_CHANNELS_OUT][DLMS_BLOCK_FRAMES];

/* ── Data handed from DSP task to meter task ─────────────────── */
#define DLMS_RTA_BANDS 32
#define RTA_N 2048         /* power of two */
#define METER_PERIOD_MS 50 /* 20 Hz to the web UI */

static float s_rta_ring[RTA_N]; /* mono mix of the ADC input */
static volatile uint32_t s_rta_wr;
static volatile float s_vu[6]; /* IN1 IN2 OUT1..OUT4, dBFS */

/* ── Helpers ─────────────────────────────────────────────────── */

static float clampf(float x, float a, float b) {
  return x < a ? a : (x > b ? b : x);
}

static float db_to_lin(float db) { return powf(10.0f, db / 20.0f); }

static float lin_to_db(float lin) {
  return lin > 1e-9f ? 20.0f * log10f(lin) : -90.0f;
}

/* Bell/shelf dengan gain ~0 dB = filter netral, tidak perlu dijalankan */
static bool peq_is_identity(const dlms_band_t *b) {
  return b->type <= DSP_PEQ_HISHELF && fabsf(b->g) < 0.05f;
}

static void build_peq(dsp_ch_ctx_t *ctx, peq_list_t *lst,
                      const dlms_channel_t *ch, int nb, float fs) {
  const uint16_t old_mask = lst->mask;
  uint16_t new_mask = 0;
  uint8_t n = 0;

  if (ch->peq_on) {
    for (int i = 0; i < nb; i++) {
      const dlms_band_t *b = &ch->peq[i];
      if (!b->on || peq_is_identity(b))
        continue;
      /* Band yang baru aktif: mulai dari state bersih (hindari klik) */
      if (!(old_mask & (1u << i)))
        dsp_biquad_reset(&ctx->peq[i]);
      dsp_calc_peq(&ctx->peq[i], b->type, b->f, b->g, b->q, fs);
      lst->idx[n++] = (uint8_t)i;
      new_mask |= (uint16_t)(1u << i);
    }
  }
  lst->n = n;
  lst->mask = new_mask;
  ctx->peq_count = (uint8_t)nb;
}

static void update_input_coeffs(dsp_ch_ctx_t *ctx, peq_list_t *pl,
                                const dlms_channel_t *ch, int ch_idx) {
  const float fs = (float)DLMS_FS;

  if (ch->subsonic_freq > 0) {
    s_subsonic_on[ch_idx] = true;
    /* BW24 High-Pass -> 2 stages of HP2 */
    // Actually dsp_calc_xover does multiple stages, we only reserved 1 biquad in s_subsonic array?
    // Wait, let's just use dsp_calc_peq with DSP_PEQ_HIGHPASS
    dsp_calc_peq(&s_subsonic[ch_idx], 6 /* DSP_PEQ_HIGHPASS */, ch->subsonic_freq, 0, 0.707f, fs);
  } else {
    s_subsonic_on[ch_idx] = false;
  }

  build_peq(ctx, pl, ch, DLMS_IN_BANDS, fs);

  if (ch->gate.on)
    dsp_gate_setup(&ctx->gate, ch->gate.thr, ch->gate.depth, ch->gate.hold,
                   ch->gate.attack, ch->gate.release, fs);

  if (ch->limiter.on)
    dsp_limiter_setup(&ctx->limiter, ch->limiter.thr, ch->limiter.ratio, ch->limiter.knee,
                      ch->limiter.attack, ch->limiter.release, fs);

  dsp_delay_set(&ctx->delay, ch->delay_ms, fs);
}

/* Map UI xover type enum to DSP engine type:
 * UI: FT_OFF=0, FT_BW12=1, FT_BW18=2, FT_BW24=3, FT_LR12=4, FT_LR24=5
 * DSP: DSP_XOVER_LR=0, DSP_XOVER_BW=1, DSP_XOVER_BES=2 */
static uint8_t ui_to_dsp_xover_type(uint8_t ui_type) {
  switch (ui_type) {
    case 1: case 2: case 3: return DSP_XOVER_BW;   // BW12, BW18, BW24
    case 4: case 5:         return DSP_XOVER_LR;   // LR12, LR24
    default:                return DSP_XOVER_BW;
  }
}

/* ── Coefficient update: output channel ──────────────────────── */
static void update_output_coeffs(dsp_ch_ctx_t *ctx, peq_list_t *pl,
                                 const dlms_channel_t *ch) {
  const float fs = (float)DLMS_FS;

  if (ch->xover.hp.on) {
    if (ctx->xover_hp_n == 0) {
      for (int i=0; i<DSP_XOVER_MAX_STAGES; i++) dsp_biquad_reset(&ctx->xover_hp[i]);
    }
    ctx->xover_hp_n = dsp_calc_xover(ctx->xover_hp, true,
                                     ui_to_dsp_xover_type(ch->xover.hp.type),
                                     ch->xover.hp.f, ch->xover.hp.slope, fs);
  } else {
    ctx->xover_hp_n = 0;
  }

  if (ch->xover.lp.on) {
    if (ctx->xover_lp_n == 0) {
      for (int i=0; i<DSP_XOVER_MAX_STAGES; i++) dsp_biquad_reset(&ctx->xover_lp[i]);
    }
    ctx->xover_lp_n = dsp_calc_xover(ctx->xover_lp, false,
                                     ui_to_dsp_xover_type(ch->xover.lp.type),
                                     ch->xover.lp.f, ch->xover.lp.slope, fs);
  } else {
    ctx->xover_lp_n = 0;
  }

  build_peq(ctx, pl, ch, DLMS_OUT_BANDS, fs);

  if (ch->gate.on)
    dsp_gate_setup(&ctx->gate, ch->gate.thr, ch->gate.depth, ch->gate.hold,
                   ch->gate.attack, ch->gate.release, fs);

  if (ch->limiter.on)
    dsp_limiter_setup(&ctx->limiter, ch->limiter.thr, ch->limiter.ratio, ch->limiter.knee,
                      ch->limiter.attack, ch->limiter.release, fs);

  dsp_delay_set(&ctx->delay, ch->delay_ms, fs);
}

/* ══════════════════════════════════════════════════════════════
 *  Meter task (Core 1): 32-band Goertzel RTA + WebSocket
 *
 *  Fixes vs. old in-line version:
 *   - ran in the audio task: cJSON + sockets + 4096 cosf() per call
 *   - Hann window recomputed inside the band loop (now a table)
 *   - 128 samples gave ~375 Hz bin width: all bands below ~400 Hz were
 *     meaningless (now 2048 samples)
 *   - Hann coherent gain (0.5) was not compensated: -6 dB error
 * ══════════════════════════════════════════════════════════════ */
static const float k_rta_freq[DLMS_RTA_BANDS] = {
    31.5f, 40,   50,   63,    80,    100,   125,   160,   200,   250,  315,
    400,   500,  630,  800,   1000,  1250,  1600,  2000,  2500,  3150, 4000,
    5000,  6300, 8000, 10000, 12500, 14000, 16000, 18000, 20000, 22000};

static float s_rta_win[RTA_N];
static float s_rta_snap[RTA_N];

#if DSP_SINAD
static float s_trig[RTA_N];  /* cos(2*pi*m/N) */
static float s_sin_x[RTA_N]; /* ADC mentah: DC dibuang, diberi window */

/* DFT satu bin (k) dari s_sin_x -> daya |Y(k)|^2 */
static float sinad_bin_power(int k) {
  const uint32_t MASK = RTA_N - 1;
  float re = 0, im = 0;
  uint32_t m = 0;
  for (int i = 0; i < RTA_N; i++) {
    re += s_sin_x[i] * s_trig[m];
    im += s_sin_x[i] * s_trig[(m + 3u * (RTA_N / 4)) & MASK]; /* sin */
    m = (m + (uint32_t)k) & MASK;
  }
  return re * re + im * im;
}

static void sinad_report(void) {
  static bool init = false;
  if (!init) {
    for (int m = 0; m < RTA_N; m++)
      s_trig[m] = cosf(2.0f * (float)M_PI * (float)m / (float)RTA_N);
    init = true;
  }
  const uint32_t MASK = RTA_N - 1;
  const uint32_t w = s_rta_wr;

  /* Window Blackman-Harris 4-suku (sidelobe sekitar -92 dB), dibangun dari
   * tabel cosinus. DC tidak dikurangi (ADC punya high-pass internal). */
  double ptot = 0, wsum2 = 0;
  for (int i = 0; i < RTA_N; i++) {
    const uint32_t u = (uint32_t)i;
    float wn = 0.35875f - 0.48829f * s_trig[u & MASK] +
               0.14128f * s_trig[(2u * u) & MASK] -
               0.01168f * s_trig[(3u * u) & MASK];
    float y = s_rta_ring[(w + u) & MASK] * wn;
    s_sin_x[i] = y;
    ptot += (double)y * y;
    wsum2 += (double)wn * wn;
  }
  if (ptot < 1e-12) {
    Serial.println("ADC: tidak ada sinyal");
    return;
  }

  /* cari puncak nada: kasar (tiap 2 bin), lalu halus */
  float bestp = 0;
  int bestk = 4;
  for (int k = 4; k <= 640; k += 2) {
    float pw = sinad_bin_power(k);
    if (pw > bestp) {
      bestp = pw;
      bestk = k;
    }
  }
  int kk = bestk;
  for (int k = bestk - 2; k <= bestk + 2; k++) {
    float pw = sinad_bin_power(k);
    if (pw > bestp) {
      bestp = pw;
      kk = k;
    }
  }

  /* energi nada = jumlah bin kk-5..kk+5 (lobus utama Blackman-Harris) */
  double esig = 0;
  for (int k = kk - 5; k <= kk + 5; k++)
    esig += sinad_bin_power(k);
  esig = 2.0 * esig / (double)RTA_N; /* dua sisi spektrum (Parseval) */

  double enoise = ptot - esig;
  if (enoise < 1e-18)
    enoise = 1e-18;
  double sinad = 10.0 * log10(esig / enoise);
  double amp = sqrt(2.0 * esig / wsum2);
  double freq = (double)kk * (double)DLMS_FS / (double)RTA_N;

  Serial.printf("ADC: nada %.0f Hz, level %.1f dBFS, SINAD %.1f dB\n", freq,
                20.0 * log10(amp + 1e-12), sinad);
}
#endif

static void meter_task(void *arg) {
  (void)arg;
  const float fs = (float)DLMS_FS;

  for (int i = 0; i < RTA_N; i++)
    s_rta_win[i] =
        0.5f - 0.5f * cosf(2.0f * (float)M_PI * (float)i / (float)(RTA_N - 1));

  float coeff[DLMS_RTA_BANDS];
  for (int b = 0; b < DLMS_RTA_BANDS; b++)
    coeff[b] = 2.0f * cosf(2.0f * (float)M_PI * k_rta_freq[b] / fs);

  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(METER_PERIOD_MS));

#if DSP_LOAD_LOG
    {
      static int logdiv = 0;
      if (++logdiv >= 20) { /* 20 x 50 ms = 1 detik */
        logdiv = 0;
        uint32_t cnt = s_proc_cnt, sum = s_proc_sum, mx = s_proc_max;
        s_proc_cnt = 0;
        s_proc_sum = 0;
        s_proc_max = 0;
        if (cnt) {
          double dsp_load = ((double)sum / cnt) * 100.0 / (DLMS_BLOCK_FRAMES * 1000000.0 / DLMS_FS);
          Serial.printf(
              "DSP (Core 0): rata-rata %lu us, maks %lu us (Load: %.1f%%)\n",
              (unsigned long)(sum / cnt), (unsigned long)mx, dsp_load);
        }
        
        uint32_t ram_tot = ESP.getHeapSize();
        uint32_t ram_use = ram_tot - ESP.getFreeHeap();
        uint32_t psram_tot = ESP.getPsramSize();
        uint32_t psram_use = psram_tot - ESP.getFreePsram();
        
        Serial.printf("RAM   : %lu / %lu bytes (%.1f%%)\n", (unsigned long)ram_use, (unsigned long)ram_tot, (double)ram_use * 100.0 / ram_tot);
        if (psram_tot > 0) {
          Serial.printf("PSRAM : %lu / %lu bytes (%.1f%%)\n", (unsigned long)psram_use, (unsigned long)psram_tot, (double)psram_use * 100.0 / psram_tot);
        } else {
          Serial.printf("PSRAM : Tidak terdeteksi / Disabled\n");
        }
        Serial.printf("ERR rx=%lu dac1=%lu dac2=%lu mutex=%lu stall=%lu (maks %lu us) glitch=%lu\n",
                      (unsigned long)s_err_rx, (unsigned long)s_err_tx1,
                      (unsigned long)s_err_tx2, (unsigned long)s_mtx_miss,
                      (unsigned long)s_stall_cnt, (unsigned long)s_stall_max_us,
                      (unsigned long)s_glitch_cnt);
        Serial.printf("VU dBFS: IN1=%.0f IN2=%.0f OUT1=%.0f OUT2=%.0f "
                      "OUT3=%.0f OUT4=%.0f\n",
                      (double)s_vu_hold[0], (double)s_vu_hold[1],
                      (double)s_vu_hold[2], (double)s_vu_hold[3],
                      (double)s_vu_hold[4], (double)s_vu_hold[5]);
        for (int i = 0; i < 6; i++)
          s_vu_hold[i] = -90.0f;
#if DSP_SINAD
        sinad_report();
#endif
      }
    }
#endif

    if (dlms_ws_client_count() == 0)
      continue;

    float vu[6];
    for (int i = 0; i < 6; i++)
      vu[i] = s_vu[i];

    /* Oldest sample first, windowed once */
    uint32_t w = s_rta_wr;
    for (int i = 0; i < RTA_N; i++)
      s_rta_snap[i] =
          s_rta_ring[(w + (uint32_t)i) & (RTA_N - 1)] * s_rta_win[i];

    float rta[DLMS_RTA_BANDS];
    for (int b = 0; b < DLMS_RTA_BANDS; b++) {
      if (k_rta_freq[b] >= fs * 0.5f) {
        rta[b] = -90;
        continue;
      }
      float c = coeff[b], q1 = 0, q2 = 0;
      for (int i = 0; i < RTA_N; i++) {
        float q0 = s_rta_snap[i] + c * q1 - q2;
        q2 = q1;
        q1 = q0;
      }
      float power = q1 * q1 + q2 * q2 - c * q1 * q2;
      /* amplitude = 2|X|/N / coherent_gain(0.5) = 4|X|/N */
      float mag = sqrtf(fmaxf(power, 1e-18f)) * (4.0f / (float)RTA_N);
      float db = 20.0f * log10f(fmaxf(mag, 1e-6f));
      rta[b] = clampf(db, -90.0f, 0.0f);
    }

    dlms_ws_broadcast_meter_rta(vu, rta, DLMS_RTA_BANDS);
  }
}

/* ══════════════════════════════════════════════════════════════
 *  Main DSP task
 * ══════════════════════════════════════════════════════════════ */
static void dsp_task(void *arg) {
  (void)arg;

  /* audio_init() DI SINI (task terkunci di core 0): driver I2S memasang
   * interrupt DMA di core tempat i2s_driver_install dipanggil. Kalau dipanggil
   * dari setup() (core 1) ISR audio ikut berebut dengan WiFi/UI. */
  Serial.println("dsp_task: audio_init di core DSP...");
  esp_err_t ae = audio_init();
  if (ae != ESP_OK) {
    Serial.printf("dsp_task: audio_init GAGAL (%d), DSP berhenti\n", (int)ae);
    vTaskDelete(NULL);
    return;
  }

  /* Isi penyangga TX dengan blok diam: arus data DAC selalu punya cadangan
   * beberapa blok, jadi stall singkat tidak langsung jadi klik/underrun. */
  {
    static int32_t zero[DLMS_BLOCK_FRAMES * 2];
    size_t sent = 0;
    for (int i = 0; i < DLMS_TX_PREFILL_BLOCKS; i++) {
      audio_write_dac1(zero, sizeof(zero), &sent, pdMS_TO_TICKS(50));
      audio_write_dac2(zero, sizeof(zero), &sent, pdMS_TO_TICKS(50));
    }
  }

  int32_t rx[DLMS_BLOCK_FRAMES * 2];
  int32_t tx1[DLMS_BLOCK_FRAMES * 2];
  int32_t tx2[DLMS_BLOCK_FRAMES * 2];

  for (int i = 0; i < DLMS_CHANNELS_IN; i++)
    dsp_ch_ctx_init(&s_in_ctx[i]);
  for (int i = 0; i < DLMS_CHANNELS_OUT; i++)
    dsp_ch_ctx_init(&s_out_ctx[i]);

  for (;;) {
    size_t got = 0;
    esp_err_t rErr = audio_read(rx, sizeof(rx), &got, pdMS_TO_TICKS(100));

    // Failsafe: Jika error atau data tidak penuh, tunggu sebentar agar tidak
    // WDT reset
    if (rErr != ESP_OK || got < sizeof(rx)) {
      s_err_rx++;
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }

    size_t frames = got / (sizeof(int32_t) * 2);
    if (frames > DLMS_BLOCK_FRAMES)
      frames = DLMS_BLOCK_FRAMES;

    const uint32_t t0 = micros();
    {
      static uint32_t t_last = 0;
      const uint32_t gap = t0 - t_last;
      /* periode blok 2,67 ms; > 8 ms berarti task tertahan */
      if (t_last != 0 && gap > 8000) {
        s_stall_cnt++;
        if (gap > s_stall_max_us)
          s_stall_max_us = gap;
      }
      t_last = t0;
    }

#if DSP_GLITCH_REPAIR
    /* ── 0. Perbaikan spike 1-sampel dari ADC ─────────────────
     * Sampel tengah yang menyimpang jauh (> TH) dari rata-rata kedua
     * tetangganya, sementara tetangganya sendiri berdekatan, tidak mungkin
     * berasal dari sinyal audio ter-filter (ADC memotong > ~21 kHz): itu bit
     * slip / gangguan clock. Diganti interpolasi. Sinyal ditunda 1 sampel. */
    {
      static int32_t z1[2] = {0, 0}, z2[2] = {0, 0};
      const float S = 1.0f / 2147483648.0f;
      const float TH = 0.75f, NB = 0.25f;
      for (size_t f = 0; f < frames; f++) {
        for (int ch = 0; ch < 2; ch++) {
          const int32_t cur = rx[f * 2 + ch];
          int32_t mid = z1[ch];
          const float a = (float)z2[ch] * S, b = (float)cur * S;
          const float m = (float)mid * S;
          if (fabsf(m - 0.5f * (a + b)) > TH && fabsf(a - b) < NB) {
            mid = (int32_t)(((int64_t)z2[ch] + (int64_t)cur) / 2);
            s_glitch_cnt++;
          }
          rx[f * 2 + ch] = mid;
          z2[ch] = mid;
          z1[ch] = cur;
        }
      }
    }
#endif

    /* ── 1. Input peak metering + RTA feed ───────────────── */
    float peak_l = 0, peak_r = 0;
    uint32_t rw = s_rta_wr;
    for (size_t i = 0; i < frames; i++) {
      float l = (float)rx[i * 2] * (1.0f / 2147483648.0f);
      float r = (float)rx[i * 2 + 1] * (1.0f / 2147483648.0f);
      float vl = fabsf(l), vr = fabsf(r);
      if (vl > peak_l)
        peak_l = vl;
      if (vr > peak_r)
        peak_r = vr;
      s_rta_ring[rw++ & (RTA_N - 1)] = 0.5f * (l + r);
    }
    s_rta_wr = rw;

    float in_db[2] = {lin_to_db(peak_l), lin_to_db(peak_r)};

    /* ── 2. Snapshot parameters (minimise mutex hold) ────── */
    /* Salinan parameter disimpan static: kalau mutex sedang dipegang task
     * web/UI (core 1), JANGAN menunggu -> pakai salinan blok sebelumnya. */
    static dlms_channel_t local_in[DLMS_CHANNELS_IN];
    static dlms_channel_t local_out[DLMS_CHANNELS_OUT];
    static bool have_params = false;

    if (xSemaphoreTake(g_state_mutex,
                       have_params ? 0 : pdMS_TO_TICKS(100)) == pdTRUE) {
      memcpy(local_in, g_dlms.in, sizeof(local_in));
      memcpy(local_out, g_dlms.out, sizeof(local_out));
      xSemaphoreGive(g_state_mutex);
      have_params = true;
    } else {
      s_mtx_miss++;
    }
    /* Peak: float 32-bit sejajar -> penulisan atomik, tanpa mutex */
    g_dlms.in_peak[0] = in_db[0];
    g_dlms.in_peak[1] = in_db[1];

#if DSP_FORCE_FULLRANGE
    for (int o = 0; o < DLMS_CHANNELS_OUT; o++) {
      local_out[o].xover.hp.on = false;
      local_out[o].xover.lp.on = false;
      local_out[o].src_in1 = (o % 2) == 0;
      local_out[o].src_in2 = (o % 2) == 1;
      local_out[o].mute = false;
    }
#endif

    /* ── 3. Update coefficients on parameter change ──────── */
    if (s_first_update || memcmp(local_in, s_prev_in, sizeof(local_in)) != 0) {
      for (int i = 0; i < DLMS_CHANNELS_IN; i++)
        update_input_coeffs(&s_in_ctx[i], &s_in_peq[i], &local_in[i], i);
      memcpy(s_prev_in, local_in, sizeof(s_prev_in));
    }
    if (s_first_update ||
        memcmp(local_out, s_prev_out, sizeof(local_out)) != 0) {
      for (int i = 0; i < DLMS_CHANNELS_OUT; i++)
        update_output_coeffs(&s_out_ctx[i], &s_out_peq[i], &local_out[i]);
      memcpy(s_prev_out, local_out, sizeof(s_prev_out));
      s_first_update = false;
    }

    /* ── 4. Pre-compute linear gains (once per block) ────── */
    float in_gain_lin[DLMS_CHANNELS_IN];
    for (int i = 0; i < DLMS_CHANNELS_IN; i++)
      in_gain_lin[i] = db_to_lin(local_in[i].gain);

    float out_gain_lin[DLMS_CHANNELS_OUT];
    float sg1_lin[DLMS_CHANNELS_OUT], sg2_lin[DLMS_CHANNELS_OUT];
    for (int i = 0; i < DLMS_CHANNELS_OUT; i++) {
      out_gain_lin[i] = db_to_lin(local_out[i].gain);
      sg1_lin[i] = db_to_lin(local_out[i].src_gain1);
      sg2_lin[i] = db_to_lin(local_out[i].src_gain2);
    }

    /* ── 5. Process input channels ───────────────────────── */
    float in_peak_post_lin[DLMS_CHANNELS_IN] = {0};

    for (int ch = 0; ch < DLMS_CHANNELS_IN; ch++) {
      dsp_ch_ctx_t *ctx = &s_in_ctx[ch];
      const dlms_channel_t *p = &local_in[ch];
      const peq_list_t *pl = &s_in_peq[ch];
      const bool gate_on = p->gate.on;
      const bool lim_on = p->limiter.on;
      const bool has_delay = ctx->delay.len != 0;
      const float g =
          p->mute ? 0.0f : in_gain_lin[ch] * (p->invert ? -1.0f : 1.0f);
      float *dst = s_proc_in[ch];
      float peak = 0;
      float min_gr_lin = 1.0f;

      for (size_t f = 0; f < frames; f++) {
        float x = (float)rx[f * 2 + ch] * (1.0f / 2147483648.0f);
        x = dsp_dc_block_tick(&ctx->dc_block, x);
        if (s_subsonic_on[ch]) {
          x = dsp_biquad_tick(&s_subsonic[ch], x);
        }
        x += 1e-15f; // Anti-denormal noise

        for (uint8_t k = 0; k < pl->n; k++)
          x = dsp_biquad_tick(&ctx->peq[pl->idx[k]], x);

        if (has_delay)
          x = dsp_delay_tick(&ctx->delay, x);
        if (gate_on)
          x = dsp_gate_tick(&ctx->gate, x);
        if (lim_on)
          x = dsp_limiter_tick(&ctx->limiter, x);

        x *= g;
        
        if (!isfinite(x)) x = 0;
        
        dst[f] = x;
        float ax = fabsf(x);
        if (ax > peak) peak = ax;
        
        if (lim_on && ctx->limiter.gain_lin < min_gr_lin)
            min_gr_lin = ctx->limiter.gain_lin;
      }
      in_peak_post_lin[ch] = peak;
      
      float gr_db = -lin_to_db(min_gr_lin);
      if (gr_db < 0) gr_db = 0;
      g_dlms.in_gr[ch] = gr_db;
    }
    
    g_dlms.in_peak_post[0] = lin_to_db(in_peak_post_lin[0]);
    g_dlms.in_peak_post[1] = lin_to_db(in_peak_post_lin[1]);

    /* ── 6. Process output channels (per channel, per block) ─ */
    float out_peak_lin[DLMS_CHANNELS_OUT] = {0};
    float out_peak_pre_lin[DLMS_CHANNELS_OUT] = {0};

    for (int o = 0; o < DLMS_CHANNELS_OUT; o++) {
      dsp_ch_ctx_t *ctx = &s_out_ctx[o];
      const dlms_channel_t *p = &local_out[o];
      const peq_list_t *pl = &s_out_peq[o];
      const bool use1 = p->src_in1, use2 = p->src_in2;
      const float g1 = sg1_lin[o], g2 = sg2_lin[o];
      const uint8_t hp_n = ctx->xover_hp_n, lp_n = ctx->xover_lp_n;
      const bool gate_on = p->gate.on;
      const bool lim_on = p->limiter.on;
      const bool has_delay = ctx->delay.len != 0;
      const float gain = out_gain_lin[o];
      const bool mute = p->mute, invert = p->invert;
      float *dst = s_proc_out[o];
      float peak = 0;
      float peak_pre = 0;
      float min_gr_lin = 1.0f;

      for (size_t f = 0; f < frames; f++) {
        float y = 1e-15f; // Anti-denormal noise
        if (use1)
          y += s_proc_in[0][f] * g1;
        if (use2)
          y += s_proc_in[1][f] * g2;
          
        float pre_av = fabsf(y);
        if (pre_av > peak_pre) peak_pre = pre_av;

        if (hp_n > 0)
          y = dsp_biquad_chain_tick(ctx->xover_hp, hp_n, y);
        if (lp_n > 0)
          y = dsp_biquad_chain_tick(ctx->xover_lp, lp_n, y);

        for (uint8_t k = 0; k < pl->n; k++)
          y = dsp_biquad_tick(&ctx->peq[pl->idx[k]], y);

        if (has_delay)
          y = dsp_delay_tick(&ctx->delay, y);
        if (gate_on)
          y = dsp_gate_tick(&ctx->gate, y);
        if (lim_on)
          y = dsp_limiter_tick(&ctx->limiter, y);

        y *= gain;
        if (mute)
          y = 0;
        if (invert)
          y = -y;

        /* NaN/Inf guard: casting NaN to int32 is undefined behaviour */
        if (!isfinite(y))
          y = 0;
        y = clampf(y, -0.999f, 0.999f);

        dst[f] = y;
        float av = fabsf(y);
        if (av > peak)
          peak = av;
          
        if (lim_on && ctx->limiter.gain_lin < min_gr_lin)
            min_gr_lin = ctx->limiter.gain_lin;
      }
      out_peak_lin[o] = peak;
      out_peak_pre_lin[o] = peak_pre;
      
      float gr_db = -lin_to_db(min_gr_lin);
      if (gr_db < 0) gr_db = 0;
      g_dlms.out_gr[o] = gr_db;
    }

    /* Soft-mute: ramp linear 5 ms, lalu tahan sunyi beberapa blok setelah
     * permintaan dilepas supaya data RX yang rusak akibat stall terbuang. */
    static float fade = 1.0f;
    static int hold = 0;
    const bool want_quiet = s_quiet_req > 0;
    if (want_quiet)
      hold = QUIET_HOLD_BLOCKS;
    else if (hold > 0 && fade <= 0.0f)
      hold--;
    const float target = (want_quiet || hold > 0) ? 0.0f : 1.0f;
    const float step = 1.0f / (float)QUIET_RAMP_SAMPLES;

    /* PCM5102 #1 = OUT1 (L) / OUT2 (R); PCM5102 #2 = OUT3 / OUT4 */
    for (size_t f = 0; f < frames; f++) {
      if (fade < target) {
        fade += step;
        if (fade > 1.0f)
          fade = 1.0f;
      } else if (fade > target) {
        fade -= step;
        if (fade < 0.0f)
          fade = 0.0f;
      }
      const float k = fade * 2147483647.0f;
      tx1[f * 2] = (int32_t)(s_proc_out[0][f] * k);
      tx1[f * 2 + 1] = (int32_t)(s_proc_out[1][f] * k);
      tx2[f * 2] = (int32_t)(s_proc_out[2][f] * k);
      tx2[f * 2 + 1] = (int32_t)(s_proc_out[3][f] * k);
    }
    s_quiet_silent = want_quiet && fade <= 0.0f;

    /* One log10 per channel per block instead of one per sample */
    float out_peak[DLMS_CHANNELS_OUT];
    float out_peak_pre[DLMS_CHANNELS_OUT];
    for (int o = 0; o < DLMS_CHANNELS_OUT; o++) {
      out_peak[o] = lin_to_db(out_peak_lin[o]);
      out_peak_pre[o] = lin_to_db(out_peak_pre_lin[o]);
    }

    /* ── 7. Write back output peaks (tanpa mutex) ────────── */
    for (int o = 0; o < DLMS_CHANNELS_OUT; o++) {
      g_dlms.out_peak[o] = out_peak[o];
      g_dlms.out_peak_pre[o] = out_peak_pre[o];
    }

    /* ── 8. LCD + web meter values ───────────────────────── */
    for (int i = 0; i < 2; i++) {

      s_vu[i] = in_db[i];
      if (in_db[i] > s_vu_hold[i])
        s_vu_hold[i] = in_db[i];
    }
    for (int i = 0; i < 4; i++) {

      s_vu[i + 2] = out_peak[i];
      if (out_peak[i] > s_vu_hold[i + 2])
        s_vu_hold[i + 2] = out_peak[i];
    }

    {
      uint32_t d = micros() - t0;
      s_proc_sum += d;
      s_proc_cnt++;
      if (d > s_proc_max)
        s_proc_max = d;
    }

    /* ── 9. Write to DACs ────────────────────────────────── */
    size_t sent = 0;
    const size_t wbytes = frames * 2 * sizeof(int32_t);
    /* Timeout terbatas: kalau DAC2 tidak punya clock, task tidak hang diam-diam */
    if (audio_write_dac1(tx1, wbytes, &sent, pdMS_TO_TICKS(50)) != ESP_OK ||
        sent < wbytes)
      s_err_tx1++;
    if (audio_write_dac2(tx2, wbytes, &sent, pdMS_TO_TICKS(50)) != ESP_OK ||
        sent < wbytes)
      s_err_tx2++;
  }
}

void dsp_quiet_begin(void) {
  __atomic_add_fetch(&s_quiet_req, 1, __ATOMIC_SEQ_CST);
  /* tunggu DSP selesai fade-out (maks 30 ms; kalau DSP tidak jalan, lanjut) */
  for (int i = 0; i < 30 && !s_quiet_silent; i++)
    vTaskDelay(pdMS_TO_TICKS(1));
  /* blok sunyi masih harus melewati antrean DMA TX sebelum flash dikunci */
  vTaskDelay(pdMS_TO_TICKS(TX_LATENCY_MS));
}

void dsp_quiet_end(void) {
  if (__atomic_sub_fetch(&s_quiet_req, 1, __ATOMIC_SEQ_CST) < 0)
    __atomic_store_n(&s_quiet_req, 0, __ATOMIC_SEQ_CST);
}

esp_err_t dsp_start(void) {
  Serial.println("dsp_start: Deleting IDLE0 from WDT...");
  TaskHandle_t idle0 = xTaskGetIdleTaskHandleForCPU(0);
  if (idle0) {
    esp_err_t we = esp_task_wdt_delete(idle0);
    Serial.printf("dsp_start: idle0 wdt delete -> %d\n", (int)we);
  }

  Serial.println("dsp_start: Creating DSP task...");
  if (xTaskCreatePinnedToCore(dsp_task, "dsp", 16384, NULL, DLMS_DSP_PRIORITY,
                              NULL, DLMS_CORE_DSP) != pdPASS) {
    Serial.println("dsp_start: Failed to create dsp_task!");
    return ESP_ERR_NO_MEM;
  }

  Serial.println("dsp_start: Creating METER task...");
  if (xTaskCreatePinnedToCore(meter_task, "meter", 8192, NULL, 3, NULL, DLMS_CORE_UI) !=
      pdPASS) {
    Serial.println("dsp_start: Failed to create meter_task!");
    return ESP_ERR_NO_MEM;
  }

  Serial.println("dsp_start: Done.");
  return ESP_OK;
}