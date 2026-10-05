#pragma once
#include "config.h"
#include "encoder.h"
#include "dlms_state.h"
#include "dlms_ws.h"
#include "app_config.h"
#include "boot_logo.h"

extern void storage_save_audio_state(void);
extern bool storage_save_preset(int slot);
extern bool storage_load_preset(int slot);
extern bool storage_preset_exists(int slot);

static int stepFreq(int f, int steps) {
  int dir = steps > 0 ? 1 : -1;
  for (int n = abs(steps); n > 0; n--) {
    int nf = (int)(f * (dir > 0 ? 1.0293f : 0.9716f) + 0.5f);
    if (nf == f) nf = f + dir;
    f = nf;
    if (f < 20)    { f = 20;    break; }
    if (f > 20000) { f = 20000; break; }
  }
  return f;
}
static float stepQ(float q, int steps) {
  int dir = steps > 0 ? 1 : -1;
  for (int n = abs(steps); n > 0; n--) {
    float st = q < 5.0f ? 0.1f : (q < 10.0f ? 0.5f : 1.0f);
    q += dir * st;
  }
  q = (int)(q * 10.0f + 0.5f) / 10.0f;
  if (q < 0.3f) q = 0.3f;
  if (q > 20.0f) q = 20.0f;
  return q;
}
#define NRATIO 10
#define NCH 4
#define NBAND 9
static const uint8_t PEQ_MAP[NBAND] = {0, 1, 3, 5, 7, 9, 11, 13, 14};
enum : uint8_t { FT_OFF, FT_BW12, FT_BW18, FT_BW24, FT_LR12, FT_LR24, FT_COUNT };
static const char* const FT_NAME[FT_COUNT] = {"OFF", "BW12", "BW18", "BW24", "LR12", "LR24"};
enum : uint8_t { SRC_L, SRC_R, SRC_LR, SRC_COUNT };
static const char* const SRC_NAME[SRC_COUNT] = {"L", "R", "L+R"};
enum : uint8_t { DU_MS, DU_METER, DU_FEET, DU_COUNT };
static const char* const DU_NAME[DU_COUNT] = {"MS", "METER", "FEET"};
static const float RATIOS[10] = {1.0f, 1.5f, 2.0f, 3.0f, 4.0f, 6.0f, 8.0f, 10.0f, 20.0f, 100.0f};

// ---------- format teks ----------
static void fmtFreq(char *b, size_t n, int f, bool withHz) {
  if (f >= 1000) {
    int k10 = (f + 50) / 100;
    if (k10 % 10 == 0) snprintf(b, n, "%dk%s", k10 / 10, withHz ? "Hz" : "");
    else               snprintf(b, n, "%d.%dk%s", k10 / 10, k10 % 10, withHz ? "Hz" : "");
  } else {
    snprintf(b, n, "%d%s", f, withHz ? "Hz" : "");
  }
}
static void fmtDb(char *b, size_t n, float db) {
  if (fabsf(db - roundf(db)) < 0.05f) snprintf(b, n, "%+d", (int)lroundf(db));
  else                                snprintf(b, n, "%+.1f", db);
}
static void fmtDelay(char *b, size_t n, float ms, uint8_t unit) {
  if (unit == DU_METER)      snprintf(b, n, "%.2f m", ms * 0.343f);
  else if (unit == DU_FEET)  snprintf(b, n, "%.2f ft", ms * 1.1253f);
  else                       snprintf(b, n, "%.1f ms", ms);
}
// ---------- matematika respons ----------
static float shapeDb(uint8_t t, float x) {
  switch (t) {
    case FT_BW12: return -10.0f * log10f(1.0f + powf(x, 4.0f));
    case FT_BW18: return -10.0f * log10f(1.0f + powf(x, 6.0f));
    case FT_BW24: return -10.0f * log10f(1.0f + powf(x, 8.0f));
    case FT_LR12: return -20.0f * log10f(1.0f + x * x);
    case FT_LR24: return -20.0f * log10f(1.0f + powf(x, 4.0f));
    default:      return 0.0f;
  }
}
static float xoverDb(const dlms_channel_t &c, float f) {
  float db = 0;
  if (c.xover.hp.type) db += shapeDb(c.xover.hp.type, c.xover.hp.f / f);
  if (c.xover.lp.type) db += shapeDb(c.xover.lp.type, f / c.xover.lp.f);
  return db;
}
static float peqDb(const dlms_band_t &b, float f) {
  if (b.g == 0.0f) return 0.0f;
  const float fs = 48000.0f;
  float A = powf(10.0f, b.g / 40.0f);
  float w0 = 2.0f * 3.14159265f * b.f / fs;
  float alpha = sinf(w0) / (2.0f * b.q);
  float cw = cosf(w0);
  float b0 = 1 + alpha * A, b1 = -2 * cw, b2 = 1 - alpha * A;
  float a0 = 1 + alpha / A, a1 = -2 * cw, a2 = 1 - alpha / A;
  float w = 2.0f * 3.14159265f * f / fs;
  float c1 = cosf(w), s1 = sinf(w), c2 = cosf(2 * w), s2 = sinf(2 * w);
  float nr = b0 + b1 * c1 + b2 * c2, ni = -(b1 * s1 + b2 * s2);
  float dr = a0 + a1 * c1 + a2 * c2, di = -(a1 * s1 + a2 * s2);
  return 10.0f * log10f((nr * nr + ni * ni) / (dr * dr + di * di));
}

#include <TFT_eSPI.h>

// ================= WARNA =================
#define RGB(r, g, b)                                                           \
  ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))
static const uint16_t C_BG = RGB(3, 12, 24);
static const uint16_t C_HDR = RGB(6, 22, 38);
static const uint16_t C_PANEL = RGB(7, 27, 46);
static const uint16_t C_EDIT = RGB(54, 32, 0);
static const uint16_t C_DIM = RGB(16, 52, 74);
static const uint16_t C_LINE = RGB(0, 110, 150);
static const uint16_t C_CYAN = RGB(0, 210, 240);
static const uint16_t C_ORG = RGB(255, 160, 20);
static const uint16_t C_GRN = RGB(40, 230, 110);
static const uint16_t C_RED = RGB(240, 60, 70);
static const uint16_t C_YEL = RGB(250, 220, 40);
static const uint16_t C_MAG = RGB(210, 90, 240);
static const uint16_t C_BLU = RGB(80, 130, 255);
static const uint16_t C_WHT = 0xFFFF;
static const uint16_t C_GRY = RGB(120, 145, 160);
static const uint16_t BAND_COL[] = {
    RGB(240, 60, 70),   RGB(255, 140, 20), RGB(250, 220, 40),
    RGB(60, 230, 90),   RGB(0, 210, 240),  RGB(80, 130, 255),
    RGB(150, 100, 255), RGB(220, 90, 240), RGB(230, 230, 230), RGB(255, 255, 255)};

namespace ui {

static TFT_eSPI tft;
static TFT_eSprite spr(&tft);

enum Scr : uint8_t {
  S_MAIN,
  S_CHAN,
  S_XOVER,
  S_PEQ,
  S_DELAY,
  S_LIM,
  S_GAIN,
  S_SUBSONIC,
  S_GATE,
  S_GLOBAL,
  S_ROUTE,
  S_PRESET,
  S_CONFIRM,
  S_METER,
  S_LCD,
  S_COPY,
  S_COUNT
};

static Scr scr = S_MAIN, meterFrom = S_MAIN;
static int sel[S_COUNT] = {0};
static bool editing = false, dirty = true;
void set_dirty() { dirty = true; }
static uint32_t autoSaveAt = 0;
static bool autoSavePending = false;
void schedule_autosave() {
  autoSavePending = true;
  autoSaveAt = millis() + 2500;
  dirty = true;
}
static int ch = 0, band = 0;
static bool is_input_ch = false;
static bool confSave = false, confYes = true;
static int confSlot = 0;
static const char *toastMsg = nullptr;
static uint32_t toastUntil = 0, lastDraw = 0;
static bool wifiBleEnabled = false;

static uint32_t lastActivityMs = 0;
static bool screenOff = false;

static int copy_src = 2; // Default OUT 1 (0=IN1, 1=IN2, 2=OUT1, 3=OUT2, 4=OUT3, 5=OUT4)
static int copy_dst = 3; // Default OUT 2

// ================= UTIL =================
static inline int wrap(int v, int n) {
  v %= n;
  if (v < 0)
    v += n;
  return v;
}
static inline float clampf(float v, float a, float b) {
  return v < a ? a : (v > b ? b : v);
}
static inline int clampi(int v, int a, int b) {
  return v < a ? a : (v > b ? b : v);
}

static void T(const char *s, int x, int y, uint8_t font, uint16_t col,
              uint8_t datum = TL_DATUM) {
  spr.setTextColor(col);
  spr.setTextDatum(datum);
  spr.drawString(s, x, y, font);
}
static void TB(const char *s, int x, int y, uint8_t font, uint16_t col,
               uint8_t datum = TL_DATUM) {
  T(s, x, y, font, col, datum);
  T(s, x + 1, y, font, col, datum); // fake bold
}

static void box(int x, int y, int w, int h, uint16_t border,
                uint16_t fill = C_PANEL, int r = 4) {
  spr.fillRoundRect(x, y, w, h, r, fill);
  spr.drawRoundRect(x, y, w, h, r, border);
}
static void fieldBox(int x, int y, int w, int h, bool s, bool edit) {
  box(x, y, w, h, s ? C_ORG : C_LINE, edit ? C_EDIT : C_PANEL, 4);
  if (s)
    spr.drawRoundRect(x + 1, y + 1, w - 2, h - 2, 3, C_ORG);
}

static const char *chTag() {
  static char b[8];
  snprintf(b, sizeof b, is_input_ch ? "IN %d" : "OUT%d", ch + 1);
  return b;
}

static void header(const char *tag, const char *title) {
  spr.fillRect(0, 0, SCR_W, 15, C_HDR);
  spr.drawFastHLine(0, 15, SCR_W, C_LINE);
  if (tag) {
    T(tag, 4, 7, 1, C_CYAN, ML_DATUM);
    TB(title, 88, 7, 1, C_WHT, MC_DATUM);
  } else {
    TB(title, 80, 7, 1, C_WHT, MC_DATUM);
  }
}

static void footer(bool backSel, const char *hint = nullptr) {
  spr.drawFastHLine(0, 113, SCR_W, C_DIM);
  T(hint ? hint : (editing ? "TURN:ADJ  CLICK:OK" : "TURN:MOVE CLICK:OK"), 2,
    120, 1, C_ORG, ML_DATUM);
  box(113, 115, 45, 12, backSel ? C_ORG : C_CYAN, backSel ? C_EDIT : C_PANEL,
      3);
  T("< BACK", 135, 121, 1, backSel ? C_ORG : C_WHT, MC_DATUM);
}

static void showToast(const char *m) {
  toastMsg = m;
  toastUntil = millis() + 1200;
  dirty = true;
}

// ================= NAVIGASI =================
static void go(Scr s) {
  if (s == S_METER)
    meterFrom = scr;
  scr = s;
  editing = false;
  dirty = true;
}
static void enter(Scr s) {
  sel[s] = 0;
  go(s);
}

static void goBack() {
  switch (scr) {
  case S_CHAN:
    go(S_MAIN);
    break;
  case S_XOVER:
  case S_PEQ:
  case S_DELAY:
  case S_LIM:
  case S_GAIN:
  case S_SUBSONIC:
  case S_GATE:
    go(S_CHAN);
    break;
  case S_GLOBAL:
    go(S_MAIN);
    break;
  case S_ROUTE:
  case S_PRESET:
  case S_LCD:
  case S_COPY:
    go(S_GLOBAL);
    break;
  case S_CONFIRM:
    go(S_PRESET);
    break;
  case S_METER:
    go(meterFrom);
    break;
  default:
    break;
  }
}

enum NavRes { N_NONE, N_ADJ, N_ACT };

// Navigasi layar dengan field: putar = pindah, klik = edit, klik lagi = OK.
// Item ke-nItems adalah tombol BACK. instantMask: item yang langsung aksi saat
// diklik.
static NavRes nav(Scr id, int nItems, const Input &in,
                  uint32_t instantMask = 0) {
  int &s = sel[id];
  if (!editing) {
    if (in.raw) {
      s = wrap(s + (in.raw > 0 ? 1 : -1), nItems + 1);
      dirty = true;
    }
    if (in.click) {
      if (s == nItems)
        goBack();
      else if (instantMask & (1u << s)) {
        dirty = true;
        return N_ACT;
      } else {
        editing = true;
        dirty = true;
      }
    }
    return N_NONE;
  }
  if (in.click) {
    editing = false;
    dirty = true;
    return N_NONE;
  }
  return in.raw ? N_ADJ : N_NONE;
}

// Navigasi menu list: return index item yang diklik, -1 kalau tidak ada
static int menuNav(Scr id, int n, const Input &in) {
  int &s = sel[id];
  if (in.raw) {
    s = wrap(s + (in.raw > 0 ? 1 : -1), n + 1);
    dirty = true;
  }
  if (in.click) {
    if (s == n) {
      goBack();
      return -1;
    }
    return s;
  }
  return -1;
}

// ================= LAYAR: MAIN =================
static void gauge(int cx, int cy, int r, float db, uint16_t col,
                  const char *lab, bool selected) {
  if (selected) {
    spr.fillCircle(cx, cy, r + 2, C_EDIT);
    spr.drawCircle(cx, cy, r + 2, C_ORG);
  }
  db = clampf(db, -60.0f, 0.0f);
  float f = (db + 60.0f) / 60.0f;
  spr.drawSmoothArc(cx, cy, r, r - 4, 30, 330, C_DIM, selected ? C_EDIT : C_BG, false);
  
  uint16_t arcCol = C_GRN;
  if (db >= 0.0f) arcCol = C_RED;
  else if (db >= -15.0f) arcCol = C_YEL;

  if (f > 0.03f)
    spr.drawSmoothArc(cx, cy, r, r - 4, 30, 30 + (uint32_t)(f * 300.0f), arcCol,
                      selected ? C_EDIT : C_BG, false);
  T(lab, cx, cy - 5, 1, col, MC_DATUM);
  char b[8];
  snprintf(b, sizeof b, "%d", (int)lroundf(db));
  T(b, cx, cy + 5, 1, C_WHT, MC_DATUM);
}

static uint16_t srcColor(uint8_t s) {
  return s == SRC_L ? C_CYAN : (s == SRC_R ? C_ORG : C_MAG);
}

static void drawMain() {
  spr.fillSprite(C_BG);
  spr.fillRect(0, 0, SCR_W, 15, C_HDR);
  spr.drawFastHLine(0, 15, SCR_W, C_LINE);
  TB(FW_NAME, 4, 7, 1, C_WHT, ML_DATUM);
  bool ss = sel[S_MAIN] == 6;
  box(112, 1, 46, 13, ss ? C_ORG : C_LINE, ss ? C_EDIT : C_PANEL, 3);
  T("SETUP", 135, 7, 1, ss ? C_ORG : C_CYAN, MC_DATUM);

  gauge(17, 32, 15, g_dlms.in_peak_post[0], C_CYAN, "L", sel[S_MAIN] == 0);
  if (g_dlms.in_gr[0] >= 0.5f) spr.fillCircle(7, 19, 3, C_RED);
  gauge(51, 32, 15, g_dlms.in_peak_post[1], C_ORG, "R", sel[S_MAIN] == 1);
  if (g_dlms.in_gr[1] >= 0.5f) spr.fillCircle(41, 19, 3, C_RED);

  box(70, 18, 88, 29, C_LINE, C_PANEL, 4);
  T("48.0kHz 32bit", 76, 23, 1, C_WHT, ML_DATUM);
  T("2 IN > 4 OUT", 76, 32, 1, C_CYAN, ML_DATUM);
  if (wifiBleEnabled) {
    T("WIFI ON", 76, 41, 1, C_GRN, ML_DATUM);
  } else {
    T("WIFI OFF", 76, 41, 1, C_GRY, ML_DATUM);
  }

  for (int i = 0; i < NCH; i++) {
    const dlms_channel_t &c = g_dlms.out[i];
    int x = 2 + i * 39, y = 50, w = 37, h = 62;
    bool s = sel[S_MAIN] == (i + 2);
    box(x, y, w, h, s ? C_CYAN : C_LINE, C_PANEL, 4);
    char nm[8];
    snprintf(nm, sizeof nm, "OUT%d", i + 1);
    if (s) {
      spr.fillRoundRect(x + 1, y + 1, w - 2, 13, 3, C_CYAN);
      T(nm, x + w / 2, y + 8, 1, C_BG, MC_DATUM);
    } else
      T(nm, x + w / 2, y + 8, 1, C_CYAN, MC_DATUM);

    if (g_dlms.out_gr[i] >= 0.5f) {
      spr.fillCircle(x + w - 6, y + 7, 2, C_RED);
    }

    float lv = c.mute ? -90.0f : g_dlms.out_peak[i];
    int lit = 0;
    if (lv >= 0.0f) lit = 7;
    else if (lv >= -3.0f) lit = 6;
    else if (lv >= -6.0f) lit = 5;
    else if (lv >= -10.0f) lit = 4;
    else if (lv >= -15.0f) lit = 3;
    else if (lv >= -25.0f) lit = 2;
    else if (lv >= -40.0f) lit = 1;

    for (int k = 0; k < 7; k++) {
      int sy = y + 17 + (6 - k) * 4;
      uint16_t col = C_GRN;
      if (k >= 6) col = C_RED;
      else if (k >= 4) col = C_YEL;
      spr.fillRect(x + 13, sy, 11, 3, k < lit ? col : C_DIM);
    }
    char g[12];
    if (c.mute)
      T("MUTE", x + w / 2, y + 48, 1, C_RED, MC_DATUM);
    else {
      fmtDb(g, sizeof g, c.gain);
      strcat(g, "dB");
      T(g, x + w / 2, y + 48, 1, C_WHT, MC_DATUM);
    }
    spr.fillRoundRect(x + 7, y + 52, 23, 8, 2, srcColor((c.src_in1 && c.src_in2 ? 2 : (c.src_in2 ? 1 : 0))));
    T(SRC_NAME[(c.src_in1 && c.src_in2 ? 2 : (c.src_in2 ? 1 : 0))], x + w / 2, y + 56, 1, C_BG, MC_DATUM);
  }
  spr.drawFastHLine(0, 113, SCR_W, C_DIM);
  T("TURN:SELECT  CLICK:OPEN", 2, 121, 1, C_ORG, ML_DATUM);
}

// ================= LAYAR: CHANNEL MENU =================
static void drawChan() {
  const dlms_channel_t &c = is_input_ch ? g_dlms.in[ch] : g_dlms.out[ch];
  spr.fillSprite(C_BG);
  char t[16];
  snprintf(t, sizeof t, is_input_ch ? "INPUT %d" : "OUT %d", ch + 1);
  header(nullptr, t);

  // Output: X-OVER, PARA EQ, DELAY, LIMITER, GAIN/MUTE (5 items)
  static const char *names_out[5] = {"X-OVER", "PARA EQ", "DELAY", "LIMITER", "GAIN/MUTE"};
  static const uint16_t icol_out[5] = {C_GRN, C_MAG, C_CYAN, C_ORG, C_YEL};
  // Input: GAIN/MUTE, SUBSONIC, GATE, PEQ, DELAY, LIMITER (6 items)
  static const char *names_in[6] = {"GAIN/MUTE", "SUBSONIC", "GATE", "PARA EQ", "DELAY", "LIMITER"};
  static const uint16_t icol_in[6] = {C_YEL, C_BLU, C_RED, C_MAG, C_CYAN, C_ORG};
  
  const char **names = is_input_ch ? names_in : names_out;
  const uint16_t *icol = is_input_ch ? icol_in : icol_out;
  int num_items = is_input_ch ? 6 : 5;

  char v[6][20], a[8], b[8], g[8];
  int v_idx = 0;
  
  if (is_input_ch) {
    // GAIN/MUTE
    fmtDb(g, sizeof g, c.gain);
    snprintf(v[v_idx++], 20, "%sdB", g);
    // SUBSONIC
    if (c.subsonic_freq > 0) {
      char sf[8]; fmtFreq(sf, sizeof sf, c.subsonic_freq, true);
      snprintf(v[v_idx++], 20, "%s", sf);
    } else {
      snprintf(v[v_idx++], 20, "OFF");
    }
    // GATE
    snprintf(v[v_idx++], 20, c.gate.on ? "ON" : "OFF");
    // PEQ
    int act = 0;
    for (int i = 0; i < DLMS_IN_BANDS; i++)
      if (c.peq[i].g != 0.0f) act++;
    snprintf(v[v_idx++], 20, "%d/%d", act, DLMS_IN_BANDS);
    // DELAY
    snprintf(v[v_idx++], 20, "%.1fms", c.delay_ms);
    // LIMITER
    snprintf(v[v_idx++], 20, "%.1fdB", c.limiter.thr);
  } else {
    // X-OVER
    fmtFreq(a, sizeof a, c.xover.hp.f, false);
    fmtFreq(b, sizeof b, c.xover.lp.f, false);
    snprintf(v[v_idx++], 20, "%s-%s", a, b);
    // PEQ
    int act = 0;
    for (int i = 0; i < DLMS_OUT_BANDS; i++)
      if (c.peq[i].g != 0.0f) act++;
    snprintf(v[v_idx++], 20, "%d/15", act);
    // DELAY
    snprintf(v[v_idx++], 20, "%.1fms", c.delay_ms);
    // LIMITER
    snprintf(v[v_idx++], 20, "%.1fdB", c.limiter.thr);
    // GAIN/MUTE
    fmtDb(g, sizeof g, c.gain);
    snprintf(v[v_idx++], 20, "%sdB", g);
  }

  int row_h = is_input_ch ? 16 : 19;
  for (int i = 0; i < num_items; i++) {
    int y = 18 + i * row_h;
    bool s = sel[S_CHAN] == i;
    box(3, y, 154, row_h - 2, s ? C_ORG : C_LINE, s ? C_EDIT : C_PANEL, 4);
    spr.fillRoundRect(8, y + (row_h-2)/2 - 3, 7, 7, 2, icol[i]);
    T(names[i], 20, y + (row_h-2)/2, 2, s ? C_ORG : C_WHT, ML_DATUM);
    T(v[i], 152, y + (row_h-2)/2, 1, s ? C_ORG : C_CYAN, MR_DATUM);
  }
  footer(sel[S_CHAN] == num_items);
}

// ================= LAYAR: X-OVER =================
static void drawXover() {
  const dlms_channel_t &c = g_dlms.out[ch];
  int s = sel[S_XOVER];
  spr.fillSprite(C_BG);
  header(chTag(), "X-OVER");

  const int gx = 20, gy = 17, gw = 136, gh = 47;
  box(gx - 1, gy - 1, gw + 2, gh + 2, C_LINE, C_BG, 2);
  auto fx = [&](float f) { return gx + (int)(gw * log10f(f / 20.0f) / 3.0f); };
  auto dy = [&](float db) {
    if (db < -60)
      db = -60;
    return gy + 2 + (int)(-db / 60.0f * (gh - 5));
  };

  static const int gf[3] = {100, 1000, 10000};
  static const char *gl[3] = {"100", "1k", "10k"};
  for (int i = 0; i < 3; i++) {
    int x = fx((float)gf[i]);
    spr.drawFastVLine(x, gy, gh, C_DIM);
    T(gl[i], x - 1, gy + gh - 5, 1, C_GRY, MR_DATUM);
  }
  static const int dl[4] = {0, -20, -40, -60};
  for (int i = 0; i < 4; i++) {
    int y = dy((float)dl[i]);
    spr.drawFastHLine(gx, y, gw, C_DIM);
    char b[6];
    snprintf(b, sizeof b, "%d", dl[i]);
    T(b, 17, y, 1, C_GRY, MR_DATUM);
  }

  float split = sqrtf((float)c.xover.hp.f * (float)c.xover.lp.f);
  int py = 0;
  for (int px = 0; px < gw; px++) {
    float f = 20.0f * powf(10.0f, 3.0f * px / (gw - 1));
    int y = dy(xoverDb(c, f));
    uint16_t col = f < split ? C_GRN : C_ORG;
    if (px > 0)
      spr.drawLine(gx + px - 1, py, gx + px, y, col);
    py = y;
  }
  int xh = clampi(fx((float)c.xover.hp.f), gx + 3, gx + gw - 4);
  int xl = clampi(fx((float)c.xover.lp.f), gx + 3, gx + gw - 4);
  for (int yy = gy; yy < gy + gh; yy += 3) {
    spr.drawPixel(xh, yy, C_GRN);
    spr.drawPixel(xl, yy, C_ORG);
  }
  spr.fillTriangle(xh - 3, gy + gh - 1, xh + 3, gy + gh - 1, xh, gy + gh - 6,
                   C_GRN);
  spr.fillTriangle(xl - 3, gy + gh - 1, xl + 3, gy + gh - 1, xl, gy + gh - 6,
                   C_ORG);

  for (int r = 0; r < 2; r++) {
    int y = 68 + r * 21;
    bool isH = (r == 0);
    uint16_t rc = isH ? C_GRN : C_ORG;
    int fv = isH ? c.xover.hp.f : c.xover.lp.f;
    uint8_t ty = isH ? c.xover.hp.type : c.xover.lp.type;
    int iv = r * 2, it = r * 2 + 1;
    spr.fillRoundRect(2, y, 31, 20, 4, rc);
    TB(isH ? "HPF" : "LPF", 17, y + 10, 1, C_BG, MC_DATUM);
    fieldBox(36, y, 74, 20, s == iv, editing && s == iv);
    char b[12], t2[16];
    fmtFreq(b, sizeof b, fv, false);
    snprintf(t2, sizeof t2, "%s Hz", b);
    T(t2, 73, y + 10, 2, ty == FT_OFF ? C_GRY : (s == iv ? C_ORG : C_WHT),
      MC_DATUM);
    fieldBox(113, y, 45, 20, s == it, editing && s == it);
    T(FT_NAME[ty], 135, y + 10, 2, s == it ? C_ORG : rc, MC_DATUM);
  }
  footer(s == 4);
}

// ================= LAYAR: PARA EQ =================
static void drawPeq() {
  const dlms_channel_t &c = is_input_ch ? g_dlms.in[ch] : g_dlms.out[ch];
  int s = sel[S_PEQ];
  spr.fillSprite(C_BG);
  header(chTag(), "PARA EQ");

  const int nb = is_input_ch ? DLMS_IN_BANDS : NBAND;
  // Input: direct map (0..4), Output: PEQ_MAP (9 of 15)
  static const uint8_t IN_MAP[5] = {0, 1, 2, 3, 4};

  int btn_w = is_input_ch ? 30 : 15;
  int btn_gap = is_input_ch ? 1 : 2;
  int btn_start = is_input_ch ? 3 : 4;

  for (int i = 0; i < nb; i++) {
    int mapped_idx = is_input_ch ? IN_MAP[i] : PEQ_MAP[i];
    int x = btn_start + i * (btn_w + btn_gap), y = 16, h = 15;
    bool act = (i == band), on = c.peq[mapped_idx].g != 0.0f;
    uint16_t bc = BAND_COL[i];
    if (act)
      spr.fillRoundRect(x, y, btn_w, h, 4, bc);
    else {
      spr.fillRoundRect(x, y, btn_w, h, 4, C_PANEL);
      spr.drawRoundRect(x, y, btn_w, h, 4, on ? bc : C_LINE);
    }
    char n[3];
    snprintf(n, sizeof n, "%d", i + 1);
    T(n, x + btn_w / 2, y + 7, 1, act ? C_BG : (on ? bc : C_GRY), MC_DATUM);
  }
  if (s == 0) {
    int x = btn_start + band * (btn_w + btn_gap);
    spr.drawRoundRect(x - 2, 14, btn_w + 4, 19, 5, C_ORG);
    if (editing)
      spr.drawRoundRect(x - 3, 13, btn_w + 6, 21, 6, C_ORG);
  }

  const int nbands_total = is_input_ch ? DLMS_IN_BANDS : DLMS_OUT_BANDS;
  const int gx = 2, gy = 34, gw = 156, gh = 51, cy = gy + gh / 2;
  box(gx, gy, gw, gh, C_LINE, C_BG, 2);
  auto fx = [&](float f) {
    return gx + 1 + (int)((gw - 3) * log10f(f / 20.0f) / 3.0f);
  };
  spr.drawFastHLine(gx + 1, cy, gw - 2, C_DIM);
  static const int gf[3] = {100, 1000, 10000};
  static const char *gl[3] = {"100", "1k", "10k"};
  for (int i = 0; i < 3; i++) {
    int x = fx((float)gf[i]);
    spr.drawFastVLine(x, gy + 1, gh - 2, C_DIM);
    T(gl[i], x - 1, gy + gh - 6, 1, C_GRY, MR_DATUM);
  }
  int py = cy;
  for (int px = 0; px < gw - 3; px++) {
    float f = 20.0f * powf(10.0f, 3.0f * px / (gw - 4));
    float db = 0;
    for (int b = 0; b < nbands_total; b++)
      db += peqDb(c.peq[b], f);
    int y = clampi(cy - (int)lroundf(db * 2.0f), gy + 2, gy + gh - 3);
    if (px > 0) {
      spr.drawLine(gx + 1 + px - 1, py, gx + 1 + px, y, C_CYAN);
      spr.drawLine(gx + 1 + px - 1, py + 1, gx + 1 + px, y + 1, C_CYAN);
    }
    py = y;
  }
  for (int b = 0; b < nb; b++) {
    int mapped_idx = is_input_ch ? IN_MAP[b] : PEQ_MAP[b];
    int x = fx((float)c.peq[mapped_idx].f);
    int y =
        clampi(cy - (int)lroundf(c.peq[mapped_idx].g * 2.0f), gy + 3, gy + gh - 4);
    if (b == band) {
      spr.fillCircle(x, y, 3, BAND_COL[b]);
      spr.drawCircle(x, y, 5, C_WHT);
    } else
      spr.fillCircle(x, y, 2, BAND_COL[b]);
  }

  int cur_mapped = is_input_ch ? IN_MAP[band] : PEQ_MAP[band];
  const dlms_band_t &pb = c.peq[cur_mapped];
  char v[3][14], g[8];
  fmtFreq(v[0], 14, pb.f, true);
  fmtDb(g, sizeof g, pb.g);
  snprintf(v[1], 14, "%sdB", g);
  snprintf(v[2], 14, "%.1f", pb.q);
  static const char *lab[3] = {"FREQ", "GAIN", "Q"};
  static const int bx[3] = {2, 57, 112}, bw[3] = {52, 52, 46};
  for (int i = 0; i < 3; i++) {
    bool ss = (s == i + 1);
    fieldBox(bx[i], 87, bw[i], 24, ss, editing && ss);
    T(lab[i], bx[i] + 5, 92, 1, C_GRY, ML_DATUM);
    T(v[i], bx[i] + bw[i] / 2, 102, 2, ss ? C_ORG : C_WHT, MC_DATUM);
  }
  footer(s == 4);
}

// ================= LAYAR: DELAY =================
static void drawDelay() {
  const dlms_channel_t &c = is_input_ch ? g_dlms.in[ch] : g_dlms.out[ch];
  int s = sel[S_DELAY];
  spr.fillSprite(C_BG);
  header(chTag(), "DELAY");
  for (int i = 0; i < DU_COUNT; i++) {
    int x = 2 + i * 53, y = 18, w = 50, h = 16;
    bool on = (0 == i);
    if (on) {
      spr.fillRoundRect(x, y, w, h, 4, C_CYAN);
      T(DU_NAME[i], x + w / 2, y + 8, 1, C_BG, MC_DATUM);
    } else {
      box(x, y, w, h, C_LINE, C_PANEL, 4);
      T(DU_NAME[i], x + w / 2, y + 8, 1, C_CYAN, MC_DATUM);
    }
    if (on && s == 0) {
      spr.drawRoundRect(x - 1, y - 1, w + 2, h + 2, 5, C_ORG);
      if (editing)
        spr.drawRoundRect(x - 2, y - 2, w + 4, h + 4, 6, C_ORG);
    }
  }
  fieldBox(2, 38, 156, 68, s == 1, editing && s == 1);
  T("TIME", 10, 46, 1, C_GRY, ML_DATUM);
  char b[20];
  fmtDelay(b, sizeof b, c.delay_ms, 0);
  T(b, 10, 64, 4, s == 1 ? C_ORG : C_WHT, ML_DATUM);
  spr.drawFastHLine(10, 90, 140, C_LINE);
  spr.fillRect(9, 84, 3, 13, C_CYAN);
  int mx = 10 + (int)(140.0f * c.delay_ms / 300.0f);
  spr.fillRect(mx - 2, 84, 5, 13, C_ORG);
  T("IN", 10, 100, 1, C_CYAN, ML_DATUM);
  T("300ms", 150, 100, 1, C_GRY, MR_DATUM);
  footer(s == 2);
}

// ================= LAYAR: LIMITER =================
static void drawLimiter() {
  const dlms_channel_t &c = is_input_ch ? g_dlms.in[ch] : g_dlms.out[ch];
  int s = sel[S_LIM];
  spr.fillSprite(C_BG);
  header(chTag(), "LIMITER");
  static const char *lab[4] = {"THRESH dB", "RATIO", "ATTACK ms", "RELEASE ms"};
  char v[4][12];
  snprintf(v[0], 12, "%.1f", c.limiter.thr);
  if (c.limiter.ratio >= 100.0f)
    snprintf(v[1], 12, "INF:1");
  else
    snprintf(v[1], 12, "%.1f:1", c.limiter.ratio);
  snprintf(v[2], 12, "%d", (int)c.limiter.attack);
  snprintf(v[3], 12, "%d", (int)c.limiter.release);
  for (int i = 0; i < 4; i++) {
    int x = (i & 1) ? 82 : 2, y = (i >> 1) ? 53 : 17;
    bool ss = (s == i);
    fieldBox(x, y, 76, 35, ss, editing && ss);
    T(lab[i], x + 6, y + 6, 1, C_CYAN, ML_DATUM);
    T(v[i], x + 6, y + 22, 4, ss ? C_ORG : C_WHT, ML_DATUM);
  }
  T("GAIN REDUCTION", 4, 93, 1, C_CYAN, ML_DATUM);
  float gr = is_input_ch ? g_dlms.in_gr[ch] : g_dlms.out_gr[ch];
  int lit = clampi((int)(gr + 0.5f), 0, 18);
  for (int k = 0; k < 18; k++) {
    uint16_t col = k < 8 ? C_GRN : (k < 14 ? C_YEL : C_RED);
    spr.fillRect(4 + k * 6, 99, 5, 10, k < lit ? col : C_DIM);
  }
  char g[10];
  snprintf(g, sizeof g, "-%.1f", gr);
  T(g, 156, 104, 2, C_WHT, MR_DATUM);
  footer(s == 4);
}

// ================= LAYAR: GAIN / MUTE =================
static void pill(int x, int y, int w, int h, bool on, uint16_t onCol,
                 const char *txt, bool degree = false) {
  uint16_t col = on ? onCol : C_GRY;
  spr.fillRoundRect(x, y, w, h, h / 2, C_BG);
  spr.drawRoundRect(x, y, w, h, h / 2, col);
  int cx = x + w / 2, cy = y + h / 2;
  if (degree) {
    int tw = spr.textWidth(txt, 1);
    T(txt, cx - 2, cy, 1, col, MC_DATUM);
    spr.drawCircle(cx - 2 + tw / 2 + 3, cy - 3, 1, col);
  } else
    T(txt, cx, cy, 1, col, MC_DATUM);
}

static void drawGain() {
  const dlms_channel_t &c = is_input_ch ? g_dlms.in[ch] : g_dlms.out[ch];
  int s = sel[S_GAIN];
  spr.fillSprite(C_BG);
  header(chTag(), "GAIN / MUTE");
  fieldBox(2, 18, 156, 46, s == 0, editing && s == 0);
  T("GAIN", 10, 25, 1, C_CYAN, ML_DATUM);
  char b[16];
  fmtDb(b, sizeof b, c.gain);
  strcat(b, " dB");
  T(b, 10, 40, 4, s == 0 ? C_ORG : C_WHT, ML_DATUM);
  float f = clampf((c.gain + 60.0f) / 72.0f, 0, 1);
  int fw = (int)(126 * f);
  spr.fillRoundRect(10, 53, 126, 5, 2, C_DIM);
  spr.fillRoundRect(10, 53, fw < 3 ? 3 : fw, 5, 2, C_CYAN);
  spr.fillRect(10 + fw - 1, 50, 2, 11, C_WHT);
  float lv = -90.0f;
  if (!c.mute) {
    if (is_input_ch) {
      // Pre-IN: mentah dari ADC ditambah pergerakan slider Gain
      lv = g_dlms.in_peak[ch] + c.gain;
    } else {
      // Pre-OUT: campuran input mentah ditambah pergerakan slider Gain
      lv = g_dlms.out_peak_pre[ch] + c.gain;
    }
  }

  // Tampilkan 9 kotak VU meter.
  // Kisaran: -40dB sampai 0dB
  int lit = 0;
  if (lv >= 0.0f) lit = 9;       // Clip
  else if (lv >= -3.0f) lit = 8;
  else if (lv >= -6.0f) lit = 7;
  else if (lv >= -10.0f) lit = 6;
  else if (lv >= -15.0f) lit = 5;
  else if (lv >= -20.0f) lit = 4;
  else if (lv >= -25.0f) lit = 3;
  else if (lv >= -30.0f) lit = 2;
  else if (lv >= -40.0f) lit = 1;

  for (int k = 0; k < 9; k++) {
    uint16_t col = C_GRN;
    if (k >= 8) col = C_RED;       // Kotak paling atas (Clip) merah
    else if (k >= 5) col = C_YEL;  // Kotak ke-6, 7, 8 kuning (-15dB ke atas)
    
    spr.fillRect(143, 22 + (8 - k) * 4, 8, 3, k < lit ? col : C_DIM);
  }
  fieldBox(2, 67, 156, 20, s == 1, false);
  T("MUTE", 10, 77, 2, C_CYAN, ML_DATUM);
  pill(108, 70, 46, 14, c.mute, C_RED, c.mute ? "ON" : "OFF");
  fieldBox(2, 89, 156, 20, s == 2, false);
  T("PHASE", 10, 99, 2, C_CYAN, ML_DATUM);
  pill(108, 92, 46, 14, c.invert, C_ORG, c.invert ? "180" : "0", true);
  footer(s == 3);
}

// ================= LAYAR: SUBSONIC (HPF Low-Cut) =================
static void drawSubsonic() {
  const dlms_channel_t &c = is_input_ch ? g_dlms.in[ch] : g_dlms.out[ch];
  int s = sel[S_SUBSONIC];
  spr.fillSprite(C_BG);
  header(chTag(), "SUBSONIC HPF");

  fieldBox(2, 20, 156, 40, s == 0, editing && s == 0);
  T("FREQUENCY", 10, 28, 1, C_CYAN, ML_DATUM);
  char b[16];
  if (c.subsonic_freq > 0) {
    fmtFreq(b, sizeof b, c.subsonic_freq, true);
  } else {
    snprintf(b, sizeof b, "OFF");
  }
  T(b, 10, 45, 4, s == 0 ? C_ORG : C_WHT, ML_DATUM);

  // Draw simple HPF response curve
  const int gx = 2, gy = 65, gw = 156, gh = 35;
  box(gx, gy, gw, gh, C_LINE, C_BG, 2);
  spr.drawFastHLine(gx + 1, gy + gh / 2, gw - 2, C_DIM);
  if (c.subsonic_freq > 0) {
    for (int px = 1; px < gw - 1; px++) {
      float f = 10.0f * powf(2000.0f / 10.0f, (float)px / (gw - 2));
      float ratio = f / (float)c.subsonic_freq;
      float db = -10.0f * log10f(1.0f + 1.0f / (ratio * ratio * ratio * ratio));
      int py = gy + gh / 2 - (int)(db * (gh - 4) / 48.0f);
      py = clampi(py, gy + 1, gy + gh - 2);
      spr.drawPixel(gx + px, py, C_BLU);
    }
  }
  T("10", gx + 2, gy + gh + 2, 1, C_GRY, ML_DATUM);
  T("1k", gx + gw / 2, gy + gh + 2, 1, C_GRY, MC_DATUM);
  footer(s == 1);
}

// ================= LAYAR: NOISE GATE =================
static void drawGate() {
  const dlms_channel_t &c = is_input_ch ? g_dlms.in[ch] : g_dlms.out[ch];
  int s = sel[S_GATE];
  spr.fillSprite(C_BG);
  header(chTag(), "NOISE GATE");

  // ON/OFF
  fieldBox(2, 17, 76, 30, s == 0, false);
  T("GATE", 10, 24, 1, C_CYAN, ML_DATUM);
  pill(12, 32, 56, 12, c.gate.on, C_GRN, c.gate.on ? "ON" : "OFF");

  // Threshold
  fieldBox(82, 17, 76, 30, s == 1, editing && s == 1);
  T("THRESH dB", 90, 24, 1, C_CYAN, ML_DATUM);
  char v[12];
  snprintf(v, 12, "%.0f", c.gate.thr);
  T(v, 90, 36, 2, s == 1 ? C_ORG : C_WHT, ML_DATUM);

  // Depth
  fieldBox(2, 50, 76, 30, s == 2, editing && s == 2);
  T("DEPTH dB", 10, 57, 1, C_CYAN, ML_DATUM);
  snprintf(v, 12, "%.0f", c.gate.depth);
  T(v, 10, 69, 2, s == 2 ? C_ORG : C_WHT, ML_DATUM);

  // Attack
  fieldBox(82, 50, 76, 30, s == 3, editing && s == 3);
  T("ATTACK ms", 90, 57, 1, C_CYAN, ML_DATUM);
  snprintf(v, 12, "%.0f", c.gate.attack);
  T(v, 90, 69, 2, s == 3 ? C_ORG : C_WHT, ML_DATUM);

  // Hold
  fieldBox(2, 83, 76, 22, s == 4, editing && s == 4);
  T("HOLD ms", 10, 88, 1, C_CYAN, ML_DATUM);
  snprintf(v, 12, "%.0f", c.gate.hold);
  T(v, 10, 98, 1, s == 4 ? C_ORG : C_WHT, ML_DATUM);

  // Release
  fieldBox(82, 83, 76, 22, s == 5, editing && s == 5);
  T("RELEASE ms", 90, 88, 1, C_CYAN, ML_DATUM);
  snprintf(v, 12, "%.0f", c.gate.release);
  T(v, 90, 98, 1, s == 5 ? C_ORG : C_WHT, ML_DATUM);

  footer(s == 6);
}

// ================= LAYAR: GLOBAL SETTING =================
static void drawGlobal() {
  spr.fillSprite(C_BG);
  header(nullptr, "GLOBAL SETTING");
  static const char *t1[6] = {"MATRIX ROUTING", "PRESET SETUP", "LEVEL METER", "COPY CHANNEL", "WIFI", "LCD SETTING"};
  static const char *t2[6] = {"IN > OUT MAP", "LOAD / SAVE", "IN + OUT VU", "COPY PARAMS", "", "BRIGHTNESS & TIMER"};
  static const uint16_t ic[6] = {C_CYAN, C_GRN, C_ORG, C_YEL, C_BLU, C_MAG};

  int s_idx = sel[S_GLOBAL];
  int scroll = s_idx > 2 ? (s_idx >= 5 ? 3 : s_idx - 2) : 0;
  for (int i = 0; i < 3; i++) {
    int idx = i + scroll;
    int y = 18 + i * 31;
    bool s = sel[S_GLOBAL] == idx;
    box(3, y, 154, 28, s ? C_GRN : C_LINE, s ? RGB(0, 50, 34) : C_PANEL, 5);
    if (idx == 0) {
      for (int a = 0; a < 2; a++)
        for (int b = 0; b < 3; b++)
          spr.fillRect(10 + b * 6, y + 9 + a * 6, 3, 3, ic[idx]);
    } else if (idx == 1) {
      spr.drawRoundRect(10, y + 9, 18, 11, 2, ic[idx]);
      spr.drawFastHLine(10, y + 11, 8, ic[idx]);
    } else if (idx == 2) {
      spr.fillRect(11, y + 15, 3, 6, C_GRN);
      spr.fillRect(16, y + 10, 3, 11, C_YEL);
      spr.fillRect(21, y + 7, 3, 14, C_RED);
    } else if (idx == 3) { // COPY CHANNEL
      spr.drawRect(11, y + 11, 10, 10, ic[idx]);
      spr.fillRect(15, y + 7, 10, 10, C_PANEL);
      spr.drawRect(15, y + 7, 10, 10, ic[idx]);
    } else if (idx == 4) { // WIFI
      spr.fillCircle(19, y + 16, 2, ic[idx]);
      spr.drawSmoothArc(19, y + 16, 6, 5, 315, 45, ic[idx], C_BG, false);
      spr.drawSmoothArc(19, y + 16, 10, 9, 315, 45, ic[idx], C_BG, false);
    } else if (idx == 5) { // LCD SETTING
      spr.fillCircle(19, y + 14, 4, ic[idx]);
      spr.drawFastVLine(19, y + 7, 2, ic[idx]);
      spr.drawFastVLine(19, y + 19, 2, ic[idx]);
      spr.drawFastHLine(12, y + 14, 2, ic[idx]);
      spr.drawFastHLine(24, y + 14, 2, ic[idx]);
    }
    T(t1[idx], 34, y + 9, 2, s ? C_GRN : C_WHT, ML_DATUM);
    if (idx == 4 && g_dlms.wifi_on) {
      T(g_dlms.wifi_mode == RACK_LCD_WIFI_AP ? g_dlms.ap_ip : g_dlms.ip, 34, y + 22, 1, C_CYAN, ML_DATUM);
    } else {
      T(t2[idx], 34, y + 22, 1, C_GRY, ML_DATUM);
    }
    if (idx == 4) {
      T(wifiBleEnabled ? "ON" : "OFF", 146, y + 15, 2,
        wifiBleEnabled ? C_GRN : C_RED, MR_DATUM);
    }
  }
  footer(sel[S_GLOBAL] == 6);
}

// ================= LAYAR: MATRIX ROUTE =================
static void drawRoute() {
  int s = sel[S_ROUTE];
  spr.fillSprite(C_BG);
  header(nullptr, "MATRIX ROUTE");
  T("IN", 16, 22, 1, C_CYAN, MC_DATUM);
  const int ly = 48, ry = 90;
  spr.drawCircle(16, ly, 9, C_CYAN);
  T("L", 16, ly, 2, C_CYAN, MC_DATUM);
  spr.drawCircle(16, ry, 9, C_ORG);
  T("R", 16, ry, 2, C_ORG, MC_DATUM);
  for (int i = 0; i < NCH; i++) {
    int y = 18 + i * 23, yo = y + 10;
    uint8_t src = (g_dlms.out[i].src_in1 && g_dlms.out[i].src_in2 ? 2 : (g_dlms.out[i].src_in2 ? 1 : 0));
    if (src == SRC_L || src == SRC_LR) {
      int xm = 34 + i * 4;
      spr.drawFastHLine(25, ly, xm - 25, C_CYAN);
      spr.drawFastVLine(xm, min(ly, yo), abs(yo - ly) + 1, C_CYAN);
      spr.drawFastHLine(xm, yo, 82 - xm, C_CYAN);
    }
    if (src == SRC_R || src == SRC_LR) {
      int xm = 36 + i * 4;
      spr.drawFastHLine(25, ry, xm - 25, C_ORG);
      spr.drawFastVLine(xm, min(ry, yo), abs(yo - ry) + 1, C_ORG);
      spr.drawFastHLine(xm, yo, 82 - xm, C_ORG);
    }
    bool ss = (s == i);
    fieldBox(82, y, 76, 20, ss, editing && ss);
    char nm[8];
    snprintf(nm, sizeof nm, "OUT%d", i + 1);
    T(nm, 89, yo, 1, ss ? C_ORG : C_WHT, ML_DATUM);
    spr.fillRoundRect(124, y + 4, 28, 12, 3, srcColor(src));
    T(SRC_NAME[src], 138, yo, 1, C_BG, MC_DATUM);
  }
  footer(s == 4);
}

// ================= LAYAR: LEVEL METER =================
static void drawMeter() {
  spr.fillSprite(C_BG);
  header(nullptr, "LEVEL METER");
  const int bot = 101;
  static const int lbl[5] = {0, -12, -24, -36, -48};
  for (int i = 0; i < 5; i++) {
    int y = bot - (int)((lbl[i] + 48) / 48.0f * 80.0f);
    char b[6];
    snprintf(b, sizeof b, "%d", lbl[i]);
    spr.drawFastHLine(24, y, 134, C_DIM);
    T(b, 1, y, 1, C_GRY, ML_DATUM);
  }
  float lv[6] = {g_dlms.in_peak_post[0], g_dlms.in_peak_post[1],
                 g_dlms.out_peak[0], g_dlms.out_peak[1], g_dlms.out_peak[2], g_dlms.out_peak[3]};
  const uint16_t base[6] = {C_CYAN, C_ORG, C_GRN, C_GRN, C_GRN, C_GRN};
  const char *nm[6] = {"L", "R", "01", "02", "03", "04"};
  for (int i = 0; i < 6; i++) {
    int x = 26 + i * 22;
    for (int k = 0; k < 20; k++) {
      float thr = -48.0f + (k + 1) * 2.4f;
      uint16_t col = C_GRN;
      if (thr >= -0.1f) col = C_RED;
      else if (thr >= -15.0f) col = C_YEL;
      spr.fillRect(x, bot - (k + 1) * 4 + 1, 18, 3, lv[i] >= thr ? col : C_DIM);
    }
    // peak hold
    float pk = (i < 2) ? g_dlms.in_peak_post[i] : g_dlms.out_peak[i - 2];
    if (pk > -48.0f)
      spr.fillRect(x, bot - (int)((pk + 48.0f) / 48.0f * 80.0f), 18, 2, C_WHT);
    T(nm[i], x + 9, 107, 1, base[i], MC_DATUM);
    
    // Limiter active indicator
    float gr = (i < 2) ? g_dlms.in_gr[i] : g_dlms.out_gr[i - 2];
    if (gr >= 0.5f) {
      spr.fillRoundRect(x + 2, 13, 14, 5, 2, C_RED);
    }
  }
  spr.drawFastVLine(68, 20, 84, C_LINE);
  footer(true, "CLICK:EXIT");
}

// ================= LAYAR: PRESET =================
static bool confReset = false;
static void drawPreset() {
  spr.fillSprite(C_BG);
  header(nullptr, "PRESET SETUP");
  int s = sel[S_PRESET];
  // 5 preset rows (LOAD + SAVE per row) = items 0..9
  // item 10 = RESET, item 11 = BACK (footer)
  for (int p = 0; p < 5; p++) {
    int y = 17 + p * 16;
    int il = p * 2, is_ = p * 2 + 1;
    // Load button
    bool sl = (s == il);
    box(3, y, 75, 14, sl ? C_GRN : C_LINE, sl ? RGB(0,50,34) : C_PANEL, 3);
    spr.fillTriangle(8, y+3, 16, y+3, 12, y+10, C_GRN);
    char lb[12]; snprintf(lb, sizeof lb, "LOAD %d", p+1);
    T(lb, 20, y+7, 1, sl ? C_GRN : C_WHT, ML_DATUM);
    // Save button
    bool ss = (s == is_);
    box(82, y, 75, 14, ss ? C_BLU : C_LINE, ss ? RGB(0,30,60) : C_PANEL, 3);
    spr.fillTriangle(87, y+10, 95, y+10, 91, y+3, C_BLU);
    char sb[12]; snprintf(sb, sizeof sb, "SAVE %d", p+1);
    T(sb, 99, y+7, 1, ss ? C_BLU : C_WHT, ML_DATUM);
  }
  // RESET button
  int ry = 17 + 5 * 16;
  bool sr = (s == 10);
  box(3, ry, 154, 14, sr ? C_RED : C_LINE, sr ? RGB(60,10,10) : C_PANEL, 3);
  T("RESET ALL TO FLAT", 80, ry+7, 1, sr ? C_RED : C_WHT, MC_DATUM);
  footer(s == 11);
}

static void drawConfirm() {
  spr.fillSprite(C_BG);
  header(nullptr, "PRESET SETUP");
  box(3, 19, 154, 106, C_LINE, C_BG, 6);
  T(confSave ? "SAVE THIS PRESET?" : "LOAD THIS PRESET?", 80, 36, 2, C_YEL,
    MC_DATUM);
  char b[12];
  snprintf(b, sizeof b, "SLOT %d", confSlot + 1);
  T(b, 80, 54, 1, C_CYAN, MC_DATUM);
  spr.fillRoundRect(14, 68, 56, 38, 8, confYes ? C_GRN : C_PANEL);
  spr.drawRoundRect(14, 68, 56, 38, 8, C_GRN);
  TB("YES", 42, 87, 2, confYes ? C_BG : C_GRN, MC_DATUM);
  spr.fillRoundRect(90, 68, 56, 38, 8, !confYes ? C_RED : C_PANEL);
  spr.drawRoundRect(90, 68, 56, 38, 8, C_RED);
  TB("NO", 118, 87, 2, !confYes ? C_BG : C_RED, MC_DATUM);
}

// ================= LAYAR: LCD SETTING =================
static void drawLcd() {
  int s = sel[S_LCD];
  spr.fillSprite(C_BG);
  header(nullptr, "LCD SETTING");
  
  fieldBox(2, 20, 156, 30, s == 0, editing && s == 0);
  T("BRIGHTNESS", 10, 27, 1, C_CYAN, ML_DATUM);
  char v[12];
  snprintf(v, sizeof(v), "%d", g_dlms.brightness);
  T(v, 146, 31, 2, s == 0 ? C_ORG : C_WHT, MR_DATUM);
  
  fieldBox(2, 55, 156, 30, s == 1, editing && s == 1);
  T("AUTO-OFF", 10, 62, 1, C_CYAN, ML_DATUM);
  if (g_dlms.lcd_timeout_min == 0) {
    snprintf(v, sizeof(v), "NEVER");
  } else {
    snprintf(v, sizeof(v), "%d MIN", g_dlms.lcd_timeout_min);
  }
  T(v, 146, 66, 2, s == 1 ? C_ORG : C_WHT, MR_DATUM);

  footer(s == 2);
}

// ================= LAYAR: COPY CHANNEL =================
static void drawCopy() {
  int s = sel[S_COPY];
  spr.fillSprite(C_BG);
  header(nullptr, "COPY CHANNEL");

  fieldBox(2, 20, 156, 26, s == 0, editing && s == 0);
  T("SOURCE", 10, 26, 1, C_CYAN, ML_DATUM);
  static const char *src_names[] = {"IN 1", "IN 2", "OUT 1", "OUT 2", "OUT 3", "OUT 4"};
  T(src_names[copy_src], 146, 26, 2, s == 0 ? C_ORG : C_WHT, MR_DATUM);

  fieldBox(2, 51, 156, 26, s == 1, editing && s == 1);
  T("TARGET", 10, 57, 1, C_CYAN, ML_DATUM);
  const char *dst_name = "";
  if (copy_src < 2) {
    static const char *in_dst_names[] = {"IN 1", "IN 2", "ALL IN"};
    dst_name = in_dst_names[copy_dst];
  } else {
    static const char *out_dst_names[] = {"OUT 1", "OUT 2", "OUT 3", "OUT 4", "ALL OUT"};
    dst_name = out_dst_names[copy_dst];
  }
  T(dst_name, 146, 57, 2, s == 1 ? C_ORG : C_WHT, MR_DATUM);

  bool s2 = (s == 2);
  box(3, 82, 154, 24, s2 ? C_RED : C_LINE, s2 ? RGB(60,10,10) : C_PANEL, 3);
  T("CLICK TO COPY", 80, 94, 1, s2 ? C_RED : C_WHT, MC_DATUM);

  footer(s == 3);
}

// ================= INPUT =================
static void handle(const Input &in) {
  if (!in.any())
    return;

  lastActivityMs = millis();
  if (screenOff) {
    // Just wake up, consume this input
    return;
  }

  if (in.longPress) {
    if (scr == S_MAIN)
      go(S_METER); // tahan di main = level meter
    else if (editing) {
      editing = false;
      dirty = true;
      autoSavePending = true;
      autoSaveAt = millis() + 3000;
    } else
      goBack();
    return;
  }
  dirty = true;
  switch (scr) {
  case S_MAIN: {
    int &s = sel[S_MAIN];
    if (in.raw)
      s = wrap(s + (in.raw > 0 ? 1 : -1), 7);
    if (in.click) {
      if (s < 2) {
        ch = s;
        is_input_ch = true;
        enter(S_CHAN);
      } else if (s < 6) {
        ch = s - 2;
        is_input_ch = false;
        enter(S_CHAN);
      } else
        enter(S_GLOBAL);
    }
  } break;

  case S_CHAN: {
    int num_items = is_input_ch ? 6 : 5;
    int r = menuNav(S_CHAN, num_items, in);
    if (r >= 0) {
      if (is_input_ch) {
        // Input: {GAIN/MUTE, SUBSONIC, GATE, PEQ, DELAY, LIMITER}
        static const Scr in_map[6] = {S_GAIN, S_SUBSONIC, S_GATE, S_PEQ, S_DELAY, S_LIM};
        enter(in_map[r]);
      } else {
        // Output: {X-OVER, PARA EQ, DELAY, LIMITER, GAIN/MUTE}
        enter((Scr)(S_XOVER + r));
      }
    }
  } break;

  case S_XOVER: {
    dlms_channel_t &c = g_dlms.out[ch];
    if (nav(S_XOVER, 4, in) == N_ADJ) {
      int s = sel[S_XOVER], d = in.raw > 0 ? 1 : -1;
      if (s == 0)
        c.xover.hp.f = stepFreq(c.xover.hp.f, in.adj(6));
      else if (s == 1) {
        c.xover.hp.type = wrap(c.xover.hp.type + d, FT_COUNT);
        c.xover.hp.on = (c.xover.hp.type != FT_OFF);
        // Map UI type to DSP slope
        switch (c.xover.hp.type) {
          case FT_BW12: c.xover.hp.slope = 12; break;
          case FT_BW18: c.xover.hp.slope = 18; break;
          case FT_BW24: c.xover.hp.slope = 24; break;
          case FT_LR12: c.xover.hp.slope = 12; break;
          case FT_LR24: c.xover.hp.slope = 24; break;
          default:      c.xover.hp.slope = 0;  break;
        }
      }
      else if (s == 2)
        c.xover.lp.f = stepFreq(c.xover.lp.f, in.adj(6));
      else if (s == 3) {
        c.xover.lp.type = wrap(c.xover.lp.type + d, FT_COUNT);
        c.xover.lp.on = (c.xover.lp.type != FT_OFF);
        switch (c.xover.lp.type) {
          case FT_BW12: c.xover.lp.slope = 12; break;
          case FT_BW18: c.xover.lp.slope = 18; break;
          case FT_BW24: c.xover.lp.slope = 24; break;
          case FT_LR12: c.xover.lp.slope = 12; break;
          case FT_LR24: c.xover.lp.slope = 24; break;
          default:      c.xover.lp.slope = 0;  break;
        }
      }
    }
  } break;

  case S_PEQ: {
    dlms_channel_t &c = is_input_ch ? g_dlms.in[ch] : g_dlms.out[ch];
    const int nb = is_input_ch ? DLMS_IN_BANDS : NBAND;
    static const uint8_t IN_MAP[5] = {0, 1, 2, 3, 4};
    if (nav(S_PEQ, 4, in) == N_ADJ) {
      int s = sel[S_PEQ];
      int mi = is_input_ch ? IN_MAP[band] : PEQ_MAP[band];
      dlms_band_t &pb = c.peq[mi];
      if (s == 0)
        band = wrap(band + (in.raw > 0 ? 1 : -1), nb);
      else if (s == 1)
        pb.f = stepFreq(pb.f, in.adj(6));
      else if (s == 2)
        pb.g = clampf(pb.g + 0.5f * in.adj(4), -12.0f, 12.0f);
      else if (s == 3)
        pb.q = stepQ(pb.q, in.adj(4));
    }
  } break;

  case S_DELAY: {
    dlms_channel_t &c = is_input_ch ? g_dlms.in[ch] : g_dlms.out[ch];
    if (nav(S_DELAY, 2, in) == N_ADJ) {
      int s = sel[S_DELAY];
      if (s == 0) {
        // disabled delayUnit
      } else
        c.delay_ms =
            clampf(roundf((c.delay_ms + 0.1f * in.adj(20)) * 10.0f) / 10.0f,
                   0.0f, 300.0f);
    }
  } break;

  case S_LIM: {
    dlms_channel_t &c = is_input_ch ? g_dlms.in[ch] : g_dlms.out[ch];
    if (nav(S_LIM, 4, in) == N_ADJ) {
      int s = sel[S_LIM];
      if (s == 0)
        c.limiter.thr = clampf(c.limiter.thr + 0.5f * in.adj(6), -40.0f, 0.0f);
      else if (s == 1) {
        // Step through ratio values
        int ri = 0;
        for (int i = 0; i < NRATIO; i++) {
          if (fabsf(c.limiter.ratio - RATIOS[i]) < 0.01f) { ri = i; break; }
        }
        ri = clampi(ri + (in.raw > 0 ? 1 : -1), 0, NRATIO - 1);
        c.limiter.ratio = RATIOS[ri];
      } else if (s == 2)
        c.limiter.attack = clampi((int)c.limiter.attack + in.adj(5), 1, 200);
      else if (s == 3)
        c.limiter.release = clampi((int)c.limiter.release + 10 * in.adj(5), 10, 2000);
    }
  } break;

  case S_GAIN: {
    dlms_channel_t &c = is_input_ch ? g_dlms.in[ch] : g_dlms.out[ch];
    NavRes r = nav(S_GAIN, 3, in, (1u << 1) | (1u << 2));
    if (r == N_ADJ)
      c.gain = clampf(c.gain + 0.5f * in.adj(6), -60.0f, 12.0f);
    else if (r == N_ACT) {
      if (sel[S_GAIN] == 1)
        c.mute = !c.mute;
      else
        c.invert = !c.invert;
    }
  } break;

  case S_SUBSONIC: {
    dlms_channel_t &c = is_input_ch ? g_dlms.in[ch] : g_dlms.out[ch];
    if (nav(S_SUBSONIC, 1, in) == N_ADJ) {
      if (sel[S_SUBSONIC] == 0) {
        // Step through: 0(OFF), 20, 25, 30, 40, 50, 60, 80, 100, 120, 150, 200
        static const int sub_freqs[] = {0, 20, 25, 30, 40, 50, 60, 80, 100, 120, 150, 200};
        int idx = 0;
        for (int i = 0; i < 12; i++) {
          if (c.subsonic_freq == sub_freqs[i]) { idx = i; break; }
        }
        idx = clampi(idx + (in.raw > 0 ? 1 : -1), 0, 11);
        c.subsonic_freq = sub_freqs[idx];
      }
    }
  } break;

  case S_GATE: {
    dlms_channel_t &c = is_input_ch ? g_dlms.in[ch] : g_dlms.out[ch];
    if (nav(S_GATE, 6, in, (1u << 0)) == N_ADJ) {
      int s = sel[S_GATE];
      if (s == 1)
        c.gate.thr = clampf(c.gate.thr + 1.0f * in.adj(4), -80.0f, 0.0f);
      else if (s == 2)
        c.gate.depth = clampf(c.gate.depth + 1.0f * in.adj(4), -80.0f, 0.0f);
      else if (s == 3)
        c.gate.attack = clampf(c.gate.attack + 1.0f * in.adj(4), 0.1f, 100.0f);
      else if (s == 4)
        c.gate.hold = clampf(c.gate.hold + 5.0f * in.adj(4), 0.0f, 2000.0f);
      else if (s == 5)
        c.gate.release = clampf(c.gate.release + 10.0f * in.adj(4), 10.0f, 5000.0f);
    } else if (scr == S_GATE && sel[S_GATE] == 0 && in.click) {
      c.gate.on = !c.gate.on;
      dirty = true;
    }
  } break;

  case S_GLOBAL: {
    int r = menuNav(S_GLOBAL, 6, in);
    if (r == 0)
      enter(S_ROUTE);
    else if (r == 1)
      enter(S_PRESET);
    else if (r == 2)
      go(S_METER);
    else if (r == 3)
      enter(S_COPY);
    else if (r == 4) {
      wifiBleEnabled = !wifiBleEnabled;
      dirty = true;
    } else if (r == 5) {
      enter(S_LCD);
    }
  } break;

  case S_ROUTE: {
    if (nav(S_ROUTE, 4, in) == N_ADJ) {
      int ri = sel[S_ROUTE];
      // Hitung src saat ini: 0=L, 1=R, 2=L+R
      uint8_t cur = (g_dlms.out[ri].src_in1 && g_dlms.out[ri].src_in2) ? 2
                  : (g_dlms.out[ri].src_in2 ? 1 : 0);
      cur = wrap(cur + (in.raw > 0 ? 1 : -1), SRC_COUNT);
      g_dlms.out[ri].src_in1 = (cur == SRC_L || cur == SRC_LR);
      g_dlms.out[ri].src_in2 = (cur == SRC_R || cur == SRC_LR);
      autoSavePending = true;
      autoSaveAt = millis() + 3000;
    }
  } break;

  case S_PRESET: {
    // 0..9 = LOAD/SAVE for 5 slots, 10 = RESET, 11 = BACK (footer via nav)
    if (nav(S_PRESET, 11, in) == N_ADJ) {
      // no adj fields, but we use nav for scroll + click
    }
    // nav handles BACK at item 11. We handle click on 0..10:
    if (in.click && sel[S_PRESET] < 11) {
      int r = sel[S_PRESET];
      if (r < 10) {
        confSlot = r / 2;
        confSave = (r % 2 == 1);
        confReset = false;
        confYes = true;
        go(S_CONFIRM);
      } else if (r == 10) {
        confReset = true;
        confYes = true;
        go(S_CONFIRM);
      }
    }
  } break;

  case S_CONFIRM: {
    if (in.raw)
      confYes = !confYes;
    if (in.click) {
      if (confYes) {
        if (confReset) {
          // Reset all channels to flat
          xSemaphoreTake(g_state_mutex, portMAX_DELAY);
          dlms_state_init();
          xSemaphoreGive(g_state_mutex);
          showToast("RESET OK");
          dlms_ws_broadcast_state();
        } else {
          bool ok = false;
          if (confSave) {
            ok = storage_save_preset(confSlot);
          } else {
            ok = storage_load_preset(confSlot);
          }
          showToast(confSave ? (ok ? "SAVED" : "SAVE FAIL")
                             : (ok ? "LOADED" : "SLOT EMPTY"));
          if (ok) dlms_ws_broadcast_state();
        }
      }
      goBack();
    }
  } break;

  case S_METER:
    if (in.click)
      goBack();
    break;

  case S_LCD: {
    if (nav(S_LCD, 2, in) == N_ADJ) {
      int s = sel[S_LCD];
      if (s == 0) {
        g_dlms.brightness = clampi(g_dlms.brightness + in.adj(5), 10, 255);
        analogWrite(PIN_TFT_BL, g_dlms.brightness);
        autoSavePending = true;
        autoSaveAt = millis() + 3000;
      } else if (s == 1) {
        g_dlms.lcd_timeout_min = clampi(g_dlms.lcd_timeout_min + in.adj(1), 0, 30);
        autoSavePending = true;
        autoSaveAt = millis() + 3000;
      }
    }
  } break;

  case S_COPY: {
    if (nav(S_COPY, 3, in, (1u<<2)) == N_ADJ) {
      int s = sel[S_COPY];
      if (s == 0) {
        int old_src = copy_src;
        copy_src = wrap(copy_src + (in.raw > 0 ? 1 : -1), 6);
        if ((old_src < 2 && copy_src >= 2) || (old_src >= 2 && copy_src < 2)) {
          copy_dst = 0; 
        }
      } else if (s == 1) {
        int max_dst = (copy_src < 2) ? 3 : 5;
        copy_dst = wrap(copy_dst + (in.raw > 0 ? 1 : -1), max_dst);
      }
    } else if (scr == S_COPY && sel[S_COPY] == 2 && in.click) {
      xSemaphoreTake(g_state_mutex, portMAX_DELAY);
      if (copy_src < 2) {
        if (copy_dst == 2) { // ALL IN
          for (int i = 0; i < 2; i++) {
            if (i != copy_src) {
              char tmp[20]; strcpy(tmp, g_dlms.in[i].name);
              g_dlms.in[i] = g_dlms.in[copy_src];
              strcpy(g_dlms.in[i].name, tmp);
            }
          }
        } else {
          char tmp[20]; strcpy(tmp, g_dlms.in[copy_dst].name);
          g_dlms.in[copy_dst] = g_dlms.in[copy_src];
          strcpy(g_dlms.in[copy_dst].name, tmp);
        }
      } else {
        if (copy_dst == 4) { // ALL OUT
          for (int i = 0; i < 4; i++) {
            if (i != (copy_src - 2)) {
              char tmp[20]; strcpy(tmp, g_dlms.out[i].name);
              g_dlms.out[i] = g_dlms.out[copy_src - 2];
              strcpy(g_dlms.out[i].name, tmp);
            }
          }
        } else {
          char tmp[20]; strcpy(tmp, g_dlms.out[copy_dst].name);
          g_dlms.out[copy_dst] = g_dlms.out[copy_src - 2];
          strcpy(g_dlms.out[copy_dst].name, tmp);
        }
      }
      xSemaphoreGive(g_state_mutex);
      showToast("COPIED!");
      autoSavePending = true;
      autoSaveAt = millis() + 3000;
      dlms_ws_broadcast_state();
    }
  } break;

  default:
    break;
  }

  // Live Sync LCD -> Web: broadcast kapanpun nilai diedit via encoder atau aksi klik
  if ((editing && in.accel != 0) || (in.click && (scr == S_GAIN || scr == S_GATE || scr == S_XOVER || scr == S_SUBSONIC || scr == S_LIM || scr == S_ROUTE || scr == S_LCD))) {
    dlms_ws_broadcast_state();
    schedule_autosave();
  }
}

// ================= RENDER =================
static void render() {
  uint32_t now = millis();
  
  if (g_dlms.lcd_timeout_min > 0) {
    if (!screenOff && (now - lastActivityMs >= g_dlms.lcd_timeout_min * 60000)) {
      screenOff = true;
      analogWrite(PIN_TFT_BL, 0);
    } else if (screenOff && (now - lastActivityMs < g_dlms.lcd_timeout_min * 60000)) {
      screenOff = false;
      analogWrite(PIN_TFT_BL, g_dlms.brightness);
    }
  } else if (screenOff) {
    screenOff = false;
    analogWrite(PIN_TFT_BL, g_dlms.brightness);
  }

  if (toastMsg && now >= toastUntil) {
    toastMsg = nullptr;
    dirty = true;
  }
  bool anim =
      (scr == S_MAIN || scr == S_GAIN || scr == S_LIM || scr == S_METER);
  if (!dirty && !(anim && now - lastDraw >= 50))
    return;

  switch (scr) {
  case S_MAIN:
    drawMain();
    break;
  case S_CHAN:
    drawChan();
    break;
  case S_XOVER:
    drawXover();
    break;
  case S_PEQ:
    drawPeq();
    break;
  case S_DELAY:
    drawDelay();
    break;
  case S_LIM:
    drawLimiter();
    break;
  case S_GAIN:
    drawGain();
    break;
  case S_SUBSONIC:
    drawSubsonic();
    break;
  case S_GATE:
    drawGate();
    break;
  case S_GLOBAL:
    drawGlobal();
    break;
  case S_ROUTE:
    drawRoute();
    break;
  case S_PRESET:
    drawPreset();
    break;
  case S_CONFIRM:
    drawConfirm();
    break;
  case S_METER:
    drawMeter();
    break;
  case S_LCD:
    drawLcd();
    break;
  case S_COPY:
    drawCopy();
    break;
  default:
    break;
  }
  if (toastMsg) {
    box(30, 48, 100, 32, C_GRN, C_BG, 6);
    TB(toastMsg, 80, 64, 2, C_GRN, MC_DATUM);
  }
  
  if (autoSavePending && now >= autoSaveAt) {
    autoSavePending = false;
    storage_save_audio_state();
  }
  
  spr.pushSprite(0, 0);
  dirty = false;
  lastDraw = now;
}

static void begin() {
  // Backlight sudah dimatikan di setup() paling awal untuk cegah flash putih.
  // Jangan nyalakan dulu — biarkan TFT init dalam keadaan gelap.
  tft.init();
  tft.setRotation(LCD_ROTATION);
  tft.fillScreen(TFT_BLACK);

  #ifdef BOOT_LOGO_W
  tft.setSwapBytes(true);
  tft.pushImage((SCR_W - BOOT_LOGO_W) / 2, (SCR_H - BOOT_LOGO_H) / 2, BOOT_LOGO_W, BOOT_LOGO_H, boot_logo);
  // Logo sudah siap di layar — baru nyalakan backlight
  analogWrite(PIN_TFT_BL, g_dlms.brightness);
  delay(3000);
  tft.fillScreen(TFT_BLACK);
  #else
  analogWrite(PIN_TFT_BL, g_dlms.brightness);
  #endif

  spr.setColorDepth(16);
  if (!spr.createSprite(SCR_W, SCR_H))
    Serial.println("ERR: sprite gagal dibuat (RAM)");
}

} // namespace ui
