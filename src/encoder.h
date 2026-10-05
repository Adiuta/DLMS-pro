#pragma once
#include <Arduino.h>
#include "driver/gpio.h"
#include "config.h"

// Event input untuk UI
struct Input {
  int  raw = 0;          // jumlah detent mentah (+ = searah jarum jam)
  int  accel = 1;        // faktor percepatan (1..20), makin cepat diputar makin besar
  bool click = false;    // klik pendek
  bool longPress = false;// tahan >= LONG_PRESS_MS

  bool any() const { return raw || click || longPress; }
  // Untuk mengubah nilai: raw * percepatan (dibatasi maxMul)
  int adj(int maxMul) const {
    int m = accel < maxMul ? accel : maxMul;
    return raw * m;
  }
};

// ---- Tabel state quadrature (full-step, anti-bounce) ----
#define R_START     0x0
#define R_CW_FINAL  0x1
#define R_CW_BEGIN  0x2
#define R_CW_NEXT   0x3
#define R_CCW_BEGIN 0x4
#define R_CCW_FINAL 0x5
#define R_CCW_NEXT  0x6
#define DIR_CW      0x10
#define DIR_CCW     0x20

namespace enc_detail {
static const uint8_t ttable[7][4] = {
  {R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},
  {R_CW_NEXT,  R_START,     R_CW_FINAL,  R_START | DIR_CW},
  {R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START},
  {R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},
  {R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START},
  {R_CCW_NEXT, R_CCW_FINAL, R_START,     R_START | DIR_CCW},
  {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},
};
static volatile uint8_t  state = R_START;
static volatile int32_t  steps = 0;
static volatile uint32_t lastUs = 0;
static volatile uint32_t intervalUs = 1000000;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR isr() {
  uint8_t pins = (gpio_get_level((gpio_num_t)ENC_B_PIN) << 1) | gpio_get_level((gpio_num_t)ENC_A_PIN);
  state = ttable[state & 0x0f][pins];
  uint8_t dir = state & 0x30;
  if (dir) {
    uint32_t now = micros();
    portENTER_CRITICAL_ISR(&mux);
    steps += (dir == DIR_CW) ? 1 : -1;
    intervalUs = now - lastUs;
    lastUs = now;
    portEXIT_CRITICAL_ISR(&mux);
  }
}
}  // namespace enc_detail

class Encoder {
 public:
  void begin() {
    pinMode(ENC_A_PIN, INPUT_PULLUP);
    pinMode(ENC_B_PIN, INPUT_PULLUP);
    pinMode(ENC_SW_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), enc_detail::isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), enc_detail::isr, CHANGE);
  }

  Input poll() {
    Input in;

    int32_t n; uint32_t iv;
    portENTER_CRITICAL(&enc_detail::mux);
    n = enc_detail::steps; enc_detail::steps = 0;
    iv = enc_detail::intervalUs;
    portEXIT_CRITICAL(&enc_detail::mux);

    if (n) {
#if ENC_REVERSE
      n = -n;
#endif
      in.raw = n;
      uint32_t ms = iv / 1000;
      in.accel = ms < 15 ? 20 : ms < 30 ? 8 : ms < 60 ? 3 : 1;
    }

    // ---- tombol: debounce + klik / tahan ----
    uint32_t now = millis();
    bool raw = digitalRead(ENC_SW_PIN) == LOW;
    if (raw != lastRaw_) { lastRaw_ = raw; lastChange_ = now; }
    if (now - lastChange_ > 25 && raw != stable_) {
      stable_ = raw;
      if (stable_) { pressAt_ = now; longFired_ = false; }
      else if (!longFired_) in.click = true;
    }
    if (stable_ && !longFired_ && now - pressAt_ > LONG_PRESS_MS) {
      longFired_ = true;
      in.longPress = true;
    }
    return in;
  }

 private:
  bool lastRaw_ = false, stable_ = false, longFired_ = false;
  uint32_t lastChange_ = 0, pressAt_ = 0;
};
