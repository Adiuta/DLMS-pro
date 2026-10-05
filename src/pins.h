#pragma once

// ========================================================
// 1. PINOUT BLOK KIRI (Kabel Urut Fisik & Rapi)
// ========================================================

// --- LCD Modul (TFT_eSPI) ---
#define TFT_MISO -1   // tidak dipakai
#define TFT_SCLK 4    // LCD_SCL
#define TFT_MOSI 5    // LCD_SDA
#define TFT_RST 6     // LCD_RES
#define TFT_DC 7      // LCD_DC
#define TFT_CS 15     // LCD_CS
#define PIN_TFT_BL 16 // LCD_BL

// --- IC PCF8574 (I2C) ---
#define I2C_SCL 17
#define I2C_SDA 18

// --- Rotary Encoder ---
#define ENC_A_PIN 9   // ENCODER_CLK
#define ENC_B_PIN 10  // ENCODER_DT
#define ENC_SW_PIN 11 // ENCODER_SW
#define ENC_REVERSE 1 // 1 = arah putar terbalik

// --- IC TM1640 ---
#define TM1640_CLK 12
#define TM1640_DIN 13

// ========================================================
// 2. PINOUT BLOK KANAN (Audio Master Sinyal Bersih)
// ========================================================

#define PIN_ADC_BCK 41 // I2S_BCK  : Bit Clock (Paralel ke semua modul)
#define PIN_ADC_LRC 42 // I2S_LRC  : Word Select / LRCK (Paralel ke semua modul)
#define PIN_ADC_SCK 40 // I2S_SCK  : Master Clock / MCLK (Ke SCK/SCKI)

// Koneksi Data I2S:
#define PIN_DAC1_DIN 21 // I2S_DOUT : Data Out ke PCM5102 #1
#define PIN_DAC2_DIN 39 // I2S_DIN1 : Data Out ke PCM5102 #2
#define PIN_ADC_DOUT 38 // I2S_DIN2 : Data In dari PCM1808 ADC
