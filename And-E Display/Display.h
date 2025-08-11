#pragma once
#include <TFT_eSPI.h>

// --- TFT pins (your defines kept for reference if needed by your setup) ---
#define TFT_CS     15
#define TFT_DC     2
#define TFT_RST    -1
#define TFT_SCK    14
#define TFT_MOSI   13
#define TFT_MISO   12
#define TFT_BL     27

#define SD_CS 5

// Display settings
#define SCREEN_WIDTH   320
#define SCREEN_HEIGHT  240

#define SLIDER_WIDTH   150
#define SLIDER_HEIGHT  10
#define SLIDER_X       14
#define SLIDER_Y       100

#define UI_info_Width  0
#define UI_info_Height 0

// Globals provided by this module
extern TFT_eSPI tft;
extern TFT_eSprite sprite;

extern int fillWidth;

typedef struct {
  uint8_t x;
  uint8_t y;
} Cursor;

extern Cursor Cursor1;

// API
void setupDisplay();
void updateDisplay();

// Helpers (exposed in case you want them)
void drawBmpTransparentToSprite(const char *filename, int16_t x, int16_t y);
