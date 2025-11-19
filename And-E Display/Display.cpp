#include "Display.h"
#include "ESPNowComm.h"   // you referenced external vars here
#include <SD.h>
#include <SPI.h>

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite sprite = TFT_eSprite(&tft);

int fillWidth = 0;
Cursor Cursor1 = {};

// --- Transparency color (pure magenta #FF00FF in RGB565) ---
#define TRANSPARENT_COLOR 0xF81F

static bool sdOK = false;
static bool spriteOK = false;

// ---------- small helpers ----------
static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// Minimal 24-bit BMP → draw into SPRITE with magenta key
void drawBmpTransparentToSprite(const char *filename, int16_t x, int16_t y) {
  if (!sdOK || !spriteOK) return;

  File bmpFile = SD.open(filename, FILE_READ);
  if (!bmpFile) {
    Serial.printf("[ERR] File not found: %s\n", filename);
    return;
  }

  // --- Basic BMP parse (uncompressed 24-bit only) ---
  uint16_t sig = bmpFile.read() | (bmpFile.read() << 8);
  if (sig != 0x4D42) { Serial.println("[ERR] Not a BMP"); bmpFile.close(); return; }

  // header fields we need
  bmpFile.seek(10);
  uint32_t pixelOffset = 0; bmpFile.read((uint8_t*)&pixelOffset, 4);

  bmpFile.seek(18);
  int32_t bmpWidth = 0, bmpHeight = 0;
  bmpFile.read((uint8_t*)&bmpWidth, 4);
  bmpFile.read((uint8_t*)&bmpHeight, 4);

  bmpFile.seek(28);
  uint16_t bpp = bmpFile.read() | (bmpFile.read() << 8);
  if (bpp != 24) { Serial.println("[ERR] Only 24-bit BMP supported"); bmpFile.close(); return; }

  // Move to pixel data
  bmpFile.seek(pixelOffset);

  // Row size is padded to 4-byte boundary
  uint32_t rowBytes = (uint32_t)bmpWidth * 3;
  uint32_t rowPad   = (4 - (rowBytes & 3)) & 3;

  Serial.printf("[INFO] BMP: %ld x %ld\n", (long)bmpWidth, (long)bmpHeight);

  // BMP is stored bottom-up
  for (int32_t row = bmpHeight - 1; row >= 0; row--) {
    for (int32_t col = 0; col < bmpWidth; col++) {
      uint8_t b = bmpFile.read();
      uint8_t g = bmpFile.read();
      uint8_t r = bmpFile.read();

      uint16_t c = rgb565(r, g, b);
      if (c != TRANSPARENT_COLOR) {
        sprite.drawPixel(x + col, y + row, c);
      }
    }
    for (uint32_t i = 0; i < rowPad; i++) (void)bmpFile.read();
  }

  bmpFile.close();
}

// ---------- Public API ----------
void setupDisplay() {
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);

  // Create a full-screen sprite (TFT_eSPI will use PSRAM if available)
  sprite.setColorDepth(16); // RGB565
  
  if (sprite.createSprite(SCREEN_WIDTH, SCREEN_HEIGHT) == nullptr) {
    Serial.println("[ERR] Sprite alloc failed (PSRAM?) — drawing direct to TFT.");
    spriteOK = false;
  } else {
    spriteOK = true;
    sprite.fillSprite(TFT_BLACK);
    Serial.printf("[OK] Sprite %dx%d created (likely in PSRAM)\n", SCREEN_WIDTH, SCREEN_HEIGHT);
  }

  // SD card
  sdOK = SD.begin(SD_CS);
  Serial.println(sdOK ? "[OK] SD card mounted" : "[ERR] SD card mount failed");

  if (sdOK) {
    File root = SD.open("/");
    for (;;) {
      File f = root.openNextFile();
      if (!f) break;
      Serial.println(f.name());
      f.close();
    }
  }

  // Example background image into sprite
  if (sdOK && spriteOK) {
    drawBmpTransparentToSprite("/test.bmp", 0, 0);
    sprite.pushSprite(0, 0);
  }
}

void fillGradient(TFT_eSprite &spr, uint16_t topColor, uint16_t bottomColor) {
  for (int y = 0; y < spr.height(); y++) {
    float ratio = (float)y / (spr.height() - 1);
    uint8_t r = ((topColor >> 11) & 0x1F) * (1 - ratio) + ((bottomColor >> 11) & 0x1F) * ratio;
    uint8_t g = ((topColor >> 5) & 0x3F) * (1 - ratio) + ((bottomColor >> 5) & 0x3F) * ratio;
    uint8_t b = (topColor & 0x1F) * (1 - ratio) + (bottomColor & 0x1F) * ratio;
    spr.drawFastHLine(0, y, spr.width(), (r << 11) | (g << 5) | b);
  }
}

void drawGradientHLine(TFT_eSprite &spr, int x, int y, int w, int h,
                       uint16_t startColor, uint16_t endColor) {
  for (int i = 0; i < w; i++) {
    float ratio = (float)i / (w - 1);

    // extract RGB565 to 8-bit channels
    uint8_t r1 = ((startColor >> 11) & 0x1F) << 3;
    uint8_t g1 = ((startColor >> 5)  & 0x3F) << 2;
    uint8_t b1 = ( startColor        & 0x1F) << 3;

    uint8_t r2 = ((endColor >> 11) & 0x1F) << 3;
    uint8_t g2 = ((endColor >> 5)  & 0x3F) << 2;
    uint8_t b2 = ( endColor        & 0x1F) << 3;

    // interpolate
    uint8_t r = r1 + (r2 - r1) * ratio;
    uint8_t g = g1 + (g2 - g1) * ratio;
    uint8_t b = b1 + (b2 - b1) * ratio;

    uint16_t color = spr.color565(r, g, b);

    // draw vertical thickness
    for (int j = 0; j < h; j++) {
      spr.drawPixel(x + i, y + j, color);
    }
  }
}




void updateDisplay() {
  // If sprite allocation failed, draw direct to TFT as a fallback
  if (!spriteOK) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("Sprite alloc failed", SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2);
    return;
  }



  // Clear this frame
  fillGradient(sprite, tft.color565(230, 216, 230), tft.color565(100, 150, 245)); 



  // --- UI drawing (your original, but to 'sprite' instead of 'tft') ---
  sprite.setTextDatum(MC_DATUM);
  sprite.setTextSize(2);
  sprite.setTextColor(TFT_BLACK);

  // Title
  sprite.drawString("And-E!", SCREEN_WIDTH / 2, 10);
  drawGradientHLine(sprite, 0, 20, SCREEN_WIDTH, 10, 
                  tft.color565(173, 216, 230), 
                  tft.color565(0, 102, 204));
                  



  // if (OTA_Mode == 1) {
  //   if (connectedToOTA == 1) {
  //     sprite.drawString("OTA Connected!", SCREEN_WIDTH / 2, SCREEN_HEIGHT - 20);
  //   } else {
  //     sprite.drawString("OTA Searching...", SCREEN_WIDTH / 2, SCREEN_HEIGHT - 20);
  //   }
  // } else {
  //   if (espnow_status == 1) {
  //     sprite.drawString("Connected to And-E!", SCREEN_WIDTH / 2, SCREEN_HEIGHT - 20);
  //     failedSends = 0;
  //   } else {
  //     sprite.drawString("We can't hear And-E :(", SCREEN_WIDTH  / 2, SCREEN_HEIGHT - 20);
  //     char failBuf[32];
  //     sprintf(failBuf, "Packets Dropped: %d", failedSends);
  //     sprite.drawString(failBuf, SCREEN_WIDTH / 2,  SCREEN_HEIGHT - 12);
  //   }
  // }

  // Throttle slider
  char throttleBuf[32];
  sprintf(throttleBuf, "Throttle: %d%%", map(PS4.throttle, 0, 1020, 0, 100));
  sprite.drawString(throttleBuf, SCREEN_WIDTH / 2, 50);

  sprite.fillRect(5, 60, SLIDER_WIDTH, SLIDER_HEIGHT, TFT_DARKGREY);
  fillWidth = map(PS4.throttle, 0, 1020, 0, SLIDER_WIDTH);
  sprite.fillRect(5, 60, fillWidth, SLIDER_HEIGHT, TFT_GREEN);

  // // RPM slider
  // char rpmBuf[32];
  // sprintf(rpmBuf, "RPM: %d", AndE.RPM);
  // sprite.drawString(rpmBuf, SCREEN_WIDTH / 2, 55);

  // sprite.fillRect(SCREEN_WIDTH + 5, 65, SLIDER_WIDTH, SLIDER_HEIGHT, TFT_DARKGREY);
  // fillWidth = map(AndE.RPM, 0, 70000, 0, SLIDER_WIDTH);
  // sprite.fillRect(SCREEN_WIDTH + 5, 65, fillWidth, SLIDER_HEIGHT, TFT_GREEN);

  // If you want the BMP on top each frame, uncomment:
  // if (sdOK) drawBmpTransparentToSprite("/test.bmp", 0, 0);

  // Push the whole frame at once
  sprite.pushSprite(0, 0);
}
