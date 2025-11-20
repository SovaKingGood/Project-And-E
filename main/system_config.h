#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include "sdkconfig.h"
#include <stdint.h>
#include <stdbool.h>

/* ========================================================================
 * HARDWARE CONFIGURATION
 * ======================================================================== */

/* ===================== SPI BUS (shared by ST7789 + XPT2046) ===================== */
#define DISPLAY_SPI_HOST        SPI3_HOST   /* VSPI */

/* Shared SCLK/MOSI; MISO only needed by touch */
#define DISPLAY_PIN_CLK         14
#define DISPLAY_PIN_MOSI        13
#define DISPLAY_PIN_MISO        -1          /* ST7789 is write-only */
#define TOUCH_PIN_MISO          12
#define TOUCH_PIN_MOSI          DISPLAY_PIN_MOSI
#define TOUCH_PIN_CLK           DISPLAY_PIN_CLK

/* =============================== DISPLAY (ST7789) =============================== */
#define DISPLAY_WIDTH           320
#define DISPLAY_HEIGHT          240
#define DISPLAY_ROTATION        0
#define DISPLAY_PIN_CS          15
#define DISPLAY_PIN_DC          2
#define DISPLAY_PIN_RST         -1          /* -1 if tied to reset/POR */
#define DISPLAY_PIN_BCKL        27          /* active-high */
#define DISPLAY_SPI_FREQ_HZ     (27 * 1000 * 1000)

/* =============================== TOUCH (XPT2046) =============================== */
#define TOUCH_PIN_CS            33
#define TOUCH_PIN_IRQ           36          /* active low (needs pull-up) */
#define TOUCH_SPI_FREQ_HZ       (1 * 1000 * 1000)
#define TOUCH_SWAP_XY           1
#define TOUCH_MIRROR_X          1
#define TOUCH_MIRROR_Y          1
#define TOUCH_PRESSURE_MIN      150
#define TOUCH_POLL_PERIOD_MS    500         /* Slower polling - not time critical */
#define TOUCH_TASK_CORE         1           /* CPU 1 with LVGL */
#define TOUCH_TASK_PRIO         1           /* Lowest priority - not time critical */
#define TOUCH_TASK_STACK        1000

/* ========================================================================
 * TASK CONFIGURATION
 * ======================================================================== */

/* =============================== LVGL CONFIG =============================== */
#define LVGL_TICK_PERIOD_MS     50          /* Slower updates - 20fps is fine for status display */
#define LVGL_TASK_PRIORITY      15          /* Higher than draw task (3) to prevent suspension, but below WiFi (23) */
#define LVGL_TASK_STACK_SIZE    16384       /* 16KB - plenty of headroom for complex UI operations + controller input */
#define TASK_CORE_DISPLAY       1           /* CPU 1 - isolated from Bluetooth/WiFi */
#ifdef CONFIG_SPIRAM
  #define LVGL_BUFFER_PIXELS    (DISPLAY_WIDTH * 20)
  #define LVGL_DOUBLE_BUFFER    0
#else
  #define LVGL_BUFFER_PIXELS    (DISPLAY_WIDTH * 10)
  #define LVGL_DOUBLE_BUFFER    0
#endif

/* =============================== DRIVE MODES =============================== */
typedef enum {
    DRIVE_MODE_PARK  = 0,
    DRIVE_MODE_SPORT = 1,
    DRIVE_MODE_DRIVE = 2,
    DRIVE_MODE_ECO   = 3,
} drive_mode_t;

/* ========================================================================
 * TIMING CONFIGURATION
 * ======================================================================== */

/* =============================== ESP-NOW CONFIG =============================== */
#define ESPNOW_ENABLED              1           /* was 0 — enable radio path */
#define ESPNOW_TARGET_MAC           {0x8C,0xBF,0xEA,0x04,0x13,0x9C}
#define ESPNOW_CHANNEL              1
#define ESPNOW_UPDATE_INTERVAL_MS   20          /* 50Hz for RC control - LOW LATENCY! */
#define ESPNOW_MAX_RETRY            2           /* Reduced retries for lower latency */
#define ESPNOW_QUEUE_SIZE           5           /* Smaller queue - fresher data */
#define ESPNOW_TASK_PRIORITY        8           /* High priority for RC control but not starving display */
#define ESPNOW_TASK_STACK           3072        /* Reduced from 4096 - task is lightweight, only sends packets */
#define ESPNOW_TASK_CORE            0           /* Same CPU as WiFi driver for minimal latency */

/* =============================== UI/Label update =============================== */
#define LABEL_UPDATE_TASK_PRIORITY  15          /* Same as LVGL task - prevents priority inversion deadlock */
#define LABEL_UPDATE_TASK_STACK     1536        /* Reduced from 2048 - monitor task is lightweight */
#define LABEL_UPDATE_TASK_CORE      1           /* CPU 1 with LVGL - keep CPU 0 free for RF */
#define LABEL_UPDATE_INTERVAL_MS    200         /* 5Hz - slower updates, CPU 0 stays free */

/* =============================== MISC/DEBUG =============================== */
#define ENABLE_TOUCH_SUPPORT        1  /* Touch screen enabled */

#endif /* SYSTEM_CONFIG_H */
