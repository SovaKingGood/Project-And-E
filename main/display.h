#pragma once

#include "lvgl.h"
#include "esp_err.h"
#include <stdbool.h>
#include "espnow_task.h"
#include "esp_timer.h"

/* Initialize HW (SPI bus, ST7789, backlight), start esp_lvgl_port,
 * add LVGL display, create/pin the touch polling task, and build the UI. */
esp_err_t display_init(void);

/* Optional helpers */
void display_set_backlight(bool on);
bool display_lock(uint32_t timeout_ms);
void display_unlock(void);

/* Accessors */
lv_display_t *display_get_lv_disp(void);

/* Controller data update function (LVGL-safe via lv_async_call) */
void display_update_controller_data(int battery, uint32_t buttons,
    int16_t lx, int16_t ly,
    int16_t rx, int16_t ry,
    uint16_t lt, uint16_t rt);

/* Start the periodic label update task (creates LVGL-safe task that shows
   controller and telemetry status on the UI). Safe to call after display_init(). */
void display_start_label_task(void);

/* Get display health status */
typedef struct {
    uint32_t last_successful_update_ms;
    uint32_t consecutive_failures;
    uint32_t uptime_seconds;
    bool is_healthy;
} display_health_t;

display_health_t display_get_health(void);