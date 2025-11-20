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

/* Legacy controller data update function (kept for compatibility) */
void display_update_controller_data(int battery, uint32_t buttons,
    int16_t lx, int16_t ly,
    int16_t rx, int16_t ry,
    uint16_t lt, uint16_t rt);

/* Shared UI model update API (NO direct LVGL calls here) */
void display_set_controller_state(const espnow_controller_data_t *state);
void display_set_telemetry_state(const espnow_telemetry_t *tel);

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

/* Safe screen load - prevents rapid successive loads and deadlocks */
void display_safe_screen_load(lv_obj_t *scr);

/* Screen2 cache management - prevents timer from accessing deleted UI objects */
void display_cache_screen2_objects(lv_obj_t *screen2, lv_obj_t *info_label);
void display_clear_screen2_cache(void);

/* Screen3 cache management - enables controller input mapping to UI elements */
void display_cache_screen3_objects(lv_obj_t *screen3, lv_obj_t *throttle_bar, lv_obj_t *brake_bar, lv_obj_t *slider1);
void display_clear_screen3_cache(void);

/* Connection status indicators - update icon blend modes on Screen1 */
void display_update_controller_status(bool connected);
void display_update_espnow_status(bool connected);