/**
 * @file espnow_task.h
 * @brief ESP-NOW Communication Task
 */
#ifndef ESPNOW_TASK_H
#define ESPNOW_TASK_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "system_config.h"

/* Who initiated a change (for future merge logic) */
typedef enum {
    CTRL_SOURCE_CONTROLLER = 0,
    CTRL_SOURCE_DISPLAY    = 1,
} ctrl_source_t;

/* =================== CONTROLLER (command) PACKET =================== */
typedef struct {
    /* header */
    uint32_t packet_id;
    uint32_t timestamp_ms;

    /* analog */
    int16_t left_stick_x, left_stick_y;
    int16_t right_stick_x, right_stick_y;
    uint8_t left_trigger, right_trigger;

    /* digital */
    union {
        uint16_t buttons_raw;
        struct {
            uint16_t dpad_up:1, dpad_down:1, dpad_left:1, dpad_right:1;
            uint16_t button_x:1, button_a:1, button_b:1, button_y:1;
            uint16_t l1:1, r1:1, l3:1, r3:1;
            uint16_t select:1, start:1, home:1, reserved:1;
        } buttons;
    };
    uint8_t dpad;          /* as reported by Bluepad32 */
    uint8_t misc_buttons;  /* select/start/home, etc. */

    /* motion */
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t accel_x, accel_y, accel_z;

    // Servo calibration ranges (min/max microseconds) for up to 5 servos
    int16_t servo_min[5];
    int16_t servo_max[5];

    /* state */
    uint8_t controller_battery;   /* 0..100 */
    uint8_t drive_mode;           /* drive_mode_t */
    uint8_t park_request;         /* 1 => request PARK/STOP */
    uint8_t source;               /* ctrl_source_t */

    /* trailing book-keeping */
    uint32_t timestamp;           /* ms */
    uint8_t  checksum;
} __attribute__((packed)) espnow_controller_data_t;

/* =================== TELEMETRY PACKET (car -> UI) =================== */
typedef struct {
    int32_t rpm;
    float accel_x, accel_y, accel_z;  /* Accelerometer data in g */
    float gyro_x,  gyro_y,  gyro_z;   /* Gyroscope data in dps */
    uint16_t batt_mv;
    uint32_t timestamp_ms;
    uint8_t  reserved;
    uint8_t  checksum;
} __attribute__((packed)) espnow_telemetry_t;

/* =================== UTILITY FUNCTIONS =================== */
/* Get current timestamp in milliseconds */
static inline uint32_t get_current_timestamp_ms(void) {
    return esp_timer_get_time() / 1000;
}

/* Check if timestamp is recent (within timeout_ms) */
static inline bool is_timestamp_recent(uint32_t timestamp_ms, uint32_t timeout_ms) {
    uint32_t current = get_current_timestamp_ms();
    return (timestamp_ms > 0) && (current - timestamp_ms < timeout_ms);
}

/* API */
esp_err_t espnow_task_init(void);
esp_err_t espnow_task_start(void);

void espnow_update_controller_data(const espnow_controller_data_t* data);
void espnow_set_drive_mode(uint8_t mode);
void espnow_set_park_request(bool on);

void espnow_get_stats(uint32_t* sent_count, uint32_t* failed_count, uint32_t* latency_ms);
void espnow_get_current_controller_data(espnow_controller_data_t* data);

/* new: pull latest telemetry if we received one */
bool espnow_get_last_telemetry(espnow_telemetry_t* out);

/* Get timestamp when telemetry was last received */
uint32_t espnow_get_last_telemetry_timestamp(void);

void espnow_task_cleanup(void);

#endif /* ESPNOW_TASK_H */
