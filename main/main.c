/**
 * @file main.c
 * @brief And-E UI Controller - High-Level Program Flow
 * 
 * This file shows the system architecture at a glance:
 * - Data structures (what flows through the system)
 * - Initialization sequence
 * - Task creation
 */

#include <stdlib.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

/* Bluepad32 / BTstack */
#include "btstack_port_esp32.h"
#include <btstack_run_loop.h>
#include <btstack_stdio_esp32.h>
#include <hci_dump.h>
#include <hci_dump_embedded_stdout.h>
#include <uni.h>

/* Project modules */
#include "system_config.h"
#include "display.h"
#include "espnow_task.h"

#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#error "Must use BLUEPAD32_PLATFORM_CUSTOM"
#endif

/* Bluepad32 platform definition */
struct uni_platform* get_my_platform(void);

/* ========================================================================
 * DATA STRUCTURES - What flows through the system
 * ======================================================================== */

/**
 * espnow_controller_data_t - Controller → Car (Command Packet)
 * 
 * Sent via ESP-NOW at 50Hz (20ms intervals)
 * 
 * Fields:
 *   - packet_id, timestamp_ms: Header info
 *   - left_stick_x/y, right_stick_x/y: Analog sticks (-512 to +511)
 *   - left_trigger, right_trigger: L2/R2 triggers (0-255)
 *   - buttons_raw / buttons: Digital button states (bitfield)
 *   - gyro_x/y/z, accel_x/y/z: Motion sensor data from PS4 controller
 *   - controller_battery: 0-100%
 *   - drive_mode: 0=PARK, 1=SPORT, 2=DRIVE, 3=ECO
 *   - checksum: Data integrity check
 * 
 * Defined in: espnow_task.h
 */

/**
 * espnow_telemetry_t - Car → Controller (Telemetry Packet)
 * 
 * Received via ESP-NOW when car responds
 * 
 * Fields:
 *   - rpm: Motor RPM
 *   - accel_x/y/z: Accelerometer data (g)
 *   - gyro_x/y/z: Gyroscope data (dps)
 *   - batt_mv: Battery voltage (millivolts)
 *   - timestamp_ms: When packet was sent
 *   - checksum: Data integrity check
 * 
 * Defined in: espnow_task.h
 */

/**
 * drive_mode_t - Drive modes
 * 
 * Values:
 *   - DRIVE_MODE_PARK  = 0
 *   - DRIVE_MODE_SPORT = 1
 *   - DRIVE_MODE_DRIVE = 2
 *   - DRIVE_MODE_ECO   = 3
 * 
 * Defined in: system_config.h
 */

/* ========================================================================
 * SYSTEM INITIALIZATION & MAIN
 * ======================================================================== */

int app_main(void) {
    /* ---- 1. CONSOLE INIT (must be first) ---- */
#ifdef CONFIG_ESP_CONSOLE_UART
#ifndef CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
    btstack_stdio_init();
#endif
#endif

    printf("\n");
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║  And-E UI Controller Starting...                          ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n");
    printf("\n");

    /* ---- 2. HARDWARE INIT ---- */
    /* NVS (required for WiFi/Bluetooth) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Display (ST7789 LCD + XPT2046 Touch + LVGL) */
    ESP_ERROR_CHECK(display_init());

    /* ---- 3. START TASKS ---- */
    /* LVGL UI update task (CPU1, priority 15) */
    display_start_label_task();

#if ESPNOW_ENABLED
    /* ESP-NOW communication task (CPU0, priority 8) */
    ESP_ERROR_CHECK(espnow_task_init());
    ESP_ERROR_CHECK(espnow_task_start());
#endif

    printf("\n");
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║  Initializing Bluetooth (Bluepad32)...                    ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n");
    printf("\n");

    /* ---- 4. BLUETOOTH INIT (Bluepad32 for PS4 controller) ---- */
    btstack_init();
    uni_platform_set_custom(get_my_platform());
    uni_init(0, NULL);

    printf("\n");
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║  System Ready - Waiting for PS4 Controller...            ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n");
    printf("\n");

    /* ---- 5. RUN BLUETOOTH EVENT LOOP (blocks forever) ---- */
    btstack_run_loop_execute();
    
    return 0;
}
