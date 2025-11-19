#include "espnow_task.h"
#include "system_config.h"
#include "esp_log.h"
#include "display.h"
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char* TAG = "ESPNOW_TASK";

/* State */
static espnow_controller_data_t current_data = {0};
static espnow_controller_data_t safe_read_data = {0};
static espnow_telemetry_t       last_telem = {0};
static bool data_updated = false;
static SemaphoreHandle_t data_mutex = NULL;
static uint32_t last_telemetry_received = 0;  /* Timestamp when we last received car telemetry */

/* Peer */
static esp_now_peer_info_t peer_info = {0};
static uint8_t target_mac[6] = ESPNOW_TARGET_MAC;

/* Stats */
static uint32_t packet_counter = 0;
static uint32_t sent_count = 0;
static uint32_t failed_count = 0;
static uint32_t last_latency_ms = 0;

/* Task */
static TaskHandle_t espnow_task_handle = NULL;

/* Simple XOR checksum (leave as-is for speed). */
static uint8_t checksum8(const void* p, size_t n) {
    const uint8_t* b = (const uint8_t*)p;
    uint8_t c = 0;
    for (size_t i = 0; i < n; ++i) c ^= b[i];
    return c;
}

/* Callbacks */
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        sent_count++;
        ESP_LOGD(TAG, "📤 Send SUCCESS to " MACSTR, MAC2STR(mac_addr));
    } else {
        failed_count++;
        ESP_LOGW(TAG, "❌ Send FAILED to " MACSTR, MAC2STR(mac_addr));
    }
}

static void espnow_recv_cb(const esp_now_recv_info_t *ri, const uint8_t *data, int len) {
    (void)ri;
    ESP_LOGD(TAG, "📨 ESPNOW packet received: %d bytes", len);

    /* Try interpret as telemetry first */
    if (len == (int)sizeof(espnow_telemetry_t)) {
        espnow_telemetry_t t;
        memcpy(&t, data, sizeof(t));
        uint8_t cs = t.checksum;
        ((espnow_telemetry_t*)&t)->checksum = 0;
        if (checksum8(&t, sizeof(t)) == cs) {
            last_telem = t;
            /* Update telemetry reception timestamp for connection detection */
            last_telemetry_received = esp_timer_get_time() / 1000;

            /* Push latest telemetry into UI model (no LVGL calls here) */
            display_set_telemetry_state(&t);

            ESP_LOGI(TAG, "📡 Telemetry received: rpm=%ld batt=%.2fV gyro=(%.1f,%.1f,%.1f) accel=(%.3f,%.3f,%.3f)",
                     (long)t.rpm, (double)t.batt_mv/1000.0,
                     (double)t.gyro_x, (double)t.gyro_y, (double)t.gyro_z,
                     (double)t.accel_x, (double)t.accel_y, (double)t.accel_z);
            return;
        } else {
            ESP_LOGW(TAG, "❌ Telemetry checksum failed!");
        }
    } else {
        ESP_LOGD(TAG, "📦 Packet size %d != telemetry size %d", len, (int)sizeof(espnow_telemetry_t));
    }
    /* If someone echoes back our command packet, ignore silently */
}

/* WiFi init for ESPNOW */
static esp_err_t espnow_wifi_init(void) {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    return ESP_OK;
}

static esp_err_t espnow_add_peer(void) {
    memset(&peer_info, 0, sizeof(peer_info));
    memcpy(peer_info.peer_addr, target_mac, 6);
    peer_info.channel = ESPNOW_CHANNEL;
    peer_info.ifidx = ESP_IF_WIFI_STA;
    peer_info.encrypt = false;
    return esp_now_add_peer(&peer_info);
}

/* Task: send at configured rate; only when data_updated */
static void espnow_communication_task(void* arg) {
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(ESPNOW_UPDATE_INTERVAL_MS);
    for (;;) {
        vTaskDelayUntil(&last, period);
        if (!data_updated) continue;

        if (data_mutex && xSemaphoreTake(data_mutex, 0) == pdTRUE) {
            current_data.packet_id = ++packet_counter;
            current_data.timestamp_ms = esp_timer_get_time() / 1000;
            current_data.checksum = 0;
            current_data.checksum = checksum8(&current_data, sizeof(current_data));
            
            /* Send packet - logging removed to prevent stack overflow */
            esp_err_t send_result = esp_now_send(target_mac, (const uint8_t*)&current_data, sizeof(current_data));
            if (send_result != ESP_OK) {
                ESP_LOGE(TAG, "❌ ESPNOW send failed: %s", esp_err_to_name(send_result));
            } else {
                ESP_LOGD(TAG, "✅ ESPNOW send OK");
            }
            /* copy to safe snapshot the same moment */
            safe_read_data = current_data;
            data_updated = false;
            xSemaphoreGive(data_mutex);
        }
    }
}

/* Public API */
esp_err_t espnow_task_init(void) {
    ESP_ERROR_CHECK(espnow_wifi_init());
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    ESP_ERROR_CHECK(espnow_add_peer());

    memset(&current_data, 0, sizeof(current_data));
    current_data.drive_mode = DRIVE_MODE_PARK;
    current_data.source     = CTRL_SOURCE_CONTROLLER;

    data_mutex = xSemaphoreCreateMutex();
    return data_mutex ? ESP_OK : ESP_FAIL;
}

esp_err_t espnow_task_start(void) {
    BaseType_t ok = xTaskCreatePinnedToCore(
        espnow_communication_task, "espnow_comm",
        ESPNOW_TASK_STACK, NULL, ESPNOW_TASK_PRIORITY,
        &espnow_task_handle, ESPNOW_TASK_CORE);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}

void espnow_update_controller_data(const espnow_controller_data_t* data) {
    if (!data || !data_mutex) return;
    if (xSemaphoreTake(data_mutex, 0) == pdTRUE) {
        current_data = *data;
        data_updated = true;
        /* Push latest controller state into UI model for Screen2 display (no LVGL calls here) */
        display_set_controller_state(&current_data);
        xSemaphoreGive(data_mutex);
    }
}

void espnow_set_drive_mode(uint8_t mode) {
    if (mode > DRIVE_MODE_ECO) return;
    current_data.drive_mode = mode;
    current_data.source = CTRL_SOURCE_DISPLAY;
    data_updated = true;
}

void espnow_set_park_request(bool on) {
    current_data.park_request = on ? 1 : 0;
    current_data.source = CTRL_SOURCE_DISPLAY;
    data_updated = true;
}

void espnow_get_stats(uint32_t* s, uint32_t* f, uint32_t* l) {
    if (s) {
        *s = sent_count;
    }
    if (f) {
        *f = failed_count;
    }
    if (l) {
        *l = last_latency_ms;
    }
}



void espnow_get_current_controller_data(espnow_controller_data_t* out) {
    if (!out) return;
    *out = safe_read_data;
}

bool espnow_get_last_telemetry(espnow_telemetry_t* out) {
    if (!out) return false;
    *out = last_telem;
    return (last_telem.timestamp_ms != 0);
}

uint32_t espnow_get_last_telemetry_timestamp(void) {
    return last_telemetry_received;
}

void espnow_task_cleanup(void) {
    if (espnow_task_handle) vTaskDelete(espnow_task_handle);
    esp_now_del_peer(target_mac);
    esp_now_deinit();
}
