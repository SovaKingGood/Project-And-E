#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include <uni.h>
#include "espnow_task.h"
#include "display.h"
#include "esp_timer.h"

typedef struct my_platform_instance_s {
    uni_gamepad_seat_t gamepad_seat;
} my_platform_instance_t;

/* Track connected controllers for simple queries */
static int connected_controller_count = 0;

int my_platform_get_connected_controller_count(void) {
    return connected_controller_count;
}

static void trigger_event_on_gamepad(uni_hid_device_t* d);
static my_platform_instance_t* get_my_platform_instance(uni_hid_device_t* d);

static void my_platform_init(int argc, const char** argv) {
    (void)argc; (void)argv;
}

static void my_platform_on_init_complete(void) {
    ESP_LOGI("PLAT", "🔵 Bluetooth init complete");
    
    uni_bt_allow_incoming_connections(true);
    uni_bt_start_scanning_and_autoconnect_unsafe();
    uni_bt_del_keys_unsafe();
}

static uni_error_t my_platform_on_device_discovered(bd_addr_t addr, const char* name, uint16_t cod, uint8_t rssi) {
    if (((cod & UNI_BT_COD_MINOR_MASK) & UNI_BT_COD_MINOR_KEYBOARD) == UNI_BT_COD_MINOR_KEYBOARD)
        return UNI_ERROR_IGNORE_DEVICE;
    return UNI_ERROR_SUCCESS;
}

static void my_platform_on_device_connected(uni_hid_device_t* d) {
    extern uint32_t esp_get_free_heap_size(void);
    (void)d;
    connected_controller_count++;
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║  ✅ CONTROLLER CONNECTED! Heap: %u bytes free            \n", (unsigned)esp_get_free_heap_size());
    printf("╚════════════════════════════════════════════════════════════╝\n");
    printf("\n");
}

static void my_platform_on_device_disconnected(uni_hid_device_t* d) {
    extern uint32_t esp_get_free_heap_size(void);
    (void)d;
    if (connected_controller_count > 0) connected_controller_count--;
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║  ❌ CONTROLLER DISCONNECTED! Heap: %u bytes free         \n", (unsigned)esp_get_free_heap_size());
    printf("╚════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    ESP_LOGI("PLAT", "If reconnecting fails, ensure controller is disconnected from PS4/other devices");
}

static uni_error_t my_platform_on_device_ready(uni_hid_device_t* d) {
    my_platform_instance_t* ins = get_my_platform_instance(d);
    ins->gamepad_seat = GAMEPAD_SEAT_A;
    trigger_event_on_gamepad(d);
    return UNI_ERROR_SUCCESS;
}

static void my_platform_on_controller_data(uni_hid_device_t* d, uni_controller_t* ctl) {
    (void)d;
    static uni_controller_t prev = {0};
    if (memcmp(&prev, ctl, sizeof(*ctl)) == 0) return;
    prev = *ctl;

    if (ctl->klass == UNI_CONTROLLER_CLASS_GAMEPAD) {
        const uni_gamepad_t* gp = &ctl->gamepad;

        espnow_controller_data_t controller_data = {
            .packet_id = 0,
            .timestamp_ms = esp_timer_get_time() / 1000,

            .left_stick_x = (int16_t)gp->axis_x,
            .left_stick_y = (int16_t)gp->axis_y,
            .right_stick_x = (int16_t)gp->axis_rx,
            .right_stick_y = (int16_t)gp->axis_ry,

            .left_trigger  = (uint8_t)(gp->brake    / 4),
            .right_trigger = (uint8_t)(gp->throttle / 4),

            .buttons_raw = gp->buttons,
            .dpad        = gp->dpad,
            .misc_buttons = gp->misc_buttons,

            .gyro_x = (int16_t)(gp->gyro[0] / 10),
            .gyro_y = (int16_t)(gp->gyro[1] / 10),
            .gyro_z = (int16_t)(gp->gyro[2] / 10),
            .accel_x = (int16_t)gp->accel[0],
            .accel_y = (int16_t)gp->accel[1],
            .accel_z = (int16_t)gp->accel[2],

            .controller_battery = ctl->battery,
            .drive_mode   = DRIVE_MODE_PARK,
            .park_request = 0,
            .source       = CTRL_SOURCE_CONTROLLER,

            .timestamp = esp_timer_get_time() / 1000,
            .checksum  = 0
        };

        if (gp->misc_buttons & 0x01) controller_data.park_request = 1;

        espnow_update_controller_data(&controller_data);
    }
}

static const uni_property_t* my_platform_get_property(uni_property_idx_t idx) { (void)idx; return NULL; }

static void my_platform_on_oob_event(uni_platform_oob_event_t event, void* data) {
    (void)data;
    switch (event) {
    default: break;
    }
}

static my_platform_instance_t* get_my_platform_instance(uni_hid_device_t* d) {
    return (my_platform_instance_t*)&d->platform_data[0];
}

static void trigger_event_on_gamepad(uni_hid_device_t* d) {
    my_platform_instance_t* ins = get_my_platform_instance(d);
    if (d->report_parser.set_player_leds)     d->report_parser.set_player_leds(d, ins->gamepad_seat);
    if (d->report_parser.set_lightbar_color)  d->report_parser.set_lightbar_color(d, 0x10, 0x10, 0x30);
}

struct uni_platform* get_my_platform(void) {
    static struct uni_platform plat = {
        .name = "custom",
        .init = my_platform_init,
        .on_init_complete = my_platform_on_init_complete,
        .on_device_discovered = my_platform_on_device_discovered,
        .on_device_connected = my_platform_on_device_connected,
        .on_device_disconnected = my_platform_on_device_disconnected,
        .on_device_ready = my_platform_on_device_ready,
        .on_oob_event = my_platform_on_oob_event,
        .on_controller_data = my_platform_on_controller_data,
        .get_property = my_platform_get_property,
    };
    return &plat;
}
