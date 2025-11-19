#include <stdlib.h>
#include "btstack_port_esp32.h"
#include <btstack_run_loop.h>
#include <btstack_stdio_esp32.h>
#include <hci_dump.h>
#include <hci_dump_embedded_stdout.h>
#include <uni.h>

#include "sdkconfig.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_heap_caps.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_xpt2046.h"
#include "esp_lcd_panel_io.h"

#include "system_config.h"
#include "display.h"
#include "espnow_task.h"

#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "ui/ui.h"
#include <stdio.h>

#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#error "Must use BLUEPAD32_PLATFORM_CUSTOM"
#endif

struct uni_platform* get_my_platform(void);

/* TAG removed to avoid unused-variable warning */



/* label task moved to display.c */



int app_main(void) {
    /* BTstack stdio MUST be first - before any other init */
#ifdef CONFIG_ESP_CONSOLE_UART
#ifndef CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
    btstack_stdio_init();
#endif
#endif

    /* Display */
    ESP_ERROR_CHECK(display_init());

    /* Start label task via display module */
    display_start_label_task();

    /* NVS (wifi radio) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

#if ESPNOW_ENABLED
    ESP_ERROR_CHECK(espnow_task_init());
    ESP_ERROR_CHECK(espnow_task_start());
#endif

    /* Bluepad/BTstack - init after all peripherals ready */
    btstack_init();
    
    /* Must be called before uni_init() */
    uni_platform_set_custom(get_my_platform());
    
    /* Init Bluepad32 */
    uni_init(0, NULL);
    
    /* This blocks forever running Bluetooth event loop */
    btstack_run_loop_execute();
    return 0;
}
