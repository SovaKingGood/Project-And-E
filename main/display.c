#include "display.h"
#include "system_config.h"

#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"

#include "esp_lcd_touch.h"
#include "esp_lcd_touch_xpt2046.h"

#include "esp_lvgl_port.h"
#include "lvgl.h"

#include "espnow_task.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "esp_task_wdt.h"

/* ESP-NOW types are now defined in espnow_task.h */

/* SquareLine Studio UI */
#include "ui/ui.h"

static const char *TAG = "DISPLAY";

/* Removed dpad_str - no longer used after display optimization */

/* ---- Tunables ----------------------------------------------------------- */
static const uint32_t LCD_SPI_HZ     = DISPLAY_SPI_FREQ_HZ;  // e.g. 40 MHz
#if ENABLE_TOUCH_SUPPORT
static const uint32_t TOUCH_SPI_HZ   = TOUCH_SPI_FREQ_HZ;    // e.g. 1 MHz
#endif
static const size_t   TRANS_SIZE     = 4096;                 // internal DMA bounce (2–4 KB is safe)

/* Handles */
static esp_lcd_panel_io_handle_t  s_panel_io = NULL;
static esp_lcd_panel_handle_t     s_panel    = NULL;

/* LVGL timer for periodic label updates */
static lv_timer_t *s_label_update_timer = NULL;

/* Health monitoring variables */
static uint32_t last_successful_update = 0;


#if ENABLE_TOUCH_SUPPORT
static esp_lcd_panel_io_handle_t  s_touch_io = NULL;
static esp_lcd_touch_handle_t     s_touch    = NULL;

static lv_display_t *s_lv_disp  = NULL;
static lv_indev_t   *s_lv_touch = NULL;
#else
static lv_display_t *s_lv_disp  = NULL;
#endif

/* Backlight */
static void init_backlight(void) {
#if (DISPLAY_PIN_BCKL >= 0)
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << DISPLAY_PIN_BCKL,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);
    gpio_set_level(DISPLAY_PIN_BCKL, 1);
#endif
}

#if ENABLE_TOUCH_SUPPORT
/* LVGL input driver -> touch read (no rotation fiddling) */
static void lv_read_touch_cb(lv_indev_t *indev, lv_indev_data_t *data) {
    (void)indev;
    data->state = LV_INDEV_STATE_RELEASED;
    if (!s_touch) return;

    esp_lcd_touch_read_data(s_touch);

    uint16_t x = 0, y = 0, strength = 0;
    uint8_t  points = 0;
    bool ok = esp_lcd_touch_get_coordinates(s_touch, &x, &y, &strength, &points, 1);
    if (ok && points > 0) {
        data->state   = LV_INDEV_STATE_PRESSED;
        data->point.x = x;
        data->point.y = y;
        // ESP_LOGI(TAG, "TOUCH x=%u y=%u p=%u", x, y, strength);
    }
}
#endif

esp_err_t display_init(void) {
    ESP_LOGI(TAG, "Init Display + Touch (ST7789 + XPT2046) — partial buffer + internal DMA bounce");

    /* ---- ONE shared SPI bus (VSPI by your config) ----
       Important: keep max_transfer_sz close to trans/bounce size. */
    spi_bus_config_t buscfg = {
        .mosi_io_num     = DISPLAY_PIN_MOSI,
        .miso_io_num     = TOUCH_PIN_MISO,      // shared MISO
        .sclk_io_num     = DISPLAY_PIN_CLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = TRANS_SIZE + 8,      // keep small since we use bounce buffer
        .flags           = SPICOMMON_BUSFLAG_MASTER,
        .intr_flags      = 0,
    };
    esp_err_t err = spi_bus_initialize(DISPLAY_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    /* ---- LCD panel IO (SPI) ---- */
    esp_lcd_panel_io_spi_config_t io_lcd = {
        .cs_gpio_num         = DISPLAY_PIN_CS,
        .dc_gpio_num         = DISPLAY_PIN_DC,
        .spi_mode            = 0,
        .pclk_hz             = LCD_SPI_HZ,
        .trans_queue_depth   = 2,        // keep low to reduce DMA buffer pressure (set 1 if needed)
        .lcd_cmd_bits        = 8,
        .lcd_param_bits      = 8,
        .on_color_trans_done = NULL,
        .user_ctx            = NULL,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)DISPLAY_SPI_HOST, &io_lcd, &s_panel_io));

    /* ---- ST7789 panel (KEEP LITTLE ENDIAN for correct colors) ---- */
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = DISPLAY_PIN_RST,               // -1 if not wired
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .data_endian    = LCD_RGB_DATA_ENDIAN_LITTLE,    // keep as LITTLE for RGB565
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(s_panel_io, &panel_cfg, &s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));

    /* No swap/mirror/gap: defaults only (we’ll rotate in lvgl_port) */
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(s_panel, 0, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));
    init_backlight();

    const int ui_hres = DISPLAY_WIDTH;
    const int ui_vres = DISPLAY_HEIGHT;

    /* ---- Touch (XPT2046) on same bus ----
       NOTE: can be disabled at build-time via ENABLE_TOUCH_SUPPORT to save RAM
       and avoid SPI bus contention while we focus on Bluetooth. */
#if ENABLE_TOUCH_SUPPORT
    esp_lcd_panel_io_spi_config_t io_touch = {
        .cs_gpio_num         = TOUCH_PIN_CS,
        .dc_gpio_num         = -1,                 // not used
        .spi_mode            = 0,
        .pclk_hz             = TOUCH_SPI_HZ,       // ~1 MHz
        .trans_queue_depth   = 1,
        .lcd_cmd_bits        = 8,
        .lcd_param_bits      = 8,
        .on_color_trans_done = NULL,
        .user_ctx            = NULL,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)DISPLAY_SPI_HOST, &io_touch, &s_touch_io));

    esp_lcd_touch_config_t tcfg = {
        .x_max = ui_hres,
        .y_max = ui_vres,
        .rst_gpio_num = -1,
        .int_gpio_num = TOUCH_PIN_IRQ,   // set -1 if not wired
        .levels = { .reset = 0, .interrupt = 0 },  // XPT IRQ active-low
        .flags = {
            .swap_xy  = 0,               // match LVGL rotation used above
            .mirror_x = 1,
            .mirror_y = 1,
        },
    };
    ESP_ERROR_CHECK(esp_lcd_touch_new_spi_xpt2046(s_touch_io, &tcfg, &s_touch));

    ESP_LOGI(TAG, "Touch ready: XPT2046");
#else
    ESP_LOGW(TAG, "Touch support disabled (ENABLE_TOUCH_SUPPORT=0)");
#endif

    /* ---- LVGL core ---- */
    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority    = LVGL_TASK_PRIORITY,
        .task_stack       = LVGL_TASK_STACK_SIZE,
        .task_affinity    = TASK_CORE_DISPLAY,
        .task_max_sleep_ms = 10,  /* Short sleep to ensure responsiveness */
        .timer_period_ms  = LVGL_TICK_PERIOD_MS,
    };

    ESP_LOGI(TAG, "Initializing LVGL port: prio=%d, stack=%d, core=%d, sleep=10ms",
             LVGL_TASK_PRIORITY, LVGL_TASK_STACK_SIZE, TASK_CORE_DISPLAY);
    
    /* Disable LVGL's internal task watchdog to avoid false triggers */
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    /* Partial buffer for LVGL (fits in regular RAM, no PSRAM needed) */
    /* Use smaller buffer (20 lines) = ~12.8KB to save more heap */
    const size_t buf_lines = 20;
    const size_t buf_bytes = (size_t)ui_hres * buf_lines * 2;

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle      = s_panel_io,
        .panel_handle   = s_panel,
        .buffer_size    = buf_bytes,      // Partial buffer in regular RAM
        .double_buffer  = 0,              // single buffer is fine
        .hres           = ui_hres,
        .vres           = ui_vres,
        .monochrome     = false,
    #if LVGL_VERSION_MAJOR >= 9
        .color_format   = LV_COLOR_FORMAT_RGB565,
    #endif
        .rotation = {                     // leave LVGL rotation OFF/neutral for ST7789 mapping you use
            .swap_xy   = 1,
            .mirror_x  = 0,
            .mirror_y  = 1,
        },
        .flags = {
            .buff_dma    = false,         // <-- use internal small bounce buffer, not a big DMA buffer
            .buff_spiram = false,        // <-- use regular RAM, not PSRAM
            .sw_rotate   = false,
            .full_refresh= false,
        },
        .trans_size = TRANS_SIZE,         // <-- internal DMA bounce buffer in DRAM/IRAM-capable heap
    };
    s_lv_disp = lvgl_port_add_disp(&disp_cfg);
    lv_display_set_default(s_lv_disp);

    ESP_LOGI(TAG, "LCD ready: %dx%d, fb=%u bytes (partial buffer, regular RAM), trans=%u bytes (internal)",
             ui_hres, ui_vres, (unsigned)buf_bytes, (unsigned)TRANS_SIZE);

#if ENABLE_TOUCH_SUPPORT
    /* Create touch input device after display is ready */
    s_lv_touch = lv_indev_create();
    lv_indev_set_type(s_lv_touch, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(s_lv_touch, lv_read_touch_cb);
    lv_indev_set_display(s_lv_touch, s_lv_disp);
    ESP_LOGI(TAG, "Touch input device registered with LVGL");
#endif

    /* ---- SquareLine UI ONLY ---- */
    if (lvgl_port_lock(0)) {  /* 0 = wait forever (safe for init) */
        ui_init();
        lvgl_port_unlock();
    }

    ESP_LOGI(TAG, "Display system ready");
    return ESP_OK;
}

/* Signature kept for other modules (no-op) */
void display_update_controller_data(int battery, uint32_t buttons,
                                    int16_t lx, int16_t ly,
                                    int16_t rx, int16_t ry,
                                    uint16_t lt, uint16_t rt) {
    (void)battery; (void)buttons; (void)lx; (void)ly; (void)rx; (void)ry; (void)lt; (void)rt;
}

/* Monitoring task to detect system hangs */
static void system_monitor_task(void* pv) {
    (void)pv;
    ESP_LOGI("MONITOR", "System monitor started (logs every 10s)");
    uint32_t last_update_check = 0;
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // Every 10 seconds
        
        uint32_t heap = esp_get_free_heap_size();
        uint32_t uptime = esp_timer_get_time() / 1000000;
        uint32_t time_since_update = (uint32_t)(esp_timer_get_time() / 1000 - last_successful_update);
        
        /* Get LVGL memory stats and screen info with proper locking */
        lv_mem_monitor_t lv_mem = {0};
        lv_obj_t *current_scr = NULL;
        extern lv_obj_t *ui_Screen2;
        bool on_screen2 = false;
        bool lock_acquired = false;
        
        if (lvgl_port_lock(100)) {  /* 100ms timeout */
            lv_mem_monitor(&lv_mem);
            current_scr = lv_scr_act();
            on_screen2 = (current_scr == ui_Screen2);
            lock_acquired = true;
            lvgl_port_unlock();
        } else {
            ESP_LOGW("MONITOR", "⚠️ LOCK TIMEOUT - can't get LVGL stats!");
        }
        
        /* Only log if something is wrong */
        if (time_since_update > 5000 || heap < 25000) {
            ESP_LOGW("MONITOR", "Uptime: %us, Heap: %u, LastUpdate: %ums ago",
                     (unsigned)uptime, (unsigned)heap, (unsigned)time_since_update);
        }
        
        /* CRITICAL: Detect if timer has stopped (alarm even if we can't check screen due to lock timeout) */
        if ((!lock_acquired || on_screen2) && time_since_update > 10000 && time_since_update != last_update_check) {
            ESP_LOGE("MONITOR", "╔═══════════════════════════════════════════════════════════╗");
            ESP_LOGE("MONITOR", "║  CRITICAL: LVGL TIMER STOPPED - SYSTEM DEADLOCKED        ║");
            ESP_LOGE("MONITOR", "║  Last update: %u seconds ago                              ║", (unsigned)(time_since_update / 1000));
            ESP_LOGE("MONITOR", "╚═══════════════════════════════════════════════════════════╝");
            
            /* Print detailed task state */
            char *task_list = malloc(3072);
            char *runtime_stats = malloc(3072);
            
            if (task_list) {
                ESP_LOGE("MONITOR", "\n=== TASK STATE (Name, State, Prio, Stack, Num) ===");
                vTaskList(task_list);
                printf("%s\n", task_list);
                free(task_list);
            }
            
            if (runtime_stats) {
                ESP_LOGE("MONITOR", "\n=== RUNTIME STATS (Name, RunTime, Percentage) ===");
                vTaskGetRunTimeStats(runtime_stats);
                printf("%s\n", runtime_stats);
                free(runtime_stats);
            }
            
            /* Check label update task state */
            ESP_LOGE("MONITOR", "Using FreeRTOS task (not LVGL timer) - check task list above");
            
            /* Print heap info */
            ESP_LOGE("MONITOR", "Heap: %u bytes free, min ever: %u bytes",
                     (unsigned)esp_get_free_heap_size(),
                     (unsigned)esp_get_minimum_free_heap_size());
            
            /* Check animation count and current screen with proper locking */
            ESP_LOGE("MONITOR", "Active LVGL animations: checking...");
            uint32_t anim_count = 0;
            lv_obj_t *scr = NULL;
            extern lv_obj_t *ui_Screen1;
            extern lv_obj_t *ui_Screen2;
            
            if (lvgl_port_lock(100)) {  /* 100ms timeout */
                anim_count = lv_anim_count_running();
                scr = lv_scr_act();
                lvgl_port_unlock();
            }
            
            ESP_LOGE("MONITOR", "Running animations: %u", (unsigned)anim_count);
            if (scr == ui_Screen1) {
                ESP_LOGE("MONITOR", "Current screen: Screen1 (with DriveWobble animation!)");
            } else if (scr == ui_Screen2) {
                ESP_LOGE("MONITOR", "Current screen: Screen2");
            } else {
                ESP_LOGE("MONITOR", "Current screen: Unknown (%p)", scr);
            }
            
            last_update_check = time_since_update;
        }
    }
}

/* ================= LABEL UPDATE TASK (FREERTOS) ================= */
/* Dedicated FreeRTOS task for label updates - ELIMINATES LVGL TIMER DEADLOCK! */
static void label_update_task(void *pv) {
    (void)pv;
    static uint32_t counter = 0;
    static uint8_t cached_mac[6] = {0};
    static bool mac_cached = false;
    static uint32_t last_log_time = 0;

    ESP_LOGI(TAG, "Label update task started on core %d", xPortGetCoreID());

    while (1) {
        uint32_t now = esp_timer_get_time() / 1000;
        
        /* Check if we're on Screen2 */
        lv_obj_t *active_screen = NULL;
        extern lv_obj_t *ui_Screen2;
        extern lv_obj_t *ui_Info;
        
        /* Acquire LVGL lock for screen check */
        if (lvgl_port_lock(500)) {  /* Increased to 500ms */
            active_screen = lv_scr_act();
            lvgl_port_unlock();
        } else {
            ESP_LOGW(TAG, "Lock timeout (screen check) at cnt=%u", (unsigned)counter);
            vTaskDelay(pdMS_TO_TICKS(LABEL_UPDATE_INTERVAL_MS));
            continue;
        }

        /* If not on Screen2, skip all work */
        if (!active_screen || active_screen != ui_Screen2) {
            vTaskDelay(pdMS_TO_TICKS(LABEL_UPDATE_INTERVAL_MS));
            continue;
        }

        uint32_t current_time = esp_timer_get_time() / 1000;

    /* Get latest data - these are fast reads from shared memory */
    espnow_controller_data_t cd = {0};
    espnow_telemetry_t tel = {0};
    espnow_get_current_controller_data(&cd);
    bool have_tel = espnow_get_last_telemetry(&tel);

    bool controller_connected = is_timestamp_recent(cd.timestamp_ms, 5000);
    bool ande_connected = false;
    uint32_t last_telemetry_received = espnow_get_last_telemetry_timestamp();
    if (have_tel && last_telemetry_received > 0) {
        uint32_t time_since_last_telem = current_time - last_telemetry_received;
        ande_connected = (time_since_last_telem < 5000);
    }

    /* Cache MAC address - only read once at startup */
    if (!mac_cached) {
        if (esp_read_mac(cached_mac, ESP_MAC_WIFI_STA) == ESP_OK) {
            mac_cached = true;
        }
    }

    /* Build display string - use smaller buffer and optimize formatting */
    char display_buffer[512];  /* Reduced from 1024 to save stack */
    int offset = 0;

    /* MAC address (cached) */
    if (mac_cached) {
        offset += snprintf(display_buffer + offset, sizeof(display_buffer) - offset,
            "MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
            cached_mac[0], cached_mac[1], cached_mac[2], 
            cached_mac[3], cached_mac[4], cached_mac[5]);
    }

    /* Controller status - simplified formatting */
    if (controller_connected) {
        offset += snprintf(display_buffer + offset, sizeof(display_buffer) - offset,
            "CTRL: Connected L(%d,%d) R(%d,%d) B:%u%%\n",
            cd.left_stick_x, cd.left_stick_y,
            cd.right_stick_x, cd.right_stick_y,
            (unsigned)cd.controller_battery);
    } else {
        offset += snprintf(display_buffer + offset, sizeof(display_buffer) - offset,
            "CTRL: Not Connected\n");
    }

    /* Car status - simplified formatting */
    if (ande_connected) {
        offset += snprintf(display_buffer + offset, sizeof(display_buffer) - offset,
            "CAR: Connected RPM:%ld B:%.1fV\n",
            (long)tel.rpm, (double)tel.batt_mv/1000.0);
    } else {
        offset += snprintf(display_buffer + offset, sizeof(display_buffer) - offset,
            "CAR: Not Found\n");
    }

    /* Counter */
    offset += snprintf(display_buffer + offset, sizeof(display_buffer) - offset,
        "Cnt:%lu", (unsigned long)counter);

    /* Safety check */
    if (offset >= sizeof(display_buffer) - 1) {
        display_buffer[sizeof(display_buffer) - 1] = '\0';
    }

        /* Update the label with proper LVGL locking */
        if (lvgl_port_lock(500)) {  /* 500ms timeout */
            lv_label_set_text(ui_Info, display_buffer);
            lvgl_port_unlock();
            
            last_successful_update = current_time;
            counter++;
        } else {
            ESP_LOGW(TAG, "Lock timeout (label update) at cnt=%u", (unsigned)counter);
        }
        
        /* Sleep until next update */
        vTaskDelay(pdMS_TO_TICKS(LABEL_UPDATE_INTERVAL_MS));
    }
}

void display_start_label_task(void) {
    /* Initialize health monitoring timestamps */
    last_successful_update = esp_timer_get_time() / 1000;
    
    /* Start system monitor task for debugging */
    BaseType_t ret = xTaskCreatePinnedToCore(
        system_monitor_task,
        "sys_monitor",
        4096,
        NULL,
        1,  /* Low priority */
        NULL,
        1   /* Core 1 */
    );
    if (ret != pdPASS) {
        ESP_LOGW(TAG, "Failed to create system monitor task");
    }

    /* Create dedicated FreeRTOS task for label updates (NO MORE LVGL TIMER!) */
    ESP_LOGI(TAG, "Creating FreeRTOS task for label updates (every %dms)", LABEL_UPDATE_INTERVAL_MS);
    
    ret = xTaskCreatePinnedToCore(
        label_update_task,
        "label_update",
        4096,  /* 4KB stack for string formatting */
        NULL,
        LABEL_UPDATE_TASK_PRIORITY,  /* Low priority - just UI */
        NULL,
        LABEL_UPDATE_TASK_CORE  /* CPU 1 with LVGL */
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "CRITICAL: Failed to create label update task!");
    } else {
        ESP_LOGI(TAG, "Label update task created successfully (FreeRTOS with LVGL locking)");
    }
}

display_health_t display_get_health(void) {
    uint32_t current_time = esp_timer_get_time() / 1000;
    display_health_t health = {
        .last_successful_update_ms = last_successful_update,
        .consecutive_failures = 0,
        .uptime_seconds = current_time / 1000,
        .is_healthy = (s_label_update_timer != NULL) &&
                     (current_time - last_successful_update < 5000)
    };
    return health;
}
