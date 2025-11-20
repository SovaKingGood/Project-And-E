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
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

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

/* ONE TIMER for all UI updates */
static lv_timer_t *s_ui_update_timer = NULL;

/* Track which screen is currently active */
typedef enum {
    ACTIVE_SCREEN_NONE = 0,
    ACTIVE_SCREEN_1,
    ACTIVE_SCREEN_2,
    ACTIVE_SCREEN_3
} active_screen_t;

static volatile active_screen_t s_active_screen = ACTIVE_SCREEN_1;

/* Health monitoring variables */
static uint32_t last_successful_update = 0;

/* -------------------------------------------------------------------------
 * Shared UI / telemetry model
 *
 *  - Written by other tasks (ESP-NOW, controller logic) via display_set_*
 *  - Read only from LVGL task (timer/event callbacks) to update widgets
 *  - Protected by a simple FreeRTOS mutex, never the LVGL mutex
 * ------------------------------------------------------------------------- */
typedef struct {
    espnow_controller_data_t controller;
    bool                     controller_valid;
    uint32_t                 controller_timestamp_ms;

    espnow_telemetry_t       telemetry;
    bool                     telemetry_valid;
    uint32_t                 telemetry_timestamp_ms;
} display_ui_model_t;

static display_ui_model_t   s_ui_model = {0};
static SemaphoreHandle_t    s_ui_model_mutex = NULL;

/* Forward declarations for internal LVGL event handlers (run in LVGL task) */
static void display_info_button_event_cb(lv_event_t *e);
static void display_start_button_event_cb(lv_event_t *e);
static void screen2_event_cb(lv_event_t *e);
static void screen2_button1_event_cb(lv_event_t *e);
static void screen3_event_cb(lv_event_t *e);

/* Forward declarations for screen update functions */
static void update_screen2(void);
static void update_screen3(void);

/* Screen change debouncing - prevents rapid successive screen loads */
static lv_obj_t *s_last_screen_load = NULL;
static uint32_t s_last_screen_load_time = 0;
static const uint32_t SCREEN_CHANGE_DEBOUNCE_MS = 300;  /* Minimum 300ms between screen changes */


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
    /* Use smaller buffer (8 lines) = ~5.1KB to save heap and reduce stack pressure */
    const size_t buf_lines = 8;
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

        /* Attach additional lightweight button event callbacks (LVGL task context) */
        extern lv_obj_t *ui_InfoButton;
        extern lv_obj_t *ui_StartButton;
        extern lv_obj_t *ui_Screen2;

        if (ui_InfoButton) {
            lv_obj_add_event_cb(ui_InfoButton,
                                display_info_button_event_cb,
                                LV_EVENT_CLICKED,
                                NULL);
        }

        if (ui_StartButton) {
            lv_obj_add_event_cb(ui_StartButton,
                                display_start_button_event_cb,
                                LV_EVENT_CLICKED,
                                NULL);
        }

        /* ui_Button2 removed from UI - no longer needed */

        /* Hook into Screen2 to cache/clear UI object pointers for timer safety */
        if (ui_Screen2) {
            lv_obj_add_event_cb(ui_Screen2, screen2_event_cb, LV_EVENT_ALL, NULL);
            /* DON'T cache Screen2 objects at init - only when screen becomes active */
            /* Screen2 timer will be enabled by SCREEN_LOAD_START event */
        }

        /* Hook into Screen2's Button1 to disable timer BEFORE leaving Screen2 */
        extern lv_obj_t *ui_Button1;
        if (ui_Button1) {
            /* Listen to ALL events so we catch PRESSING before CLICKED */
            lv_obj_add_event_cb(ui_Button1, screen2_button1_event_cb, LV_EVENT_ALL, NULL);
        }

        /* Hook into Screen3 to cache/clear controller input UI objects */
        extern lv_obj_t *ui_Screen3;
        
        if (ui_Screen3) {
            lv_obj_add_event_cb(ui_Screen3, screen3_event_cb, LV_EVENT_ALL, NULL);
            /* DON'T cache Screen3 objects at init - only when screen becomes active */
            /* Screen3 timer will be enabled by SCREEN_LOAD_START event */
        }

        lvgl_port_unlock();
    }

    /* Initialize shared UI model mutex (used by other tasks and LVGL timer) */
    s_ui_model_mutex = xSemaphoreCreateMutex();
    if (s_ui_model_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create UI model mutex");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Display system ready");
    return ESP_OK;
}

/* Safe screen load wrapper - prevents rapid successive loads and deadlocks */
/* Note: Not currently used since we can't modify UI code, but kept for future use */
void display_safe_screen_load(lv_obj_t *scr)
{
    if (scr == NULL) {
        ESP_LOGW(TAG, "display_safe_screen_load: NULL screen pointer");
        return;
    }

    /* Check if already active */
    lv_obj_t *active = lv_scr_act();
    if (active == scr) {
        return;  /* Already on this screen */
    }

    /* Debounce: prevent rapid successive screen changes */
    uint32_t now = esp_timer_get_time() / 1000;  /* Convert to ms */
    if (s_last_screen_load == scr && (now - s_last_screen_load_time) < SCREEN_CHANGE_DEBOUNCE_MS) {
        ESP_LOGD(TAG, "Screen change debounced: same screen within %lu ms", (unsigned long)SCREEN_CHANGE_DEBOUNCE_MS);
        return;
    }

    /* All checks passed - safe to load */
    /* Note: lv_screen_load() internally checks if screen is already active */
    s_last_screen_load = scr;
    s_last_screen_load_time = now;
    lv_screen_load(scr);
}

/* Signature kept for other modules (no-op) */
void display_update_controller_data(int battery, uint32_t buttons,
                                    int16_t lx, int16_t ly,
                                    int16_t rx, int16_t ry,
                                    uint16_t lt, uint16_t rt) {
    (void)battery; (void)buttons; (void)lx; (void)ly; (void)rx; (void)ry; (void)lt; (void)rt;
}

/* ----------------------------------------------------------------------------
 * SHARED UI MODEL UPDATE API
 *
 * These functions are called from NON-LVGL tasks (e.g., ESPNOW) to push
 * fresh data into the UI model. No LVGL APIs are used here; they only
 * update RAM under a FreeRTOS mutex. Actual widget updates are performed
 * from the LVGL task (timer callback) to avoid deadlocks.
 * ------------------------------------------------------------------------- */

void display_set_controller_state(const espnow_controller_data_t *state)
{
    if (!state || !s_ui_model_mutex) {
        return;
    }

    /* NON-BLOCKING: Don't wait for mutex - Bluetooth stack can't afford delays during connection */
    if (xSemaphoreTake(s_ui_model_mutex, 0) == pdTRUE) {
        s_ui_model.controller = *state;
        s_ui_model.controller_valid = true;
        s_ui_model.controller_timestamp_ms = esp_timer_get_time() / 1000;
        xSemaphoreGive(s_ui_model_mutex);
    }
    /* If mutex is busy, silently skip this update - next controller packet will try again */
}

void display_set_telemetry_state(const espnow_telemetry_t *tel)
{
    if (!tel || !s_ui_model_mutex) {
        return;
    }

    /* NON-BLOCKING: Don't wait for mutex - avoid blocking ESP-NOW receive callback */
    if (xSemaphoreTake(s_ui_model_mutex, 0) == pdTRUE) {
        s_ui_model.telemetry = *tel;
        s_ui_model.telemetry_valid = true;
        s_ui_model.telemetry_timestamp_ms = esp_timer_get_time() / 1000;
        xSemaphoreGive(s_ui_model_mutex);
    }
    /* If mutex is busy, silently skip this update - next telemetry packet will try again */
}

/* ----------------------------------------------------------------------------
 * ADDITIONAL BUTTON EVENT HOOKS
 *
 * These run in LVGL task context (called by LVGL when buttons are clicked).
 * They MUST remain lightweight and must not block or perform long-running
 * operations. Business logic should be delegated to other tasks via the
 * shared UI model or separate queues.
 * ------------------------------------------------------------------------- */

static void display_info_button_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }

    ESP_LOGI(TAG, "🟢 InfoButton clicked - navigating TO Screen2");
    /* Screen2 will be loaded - screen event handler will enable timer */
}

static void display_start_button_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }

    ESP_LOGI(TAG, "🟢 StartButton clicked - navigating TO Screen3");
    /* Screen3 will be loaded - event handler will enable controller input timer */
}

/* display_button2_event_cb removed - ui_Button2 no longer exists in UI */

/* Screen2's Button1 event handler - clears cache BEFORE leaving Screen2 */
static void screen2_button1_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    
    if (code == LV_EVENT_CLICKED) {
        ESP_LOGI(TAG, "Button1 clicked - returning to Screen1");
        display_clear_screen2_cache();
    }
}

/* Screen2 lifecycle event handler - manages timer enable/disable */
static void screen2_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    
    extern lv_obj_t *ui_Screen2;
    extern lv_obj_t *ui_Info;

    if (code == LV_EVENT_SCREEN_LOAD_START) {
        ESP_LOGI(TAG, "Screen2 LOAD_START");
        if (ui_Screen2 && ui_Info) {
            display_cache_screen2_objects(ui_Screen2, ui_Info);
        }
    } else if (code == LV_EVENT_SCREEN_LOADED) {
        ESP_LOGI(TAG, "Screen2 LOADED");
        if (ui_Screen2 && ui_Info && s_active_screen != ACTIVE_SCREEN_2) {
            display_cache_screen2_objects(ui_Screen2, ui_Info);
        }
    } else if (code == LV_EVENT_SCREEN_UNLOAD_START) {
        ESP_LOGI(TAG, "Screen2 UNLOAD_START");
        display_clear_screen2_cache();
    } else if (code == LV_EVENT_SCREEN_UNLOADED) {
        ESP_LOGI(TAG, "Screen2 UNLOADED");
        display_clear_screen2_cache();
    } else if (code == LV_EVENT_DELETE) {
        ESP_LOGI(TAG, "Screen2 DELETE");
        display_clear_screen2_cache();
    }
}

/* Screen3 lifecycle event handler - manages controller input timer enable/disable */
static void screen3_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    
    extern lv_obj_t *ui_Screen3;
    extern lv_obj_t *ui_ThrottleBar;
    extern lv_obj_t *ui_BrakeBar;
    extern lv_obj_t *ui_Slider1;

    if (code == LV_EVENT_SCREEN_LOAD_START) {
        ESP_LOGI(TAG, "Screen3 LOAD_START");
        if (ui_Screen3 && ui_ThrottleBar && ui_BrakeBar && ui_Slider1) {
            display_cache_screen3_objects(ui_Screen3, ui_ThrottleBar, ui_BrakeBar, ui_Slider1);
        }
    } else if (code == LV_EVENT_SCREEN_UNLOAD_START) {
        ESP_LOGI(TAG, "Screen3 UNLOAD_START");
        display_clear_screen3_cache();
    } else if (code == LV_EVENT_DELETE) {
        ESP_LOGI(TAG, "Screen3 DELETE");
        display_clear_screen3_cache();
    }
}

/* Removed unused screen3_button2_event_cb - no longer needed with unified timer */

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

        /* Only log if something is wrong - simplified to avoid stack overflow */
        if (time_since_update > 5000 || heap < 25000) {
            ESP_LOGW("MONITOR", "Uptime: %us, Heap: %u, LastUpdate: %ums ago",
                     (unsigned)uptime, (unsigned)heap, (unsigned)time_since_update);
        }
        
        /* CRITICAL: Detect if LVGL timer has stopped - minimal logging to save stack */
        if (time_since_update > 10000 && time_since_update != last_update_check) {
            ESP_LOGE("MONITOR", "LVGL TIMER STOPPED! Last update: %u seconds ago",
                     (unsigned)(time_since_update / 1000));
            last_update_check = time_since_update;
        }
    }
}

/* ================= UNIFIED UI UPDATE TIMER (LVGL CONTEXT) ================= */
/* ONE timer for all screens - runs inside LVGL task via lv_timer_handler() */
/* NO lvgl_port_lock needed - already in LVGL context */

/* Cached UI object pointers */
static lv_obj_t *s_cached_info_label = NULL;      /* Screen2 */
static lv_obj_t *s_cached_throttle_bar = NULL;    /* Screen3 */
static lv_obj_t *s_cached_brake_bar = NULL;       /* Screen3 */
static lv_obj_t *s_cached_slider1 = NULL;         /* Screen3 */

static void ui_update_timer_cb(lv_timer_t *timer)
{
    (void)timer;

    /* Always update heartbeat (proves LVGL is alive) */
    last_successful_update = esp_timer_get_time() / 1000;

    /* Route to appropriate screen handler */
    if (s_active_screen == ACTIVE_SCREEN_2) {
        update_screen2();
    } else if (s_active_screen == ACTIVE_SCREEN_3) {
        update_screen3();
    }
    /* Screen1 needs no updates */
}

/* ================= SCREEN 2 UPDATE (INFO/STATUS DISPLAY) ================= */
static void update_screen2(void)
{
    static uint32_t counter = 0;
    static uint8_t  cached_mac[6] = {0};
    static bool     mac_cached = false;
    static char display_buffer[256];  /* Static buffer */

    /* Validate cached pointer */
    if (!s_cached_info_label) {
        return;
    }

    /* Snapshot UI model data */
    bool controller_valid = false;
    bool telemetry_valid  = false;
    uint32_t controller_ts = 0;
    uint32_t telemetry_ts  = 0;
    int16_t lx = 0, ly = 0, rx = 0, ry = 0;
    uint8_t batt_pct = 0;
    int32_t rpm = 0;
    uint16_t batt_mv = 0;

    if (!s_ui_model_mutex) {
        return;
    }

    if (xSemaphoreTake(s_ui_model_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (s_ui_model.controller_valid) {
            lx = s_ui_model.controller.left_stick_x;
            ly = s_ui_model.controller.left_stick_y;
            rx = s_ui_model.controller.right_stick_x;
            ry = s_ui_model.controller.right_stick_y;
            batt_pct = s_ui_model.controller.controller_battery;
            controller_valid = true;
            controller_ts = s_ui_model.controller_timestamp_ms;
        }
        if (s_ui_model.telemetry_valid) {
            rpm = s_ui_model.telemetry.rpm;
            batt_mv = s_ui_model.telemetry.batt_mv;
            telemetry_valid = true;
            telemetry_ts = s_ui_model.telemetry_timestamp_ms;
        }
        xSemaphoreGive(s_ui_model_mutex);
    } else {
        return;  /* Skip if mutex busy */
    }

    bool controller_connected = controller_valid && is_timestamp_recent(controller_ts, 5000);
    bool ande_connected = telemetry_valid && is_timestamp_recent(telemetry_ts, 5000);

    /* Cache MAC address - only read once at startup */
    if (!mac_cached) {
        if (esp_read_mac(cached_mac, ESP_MAC_WIFI_STA) == ESP_OK) {
            mac_cached = true;
        }
    }

    /* Build display string using static buffer - minimal stack usage */
    int len = 0;

    /* MAC address (cached) */
    if (mac_cached) {
        len += snprintf(display_buffer + len, sizeof(display_buffer) - len,
                        "MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
                        cached_mac[0], cached_mac[1], cached_mac[2],
                        cached_mac[3], cached_mac[4], cached_mac[5]);
    }

    /* Controller status - use simpler formatting to reduce stack */
    if (controller_connected && len < (int)sizeof(display_buffer) - 50) {
        len += snprintf(display_buffer + len, sizeof(display_buffer) - len,
                        "CTRL: OK L(%d,%d) R(%d,%d) B:%u%%\n",
                        (int)lx, (int)ly, (int)rx, (int)ry, (unsigned)batt_pct);
    } else if (len < (int)sizeof(display_buffer) - 30) {
        len += snprintf(display_buffer + len, sizeof(display_buffer) - len,
                        "CTRL: Not Connected\n");
    }

    /* Car status - use integer math to avoid floating point on stack */
    if (ande_connected && len < (int)sizeof(display_buffer) - 40) {
        int batt_v_int = batt_mv / 1000;
        int batt_v_frac = (batt_mv % 1000) / 100;
        len += snprintf(display_buffer + len, sizeof(display_buffer) - len,
                        "CAR: OK RPM:%ld B:%d.%dV\n",
                        (long)rpm, batt_v_int, batt_v_frac);
    } else if (len < (int)sizeof(display_buffer) - 25) {
        len += snprintf(display_buffer + len, sizeof(display_buffer) - len,
                        "CAR: Not Found\n");
    }

    /* Counter */
    if (len < (int)sizeof(display_buffer) - 15) {
        snprintf(display_buffer + len, sizeof(display_buffer) - len,
                 "Cnt:%lu", (unsigned long)counter);
    }

    /* Update label */
    lv_label_set_text(s_cached_info_label, display_buffer);
    counter++;
}

/* ================= SCREEN 3 UPDATE (CONTROLLER INPUT DISPLAY) ================= */
static void update_screen3(void)
{
    /* Validate cached pointers */
    if (!s_cached_throttle_bar || !s_cached_brake_bar || !s_cached_slider1) {
        return;
    }

    /* Read controller data from UI model */
    if (!s_ui_model_mutex) {
        return;
    }

    if (xSemaphoreTake(s_ui_model_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return;  /* Skip if mutex busy */
    }

    bool controller_valid = s_ui_model.controller_valid;
    int16_t left_stick_x = s_ui_model.controller.left_stick_x;
    uint8_t right_trigger = s_ui_model.controller.right_trigger;  // R2 = throttle
    uint8_t left_trigger = s_ui_model.controller.left_trigger;    // L2 = brake

    xSemaphoreGive(s_ui_model_mutex);

    /* If no controller, set to neutral */
    if (!controller_valid) {
        lv_bar_set_value(s_cached_throttle_bar, 0, LV_ANIM_OFF);
        lv_bar_set_value(s_cached_brake_bar, 0, LV_ANIM_OFF);
        lv_slider_set_value(s_cached_slider1, 50, LV_ANIM_OFF);  // Center
        return;
    }

    /* Map controller values to UI (0-100 range) */
    int32_t throttle_percent = (right_trigger * 100) / 255;
    int32_t brake_percent = (left_trigger * 100) / 255;
    int32_t steering_percent = ((left_stick_x + 512) * 100) / 1024;

    /* Clamp to valid range */
    if (throttle_percent < 0) throttle_percent = 0;
    if (throttle_percent > 100) throttle_percent = 100;
    if (brake_percent < 0) brake_percent = 0;
    if (brake_percent > 100) brake_percent = 100;
    if (steering_percent < 0) steering_percent = 0;
    if (steering_percent > 100) steering_percent = 100;

    /* Update UI widgets */
    lv_bar_set_value(s_cached_throttle_bar, throttle_percent, LV_ANIM_OFF);
    lv_bar_set_value(s_cached_brake_bar, brake_percent, LV_ANIM_OFF);
    lv_slider_set_value(s_cached_slider1, steering_percent, LV_ANIM_OFF);
}

/* Screen cache management - simple and clean */
void display_cache_screen2_objects(lv_obj_t *screen2, lv_obj_t *info_label)
{
    (void)screen2;  /* Not needed anymore */
    s_cached_info_label = info_label;
    s_active_screen = ACTIVE_SCREEN_2;
    ESP_LOGI(TAG, "Screen2 active");
}

void display_clear_screen2_cache(void)
{
    s_active_screen = ACTIVE_SCREEN_1;
    s_cached_info_label = NULL;
    ESP_LOGI(TAG, "Screen2 inactive");
}

void display_cache_screen3_objects(lv_obj_t *screen3, lv_obj_t *throttle_bar, lv_obj_t *brake_bar, lv_obj_t *slider1)
{
    (void)screen3;  /* Not needed anymore */
    s_cached_throttle_bar = throttle_bar;
    s_cached_brake_bar = brake_bar;
    s_cached_slider1 = slider1;
    s_active_screen = ACTIVE_SCREEN_3;
    ESP_LOGI(TAG, "Screen3 active");
}

void display_clear_screen3_cache(void)
{
    s_active_screen = ACTIVE_SCREEN_1;
    s_cached_throttle_bar = NULL;
    s_cached_brake_bar = NULL;
    s_cached_slider1 = NULL;
    ESP_LOGI(TAG, "Screen3 inactive");
}

void display_start_label_task(void) {
    /* Initialize health monitoring timestamps */
    last_successful_update = esp_timer_get_time() / 1000;

    /* Start system monitor task for debugging (will be simplified later) */
    BaseType_t ret = xTaskCreatePinnedToCore(
        system_monitor_task,
        "sys_monitor",
        3072,  /* 3KB - increased to prevent stack overflow */
        NULL,
        LABEL_UPDATE_TASK_PRIORITY,  /* Same priority as LVGL - prevents priority inversion */
        NULL,
        1   /* Core 1 */
    );
    if (ret != pdPASS) {
        ESP_LOGW(TAG, "Failed to create system monitor task");
    }

    /* Create unified UI update timer - runs in LVGL task, no locks needed */
    ESP_LOGI(TAG, "Creating unified UI update timer");

    if (lvgl_port_lock(0)) {  /* Safe during init */
        /* ONE timer for all screens - 50ms for responsive controller input */
        s_ui_update_timer = lv_timer_create(ui_update_timer_cb, 50, NULL);
        lvgl_port_unlock();
    } else {
        ESP_LOGE(TAG, "Failed to acquire LVGL lock to create UI update timer");
    }

    if (s_ui_update_timer == NULL) {
        ESP_LOGE(TAG, "CRITICAL: Failed to create UI update timer!");
    } else {
        ESP_LOGI(TAG, "UI update timer created successfully (50ms, screen-aware)");
    }
}

display_health_t display_get_health(void) {
    uint32_t current_time = esp_timer_get_time() / 1000;
    display_health_t health = {
        .last_successful_update_ms = last_successful_update,
        .consecutive_failures = 0,
        .uptime_seconds = current_time / 1000,
        .is_healthy = (s_ui_update_timer != NULL) &&
                     (current_time - last_successful_update < 5000)
    };
    return health;
}

/* ================= CONNECTION STATUS INDICATORS ================= */
/* Update controller connection status icon on Screen1 */
void display_update_controller_status(bool connected)
{
    extern lv_obj_t *ui_ControllerStatus;
    
    if (!ui_ControllerStatus) {
        return;
    }
    
    if (!lvgl_port_lock(10)) {
        ESP_LOGW(TAG, "Failed to lock display for controller status update");
        return;
    }
    
    if (connected) {
        /* Controller connected - set blend mode to NORMAL (icon visible/bright) */
        lv_obj_set_style_blend_mode(ui_ControllerStatus, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_DEFAULT);
    } else {
        /* Controller disconnected - set blend mode to SUBTRACTIVE (icon dimmed) */
        lv_obj_set_style_blend_mode(ui_ControllerStatus, LV_BLEND_MODE_SUBTRACTIVE, LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    
    lvgl_port_unlock();
}

/* Update ESP-NOW connection status icon on Screen1 */
void display_update_espnow_status(bool connected)
{
    extern lv_obj_t *ui_WiFiStatus;
    
    if (!ui_WiFiStatus) {
        return;
    }
    
    if (!lvgl_port_lock(10)) {
        ESP_LOGW(TAG, "Failed to lock display for ESP-NOW status update");
        return;
    }
    
    if (connected) {
        /* ESP-NOW connected - set blend mode to NORMAL (icon visible/bright) */
        lv_obj_set_style_blend_mode(ui_WiFiStatus, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_DEFAULT);
    } else {
        /* ESP-NOW disconnected - set blend mode to SUBTRACTIVE (icon dimmed) */
        lv_obj_set_style_blend_mode(ui_WiFiStatus, LV_BLEND_MODE_SUBTRACTIVE, LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    
    lvgl_port_unlock();
}
