/**
 * lv_conf.h for ESP32 without PSRAM - optimized for regular RAM
 */

 #if 1 /* Enable LVGL config */

 #ifndef LV_CONF_H
#define LV_CONF_H
 
/*====================
   COLOR SETTINGS
 *====================*/
 #define LV_COLOR_DEPTH     16      /* RGB565 for ST7789 */
 #define LV_COLOR_16_SWAP   0
 
/*=========================
   MEMORY SETTINGS
 *=========================*/
 #define LV_USE_STDLIB_MALLOC    LV_STDLIB_BUILTIN
 #define LV_MEM_CUSTOM           0
 #define LV_MEM_SIZE             (32 * 1024)   /* 32KB pool - aggressive memory saving */
 #define LV_MEM_ADR              0
 
/*====================
   HAL SETTINGS
 *====================*/
 #define LV_DEF_REFR_PERIOD  50   /* ~20 FPS target - slower refresh to reduce memory pressure */
 #define LV_DPI_DEF          130
 
/*====================
   DRAW BUFFERS
 *====================*/
 #define LV_DRAW_BUF_STRIDE_ALIGN   4
 #define LV_DRAW_BUF_ALIGN          4
 
/* Reduced buffer sizes for regular RAM (no PSRAM):
 * Partial buffer approach - LVGL will render in chunks
 * Very small sizes to avoid heap fragmentation issues
 */
 #define LV_DRAW_LAYER_SIMPLE_BUF_SIZE   (2 * 1024)   /* 2 KB - minimal */
 #define LV_DRAW_LAYER_MAX_MEMORY        (4 * 1024)   /* 4 KB max - extreme memory saving */
 
 /*====================
    LOGGING (off for perf)
  *====================*/
 #define LV_USE_LOG              0
 
 /*====================
    ASSERTS
  *====================*/
 #define LV_USE_ASSERT_NULL      1
 #define LV_USE_ASSERT_MALLOC    1
 
 /*====================
    FONTS
  *====================*/
#define LV_FONT_MONTSERRAT_14 1
 #define LV_FONT_DEFAULT         &lv_font_montserrat_14
 #define LV_USE_FONT_COMPRESSED  0
 
 /*====================
    WIDGETS (minimal set)
  *====================*/
 #define LV_USE_LABEL        1
 #define LV_USE_BUTTON       1
 #define LV_USE_SLIDER       1
 #define LV_USE_BAR          1
 #define LV_USE_TABVIEW      1
 #define LV_USE_ARC          1
 
 /*====================
    THEMES
  *====================*/
 #define LV_USE_THEME_DEFAULT    1
 #define LV_THEME_DEFAULT_DARK   0
 
 /*====================
    DEMOS (disable)
  *====================*/
 #define LV_BUILD_DEMOS          0
 #define LV_BUILD_EXAMPLES       0
 
 #endif /* LV_CONF_H */
 
 #endif /* End of "Content enable" */
 
