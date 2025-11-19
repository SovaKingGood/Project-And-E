/**
 * @file ui_config.h
 * @brief LVGL UI Configuration - Colors, Fonts, Sizes, and Constants
 * 
 * This file contains all the visual configuration for your LVGL interface.
 * Edit these values to customize the look and feel of your UI.
 */

#ifndef UI_CONFIG_H
#define UI_CONFIG_H

#include "lvgl.h"

// =============================================================================
// SCREEN DIMENSIONS
// =============================================================================
#define UI_SCREEN_WIDTH     320
#define UI_SCREEN_HEIGHT    240

// =============================================================================
// COLORS - Easy to customize your theme here!
// =============================================================================
#define UI_COLOR_BLACK      lv_color_hex(0x000000)
#define UI_COLOR_WHITE      lv_color_hex(0xFFFFFF)
#define UI_COLOR_DARK_GRAY  lv_color_hex(0x1a1a1a)
#define UI_COLOR_LIGHT_GRAY lv_color_hex(0x808080)

// Theme colors
#define UI_COLOR_PRIMARY    lv_color_hex(0x00ff41)  // Bright green
#define UI_COLOR_SECONDARY  lv_color_hex(0x0080ff)  // Blue
#define UI_COLOR_DANGER     lv_color_hex(0xff4444)  // Red
#define UI_COLOR_WARNING    lv_color_hex(0xffaa00)  // Orange
#define UI_COLOR_SUCCESS    lv_color_hex(0x00ff41)  // Green

// Drive mode colors
#define UI_COLOR_PARK       lv_color_hex(0x808080)  // Gray
#define UI_COLOR_SPORT      lv_color_hex(0xff4444)  // Red
#define UI_COLOR_DRIVE      lv_color_hex(0x00ff41)  // Green
#define UI_COLOR_ECO        lv_color_hex(0x0080ff)  // Blue

// =============================================================================
// FONTS - Add your custom fonts here
// =============================================================================
#define UI_FONT_SMALL       &lv_font_montserrat_12
#define UI_FONT_MEDIUM      &lv_font_montserrat_16
#define UI_FONT_LARGE       &lv_font_montserrat_16
// Note: Add lv_font_montserrat_18, lv_font_montserrat_10 if you want them

// =============================================================================
// SIZES AND SPACING
// =============================================================================
#define UI_HEADER_HEIGHT    50
#define UI_BUTTON_HEIGHT    40
#define UI_BUTTON_WIDTH     80
#define UI_BIG_BUTTON_HEIGHT 60
#define UI_BIG_BUTTON_WIDTH  120
#define UI_PANEL_PADDING    10
#define UI_ELEMENT_SPACING  10

// =============================================================================
// ANIMATION SETTINGS
// =============================================================================
#define UI_ANIM_TIME_FAST   150   // milliseconds
#define UI_ANIM_TIME_NORMAL 300   // milliseconds
#define UI_ANIM_TIME_SLOW   500   // milliseconds

// =============================================================================
// UPDATE INTERVALS
// =============================================================================
#define UI_UPDATE_INTERVAL_FAST    100   // 100ms - for animations
#define UI_UPDATE_INTERVAL_NORMAL  1000  // 1 second - for data updates
#define UI_UPDATE_INTERVAL_SLOW    5000  // 5 seconds - for system stats

#endif // UI_CONFIG_H
