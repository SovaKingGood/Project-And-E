# Grab all SquareLine UI sources (includes ui/assets/*.c now)
file(GLOB_RECURSE UI_SOURCES "ui/*.c")

set(srcs
    "main.c"
    "my_platform.c"
    "display.c"
    "espnow_task.c"
)

set(requires
    bluepad32
    btstack
    driver
    esp_lcd
    esp_wifi
    esp_lvgl_port
    lvgl
    esp_lcd_touch
    esp_lcd_touch_xpt2046
    esp_timer
    freertos
    heap
)

idf_component_register(
    SRCS        ${srcs} ${UI_SOURCES}
    INCLUDE_DIRS "." "ui"
    REQUIRES    ${requires}
)
