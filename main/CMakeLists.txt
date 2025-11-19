# Explicitly list all SquareLine UI sources
set(UI_SOURCES
    "ui/ui.c"
    "ui/ui_helpers.c"
    "ui/screens/ui_Screen1.c"
    "ui/screens/ui_Screen2.c"
    "ui/screens/ui_Screen3.c"
    "ui/components/ui_comp.c"
    "ui/components/ui_comp_button1.c"
    "ui/components/ui_comp_hook.c"
    "ui/images/ui_img_945321139.c"
    "ui/images/ui_img_686816875.c"
    "ui/images/ui_img_1471171794.c"
    "ui/images/ui_img_836209702.c"
    "ui/images/ui_img_1577682898.c"
    "ui/fonts/ui_font_Chokokutai.c"
)

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
