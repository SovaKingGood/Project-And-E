SET(SOURCES components/ui_comp_button1.c
    components/ui_comp.c
    screens/ui_Screen1.c
    screens/ui_Screen2.c
    screens/ui_Screen3.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    images/ui_img_592052946.c
    images/ui_img_686816875.c
    images/ui_img_1471171794.c
    images/ui_img_836209702.c
    images/ui_img_1768070411.c
    images/ui_img_493105991.c
    images/ui_img_1577682898.c
    images/ui_img_945321139.c
    fonts/ui_font_Chokokutai.c)

add_library(ui ${SOURCES})
