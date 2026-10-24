file(GLOB BOARD_SRCS "boards/*.c")

set(req esp_lcd esp_driver_i2c esp_driver_gpio esp_driver_ledc
    esp_io_expander esp_io_expander_ch422
    esp_lcd_touch esp_lcd_touch_gt911 esp_lcd_panel_io_additions
    esp_lvgl_adapter esp_common)

idf_component_register(SRCS "board_hal.c" "board_hal_lvgl.c" ${BOARD_SRCS}
                    INCLUDE_DIRS "include"
                    REQUIRES ${req})
