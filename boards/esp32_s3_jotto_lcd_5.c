/*
 * SPDX-FileCopyrightText: 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Board: Jotto ESP32-S3 LCD 5 (800x480 RGB + GT911 + GPIO backlight).
 *
 * Wiring:
 *  - RGB panel: VSYNC=16, HSYNC=17, DE=15, PCLK=18,
 *    D0..D15 = 46,3,20,19,8,14,13,12,11,10,9,0,45,48,47,21 (B0..B4,
 *    G0..G5, R0..R4). DISP pin not connected.
 *  - I2C0 SDA=41, SCL=42, shared by the GT911 touch controller.
 *  - Touch: reset=38, interrupt/address strap=7. The reset sequence below
 *    keeps INT low so the GT911 comes up at address 0x5D; the driver then
 *    polls the controller (INT/RST are NC in the driver config).
 *  - The secondary sensor I2C bus (I2C1 SDA=5, SCL=6) is not initialized by
 *    this display/touch HAL.
 *  - Backlight: GPIO4, active high, PWM dimming through LEDC.
 *
 * The panel timing is 16 MHz with 800x480 visible pixels. Framebuffers live
 * in PSRAM, so PSRAM must be enabled in the generated sdkconfig.
 */

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_io_i2c.h"
#include "esp_lcd_touch_gt911.h"
#include "esp_lv_adapter.h"

#include "board_hal.h"
#include "board_hal_priv.h"

/* ================================ wiring ================================= */

#define BOARD_I2C_PORT          (I2C_NUM_0)
#define BOARD_I2C_SDA_GPIO      (41)
#define BOARD_I2C_SCL_GPIO      (42)
#define BOARD_I2C_CLK_HZ        (100000)

#define BOARD_LCD_H_RES         (800)
#define BOARD_LCD_V_RES         (480)
#define BOARD_LCD_ROTATION      (ESP_LV_ADAPTER_ROTATE_0)

#define BOARD_LCD_PCLK_HZ       (16 * 1000 * 1000)
#define BOARD_LCD_HSYNC_PW      (40)
#define BOARD_LCD_HSYNC_BP      (40)
#define BOARD_LCD_HSYNC_FP      (48)
#define BOARD_LCD_VSYNC_PW      (23)
#define BOARD_LCD_VSYNC_BP      (32)
#define BOARD_LCD_VSYNC_FP      (13)

#define BOARD_LCD_VSYNC_GPIO    (GPIO_NUM_16)
#define BOARD_LCD_HSYNC_GPIO    (GPIO_NUM_17)
#define BOARD_LCD_DE_GPIO       (GPIO_NUM_15)
#define BOARD_LCD_PCLK_GPIO     (GPIO_NUM_18)
#define BOARD_LCD_DATA_GPIOS    { 46, 3, 20, 19, 8, 14, 13, 12, 11, 10, 9, 0, 45, 48, 47, 21 }

#define BOARD_BL_GPIO           (4)
#define BOARD_BL_LEDC_TIMER     (LEDC_TIMER_0)
#define BOARD_BL_LEDC_CHANNEL   (LEDC_CHANNEL_0)
#define BOARD_BL_PWM_HZ         (5000)

#define BOARD_TOUCH_RST_GPIO    (GPIO_NUM_38)
#define BOARD_TOUCH_INT_GPIO    (GPIO_NUM_7)
#define BOARD_TOUCH_I2C_ADDR    (ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS)

/* ============================== end wiring =============================== */

static const char *TAG = "board_jotto5";

typedef struct {
    board_hal_backlight_t backlight;
    bool backlight_ready;
    bool disp_on;
} board_jotto5_ctx_t;

static board_jotto5_ctx_t s_ctx;

static esp_lcd_touch_io_gt911_config_t s_gt911_cfg = {
    .dev_addr = BOARD_TOUCH_I2C_ADDR,
};

/* ---- forward declarations (the ops table) ---- */
static esp_err_t lcd5_init(board_hal_t *hal);
static esp_err_t lcd5_deinit(board_hal_t *hal);
static esp_err_t lcd5_display_on(board_hal_t *hal);
static esp_err_t lcd5_display_off(board_hal_t *hal);
static esp_err_t lcd5_set_brightness(board_hal_t *hal, uint8_t percent);
static esp_err_t lcd5_get_brightness(board_hal_t *hal, uint8_t *percent_out);
static esp_err_t lcd5_touch_read(board_hal_t *hal, uint16_t *x, uint16_t *y, uint8_t *count);

static const board_hal_ops_t s_ops = {
    .init = lcd5_init,
    .deinit = lcd5_deinit,
    .display_on = lcd5_display_on,
    .display_off = lcd5_display_off,
    .set_brightness = lcd5_set_brightness,
    .get_brightness = lcd5_get_brightness,
    .touch_read = lcd5_touch_read,
    .audio_init = NULL,     /* reserved */
    .audio_deinit = NULL,   /* reserved */
    .camera_init = NULL,    /* reserved */
    .camera_deinit = NULL,  /* reserved */
};

board_hal_t board_esp32_s3_jotto_lcd_5 = {
    .name = "esp32_s3_jotto_lcd_5",
    .caps = BOARD_HAL_CAP_DISPLAY | BOARD_HAL_CAP_BRIGHTNESS | BOARD_HAL_CAP_TOUCH,
    .ops = &s_ops,
    .hor_res = BOARD_LCD_H_RES,
    .ver_res = BOARD_LCD_V_RES,
    .rotation = BOARD_LCD_ROTATION,
    .i2c_bus = NULL,
    .io_expander = NULL,
    .panel_io = NULL,   /* RGB panel: no panel IO */
    .panel = NULL,
    .touch_io = NULL,
    .touch = NULL,
    .lv_display = NULL,
    .lv_touch_indev = NULL,
    .ctx = &s_ctx,
    .reserved_audio = NULL,
    .reserved_camera = NULL,
};

static esp_err_t lcd5_panel_create(board_hal_t *hal)
{
    /* Keep the framebuffer count in sync with the rotation used by LVGL. */
    const uint8_t num_fbs =
        esp_lv_adapter_get_required_frame_buffer_count(ESP_LV_ADAPTER_TEAR_AVOID_MODE_DEFAULT_RGB,
                                                       BOARD_LCD_ROTATION);

    const int data_gpios[16] = BOARD_LCD_DATA_GPIOS;
    esp_lcd_rgb_panel_config_t panel_cfg = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .timings = {
            .pclk_hz = BOARD_LCD_PCLK_HZ,
            .h_res = BOARD_LCD_H_RES,
            .v_res = BOARD_LCD_V_RES,
            .hsync_pulse_width = BOARD_LCD_HSYNC_PW,
            .hsync_back_porch = BOARD_LCD_HSYNC_BP,
            .hsync_front_porch = BOARD_LCD_HSYNC_FP,
            .vsync_pulse_width = BOARD_LCD_VSYNC_PW,
            .vsync_back_porch = BOARD_LCD_VSYNC_BP,
            .vsync_front_porch = BOARD_LCD_VSYNC_FP,
            .flags = {
                .pclk_active_neg = true,
            },
        },
        .data_width = 16,
        .in_color_format = LCD_COLOR_FMT_RGB565,
        .out_color_format = LCD_COLOR_FMT_RGB565,
        .num_fbs = num_fbs,
        .bounce_buffer_size_px = BOARD_LCD_H_RES * 10,
        .dma_burst_size = 64,
        .hsync_gpio_num = BOARD_LCD_HSYNC_GPIO,
        .vsync_gpio_num = BOARD_LCD_VSYNC_GPIO,
        .de_gpio_num = BOARD_LCD_DE_GPIO,
        .pclk_gpio_num = BOARD_LCD_PCLK_GPIO,
        .disp_gpio_num = GPIO_NUM_NC,
        .data_gpio_nums = {
            data_gpios[0], data_gpios[1], data_gpios[2], data_gpios[3],
            data_gpios[4], data_gpios[5], data_gpios[6], data_gpios[7],
            data_gpios[8], data_gpios[9], data_gpios[10], data_gpios[11],
            data_gpios[12], data_gpios[13], data_gpios[14], data_gpios[15],
        },
        .flags = {
            .fb_in_psram = true,
        },
    };

    ESP_RETURN_ON_ERROR(esp_lcd_new_rgb_panel(&panel_cfg, &hal->panel), TAG, "New RGB panel failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(hal->panel), TAG, "Panel init failed");
    return ESP_OK;
}

static esp_err_t lcd5_touch_hw_reset(void)
{
    const gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOARD_TOUCH_RST_GPIO) | (1ULL << BOARD_TOUCH_INT_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "Touch GPIO config failed");

    ESP_RETURN_ON_ERROR(gpio_set_level(BOARD_TOUCH_RST_GPIO, 0), TAG, "Touch reset assert failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(BOARD_TOUCH_INT_GPIO, 0), TAG, "Touch address strap failed");
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_RETURN_ON_ERROR(gpio_set_level(BOARD_TOUCH_RST_GPIO, 1), TAG, "Touch reset release failed");
    vTaskDelay(pdMS_TO_TICKS(50));
    return ESP_OK;
}

static esp_err_t lcd5_touch_create(board_hal_t *hal)
{
    ESP_RETURN_ON_ERROR(lcd5_touch_hw_reset(), TAG, "Touch HW reset failed");

    esp_lcd_panel_io_i2c_config_t tp_io_cfg = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    tp_io_cfg.scl_speed_hz = 400000;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(hal->i2c_bus, &tp_io_cfg, &hal->touch_io),
                        TAG, "Touch panel IO create failed");

    /* The display is not rotated, so the touch frame matches the panel frame. */
    bool swap_xy = false, mirror_x = false, mirror_y = false;
    switch (BOARD_LCD_ROTATION) {
    case ESP_LV_ADAPTER_ROTATE_90:
        mirror_x = true; swap_xy = true;
        break;
    case ESP_LV_ADAPTER_ROTATE_180:
        mirror_x = true; mirror_y = true;
        break;
    case ESP_LV_ADAPTER_ROTATE_270:
        mirror_y = true; swap_xy = true;
        break;
    case ESP_LV_ADAPTER_ROTATE_0:
    default:
        break;
    }

    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = BOARD_LCD_H_RES,
        .y_max = BOARD_LCD_V_RES,
        .rst_gpio_num = GPIO_NUM_NC, /* reset done above through GPIO38 */
        .int_gpio_num = GPIO_NUM_NC, /* polling mode; GPIO7 stays low for 0x5D */
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = swap_xy,
            .mirror_x = mirror_x,
            .mirror_y = mirror_y,
        },
        .driver_data = &s_gt911_cfg,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_touch_new_i2c_gt911(hal->touch_io, &tp_cfg, &hal->touch),
                        TAG, "GT911 create failed");
    return ESP_OK;
}

static esp_err_t lcd5_init(board_hal_t *hal)
{
    ESP_RETURN_ON_FALSE(hal, ESP_ERR_INVALID_ARG, TAG, "Invalid hal");
    esp_err_t ret = ESP_OK;
    board_jotto5_ctx_t *ctx = (board_jotto5_ctx_t *)hal->ctx;
    ctx->backlight_ready = false;
    ctx->disp_on = false;

    /* 1. Backlight PWM, initially off. */
    const board_hal_backlight_cfg_t bl_cfg = {
        .gpio = BOARD_BL_GPIO,
        .ledc_timer = BOARD_BL_LEDC_TIMER,
        .ledc_channel = BOARD_BL_LEDC_CHANNEL,
        .freq_hz = BOARD_BL_PWM_HZ,
        .active_low = false,
    };
    ESP_GOTO_ON_ERROR(board_hal_backlight_init(&bl_cfg, &ctx->backlight), err, TAG,
                      "Backlight init failed");
    ctx->backlight_ready = true;
    ESP_GOTO_ON_ERROR(board_hal_backlight_set(&ctx->backlight, 0), err, TAG,
                      "Backlight off failed");

    /* 2. I2C bus for the GT911. */
    const board_hal_i2c_cfg_t i2c_cfg = {
        .port = BOARD_I2C_PORT,
        .sda_io = BOARD_I2C_SDA_GPIO,
        .scl_io = BOARD_I2C_SCL_GPIO,
        .clk_hz = BOARD_I2C_CLK_HZ,
    };
    ESP_GOTO_ON_ERROR(board_hal_i2c_bus_create(&i2c_cfg, &hal->i2c_bus), err, TAG,
                      "I2C init failed");

    /* 3. RGB panel. */
    ESP_GOTO_ON_ERROR(lcd5_panel_create(hal), err, TAG, "Panel init failed");

    /* 4. GT911 touch. */
    ESP_GOTO_ON_ERROR(lcd5_touch_create(hal), err, TAG, "Touch init failed");

    ESP_LOGI(TAG, "Board '%s' ready", hal->name);
    return ESP_OK;

err:
    lcd5_deinit(hal);
    return ret;
}

static esp_err_t lcd5_deinit(board_hal_t *hal)
{
    if (!hal) {
        return ESP_ERR_INVALID_ARG;
    }
    board_jotto5_ctx_t *ctx = (board_jotto5_ctx_t *)hal->ctx;
    if (ctx && ctx->backlight_ready) {
        (void)board_hal_backlight_set(&ctx->backlight, 0);
        ctx->backlight_ready = false;
    }
    if (hal->touch) {
        esp_lcd_touch_del(hal->touch);
        hal->touch = NULL;
    }
    if (hal->touch_io) {
        esp_lcd_panel_io_del(hal->touch_io);
        hal->touch_io = NULL;
    }
    if (hal->panel) {
        esp_lcd_panel_del(hal->panel);
        hal->panel = NULL;
    }
    /* hal->panel_io stays NULL on RGB */
    if (hal->i2c_bus) {
        i2c_del_master_bus(hal->i2c_bus);
        hal->i2c_bus = NULL;
    }
    return ESP_OK;
}

static esp_err_t lcd5_display_on(board_hal_t *hal)
{
    ESP_RETURN_ON_FALSE(hal && hal->panel, ESP_ERR_NOT_SUPPORTED, TAG, "No panel");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(hal->panel, true), TAG, "Display on failed");
    ((board_jotto5_ctx_t *)hal->ctx)->disp_on = true;
    return ESP_OK;
}

static esp_err_t lcd5_display_off(board_hal_t *hal)
{
    ESP_RETURN_ON_FALSE(hal && hal->panel, ESP_ERR_NOT_SUPPORTED, TAG, "No panel");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(hal->panel, false), TAG, "Display off failed");
    ((board_jotto5_ctx_t *)hal->ctx)->disp_on = false;
    return ESP_OK;
}

static esp_err_t lcd5_set_brightness(board_hal_t *hal, uint8_t percent)
{
    ESP_RETURN_ON_FALSE(hal && hal->ctx, ESP_ERR_INVALID_ARG, TAG, "Invalid arg");
    board_jotto5_ctx_t *ctx = (board_jotto5_ctx_t *)hal->ctx;
    ESP_RETURN_ON_FALSE(ctx->backlight_ready, ESP_ERR_INVALID_STATE, TAG, "No backlight");
    if (percent > 100) {
        percent = 100;
    }
    ESP_RETURN_ON_ERROR(board_hal_backlight_set(&ctx->backlight, percent), TAG,
                        "Backlight set failed");
    return ESP_OK;
}

static esp_err_t lcd5_get_brightness(board_hal_t *hal, uint8_t *percent_out)
{
    ESP_RETURN_ON_FALSE(hal && hal->ctx && percent_out, ESP_ERR_INVALID_ARG, TAG, "Invalid arg");
    board_jotto5_ctx_t *ctx = (board_jotto5_ctx_t *)hal->ctx;
    ESP_RETURN_ON_FALSE(ctx->backlight_ready, ESP_ERR_INVALID_STATE, TAG, "No backlight");
    *percent_out = ctx->backlight.percent;
    return ESP_OK;
}

static esp_err_t lcd5_touch_read(board_hal_t *hal, uint16_t *x, uint16_t *y, uint8_t *count)
{
    ESP_RETURN_ON_FALSE(hal && x && y && count, ESP_ERR_INVALID_ARG, TAG, "Invalid arg");
    ESP_RETURN_ON_FALSE(hal->touch, ESP_ERR_NOT_SUPPORTED, TAG, "No touch");
    ESP_RETURN_ON_ERROR(esp_lcd_touch_read_data(hal->touch), TAG, "Touch read failed");
    uint16_t xs[1] = {0}, ys[1] = {0}, strengths[1] = {0};
    uint8_t n = 0;
    esp_lcd_touch_get_coordinates(hal->touch, xs, ys, strengths, &n, 1);
    *x = xs[0];
    *y = ys[0];
    *count = n;
    return ESP_OK;
}
