/*
 * SPDX-FileCopyrightText: 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Board: Waveshare ESP32-S3-Touch-LCD-7 (800x480 RGB + GT911 + CH422G).
 *
 * Wiring (from Waveshare wiki + official ESP-IDF demo
 * waveshareteam/ESP32-S3-Touch-LCD-7, 09_lvgl_v9_demo):
 *  - RGB panel, raw (no vendor init commands): VSYNC=3, HSYNC=46, DE=5,
 *    PCLK=7, D0..D15 = 14,38,18,17,10,39,0,45,48,47,21,1,2,42,41,40.
 *    DISP pin not connected.
 *  - I2C0 SDA=8, SCL=9, 400kHz — shared by CH422G + GT911 (0x5D).
 *  - Touch GT911 in polling mode (INT/RST = NC for the driver). Hardware
 *    reset: GPIO4 (= TP_INT line) selects the I2C address, EXIO1 (= TP_RST
 *    on CH422) pulses the reset.
 *  - Backlight: NO GPIO PWM on this board — enable pin is EXIO2 on CH422
 *    (on/off only, no dimming).
 *
 * CH422 IO-byte map used here (verified against Waveshare's byte writes
 * 0x2C/0x2E/0x1E to address 0x38, where EXIOn == IO bit n):
 *  - IO1 (EXIO1) TP_RST, active low
 *  - IO2 (EXIO2) DISP/backlight enable, active high
 *  - IO3 (EXIO3) LCD_RST, active low (kept released)
 *  - IO4 (EXIO4) SD_CS, active low (kept deselected)
 *  - IO5 (EXIO5) USB_SEL: low = USB mode, high = CAN mode (kept USB)
 * Steady-state IO byte = 0x1E.
 *
 * NOTE: framebuffers live in PSRAM (3x 800x480x2 = ~2.3MB). PSRAM must be
 * enabled (see sdkconfig.defaults). num_fbs is computed from the same tear
 * mode/rotation the LVGL attach uses, so the two always agree.
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
#include "esp_io_expander_ch422.h"

#include "board_hal.h"
#include "board_hal_priv.h"

/* ================================ wiring ================================= */

#define BOARD_I2C_PORT          (I2C_NUM_0)
#define BOARD_I2C_SDA_GPIO      (8)
#define BOARD_I2C_SCL_GPIO      (9)

#define BOARD_LCD_H_RES         (800)
#define BOARD_LCD_V_RES         (480)
#define BOARD_LCD_ROTATION      (ESP_LV_ADAPTER_ROTATE_0)

/* User-supplied 35Hz RGB timing */
#define BOARD_LCD_PCLK_HZ           (18 * 1000 * 1000)
#define BOARD_LCD_HSYNC_PW          (40)
#define BOARD_LCD_HSYNC_BP          (40)
#define BOARD_LCD_HSYNC_FP          (48)
#define BOARD_LCD_VSYNC_PW          (23)
#define BOARD_LCD_VSYNC_BP          (32)
#define BOARD_LCD_VSYNC_FP          (13)

#define BOARD_LCD_VSYNC_GPIO    (GPIO_NUM_3)
#define BOARD_LCD_HSYNC_GPIO    (GPIO_NUM_46)
#define BOARD_LCD_DE_GPIO       (GPIO_NUM_5)
#define BOARD_LCD_PCLK_GPIO     (GPIO_NUM_7)
#define BOARD_LCD_DATA_GPIOS    { 14, 38, 18, 17, 10, 39, 0, 45, 48, 47, 21, 1, 2, 42, 41, 40 }

/* Backlight is on the expander (no GPIO PWM on this board) */
#define BOARD_BL_GPIO           (-1)

/* CH422 IO-bit numbers (expander pins 4..11 == IO0..IO7) */
#define CH422_IO_TP_RST         (1) /* EXIO1 */
#define CH422_IO_BL             (2) /* EXIO2 */
#define CH422_IO_LCD_RST        (3) /* EXIO3 */
#define CH422_IO_SD_CS          (4) /* EXIO4 */
#define CH422_IO_USB_SEL        (5) /* EXIO5 */

#define CH422_PIN(n)            (IO_EXPANDER_PIN_NUM_0 << ((n) + 4))
#define CH422_STEADY_IO_BYTE    (0x1E)

/* Touch */
#define BOARD_TOUCH_INT_GPIO    (GPIO_NUM_4) /* TP_INT: driven low during reset = address 0x5D */
#define BOARD_TOUCH_I2C_ADDR    (ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS)

/* ============================== end wiring =============================== */

static const char *TAG = "board_7inch";

typedef struct {
    uint8_t brightness; /* 0 or 100: expander backlight is on/off only */
    bool disp_on;
} board_7inch_ctx_t;

static board_7inch_ctx_t s_ctx;

/* ---- forward declarations (the ops table) ---- */
static esp_err_t lcd7_init(board_hal_t *hal);
static esp_err_t lcd7_deinit(board_hal_t *hal);
static esp_err_t lcd7_display_on(board_hal_t *hal);
static esp_err_t lcd7_display_off(board_hal_t *hal);
static esp_err_t lcd7_set_brightness(board_hal_t *hal, uint8_t percent);
static esp_err_t lcd7_get_brightness(board_hal_t *hal, uint8_t *percent_out);
static esp_err_t lcd7_touch_read(board_hal_t *hal, uint16_t *x, uint16_t *y, uint8_t *count);

static const board_hal_ops_t s_ops = {
    .init = lcd7_init,
    .deinit = lcd7_deinit,
    .display_on = lcd7_display_on,
    .display_off = lcd7_display_off,
    .set_brightness = lcd7_set_brightness,
    .get_brightness = lcd7_get_brightness,
    .touch_read = lcd7_touch_read,
    .audio_init = NULL,     /* reserved */
    .audio_deinit = NULL,   /* reserved */
    .camera_init = NULL,    /* reserved */
    .camera_deinit = NULL,  /* reserved */
};

board_hal_t board_esp32_s3_touch_lcd_7 = {
    .name = "esp32_s3_touch_lcd_7",
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

/**
 * @brief Drive the whole CH422 IO byte to the Waveshare steady state (0x1E).
 *
 * All used pins become outputs; OC0-3 stay outputs high (unused on this board).
 */
static esp_err_t lcd7_expander_steady_state(board_hal_t *hal)
{
    /* OC0-3 (pins 0-3) + IO0-7 (pins 4-11): everything output ... */
    ESP_RETURN_ON_ERROR(esp_io_expander_set_dir(hal->io_expander, 0xFFF, IO_EXPANDER_OUTPUT),
                        TAG, "Expander set dir failed");
    /* ... high, except the active-low / strap pins that must stay low. */
    uint32_t out = 0xFFF;
    out &= ~CH422_PIN(0);               /* IO0 low (matches 0x1E) */
    out &= ~CH422_PIN(CH422_IO_USB_SEL); /* USB mode, not CAN */
    out &= ~CH422_PIN(6);               /* IO6 low (matches 0x1E) */
    out &= ~CH422_PIN(7);               /* IO7 low (matches 0x1E) */
    /* IO1 TP_RST=1, IO2 BL=1, IO3 LCD_RST=1, IO4 SD_CS=1 already high in 0xFFF */
    for (int i = 0; i < 12; i++) {
        ESP_RETURN_ON_ERROR(esp_io_expander_set_level(hal->io_expander, BIT(i), (out >> i) & 1),
                            TAG, "Expander set level failed");
    }
    return ESP_OK;
}

/**
 * @brief GT911 hardware reset, exactly per Waveshare demo.
 *
 * GPIO4 (TP_INT) low selects address 0x5D; EXIO1 (TP_RST) pulses low->high.
 * GPIO4 is intentionally left driven low afterwards, same as the demo.
 */
static esp_err_t lcd7_touch_hw_reset(board_hal_t *hal)
{
    const gpio_config_t int_cfg = {
        .pin_bit_mask = 1ULL << BOARD_TOUCH_INT_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&int_cfg), TAG, "TP_INT gpio config failed");

    ESP_RETURN_ON_ERROR(esp_io_expander_set_level(hal->io_expander, CH422_PIN(CH422_IO_TP_RST), 0),
                        TAG, "TP_RST assert failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(gpio_set_level(BOARD_TOUCH_INT_GPIO, 0), TAG, "TP_INT low failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(esp_io_expander_set_level(hal->io_expander, CH422_PIN(CH422_IO_TP_RST), 1),
                        TAG, "TP_RST release failed");
    vTaskDelay(pdMS_TO_TICKS(200));
    return ESP_OK;
}

static esp_err_t lcd7_panel_create(board_hal_t *hal)
{
    /* Same tear mode/rotation the LVGL attach uses -> matching framebuffer count.
     * Non-zero rotation needs 3 full frame buffers; keep in sync with
     * board_hal_lvgl_attach() (hal->rotation). */
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

static esp_err_t lcd7_touch_create(board_hal_t *hal)
{
    ESP_RETURN_ON_ERROR(lcd7_touch_hw_reset(hal), TAG, "Touch HW reset failed");

    esp_lcd_panel_io_i2c_config_t tp_io_cfg = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(hal->i2c_bus, &tp_io_cfg, &hal->touch_io),
                        TAG, "Touch panel IO create failed");

    /* Touch frame = native panel frame (800x480). The LVGL adapter only
     * scales touch coords, it does NOT rotate them, so the GT911 driver
     * flags must be the exact inverse of the display rotation
     * (see display_coord_to_phy in the adapter: 90° -> phy_x = W-1-lv_y,
     * phy_y = lv_x). Driver applies mirror_x/y first, then swap_xy.
     *   0°:   identity                       180°: mirror_x + mirror_y
     *   90°:  mirror_x + swap_xy             270°: mirror_y + swap_xy   */
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
        .rst_gpio_num = GPIO_NUM_NC, /* reset done above via GPIO4 + EXIO1 */
        .int_gpio_num = GPIO_NUM_NC, /* polling mode */
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = swap_xy,
            .mirror_x = mirror_x,
            .mirror_y = mirror_y,
        },
    };
    ESP_RETURN_ON_ERROR(esp_lcd_touch_new_i2c_gt911(hal->touch_io, &tp_cfg, &hal->touch),
                        TAG, "GT911 create failed");
    return ESP_OK;
}

static esp_err_t lcd7_init(board_hal_t *hal)
{
    ESP_RETURN_ON_FALSE(hal, ESP_ERR_INVALID_ARG, TAG, "Invalid hal");
    esp_err_t ret = ESP_OK;
    board_7inch_ctx_t *ctx = (board_7inch_ctx_t *)hal->ctx;
    ctx->brightness = 0;
    ctx->disp_on = false;

    /* 1. Shared I2C bus for CH422 + GT911 */
    const board_hal_i2c_cfg_t i2c_cfg = {
        .port = BOARD_I2C_PORT,
        .sda_io = BOARD_I2C_SDA_GPIO,
        .scl_io = BOARD_I2C_SCL_GPIO,
        .clk_hz = 400000,
    };
    ESP_RETURN_ON_ERROR(board_hal_i2c_bus_create(&i2c_cfg, &hal->i2c_bus), TAG, "I2C init failed");

    /* 2. CH422 expander to Waveshare steady state (backlight off for now) */
    ESP_GOTO_ON_ERROR(esp_io_expander_new_i2c_ch422(hal->i2c_bus, 0, &hal->io_expander),
                      err, TAG, "CH422 init failed");
    ESP_GOTO_ON_ERROR(lcd7_expander_steady_state(hal), err, TAG, "Expander setup failed");
    ESP_GOTO_ON_ERROR(esp_io_expander_set_level(hal->io_expander, CH422_PIN(CH422_IO_BL), 0),
                      err, TAG, "Backlight off failed");

    /* 3. RGB panel */
    ESP_GOTO_ON_ERROR(lcd7_panel_create(hal), err, TAG, "Panel init failed");

    /* 4. Touch */
    ESP_GOTO_ON_ERROR(lcd7_touch_create(hal), err, TAG, "Touch init failed");

    ESP_LOGI(TAG, "Board '%s' ready", hal->name);
    return ESP_OK;

err:
    lcd7_deinit(hal);
    return ret;
}

static esp_err_t lcd7_deinit(board_hal_t *hal)
{
    if (!hal) {
        return ESP_ERR_INVALID_ARG;
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
    if (hal->io_expander) {
        esp_io_expander_del(hal->io_expander);
        hal->io_expander = NULL;
    }
    if (hal->i2c_bus) {
        i2c_del_master_bus(hal->i2c_bus);
        hal->i2c_bus = NULL;
    }
    return ESP_OK;
}

static esp_err_t lcd7_display_on(board_hal_t *hal)
{
    ESP_RETURN_ON_FALSE(hal && hal->panel, ESP_ERR_NOT_SUPPORTED, TAG, "No panel");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(hal->panel, true), TAG, "Display on failed");
    ((board_7inch_ctx_t *)hal->ctx)->disp_on = true;
    return ESP_OK;
}

static esp_err_t lcd7_display_off(board_hal_t *hal)
{
    ESP_RETURN_ON_FALSE(hal && hal->panel, ESP_ERR_NOT_SUPPORTED, TAG, "No panel");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(hal->panel, false), TAG, "Display off failed");
    ((board_7inch_ctx_t *)hal->ctx)->disp_on = false;
    return ESP_OK;
}

static esp_err_t lcd7_set_brightness(board_hal_t *hal, uint8_t percent)
{
    ESP_RETURN_ON_FALSE(hal && hal->io_expander, ESP_ERR_INVALID_STATE, TAG, "No expander");
    if (percent > 100) {
        percent = 100;
    }
    /* EXIO2 is a plain enable: any percent > 0 means on */
    ESP_RETURN_ON_ERROR(esp_io_expander_set_level(hal->io_expander, CH422_PIN(CH422_IO_BL),
                                                  percent ? 1 : 0), TAG, "Backlight set failed");
    ((board_7inch_ctx_t *)hal->ctx)->brightness = percent ? 100 : 0;
    return ESP_OK;
}

static esp_err_t lcd7_get_brightness(board_hal_t *hal, uint8_t *percent_out)
{
    ESP_RETURN_ON_FALSE(hal && percent_out, ESP_ERR_INVALID_ARG, TAG, "Invalid arg");
    *percent_out = ((board_7inch_ctx_t *)hal->ctx)->brightness;
    return ESP_OK;
}

static esp_err_t lcd7_touch_read(board_hal_t *hal, uint16_t *x, uint16_t *y, uint8_t *count)
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
