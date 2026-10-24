/*
 * SPDX-FileCopyrightText: 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Shared helpers reusable by every board file.
 *
 * These are intentionally boring building blocks (I2C bus, LEDC backlight,
 * expander pin pulse) so each `boards*.c` stays focused on wiring + config.
 */

#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "board_hal.h"
#include "board_hal_priv.h"

static const char *TAG = "board_hal";

esp_err_t board_hal_i2c_bus_create(const board_hal_i2c_cfg_t *cfg, i2c_master_bus_handle_t *out_bus)
{
    ESP_RETURN_ON_FALSE(cfg && out_bus, ESP_ERR_INVALID_ARG, TAG, "Invalid arg");
    ESP_RETURN_ON_FALSE(cfg->sda_io >= 0 && cfg->scl_io >= 0, ESP_ERR_INVALID_ARG, TAG,
                        "I2C SDA/SCL pins not set");

    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = cfg->port,
        .sda_io_num = (gpio_num_t)cfg->sda_io,
        .scl_io_num = (gpio_num_t)cfg->scl_io,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, out_bus), TAG, "New I2C bus failed");
    ESP_LOGI(TAG, "I2C bus %d ready (SDA=%d SCL=%d %uHz)", cfg->port, cfg->sda_io, cfg->scl_io,
             (unsigned)cfg->clk_hz);
    return ESP_OK;
}

esp_err_t board_hal_backlight_init(const board_hal_backlight_cfg_t *cfg, board_hal_backlight_t *out_bl)
{
    ESP_RETURN_ON_FALSE(cfg && out_bl, ESP_ERR_INVALID_ARG, TAG, "Invalid arg");
    ESP_RETURN_ON_FALSE(cfg->gpio >= 0, ESP_ERR_INVALID_ARG, TAG, "Backlight GPIO not set");
    memset(out_bl, 0, sizeof(*out_bl));

    const ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = cfg->ledc_timer,
        .freq_hz = cfg->freq_hz ? cfg->freq_hz : 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_cfg), TAG, "LEDC timer config failed");

    const ledc_channel_config_t ch_cfg = {
        .gpio_num = cfg->gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = cfg->ledc_channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = cfg->ledc_timer,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch_cfg), TAG, "LEDC channel config failed");

    out_bl->channel = cfg->ledc_channel;
    out_bl->active_high = !cfg->active_low;
    out_bl->percent = 0;
    return ESP_OK;
}

esp_err_t board_hal_backlight_set(board_hal_backlight_t *bl, uint8_t percent)
{
    ESP_RETURN_ON_FALSE(bl, ESP_ERR_INVALID_ARG, TAG, "Invalid arg");
    if (percent > 100) {
        percent = 100;
    }
    /* 10-bit duty */
    uint32_t duty = ((uint32_t)percent * 1023U) / 100U;
    if (!bl->active_high) {
        duty = 1023U - duty;
    }
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, bl->channel, duty), TAG, "LEDC set duty failed");
    ESP_RETURN_ON_ERROR(ledc_update_duty(LEDC_LOW_SPEED_MODE, bl->channel), TAG, "LEDC update duty failed");
    bl->percent = percent;
    return ESP_OK;
}

esp_err_t board_hal_expander_pulse(esp_io_expander_handle_t io, uint32_t pin_mask, bool active_high,
                                   uint32_t hold_ms)
{
    ESP_RETURN_ON_FALSE(io, ESP_ERR_INVALID_ARG, TAG, "Invalid expander handle");
    ESP_RETURN_ON_ERROR(esp_io_expander_set_dir(io, pin_mask, IO_EXPANDER_OUTPUT), TAG,
                        "Expander set dir failed");
    ESP_RETURN_ON_ERROR(esp_io_expander_set_level(io, pin_mask, active_high ? 1 : 0), TAG,
                        "Expander assert failed");
    if (hold_ms) {
        vTaskDelay(pdMS_TO_TICKS(hold_ms));
    }
    ESP_RETURN_ON_ERROR(esp_io_expander_set_level(io, pin_mask, active_high ? 0 : 1), TAG,
                        "Expander release failed");
    return ESP_OK;
}
