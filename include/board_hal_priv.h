/*
 * SPDX-FileCopyrightText: 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Private shared types for board files (not part of the app-facing API).
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_io_expander.h"

#ifdef __cplusplus
extern "C" {
#endif

/** I2C bus wiring for a board */
typedef struct {
    i2c_port_t port;        /*!< e.g. I2C_NUM_0 */
    int sda_io;             /*!< GPIO, or -1 (GPIO_NUM_NC) if unset */
    int scl_io;             /*!< GPIO, or -1 (GPIO_NUM_NC) if unset */
    uint32_t clk_hz;        /*!< Bus clock, e.g. 400000 (informational; device cfgs set their own) */
} board_hal_i2c_cfg_t;

/** Backlight wiring for a board */
typedef struct {
    int gpio;                       /*!< Backlight GPIO, or -1 if controlled elsewhere (e.g. expander) */
    ledc_timer_t ledc_timer;        /*!< e.g. LEDC_TIMER_0 */
    ledc_channel_t ledc_channel;    /*!< e.g. LEDC_CHANNEL_0 */
    uint32_t freq_hz;               /*!< PWM frequency, 0 = 5000 Hz default */
    bool active_low;                /*!< true if 0% duty = full brightness */
} board_hal_backlight_cfg_t;

/** Runtime state for an LEDC backlight (lives in the board's ctx) */
typedef struct {
    ledc_channel_t channel;
    bool active_high;
    uint8_t percent;
} board_hal_backlight_t;

esp_err_t board_hal_i2c_bus_create(const board_hal_i2c_cfg_t *cfg, i2c_master_bus_handle_t *out_bus);
esp_err_t board_hal_backlight_init(const board_hal_backlight_cfg_t *cfg, board_hal_backlight_t *out_bl);
esp_err_t board_hal_backlight_set(board_hal_backlight_t *bl, uint8_t percent);

/**
 * @brief Drive an expander pin high/low with an optional hold (e.g. LCD reset pulse).
 */
esp_err_t board_hal_expander_pulse(esp_io_expander_handle_t io, uint32_t pin_mask, bool active_high,
                                   uint32_t hold_ms);

#ifdef __cplusplus
}
#endif
