/*
 * SPDX-FileCopyrightText: 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Generic board HAL interface ("abstract class" in C).
 *
 * A board file (see `boards/`) fills in a `board_hal_t` instance: its name,
 * capability flags, an ops table with function pointers, and board-private
 * context. Application code in `main` only ever talks to this interface, so
 * swapping boards means swapping one struct pointer — no app changes.
 *
 * Current scope: Display, Brightness, Touch, LVGL attach (see board_hal_lvgl.h).
 * Reserved for later: AudioCodec (`reserved_audio`), Camera (`reserved_camera`)
 * plus the `audio_*` / `camera_*` ops slots. They are NULL today and every
 * wrapper returns ESP_ERR_NOT_SUPPORTED when the slot is empty.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_bit_defs.h"
#include "driver/i2c_master.h"
#include "esp_lcd_types.h"
#include "esp_lcd_touch.h"
#include "esp_io_expander.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct board_hal board_hal_t;

/**
 * @brief Capability flags advertised by a board in `board_hal_t.caps`
 */
typedef enum {
    BOARD_HAL_CAP_DISPLAY    = BIT0, /*!< esp_lcd panel brought up by init() */
    BOARD_HAL_CAP_BRIGHTNESS = BIT1, /*!< set_brightness()/get_brightness() work */
    BOARD_HAL_CAP_TOUCH      = BIT2, /*!< touch_read() work */
    BOARD_HAL_CAP_AUDIO      = BIT3, /*!< reserved: audio codec (not implemented yet) */
    BOARD_HAL_CAP_CAMERA     = BIT4, /*!< reserved: camera (not implemented yet) */
} board_hal_cap_t;

/**
 * @brief Board operations table — the "virtual methods".
 *
 * A board only needs to fill the slots matching its `caps`. Leave the rest
 * NULL; the inline wrappers below translate that to ESP_ERR_NOT_SUPPORTED.
 */
typedef struct {
    esp_err_t (*init)(board_hal_t *hal);
    esp_err_t (*deinit)(board_hal_t *hal);

    /* Display */
    esp_err_t (*display_on)(board_hal_t *hal);
    esp_err_t (*display_off)(board_hal_t *hal);

    /* Brightness (0..100 percent) */
    esp_err_t (*set_brightness)(board_hal_t *hal, uint8_t percent);
    esp_err_t (*get_brightness)(board_hal_t *hal, uint8_t *percent_out);

    /* Touch: reads latest state, `count` = 0 when untouched */
    esp_err_t (*touch_read)(board_hal_t *hal, uint16_t *x, uint16_t *y, uint8_t *count);

    /* Reserved for later: audio codec */
    esp_err_t (*audio_init)(board_hal_t *hal);
    esp_err_t (*audio_deinit)(board_hal_t *hal);

    /* Reserved for later: camera */
    esp_err_t (*camera_init)(board_hal_t *hal);
    esp_err_t (*camera_deinit)(board_hal_t *hal);
} board_hal_ops_t;

/**
 * @brief Board instance. Declared (and owned) by the board file, used by the app.
 */
struct board_hal {
    const char *name;               /*!< Human-readable board name */
    uint32_t caps;                  /*!< Bitwise OR of `board_hal_cap_t` */
    const board_hal_ops_t *ops;     /*!< Operations table */

    /* Panel geometry. Set by the board file; used by LVGL attach. */
    uint16_t hor_res;               /*!< Horizontal resolution in pixels */
    uint16_t ver_res;               /*!< Vertical resolution in pixels */
    uint16_t rotation;              /*!< 0, 90, 180 or 270 */

    /* Handles populated by init(). App code may read these (e.g. to attach LVGL). */
    i2c_master_bus_handle_t i2c_bus;
    esp_io_expander_handle_t io_expander;
    esp_lcd_panel_io_handle_t panel_io;
    esp_lcd_panel_handle_t panel;
    esp_lcd_panel_io_handle_t touch_io;
    esp_lcd_touch_handle_t touch;

    /* Populated by board_hal_lvgl_attach(). */
    void *lv_display;               /*!< lv_display_t* (void* keeps core header LVGL-free) */
    void *lv_touch_indev;           /*!< lv_indev_t* (void* keeps core header LVGL-free) */

    void *ctx;                      /*!< Board-private state (defined by the board file) */

    void *reserved_audio;           /*!< Reserved: audio codec handle (NULL for now) */
    void *reserved_camera;          /*!< Reserved: camera handle (NULL for now) */
};

static inline bool board_hal_has_cap(const board_hal_t *hal, uint32_t cap)
{
    return hal && (hal->caps & cap);
}

static inline esp_err_t board_hal_init(board_hal_t *hal)
{
    if (!hal || !hal->ops || !hal->ops->init) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    return hal->ops->init(hal);
}

static inline esp_err_t board_hal_deinit(board_hal_t *hal)
{
    if (!hal || !hal->ops || !hal->ops->deinit) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    return hal->ops->deinit(hal);
}

static inline esp_err_t board_hal_display_on(board_hal_t *hal)
{
    if (!hal || !hal->ops || !hal->ops->display_on) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    return hal->ops->display_on(hal);
}

static inline esp_err_t board_hal_display_off(board_hal_t *hal)
{
    if (!hal || !hal->ops || !hal->ops->display_off) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    return hal->ops->display_off(hal);
}

static inline esp_err_t board_hal_set_brightness(board_hal_t *hal, uint8_t percent)
{
    if (!hal || !hal->ops || !hal->ops->set_brightness) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    return hal->ops->set_brightness(hal, percent > 100 ? 100 : percent);
}

static inline esp_err_t board_hal_get_brightness(board_hal_t *hal, uint8_t *percent_out)
{
    if (!hal || !hal->ops || !hal->ops->get_brightness) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    return hal->ops->get_brightness(hal, percent_out);
}

static inline esp_err_t board_hal_touch_read(board_hal_t *hal, uint16_t *x, uint16_t *y, uint8_t *count)
{
    if (!hal || !hal->ops || !hal->ops->touch_read) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    return hal->ops->touch_read(hal, x, y, count);
}

#ifdef __cplusplus
}
#endif
