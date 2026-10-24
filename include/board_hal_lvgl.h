/*
 * SPDX-FileCopyrightText: 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief LVGL attach for any board: one call, no board-specific code in main.
 *
 * Data flow: board file fills `hal->panel`, `hal->touch`, `hor_res/ver_res`
 * during init()  →  main calls board_hal_lvgl_attach(hal, NULL)  →
 * adapter init + RGB display register + touch register + adapter start.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_lv_adapter.h"
#include "board_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Optional overrides. Pass NULL for "take everything from the board struct".
 */
typedef struct {
    uint16_t hor_res;       /*!< 0 = use hal->hor_res */
    uint16_t ver_res;       /*!< 0 = use hal->ver_res */
    int rotation;           /*!< -1 = use hal->rotation (0/90/180/270) */
    bool skip_touch;        /*!< true = don't register touch even if hal->touch exists */
    bool stack_in_psram;    /*!< place LVGL task stack in PSRAM when available */
} board_hal_lvgl_cfg_t;

/**
 * @brief Attach the board's panel (+touch) to LVGL via esp_lvgl_adapter.
 *
 * Safe to call once; requires hal->panel (i.e. board init + display ready).
 * Fills hal->lv_display / hal->lv_touch_indev.
 *
 * @param hal Board instance (already init'ed)
 * @param cfg Optional overrides, or NULL for board defaults
 * @return lv_display_t* on success, NULL on failure
 */
lv_display_t *board_hal_lvgl_attach(board_hal_t *hal, const board_hal_lvgl_cfg_t *cfg);

/**
 * @brief Detach LVGL (unregister touch + display, deinit adapter).
 */
esp_err_t board_hal_lvgl_detach(board_hal_t *hal);

#ifdef __cplusplus
}
#endif
