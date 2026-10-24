/*
 * SPDX-FileCopyrightText: 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Registry of available boards. Include this from `main` and pick one.
 *
 * To add a board: copy `boards/board_template.c` to `boards/<name>.c`
 * (it is picked up automatically, see CMakeLists.txt), declare it here,
 * and point your app at it.
 */

#pragma once

#include "board_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

extern board_hal_t board_esp32_s3_touch_lcd_7; /*!< Waveshare ESP32-S3-Touch-LCD-7 */
extern board_hal_t board_esp32_s3_jotto_lcd_5; /*!< Jotto ESP32-S3 LCD 5 (800x480) */

#ifdef __cplusplus
}
#endif
