/*
 * SPDX-FileCopyrightText: 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_check.h"
#include "esp_log.h"

#include "board_hal_lvgl.h"

static const char *TAG = "board_hal_lvgl";

static esp_lv_adapter_rotation_t map_rotation(int deg)
{
    switch (deg) {
    case 90:
        return ESP_LV_ADAPTER_ROTATE_90;
    case 180:
        return ESP_LV_ADAPTER_ROTATE_180;
    case 270:
        return ESP_LV_ADAPTER_ROTATE_270;
    case 0:
    default:
        return ESP_LV_ADAPTER_ROTATE_0;
    }
}

lv_display_t *board_hal_lvgl_attach(board_hal_t *hal, const board_hal_lvgl_cfg_t *cfg)
{
    if (!hal || !hal->panel) {
        ESP_LOGE(TAG, "No panel — run board_hal_init() with a wired display first");
        return NULL;
    }
    if (hal->lv_display) {
        ESP_LOGW(TAG, "Already attached");
        return (lv_display_t *)hal->lv_display;
    }

    uint16_t hor = (cfg && cfg->hor_res) ? cfg->hor_res : hal->hor_res;
    uint16_t ver = (cfg && cfg->ver_res) ? cfg->ver_res : hal->ver_res;
    int rot = (cfg && cfg->rotation >= 0) ? cfg->rotation : (int)hal->rotation;
    if (!hor || !ver) {
        ESP_LOGE(TAG, "Board '%s' has no resolution set (hor=%u ver=%u)", hal->name, hor, ver);
        return NULL;
    }

    /* 1. Adapter init (once per boot) */
    if (!esp_lv_adapter_is_initialized()) {
        esp_lv_adapter_config_t acfg = ESP_LV_ADAPTER_DEFAULT_CONFIG();
        acfg.stack_in_psram = cfg ? cfg->stack_in_psram : false;
        if (esp_lv_adapter_init(&acfg) != ESP_OK) {
            ESP_LOGE(TAG, "Adapter init failed");
            return NULL;
        }
    }

    /* 2. Register RGB display straight from the board handles */
    esp_lv_adapter_display_config_t disp_cfg = ESP_LV_ADAPTER_DISPLAY_RGB_DEFAULT_CONFIG(
        hal->panel, hal->panel_io, hor, ver, map_rotation(rot));
    lv_display_t *disp = esp_lv_adapter_register_display(&disp_cfg);
    if (!disp) {
        ESP_LOGE(TAG, "Register display failed");
        return NULL;
    }

    /* 3. Register touch (adapter then owns sampling — no manual polling needed) */
    lv_indev_t *indev = NULL;
    bool want_touch = !(cfg && cfg->skip_touch);
    if (want_touch && hal->touch) {
        esp_lv_adapter_touch_config_t touch_cfg = ESP_LV_ADAPTER_TOUCH_DEFAULT_CONFIG(disp, hal->touch);
        indev = esp_lv_adapter_register_touch(&touch_cfg);
        if (!indev) {
            ESP_LOGW(TAG, "Register touch failed, continuing without touch");
        }
    }

    /* 4. Start LVGL task */
    if (esp_lv_adapter_start() != ESP_OK) {
        ESP_LOGE(TAG, "Adapter start failed");
        return NULL;
    }

    hal->lv_display = disp;
    hal->lv_touch_indev = indev;
    ESP_LOGI(TAG, "LVGL attached: %ux%u rot=%d touch=%s", hor, ver, rot, indev ? "yes" : "no");
    return disp;
}

esp_err_t board_hal_lvgl_detach(board_hal_t *hal)
{
    ESP_RETURN_ON_FALSE(hal, ESP_ERR_INVALID_ARG, TAG, "Invalid hal");
    if (hal->lv_touch_indev) {
        esp_lv_adapter_unregister_touch((lv_indev_t *)hal->lv_touch_indev);
        hal->lv_touch_indev = NULL;
    }
    if (hal->lv_display) {
        ESP_RETURN_ON_ERROR(esp_lv_adapter_unregister_display((lv_display_t *)hal->lv_display),
                            TAG, "Unregister display failed");
        hal->lv_display = NULL;
    }
    return esp_lv_adapter_deinit();
}
