#pragma once

#ifndef GRID_COMPOSER_H
#define GRID_COMPOSER_H

#include "esp_err.h"

#include "grid_composer_defs.h"
#include "grid_composer_renderer.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct grid_composer *grid_composer_handle;

esp_err_t
grid_composer_init(grid_composer_handle *out_composer);
esp_err_t
grid_composer_del(grid_composer_handle composer);

esp_err_t
grid_composer_add_cell(grid_composer_handle composer, grid_composer_renderer *renderer);

esp_err_t
grid_composer_draw_queue_send(grid_composer_handle composer, const grid_composer_draw_descriptor *render_desc, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif