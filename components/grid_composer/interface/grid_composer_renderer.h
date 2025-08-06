#pragma once
#ifndef GRID_COMPOSER_RENDERER_H
#define GRID_COMPOSER_RENDERER_H

#include "grid_composer_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  void *user_ctx;

  esp_err_t (*draw_text)(void *ctx, const grid_composer_text_info *, uint16_t color, bool fill);
  esp_err_t (*draw_figure)(void *ctx, const grid_composer_figure_info *, uint16_t color, bool fill);
  esp_err_t (*clear)(void *ctx);
} grid_composer_renderer;

#ifdef __cplusplus
}
#endif

#endif