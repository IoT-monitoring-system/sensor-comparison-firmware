#pragma once
#ifndef ADAFRUIT_RENDERER_H
#define ADAFRUIT_RENDERER_H

#define SH110X_NO_SPLASH 1

#include "Arduino.h"
#include "Wire.h"

#include "Adafruit_GFX.h"
#include "Adafruit_SH110X.h"
#include "U8g2_for_Adafruit_GFX.h"

#include "grid_composer_defs.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  Adafruit_GrayOLED *oled;
  U8G2_FOR_ADAFRUIT_GFX *u8g2;
  TwoWire *tw;
  uint16_t idx;
} adafruit_renderer_ctx;

esp_err_t
adafruit_gfx_clear(void *ctx);
esp_err_t
adafruit_gfx_draw_figure(void *ctx, const grid_composer_figure_info *info, uint16_t color, bool fill);
esp_err_t
adafruit_gfx_draw_text(void *ctx, const grid_composer_text_info *info, uint16_t color, bool fill);
void
adafruit_pca_select(TwoWire *tw, uint16_t i);

#ifdef __cplusplus
}
#endif
#endif