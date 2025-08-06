#pragma once

#ifndef GRID_COMPOSER_DEFS_H
#define GRID_COMPOSER_DEFS_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GRID_COMPOSER_TEXT_DESCRIPTOR(_row, _col, _x, _y, _v_align, _h_align, _text, _font, _color, _clear_before)               \
  (grid_composer_draw_descriptor) {                                                                                              \
    .cell_row = (_row), .cell_col = (_col), .clear_before = _clear_before,                                                       \
    .draw_obj =                                                                                                                  \
    {.fill = false,                                                                                                              \
     .color = (_color),                                                                                                          \
     .content_type = GRID_COMPOSER_CONTENT_TEXT,                                                                                 \
     .text = {                                                                                                                   \
         .x = (_x),                                                                                                              \
         .y = (_y),                                                                                                              \
         .v_align = (_v_align),                                                                                                  \
         .h_align = (_h_align),                                                                                                  \
         .text = (_text),                                                                                                        \
         .font = (_font),                                                                                                        \
     } }                                                                                                                         \
  }

#define GRID_COMPOSER_RECT_DESCRIPTOR(_row, _col, _x, _y, _w, _h, _fill, _color, _clear_before)                                  \
  (grid_composer_draw_descriptor) {                                                                                              \
    .cell_row = (_row), .cell_col = (_col), .clear_before = _clear_before,                                                       \
    .draw_obj =                                                                                                                  \
    {.fill = (_fill),                                                                                                            \
     .color = (_color),                                                                                                          \
     .content_type = GRID_COMPOSER_CONTENT_FIGURE,                                                                               \
     .figure = {.type = GRID_COMPOSER_FIGURE_RECT,                                                                               \
                .info.rect = {                                                                                                   \
                    .x = (_x),                                                                                                   \
                    .y = (_y),                                                                                                   \
                    .w = (_w),                                                                                                   \
                    .h = (_h),                                                                                                   \
                }} }                                                                                                             \
  }

#define GRID_COMPOSER_ROUND_RECT_DESCRIPTOR(_row, _col, _x, _y, _w, _h, _r, _fill, _color, _clear_before)                        \
  (grid_composer_draw_descriptor) {                                                                                              \
    .cell_row = (_row), .cell_col = (_col), .clear_before = _clear_before,                                                       \
    .draw_obj =                                                                                                                  \
    {.fill = (_fill),                                                                                                            \
     .color = (_color),                                                                                                          \
     .content_type = GRID_COMPOSER_CONTENT_FIGURE,                                                                               \
     .figure = {.type = GRID_COMPOSER_FIGURE_ROUND_RECT,                                                                         \
                .info.round_rect = {                                                                                             \
                    .x = (_x),                                                                                                   \
                    .y = (_y),                                                                                                   \
                    .w = (_w),                                                                                                   \
                    .h = (_h),                                                                                                   \
                    .r = (_r),                                                                                                   \
                }} }                                                                                                             \
  }

#define GRID_COMPOSER_CIRCLE_DESCRIPTOR(_row, _col, _x0, _y0, _r, _fill, _color, _clear_before)                                  \
  (grid_composer_draw_descriptor) {                                                                                              \
    .cell_row = (_row), .cell_col = (_col), .clear_before = _clear_before,                                                       \
    .draw_obj =                                                                                                                  \
    {.fill = (_fill),                                                                                                            \
     .color = (_color),                                                                                                          \
     .content_type = GRID_COMPOSER_CONTENT_FIGURE,                                                                               \
     .figure = {.type = GRID_COMPOSER_FIGURE_CIRCLE,                                                                             \
                .info.circle = {                                                                                                 \
                    .x0 = (_x0),                                                                                                 \
                    .y0 = (_y0),                                                                                                 \
                    .r = (_r),                                                                                                   \
                }} }                                                                                                             \
  }

#define GRID_COMPOSER_TRIANGLE_DESCRIPTOR(_row, _col, _x0, _y0, _x1, _y1, _x2, _y2, _fill, _color, _clear_before)                \
  (grid_composer_draw_descriptor) {                                                                                              \
    .cell_row = (_row), .cell_col = (_col), .clear_before = _clear_before,                                                       \
    .draw_obj =                                                                                                                  \
    {.fill = (_fill),                                                                                                            \
     .color = (_color),                                                                                                          \
     .content_type = GRID_COMPOSER_CONTENT_FIGURE,                                                                               \
     .figure = {.type = GRID_COMPOSER_FIGURE_TRIANGLE,                                                                           \
                .info.triangle = {                                                                                               \
                    .x0 = (_x0),                                                                                                 \
                    .y0 = (_y0),                                                                                                 \
                    .x1 = (_x1),                                                                                                 \
                    .y1 = (_y1),                                                                                                 \
                    .x2 = (_x2),                                                                                                 \
                    .y2 = (_y2),                                                                                                 \
                }} }                                                                                                             \
  }

#define GRID_COMPOSER_LINE_DESCRIPTOR(_row, _col, _x0, _y0, _x1, _y1, _w, _h, _fill, _color, _clear_before)                      \
  (grid_composer_draw_descriptor) {                                                                                              \
    .cell_row = (_row), .cell_col = (_col), .clear_before = _clear_before,                                                       \
    .draw_obj =                                                                                                                  \
    {.fill = (_fill),                                                                                                            \
     .color = (_color),                                                                                                          \
     .content_type = GRID_COMPOSER_CONTENT_FIGURE,                                                                               \
     .figure = {.type = GRID_COMPOSER_FIGURE_LINE,                                                                               \
                .info.line = {                                                                                                   \
                    .x0 = (_x0),                                                                                                 \
                    .y0 = (_y0),                                                                                                 \
                    .x1 = (_x1),                                                                                                 \
                    .y1 = (_y1),                                                                                                 \
                    .w = (_w),                                                                                                   \
                    .h = (_h),                                                                                                   \
                }} }                                                                                                             \
  }

#define GRID_COMPOSER_CLEAR_DESCRIPTOR(_row, _col)                                                                               \
  (grid_composer_draw_descriptor) {                                                                                              \
    .cell_row = (_row), .cell_col = (_col), .draw_obj = {                                                                        \
      .content_type = GRID_COMPOSER_CONTENT_CLEAR,                                                                               \
                                                                                                                                 \
    }                                                                                                                            \
  }

typedef enum {
  GRID_COMPOSER_CONTENT_INVALID = -1,
  GRID_COMPOSER_CONTENT_TEXT,
  GRID_COMPOSER_CONTENT_FIGURE,
  GRID_COMPOSER_CONTENT_CLEAR,
} grid_composer_content_type;

typedef enum {
  GRID_COMPOSER_FIGURE_INVALID = -1,
  GRID_COMPOSER_FIGURE_RECT,
  GRID_COMPOSER_FIGURE_ROUND_RECT,
  GRID_COMPOSER_FIGURE_CIRCLE,
  GRID_COMPOSER_FIGURE_TRIANGLE,
  GRID_COMPOSER_FIGURE_LINE,
} grid_composer_figure_type;

typedef enum {
  GRID_COMPOSER_H_ALIGN_NONE = 0,
  GRID_COMPOSER_H_ALIGN_LEFT,
  GRID_COMPOSER_H_ALIGN_CENTER,
  GRID_COMPOSER_H_ALIGN_RIGHT,
} grid_composer_h_align;

typedef enum {
  GRID_COMPOSER_V_ALIGN_NONE = 0,
  GRID_COMPOSER_V_ALIGN_TOP,
  GRID_COMPOSER_V_ALIGN_CENTER,
  GRID_COMPOSER_V_ALIGN_BOTTOM,
} grid_composer_v_align;

typedef struct {
  grid_composer_figure_type type;
  union {
    struct figure_rect_info {
      int16_t x;
      int16_t y;
      int16_t w;
      int16_t h;
    } rect;

    struct figure_round_rect_info {
      int16_t x;
      int16_t y;
      int16_t w;
      int16_t h;
      int16_t r;
    } round_rect;

    struct figure_circle_info {
      int16_t x0;
      int16_t y0;
      int16_t r;
    } circle;

    struct figure_triangle_info {
      int16_t x0;
      int16_t y0;
      int16_t x1;
      int16_t y1;
      int16_t x2;
      int16_t y2;
    } triangle;

    struct figure_line_info {
      int16_t x0;
      int16_t y0;
      int16_t x1;
      int16_t y1;
      uint16_t w; // Used only for horizontal lines
      uint16_t h; // Used only for vertical lines
    } line;
  } info;
} grid_composer_figure_info;

typedef struct {
  int16_t x;
  int16_t y;
  grid_composer_v_align v_align;
  grid_composer_h_align h_align;
  const char *text;
  const uint8_t *font;
} grid_composer_text_info;

typedef struct {
  bool fill;
  uint16_t color;
  grid_composer_content_type content_type;
  union {
    grid_composer_figure_info figure;
    grid_composer_text_info text;
  };
} grid_composer_draw_obj_info;

typedef struct {
  int8_t cell_row;
  int8_t cell_col;

  bool clear_before;

  grid_composer_draw_obj_info draw_obj;
} grid_composer_draw_descriptor;

#ifdef __cplusplus
}
#endif

#endif