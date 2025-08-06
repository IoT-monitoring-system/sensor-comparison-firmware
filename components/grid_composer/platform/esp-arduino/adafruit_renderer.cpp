#include "esp_check.h"
#include "esp_err.h"

#include "adafruit_renderer_private.h"

#include "adafruit_renderer.h"

static const char *TAG = "adafruit_renderer";

esp_err_t
adafruit_gfx_draw_figure(void *ctx, const grid_composer_figure_info *info, uint16_t color, bool fill) {
  ESP_RETURN_ON_FALSE(ctx, ESP_ERR_INVALID_ARG, TAG, "invalid gfx ctx");
  adafruit_renderer_ctx *gfx_ctx = (adafruit_renderer_ctx *)ctx;

  ESP_RETURN_ON_FALSE(gfx_ctx->u8g2, ESP_ERR_INVALID_ARG, TAG, "invalid u8g2 renderer");
  ESP_RETURN_ON_FALSE(gfx_ctx->oled, ESP_ERR_INVALID_ARG, TAG, "invalid oled renderer");
  ESP_RETURN_ON_FALSE(gfx_ctx->tw, ESP_ERR_INVALID_ARG, TAG, "invalid tw object");

  adafruit_pca_select(gfx_ctx->tw, gfx_ctx->idx);

  Adafruit_GrayOLED *oled = gfx_ctx->oled;

  switch (info->type) {
  case GRID_COMPOSER_FIGURE_RECT:
    if (fill) {
      oled->fillRect(info->info.rect.x, info->info.rect.y, info->info.rect.w, info->info.rect.h, color);
    } else {
      oled->drawRect(info->info.rect.x, info->info.rect.y, info->info.rect.w, info->info.rect.h, color);
    }
    break;
  case GRID_COMPOSER_FIGURE_ROUND_RECT:
    if (fill) {
      oled->fillRoundRect(info->info.round_rect.x, info->info.round_rect.y, info->info.round_rect.w, info->info.round_rect.h,
                          info->info.round_rect.r, color);
    } else {
      oled->drawRoundRect(info->info.round_rect.x, info->info.round_rect.y, info->info.round_rect.w, info->info.round_rect.h,
                          info->info.round_rect.r, color);
    }
    break;
  case GRID_COMPOSER_FIGURE_CIRCLE:
    if (fill) {
      oled->fillCircle(info->info.circle.x0, info->info.circle.y0, info->info.circle.r, color);
    } else {
      oled->drawCircle(info->info.circle.x0, info->info.circle.y0, info->info.circle.r, color);
    }
    break;
  case GRID_COMPOSER_FIGURE_TRIANGLE:
    if (fill) {
      oled->fillTriangle(info->info.triangle.x0, info->info.triangle.y0, info->info.triangle.x1, info->info.triangle.y1,
                         info->info.triangle.x2, info->info.triangle.y2, color);
    } else {
      oled->drawTriangle(info->info.triangle.x0, info->info.triangle.y0, info->info.triangle.x1, info->info.triangle.x1,
                         info->info.triangle.x2, info->info.triangle.y2, color);
    }
    break;
  case GRID_COMPOSER_FIGURE_LINE:
    oled->drawLine(info->info.line.x0, info->info.line.y0, info->info.line.x1, info->info.line.y1, color);
    break;
  default:
    return ESP_ERR_INVALID_ARG;
  }
  oled->display();

  return ESP_OK;
}
esp_err_t
adafruit_gfx_draw_text(void *ctx, const grid_composer_text_info *info, uint16_t color, bool fill) {
  ESP_RETURN_ON_FALSE(ctx, ESP_ERR_INVALID_ARG, TAG, "invalid gfx ctx");
  adafruit_renderer_ctx *gfx_ctx = (adafruit_renderer_ctx *)ctx;

  ESP_RETURN_ON_FALSE(gfx_ctx->u8g2, ESP_ERR_INVALID_ARG, TAG, "invalid u8g2 renderer");
  ESP_RETURN_ON_FALSE(gfx_ctx->oled, ESP_ERR_INVALID_ARG, TAG, "invalid oled renderer");
  ESP_RETURN_ON_FALSE(gfx_ctx->tw, ESP_ERR_INVALID_ARG, TAG, "invalid tw object");

  adafruit_pca_select(gfx_ctx->tw, gfx_ctx->idx);

  Adafruit_GrayOLED *oled = gfx_ctx->oled;
  U8G2_FOR_ADAFRUIT_GFX *gfx = gfx_ctx->u8g2;

  gfx->setFont(info->font);
  gfx->setForegroundColor(color);

  int ascent = gfx->getFontAscent();
  int descent = gfx->getFontDescent();

  int text_width = gfx->getUTF8Width(info->text);
  int x = info->x;
  int y = info->y;

  switch (info->h_align) {
  case GRID_COMPOSER_H_ALIGN_LEFT:
    break;
  case GRID_COMPOSER_H_ALIGN_RIGHT:
    x = oled->width() - text_width;
    break;
  case GRID_COMPOSER_H_ALIGN_CENTER:
    x = (oled->width() - text_width) / 2;
    break;
  case GRID_COMPOSER_H_ALIGN_NONE:
    break;
  default:
    ESP_LOGE(TAG, "alignment not supported");
  }

  switch (info->v_align) {
  case GRID_COMPOSER_V_ALIGN_TOP:
    y = ascent;
    break;
  case GRID_COMPOSER_V_ALIGN_BOTTOM:
    y = oled->height() + descent;
    break;
  case GRID_COMPOSER_V_ALIGN_CENTER:
    y = (oled->height() + ascent - descent) / 2;
    break;
  case GRID_COMPOSER_V_ALIGN_NONE:
    break;
  default:
    ESP_LOGE(TAG, "alignment not supported");
  }
  ESP_RETURN_ON_FALSE(y >= 0 && x >= 0, ESP_ERR_INVALID_ARG, TAG, "negative cursor position not supported");

  gfx->drawUTF8(x, y, info->text);

  oled->display();

  return ESP_OK;
}
esp_err_t
adafruit_gfx_clear(void *ctx) {
  ESP_RETURN_ON_FALSE(ctx, ESP_ERR_INVALID_ARG, TAG, "invalid gfx ctx");
  adafruit_renderer_ctx *gfx_ctx = (adafruit_renderer_ctx *)ctx;

  ESP_RETURN_ON_FALSE(gfx_ctx->u8g2, ESP_ERR_INVALID_ARG, TAG, "invalid u8g2 renderer");
  ESP_RETURN_ON_FALSE(gfx_ctx->oled, ESP_ERR_INVALID_ARG, TAG, "invalid oled renderer");
  ESP_RETURN_ON_FALSE(gfx_ctx->tw, ESP_ERR_INVALID_ARG, TAG, "invalid tw object");

  adafruit_pca_select(gfx_ctx->tw, gfx_ctx->idx);

  Adafruit_GrayOLED *oled = gfx_ctx->oled;
  U8G2_FOR_ADAFRUIT_GFX *gfx = gfx_ctx->u8g2;

  gfx->home();

  oled->clearDisplay();
  oled->display();
  return ESP_OK;
}

void
adafruit_pca_select(TwoWire *tw, uint16_t i) {
  if (i > 7)
    return;

  tw->beginTransmission(PCA1_ADDR);
  tw->write(1 << i);
  tw->endTransmission();
}
