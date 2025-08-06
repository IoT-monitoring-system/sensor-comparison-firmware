#include "esp_check.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "grid_composer.h"
#include "grid_composer_renderer.h"
#include "private/grid_composer_private.h"

enum cell_dev_state {
  DEV_STATE_INVALID = 0,
  DEV_STATE_INITIALIZED,
};

struct grid_composer_cell_dev {
  enum cell_dev_state state;
  grid_composer_renderer renderer;
};

struct grid_composer {
  struct grid_composer_cell_dev *cell_devices[GRID_COMPOSER_CELL_MAX_ROWS][GRID_COMPOSER_CELL_MAX_COLS];
  QueueHandle_t draw_queue;
  TaskHandle_t draw_update_task_handle;
};

static const char *TAG = "grid_composer_module";

static struct grid_composer grid_composer_instance;

static void
task_draw_update(void *arg);

static esp_err_t
draw(struct grid_composer_cell_dev *cell_dev, const grid_composer_draw_obj_info *draw_obj, bool clear_before);

static esp_err_t
push_cell(grid_composer_handle composer, struct grid_composer_cell_dev *cell_dev);

static bool
validate_renderer(const grid_composer_renderer *renderer);

esp_err_t
grid_composer_init(grid_composer_handle *out_composer) {
  esp_err_t ret = ESP_OK;

  grid_composer_instance.draw_queue = xQueueCreate(GRID_COMPOSER_DRAW_QUEUE_MAX_ITEMS, sizeof(grid_composer_draw_descriptor));

  ESP_RETURN_ON_FALSE(grid_composer_instance.draw_queue, ESP_ERR_NO_MEM, TAG, "failed to create draw queue, not enough memory");

  ESP_RETURN_ON_FALSE(xTaskCreatePinnedToCore(task_draw_update, "grid draw tsk", GRID_COMPOSER_DRAW_TASK_STACK,
                                              &grid_composer_instance, GRID_COMPOSER_DRAW_TASK_PRIO,
                                              &grid_composer_instance.draw_update_task_handle,
                                              GRID_COMPOSER_DRAW_TASK_CORE) == pdTRUE,
                      ESP_ERR_NO_MEM, TAG, "failed to create draw update task, not enough memory");

  *out_composer = &grid_composer_instance;
  return ret;
}
esp_err_t
grid_composer_del(grid_composer_handle composer) {
  ESP_RETURN_ON_FALSE(composer, ESP_ERR_INVALID_ARG, TAG, "invalid composer");

  for (int r = 0; r < GRID_COMPOSER_CELL_MAX_ROWS; ++r) {
    for (int c = 0; c < GRID_COMPOSER_CELL_MAX_COLS; ++c) {
      struct grid_composer_cell_dev *cell = composer->cell_devices[r][c];
      if (cell) {
        cell->state = DEV_STATE_INVALID;
        cell->renderer.clear = NULL;
        cell->renderer.draw_figure = NULL;
        cell->renderer.draw_text = NULL;
        cell->renderer.user_ctx = NULL;

        free(cell);
        composer->cell_devices[r][c] = NULL;
      }
    }
  }

  if (composer->draw_queue) {
    vQueueDelete(composer->draw_queue);
    composer->draw_queue = NULL;
  }

  if (composer->draw_update_task_handle) {
    vTaskDelete(composer->draw_update_task_handle);
    composer->draw_update_task_handle = NULL;
  }

  return ESP_OK;
}

esp_err_t
grid_composer_draw_queue_send(grid_composer_handle composer, const grid_composer_draw_descriptor *draw_desc,
                              uint32_t timeout_ms) {
  esp_err_t ret = ESP_OK;

  ESP_RETURN_ON_FALSE(xQueueSend(composer->draw_queue, draw_desc, pdMS_TO_TICKS(timeout_ms)) == pdTRUE, ESP_FAIL, TAG,
                      "failed to send a render descriptor to the render queue");

  return ret;
}

esp_err_t
grid_composer_add_cell(grid_composer_handle composer, grid_composer_renderer *renderer) {
  esp_err_t ret = ESP_OK;
  ESP_RETURN_ON_FALSE(composer && renderer, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  ESP_RETURN_ON_FALSE(validate_renderer(renderer), ESP_ERR_INVALID_ARG, TAG, "invalid renderer object");

  struct grid_composer_cell_dev *cell_dev = calloc(1, sizeof(struct grid_composer_cell_dev));
  ESP_RETURN_ON_FALSE(cell_dev, ESP_ERR_NO_MEM, TAG, "failed to allocate a cell device");

  cell_dev->renderer = *renderer;
  cell_dev->state = DEV_STATE_INITIALIZED;

  ESP_GOTO_ON_ERROR(push_cell(composer, cell_dev), err, TAG, "failed to add a cell device, make sure there is enough space");

  return ESP_OK;
err:
  if (cell_dev) {
    free(cell_dev);
  }
  return ret;
}

static void
task_draw_update(void *arg) {
  if (NULL == arg)
    return;

  struct grid_composer *grid_composer_inst = (struct grid_composer *)arg;

  grid_composer_draw_descriptor draw_desc;
  for (;;) {
    if (xQueueReceive(grid_composer_inst->draw_queue, &draw_desc, portMAX_DELAY) == pdTRUE) {
      if (draw_desc.cell_row >= GRID_COMPOSER_CELL_MAX_ROWS || draw_desc.cell_col >= GRID_COMPOSER_CELL_MAX_COLS) {
        ESP_LOGE(TAG, "invalid cell row or column idx");
        continue;
      }
      // SUGGESTION: Use locks?
      ESP_ERROR_CHECK_WITHOUT_ABORT(draw(grid_composer_inst->cell_devices[draw_desc.cell_row][draw_desc.cell_col],
                                         &draw_desc.draw_obj, draw_desc.clear_before));
    }
    // vTaskDelay(pdMS_TO_TICKS((uint32_t)(1000 / GRID_COMPOSER_DRAW_TASK_UPDATE_RATE)));
  }
}

static esp_err_t
draw(struct grid_composer_cell_dev *cell_dev, const grid_composer_draw_obj_info *draw_obj, bool clear_before) {
  ESP_RETURN_ON_FALSE(cell_dev && draw_obj, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  if (clear_before) {
    ESP_RETURN_ON_FALSE(cell_dev->renderer.clear, ESP_FAIL, TAG, "no clear method on renderer");
    ESP_RETURN_ON_ERROR(cell_dev->renderer.clear(cell_dev->renderer.user_ctx), TAG, "failed to clear a cell");
  }

  switch (draw_obj->content_type) {
  case GRID_COMPOSER_CONTENT_TEXT:
    ESP_RETURN_ON_FALSE(cell_dev->renderer.draw_text, ESP_FAIL, TAG, "no draw_text method on renderer");
    ESP_RETURN_ON_ERROR(
        cell_dev->renderer.draw_text(cell_dev->renderer.user_ctx, &draw_obj->text, draw_obj->color, draw_obj->fill), TAG,
        "failed to draw text");
    break;
  case GRID_COMPOSER_CONTENT_FIGURE:
    ESP_RETURN_ON_FALSE(cell_dev->renderer.draw_figure, ESP_FAIL, TAG, "no draw_figure method on renderer");
    ESP_RETURN_ON_ERROR(
        cell_dev->renderer.draw_figure(cell_dev->renderer.user_ctx, &draw_obj->figure, draw_obj->color, draw_obj->fill), TAG,
        "failed to draw a figure");
    break;
  case GRID_COMPOSER_CONTENT_CLEAR:
    ESP_RETURN_ON_FALSE(cell_dev->renderer.clear, ESP_FAIL, TAG, "no clear method on renderer");
    ESP_RETURN_ON_ERROR(cell_dev->renderer.clear(cell_dev->renderer.user_ctx), TAG, "failed to clear a cell");
    break;
  default:
    return ESP_ERR_INVALID_ARG;
  }

  return ESP_OK;
}

static esp_err_t
push_cell(grid_composer_handle composer, struct grid_composer_cell_dev *cell_dev) {
  ESP_RETURN_ON_FALSE(composer && cell_dev, ESP_ERR_INVALID_ARG, TAG, "invalid args");

  for (uint8_t r = 0; r < GRID_COMPOSER_CELL_MAX_ROWS; ++r) {
    for (uint8_t c = 0; c < GRID_COMPOSER_CELL_MAX_COLS; ++c) {
      if (composer->cell_devices[r][c] == NULL) {
        composer->cell_devices[r][c] = cell_dev;
        ESP_LOGI(TAG, "Added sell at pos: %u %u", r, c);
        return ESP_OK;
      }
    }
  }

  ESP_LOGW(TAG, "no empty slot to push panel");
  return ESP_ERR_NO_MEM;
}

static bool
validate_renderer(const grid_composer_renderer *renderer) {
  return renderer->clear && renderer->draw_figure && renderer->draw_text;
}

// esp_err_t
// grid_composer_pop_cell(grid_composer_handle composer) {
//   ESP_RETURN_ON_FALSE(composer, ESP_ERR_INVALID_ARG, TAG, "invalid composer");

//   for (int r = GRID_COMPOSER_CELL_MAX_ROWS - 1; r >= 0; --r) {
//     for (int c = GRID_COMPOSER_CELL_MAX_COLS - 1; c >= 0; --c) {
//       if (composer->cell_devices[r][c].state != DEV_STATE_INVALID) {
//         composer->cell_devices[r][c].state = DEV_STATE_INVALID;
//         composer->cell_devices[r][c].renderer.clear = NULL;
//         composer->cell_devices[r][c].renderer.draw_figure = NULL;
//         composer->cell_devices[r][c].renderer.draw_text = NULL;
//         composer->cell_devices[r][c].renderer.user_ctx = NULL;
//         return ESP_OK;
//       }
//     }
//   }

//   ESP_LOGW(TAG, "no panel to pop");
//   return ESP_FAIL;
// }
