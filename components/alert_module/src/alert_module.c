#include "memory.h"

#include "esp_check.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "alert_module.h"
#include "private/alert_module_private.h"

struct alert_module_instance {
  TaskHandle_t task;
  uint64_t task_period_ms;

  struct alert_internal alerts[ALERT_MODULE_MAX_ALERTS];
};

static char const *TAG = "alert_module";

static struct alert_module_instance alert_module_instance_global;

static void
alert_module_task(void *arg);

static esp_err_t
set_alert_state(struct alert_internal *alert, enum alert_state new_state);

static bool
notify_alert_state(const struct alert_internal *alert, uint32_t notify_mask);

esp_err_t
alert_module_init(const struct alert_module_config *module_cfg, alert_module_handle *out_module) {
  esp_err_t ret = ESP_OK;
  ESP_RETURN_ON_FALSE(module_cfg && out_module, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  struct alert_module_instance *alert_module = &alert_module_instance_global;
  for (uint8_t i = 0; i < ALERT_MODULE_MAX_ALERTS; i++) {
    alert_module->alerts[i].state = ALERT_STATE_INVALID;
    alert_module->alerts[i].access_mux = NULL;
    alert_module->alerts[i].state_change_handler = NULL;
  }
  ESP_RETURN_ON_FALSE(xTaskCreatePinnedToCore(alert_module_task, ALERT_MODULE_TASK_NAME, module_cfg->task_stack_size,
                                              alert_module, module_cfg->task_priority, &alert_module->task,
                                              module_cfg->task_core_affinity) == pdPASS,
                      ESP_ERR_NO_MEM, TAG, "failed to create alert task");

  alert_module->task_period_ms = module_cfg->task_period_ms;

  *out_module = alert_module;

  return ret;
}

esp_err_t
alert_module_del(alert_module_handle module) {
  esp_err_t ret = ESP_OK;
  ESP_RETURN_ON_FALSE(module, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  vTaskDelete(module->task);

  memset(module->alerts, 0, sizeof(module->alerts));

  // deallocate alert_module_instance if dynamically allocated

  return ret;
}

esp_err_t
alert_module_register_alert(alert_module_handle module, const struct alert_config *alert_cfg) {
  esp_err_t ret = ESP_OK;
  ESP_RETURN_ON_FALSE(module && alert_cfg, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  ESP_RETURN_ON_FALSE(alert_cfg->alert_id < ALERT_MODULE_MAX_ALERTS, ESP_ERR_INVALID_ARG, TAG,
                      "invalid argument, max alert id:%d", ALERT_MODULE_MAX_ALERTS);
  ESP_RETURN_ON_FALSE(module->alerts[alert_cfg->alert_id].state == ALERT_STATE_INVALID, ESP_ERR_INVALID_ARG, TAG,
                      "invalid argument, this alert_id is already registered, state:%d",
                      module->alerts[alert_cfg->alert_id].state); // WARNING: Carefull here, race condition possible
                                                                  // if already initialized?

  module->alerts[alert_cfg->alert_id].access_mux = xSemaphoreCreateMutex();
  ESP_RETURN_ON_FALSE(module->alerts[alert_cfg->alert_id].access_mux, ESP_ERR_NO_MEM, TAG,
                      "failed to create access_mux for an alert");

  ESP_RETURN_ON_FALSE(
      xSemaphoreTake(module->alerts[alert_cfg->alert_id].access_mux, pdMS_TO_TICKS(ALERT_MODULE_ACCESS_MUX_TIMEOUT_MS)) == pdTRUE,
      ESP_FAIL, TAG, "failed to acquire access_mux for an alert");

  module->alerts[alert_cfg->alert_id].alert_id = alert_cfg->alert_id;
  module->alerts[alert_cfg->alert_id].auto_expire = alert_cfg->auto_expire;
  module->alerts[alert_cfg->alert_id].expiry_time_ms = alert_cfg->expiry_time_ms;
  module->alerts[alert_cfg->alert_id].state_change_handler = alert_cfg->state_change_handler;
  module->alerts[alert_cfg->alert_id].state = ALERT_STATE_NORMAL;
  module->alerts[alert_cfg->alert_id].alert_notify_mask = alert_cfg->alert_notify_mask;

  strncpy(module->alerts[alert_cfg->alert_id].name, alert_cfg->name, ALERT_MODULE_NAME_MAX_LEN - 1);
  module->alerts[alert_cfg->alert_id].name[ALERT_MODULE_NAME_MAX_LEN - 1] = '\0';

  xSemaphoreGive(module->alerts[alert_cfg->alert_id].access_mux);

  return ret;
}
esp_err_t
alert_module_unregister_alert(alert_module_handle module, uint8_t alert_id) {
  esp_err_t ret = ESP_OK;
  ESP_RETURN_ON_FALSE(module, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  ESP_RETURN_ON_FALSE(alert_id < ALERT_MODULE_MAX_ALERTS, ESP_ERR_INVALID_ARG, TAG, "invalid argument, max alert id:%d",
                      ALERT_MODULE_MAX_ALERTS);

  ESP_RETURN_ON_FALSE(module->alerts[alert_id].state != ALERT_STATE_INVALID, ESP_ERR_INVALID_ARG, TAG,
                      "invalid argument, this alert_id is not registered"); // WARNING: Carefull here, race condition possible
                                                                            // if already initialized?
  ESP_RETURN_ON_FALSE(xSemaphoreTake(module->alerts[alert_id].access_mux, pdMS_TO_TICKS(ALERT_MODULE_ACCESS_MUX_TIMEOUT_MS)) ==
                          pdTRUE,
                      ESP_FAIL, TAG, "failed to acquire access_mux for an alert");

  module->alerts[alert_id].auto_expire = false;
  module->alerts[alert_id].expiry_time_ms = 0U;
  module->alerts[alert_id].state_change_handler = NULL;
  module->alerts[alert_id].state = ALERT_STATE_INVALID;
  module->alerts[alert_id].alert_notify_mask = 0U;

  xSemaphoreGive(module->alerts[alert_id].access_mux);

  return ret;
}

esp_err_t
alert_module_set_alert(alert_module_handle module, uint8_t alert_id) {
  esp_err_t ret = ESP_OK;
  ESP_RETURN_ON_FALSE(module, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  ESP_RETURN_ON_FALSE(alert_id < ALERT_MODULE_MAX_ALERTS, ESP_ERR_INVALID_ARG, TAG, "invalid argument, max alert id:%d",
                      ALERT_MODULE_MAX_ALERTS);
  ESP_RETURN_ON_FALSE(module->alerts[alert_id].state != ALERT_STATE_INVALID, ESP_ERR_INVALID_ARG, TAG,
                      "invalid argument, this alert_id is not registered");

  ESP_RETURN_ON_FALSE(xSemaphoreTake(module->alerts[alert_id].access_mux, pdMS_TO_TICKS(ALERT_MODULE_ACCESS_MUX_TIMEOUT_MS)) ==
                          pdTRUE,
                      ESP_FAIL, TAG, "failed to acquire access_mux for an alert");
  ESP_RETURN_ON_ERROR(set_alert_state(&module->alerts[alert_id], ALERT_STATE_SET), TAG, "failed to set state for an alert");
  xSemaphoreGive(module->alerts[alert_id].access_mux);

  return ret;
}
esp_err_t
alert_module_reset_alert(alert_module_handle module, uint8_t alert_id) {
  esp_err_t ret = ESP_OK;
  ESP_RETURN_ON_FALSE(module, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  ESP_RETURN_ON_FALSE(alert_id < ALERT_MODULE_MAX_ALERTS, ESP_ERR_INVALID_ARG, TAG, "invalid argument, max alert id:%d",
                      ALERT_MODULE_MAX_ALERTS);
  ESP_RETURN_ON_FALSE(module->alerts[alert_id].state != ALERT_STATE_INVALID, ESP_ERR_INVALID_ARG, TAG,
                      "invalid argument, this alert_id is not registered");

  ESP_RETURN_ON_FALSE(xSemaphoreTake(module->alerts[alert_id].access_mux, pdMS_TO_TICKS(ALERT_MODULE_ACCESS_MUX_TIMEOUT_MS)) ==
                          pdTRUE,
                      ESP_FAIL, TAG, "failed to acquire access_mux for an alert");
  ESP_RETURN_ON_ERROR(set_alert_state(&module->alerts[alert_id], ALERT_STATE_NORMAL), TAG, "failed to set state for an alert");
  xSemaphoreGive(module->alerts[alert_id].access_mux);

  return ret;
}

static void
alert_module_task(void *arg) {
  ESP_RETURN_VOID_ON_FALSE(arg, TAG, "invalid argument");
  struct alert_module_instance *alert_module = (struct alert_module_instance *)arg;
  for (;;) {
    for (uint8_t i = 0; i < ALERT_MODULE_MAX_ALERTS; ++i) {
      if (alert_module->alerts[i].access_mux == NULL) {
        continue;
      }

      if (xSemaphoreTake(alert_module->alerts[i].access_mux, pdMS_TO_TICKS(ALERT_MODULE_ACCESS_MUX_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "failed to acquire access_mux for alert:%u", i);
        continue;
      }

      switch (alert_module->alerts[i].state) {
      case ALERT_STATE_NORMAL: {
        notify_alert_state(&alert_module->alerts[i], ALERT_NOTIFY_NORMAL);
      } break;

      case ALERT_STATE_SET: {
        if (alert_module->alerts[i].auto_expire) {
          uint64_t time_diff_ms = ((uint64_t)esp_timer_get_time() - alert_module->alerts[i].state_change_time_us) / 1000ULL;

          if (time_diff_ms >= alert_module->alerts[i].expiry_time_ms) {
            set_alert_state(&alert_module->alerts[i], ALERT_STATE_EXPIRED);
            break;
          }
        }

        bool handled = notify_alert_state(&alert_module->alerts[i], ALERT_NOTIFY_SET);
        set_alert_state(&alert_module->alerts[i], handled ? ALERT_STATE_NORMAL : ALERT_STATE_SET);
      } break;

      case ALERT_STATE_EXPIRED: {
        bool handled = notify_alert_state(&alert_module->alerts[i], ALERT_NOTIFY_EXPIRED);
        set_alert_state(&alert_module->alerts[i], handled ? ALERT_STATE_NORMAL : ALERT_STATE_EXPIRED);
      }
      case ALERT_STATE_INVALID:
      default: {
      }
      }
      xSemaphoreGive(alert_module->alerts[i].access_mux);
    }

    vTaskDelay(pdMS_TO_TICKS(alert_module->task_period_ms));
  }
};

static bool
notify_alert_state(const struct alert_internal *alert, uint32_t notify_mask) {
  if (alert->alert_notify_mask & notify_mask) {
    if (NULL == alert->state_change_handler)
      return false;

    struct alert_descriptor alert_desc = {
        .alert_id = alert->alert_id,
        .state = alert->state,
        .timestamp_us = alert->state_change_time_us,
    };

    strncpy(alert_desc.name, alert->name, ALERT_MODULE_NAME_MAX_LEN - 1);
    alert_desc.name[ALERT_MODULE_NAME_MAX_LEN - 1] = '\0';

    return alert->state_change_handler(&alert_desc);
  }
  return false;
}

static esp_err_t
set_alert_state(struct alert_internal *alert, enum alert_state new_state) {
  esp_err_t ret = ESP_OK;
  ESP_RETURN_ON_FALSE(alert, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  alert->state = new_state;
  alert->state_change_time_us = (uint64_t)esp_timer_get_time();

  return ret;
}
