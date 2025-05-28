/*FIXME Should be part of the config*/
#define CONFIG_MEASUREMENT_TASK_HEAP_DEBUG 0
#if CONFIG_MEASUREMENT_TASK_HEAP_DEBUG
#include "esp_heap_caps.h"
#endif

#include "StateManagementModule.h"

static const char *TAG_STATE_MANAGEMENT = "StateManagement";

StateManager *StateManager::instance_p = nullptr;
SemaphoreHandle_t StateManager::instance_mux = nullptr;
StaticSemaphore_t StateManager::instance_static_mux;

StateManager *StateManager::getInstance() {
  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
    ESP_LOGE(TAG_STATE_MANAGEMENT, "Scheduler not started.");
    return nullptr;
  }

  if (instance_mux == nullptr) {
    instance_mux = xSemaphoreCreateMutexStatic(&instance_static_mux);
    if (instance_mux == NULL) {
      ESP_LOGE(TAG_STATE_MANAGEMENT, "Mutex creation failed");
      return nullptr;
    }
  }

  if (xSemaphoreTake(
          instance_mux, pdMS_TO_TICKS(STATE_MANAGEMENT_INSTANCE_MUX_TIMEOUT)) ==
      pdFALSE) {
    ESP_LOGE(TAG_STATE_MANAGEMENT, "getInstance() timeout%s", "");
    return nullptr;
  }

  if (instance_p == nullptr) {
    instance_p = new StateManager();
  }

  xSemaphoreGive(instance_mux);

  return instance_p;
}

void StateManager::registerState(
    AppState state,
    StateFunc enter,
    StateFunc exit) {
  stateHandlers[state] = StateHandler{state, enter, exit};
}

esp_err_t StateManager::requestTransition(AppState state) {
  if (state >= STATE_COUNT) {
    ESP_LOGE(TAG_STATE_MANAGEMENT, "Unknown state requested");
    return ESP_ERR_SM_STATE_INVALID;
  }

  if (xSemaphoreTake(
          stateTransitionMutex,
          pdMS_TO_TICKS(STATE_MANAGEMENT_TRANSITION_MUX_TIMEOUT)) == pdFALSE) {
    ESP_LOGE(TAG_STATE_MANAGEMENT, "rT, State transition mutex timeout%s", "");
    return ESP_ERR_SM_TRANSITION_TIMEOUT;
  }

  if (!transitionHandled) {
    ESP_LOGE(TAG_STATE_MANAGEMENT, "Unhandled state pending");
    return ESP_ERR_SM_TRANSITION_PENDING;
  }

  nextState = state;
  transitionHandled = false;
  xSemaphoreGive(stateTransitionMutex);

  return ESP_OK;
}

void StateManager::runSmTask(void *pvParameter) {
  ESP_LOGI(TAG_STATE_MANAGEMENT, "Initialized state transition mutex");
  while (1) {
#if CONFIG_MEASUREMENT_TASK_HEAP_DEBUG
    multi_heap_info_t heap_info;
    heap_caps_get_info(&heap_info, MALLOC_CAP_DEFAULT);

    ESP_LOGI("HeapInfo", "Total free bytes: %d", heap_info.total_free_bytes);
    ESP_LOGI(
        "HeapInfo", "Largest free block: %d", heap_info.largest_free_block);
    ESP_LOGI(
        "HeapInfo",
        "Minimum free bytes ever: %d",
        heap_info.minimum_free_bytes);
    ESP_LOGI(
        "HeapInfo", "Total allocated blocks: %d", heap_info.allocated_blocks);
    ESP_LOGI("HeapInfo", "Total free blocks: %d", heap_info.free_blocks);
#endif
    if (nextState != currentState) {
      ESP_LOGI(
          TAG_STATE_MANAGEMENT,
          "Current State: %i; Entering: %i",
          currentState,
          nextState);

      if (stateHandlers[currentState].onExit) {
        ESP_LOGI(TAG_STATE_MANAGEMENT, "Executing onExit state handler");
        exitStateError = stateHandlers[currentState].onExit();
      }
      if (exitStateError != ESP_OK) {
        xTaskNotifyGive(stateTransitionErrorTaskHandle);
      }

      currentState = nextState;

      if (stateHandlers[currentState].onEnter) {
        ESP_LOGI(TAG_STATE_MANAGEMENT, "Executing onEnter state handler");
        enterStateError = stateHandlers[currentState].onEnter();
      }
      if (enterStateError != ESP_OK) {
        xTaskNotifyGive(stateTransitionErrorTaskHandle);
      }

      if (xSemaphoreTake(
              stateTransitionMutex,
              pdMS_TO_TICKS(STATE_MANAGEMENT_TRANSITION_MUX_TIMEOUT)) ==
          pdFALSE) {
        ESP_LOGE(
            TAG_STATE_MANAGEMENT, "smT, State transition mutex timeout%s", "");
        return;
      }
      transitionHandled = true;
      xSemaphoreGive(stateTransitionMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(1000U / STATE_MANAGEMENT_STATE_POLL_FREQ));
  }
}

void StateManager::runStateTransitionErrorTask(void *pvParameter) {
  bool taskWoken = false;

  while (1) {
    if (!taskWoken) {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      taskWoken = true;
    }
    ESP_LOGE(
        TAG_STATE_MANAGEMENT,
        "State transition failed, from: %i --> to: %i",
        currentState,
        nextState);
    ESP_LOGE(
        TAG_STATE_MANAGEMENT,
        "Error codes: \nEnter: %i; Exit: %i",
        enterStateError,
        exitStateError);

    vTaskSuspend(smTaskHandle);
    vTaskDelay(pdMS_TO_TICKS(
        1000U / STATE_MANAGEMENT_STATE_TRANSITION_ERROR_POLL_FREQ));
  }
}

void StateManager::smTask(void *pvParameter) {
  StateManager *self = static_cast<StateManager *>(pvParameter);
  self->runSmTask(pvParameter);
}

void StateManager::stateTransitionErrorTask(void *pvParameter) {
  StateManager *self = static_cast<StateManager *>(pvParameter);
  self->runStateTransitionErrorTask(pvParameter);
}

void StateManager::run() {
  stateTransitionMutex = xSemaphoreCreateMutex();
  if (stateTransitionMutex == NULL) {
    ESP_LOGE(
        TAG_STATE_MANAGEMENT,
        "State transition mutex initialization failed%s",
        "");
    return;
  }

  BaseType_t res = xTaskCreate(
      &stateTransitionErrorTask,
      "SMErrorTask",
      4096,
      this,
      STATE_MANAGEMENT_STATE_TRANSITION_ERROR_TASK_PRIO,
      &stateTransitionErrorTaskHandle);
  if (res == pdFAIL) {
    ESP_LOGE(
        TAG_STATE_MANAGEMENT,
        "stateTransitionErrorTask initialization failed%s",
        "");
    return;
  }

  res = xTaskCreate(
      &smTask, "SMTask", 4096, this, STATE_MANAGEMENT_TASK_PRIO, &smTaskHandle);
  if (res == pdFAIL) {
    ESP_LOGE(TAG_STATE_MANAGEMENT, "smTask initialization failed%s", "");
    return;
  }
}
