#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*FIXME Should be part of the config*/
#define CONFIG_MEASUREMENT_TASK_HEAP_DEBUG 0
#if CONFIG_MEASUREMENT_TASK_HEAP_DEBUG
#include "esp_heap_caps.h"
#endif
#include "esp_log.h"

#include "MeasurementTask.h"

#include "MeasurementDatatypes.h"
#include "MeasurementModuleErrors.h"

static int findFirstSetBit(uint32_t num);

MeasurementTask::MeasurementTask()
    : EventProducer<MeasurementTaskEvent>(MEASUREMENT_TASK_TOTAL_EVENTS) {
  this->measurementTaskFunc = [this](void *pvParameter) -> void * {
    return this->defaultMeasurementTask(pvParameter);
  };

  this->eventloopConfig = DEFAULT_EVENTLOOP_CONFIG;
  this->measurementTaskConfig = DEFAULT_MEASUREMENT_TASK_CONFIG;
}

esp_err_t MeasurementTask::configure(
    const MeasurementTaskConfig *taskConfig,
    const MeasurementTaskEventloopConfig *eventloopConfig) {
  if (!taskConfig || !eventloopConfig)
    return ESP_ERR_INVALID_ARG;

  if (taskConfig->pollingRate == 0)
    return ESP_ERR_INVALID_POLLING_RATE;
  // TODO - Implement checks
  //  if (taskConfig->pcName)
  //    return ESP_ERR_INVALID_PC_NAME;
  //  if (taskConfig->usStackDepth)
  //    return ESP_ERR_INVALID_STACK_DEPTH;
  //  if (taskConfig->uxPriority)
  //    return ESP_ERR_INVALID_PRIORITY;
  //  if (taskConfig->xCoreID)
  //    return ESP_ERR_INVALID_CORE_ID;

  this->measurementTaskConfig = *taskConfig;
  this->eventloopConfig = *eventloopConfig;

  BaseType_t result = xTaskCreatePinnedToCore(
      MeasurementTask::measurementTaskWrapper,
      this->measurementTaskConfig.pcName,
      this->measurementTaskConfig.usStackDepth,
      this,
      this->measurementTaskConfig.uxPriority,
      &this->_measurementTaskHandle,
      this->measurementTaskConfig.xCoreID);

  if (!result)
    return ESP_ERR_TASK_CREATE_FAIL;

  this->isConfigured = true;

  return ESP_OK;
};

/**
 * @brief Start, start the measurement after the configuration.
 *
 */
esp_err_t MeasurementTask::start() {
  if (this->isRunning)
    return ESP_ERR_IS_RUNNING;
  if (!this->isConfigured)
    return ESP_ERR_TASK_NOT_CONFIGURED;
  if (this->eventConsumers.size() == 0)
    return ESP_ERR_NO_EVENT_HANDLERS;

  vTaskResume(this->_measurementTaskHandle);
  this->isRunning = true;

  return ESP_OK;
}

/**
 * @brief Stop, temporarily stop the execution.
 *
 */
esp_err_t MeasurementTask::stop() {
  if (!this->isRunning)
    return ESP_ERR_NOT_RUNNING;
  if (!this->isConfigured)
    return ESP_ERR_TASK_NOT_CONFIGURED;

  vTaskSuspend(this->_measurementTaskHandle);

  this->isRunning = false;

  return ESP_OK;
}

esp_err_t MeasurementTask::reset() {
  if (!this->isConfigured)
    return ESP_ERR_TASK_NOT_CONFIGURED;

  this->eventloopConfig = DEFAULT_EVENTLOOP_CONFIG;
  this->measurementTaskConfig = DEFAULT_MEASUREMENT_TASK_CONFIG;

  /*TODO - Clear event handlers */

  this->isConfigured = false;
  this->isRunning = false;

  vTaskDelete(this->_measurementTaskHandle);

  return ESP_OK;
}

esp_err_t MeasurementTask::setMeasurementTask(
    const std::function<void *(void *)> &taskFunction) {
  if (this->isRunning)
    return ESP_ERR_IS_RUNNING;

  this->measurementTaskFunc = taskFunction;

  return ESP_OK;
}

void MeasurementTask::periodicMeasurementTask(void *pvParameter) {
  vTaskSuspend(NULL);
  while (1) {
#if CONFIG_MEASUREMENT_TASK_HEAP_DEBUG
    size_t heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    ESP_LOGI(
        this->measurementTaskConfig.pcName,
        "Heap before measurement: %u",
        heap);

    uint32_t stackLeft = uxTaskGetStackHighWaterMark2(NULL);
    ESP_LOGI(
        this->measurementTaskConfig.pcName,
        "Stack before measurement: %lu",
        stackLeft);

#endif

#if CONFIG_MEASUREMENT_TASK_DYNAMIC_CAST
    MeasurementBase *data =
        dynamic_cast<MeasurementBase *>(this->measurementTaskFunc(pvParameter));
#else
    MeasurementBase *data =
        static_cast<MeasurementBase *>(this->measurementTaskFunc(pvParameter));
#endif

    esp_err_t res = EventProducer<MeasurementTaskEvent>::produceEvent(
        data,
        data->size,
        MEASUREMENT_TASK_DATA_UPDATE_EVENT,
        this->eventloopConfig.sendWaitTicks,
        this->eventloopConfig.accessWaitTicks,
        this->eventloopConfig.receiveWaitTicks);

#if CONFIG_MEASUREMENT_TASK_HEAP_DEBUG
    heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    ESP_LOGI(
        this->measurementTaskConfig.pcName, "Heap after measurement: %u", heap);

    stackLeft = uxTaskGetStackHighWaterMark2(NULL);
    ESP_LOGI(
        this->measurementTaskConfig.pcName,
        "Stack after measurement: %lu",
        stackLeft);

#endif

    vTaskDelay(pdMS_TO_TICKS(1000 / this->measurementTaskConfig.pollingRate));

    delete data;
  };

  return;
}

esp_err_t MeasurementTask::registerEventConsumer(
    EventConsumer<MeasurementTaskEvent> *eventConsumer,
    MeasurementTaskEvent event) {

  if (this->isRunning)
    return ESP_ERR_IS_RUNNING;
  if (!this->isConfigured)
    return ESP_ERR_TASK_NOT_CONFIGURED;

  esp_err_t res = EventProducer<MeasurementTaskEvent>::registerEventConsumer(
      eventConsumer, event);

  return res;
}

esp_err_t MeasurementTask::unregisterEventConsumer(
    EventConsumer<MeasurementTaskEvent> *eventConsumer,
    MeasurementTaskEvent event) {

  if (this->isRunning)
    return ESP_ERR_IS_RUNNING;
  if (!this->isConfigured)
    return ESP_ERR_TASK_NOT_CONFIGURED;

  esp_err_t res = EventProducer<MeasurementTaskEvent>::unregisterEventConsumer(
      eventConsumer, event);

  return res;
}

void *MeasurementTask::defaultMeasurementTask(void *pvParameter) {
  using defaultData = MeasurementTaskData<DefaultData>;

  defaultData *def = new defaultData();

  return def;
}
