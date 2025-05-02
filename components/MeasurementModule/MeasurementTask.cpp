#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "MeasurementTask.h"

#include "MeasurementDatatypes.h"
#include "MeasurementModuleErrors.h"

static const char *TAG_MEASUREMENT_TASK = "MeasurementTask";

static int findFirstSetBit(uint32_t num);

MeasurementTask::MeasurementTask()
    : EventProducer<MeasurementTaskEvent>(
          MEASUREMENT_TASK_TOTAL_EVENTS,
          DEFAULT_ESWAITS_CONFIG) {

  // this->measurementTaskFunc = this->defaultMeasurementTask;

  this->measurementTaskConfig = DEFAULT_MEASUREMENT_TASK_CONFIG;
}

esp_err_t MeasurementTask::configure(const MeasurementTaskConfig &taskConfig) {
  if (taskConfig.sampleRate <= 0)
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

  this->measurementTaskConfig = taskConfig;

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
    return ESP_OK;
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
    return ESP_OK;
  if (!this->isConfigured)
    return ESP_ERR_TASK_NOT_CONFIGURED;

  esp_err_t err = ESP_OK;

  if (hooks[MT_PRE_STOP_HOOK])
    err = hooks[MT_PRE_STOP_HOOK]();
  if (err != ESP_OK)
    return err;

  vTaskSuspend(this->_measurementTaskHandle);
  ESP_LOGI(TAG_MEASUREMENT_TASK, "Suspended a task");
  this->isRunning = false;

  if (hooks[MT_POST_STOP_HOOK])
    err = hooks[MT_POST_STOP_HOOK]();
  if (err != ESP_OK)
    return err;

  return ESP_OK;
}

esp_err_t MeasurementTask::reset() {
  if (!this->isConfigured)
    return ESP_ERR_TASK_NOT_CONFIGURED;

  this->measurementTaskConfig = DEFAULT_MEASUREMENT_TASK_CONFIG;

  /*TODO - Clear event handlers */

  this->isConfigured = false;
  this->isRunning = false;

  vTaskDelete(this->_measurementTaskHandle);

  return ESP_OK;
}

esp_err_t MeasurementTask::setSamplingRate(float sampleRate) {
  esp_err_t err = ESP_OK;

  if (sampleRate < 0)
    return ESP_ERR_INVALID_POLLING_RATE;

  this->measurementTaskConfig.sampleRate = sampleRate;

  return err;
}

const MeasurementTaskConfig &MeasurementTask::getConfiguration() {
  return this->measurementTaskConfig;
}

esp_err_t MeasurementTask::setMeasurementTask(
    MeasurementFunction taskFunction) {
  if (this->isRunning)
    return ESP_ERR_IS_RUNNING;

  this->measurementTaskFunc = taskFunction;

  return ESP_OK;
}

void MeasurementTask::periodicMeasurementTask(void *pvParameter) {
  vTaskSuspend(NULL);
  MeasurementProducerHelper helper(this);

  esp_err_t err = ESP_OK;

  while (1) {
    err = this->measurementTaskFunc(helper);

    if (err != ESP_OK) {
      ESP_LOGE(TAG_MEASUREMENT_TASK, "Failed to produce event: %u", err);
      return;
    }
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

esp_err_t MeasurementTask::registerHook(
    MTHook hook,
    MeasurementTaskLifecycleHook hookType) {
  if (hookType >= MT_TOTAL_HOOKS || hook == nullptr)
    return ESP_ERR_INVALID_ARG;

  hooks[hookType] = hook;

  return ESP_OK;
}

esp_err_t MeasurementTask::unregisterHook(
    MeasurementTaskLifecycleHook hookType) {
  if (hookType >= MT_TOTAL_HOOKS)
    return ESP_ERR_INVALID_ARG;

  hooks[hookType] = nullptr;

  return ESP_OK;
}

esp_err_t MeasurementProducerHelper::produceMeasurement(
    void *data,
    size_t size) {

  const MeasurementTaskConfig mtCfg = this->mt->getConfiguration();

  esp_err_t err = this->mt->produceEvent(
      data,
      size,
      MEASUREMENT_TASK_DATA_UPDATE_EVENT,
      mtCfg.receiveWaitTicks,
      mtCfg.sendWaitTicks,
      mtCfg.accessWaitTicks);

  vTaskDelay(pdMS_TO_TICKS(1000 / mtCfg.sampleRate));

  return err;
}
