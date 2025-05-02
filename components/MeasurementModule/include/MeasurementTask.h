#pragma once
#ifndef MEASUREMENTTASK_H
#define MEASUREMENTTASK_H

// #include <functional>

#include "EventSystem.hpp"
#include "MeasurementDatatypes.h"

#include "MeasurementModuleDatatypes.h"

class MeasurementModule;
class MeasurementProducerHelper;

typedef esp_err_t (*MeasurementFunction)(MeasurementProducerHelper &);
typedef esp_err_t (*MTHook)(void);
// using MeasurementFunction = std::function<void(MeasurementProducerHelper &)>;

class MeasurementTask : public EventProducer<MeasurementTaskEvent> {
  friend class MeasurementModule;

public:
  MeasurementTask();
  ~MeasurementTask() = default;

  /**
   * @brief Configure the task.
   *
   */
  esp_err_t configure(const MeasurementTaskConfig &taskConfig);

  /**
   * @brief Start, start the measurement after the configuration.
   *
   */
  esp_err_t start();

  /**
   * @brief Stop, temporarily stop the execution.
   *
   */
  esp_err_t stop();

  /**
   * @brief Reset, free all resources, reset configuration.
   * ! FIX
   */
  esp_err_t reset();

  esp_err_t setMeasurementTask(MeasurementFunction taskFunction);
  esp_err_t setSamplingRate(float sampleRate);

  const MeasurementTaskConfig &getConfiguration();

  esp_err_t registerEventConsumer(
      EventConsumer<MeasurementTaskEvent> *eventConsumer,
      MeasurementTaskEvent event);
  esp_err_t unregisterEventConsumer(
      EventConsumer<MeasurementTaskEvent> *eventConsumer,
      MeasurementTaskEvent event);

  esp_err_t registerHook(MTHook hook, MeasurementTaskLifecycleHook hookType);
  esp_err_t unregisterHook(MeasurementTaskLifecycleHook hookType);

private:
  bool isConfigured = false;
  bool isRunning = false;

  MTHook hooks[MT_TOTAL_HOOKS]{};
  MeasurementTaskConfig measurementTaskConfig;

  TaskHandle_t _measurementTaskHandle = NULL;

  MeasurementFunction measurementTaskFunc;
  static void defaultMeasurementTask(MeasurementProducerHelper &producer);

  void periodicMeasurementTask(void *pvParameters);

  static void measurementTaskWrapper(void *pvParameters) {
    MeasurementTask *instance = static_cast<MeasurementTask *>(pvParameters);
    instance->periodicMeasurementTask(pvParameters);
  }
};

class MeasurementProducerHelper {
private:
  MeasurementTask *mt;

public:
  MeasurementProducerHelper(MeasurementTask *mt) : mt(mt) {}

  esp_err_t produceMeasurement(void *data, size_t size);
};

#endif