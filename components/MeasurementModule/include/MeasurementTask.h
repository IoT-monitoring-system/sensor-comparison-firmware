#pragma once
#ifndef MEASUREMENTTASK_H
#define MEASUREMENTTASK_H

#include <functional>

#include "EventSystem.hpp"

#include "MeasurementModuleDatatypes.h"

class MeasurementModule;

class MeasurementTask : public EventProducer<MeasurementTaskEvent> {
  friend class MeasurementModule;

public:
  MeasurementTask();
  ~MeasurementTask() = default;

  /**
   * @brief Configure the task.
   *
   */
  esp_err_t configure(
      const MeasurementTaskConfig *taskConfig,
      const MeasurementTaskEventloopConfig *eventloopConfig);

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

  esp_err_t setMeasurementTask(
      const std::function<void *(void *)> &taskFunction);

  MeasurementTaskConfig getTaskConfiguration();
  MeasurementTaskEventloopConfig getEventloopConfiguration();

  esp_err_t registerEventConsumer(
      EventConsumer<MeasurementTaskEvent> *eventConsumer,
      MeasurementTaskEvent event);
  esp_err_t unregisterEventConsumer(
      EventConsumer<MeasurementTaskEvent> *eventConsumer,
      MeasurementTaskEvent event);

private:
  bool isConfigured = false;
  bool isRunning = false;

  MeasurementTaskConfig measurementTaskConfig;
  MeasurementTaskEventloopConfig eventloopConfig;

  TaskHandle_t _measurementTaskHandle = NULL;

  std::function<void *(void *)> measurementTaskFunc;
  void *defaultMeasurementTask(void *pvParameters);

  void periodicMeasurementTask(void *pvParameters);

  static void measurementTaskWrapper(void *pvParameters) {
    MeasurementTask *instance = static_cast<MeasurementTask *>(pvParameters);
    instance->periodicMeasurementTask(pvParameters);
  }
};

#endif