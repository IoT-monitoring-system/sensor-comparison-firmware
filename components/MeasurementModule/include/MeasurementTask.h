#pragma once
#ifndef MEASUREMENTTASK_H
#define MEASUREMENTTASK_H

#include <functional>

#include "EventSystem.hpp"

#include "Datatypes.h"
#include "MeasurementDatatypes.h"

class MeasurementModule;

class MeasurementTask : public EventProducer<measurement_task_event> {
  friend class MeasurementModule;

public:
  MeasurementTask();
  ~MeasurementTask() = default;

  /**
   * @brief Configure the task.
   *
   */
  esp_err_t configure(
      const measurement_task_config *taskConfig,
      const measurement_task_eventloop_config *eventloopConfig);

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

  measurement_task_config getTaskConfiguration();
  measurement_task_eventloop_config getEventloopConfiguration();

  esp_err_t registerEventHandler(
      EventConsumer<measurement_task_event> *eventConsumer,
      measurement_task_event event);
  esp_err_t unregisterEventHandler(
      EventConsumer<measurement_task_event> *eventConsumer,
      measurement_task_event event);

  // Sensor *getSensor();

private:
  bool isConfigured = false;
  bool isRunning = false;

  // std::unordered_map<measurement_task_event, std::vector<EventHandler *>>
  //     eventHandlers;
  // std::unordered_map<measurement_task_event, EventGroupHandle_t> eventGroups;

  // Sensor *sensor;
  measurement_task_config measurementTaskConfig;
  measurement_task_eventloop_config eventloopConfig;

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