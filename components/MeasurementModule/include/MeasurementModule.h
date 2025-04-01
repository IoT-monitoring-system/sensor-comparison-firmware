#pragma once
#ifndef MEASUREMENTMODULE_H
#define MEASUREMENTMODULE_H

#include <vector>

#include "MeasurementTask.h"

class MeasurementModule {
public:
  /**
   * @brief Configure the module.
   *
   */
  esp_err_t configure();

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

  esp_err_t addMeasurementTask(MeasurementTask *measurementTask);
  esp_err_t removeMeasurementTask(MeasurementTask *measurementTask);

  uint32_t getTotalNumTasks();
  uint32_t getNumTasksRunning();

private:
  std::vector<MeasurementTask *> tasks;
};

#endif