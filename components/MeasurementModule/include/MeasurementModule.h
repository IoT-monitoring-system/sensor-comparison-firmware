#pragma once
#ifndef MEASUREMENTMODULE_H
#define MEASUREMENTMODULE_H

#include <memory>
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

  esp_err_t startMT(const std::string &mtName);
  esp_err_t stopMT(const std::string &mtName);

  MeasurementTask *getMTByName(const std::string &mtName);

  MeasurementTask *addMT(
      MeasurementFunction measFunc,
      const MeasurementTaskConfig &cfg);
  esp_err_t removeMT(const std::string &mtName);

  uint32_t getTotalNumTasks();
  uint32_t getNumTasksRunning();

private:
  std::vector<std::unique_ptr<MeasurementTask>> tasks;
};

#endif