#pragma once
#ifndef MEASUREMENTMODULE_H
#define MEASUREMENTMODULE_H

#include <vector>

#include "MeasurementTask.h"

class MeasurementModule {
public:
  MeasurementModule();
  ~MeasurementModule();

  /**
   * @brief Configure the module.
   *
   */
  void configure();

  /**
   * @brief Start, start the measurement after the configuration.
   *
   */
  void start();

  /**
   * @brief Stop, temporarily stop the execution.
   *
   */
  void stop();

  /**
   * @brief Terminate, free all resources.
   *
   */
  void terminate();

  void addMeasurementTask(MeasurementTask *measurementTask);

  void removeMeasurementTask(MeasurementTask *measurementTask);

private:
  std::vector<MeasurementTask *> tasks{};
};

#endif