#include <numeric>

#include "MeasurementModule.h"

#include "MeasurementModuleErrors.h"

// MeasurementModule::MeasurementModule(){

// };

esp_err_t MeasurementModule::configure() {
  for (auto mTask : this->tasks) {
    if (!mTask->isConfigured)
      return ESP_ERR_TASK_NOT_CONFIGURED;
  }
  return ESP_OK;
};

esp_err_t MeasurementModule::start() {
  for (auto mTask : this->tasks) {
    esp_err_t res = mTask->start();
    if (res != ESP_OK)
      return res;
  }
  return ESP_OK;
};

esp_err_t MeasurementModule::stop() {
  for (auto mTask : this->tasks) {
    esp_err_t res = mTask->stop();
    if (res != ESP_OK)
      return res;
  }
  return ESP_OK;
};

esp_err_t MeasurementModule::addMeasurementTask(
    MeasurementTask *measurementTask) {
  if (!measurementTask)
    return ESP_ERR_INVALID_ARG;

  for (auto mTask : this->tasks) {
    if (measurementTask->measurementTaskConfig.pcName ==
        mTask->measurementTaskConfig.pcName)
      return ESP_ERR_TASK_NAME_TAKEN;
  }
  this->tasks.push_back(measurementTask);
  return ESP_OK;
};

esp_err_t MeasurementModule::removeMeasurementTask(
    MeasurementTask *measurementTask) {

  if (!measurementTask)
    return ESP_ERR_INVALID_ARG;

  auto it = std::find(this->tasks.begin(), this->tasks.end(), measurementTask);

  if (it == this->tasks.end())
    return ESP_ERR_TASK_NOT_FOUND;

  this->tasks.erase(it);

  return ESP_OK;
};

uint32_t MeasurementModule::getTotalNumTasks() { return this->tasks.size(); };
uint32_t MeasurementModule::getNumTasksRunning() {
  uint32_t tasksRunning = std::accumulate(
      this->tasks.begin(),
      this->tasks.end(),
      0,
      [](uint32_t acc, const auto &mT) { return acc + mT->isRunning; });

  return tasksRunning;
};
