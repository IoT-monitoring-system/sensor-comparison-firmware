#include <numeric>

#include "MeasurementModule.h"

#include "MeasurementModuleErrors.h"

esp_err_t MeasurementModule::configure() {
  for (auto &mTask : this->tasks) {
    if (!mTask->isConfigured)
      return ESP_ERR_TASK_NOT_CONFIGURED;
  }
  return ESP_OK;
};

esp_err_t MeasurementModule::start() {
  for (auto &mTask : this->tasks) {
    esp_err_t res = mTask->start();
    if (res != ESP_OK)
      return res;
  }
  return ESP_OK;
};

esp_err_t MeasurementModule::stop() {
  for (auto &mTask : this->tasks) {
    esp_err_t res = mTask->stop();
    if (res != ESP_OK)
      return res;
  }
  return ESP_OK;
};

esp_err_t MeasurementModule::startMT(const std::string &mtName) {
  auto it = std::find_if(
      this->tasks.begin(),
      this->tasks.end(),
      [&](const std::unique_ptr<MeasurementTask> &t) {
        return t->measurementTaskConfig.pcName == mtName;
      });

  if (it == this->tasks.end())
    return ESP_ERR_TASK_NOT_FOUND;

  return (*it)->start();
}

esp_err_t MeasurementModule::stopMT(const std::string &mtName) {
  auto it = std::find_if(
      this->tasks.begin(),
      this->tasks.end(),
      [&](const std::unique_ptr<MeasurementTask> &t) {
        return t->measurementTaskConfig.pcName == mtName;
      });

  if (it == this->tasks.end())
    return ESP_ERR_TASK_NOT_FOUND;

  return (*it)->stop();
}

MeasurementTask *MeasurementModule::addMT(
    MeasurementFunction measFunc,
    const MeasurementTaskConfig &cfg) {

  esp_err_t err = ESP_OK;
  if (!measFunc)
    return nullptr;

  for (auto &mTask : this->tasks) {
    if (cfg.pcName == mTask->measurementTaskConfig.pcName)
      return nullptr;
  }

  auto newMeasurementTask = std::make_unique<MeasurementTask>();
  err = newMeasurementTask->configure(cfg);
  if (err != ESP_OK)
    return nullptr;

  err = newMeasurementTask->setMeasurementTask(measFunc);
  if (err != ESP_OK)
    return nullptr;

  this->tasks.push_back(std::move(newMeasurementTask));

  return this->tasks.back().get();
}

esp_err_t MeasurementModule::removeMT(const std::string &mtName) {
  auto it = std::find_if(
      this->tasks.begin(),
      this->tasks.end(),
      [&](const std::unique_ptr<MeasurementTask> &t) {
        return t->measurementTaskConfig.pcName == mtName;
      });

  if (it == this->tasks.end())
    return ESP_ERR_TASK_NOT_FOUND;

  this->tasks.erase(it);

  return ESP_OK;
};

MeasurementTask *MeasurementModule::getMTByName(const std::string &mtName) {
  auto it = std::find_if(
      this->tasks.begin(),
      this->tasks.end(),
      [&](const std::unique_ptr<MeasurementTask> &t) {
        return t->measurementTaskConfig.pcName == mtName;
      });

  if (it == this->tasks.end())
    return nullptr;

  return it->get();
}

uint32_t MeasurementModule::getTotalNumTasks() { return this->tasks.size(); };
uint32_t MeasurementModule::getNumTasksRunning() {
  uint32_t tasksRunning = std::accumulate(
      this->tasks.begin(),
      this->tasks.end(),
      0,
      [](uint32_t acc, const auto &mT) { return acc + mT->isRunning; });

  return tasksRunning;
};
