#include "BME6xxScheduler.h"
#include "BME6xxManagerErrors.h"

#include "esp_log.h"

#ifdef ESP_PLATFORM
static uint64_t timestamp_millis() { return esp_timer_get_time() / 1000U; }
#endif

const uint8_t MAX_HEATER_DURATION = 200U;
const uint8_t GAS_WAIT_SHARED = 120U;
const uint8_t SENSOR_WAKE_UP_TIME_OFFSET = 10;

esp_err_t BME6xxScheduler::initialize() { return ESP_OK; }

esp_err_t BME6xxScheduler::resetScheduleData(BMEMngrSensor &sensor) {
  sensor.scheduleInfo.dutyCycleIndex = 0;
  sensor.scheduleInfo.heaterIndex = 0;
  sensor.scheduleInfo.wakeUpTime = 0;

  return ESP_OK;
}

bool BME6xxScheduler::selectNextSensor(
    std::vector<BMEMngrSensor> &sensors,
    BME6xxMode mode,
    uint64_t &wakeUpTime,
    uint8_t &num) {
  if (sensors.empty())
    return false;

  // Will be overwritten by a value that is < numSensors in case any sensor will
  // actually be scheduled
  num = sensors.size();

  for (uint8_t i = 0; i < sensors.size(); i++) {
    // ESP_LOGI(
    //     (char *)"BME6xxScheduler",
    //     "Considering sensor: %u, its index: %u; Wake Up Time: %llu; Time now: "
    //     "%llu",
    //     static_cast<uint8_t>(sensors[i].id),
    //     i,
    //     sensors[i].scheduleData.wakeUpTime,
    //     timestamp_millis());
    if ((sensors[i].config.mode == mode) &&
        (sensors[i].scheduleInfo.wakeUpTime < wakeUpTime)) {
      wakeUpTime = sensors[i].scheduleInfo.wakeUpTime;
      num = i;
    }
  }

  if (num < sensors.size()) {
    // ESP_LOGI(
    //     (char *)"BME6xxScheduler",
    //     "Decided on: %u, its index: %u; Wake Up Time: %llu; Time "
    //     "now: "
    //     "%llu",
    //     static_cast<uint8_t>(sensors[num].id),
    //     num,
    //     sensors[num].scheduleData.wakeUpTime,
    //     timestamp_millis());
    return true;
  }
  return false;
}

esp_err_t BME6xxScheduler::scheduleWakeUp(
    BMEMngrSensor &sensor,
    uint64_t wakeUpTime,
    uint8_t nextHeaterIndex) {

  sensor.scheduleInfo.heaterIndex = nextHeaterIndex;
  sensor.scheduleInfo.wakeUpTime = wakeUpTime;
  return ESP_OK;
}
esp_err_t BME6xxScheduler::scheduleWakeUpShared(
    BMEMngrSensor &sensor,
    uint64_t timestamp,
    uint8_t nextHeaterIndex) {
  return scheduleWakeUp(sensor, timestamp + GAS_WAIT_SHARED, nextHeaterIndex);
}

bool BME6xxScheduler::scheduleSensor(std::vector<BMEMngrSensor> &sensors) {
  if (sensors.empty())
    return false;

  uint64_t wakeUpTime = timestamp_millis() + SENSOR_WAKE_UP_TIME_OFFSET;
  return (
      selectNextSensor(
          sensors, BME6xxMode::PARALLEL, wakeUpTime, lastScheduledSensor) ||
      selectNextSensor(
          sensors, BME6xxMode::SLEEP, wakeUpTime, lastScheduledSensor));
}
esp_err_t BME6xxScheduler::updateHeatingStep(
    BMEMngrSensor &sensor,
    uint8_t currentHeaterIndex,
    BME6xxConfigurator &configurator) {
  esp_err_t err = ESP_OK;

  uint64_t nextWakeUp = timestamp_millis() + GAS_WAIT_SHARED;

  if (sensor.scheduleInfo.heaterIndex == sensor.config.heaterProfile.length) {
    sensor.scheduleInfo.heaterIndex = 0;
    sensor.scheduleInfo.dutyCycleIndex++;

    if (sensor.scheduleInfo.dutyCycleIndex >=
        sensor.config.dutyCycleProfile.numScans) {
      sensor.scheduleInfo.dutyCycleIndex = 0;

      err = configurator.setMode(sensor, BME6xxMode::SLEEP);
      if (err != ESP_OK)
        return err;

      nextWakeUp =
          timestamp_millis() + sensor.config.dutyCycleProfile.sleepDuration;

      if (err != ESP_OK)
        return err;

      return ESP_OK;
    }
  }
  err = scheduleWakeUp(sensor, nextWakeUp, currentHeaterIndex + 1);

  return err;
}
esp_err_t BME6xxScheduler::updateHeatingStep(
    BMEMngrSensor &sensor,
    uint8_t currentHeaterIndex,
    uint64_t currentTimestamp,
    BME6xxConfigurator &configurator) {
  esp_err_t err = ESP_OK;

  uint64_t nextWakeUp = currentTimestamp + GAS_WAIT_SHARED;
  uint8_t nextHeaterIndex = currentHeaterIndex + 1;
  if (nextHeaterIndex == sensor.config.heaterProfile.length) {
    nextHeaterIndex = 0;
    sensor.scheduleInfo.dutyCycleIndex++;

    ESP_LOGD(
        (char *)"BME6xxScheduler",
        "Sensor: %lu; Scan cycle finished, number scanning cycles %u/%u",
        sensor.id,
        sensor.scheduleInfo.dutyCycleIndex,
        sensor.config.dutyCycleProfile.numScans);

    if (sensor.scheduleInfo.dutyCycleIndex >=
        sensor.config.dutyCycleProfile.numScans) {
      sensor.scheduleInfo.dutyCycleIndex = 0;

      err = configurator.setMode(sensor, BME6xxMode::SLEEP);
      if (err != ESP_OK)
        return err;

      nextWakeUp += sensor.config.dutyCycleProfile.sleepDuration;

      if (err != ESP_OK)
        return err;

      ESP_LOGD(
          (char *)"BME6xxScheduler",
          "Sensor: %lu; Sleeping for %llums; Wake up time: %llu",
          sensor.id,
          sensor.config.dutyCycleProfile.sleepDuration,
          nextWakeUp);
    }
  }
  err = scheduleWakeUp(sensor, nextWakeUp, nextHeaterIndex);

  return err;
}

esp_err_t BME6xxScheduler::getLastScheduledSensor(
    std::vector<BMEMngrSensor> &sensors,
    BMEMngrSensor *&sensOut) {
  if (sensors.empty())
    return ESP_ERR_INVALID_ARG;

  if (lastScheduledSensor >= sensors.size())
    return ESP_ERR_SENSOR_INDEX_ERROR;

  sensOut = &sensors[lastScheduledSensor];

  // ESP_LOGI((char*)"Scheduler","Id: %lu; Wake up time: %llu ", sensOut->id,
  // sensOut->scheduleData.wakeUpTime);

  if (sensOut == nullptr)
    return ESP_ERR_SENSOR_INDEX_ERROR;

  return ESP_OK;
}
