#include "BME6xxConfigurator.h"
#include "esp_log.h"

const char *TAG = "BME6xxConfigurator";

esp_err_t BME6xxConfigurator::checkStatus(BME6xxSensor &sensor) {
  BME6xxStatus status = sensor.bme6xxCheckStatus();

  ESP_LOGI(TAG, "Sensor status: %i", static_cast<int8_t>(status));

  if (status != BME6xxStatus::OK)
    return ESP_FAIL;

  return ESP_OK;
}

BME6xxConfigurator::BME6xxConfigurator() {}

esp_err_t BME6xxConfigurator::setMode(BMEMngrSensor &sensor, BME6xxMode mode) {
  esp_err_t err = ESP_OK;

  sensor.device->bme6xxSetOpMode(mode);
  err = checkStatus(*sensor.device);
  if (err != ESP_OK)
    return ESP_FAIL;

  sensor.config.mode = mode;

  return ESP_OK;
}

esp_err_t BME6xxConfigurator::initialize() { return ESP_OK; }

esp_err_t BME6xxConfigurator::configureSensor(
    BMEMngrSensor &sensor,
    BMESensorConfig &config) {

  esp_err_t err = ESP_OK;

  err = configureOS(sensor, config.bme6xxDevCfg.os);
  if (err != ESP_OK)
    return err;

  err = configureHeaterProfile(sensor, config.heaterProfile);
  if (err != ESP_OK)
    return err;

  sensor.config = config;

  return err;
}
esp_err_t BME6xxConfigurator::resetSensor(BMEMngrSensor &sensor) {
  esp_err_t err = ESP_OK;

  sensor.device->bme6xxSoftReset();

  err = checkStatus(*sensor.device);
  if (err != ESP_OK)
    return err;

  err = resetHeaterProfile(sensor);
  if (err != ESP_OK)
    return err;

  return err;
}

esp_err_t BME6xxConfigurator::configureHeaterProfile(
    BMEMngrSensor &sensor,
    BMEHeaterProfile &profile) {

  uint32_t sharedHeatrDur =
      profile.timeBase - (sensor.device->bme6xxGetMeasDur(sensor.config.mode) /
                          static_cast<int64_t>(1000));

  sensor.device->bme6xxSetHeaterProf(
      profile.temperature, profile.duration, sharedHeatrDur, profile.length);

  return checkStatus(*sensor.device);
}
esp_err_t BME6xxConfigurator::resetHeaterProfile(BMEMngrSensor &sensor) {
  BMEDutyCycleProfile newBMEDutyCycleProf = BMEDutyCycleProfile{};
  BMEHeaterProfile newBMEHeaterProf = BMEHeaterProfile{};

  sensor.config.dutyCycleProfile = newBMEDutyCycleProf;
  sensor.config.heaterProfile = newBMEHeaterProf;

  return ESP_OK;
}

esp_err_t BME6xxConfigurator::configureOS(BMEMngrSensor &sensor, BME6xxOS &os) {
  sensor.device->bme6xxSetOS(os);

  return checkStatus(*sensor.device);
}
esp_err_t BME6xxConfigurator::resetOS(BMEMngrSensor &sensor) {
  sensor.config.bme6xxDevCfg.os = BME6xxOS{};

  return ESP_OK;
}