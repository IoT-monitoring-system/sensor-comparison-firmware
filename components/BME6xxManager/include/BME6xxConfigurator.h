#pragma once
#ifndef BME6XXCONFIGURATOR_H
#define BME6XXCONFIGURATOR_H

#include "esp_types.h"

#include "Arduino.h"
#include "ArduinoJson.h"
#include "SPI.h"
#include "Wire.h"

#include "BME6xxManagerTypes.h"

class BME6xxConfigurator {
public:
  BME6xxConfigurator();

  esp_err_t initialize();

  esp_err_t setMode(BMEMngrSensor &sensor, BME6xxMode mode);

  esp_err_t configureSensor(BMEMngrSensor &sensor, BMESensorConfig &config);
  esp_err_t resetSensor(BMEMngrSensor &sensor);

  esp_err_t configureHeaterProfile(
    BMEMngrSensor &sensor,
      BMEHeaterProfile &profile);
  esp_err_t resetHeaterProfile(BMEMngrSensor &sensor);

  esp_err_t configureOS(BMEMngrSensor &sensor, BME6xxOS &os);
  esp_err_t resetOS(BMEMngrSensor &sensor);

private:
  esp_err_t checkStatus(BME6xxSensor &sensor);
};

#endif