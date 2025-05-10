#pragma once
#ifndef BME6XXMANAGERTYPES_H
#define BME6XXMANAGERTYPES_H

#include <string>

#include "esp_log.h"

#include "Arduino.h"

#include "BME6xxSensor.h"

#define MAX_BME6XX_UNITS 8U

enum class BMEIntfType { BME_INTF_SPI, BME_INTF_I2C, BME_INTF_MUX };

enum class BMEMngrState {
  STATE_DEFAULT,
  INVALID,

  INITIALIZED,
  CONFIGURED,
  RUNNING,
};

enum class BMESensorState {
  STATE_DEFAULT,
  INVALID,

  INITIALIZED,
  CONFIGURED,
  RUNNING,

};

/*!
 * @brief Structure to hold heater profile data
 */
struct BMEHeaterProfile {
  std::string id;
  int16_t timeBase = 140;
  uint16_t temperature[10]{};
  uint16_t duration[10]{};
  uint64_t heatCycleDuration;
  uint8_t length;
};

struct BMEDutyCycleProfile {
  std::string id;
  uint8_t numScans;
  uint8_t numSleeps;
  uint64_t sleepDuration;
};

// struct BMECalibrationData {};

struct BMESensorConfig {
  BMEHeaterProfile heaterProfile;
  BMEDutyCycleProfile dutyCycleProfile;
  BME6xxConf bme6xxDevCfg;
  BME6xxMode mode = BME6xxMode::SLEEP;
};

struct BMESensorScheduleData {
  uint64_t wakeUpTime = 0;
  uint8_t dutyCycleIndex = 0;
  uint8_t heaterIndex = 0;
};

struct BMESensorStateData {
  BMESensorState state = BMESensorState::INVALID;
  BME6xxData lastData[3]{};
};

/*!
 * @brief Structure to hold sensor state information
 */
struct BMEMngrSensor {
  uint32_t id;
  BME6xxSensor *device;
  BMESensorConfig config;
  BMESensorStateData stateData;
  BMESensorScheduleData scheduleData;
};

struct BMESensorData {
  uint32_t sensorId = 0;
  BME6xxData data[3]{};
  size_t dataLen = 0;
};

typedef esp_err_t(setModeFunc)(BMEMngrSensor *sens, BME6xxMode mode);
// typedef esp_err_t(setModeFunc)(BMEMngrSensor *sens);

#endif