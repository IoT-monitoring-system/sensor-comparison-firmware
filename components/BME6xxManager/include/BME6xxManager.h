#pragma once
#ifndef BME6XXMANAGER_H
#define BME6XXMANAGER_H

/* Include of Arduino Core */
#include "FS.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <SPI.h>
#include <Wire.h>

#include "bme68xLibrary.h"

#include "commMux.h"

#include "BME6xxManagerTypes.h"
#include "utils.h"

#include "FSFile.h"

#define NUM_BME6XX_UNITS 8U

#define HEATER_TIME_BASE 140U
#define MAX_HEATER_DURATION 200U
#define GAS_WAIT_SHARED 140U

class BME6xxManager {
private:
  static bme6xx_sensor _sensors[NUM_BME6XX_UNITS];
  BME68x bme68xSensors[NUM_BME6XX_UNITS];
  bme68x_data _field_data[3];
  uint8_t addedSensorsCounter = 0;
  uint8_t lastScheduledSensor = 0;

  JsonDocument _configDoc;

  int8_t initialize_sensor(uint8_t sensor_number, uint32_t &sensor_id);

  int8_t set_heater_profile(
      const String &heater_profile_str,
      const String &duty_cycle_str,
      bme6xx_heater_profile &heater_profile,
      uint8_t sensor_number);

  int8_t configure_sensor(
      bme6xx_heater_profile &heater_profile,
      uint8_t sensor_number);

public:
  BME6xxManager();

  inline bme6xx_sensor *get_sensor(uint8_t num) {
    bme6xx_sensor *sensor = nullptr;

    if (num < NUM_BME6XX_UNITS) {
      sensor = &_sensors[num];
    }
    return sensor;
  };
  inline bme6xx_sensor *get_last_scheduled_sensor() {
    bme6xx_sensor *sensor = nullptr;

    if (lastScheduledSensor < NUM_BME6XX_UNITS) {
      sensor = &_sensors[lastScheduledSensor];
    }
    return sensor;
  };

  inline bool select_next_sensor(
      uint64_t &wake_up_time,
      uint8_t &num,
      uint8_t mode) {
    num = (uint8_t)0xFF;

    for (uint8_t i = 0; i < NUM_BME6XX_UNITS; i++) {
      if (_sensors[i].is_configured) {
        if ((_sensors[i].mode == mode) &&
            (_sensors[i].wake_up_time < wake_up_time)) {
          wake_up_time = _sensors[i].wake_up_time;
          num = i;
        }
      }
    }

    if (num < NUM_BME6XX_UNITS) {
      return true;
    }
    return false;
  };

  inline bool schedule_sensor(uint8_t &num) {
    uint64_t wake_up_time = utils::get_tick_ms() + 20;
    return (
        select_next_sensor(wake_up_time, num, BME68X_PARALLEL_MODE) ||
        select_next_sensor(wake_up_time, num, BME68X_SLEEP_MODE));
  };
  inline bool schedule_sensor() {
    uint64_t wake_up_time = utils::get_tick_ms() + 20;
    return (
        select_next_sensor(
            wake_up_time, lastScheduledSensor, BME68X_PARALLEL_MODE) ||
        select_next_sensor(
            wake_up_time, lastScheduledSensor, BME68X_SLEEP_MODE));
  };

  /* Push an initialized sensor into the collection */
  bme6xx_ret_code push_sensor(BME68x sensorToAdd);
  bme6xx_ret_code pop_sensor();

  /* Initialize and push the maximum possible number of sensors into a
   * collection, in this case the commMux interface is used*/
  bme6xx_ret_code initialize_all_sensors();

  /* Initialize and push the provided number of sensors into a collection, in
   * this case the commMux interface is used but supports custom SPI and TwoWire
   * objects */
  bme6xx_ret_code initialize_all_sensors(
      SPIClass &customSPI,
      TwoWire &customWire,
      uint8_t numSensors);

  BME68x (&getSensors())[NUM_BME6XX_UNITS] { return bme68xSensors; }

  /* Configure the collection of sensors using the provided .bmeconfig file as
   * bytes*/
  bme6xx_ret_code begin(FSFile &configFile);

  bme6xx_ret_code collect_data(uint8_t num, bme68x_data *data[3]);

  /* Collects data from the last scheduled sensor */
  bme6xx_ret_code collect_data(bme68x_data *data[3]);

  void update_heating_step(uint8_t gas_index);
};

#endif
