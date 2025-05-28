#pragma once
#ifndef BME6XXSENSOR_H
#define BME6XXSENSOR_H

#include <cstddef>
#include <cstdint>
#include <variant>

#include "BME6xxSensorDatatypes.h"

class BME6xxSensor {
public:
  virtual void bme6xxSetHeaterProf(
      uint16_t *temp,
      uint16_t *mul,
      uint16_t sharedHeatrDur,
      uint8_t profileLen) = 0;

  virtual void bme6xxSetHeaterProf(uint16_t temp, uint16_t dur) = 0;

  virtual uint32_t bme6xxGetMeasDur(BME6xxMode opMode = BME6xxMode::SLEEP) = 0;

  virtual void bme6xxSetOS(
      BME6xxOversampling osTemp = BME6xxOversampling::X2,
      BME6xxOversampling osPres = BME6xxOversampling::X16,
      BME6xxOversampling osHum = BME6xxOversampling::X1) = 0;
  virtual void bme6xxSetOS(BME6xxOS &os) = 0;

  virtual uint32_t bme6xxGetUniqueId() = 0;

  virtual void bme6xxSetOpMode(BME6xxMode opMode = BME6xxMode::SLEEP) = 0;

  virtual BME6xxStatus bme6xxCheckStatus() = 0;

  virtual uint8_t bme6xxFetchData() = 0;

  virtual size_t bme6xxGetAllData(BME6xxData *dataOut, size_t maxLen) = 0;
  virtual size_t bme6xxGetData(BME6xxData &dataOut) = 0;

  virtual void bme6xxSelftestCheck() = 0;

  virtual void bme6xxSoftReset() = 0;

  virtual const char *bme6xxGetType() = 0;
};

#endif