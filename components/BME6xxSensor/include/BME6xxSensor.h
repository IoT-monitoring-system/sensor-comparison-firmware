#pragma once
#ifndef BME6XXSENSOR_H
#define BME6XXSENSOR_H

#include <cstddef>
#include <cstdint>
#include <variant>

#include "BME6xxSensorDatatypes.h"

class BME6xxSensor {
public:
  virtual void SetHeaterProfile(
      uint16_t *temp,
      uint16_t *mul,
      uint16_t sharedHeatrDur,
      uint8_t profileLen) = 0;

  virtual void SetHeaterProfile(uint16_t temp, uint16_t dur) = 0;

  virtual uint32_t GetMeasurementDuration(BME6xxMode opMode = BME6xxMode::SLEEP) = 0;

  virtual void SetOversampling(
      BME6xxOversampling osTemp = BME6xxOversampling::X2,
      BME6xxOversampling osPres = BME6xxOversampling::X16,
      BME6xxOversampling osHum = BME6xxOversampling::X1) = 0;
  virtual void SetOversampling(BME6xxOS &os) = 0;

  virtual uint32_t GetUniqueId() = 0;

  virtual void SetOperationMode(BME6xxMode opMode = BME6xxMode::SLEEP) = 0;

  virtual BME6xxStatus CheckStatus() = 0;

  virtual uint8_t FetchData() = 0;

  virtual size_t GetAllData(BME6xxData *dataOut, size_t maxLen) = 0;
  virtual size_t GetData(BME6xxData &dataOut) = 0;

  virtual void SelftestCheck() = 0;

  virtual void SoftReset() = 0;

  virtual const char *GetType() = 0;
};

#endif