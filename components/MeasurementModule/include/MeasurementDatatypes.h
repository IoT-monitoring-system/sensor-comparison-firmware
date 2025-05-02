#pragma once
#ifndef MEASUREMENTDATATYPES_H
#define MEASUREMENTDATATYPES_H

#include "esp_types.h"

#include "ArduinoJson.h"

#include <bitset>
#include <vector>

struct MeasurementBase {
  size_t size;
  uint64_t timestamp;

  virtual void prepareJSONRepresentation(JsonObject &json) {
    json["timestamp"] = timestamp;
  }

  virtual ~MeasurementBase() = default;
};

#endif