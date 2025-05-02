#pragma once
#ifndef BME6XXCFGSERIALIZER_H
#define BME6XXCFGSERIALIZER_H

#include <vector>

#include "esp_system.h"

#include "Arduino.h"
#include "ArduinoJson.h"

#include "FSFile.h"

#include "BME6xxManagerTypes.h"

class BME6xxCfgSerializer {
public:
  esp_err_t serializeConfig();
  esp_err_t deserializeConfig(
      FSFile &cfg,
      std::vector<BMESensorConfig> &configurations);

private:
  JsonDocument configDoc;
};

#endif