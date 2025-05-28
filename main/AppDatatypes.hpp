#pragma once
#ifndef APPDATATYPES_H
#define APPDATATYPES_H

#include <memory>

#include "MeasurementDatatypes.h"

enum AppState {
  STATE_IDLE = 0,
  STATE_NORMAL,
  STATE_TRAIN_DATA_COLLECTION,

  STATE_COUNT
};

#define CMD_STR_ARG_MAX_LEN 255
#define CMD_TYPE_SIZE uint16_t

// First byte - command type subgroup
// Second byte - specific action
enum CommandType : uint16_t {
  SET_STATE = 0x1000,

  NORMAL_STATE_SET_FREQ = 0x2000,
  NORMAL_STATE_INCLUDE_ACCEL,
  NORMAL_STATE_INCLUDE_POS,
  NORMAL_STATE_INCLUDE_BATTERY,
  NORMAL_STATE_INCLUDE_VOLTAGE,
  NORMAL_STATE_INCLUDE_GAS,

  REC_TRAIN_DATA_STATE_SET_FREQ = 0x3000,
  REC_TRAIN_DATA_STATE_SET_LABEL,
  REC_TRAIN_DATA_STATE_SET_DURATON,
  REC_TRAIN_DATA_STATE_SCHED_START_TIME,
  REC_TRAIN_DATA_STATE_SET_SESSION,


  GET_BATTERY_LEVEL = 0x4000,
  GET_VOLTAGE_LEVEL,

  UNKNOWN = 0xFFFF
};

static CommandType supportedCMDs[] = {
    SET_STATE,
    REC_TRAIN_DATA_STATE_SET_FREQ,
    REC_TRAIN_DATA_STATE_SET_LABEL,
    REC_TRAIN_DATA_STATE_SET_SESSION,
    REC_TRAIN_DATA_STATE_SET_DURATON,
    REC_TRAIN_DATA_STATE_SCHED_START_TIME};

#define NUM_SUPPORTED_CMDs sizeof(supportedCMDs) / sizeof(supportedCMDs[0])

struct CTRLData {
  CommandType cmd;
  union ArgValue {
    float f;
    int32_t li;
    uint32_t lui;
    uint64_t llui;
    void *ptr;
    char str[CMD_STR_ARG_MAX_LEN];
  } arg;
};

enum AlertType {
  ALERT_NONE,

  ALERT_BATTERY_LOW,
  ALER_CURRENT_HIGH,
  ALERT_VOLTAGE_LOW,
  ALERT_VOLTAGE_HIGH,
  ALERT_ACCEL_HIGH,
  ALERT_ACCEL_FREE_FALL,
};

struct INA260Data : MeasurementBase {
  float voltage;
  float current;
  float power;

  void prepareJSONRepresentation(JsonObject &json) override {
    MeasurementBase::prepareJSONRepresentation(json);

    json["type"] = "INA260";
    json["voltage"] = voltage;
    json["current"] = current;
    json["power"] = power;
  }
};

struct ADXL345Data : MeasurementBase {
  float x;
  float y;
  float z;

  void prepareJSONRepresentation(JsonObject &json) override {
    MeasurementBase::prepareJSONRepresentation(json);

    json["type"] = "ADXL345";
    json["x"] = x;
    json["y"] = y;
    json["z"] = z;
  };
};


struct AlertData : MeasurementBase {
  AlertType alert;

  void prepareJSONRepresentation(JsonObject &json) override {
    MeasurementBase::prepareJSONRepresentation(json);

    json["type"] = "ALERT";
    json["alert"] = alert;
  }
};

struct PowerTelemetryData : MeasurementBase {
  float batLevel;
  float voltage;

  void prepareJSONRepresentation(JsonObject &json) override {
    MeasurementBase::prepareJSONRepresentation(json);

    json["type"] = "PWR_TELE";
    json["voltage"] = voltage;
    json["batLevel"] = batLevel;
  }
};

struct BME6XXData : MeasurementBase {
  const char *type;
  uint8_t sensorIdx;
  uint32_t sensorId;
  float temperature;
  float pressure;
  float humidity;
  float gasResistance;
  float compensGasResistance;
  float co2Eq;
  float breathVOC;
  float tvoc;
  float iaq;
  float staticIaq;
  int8_t sensorStable = -1;
  int8_t sensorRunIn = -1;
  int8_t gasIndex = -1;

  void prepareJSONRepresentation(JsonObject &json) override {
    MeasurementBase::prepareJSONRepresentation(json);

    json["type"] = type;
    json["s_Id"] = sensorId;
    json["temp"] = temperature;
    json["press"] = pressure;
    json["humid"] = humidity;
    json["gasRes"] = gasResistance;
    json["compensGasRes"] = compensGasResistance;
    json["co2Eq"] = co2Eq;
    json["breathVOC"] = breathVOC;
    json["tvoc"] = tvoc;
    json["iaq"] = iaq;
    json["staticIaq"] = staticIaq;
    if (gasIndex >= 0)
      json["gasIndex"] = gasIndex;
    if (sensorStable >= 0)
      json["stable"] = sensorStable;
    if (sensorRunIn >= 0)
      json["runin"] = sensorRunIn;
  }
};


struct MQTTDataContainer {
  std::string deviceId;
  std::string cluster;
  std::string label;
  std::string session;
  time_t timeposix;
  std::vector<std::unique_ptr<MeasurementBase>> objects;

  MQTTDataContainer(std::string id, std::string cluster)
      : deviceId(id), cluster(cluster) {}

  void addObject(std::unique_ptr<MeasurementBase> obj) {
    if (obj) {
      objects.push_back(std::move(obj));
    }
  }

  void eraseObjects() { objects.clear(); }

  void setTimeposix(time_t time) { timeposix = time; }

  void prepareJSONRepresentation(JsonObject &json) {
    json["deviceId"] = deviceId;
    json["cluster"] = cluster;
    json["timeposix"] = (uint64_t)timeposix;
    json["label"] = label;
    json["session"] = session;

    if (objects.size() > 0) {
      JsonArray dataArray = json["data"].to<JsonArray>();
      for (auto &obj : objects) {
        JsonObject sensorJson = dataArray.add<JsonObject>();
        obj.get()->prepareJSONRepresentation(sensorJson);
      }
    } else {
      ESP_LOGW(
          "MQTTDataContainer", "No objects available for JSON representation.");
    }
  }
};

#endif