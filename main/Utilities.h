#pragma once
#ifndef UTILITIES_H
#define UTILITIES_H

#include "esp_log.h"
#include "esp_types.h"
#include "esp_wifi.h"

#include <Arduino.h>
#include <WiFi.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "ArduinoJson.h"
#include "PubSubClient.h"

#include "AppDatatypes.hpp"

#define TAG_UTIL "Utilities"

#define SNTP_SERVER "pool.ntp.org"
#define SNTP_GMT_OFFSET_SEC 0L
#define SNTP_DAYLIGHT_OFFSET_SEC 0
#define SNTP_MAX_RETRIES 10
#define SNTP_RETRY_DELAY 1000

#define MQTT_MAX_PAYLOAD_SIZE 1024
#define MQTT_SERVER "xl9p.tplinkdns.com"
#define MQTT_PORT 1883
#define MQTT_USERNAME "tracker"
#define MQTT_PASSWORD "test12345"
#define MQTT_MAX_RETRIES 10
#define MQTT_RETRY_DELAY 5000

#define WIFI_SSID "TP-Link_C73C"
#define WIFI_PASSWORD "qdmmdnwm"
#define WIFI_MAX_RETRIES 10
#define WIFI_RETRY_DELAY 5000
#define WIFI_CONNECT_WAIT 5000
#define WIFI_MODE WIFI_STA
#define WIFI_TASK_STACK_SIZE 8192


class Utilities {
public:
  static CTRLData bytesToCTRLData(byte *bytes, size_t length);
  static CTRLData jsonToCTRLData(JsonDocument &json);
  static time_t POSIXLocalTime();

  static esp_err_t configureTime();
  static esp_err_t configureWiFi();
  static esp_err_t configureMQTT(const String &id, MQTT_CALLBACK_SIGNATURE);

  static esp_err_t MQTTPublish(const String &topic, const JsonDocument &doc);
  static esp_err_t MQTTSubscribe(const String &topic);
  static esp_err_t MQTTUnsubscribe(const String &topic);

  static bool MQTTRun();
};

#endif