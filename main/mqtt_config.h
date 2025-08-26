#pragma once
#ifndef MQTT_CONFIG_H
#define MQTT_CONFIG_H

#define MQTT_SERVER            "192.168.0.124"
#define MQTT_PORT              8884U
#define MQTT_USERNAME_COLLAR   "esp32_1"
#define MQTT_USERNAME_HOMEHUB  "esp32_2"
#define MQTT_USERNAME_HUMIDITY "esp32_3"
#define MQTT_PASSWORD          "Test12345"
#define MQTT_DATA_OUT_TOPIC    "/IoT-Monitoring-System/DEVICE_OUT/DATA"
#define MQTT_BUFFER_COUNT      3U
#define MQTT_MAX_MESSAGE_SIZE  4096U

#endif