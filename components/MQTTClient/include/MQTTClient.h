#pragma once

#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include "stdint.h"
#include <functional>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_system.h"

#include "mqtt_client.h"

#include "MQTTClientDefs.h"

static const char *MQTT_CLIENT_TAG = "MQTTClient";

class MQTTClient {
public:
  MQTTClient();

  esp_err_t start();
  esp_err_t stop();

  esp_err_t connect(bool block, uint32_t timeoutMs);
  esp_err_t connect(const char *username, const char *password, bool block, uint32_t timeoutMs);

  esp_err_t disconnect();

  esp_err_t publish(const char *topic, const char *payload, uint8_t qos, bool retain);
  esp_err_t enqueue(const char *topic, const char *payload, uint8_t qos, bool retain);

  esp_err_t subscribe(const char *topic, uint8_t qos);
  esp_err_t unsubscribe(const char *topic);

  esp_err_t setConfig(const MQTTClientConfig *config);

  esp_err_t setAddressCfg(const MQTTAddressConfig *config);
  esp_err_t setVerificationCfg(const MQTTVerificationConfig *config);
  esp_err_t setCredentialsCfg(const MQTTCredentialsConfig *config);
  esp_err_t setAuthenticationCfg(const MQTTAuthenticationConfig *config);
  esp_err_t setSessionCfg(const MQTTSessionConfig *config);
  esp_err_t setLastWillCfg(const MQTTLastWillConfig *config);
  esp_err_t setNetworkCfg(const MQTTNetworkConfig *config);
  esp_err_t setTaskCfg(const MQTTTaskConfig *config);
  esp_err_t setBufferCfg(const MQTTBufferConfig *config);
  esp_err_t setOutboxCfg(uint64_t limit);

  esp_err_t registerEventHandler(MQTT_EVENT_HANDLER_SIGNATURE handler, MQTTEventType event);
  esp_err_t unregisterEventHandler(MQTTEventType event);

  MQTTClientState getStatus();

  ~MQTTClient();

private:
  MQTTClientState _state = MQTTClientState::DISCONNECTED;
  esp_mqtt_client_handle_t _client = nullptr;
  esp_mqtt_client_config_t _clientCfg;

  MQTT_EVENT_HANDLER_SIGNATURE dataEventHandler = nullptr;
  MQTT_EVENT_HANDLER_SIGNATURE subscriptionEventHandler = nullptr;
  MQTT_EVENT_HANDLER_SIGNATURE publishedEventHandler = nullptr;
  MQTT_EVENT_HANDLER_SIGNATURE deletedEventHandler = nullptr;
  MQTT_EVENT_HANDLER_SIGNATURE customEventHandler = nullptr;

  MQTT_EVENT_HANDLER_SIGNATURE errorEventHandler = nullptr;
  MQTT_EVENT_HANDLER_SIGNATURE connectionEventHandler = nullptr;

  esp_err_t mqttEventHandler(esp_event_base_t base, int32_t eventId, void *eventData);

  static void mqttEventHandlerStatic(void *handlerArgs, esp_event_base_t base, int32_t eventId, void *eventData) {
    MQTTClient *instance = static_cast<MQTTClient *>(handlerArgs);
    esp_err_t error = instance->mqttEventHandler(base, eventId, eventData);
    if (error != ESP_OK) {
      ESP_LOGE(MQTT_CLIENT_TAG, "Error calling event handler: %u", error);
    }
  }

  void setDefaultEventHandlers();

  void handlerOnMqttData(MQTTClient &mqttClient, int32_t eventId, esp_mqtt_event_handle_t eventData);
  void handlerOnMqttSubscription(MQTTClient &mqttClient, int32_t eventId, esp_mqtt_event_handle_t eventData);
  void handlerOnMqttConnection(MQTTClient &mqttClient, int32_t eventId, esp_mqtt_event_handle_t eventData);
  void handlerOnMqttPublished(MQTTClient &mqttClient, int32_t eventId, esp_mqtt_event_handle_t eventData);
  void handlerOnMqttError(MQTTClient &mqttClient, int32_t eventId, esp_mqtt_event_handle_t eventData);
  void handlerOnMqttDeleted(MQTTClient &mqttClient, int32_t eventId, esp_mqtt_event_handle_t eventData);
  void handlerOnMqttCustom(MQTTClient &mqttClient, int32_t eventId, esp_mqtt_event_handle_t eventData);
};

#endif