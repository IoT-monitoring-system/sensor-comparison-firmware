#include "esp_log.h"

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#include "MQTTClient.h"

static EventGroupHandle_t mqttEventGroup;

void MQTTClient::handlerOnMqttData(MQTTClient &mqttClient, int32_t eventId, esp_mqtt_event_handle_t eventData) {
  //   - msg_id               message id
  //   - topic                pointer to the received topic
  //   - topic_len            length of the topic
  //   - data                 pointer to the received data
  //   - data_len             length of the data for this event
  //   - current_data_offset  offset of the current data for this event
  //   - total_data_len       total length of the data received
  //   - retain               retain flag of the message
  //   - qos                  QoS level of the message
  //   - dup                  dup flag of the message
  //   Note: Multiple MQTT_EVENT_DATA could be fired for one
  // message, if it is longer than internal buffer. In that
  // case only first event contains topic pointer and length,
  // other contain data only with current data length and
  // current data offset updating.
  int msgId = eventData->msg_id;

  if (msgId < 0) {
    ESP_LOGE(MQTT_CLIENT_TAG, "Error receiving a message on %s; Error: %i",
             eventData->topic_len ? eventData->topic : "None", msgId);
    return;
  }

  ESP_LOGI(MQTT_CLIENT_TAG, "Received on %.*s; Content: %.*s", eventData->topic_len, eventData->topic,
           eventData->data_len, eventData->data);
};
void MQTTClient::handlerOnMqttPublished(MQTTClient &mqttClient, int32_t eventId, esp_mqtt_event_handle_t eventData) {
  int msgId = eventData->msg_id;

  if (msgId < 0) {
    ESP_LOGE(MQTT_CLIENT_TAG, "Failed to publish a message; Error: %i", msgId);
    return;
  }
  ESP_LOGI(MQTT_CLIENT_TAG, "Published a message");
};
void MQTTClient::handlerOnMqttSubscription(MQTTClient &mqttClient, int32_t eventId, esp_mqtt_event_handle_t eventData) {
  int msgId = eventData->msg_id;
  esp_mqtt_error_type_t errorType = eventData->error_handle->error_type;

  if (errorType == MQTT_ERROR_TYPE_SUBSCRIBE_FAILED || msgId < 0) {
    if (eventData->topic_len)
      ESP_LOGE(MQTT_CLIENT_TAG, "Failed to subscribe to %s; Broker message: %s", eventData->topic,
               eventData->data_len ? eventData->data : "None");
    return;
  }

  if (eventData->topic_len)
    ESP_LOGI(MQTT_CLIENT_TAG, "Subscribed to: %s", eventData->topic);
};
void MQTTClient::handlerOnMqttDeleted(MQTTClient &mqttClient, int32_t eventId, esp_mqtt_event_handle_t eventData) {
  ESP_LOGI(MQTT_CLIENT_TAG, "An expired message was deleted from the internal outbox");
};
void MQTTClient::handlerOnMqttCustom(MQTTClient &mqttClient, int32_t eventId, esp_mqtt_event_handle_t eventData) {
  ESP_LOGI(MQTT_CLIENT_TAG, "Custom event handler triggered");
};

/**
 * | MQTT_ERROR_TYPE_TCP_TRANSPORT | esp_tls_last_esp_err, esp_tls_stack_err, esp_tls_cert_verify_flags, sock_errno |
 * Error reported from tcp_transport/esp-tls | | MQTT_ERROR_TYPE_CONNECTION_REFUSED | connect_return_code | Internal
 * error reported from *MQTT* broker on connection |
 */
void MQTTClient::handlerOnMqttError(MQTTClient &mqttClient, int32_t eventId, esp_mqtt_event_handle_t eventData) {
  esp_mqtt_error_type_t errorType = eventData->error_handle->error_type;
  esp_mqtt_connect_return_code_t connectCode = eventData->error_handle->connect_return_code;

  switch (errorType) {
  case MQTT_ERROR_TYPE_NONE: {
    break;
  }
  case MQTT_ERROR_TYPE_CONNECTION_REFUSED: {
    this->_state = static_cast<MQTTClientState>(connectCode);
    ESP_LOGE(MQTT_CLIENT_TAG, "Failed to connect, reason: %u", connectCode);
    break;
  }
  case MQTT_ERROR_TYPE_TCP_TRANSPORT: {
    this->_state = MQTTClientState::CONNECT_TRANSPORT_ERROR;
    ESP_LOGE(MQTT_CLIENT_TAG, "Transport error: %i, %i, %i, %i", eventData->error_handle->esp_tls_last_esp_err,
             eventData->error_handle->esp_tls_stack_err, eventData->error_handle->esp_tls_cert_verify_flags,
             eventData->error_handle->esp_transport_sock_errno);
    break;
  }
  case MQTT_ERROR_TYPE_SUBSCRIBE_FAILED: {
    if (eventData->topic_len)
      ESP_LOGE(MQTT_CLIENT_TAG, "Failed to subscribe to %s", eventData->topic);
  }
  };
};
void MQTTClient::handlerOnMqttConnection(MQTTClient &mqttClient, int32_t eventId, esp_mqtt_event_handle_t eventData) {
  switch (static_cast<esp_mqtt_event_id_t>(eventId)) {
  case MQTT_EVENT_DISCONNECTED: {
    this->_state = MQTTClientState::DISCONNECTED;
    break;
  }
  case MQTT_EVENT_BEFORE_CONNECT: {
    break;
  }
  case MQTT_EVENT_CONNECTED: {
    this->_state = MQTTClientState::CONNECTED;
    xEventGroupSetBits(mqttEventGroup, MQTT_CLIENT_CONNECTED_BIT);
    break;
  }
  default: {
    ESP_LOGE(MQTT_CLIENT_TAG, "Unexpected connection event, %li", eventId);
  }
  }
};

MQTTClient::MQTTClient() { this->setDefaultEventHandlers(); }

MQTTClient::~MQTTClient() {
  if (this->_client) {
    esp_mqtt_client_unregister_event(this->_client, static_cast<esp_mqtt_event_id_t>(ESP_EVENT_ANY_ID),
                                     this->mqttEventHandlerStatic);
    this->disconnect();
    this->stop();
    esp_mqtt_client_destroy(this->_client);
  }
};

esp_err_t MQTTClient::start() {
  esp_err_t error = ESP_OK;

  mqttEventGroup = xEventGroupCreate();

  error = esp_event_loop_create_default();
  error = (error == ESP_ERR_INVALID_STATE) ? ESP_OK : error;
  if (error != ESP_OK)
    return error;

  this->_client = esp_mqtt_client_init(&this->_clientCfg);
  if (this->_client == nullptr)
    return ESP_ERR_MQTT_CLIENT_INIT_FAIL;

  return esp_mqtt_client_register_event(this->_client, static_cast<esp_mqtt_event_id_t>(ESP_EVENT_ANY_ID),
                                        this->mqttEventHandlerStatic, this);
};
esp_err_t MQTTClient::stop() {
  esp_err_t error = ESP_OK;

  if (this->_client == nullptr)
    return ESP_ERR_MQTT_CLIENT_NOT_INIT;

  return esp_mqtt_client_stop(this->_client);
};

esp_err_t MQTTClient::connect(bool block, uint32_t timeoutMs) {
  esp_err_t error = esp_mqtt_client_start(this->_client);
  if (error != ESP_OK)
    return error;

  if (block) {
    EventBits_t bits =
        xEventGroupWaitBits(mqttEventGroup, MQTT_CLIENT_CONNECTED_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(timeoutMs));

    if (!bits) {
      this->stop();
      return ESP_ERR_MQTT_CLIENT_CONNECT_FAILED;
    }
  }

  return error;
};
esp_err_t MQTTClient::connect(const char *username, const char *password, bool block, uint32_t timeoutMs) {
  esp_err_t error = ESP_OK;

  if (username == nullptr || password == nullptr)
    return ESP_ERR_INVALID_ARG;

  this->_clientCfg.credentials.username = username;
  this->_clientCfg.credentials.authentication.password = password;
  error = esp_mqtt_set_config(this->_client, &this->_clientCfg);
  if (error != ESP_OK)
    return error;

  error = esp_mqtt_client_start(this->_client);
  if (error != ESP_OK)
    return error;

  if (block) {
    EventBits_t bits =
        xEventGroupWaitBits(mqttEventGroup, MQTT_CLIENT_CONNECTED_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(timeoutMs));

    if (!bits) {
      this->stop();
      return ESP_ERR_MQTT_CLIENT_CONNECT_FAILED;
    }
  }

  return error;
};

esp_err_t MQTTClient::disconnect() {
  esp_err_t error = ESP_OK;
  if (this->_state == MQTTClientState::DISCONNECTED)
    return ESP_OK;

  error = esp_mqtt_client_disconnect(this->_client);
  if (error != ESP_OK)
    return error;

  this->_state = MQTTClientState::DISCONNECTED;

  return error;
};

esp_err_t MQTTClient::publish(const char *topic, const char *payload, uint8_t qos, bool retain) {
  if (this->_state != MQTTClientState::CONNECTED)
    return ESP_ERR_MQTT_CLIENT_INVALID_STATE;

  int msgId = esp_mqtt_client_publish(this->_client, topic, payload, 0, qos, retain);
  if (msgId < 0)
    return ESP_ERR_MQTT_PUBLISH_FAILED;

  return ESP_OK;
};
esp_err_t MQTTClient::enqueue(const char *topic, const char *payload, uint8_t qos, bool retain) {
  int msgId = esp_mqtt_client_enqueue(this->_client, topic, payload, 0, qos, retain, (qos & 0x01) | 1);
  if (msgId < 0)
    return ESP_ERR_MQTT_ENQUEUE_FAILED;

  return ESP_OK;
};

esp_err_t MQTTClient::subscribe(const char *topic, uint8_t qos) {
  if (this->_state != MQTTClientState::CONNECTED)
    return ESP_ERR_MQTT_CLIENT_INVALID_STATE;

  int msgId = esp_mqtt_client_subscribe_single(this->_client, topic, qos);
  if (msgId < 0)
    return ESP_ERR_MQTT_SUBSCRIBE_FAILED;

  return ESP_OK;
};
esp_err_t MQTTClient::unsubscribe(const char *topic) {
  if (this->_state != MQTTClientState::CONNECTED)
    return ESP_ERR_MQTT_CLIENT_INVALID_STATE;

  int msgId = esp_mqtt_client_unsubscribe(this->_client, topic);
  if (msgId < 0)
    return ESP_ERR_MQTT_UNSUBSCRIBE_FAILED;

  return ESP_OK;
};

esp_err_t MQTTClient::setConfig(const MQTTClientConfig *config) {
  if (this->_state == MQTTClientState::CONNECTED)
    return ESP_ERR_MQTT_CLIENT_INVALID_STATE;

  if (config == nullptr)
    return ESP_ERR_INVALID_ARG;

  this->_clientCfg = *config;

  return esp_mqtt_set_config(this->_client, &this->_clientCfg);
};

esp_err_t MQTTClient::setAddressCfg(const MQTTAddressConfig *config) {
  if (this->_state == MQTTClientState::CONNECTED)
    return ESP_ERR_MQTT_CLIENT_INVALID_STATE;

  if (config == nullptr)
    return ESP_ERR_INVALID_ARG;

  this->_clientCfg.broker.address = *config;

  return esp_mqtt_set_config(this->_client, &this->_clientCfg);
};
esp_err_t MQTTClient::setVerificationCfg(const MQTTVerificationConfig *config) {
  if (this->_state == MQTTClientState::CONNECTED)
    return ESP_ERR_MQTT_CLIENT_INVALID_STATE;

  if (config == nullptr)
    return ESP_ERR_INVALID_ARG;

  this->_clientCfg.broker.verification = *config;

  return esp_mqtt_set_config(this->_client, &this->_clientCfg);
};
esp_err_t MQTTClient::setCredentialsCfg(const MQTTCredentialsConfig *config) {
  if (this->_state == MQTTClientState::CONNECTED)
    return ESP_ERR_MQTT_CLIENT_INVALID_STATE;

  if (config == nullptr)
    return ESP_ERR_INVALID_ARG;

  this->_clientCfg.credentials.client_id = config->client_id;
  this->_clientCfg.credentials.set_null_client_id = config->set_null_client_id;
  this->_clientCfg.credentials.username = config->username;

  return esp_mqtt_set_config(this->_client, &this->_clientCfg);
};
esp_err_t MQTTClient::setAuthenticationCfg(const MQTTAuthenticationConfig *config) {
  if (this->_state == MQTTClientState::CONNECTED)
    return ESP_ERR_MQTT_CLIENT_INVALID_STATE;

  if (config == nullptr)
    return ESP_ERR_INVALID_ARG;

  this->_clientCfg.credentials.authentication = *config;

  return esp_mqtt_set_config(this->_client, &this->_clientCfg);
};
esp_err_t MQTTClient::setSessionCfg(const MQTTSessionConfig *config) {
  if (this->_state == MQTTClientState::CONNECTED)
    return ESP_ERR_MQTT_CLIENT_INVALID_STATE;

  if (config == nullptr)
    return ESP_ERR_INVALID_ARG;

  this->_clientCfg.session.disable_clean_session = config->disable_clean_session;
  this->_clientCfg.session.disable_keepalive = config->disable_keepalive;
  this->_clientCfg.session.keepalive = config->keepalive;
  this->_clientCfg.session.message_retransmit_timeout = config->message_retransmit_timeout;
  this->_clientCfg.session.protocol_ver = config->protocol_ver;

  return esp_mqtt_set_config(this->_client, &this->_clientCfg);
};
esp_err_t MQTTClient::setLastWillCfg(const MQTTLastWillConfig *config) {
  if (this->_state == MQTTClientState::CONNECTED)
    return ESP_ERR_MQTT_CLIENT_INVALID_STATE;

  if (config == nullptr)
    return ESP_ERR_INVALID_ARG;

  this->_clientCfg.session.last_will = *config;

  return esp_mqtt_set_config(this->_client, &this->_clientCfg);
};
esp_err_t MQTTClient::setNetworkCfg(const MQTTNetworkConfig *config) {
  if (this->_state == MQTTClientState::CONNECTED)
    return ESP_ERR_MQTT_CLIENT_INVALID_STATE;

  if (config == nullptr)
    return ESP_ERR_INVALID_ARG;

  this->_clientCfg.network = *config;

  return esp_mqtt_set_config(this->_client, &this->_clientCfg);
};
esp_err_t MQTTClient::setTaskCfg(const MQTTTaskConfig *config) {
  if (this->_state == MQTTClientState::CONNECTED)
    return ESP_ERR_MQTT_CLIENT_INVALID_STATE;

  if (config == nullptr)
    return ESP_ERR_INVALID_ARG;

  this->_clientCfg.task = *config;

  return esp_mqtt_set_config(this->_client, &this->_clientCfg);
};
esp_err_t MQTTClient::setBufferCfg(const MQTTBufferConfig *config) {
  if (this->_state == MQTTClientState::CONNECTED)
    return ESP_ERR_MQTT_CLIENT_INVALID_STATE;

  if (config == nullptr)
    return ESP_ERR_INVALID_ARG;

  this->_clientCfg.buffer = *config;

  return esp_mqtt_set_config(this->_client, &this->_clientCfg);
};
esp_err_t MQTTClient::setOutboxCfg(uint64_t limit) {
  if (this->_state == MQTTClientState::CONNECTED)
    return ESP_ERR_MQTT_CLIENT_INVALID_STATE;

  this->_clientCfg.outbox.limit = limit;

  return esp_mqtt_set_config(this->_client, &this->_clientCfg);
};

esp_err_t MQTTClient::registerEventHandler(MQTT_EVENT_HANDLER_SIGNATURE handler, MQTTEventType event) {
  switch (event) {
  case MQTTEventType::DATA_EVENT: {
    this->dataEventHandler = handler;
    break;
  }
  case MQTTEventType::PUBLISH_EVENT: {
    this->publishedEventHandler = handler;
    break;
  }
  case MQTTEventType::SUBSCRIBE_EVENT: {
    this->subscriptionEventHandler = handler;
    break;
  }
  case MQTTEventType::DELETED_EVENT: {
    this->deletedEventHandler = handler;
    break;
  }
  case MQTTEventType::CUSTOM_EVENT: {
    this->customEventHandler = handler;
    break;
  }

  case MQTTEventType::CONNECTION_EVENT:
  case MQTTEventType::ERROR_EVENT:
    ESP_LOGW(MQTT_CLIENT_TAG, "Redefining this callback will break the client state management");
  default: {
    return ESP_ERR_INVALID_ARG;
  }
  }

  return ESP_OK;
};
esp_err_t MQTTClient::unregisterEventHandler(MQTTEventType event) {
  switch (event) {
  case MQTTEventType::DATA_EVENT: {
    this->dataEventHandler = nullptr;
    break;
  }
  case MQTTEventType::PUBLISH_EVENT: {
    this->publishedEventHandler = nullptr;
    break;
  }
  case MQTTEventType::SUBSCRIBE_EVENT: {
    this->subscriptionEventHandler = nullptr;
    break;
  }
  case MQTTEventType::DELETED_EVENT: {
    this->deletedEventHandler = nullptr;
    break;
  }
  case MQTTEventType::CUSTOM_EVENT: {
    this->customEventHandler = nullptr;
    break;
  }

  case MQTTEventType::CONNECTION_EVENT:
  case MQTTEventType::ERROR_EVENT:
    ESP_LOGW(MQTT_CLIENT_TAG, "Unregistering this callback will break the client state management");
  default: {
    return ESP_ERR_INVALID_ARG;
  }
  }

  return ESP_OK;
};

MQTTClientState MQTTClient::getStatus() { return this->_state; };

esp_err_t MQTTClient::mqttEventHandler(esp_event_base_t base, int32_t event_id, void *event_data) {
  esp_mqtt_event_handle_t event = static_cast<esp_mqtt_event_handle_t>(event_data);

  switch (event->event_id) {
  case MQTT_EVENT_DISCONNECTED:
  case MQTT_EVENT_CONNECTED:
  case MQTT_EVENT_BEFORE_CONNECT: {
    if (this->connectionEventHandler == nullptr)
      return ESP_ERR_INVALID_EVENT_HANDLER;
    this->connectionEventHandler(*this, event_id, event);
    break;
  }
  case MQTT_EVENT_SUBSCRIBED:
  case MQTT_EVENT_UNSUBSCRIBED: {
    if (this->subscriptionEventHandler == nullptr)
      return ESP_ERR_INVALID_EVENT_HANDLER;
    this->subscriptionEventHandler(*this, event_id, event);
    break;
  }
  case MQTT_EVENT_PUBLISHED: {
    if (this->publishedEventHandler == nullptr)
      return ESP_ERR_INVALID_EVENT_HANDLER;
    this->publishedEventHandler(*this, event_id, event);
    break;
  }
  case MQTT_EVENT_DATA: {
    if (this->dataEventHandler == nullptr)
      return ESP_ERR_INVALID_EVENT_HANDLER;
    this->dataEventHandler(*this, event_id, event);
    break;
  }
  case MQTT_EVENT_ERROR: {
    if (this->errorEventHandler == nullptr)
      return ESP_ERR_INVALID_EVENT_HANDLER;
    this->errorEventHandler(*this, event_id, event);
    break;
  }
  case MQTT_EVENT_DELETED: {
    if (this->deletedEventHandler == nullptr)
      return ESP_ERR_INVALID_EVENT_HANDLER;
    this->deletedEventHandler(*this, event_id, event);
    break;
  }
  default: {
    if (this->customEventHandler == nullptr)
      return ESP_ERR_INVALID_EVENT_HANDLER;
    this->customEventHandler(*this, event_id, event);
  }
  }
  return ESP_OK;
};

void MQTTClient::setDefaultEventHandlers() {
  this->connectionEventHandler = [this](MQTTClient &client, int32_t eventId, esp_mqtt_event_handle_t eventData) {
    this->handlerOnMqttConnection(client, eventId, eventData);
  };
  this->errorEventHandler = [this](MQTTClient &client, int32_t eventId, esp_mqtt_event_handle_t eventData) {
    this->handlerOnMqttError(client, eventId, eventData);
  };
  this->dataEventHandler = [this](MQTTClient &client, int32_t eventId, esp_mqtt_event_handle_t eventData) {
    this->handlerOnMqttData(client, eventId, eventData);
  };
  this->publishedEventHandler = [this](MQTTClient &client, int32_t eventId, esp_mqtt_event_handle_t eventData) {
    this->handlerOnMqttPublished(client, eventId, eventData);
  };
  this->subscriptionEventHandler = [this](MQTTClient &client, int32_t eventId, esp_mqtt_event_handle_t eventData) {
    this->handlerOnMqttSubscription(client, eventId, eventData);
  };
  this->deletedEventHandler = [this](MQTTClient &client, int32_t eventId, esp_mqtt_event_handle_t eventData) {
    this->handlerOnMqttDeleted(client, eventId, eventData);
  };
  this->customEventHandler = [this](MQTTClient &client, int32_t eventId, esp_mqtt_event_handle_t eventData) {
    this->handlerOnMqttCustom(client, eventId, eventData);
  };
}