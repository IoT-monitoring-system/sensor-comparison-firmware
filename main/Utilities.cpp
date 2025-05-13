#include "Utilities.h"

WiFiClient espClient;
PubSubClient client(espClient);

static int32_t task_create_pinned_to_core_wrapper(
    void *task_func,
    const char *name,
    uint32_t stack_depth,
    void *param,
    uint32_t prio,
    void *task_handle,
    uint32_t core_id) {
  return (uint32_t)xTaskCreatePinnedToCore(
      (TaskFunction_t)task_func,
      name,
      WIFI_TASK_STACK_SIZE,
      param,
      prio,
      (TaskHandle_t *)task_handle,
      (core_id < portNUM_PROCESSORS ? core_id : tskNO_AFFINITY));
}

static int32_t task_create_wrapper(
    void *task_func,
    const char *name,
    uint32_t stack_depth,
    void *param,
    uint32_t prio,
    void *task_handle) {
  return (uint32_t)xTaskCreate(
      (TaskFunction_t)task_func,
      name,
      WIFI_TASK_STACK_SIZE,
      param,
      prio,
      (TaskHandle_t *)task_handle);
}

static wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

inline bool checkTimeSync() {
  if (time(NULL) < 1600000000U) {
    ESP_LOGE(TAG_UTIL, "Time is not synchronized%s", "");
    return false;
  }
  return true;
}
inline bool checkWiFiConn() {
  if (WiFi.status() != WL_CONNECTED) {
    ESP_LOGE(TAG_UTIL, "WiFi is not connected, rc=%u", WiFi.status());
    return false;
  }
  return true;
}
inline bool checkMQTTConn() {
  if (!client.connected()) {
    ESP_LOGE(TAG_UTIL, "MQTT is not connected, rc=%i", client.state());
    return false;
  }
  return true;
}
inline uint16_t calculateChecksum(byte *data, size_t length) {
  uint16_t checksum = 0;

  for (size_t i = 0; i < length; ++i) {
    checksum ^= data[i];
  }

  return checksum;
}
inline bool verifyChecksum(byte *buffer) {
  uint16_t checksum = calculateChecksum(buffer, 10);
  uint16_t receivedChecksum = (buffer[10] << 8) | buffer[11];
  return checksum == receivedChecksum;
}

/* <command><argument><checksum>\r\n */
CTRLData Utilities::bytesToCTRLData(byte *bytes, size_t length) {
  CTRLData data;

  if (length != 14) {
    data.cmd = UNKNOWN;
    return data;
  }
  if (bytes[12] != 0x0D || bytes[13] != 0x0A) {
    data.cmd = UNKNOWN;
    return data;
  }
  if (!verifyChecksum(bytes)) {
    data.cmd = UNKNOWN;
    return data;
  }

  uint16_t concBytes = (bytes[0] << 8) | bytes[1];

  for (uint16_t i = 0; i < NUM_SUPPORTED_CMDs; i++) {
    if (concBytes == supportedCMDs[i]) {
      data.cmd = supportedCMDs[i];

      uint64_t arg = 0;
      for (int i = 0; i < 8; i++) {
        arg |= (static_cast<uint64_t>(bytes[i + 2]) << (8 * (7 - i)));
      }

      switch (supportedCMDs[i]) {
      case REC_TRAIN_DATA_STATE_SET_FREQ: {
        if (arg > std::numeric_limits<float>::max()) {
          data.cmd = UNKNOWN;
        } else {
          float *argFloat = reinterpret_cast<float *>(&arg);
          data.arg.f = *argFloat;
        }
        break;
      }
      case REC_TRAIN_DATA_STATE_SET_LABEL: {
        memcpy(data.arg.str, &bytes[2], 8);
        data.arg.str[8] = '\0';
        break;
      }
      case REC_TRAIN_DATA_STATE_SET_DURATON: {
        if (arg > std::numeric_limits<long unsigned int>::max()) {
          data.cmd = UNKNOWN;
        } else {
          data.arg.lui = arg;
        }
        break;
      }
      case REC_TRAIN_DATA_STATE_SCHED_START_TIME: {
        if (arg > std::numeric_limits<long long unsigned int>::max()) {
          data.cmd = UNKNOWN;
        } else {
          data.arg.llui = arg;
        }
        break;
      }
      case SET_STATE: {
        if (arg >= STATE_COUNT) {
          data.cmd = UNKNOWN;
        } else {
          data.arg.lui = arg;
        }
        break;
      }
      default: {
        data.arg.ptr = nullptr;
      }
      }
    }
  }
  return data;
}

CTRLData Utilities::jsonToCTRLData(JsonDocument &json) {
  CommandType cmd = json["cmd"].as<CommandType>();

  CTRLData ctrlData;

  switch (cmd) {
  case REC_TRAIN_DATA_STATE_SET_FREQ: {
    float freq = json["arg"].as<float>();

    if (freq > std::numeric_limits<float>::max()) {
      ctrlData.cmd = UNKNOWN;
    }

    ctrlData.cmd = cmd;
    ctrlData.arg.f = freq;
    break;
  }
  case REC_TRAIN_DATA_STATE_SET_LABEL: {
    std::string label = json["arg"].as<std::string>();

    if (label.length() > CMD_STR_ARG_MAX_LEN) {
      ctrlData.cmd = UNKNOWN;
      break;
    }

    ctrlData.cmd = cmd;
    strncpy(ctrlData.arg.str, label.c_str(), sizeof(ctrlData.arg.str));
    ctrlData.arg.str[sizeof(ctrlData.arg.str) - 1] = '\0';

    break;
  }
  case REC_TRAIN_DATA_STATE_SET_SESSION: {
    std::string session = json["arg"].as<std::string>();

    if (session.length() > CMD_STR_ARG_MAX_LEN) {
      ctrlData.cmd = UNKNOWN;
      break;
    }

    ctrlData.cmd = cmd;
    strncpy(ctrlData.arg.str, session.c_str(), sizeof(ctrlData.arg.str));
    ctrlData.arg.str[sizeof(ctrlData.arg.str) - 1] = '\0';

    break;
  }
  case REC_TRAIN_DATA_STATE_SET_DURATON: {
    uint32_t dur = json["arg"].as<uint32_t>();

    if (dur > std::numeric_limits<long unsigned int>::max()) {
      ctrlData.cmd = UNKNOWN;
      break;
    }

    ctrlData.cmd = cmd;
    ctrlData.arg.lui = dur;

    break;
  }
  case REC_TRAIN_DATA_STATE_SCHED_START_TIME: {
    uint64_t sched = json["arg"].as<uint64_t>();

    if (sched > std::numeric_limits<long long unsigned int>::max()) {
      ctrlData.cmd = UNKNOWN;
    }

    ctrlData.cmd = cmd;
    ctrlData.arg.llui = sched;

    break;
  }
  case SET_STATE: {
    uint8_t state = json["arg"].as<uint8_t>();

    if (state >= STATE_COUNT) {
      ctrlData.cmd = UNKNOWN;
      break;
    }

    ctrlData.cmd = cmd;
    ctrlData.arg.lui = state;

    break;
  }
  default: {
    ctrlData.cmd = cmd;
  }
  }

  return ctrlData;
}

time_t Utilities::POSIXLocalTime() {
  configureTime();
  time_t now = time(NULL);
  return now;
}

esp_err_t Utilities::configureTime() {
  if (checkTimeSync())
    return ESP_OK;

  for (uint8_t i = 0; i < SNTP_MAX_RETRIES; i++) {
    ESP_LOGI(
        TAG_UTIL, "Synchronizing time, attempt %u/%u", i + 1, SNTP_MAX_RETRIES);
    configTime(SNTP_GMT_OFFSET_SEC, SNTP_DAYLIGHT_OFFSET_SEC, SNTP_SERVER);

    if (checkTimeSync())
      return ESP_OK;

    if (i < SNTP_MAX_RETRIES) {
      ESP_LOGE(
          TAG_UTIL,
          "Time sync failed, retrying in %u milliseconds",
          SNTP_RETRY_DELAY);
    } else {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(SNTP_RETRY_DELAY));
  }
  ESP_LOGE(TAG_UTIL, "Time sync failed, exiting%s", "");

  return ESP_FAIL;
}

esp_err_t Utilities::configureWiFi() {
  if (checkWiFiConn()) {
    return ESP_OK;
  }

  cfg.osi_funcs->_task_create = task_create_wrapper;
  cfg.osi_funcs->_task_create_pinned_to_core =
      task_create_pinned_to_core_wrapper;

  for (uint8_t i = 0; i < WIFI_MAX_RETRIES; i++) {
    uint64_t connectStartMS = millis();

    ESP_LOGI(
        TAG_UTIL, "Connecting to WiFi, attempt %u/%u", i + 1, WIFI_MAX_RETRIES);
    WiFi.begin(&cfg, WIFI_SSID, WIFI_PASSWORD);

    while (!checkWiFiConn() &&
           (millis() - connectStartMS <= WIFI_CONNECT_WAIT)) {
      vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (checkWiFiConn())
      return ESP_OK;

    if (i < WIFI_MAX_RETRIES) {
      ESP_LOGE(
          TAG_UTIL,
          "WiFi connection failed, retrying in %u milliseconds",
          WIFI_RETRY_DELAY);
    } else {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(WIFI_RETRY_DELAY));
  }
  ESP_LOGE(TAG_UTIL, "WiFi connection failed, exiting%s", "");

  return ESP_FAIL;
}

esp_err_t Utilities::configureMQTT(const String &id, MQTT_CALLBACK_SIGNATURE) {
  if (checkMQTTConn()) {
    return ESP_OK;
  }

  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);

  for (uint8_t i = 0; i <= MQTT_MAX_RETRIES; i++) {
    ESP_LOGI(
        TAG_UTIL, "Connecting to MQTT, attempt %u/%u", i, MQTT_MAX_RETRIES);

    client.connect(id.c_str(), MQTT_USERNAME, MQTT_PASSWORD);

    if (checkMQTTConn()) {
      return ESP_OK;
    }

    if (i <= MQTT_MAX_RETRIES) {
      ESP_LOGE(
          TAG_UTIL,
          "MQTT connection failed, retrying in %u milliseconds",
          MQTT_RETRY_DELAY);
    } else {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(MQTT_RETRY_DELAY));
  }
  ESP_LOGE(TAG_UTIL, "MQTT connection failed, exiting%s", "");

  return ESP_FAIL;
}

esp_err_t Utilities::MQTTPublish(const String &topic, const JsonDocument &doc) {
  char jsonMessage[MQTT_MAX_PAYLOAD_SIZE];
  serializeJson(doc, jsonMessage);

  if (client.publish(topic.c_str(), jsonMessage)) {
    return ESP_OK;
  } else {
    ESP_LOGE(TAG_UTIL, "Failed to publish message%s", "");
    return ESP_FAIL;
  }
}

esp_err_t Utilities::MQTTSubscribe(const String &topic) {
  if (client.subscribe(topic.c_str())) {
    ESP_LOGI(
        TAG_UTIL, "MQTT successfully subscribed to %s topic", topic.c_str());
    return ESP_OK;
  }
  ESP_LOGE(TAG_UTIL, "MQTT failed to subscribe to %s topic", topic.c_str());

  return ESP_FAIL;
}

esp_err_t Utilities::MQTTUnsubscribe(const String &topic) {
  if (client.unsubscribe(topic.c_str())) {
    ESP_LOGI(
        TAG_UTIL,
        "MQTT successfully unsubscribed from %s topic",
        topic.c_str());
    return ESP_OK;
  }
  ESP_LOGE(TAG_UTIL, "MQTT failed to unsubscribe from %s topic", topic.c_str());

  return ESP_FAIL;
}

bool Utilities::MQTTRun() { return client.loop(); }