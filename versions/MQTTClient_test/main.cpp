#include <sstream>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include "soc/soc_caps.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <Arduino.h>
#include <HardwareSerial.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>

#include "MeasurementDatatypes.h"
#include "MeasurementModule.h"
#include "MeasurementModuleErrors.h"
#include "MeasurementTask.h"

#include "EventSystem.hpp"

#include "Adafruit_NeoPixel.h"
#include "bme68xLibrary.h"
#include "bme69xLibrary.h"

#include "BME6xxManager.h"
#include "BME6xxManagerErrors.h"
#include "FSManager.h"

#include "AppDatatypes.hpp"
#include "AppModules/RTCSyncModule.h"
#include "AppModules/StateManagementModule.h"
#include "Config.h"
#include "MQTTClient.h"
#include "Utilities.h"

static const char *TAG = "Main-App";

extern "C" void app_main();

// esp_register_shutdown_handler

static StateManager *stateManager;

uint32_t ledColor = 0;
uint8_t ledBright = 0;
uint32_t ledOnHold = 0;
uint32_t ledOffHold = portMAX_DELAY;
SemaphoreHandle_t ledChangeSem;
TaskHandle_t ledTask;

MQTTClient mqttClient;

Adafruit_NeoPixel ws2812b = Adafruit_NeoPixel(1, WS2812B_PIN, NEO_RGB + NEO_KHZ800);

// TODO: Separate class
void reportStatusLED(esp_err_t err) {
  if (xSemaphoreTake(ledChangeSem, pdMS_TO_TICKS(10000)) == pdTRUE) {
    if (err == ESP_OK) {
      ledColor = ws2812b.Color(0, 255, 0);
      ledBright = 100;
      ledOnHold = 500;
      ledOffHold = 500;
    } else if (err > ESP_ERR_BME_MNGR_BASE && err < ESP_ERR_BME_MNGR_END) {
      // BME manager error
      ledColor = ws2812b.Color(255, 0, 0);
      ledBright = 90;
      ledOnHold = 5000;
      ledOffHold = 1000;
    } else if (err > ESP_ERR_MEASUREMENT_TASK_BASE && err < ESP_ERR_MEASUREMENT_TASK_END) {
      ledColor = ws2812b.Color(255, 0, 0);
      ledBright = 90;
      ledOnHold = 5000;
      ledOffHold = 1000;

      // Measurement module error
    } else if (err > ESP_ERR_EVENT_SYSTEM_BASE && err < ESP_ERR_EVENT_SYSTEM_END) {

      ledColor = ws2812b.Color(255, 0, 0);
      ledBright = 90;
      ledOnHold = 3000;
      ledOffHold = 500;
    } else if (err == ESP_FAIL) {
      ledColor = ws2812b.Color(255, 0, 0);
      ledBright = 90;
      ledOnHold = 500;
      ledOffHold = 500;
    } else if (err == 0x8001) {
      ledColor = ws2812b.Color(255, 255, 0);
      ledBright = 90;
      ledOnHold = 5000;
      ledOffHold = 1000;
    } else {
      ledColor = ws2812b.Color(255, 255, 0);
      ledBright = 90;
      ledOnHold = 500;
      ledOffHold = 500;
    }
    xSemaphoreGive(ledChangeSem);
    xTaskNotifyGive(ledTask);
  }
}

// #region STATE CONTROL

void initStateManagement() {
  stateManager = StateManager::getInstance();
  if (stateManager == nullptr) {
    ESP_LOGE(TAG, "stateManager is null");
    return;
  }
  ESP_LOGI(TAG, "stateManager initialized%s", "");

  stateManager->run();
}

// #endregion

void initSerial() {
  Serial.begin(115200);

  while (!Serial)
    vTaskDelay(pdMS_TO_TICKS(100));
  vTaskDelay(pdMS_TO_TICKS(1000));
  ESP_LOGI(TAG, "Serial Initialized");
}

void initMQTT() {
  esp_err_t error = ESP_OK;

  error = mqttClient.start();
  if (error != ESP_OK) {
    ESP_LOGE(TAG, "Error starting MQTT client, %u", error);
    return;
  }

  MQTTAddressConfig addrCfg = {
      .hostname = MQTT_SERVER,
      .transport = MQTT_TRANSPORT_OVER_TCP,
      .path = "/",
      .port = MQTT_PORT,
  };
  error = mqttClient.setAddressCfg(&addrCfg);
  if (error != ESP_OK) {
    ESP_LOGE(TAG, "Error setting MQTT address config, %u", error);
    return;
  }

  MQTTCredentialsConfig credCfg = {
      .username = MQTT_USERNAME,
      .client_id = DEVICE_ID,
  };
  error = mqttClient.setCredentialsCfg(&credCfg);
  if (error != ESP_OK) {
    ESP_LOGE(TAG, "Error setting MQTT credentials config, %u", error);
    return;
  }

  MQTTAuthenticationConfig authCfg = {
      .password = MQTT_PASSWORD,
  };
  error = mqttClient.setAuthenticationCfg(&authCfg);
  if (error != ESP_OK) {
    ESP_LOGE(TAG, "Error setting MQTT authentication config, %u", error);
    return;
  }

  error = mqttClient.connect(true, 15000);
  if (error != ESP_OK) {
    ESP_LOGE(TAG, "Error connecting to MQTT broker, %u", error);
    return;
  }

  error = mqttClient.subscribe(MQTT_TOPIC_CTRL_PATH, 0);
  error = mqttClient.subscribe(MQTT_TOPIC_CLUSTER_CTRL_PATH, 0);
  if (error != ESP_OK) {
    ESP_LOGE(TAG, "Error subscribing to MQTT topics, %u", error);
    return;
  }
}

void initWiFi() {
  esp_err_t res = ESP_OK;

  res = Utilities::configureWiFi();
  if (res != ESP_OK)
    return;
}
void mqttDataSendTask(void *pvParameter) {
  float delay = std::abs(MQTT_DATA_SEND_FREQ) == 0 ? 0 : 1000 / std::abs(MQTT_DATA_SEND_FREQ);
  std::stringstream ss;
  while (1) {
    ss.str("");
    ss.clear();

    ss << "Time: " << esp_timer_get_time();

    mqttClient.publish(MQTT_TOPIC_DATA_PATH, ss.str().c_str(), 2, 0);

    vTaskDelay(pdMS_TO_TICKS(delay));
  }
}

// #region WS2812 RELATED
void initWS2812b(void) {
  ws2812b.begin();
  ws2812b.show();
  ESP_LOGI(TAG, "WS2812b Initialized");
  vTaskDelay(pdMS_TO_TICKS(1000));
}
void ws2812b_cycler(void *pvParameter) {
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  while (true) {
    xSemaphoreTake(ledChangeSem, portMAX_DELAY);

    ws2812b.setPixelColor(0, ws2812b.Color(0, 0, 0));
    ws2812b.show();

    vTaskDelay(pdMS_TO_TICKS(ledOffHold));

    ws2812b.setBrightness(ledBright);
    ws2812b.setPixelColor(0, ledColor);
    ws2812b.show();

    vTaskDelay(pdMS_TO_TICKS(ledOnHold));
    xSemaphoreGive(ledChangeSem);
  }
}
// #endregion

void app_main() {
  initArduino();
  vTaskDelay(pdMS_TO_TICKS(1000));

  esp_log_level_set("gpio", ESP_LOG_ERROR);
  esp_log_level_set("wifi", ESP_LOG_ERROR);
  esp_log_level_set("wifi_init", ESP_LOG_ERROR);
  esp_log_level_set("phy_init", ESP_LOG_ERROR);

  initStateManagement();

  initWS2812b();

  ledChangeSem = xSemaphoreCreateMutex();
  if (ledChangeSem == NULL)
    ESP_LOGE(TAG, "Failed to create mutex");

  xTaskCreatePinnedToCore(&ws2812b_cycler, "led", 2048, NULL, 3, &ledTask, 0);

  initWiFi();
  initMQTT();

  xTaskCreatePinnedToCore(&mqttDataSendTask, "mqttTask", 8196, NULL, 10, NULL, 1);
}