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

#include "Adafruit_INA260.h"
#include "Adafruit_NeoPixel.h"
#include "BSEC3.h"
#include "SparkFun_ADXL345.h"
#include "bme68xLibrary.h"
#include "bme69xLibrary.h"
#include "bsecConfig/bsec_iaq.h"
// #include "commMux.h"

#include "BME6xxManager.h"
#include "BME6xxManagerErrors.h"
#include "FSManager.h"

#include "AppDatatypes.hpp"
// #include "AppModules/ClockSyncModule.h"
#include "AppModules/RTCSyncModule.h"
#include "AppModules/StateManagementModule.h"
#include "Config.h"
#include "Utilities.h"

static const char *TAG = "Main-App";

extern "C" void app_main();

// esp_register_shutdown_handler

static StateManager *stateManager;

RTCSyncModule rtc;

uint64_t timeStartS = 0;

char tdcLabel[TDC_LABEL_LENGTH] = "";
char tdcSession[TDC_SESSION_LENGTH] = "";
uint32_t tdcDurationSec = 300U;
uint64_t tdcSchedStartTimeSec = 0U;
uint64_t rtcToESPOffset = 0U;
bool tdcScheduled = false;
TaskHandle_t tdcDurationTrackerTaskHandle;

uint32_t ledColor = 0;
uint8_t ledBright = 0;
uint32_t ledOnHold = 0;
uint32_t ledOffHold = portMAX_DELAY;
SemaphoreHandle_t ledChangeSem;
TaskHandle_t ledTask;

FSManager fsManager;

MeasurementModule measurementModule;
MeasurementTask bmeMT;
MeasurementTask bsecMT;
MeasurementTask *adxl345MT;
MeasurementTask *ina260MT;

EventProducer<MeasurementTaskEvent> pwrTeleProducer{
    MEASUREMENT_TASK_TOTAL_EVENTS,
    DEFAULT_ESWAITS_CONFIG};
EventProducer<MeasurementTaskEvent> voltLevelDetectProducer{
    MEASUREMENT_TASK_TOTAL_EVENTS,
    DEFAULT_ESWAITS_CONFIG};
EventProducer<MeasurementTaskEvent> accelLevelDetectProducer{
    MEASUREMENT_TASK_TOTAL_EVENTS,
    DEFAULT_ESWAITS_CONFIG};

EventConsumer<MeasurementTaskEvent>
    MQTTConsumer{512, RINGBUF_TYPE_NOSPLIT, DEFAULT_ESWAITS_CONFIG};
EventConsumer<MeasurementTaskEvent>
    voltLevelDetectConsumer{512, RINGBUF_TYPE_NOSPLIT, DEFAULT_ESWAITS_CONFIG};
EventConsumer<MeasurementTaskEvent>
    pwrTeleConsumer{512, RINGBUF_TYPE_NOSPLIT, DEFAULT_ESWAITS_CONFIG};
EventConsumer<MeasurementTaskEvent>
    accelLevelDetectConsumer{512, RINGBUF_TYPE_NOSPLIT, DEFAULT_ESWAITS_CONFIG};

SPIClass SPIBus2(FSPI); // Specific pins
SPIClass SPIBus3(HSPI); // Any pins

Adafruit_NeoPixel ws2812b =
    Adafruit_NeoPixel(1, WS2812B_PIN, NEO_RGB + NEO_KHZ800);
Adafruit_INA260 ina260 = Adafruit_INA260();
ADXL345 adxl345;

// Make sure that the bme6xxManager sampling isn't running together with bsec,
// otherwise there will be a confilct in sensor scheduling
BME6xxManager bme6xxManager;
BME69x bme690_1;
BME69x bme690_2;

#define BME_NUM_OF_SENS 1U
#define BSEC_NUM_OF_OUTPUTS 15U
#define BSEC_SAMPLE_FREQ BSEC_SAMPLE_RATE_LP
BSEC3 bsecInstances[BME_NUM_OF_SENS];
uint8_t bsecMemBlocks[BME_NUM_OF_SENS][BSEC_INSTANCE_SIZE];
bsecVirtualSensor bsecSensorOutputs[BSEC_NUM_OF_OUTPUTS] = {
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_TVOC_EQUIVALENT,

    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    BSEC_OUTPUT_COMPENSATED_GAS,
    BSEC_OUTPUT_GAS_PERCENTAGE,

    BSEC_OUTPUT_STABILIZATION_STATUS,
    BSEC_OUTPUT_RUN_IN_STATUS,
};

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
    } else if (
        err > ESP_ERR_MEASUREMENT_TASK_BASE &&
        err < ESP_ERR_MEASUREMENT_TASK_END) {
      ledColor = ws2812b.Color(255, 0, 0);
      ledBright = 90;
      ledOnHold = 5000;
      ledOffHold = 1000;

      // Measurement module error
    } else if (
        err > ESP_ERR_EVENT_SYSTEM_BASE && err < ESP_ERR_EVENT_SYSTEM_END) {

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
// State Normal
esp_err_t stateNormalEnter() {
  esp_err_t err = ESP_OK;

  // err = measurementModule.startMT(ina260MT->getConfiguration().pcName);
  // if (err != ESP_OK) {
  //   ESP_LOGE(TAG, "Failed to start ina260MT");
  //   return err;
  // }

  // err = measurementModule.startMT(adxl345MT->getConfiguration().pcName);
  // if (err != ESP_OK) {
  //   ESP_LOGE(TAG, "Failed to start adxl345MT");
  //   return err;
  // }

  // err = measurementModule.startMT(bsecMT->getConfiguration().pcName);
  // if (err != ESP_OK) {
  //   ESP_LOGE(TAG, "Failed to start bsecMT");
  //   return err;
  // }
  return err;
}
esp_err_t stateNormalExit() {
  esp_err_t err = ESP_OK;
  // err = measurementModule.stopMT(ina260MT->getConfiguration().pcName);
  // if (err != ESP_OK) {
  //   ESP_LOGE(TAG, "Failed to stop ina260MT");
  //   return err;
  // }

  // err = measurementModule.stopMT(adxl345MT->getConfiguration().pcName);
  // if (err != ESP_OK) {
  //   ESP_LOGE(TAG, "Failed to stop adxl345MT");
  //   return err;
  // }

  // err = measurementModule.stopMT(bsecMT->getConfiguration().pcName);
  // if (err != ESP_OK) {
  //   ESP_LOGE(TAG, "Failed to stop bsecMT");
  //   return err;
  // }
  return err;
}
// State Normal

// State Idle
esp_err_t stateIdleEnter() {
  bmeMT.stop();

  return ESP_OK;
}
esp_err_t stateIdleExit() { return ESP_OK; }
// State Idle

// State Train Data Collection
esp_err_t stateTrainDataCollectionEnter() {
  esp_err_t err = ESP_OK;

  ESP_LOGE(TAG, "About to start a MT");
  // err = measurementModule.startMT(bmeMT->getConfiguration().pcName);
  err = bmeMT.start();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start bmeMT");
    return err;
  }
  ESP_LOGE(TAG, "Started a MT");

  // err = measurementModule.startMT(ina260MT->getConfiguration().pcName);
  // if (err != ESP_OK) {
  //   ESP_LOGE(TAG, "Failed to start ina260MT");
  //   return err;
  // }

  ESP_LOGE(TAG, "About to notify a task");
  xTaskNotifyGive(tdcDurationTrackerTaskHandle);
  ESP_LOGE(TAG, "Notified a task");

  return err;
}
esp_err_t stateTrainDataCollectionExit() {
  esp_err_t err = ESP_OK;

  // err = measurementModule.stopMT(bmeMT->getConfiguration().pcName);
  err = bmeMT.stop();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to stop bmeMT");
    return err;
  }

  // err = measurementModule.stopMT(ina260MT->getConfiguration().pcName);
  // if (err != ESP_OK) {
  //   ESP_LOGE(TAG, "Failed to stop ina260MT");
  //   return err;
  // }

  strncpy((char *)"", tdcLabel, 1);
  tdcLabel[0] = '\0';

  return err;
}
// State Train Data Collection

void initStateManagement() {
  stateManager = StateManager::getInstance();
  if (stateManager == nullptr) {
    ESP_LOGE(TAG, "stateManager is null");
    return;
  }
  ESP_LOGI(TAG, "stateManager initialized%s", "");

  stateManager->registerState(STATE_IDLE, stateIdleEnter, stateIdleExit);
  stateManager->registerState(STATE_NORMAL, stateNormalEnter, stateNormalExit);
  stateManager->registerState(
      STATE_TRAIN_DATA_COLLECTION,
      stateTrainDataCollectionEnter,
      stateTrainDataCollectionExit);
  ESP_LOGI(TAG, "States registered%s", "");

  stateManager->run();
}

// #endregion

// #region INIT SYSTEM PERIPHERALS
void initSerial() {
  Serial.begin(115200);

  while (!Serial)
    vTaskDelay(pdMS_TO_TICKS(100));
  vTaskDelay(pdMS_TO_TICKS(1000));
  ESP_LOGI(TAG, "Serial Initialized");
}
void initI2C() {
  Wire.end();
  if (!Wire.begin(I2C_SDA_BUS0, I2C_SCL_BUS0, I2C_FREQ_BUS0)) {
    ESP_LOGE(TAG, "I2C Bus 0 init failed.");
    reportStatusLED(ESP_FAIL);
    return;
  }

  if (!Wire1.begin(I2C_SDA_BUS1, I2C_SCL_BUS1, I2C_FREQ_BUS1)) {
    ESP_LOGE(TAG, "I2C Bus 1 init failed.");
    reportStatusLED(ESP_FAIL);
    return;
  }
  ESP_LOGI(TAG, "I2C Initialized");
  vTaskDelay(pdMS_TO_TICKS(1000));
}
void initSPI() {
  SPIBus2.begin();
  ESP_LOGI(TAG, "SPI Bus 2 Initialized");

  SPIBus3.begin(SPI_SCLK_BUS3, SPI_MISO_BUS3, SPI_MOSI_BUS3);
  ESP_LOGI(TAG, "SPI Bus 3 Initialized");

  vTaskDelay(pdMS_TO_TICKS(1000));
}
// #endregion

void initRTC() {
  esp_err_t err = ESP_OK;

  err = rtc.begin(&Wire);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "RTC initialization failed");
    return;
  }

  if (!rtc.isRunning()) {
    ESP_LOGE(TAG, "RTC is not running, time is not configured");
    return;
  }

  timeStartS = rtc.getRTCUnixTime();
  rtcToESPOffset = timeStartS * 1e6 - esp_timer_get_time();
}

// #region INIT FILE SYSTEM
bool initLittleFS() {
  if (!LittleFS.begin(true, "/littlefs", 10, "littlefs")) {
    ESP_LOGE(TAG, "LittleFS init failed");
    reportStatusLED(ESP_FAIL);
    return false;
  }

  ESP_LOGI(TAG, "LittleFS Initialized");
  vTaskDelay(pdMS_TO_TICKS(1000));

  return true;
}
bool initSD() {
  if (!SD.begin(SPI_SD_CS_PIN, SPIBus2, SPI_FREQ_BUS2, "/", 15, true)) {
    ESP_LOGE(TAG, "SD init failed");
    reportStatusLED(ESP_FAIL);
    return false;
  }

  ESP_LOGI(TAG, "SD Initialized");
  vTaskDelay(pdMS_TO_TICKS(1000));

  return true;
}
void initFSManager(file_system_type fsType) {
  esp_err_t error = ESP_OK;

  switch (fsType) {
  case FS_MANAGER_LITTLE_FS: {
    if (!initLittleFS())
      return;

    error = fsManager.initializeFileSystem(LittleFS, FS_MANAGER_LITTLE_FS);
    break;
  }
  case FS_MANAGER_SD: {
    if (!initSD())
      return;

    error = fsManager.initializeFileSystem(SD, FS_MANAGER_SD);
    break;
  }
  default: {
    ESP_LOGE(TAG, "Invalid FS type");
    break;
  }
  }

  if (error != ESP_OK) {
    ESP_LOGE(TAG, "FSManager init fail");
    reportStatusLED(ESP_FAIL);
    return;
  }

  ESP_LOGI(TAG, "FSManager Initialized");
  vTaskDelay(1000);
}
// #endregion

// #region MQTT RELATED
void MQTTCTRLcallback(char *topic, byte *payload, unsigned int length) {
  byte copiedData[length];
  memcpy(copiedData, payload, length);

  std::string message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  JsonDocument doc;

  DeserializationError error = deserializeJson(doc, message);
  if (error) {
    ESP_LOGE(TAG, "deserializeJson() failed, error: %s", error.c_str());
    return;
  }

  CTRLData ctrlData = Utilities::jsonToCTRLData(doc);

  esp_err_t err = ESP_OK;
  switch (ctrlData.cmd) {
  case REC_TRAIN_DATA_STATE_SET_FREQ: {
    err = bmeMT.setSamplingRate(ctrlData.arg.f);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "REC_TRAIN_DATA_STATE_SET_FREQ setSamplingRate failed");
      return;
    }
    break;
  }
  case REC_TRAIN_DATA_STATE_SET_LABEL: {
    strncpy(tdcLabel, ctrlData.arg.str, sizeof(tdcLabel));
    break;
  }
  case REC_TRAIN_DATA_STATE_SET_SESSION: {
    strncpy(tdcSession, ctrlData.arg.str, sizeof(tdcSession));
    break;
  }
  case REC_TRAIN_DATA_STATE_SET_DURATON: {
    tdcDurationSec = ctrlData.arg.lui;
    break;
  }
  case REC_TRAIN_DATA_STATE_SCHED_START_TIME: {
    tdcSchedStartTimeSec = ctrlData.arg.llui;
    tdcScheduled = true;
    stateManager->requestTransition(AppState::STATE_TRAIN_DATA_COLLECTION);
    break;
  }
  case SET_STATE: {
    ESP_LOGI(
        TAG, "MQTT state transition request, received data length: %u", length);
    stateManager->requestTransition((AppState)ctrlData.arg.lui);
    break;
  }
  default: {
    ESP_LOGW(
        TAG, "Unknown command, length: %u, command: %i", length, ctrlData.cmd);
    break;
  }
  };
}
void initWiFiTimeMQTT() {
  esp_err_t res = ESP_OK;

  res = Utilities::configureWiFi();
  if (res != ESP_OK)
    return;

  res = Utilities::configureMQTT(DEVICE_ID, MQTTCTRLcallback);
  if (res != ESP_OK)
    return;

  res = Utilities::MQTTSubscribe(MQTT_TOPIC_CTRL_PATH);
  if (res != ESP_OK)
    return;
  res = Utilities::MQTTSubscribe(MQTT_TOPIC_CLUSTER_CTRL_PATH);
  if (res != ESP_OK)
    return;
}
void mqttLoopTask(void *pvParameter) {
  while (1) {
    if (!Utilities::MQTTRun())
      initWiFiTimeMQTT();
    vTaskDelay(pdMS_TO_TICKS(1000U / MQTT_LOOP_POLL_FREQ));
  }
}
void mqttDataSendTask(void *pvParameter) {
  float delay = std::abs(MQTT_DATA_SEND_FREQ) == 0
                    ? 0
                    : 1000 / std::abs(MQTT_DATA_SEND_FREQ);

  JsonDocument doc;
  JsonObject root = doc.to<JsonObject>();

  MQTTDataContainer mqttData(DEVICE_ID, CLUSTER_ID);
  while (1) {
    while (mqttData.objects.size() < MQTT_MAX_AGGREGATE_PACKETS) {
      struct EventDescriptor<MeasurementTaskEvent> *item;
      MQTTConsumer.listenForEvents(
          sizeof(struct EventDescriptor<MeasurementTaskEvent>),
          &item,
          portMAX_DELAY,
          portMAX_DELAY);

      if (item) {
        if (item->event == MEASUREMENT_TASK_DATA_UPDATE_EVENT) {
          if (item->data) {
            auto *data = static_cast<MeasurementBase *>(item->data);

            std::unique_ptr<MeasurementBase> ptr(std::move(data));

            mqttData.addObject(std::move(ptr));

            item->data = nullptr;
          }
        }
        delete item;
      }
    }

    if (mqttData.objects.size()) {
      mqttData.label = std::string(tdcLabel);
      mqttData.session = std::string(tdcSession);
      mqttData.prepareJSONRepresentation(root);
      mqttData.setTimeposix(rtc.getRTCUnixTime()); // Time here
      if (Utilities::MQTTPublish(MQTT_TOPIC_DATA_PATH, doc) != ESP_OK) {
        initWiFiTimeMQTT();
      }
      mqttData.eraseObjects();
      doc.clear();
    }

    vTaskDelay(pdMS_TO_TICKS(delay));
  }
}
// #endregion

// #region Train Data Collection
void tdcDurationTrackerTask(void *pvParameter) {
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(tdcDurationSec * 1000));
    ESP_LOGI(TAG, "tdcDurationTrackerTask state transition request");
    stateManager->requestTransition(STATE_IDLE);
  }
}
// #endregion

void monitor_all_tasks() {
  char *buffer = (char *)malloc(1024);
  if (buffer) {
    vTaskList(buffer);
    ESP_LOGI("TASKS", "\nTask Name\tState\tPrio\tStack\tNum\n%s", buffer);
    free(buffer);
  }
}

void bme6xxCalibrationTask(void *pvParameter) {}

// #region BSEC2 RELATED
esp_err_t bsecMeasurementFunc(MeasurementProducerHelper &eventProducer) {
  const bsecOutputs *outputs;
  BME6xxData bmeData;
  esp_err_t err = ESP_OK;

  if (tdcScheduled) {
    uint64_t timeNowUS = esp_timer_get_time() + rtcToESPOffset;
    uint64_t tdcSchedStartTimeUS = tdcSchedStartTimeSec * 1000000ULL;

    if (tdcSchedStartTimeUS < timeNowUS) {
      ESP_LOGE(
          TAG,
          "Scheduled time is invalid; Scheduled: %llu; Now: %llu",
          tdcSchedStartTimeUS,
          timeNowUS);
      reportStatusLED(ESP_ERR_TIME_SCHEDULE_INVALID);

      return ESP_ERR_TIME_SCHEDULE_INVALID;
    }

    uint64_t sleepTimeUS = (tdcSchedStartTimeUS - timeNowUS);

    ESP_LOGI(
        TAG,
        "Scheduled time: %llu; Time now: %llu; Sleep time: %llu",
        tdcSchedStartTimeUS,
        timeNowUS,
        sleepTimeUS);

    vTaskDelay(pdMS_TO_TICKS(sleepTimeUS / 1000));

    tdcScheduled = false;
  }

  for (uint8_t i = 0; i < BME_NUM_OF_SENS; i++) {
    if (!bsecInstances[i].run()) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    outputs = bsecInstances[i].getBSECOutputs();
    bmeData = bsecInstances[i].getBMEData();

    if (outputs && outputs->nOutputChnls) {
      BME6XXData newEventData;
      for (uint8_t i = 0; i < outputs->nOutputChnls; i++) {
        newEventData.timestamp = bmeData.meas_timestamp;
        switch (outputs->outputChnls[i].sensor_id) {
        case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE: {
          newEventData.temperature = outputs->outputChnls[i].signal;
          break;
        }
        case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY: {
          newEventData.humidity = outputs->outputChnls[i].signal;
          break;
        }
        case BSEC_OUTPUT_COMPENSATED_GAS: {
          newEventData.compensGasResistance = outputs->outputChnls[i].signal;
          break;
        }
        case BSEC_OUTPUT_RAW_GAS: {
          newEventData.gasResistance = outputs->outputChnls[i].signal;
          break;
        }
        case BSEC_OUTPUT_STABILIZATION_STATUS: {
          newEventData.sensorStable = outputs->outputChnls[i].signal;
          break;
        }
        case BSEC_OUTPUT_RUN_IN_STATUS: {
          newEventData.sensorRunIn = outputs->outputChnls[i].signal;
          break;
        }
        case BSEC_OUTPUT_RAW_PRESSURE: {
          newEventData.pressure = outputs->outputChnls[i].signal;
          break;
        }
        case BSEC_OUTPUT_CO2_EQUIVALENT: {
          newEventData.co2Eq = outputs->outputChnls[i].signal;
          break;
        }
        case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT: {
          newEventData.breathVOC = outputs->outputChnls[i].signal;
          break;
        }
        case BSEC_OUTPUT_IAQ: {
          newEventData.iaq = outputs->outputChnls[i].signal;
          break;
        }
        case BSEC_OUTPUT_STATIC_IAQ: {
          newEventData.staticIaq = outputs->outputChnls[i].signal;
          break;
        }
        case BSEC_OUTPUT_TVOC_EQUIVALENT: {
          newEventData.tvoc = outputs->outputChnls[i].signal;
          break;
        }
        default: {
        }
        }
      }

      newEventData.sensorId = i;
      newEventData.size = sizeof(newEventData);

      // ESP_LOGI(
      //     TAG,
      //     "Temperature: %f; Humidity: %f; Gas: %f; Pressure: %f; CO2 "
      //     "Equivalent: %f; "
      //     "Breath VOC Equivalent: %f; IAQ: %f; Static IAQ: %f; TVOC "
      //     "Equivalent: %f; Run-In Status: %u; Stable: %u; Timestamp: %llu",
      //     newEventData.temperature,
      //     newEventData.humidity,
      //     newEventData.gasResistance,
      //     newEventData.pressure,
      //     newEventData.co2Eq,
      //     newEventData.breathVOC,
      //     newEventData.iaq,
      //     newEventData.staticIaq,
      //     newEventData.tvoc,
      //     newEventData.sensorRunIn,
      //     newEventData.sensorStable,
      //     newEventData.timestamp);

      err = eventProducer.produceMeasurement(
          (void *)&newEventData, newEventData.size);
    }
    vTaskDelay(pdMS_TO_TICKS(30));
  }
  return err;
}

esp_err_t bmeMTPostStopHook();

void initBSEC() {
  auto sensors = bme6xxManager.getSensors();

  if (sensors.empty()) {
    ESP_LOGE(TAG, "BME6xxManager no sensors");
    return;
  }

  for (uint8_t i = 0; i < BME_NUM_OF_SENS; i++) {
    bsecInstances[i].allocateMemory(bsecMemBlocks[i]);

    if (!bsecInstances[i].begin(*sensors[i], i)) {
      ESP_LOGE(TAG, "BSEC3 begin failed%s", "");
      return;
    }

    bsecInstances[i].setTemperatureOffset(BSEC_SAMPLE_FREQ);
    bsecInstances[i].setTVOCBaselineCalibration(false);

    if (!bsecInstances[i].setConfig(bsec_config_iaq)) {
      ESP_LOGE(TAG, "BSEC3 setConfig failed: %i", bsecInstances[i].status);
      return;
    }

    if (!bsecInstances[i].updateSubscription(
            bsecSensorOutputs, BSEC_NUM_OF_OUTPUTS, BSEC_SAMPLE_FREQ)) {
      ESP_LOGE(
          TAG, "BSEC3 updateSubscription failed: %i", bsecInstances[i].status);
      return;
    }
  }

  MeasurementTaskConfig taskConfig{
      .sampleRate = CONFIG_TELE_BSEC_SAMPLING_FREQUENCY,
      .usStackDepth = 6144U,
      .xCoreID = 0U,
      .uxPriority = 5U,
      .pcName = (char *)"BSEC",
      .sendWaitTicks = pdMS_TO_TICKS(5000),
      .accessWaitTicks = pdMS_TO_TICKS(1000),
      .receiveWaitTicks = pdMS_TO_TICKS(10000),
  };

  esp_err_t err = ESP_OK;

  err = bsecMT.registerHook(bmeMTPostStopHook, MT_POST_STOP_HOOK);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "bsec registerHook failed");
    return;
  }

  err = bsecMT.setMeasurementTask(bsecMeasurementFunc);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "bsecMT setMeasurementTask failed");
    return;
  }
  err = bsecMT.configure(taskConfig);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "bsecMT configure failed");
    return;
  }

  ESP_LOGI(TAG, "BSEC Initialized");
  vTaskDelay(pdMS_TO_TICKS(1000));
}
// #endregion

// #region BME6XX RELATED
esp_err_t bmeMTPostStopHook() {
  esp_err_t err = ESP_OK;

  ESP_LOGI(TAG, "Executed post stop hook");

  err = bme6xxManager.sleepAll();

  // bme6xxManager.resetSensors();
  return err;
}
esp_err_t bmeMTPreStartHook() {
  esp_err_t err = ESP_OK;
  // bme6xxManager.loadConfig(FSFile & configFile);

  // bme6xxManager.configure();
  // bme6xxManager.begin();

  return err;
}

esp_err_t bme6xxMeasurementFunc(MeasurementProducerHelper &eventProducer) {
  esp_err_t err = ESP_OK;

  if (tdcScheduled) {
    uint64_t timeNowUS = esp_timer_get_time() + rtcToESPOffset;
    uint64_t tdcSchedStartTimeUS = tdcSchedStartTimeSec * 1000000ULL;

    if (tdcSchedStartTimeUS < timeNowUS) {
      ESP_LOGE(
          TAG,
          "Scheduled time is invalid; Scheduled: %llu; Now: %llu",
          tdcSchedStartTimeUS,
          timeNowUS);
      reportStatusLED(ESP_ERR_TIME_SCHEDULE_INVALID);

      return ESP_ERR_TIME_SCHEDULE_INVALID;
    }

    uint64_t sleepTimeUS = (tdcSchedStartTimeUS - timeNowUS);

    ESP_LOGI(
        TAG,
        "Scheduled time: %llu; Time now: %llu; Sleep time: %llu",
        tdcSchedStartTimeUS,
        timeNowUS,
        sleepTimeUS);

    vTaskDelay(pdMS_TO_TICKS(sleepTimeUS / 1000));

    tdcScheduled = false;
  }

  BMESensorData sensorData{};
  while (bme6xxManager.scheduleSensor()) {
    err = bme6xxManager.collectData(sensorData);

    if (err == ESP_ERR_SENSOR_NO_NEW_DATA) {
      err = ESP_OK;
    }

    // reportStatusLED(err);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Error: %i", err);
      return err;
    }

    for (uint8_t i = 0; i < sensorData.dataLen; i++) {
      uint64_t timestampNow = esp_timer_get_time();
      timeStartS = rtc.getRTCUnixTime();

      BME6XXData newEventData{};
      newEventData.gasIndex = sensorData.data[i].gas_index;
      newEventData.gasResistance = sensorData.data[i].gas_resistance;
      newEventData.humidity = sensorData.data[i].humidity;
      newEventData.pressure = (sensorData.data[i].pressure * 0.01f);
      newEventData.temperature = sensorData.data[i].temperature;
      newEventData.sensorId = sensorData.sensorId;
      newEventData.timestamp =
          (timeStartS * 1000000U) -
          (timestampNow - sensorData.data[i].meas_timestamp);
      newEventData.sensorStable =
          (sensorData.data[i].status & HEAT_STAB_MSK) ? 1 : 0;
      newEventData.size = sizeof(newEventData);

      // ESP_LOGI(
      //     TAG,
      //     "Sensor ID: %lu; Heater index: %u; Gas resistance: %f; Humidity:
      //     %f; " "Pressure: %f; Temperature: %f; Timestamp: %llu",
      //     newEventData.sensorId,
      //     newEventData.gasIndex,
      //     newEventData.gasResistance,
      //     newEventData.humidity,
      //     newEventData.pressure,
      //     newEventData.temperature,
      //     newEventData.timestamp);

      ESP_LOGI(TAG, "Sending measurement");

      err = eventProducer.produceMeasurement(
          (void *)&newEventData, newEventData.size);

      if (err != ESP_OK) {
        reportStatusLED(err);
        return err;
      }
    }
  }

  vTaskDelay(pdMS_TO_TICKS(10));

  return err;
}
void initBME6xx_n() {
  int8_t res = 0;
  // bme688_1.begin(0x77, Wire);
  // res = bme688_1.checkStatus();
  // if (res != BME68X_OK) {
  //   ESP_LOGE(TAG, "Error initializing bme688_1 sensor, code: %i", res);
  //   reportStatusLED(ESP_ERR_BME6XX_DRIVER_ERROR);
  //   return;
  // }
  // ESP_LOGI(TAG, "bme688_1 Initialized");

  // bme688_2.begin(0x77, Wire1);
  // res = bme688_2.checkStatus();
  // if (res != BME68X_OK) {
  //   ESP_LOGE(TAG, "Error initializing bme688_2 sensor, code: %i", res);
  //   reportStatusLED(ESP_ERR_BME6XX_DRIVER_ERROR);
  //   return;
  // }
  // ESP_LOGI(TAG, "bme688_2 Initialized");

  bme690_1.begin(0x76, Wire);
  res = bme690_1.checkStatus();
  if (res != BME69X_OK) {
    ESP_LOGE(TAG, "Error initializing bme690_1 sensor, code: %i", res);
    return;
  }
  ESP_LOGI(TAG, "bme690_1 Initialized");

  // bme690_2.begin(0x76, Wire1);
  // res = bme690_2.checkStatus();
  // if (res != BME69X_OK) {
  //   ESP_LOGE(TAG, "Error initializing bme690_2 sensor, code: %i", res);
  //   return;
  // }
  // ESP_LOGI(TAG, "bme690_2 Initialized");

  // comm_mux_beginI2C(Wire);

  // for (uint8_t i = 1; i < NUM_BME6XX_UNITS; i++) {
  //   comm_setup[i] = comm_mux_set_config(Wire, SPIBus3, i, comm_setup[i]);

  //   bme688_n[i].begin(
  //       BME68X_SPI_INTF,
  //       comm_mux_read,
  //       comm_mux_write,
  //       comm_mux_delay,
  //       &comm_setup[i]);
  //   res = bme688_n[i].checkStatus();
  //   if (res != BME68X_OK) {
  //     ESP_LOGE(TAG, "Error initializing bme688_%u sensor, code: %i", i, res);
  //     reportStatusLED(ESP_ERR_BME6XX_DRIVER_ERROR);
  //     return;
  //   }
  // }
  vTaskDelay(pdMS_TO_TICKS(1000));
}

void initBMEManager() {
  esp_err_t err = ESP_OK;

  std::unique_ptr<FSFile> bmeCfg(
      fsManager.readFile("/x8default_1sens.bmeconfig"));

  // err = bme6xxManager.addSensor(bme688_1, true);
  // if (err != ESP_OK) {
  //   ESP_LOGE(
  //       TAG, "Failed to add bme688_1 sensor to bme6xxManager, err: %i", err);
  //   reportStatusLED(err);

  //   return;
  // }

  // err = bme6xxManager.addSensor(bme688_2, true);
  // if (err != ESP_OK) {
  //   ESP_LOGE(
  //       TAG, "Failed to add bme688_2 sensor to bme6xxManager, err: %i", err);
  //   reportStatusLED(err);

  //   return;
  // }

  err = bme6xxManager.addSensor(bme690_1, true);
  if (err != ESP_OK) {
    ESP_LOGE(
        TAG, "Failed to add bme690_1 sensor to bme6xxManager, err: %i", err);
    return;
  }

  // err = bme6xxManager.addSensor(bme690_2, true);
  // if (err != ESP_OK) {
  //   ESP_LOGE(
  //       TAG, "Failed to add bme690_2 sensor to bme6xxManager, err: %i", err);
  //   return;
  // }

  err = bme6xxManager.configure(*bmeCfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to load config in bme6xxManager, err: %i", err);
    return;
  }

  err = bme6xxManager.begin();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start bme6xxManager, err: %i", err);
    return;
  }

  MeasurementTaskConfig taskConfig{
      .sampleRate = CONFIG_TELE_BME688_SAMPLING_FREQUENCY,
      .usStackDepth = 8192U,
      .xCoreID = 0U,
      .uxPriority = 5U,
      .pcName = (char *)"BME688",
      .sendWaitTicks = pdMS_TO_TICKS(5000),
      .accessWaitTicks = pdMS_TO_TICKS(1000),
      .receiveWaitTicks = pdMS_TO_TICKS(10000),
  };

  err = bmeMT.registerHook(bmeMTPostStopHook, MT_POST_STOP_HOOK);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "bmeManager registerHook failed");
    return;
  }
  err = bmeMT.setMeasurementTask(bme6xxMeasurementFunc);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "bmeManager setMeasurementTask failed");
    return;
  }
  err = bmeMT.configure(taskConfig);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "bmeManager configure failed");
    return;
  }
  ESP_LOGI(TAG, "bmeManager Initialized");

  vTaskDelay(pdMS_TO_TICKS(1000));
}
// #endregion

// #region INA260 RELATED
esp_err_t INA260MeasurementFunc(MeasurementProducerHelper &eventProducer) {

  INA260Data data;

  data.current = ina260.readCurrent();
  data.voltage = ina260.readBusVoltage();
  data.power = ina260.readPower();
  data.size = sizeof(INA260Data);

  return eventProducer.produceMeasurement((void *)&data, data.size);
}
void pwrTeleTask(void *pvParameter) {
  esp_err_t err = ESP_OK;

  struct EventDescriptor<MeasurementTaskEvent> *item;

  float currentAccumulator = 0;
  uint32_t samplesCount = 0;

  uint32_t timeStartMS = millis();
  uint32_t timeEndMS = timeStartMS + CONFIG_TELE_BAT_CALC_ACCUM_TIME_S * 1000;

  float mAhConsumption = 0;
  float current = 0;
  float voltage = 0;
  while (1) {
    while (pwrTeleConsumer.listenForEvents(
               sizeof(struct EventDescriptor<MeasurementTaskEvent>),
               &item,
               portMAX_DELAY) != ESP_OK) {
    }

    if (item) {
      if (item->event == MEASUREMENT_TASK_DATA_UPDATE_EVENT) {
        INA260Data *data = static_cast<INA260Data *>(item->data);

        if (data) {
          voltage = data->voltage;
          current = data->current;

          currentAccumulator += current;
          samplesCount++;

          uint32_t timeNowMS = millis();
          if (timeNowMS >= timeEndMS) {
            float avgCurrent = currentAccumulator / samplesCount;
            double deltaTime = (timeNowMS - timeStartMS);
            mAhConsumption += float(avgCurrent * (deltaTime / 3600000.0));

            currentAccumulator = 0;
            samplesCount = 0;
            timeStartMS = timeNowMS;
            timeEndMS = timeStartMS + CONFIG_TELE_BAT_CALC_ACCUM_TIME_S * 1000;

            PowerTelemetryData pwrTeleData;
            pwrTeleData.batLevel = mAhConsumption;
            pwrTeleData.voltage = voltage;
            pwrTeleData.timestamp = millis();
            pwrTeleData.size = sizeof(PowerTelemetryData);

            err = pwrTeleProducer.produceEvent(
                &pwrTeleData,
                pwrTeleData.size,
                MEASUREMENT_TASK_DATA_UPDATE_EVENT);

            if (err)
              ESP_LOGE(TAG, "pwrTeleTask failed to produce event");
          }
          if (item->data) {
            free(item->data);
          }
        }
      }
      free(item);
    }
  }
}
void voltLevelDetectTask(void *pvParameter) {
  esp_err_t err = ESP_OK;

  struct EventDescriptor<MeasurementTaskEvent> *item;

  float voltage = 0;
  while (1) {
    while (voltLevelDetectConsumer.listenForEvents(
               sizeof(struct EventDescriptor<MeasurementTaskEvent>),
               &item,
               portMAX_DELAY) != ESP_OK) {
    }

    if (item) {
      if (item->event == MEASUREMENT_TASK_DATA_UPDATE_EVENT) {
        INA260Data *data = static_cast<INA260Data *>(item->data);

        if (data) {
          voltage = data->voltage;
          AlertData newAlert;
          newAlert.alert = ALERT_NONE;
          newAlert.size = sizeof(AlertData);
          newAlert.timestamp = millis();

          if (voltage >= CONFIG_VOLT_LEVEL_THRESHOLD_HIGH) {
            newAlert.alert = ALERT_VOLTAGE_HIGH;
          } else if (voltage <= CONFIG_VOLT_LEVEL_THRESHOLD_LOW) {
            newAlert.alert = ALERT_VOLTAGE_LOW;
          }

          if (newAlert.alert != ALERT_NONE) {
            err = voltLevelDetectProducer.produceEvent(
                &newAlert, newAlert.size, MEASUREMENT_TASK_DATA_UPDATE_EVENT);

            if (err)
              ESP_LOGE(TAG, "voltLevelDetectTask failed to produce event");
          }

          if (item->data) {
            free(item->data);
          }
        }
      }
      free(item);
    }
  }
}
void initINA260(void) {
  bool status = ina260.begin(0x40, &Wire);
  if (!status) {
    ESP_LOGE(TAG, "Unable to initialize the INA260 sensor%s", " ");
    return;
  }

  ina260.setCurrentConversionTime(INA260_TIME_1_1_ms);
  ina260.setVoltageConversionTime(INA260_TIME_1_1_ms);
  ina260.setAveragingCount(INA260_COUNT_1);

  // MeasurementTaskConfig taskConfig{
  //     .sampleRate = CONFIG_TELE_INA260_SAMPLING_FREQUENCY,
  //     .usStackDepth = 2632U,
  //     .xCoreID = 0U,
  //     .uxPriority = 5U,
  //     .pcName = (char *)"INA260",
  //     .sendWaitTicks = pdMS_TO_TICKS(5000),
  //     .accessWaitTicks = pdMS_TO_TICKS(1000),
  //     .receiveWaitTicks = pdMS_TO_TICKS(10000),
  // };

  // ina260MT = measurementModule.addMT(INA260MeasurementFunc, taskConfig);
  // if (ina260MT == nullptr) {
  //   ESP_LOGE(TAG, "INA260 configure failed");
  //   return;
  // }

  ESP_LOGI(TAG, "INA260 Initialized");
  vTaskDelay(pdMS_TO_TICKS(1000));
}
// #endregion

// #region ADXL345 RELATED
esp_err_t ADXL345MeasurementFunc(MeasurementProducerHelper &eventProducer) {
  ADXL345Data data;

  adxl345.readAccel(&data.x, &data.y, &data.z);
  data.size = sizeof(data);

  ESP_LOGI(TAG, "x: %f; y: %f; z: %f ", data.x, data.y, data.z);

  vTaskDelay(pdMS_TO_TICKS(100));

  // eventProducer.produceMeasurement((void *)&data, data.size);

  return ESP_OK;
}
void accelLevelDetectTask(void *pvParameter) {
  esp_err_t err = ESP_OK;

  struct EventDescriptor<MeasurementTaskEvent> *item;
  while (1) {
    while (accelLevelDetectConsumer.listenForEvents(
               sizeof(struct EventDescriptor<MeasurementTaskEvent>),
               &item,
               portMAX_DELAY) != ESP_OK) {
      vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (item) {
      if (item->event == MEASUREMENT_TASK_DATA_UPDATE_EVENT) {
        ADXL345Data *data = static_cast<ADXL345Data *>(item->data);

        if (data) {
          float x_mps, y_mps, z_mps = 0;
          x_mps = data->x * -9.80665;
          y_mps = data->y * -9.80665;
          z_mps = data->z * -9.80665;

          AlertData newAlert;
          newAlert.alert = ALERT_NONE;
          newAlert.timestamp = millis();
          newAlert.size = sizeof(AlertData);

          if (x_mps > CONFIG_ACCEL_LEVEL_THRESHOLD_HIGH ||
              y_mps > CONFIG_ACCEL_LEVEL_THRESHOLD_HIGH ||
              z_mps > CONFIG_ACCEL_LEVEL_THRESHOLD_HIGH) {
            newAlert.alert = ALERT_ACCEL_HIGH;
          }

          if (newAlert.alert != ALERT_NONE) {
            err = accelLevelDetectProducer.produceEvent(
                (void *)&newAlert,
                newAlert.size,
                MEASUREMENT_TASK_DATA_UPDATE_EVENT);

            if (err != ESP_OK)
              ESP_LOGE(TAG, "accelLevelDetectTask failed to produce event");
          }

          if (item->data) {
            free(item->data);
          }
        }
      }

      free(item);
    }
  }
}
void initADXL345() {
  pinMode(GPIO_NUM_8, OUTPUT);

  adxl345 = ADXL345(GPIO_NUM_8, &SPIBus3);

  adxl345.powerOn();
  adxl345.setRangeSetting(16); // Accepted values are 2g, 4g, 8g or 16g
  adxl345.setFullResBit(true);

  MeasurementTaskConfig taskConfig{
      .sampleRate = CONFIG_TELE_ADXL345_SAMPLING_FREQUENCY,
      .usStackDepth = 4096U,
      .xCoreID = 0U,
      .uxPriority = 5U,
      .pcName = (char *)"ADXL345",
      .sendWaitTicks = pdMS_TO_TICKS(5000),
      .accessWaitTicks = pdMS_TO_TICKS(1000),
      .receiveWaitTicks = pdMS_TO_TICKS(10000),
  };

  adxl345MT = measurementModule.addMT(ADXL345MeasurementFunc, taskConfig);
  if (adxl345MT == nullptr) {
    ESP_LOGE(TAG, "ADXL345 configure failed");
    return;
  }

  ESP_LOGI(TAG, "ADXL345 Initialized");
  vTaskDelay(pdMS_TO_TICKS(1000));
}
// #endregion

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

  xTaskCreatePinnedToCore(&ws2812b_cycler, "led", 4096, NULL, 3, &ledTask, 0);

  initSerial();
  initI2C();
  initSPI();

  initRTC();

  initFSManager(FS_MANAGER_LITTLE_FS);

  initWiFiTimeMQTT();

  // initINA260();
  // initADXL345();

  initBME6xx_n();
  initBMEManager();
  // initBSEC();

  xTaskCreatePinnedToCore(
      &mqttDataSendTask, "mqttTask", 4096, NULL, 10, NULL, 1);
  xTaskCreatePinnedToCore(
      &mqttLoopTask, "mqttLoopTask", 4096, NULL, 10, NULL, 1);

  // xTaskCreatePinnedToCore(
  //     &accelLevelDetectTask, "acclLvlDet", 4096, NULL, 6, NULL, 0);

  // xTaskCreatePinnedToCore(
  //     &voltLevelDetectTask, "vltLvlDet", 4096, NULL, 7, NULL, 0);
  // xTaskCreatePinnedToCore(&pwrTeleTask, "pwrTele", 4096, NULL, 5, NULL, 0);

  xTaskCreatePinnedToCore(
      &tdcDurationTrackerTask,
      "tdcDurTrackTsk",
      4096,
      NULL,
      7,
      &tdcDurationTrackerTaskHandle,
      0);

  esp_err_t res;

  res = bmeMT.registerEventConsumer(
      &MQTTConsumer, MEASUREMENT_TASK_DATA_UPDATE_EVENT);
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "bmeMT.registerEventConsumer() failed");
    return;
  }
  // res = bsecMT->registerEventConsumer(
  //     &MQTTConsumer, MEASUREMENT_TASK_DATA_UPDATE_EVENT);
  // if (res != ESP_OK) {
  //   ESP_LOGE(TAG, "bsecMT.registerEventConsumer() failed");
  //   return;
  // }

  // res = ina260MT->registerEventConsumer(
  //     &voltLevelDetectConsumer, MEASUREMENT_TASK_DATA_UPDATE_EVENT);
  // if (res != ESP_OK) {
  //   ESP_LOGE(TAG, "ina260MT.registerEventConsumer() failed");
  //   return;
  // }
  // res = ina260MT->registerEventConsumer(
  //     &pwrTeleConsumer, MEASUREMENT_TASK_DATA_UPDATE_EVENT);
  // if (res != ESP_OK) {
  //   ESP_LOGE(TAG, "ina260MT.registerEventConsumer() failed");
  //   return;
  // }

  // res = adxl345MT->registerEventConsumer(
  //     &accelLevelDetectConsumer, MEASUREMENT_TASK_DATA_UPDATE_EVENT);
  // if (res != ESP_OK) {
  //   ESP_LOGE(TAG, "adxl345MT.registerEventConsumer() failed");
  //   return;
  // }

  // res = pwrTeleProducer.registerEventConsumer(
  //     &MQTTConsumer, MEASUREMENT_TASK_DATA_UPDATE_EVENT);
  // if (res != ESP_OK) {
  //   ESP_LOGE(TAG, "pwrTeleProducer.registerEventConsumer() failed");
  //   return;
  // }
  // res = voltLevelDetectProducer.registerEventConsumer(
  //     &MQTTConsumer, MEASUREMENT_TASK_DATA_UPDATE_EVENT);
  // if (res != ESP_OK) {
  //   ESP_LOGE(TAG, "voltLevelDetectProducer.registerEventConsumer()
  //   failed"); return;
  // }

  // res = accelLevelDetectProducer.registerEventConsumer(
  //     &MQTTConsumer, MEASUREMENT_TASK_DATA_UPDATE_EVENT);
  // if (res != ESP_OK) {
  //   ESP_LOGE(TAG, "accelLevelDetectProducer.registerEventConsumer()
  //   failed"); return;
  // }
  ESP_LOGI(TAG, "Registered event consumers%s", "");

  res = measurementModule.configure();
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "measurementModule.configure() failed");
    return;
  }
  ESP_LOGI(TAG, "Configured measurement module%s", "");

  vTaskDelay(pdMS_TO_TICKS(5000));

  reportStatusLED(ESP_OK);

  // adxl345MT->start();
  // bsecMT.start();
  // bmeMT.start();
  ESP_LOGI(TAG, "app_main state transition request");
  stateManager->requestTransition(STATE_IDLE);
}