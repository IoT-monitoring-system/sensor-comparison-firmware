#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include "soc/soc_caps.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <Arduino.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <Wire.h>

#include "Adafruit_INA260.h"
#include "Adafruit_NeoPixel.h"

#include "MeasurementDatatypes.h"
#include "MeasurementModule.h"
#include "MeasurementTask.h"

#include "EventSystem.hpp"

static const char *TAG = "demo";

#define I2C_BUS_SDA GPIO_NUM_1
#define I2C_BUS_SCL GPIO_NUM_2

#define WS2812B_PIN GPIO_NUM_21

#define CONFIG_TELE_INA260_SAMPLING_FREQUENCY 100
#define CONFIG_POWER_PRIMARY_BATTERY_CAPACITY 3000

#define CONFIG_MEASUREMENT_TASK_HEAP_DEBUG 1

extern "C" void app_main();

// esp_register_shutdown_handler

using INA260Data = MeasurementTaskData<VoltageData, CurrentData, PowerData>;
using ADXL345Data = MeasurementTaskData<>;
using BME688Data = MeasurementTaskData<>;

MeasurementModule measurementModule{};

MeasurementTask ADXL345MeasurementTask{};
MeasurementTask BME688MeasurementTask{};
MeasurementTask INA260MeasurementTask{};

EventConsumer<MeasurementTaskEvent>
    INA260EventConsumer{200U, RINGBUF_TYPE_NOSPLIT, pdMS_TO_TICKS(1000)};
EventConsumer<MeasurementTaskEvent>
    ADXL345EventConsumer{200U, RINGBUF_TYPE_NOSPLIT, pdMS_TO_TICKS(1000)};
EventConsumer<MeasurementTaskEvent>
    BME688EventConsumer{200U, RINGBUF_TYPE_NOSPLIT, pdMS_TO_TICKS(1000)};

TwoWire i2c_bus(0);

Adafruit_NeoPixel ws2812b =
    Adafruit_NeoPixel(1, WS2812B_PIN, NEO_RGB + NEO_KHZ800);
Adafruit_INA260 ina260 = Adafruit_INA260();

/* #region INIT SYSTEM PERIPHERALS */
void initSerial(void) {
  Serial.begin(115200);

  while (!Serial)
    vTaskDelay(pdMS_TO_TICKS(100));
  vTaskDelay(pdMS_TO_TICKS(1000));
  Serial.println("Serial Initialized");
}
void initI2C(void) {
  if (!i2c_bus.begin(I2C_BUS_SDA, I2C_BUS_SCL, 400000U)) {
    ESP_LOGE(TAG, "I2C Init failed.");
    return;
  }
  Serial.println("I2C Initialized");
  vTaskDelay(pdMS_TO_TICKS(1000));
}
/* #endregion */

/* #region INA260 RELATED */
void *INA260MeasurementFunc(void *pvParameter) {
  INA260Data *data = new INA260Data();

  data->current = ina260.readCurrent();
  data->voltage = ina260.readBusVoltage();
  data->power = ina260.readPower();

  return data;
}

void ina260ProcessingTask(void *pvParameter) {
  struct EventDescriptor<MeasurementTaskEvent> *item;
  while (1) {
    while (INA260EventConsumer.listenForEvents(
               sizeof(struct EventDescriptor<MeasurementTaskEvent>),
               &item,
               portMAX_DELAY) != ESP_OK)
      ;

    if (item->event == MEASUREMENT_TASK_DATA_UPDATE_EVENT) {
      INA260Data *data = static_cast<INA260Data *>(item->data);

      if (data->size > 0) {
        ESP_LOGI(TAG, "Current: %fmA", data->current);
      }
    }

    if (item->data) {
      free(item->data);
    }
    free(item);
  }
}

void initINA260(void) {
  bool status = ina260.begin(0x40, &i2c_bus);
  if (!status) {
    ESP_LOGE(TAG, "Unable to initialize the INA260 sensor%s", " ");
    return;
  }

  ina260.setCurrentConversionTime(INA260_TIME_1_1_ms);
  ina260.setVoltageConversionTime(INA260_TIME_1_1_ms);
  ina260.setAveragingCount(INA260_COUNT_1);

  MeasurementTaskConfig taskConfig{
      .pollingRate = CONFIG_TELE_INA260_SAMPLING_FREQUENCY,
      .usStackDepth = 2632U,
      .xCoreID = 0U,
      .uxPriority = 5U,
      .pcName = (char *)"INA260",
  };

  MeasurementTaskEventloopConfig eventloopConfig{
      .sendWaitTicks = pdMS_TO_TICKS(5000),
      .accessWaitTicks = pdMS_TO_TICKS(1000),
      .receiveWaitTicks = pdMS_TO_TICKS(10000),
  };

  esp_err_t res =
      INA260MeasurementTask.configure(&taskConfig, &eventloopConfig);
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "INA260 configure failed");
    return;
  }

  res = INA260MeasurementTask.setMeasurementTask(INA260MeasurementFunc);
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "INA260 setMeasurementTask failed");
    return;
  }

  Serial.println("INA260 Initialized");
  vTaskDelay(pdMS_TO_TICKS(1000));
}
/* #endregion */

/* #region WS2812 RELATED */
void initWS2812b(void) {
  ws2812b.begin();
  ws2812b.show();
  Serial.println("WS2812b Initialized");
  vTaskDelay(pdMS_TO_TICKS(1000));
}
void ws2812b_cycler(void *pvParameter) {
  while (true) {
    ws2812b.setPixelColor(0, ws2812b.Color(0, 0, 0));
    ws2812b.show();

    vTaskDelay(pdMS_TO_TICKS(1000));

    ws2812b.setBrightness(100);
    ws2812b.setPixelColor(0, ws2812b.Color(255, 0, 255));
    ws2812b.show();

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
/* #endregion */

void app_main() {
  initArduino();
  vTaskDelay(pdMS_TO_TICKS(1000));

  initSerial();
  initI2C();

  initWS2812b();
  initINA260();

  xTaskCreatePinnedToCore(&ws2812b_cycler, "led", 2048, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(
      &ina260ProcessingTask, "ina260prcs", 3072, NULL, 4, NULL, 0);
  // xTaskCreatePinnedToCore(
  //     &, "", 3072, NULL, 4, NULL, 0);
  // xTaskCreatePinnedToCore(
  //     &, "", 3072, NULL, 4, NULL, 0);

  esp_err_t res;

  res = INA260MeasurementTask.registerEventConsumer(
      &INA260EventConsumer, MEASUREMENT_TASK_DATA_UPDATE_EVENT);
  if (res != ESP_OK) {
    ESP_LOGE(
        TAG,
        "INA260MeasurementTask.registerEventConsumer(eventConsumerCurrent) "
        "failed");
    return;
  }

  ESP_LOGI(TAG, "Registered event consumers%s", "");

  res = measurementModule.addMeasurementTask(&INA260MeasurementTask);
  if (res != ESP_OK) {
    ESP_LOGE(
        TAG,
        "measurementModule.addMeasurementTask(INA260MeasurementTask) failed");
    return;
  }
  res = measurementModule.addMeasurementTask(&ADXL345MeasurementTask);
  if (res != ESP_OK) {
    ESP_LOGE(
        TAG,
        "measurementModule.addMeasurementTask(ADXL345MeasurementTask) failed");
    return;
  }
  res = measurementModule.addMeasurementTask(&BME688MeasurementTask);
  if (res != ESP_OK) {
    ESP_LOGE(
        TAG,
        "measurementModule.addMeasurementTask(BME688MeasurementTask) failed");
    return;
  }

  ESP_LOGI(TAG, "Added measurement task%s", "");

  res = measurementModule.configure();
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "measurementModule.configure() failed");
    return;
  }
  ESP_LOGI(TAG, "Configured measurement module%s", "");

  res = measurementModule.start();
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "measurementModule.start() failed");
    return;
  }
  ESP_LOGI(TAG, "Started measurement module%s", "");

  vTaskDelay(pdMS_TO_TICKS(10000));
  ESP_LOGI(TAG, "Stopping%s", "");
  res = measurementModule.stop();
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "measurementModule.stop() failed");
    return;
  }

  vTaskDelay(pdMS_TO_TICKS(3000));
  ESP_LOGI(TAG, "Starting%s", "");
  res = measurementModule.start();
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "measurementModule.start() failed");
    return;
  }

  vTaskDelay(pdMS_TO_TICKS(5000));
  ESP_LOGI(TAG, "Stopping%s", "");
  res = measurementModule.stop();
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "measurementModule.stop() failed");
    return;
  }
}