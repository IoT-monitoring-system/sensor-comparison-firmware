#include "float.h"
#include "math.h"
#include "stdio.h"

#include "sdkconfig.h"

#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "Arduino.h"
#include "WiFi.h"
#include "Wire.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "BME6xxManager.h"
#include "BME6xxManagerErrors.h"
#include "bme68xLibrary.h"
#include "bme69xLibrary.h"

#include "SparkFun_ADXL345.h"

#include "FSManager.h"
#include "MQTTClient.h"

#include "cbor_sensor_encoder.h"

#include "adc_module.h"
#include "gnss_module.h"
#include "mqtt_module.h"
#include "sntp_module.h"

#include "app_errors.h"

#include "alert_module.h"
#include "mqtt_config.h"
#include "peripheral_config.h"
#include "sntp_config.h"
#include "wifi_config.h"

#define DEVICE_ID "esp32_node_1"

#define DATA_AGGREGATION_MAX_PAYLOADS       25U
#define DATA_AGGREGATION_CBOR_OVERHEAD_COEF 1.2f /* Assuming 20% overhead */

#define TASK_GNSS_SAMPLING_REPORT_PERIOD_MS 5000U
#define TASK_GNSS_SAMPLING_QUEUE_SIZE       10U

#define TASK_FALL_DETECTION_SAMPLE_PERIOD_MS 40U
#define TASK_FALL_DETECTION_CALC_PERIOD_MS   1000U

#define ALERT_AGGREGATION_PERIOD_MS 60000U
#define ALERT_MAX_ALERTS            SENSOR_MAX_FIELDS
#define ALERT_HANDLER_TIMEOUT_MS    1000U
#define ALERT_FALL                  0U
#define ALERT_FALL_EXPIRY_MS        14000U
#define ALERT_GAS                   1U
#define ALERT_GAS_EXPIRY_MS         11000U

#define TASK_QUEUE_SEND_TIMEOUT_MS 1000U

static const char *TAG = "monitoring_node";

typedef struct {
  uint8_t buffer[MQTT_MAX_MESSAGE_SIZE];
  uint32_t length;
} mqtt_message;

extern const uint8_t client_crt_start[] asm("_binary_client_crt_start");
extern const uint8_t client_crt_end[] asm("_binary_client_crt_end");
extern const uint8_t client_key_start[] asm("_binary_client_key_start");
extern const uint8_t client_key_end[] asm("_binary_client_key_end");
extern const uint8_t root_cert_pem_start[] asm("_binary_rootCA_pem_start");
extern const uint8_t root_cert_pem_end[] asm("_binary_rootCA_pem_end");

BME69x bme690;
BME6xxManager bme6xx_manager;

ADXL345 adxl345;

mqtt_module_handle mqtt_module;
alert_module_handle alert_module;
gnss_module_handle gnss_module;

uint64_t boot_to_utc_offset_us;

TaskHandle_t task_bme6xx_sampling_handle;
TaskHandle_t task_gnss_sampling_handle;
TaskHandle_t task_fall_detection_handle;

TaskHandle_t task_alert_handle;

TaskHandle_t task_sensor_data_aggregation_handle;
TaskHandle_t task_mqtt_sending_handle;

QueueHandle_t gnss_event_data_queue_handle;
QueueHandle_t alert_data_queue_handle;

QueueHandle_t data_aggregation_queue_handle;

QueueHandle_t mqtt_free_queue;
QueueHandle_t mqtt_filled_queue;

mqtt_message mqtt_buffers[MQTT_BUFFER_COUNT];

esp_err_t
init_i2c();

esp_err_t
init_littlefs();

esp_err_t
init_bme6xx();
esp_err_t
init_bme_manager();
esp_err_t
init_adxl345();
esp_err_t
init_gnss_module();
void gnss_data_event_handler(struct gnss_module_sentence);

esp_err_t
init_alert_module();

esp_err_t
init_wifi();
esp_err_t
init_sntp();
esp_err_t
init_mqtt();

void
task_bme6xx_sampling(void *arg);
void
task_gnss_sampling(void *arg);
void
task_fall_detection(void *arg);
void
task_alert(void *arg);
bool
alert_handler(const alert_descriptor *alert);

void
task_sensor_data_aggregation(void *arg);

void
task_mqtt_sending(void *arg);
esp_err_t
mqtt_data_event_handler(mqtt_module_handle mqtt_module, int32_t event_id, esp_mqtt_event_handle_t event_handle);

void
task_error_handling(void *arg);

void
print_heap_caps_info_debug();

extern "C" void
app_main() {
  initArduino();
  vTaskDelay(pdMS_TO_TICKS(1000));

  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK_WITHOUT_ABORT(ret);

  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_loop_create_default());

  vTaskDelay(pdMS_TO_TICKS(1000));

  init_wifi();
  init_sntp();

  init_alert_module();

  init_littlefs();

  init_i2c();

  init_bme6xx();
  init_bme_manager();

  init_adxl345();

  alert_data_queue_handle = xQueueCreate(ALERT_MAX_ALERTS, sizeof(alert_descriptor));
  if (NULL == alert_data_queue_handle) {
    ESP_LOGE(TAG, "Failed to initialize alert_data_queue_handle");
    return;
  }

  gnss_event_data_queue_handle = xQueueCreate(TASK_GNSS_SAMPLING_QUEUE_SIZE, sizeof(gnss_module_sentence));
  if (NULL == gnss_event_data_queue_handle) {
    ESP_LOGE(TAG, "Failed to initialize gnss_event_data_queue");
    return;
  }

  data_aggregation_queue_handle = xQueueCreate(DATA_AGGREGATION_MAX_PAYLOADS, sizeof(sensor_payload_t));
  if (NULL == data_aggregation_queue_handle) {
    ESP_LOGE(TAG, "Failed to initialize data_aggregation_queue");
    return;
  }

  mqtt_free_queue = xQueueCreate(MQTT_BUFFER_COUNT, sizeof(mqtt_message *));
  if (NULL == mqtt_free_queue) {
    ESP_LOGE(TAG, "Failed to initialize mqtt_free_queue");
    return;
  }
  mqtt_filled_queue = xQueueCreate(MQTT_BUFFER_COUNT, sizeof(mqtt_message *));
  if (NULL == mqtt_filled_queue) {
    ESP_LOGE(TAG, "Failed to initialize mqtt_filled_queue");
    return;
  }
  for (int i = 0; i < MQTT_BUFFER_COUNT; ++i) {
    mqtt_message *msg = &mqtt_buffers[i];
    xQueueSend(mqtt_free_queue, &msg, 0);
  }

  xTaskCreatePinnedToCore(task_alert, "alertAggrTsk", 3144U, NULL, 14, &task_alert_handle, 0U);

  xTaskCreatePinnedToCore(task_mqtt_sending, "mqttTsk", 3072U, NULL, 9, &task_mqtt_sending_handle, 1U);
  xTaskCreatePinnedToCore(task_sensor_data_aggregation, "dataAggrTsk", 3072U, NULL, 8, &task_sensor_data_aggregation_handle, 0U);

  xTaskCreatePinnedToCore(task_bme6xx_sampling, "bme6xxTsk", 3072U, NULL, 6, &task_bme6xx_sampling_handle, 0U);
  xTaskCreatePinnedToCore(task_gnss_sampling, "gnssTsk", 3072U, NULL, 6, &task_gnss_sampling_handle, 0U);
  xTaskCreatePinnedToCore(task_fall_detection, "fallDetectTsk", 3072U, NULL, 6, &task_fall_detection_handle, 0U);

  // init_gnss_module();

  init_mqtt();

  for (;;) {
    print_heap_caps_info_debug();

    ESP_LOGI(TAG, "");

    char *task_buffer = (char *)pvPortMalloc(2048);
    if (task_buffer == NULL) {
      ESP_LOGE(TAG, "Failed to allocate memory for task list");
      return;
    }

    vTaskList(task_buffer);
    ESP_LOGI(TAG, "\nTask Name\tState\tPrio\tStack\tNum\n%s", task_buffer);

    vPortFree(task_buffer);

    vTaskDelay(pdMS_TO_TICKS(4000U));
  }
}

esp_err_t
init_i2c() {
  Wire.end();

  ESP_RETURN_ON_FALSE(Wire.begin(I2C_SDA_BUS0, I2C_SCL_BUS0, I2C_FREQ_BUS0), ESP_ERR_MAIN_APP_I2C_FAIL, TAG,
                      "Failed to initialize I2C bus 0");

  ESP_RETURN_ON_FALSE(Wire1.begin(I2C_SDA_BUS1, I2C_SCL_BUS1, I2C_FREQ_BUS1), ESP_ERR_MAIN_APP_I2C_FAIL, TAG,
                      "Failed to initialize I2C bus 1");

  ESP_LOGD(TAG, "Initialized I2C");
  return ESP_OK;
}
esp_err_t
init_littlefs() {
  ESP_RETURN_ON_FALSE(LittleFS.begin(true, "/littlefs", 10, "littlefs"), ESP_ERR_MAIN_APP_LITTLEFS_FAIL, TAG,
                      "Failed to start LittleFS");

  ESP_LOGD(TAG, "Initialized LittleFS");

  return ESP_OK;
}

esp_err_t
init_bme6xx() {
  bme690.begin(BME690_I2C_ADDRESS, Wire);
  ESP_RETURN_ON_FALSE(static_cast<BME6xxStatus>(bme690.status) == BME6xxStatus::OK, ESP_ERR_MAIN_APP_BME690_FAIL, TAG,
                      "Failed to initialize BME690 sensor, code: %i", bme690.status);

  ESP_LOGD(TAG, "Initialized BME690");

  return ESP_OK;
}
esp_err_t
init_bme_manager() {
  FSManager fsManager;
  ESP_RETURN_ON_ERROR(fsManager.initializeFileSystem(LittleFS, FSManager::FS_MANAGER_LITTLE_FS), TAG,
                      "Failed to initialize FSManager");
  std::unique_ptr<FSFile> bmeCfg = std::unique_ptr<FSFile>(fsManager.readFile("/x8default_1sens.bmeconfig"));
  ESP_RETURN_ON_FALSE(bmeCfg, ESP_ERR_MAIN_APP_BME_MANAGER_FAIL, TAG, "Invalid BME config");

  ESP_RETURN_ON_ERROR(bme6xx_manager.addSensor(bme690, true), TAG, "Failed to add a bme690 sensor");

  ESP_RETURN_ON_ERROR(bme6xx_manager.configure(*bmeCfg), TAG, "Failed to configure BMEManager");
  ESP_RETURN_ON_ERROR(bme6xx_manager.begin(), TAG, "Failed to start BMEManager");

  ESP_LOGD(TAG, "bmeManager Initialized");

  return ESP_OK;
}
esp_err_t
init_adxl345() {
  ESP_RETURN_ON_FALSE(adxl345.begin(ADXL345_I2C_ADDR_LOW, Wire), ESP_ERR_MAIN_APP_ADXL345_FAIL, TAG,
                      "Failed to initialize ADXL345 sensor");

  adxl345.powerOn();
  adxl345.setRangeSetting(16);
  adxl345.setFullResBit(true);
  adxl345.set_bw(0b00001000);
  adxl345.setLowPower(true);

  return ESP_OK;
}
esp_err_t
init_gnss_module() {
  uart_config_t uart_config = {
      .baud_rate = 115000,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  ESP_ERROR_CHECK_WITHOUT_ABORT(gnss_module_init(&uart_config, UART_NUM_1, GPIO_NUM_12, GPIO_NUM_13, &gnss_module));
  ESP_ERROR_CHECK_WITHOUT_ABORT(gnss_module_register_event_handler(gnss_module, gnss_data_event_handler));

  enum minmea_sentence_id sentences[] = {
      MINMEA_SENTENCE_RMC,
  };
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      gnss_module_update_parse_sentences(gnss_module, sentences, (sizeof(sentences) / sizeof(enum minmea_sentence_id))));
  ESP_ERROR_CHECK_WITHOUT_ABORT(gnss_module_start(gnss_module));
  ESP_LOGD(TAG, "Initialized GNSS module");

  return ESP_OK;
}

esp_err_t
init_alert_module() {
  struct alert_module_config module_cfg = {
      .task_stack_size = 4096U,
      .task_priority = 20U,
      .task_core_affinity = 1U,
      .task_period_ms = 3500U,
  };
  ESP_RETURN_ON_ERROR(alert_module_init(&module_cfg, &alert_module), TAG, "Failed to initialize Alert Module");
  ESP_LOGD(TAG, "Initialized Alert Module");
  return ESP_OK;
}

esp_err_t
init_wifi() {
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();

    WiFi.mode(WIFI_STA);
    WiFi.enableSTA(true);

    WiFi.begin(WIFI_SSID, WIFI_PASS);

    vTaskDelay(pdMS_TO_TICKS(5000));
  }

  ESP_LOGD(TAG, "Connected to a WiFi Access Point");

  return ESP_OK;
}

esp_err_t
init_sntp() {
  sntp_module_config sntp_cfg = {
      .server = SNTP_SERVER_1,
  };
  ESP_RETURN_ON_ERROR(sntp_module_init(&sntp_cfg), TAG, "Failed to initialize SNTP module");
  ESP_RETURN_ON_ERROR(sntp_module_start(), TAG, "Failed to start SNTP module");
  ESP_RETURN_ON_ERROR(sntp_module_sync_wait(SNTP_SYNC_WAIT_MS), TAG, "Failed to sync SNTP module");

  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  uint64_t sntp_synced_utc_us = (uint64_t)tv_now.tv_sec * 1000000L + (uint64_t)tv_now.tv_usec;
  uint64_t boot_time_us = esp_timer_get_time();

  boot_to_utc_offset_us = sntp_synced_utc_us - boot_time_us;

  ESP_LOGD(TAG, "Initialized SNTP module");
  return ESP_OK;
}
esp_err_t
init_mqtt() {
  ESP_RETURN_ON_ERROR(mqtt_module_init(&mqtt_module), TAG, );
  mqtt_address_config addr_cfg = {
      .hostname = MQTT_SERVER,
      .transport = MQTT_TRANSPORT_OVER_SSL,
      .path = "/",
      .port = MQTT_PORT,
  };

  ESP_ERROR_CHECK_WITHOUT_ABORT(mqtt_module_register_event_handler(mqtt_module, MQTT_MODULE_DATA_EVENT, mqtt_data_event_handler));

  ESP_ERROR_CHECK_WITHOUT_ABORT(mqtt_module_set_address_cfg(mqtt_module, &addr_cfg));
  mqtt_credentials_config cred_cfg = {
      .username = MQTT_USERNAME_COLLAR,
  };
  ESP_ERROR_CHECK_WITHOUT_ABORT(mqtt_module_set_credentials_cfg(mqtt_module, &cred_cfg));
  mqtt_verification_config verif_cfg = {
      .certificate = (const char *)root_cert_pem_start,
  };
  ESP_ERROR_CHECK_WITHOUT_ABORT(mqtt_module_set_verification_cfg(mqtt_module, &verif_cfg));
  mqtt_auth_config auth_cfg = {
      .password = MQTT_PASSWORD,
      .certificate = (const char *)client_crt_start,
      .key = (const char *)client_key_start,
  };
  ESP_ERROR_CHECK_WITHOUT_ABORT(mqtt_module_set_authentication_cfg(mqtt_module, &auth_cfg));
  mqtt_session_config sesh_cfg = {
      .disable_clean_session = true,

  };
  ESP_ERROR_CHECK_WITHOUT_ABORT(mqtt_module_set_session_cfg(mqtt_module, &sesh_cfg));
  ESP_ERROR_CHECK_WITHOUT_ABORT(mqtt_module_connect(mqtt_module, 15000));
  ESP_ERROR_CHECK_WITHOUT_ABORT(mqtt_module_subscribe(mqtt_module, "/IoT-Clock-RoomMonitor/DEVICE_IN/CTRL", 0));

  ESP_LOGD(TAG, "Initialized MQTT module");
  return ESP_OK;
}

void
task_bme6xx_sampling(void *arg) {

  // if (tdcScheduled) {
  //   uint64_t timeNowUS = esp_timer_get_time() + rtcToESPOffset;
  //   uint64_t tdcSchedStartTimeUS = tdcSchedStartTimeSec * 1000000ULL;

  //   if (tdcSchedStartTimeUS < timeNowUS) {
  //     ESP_LOGE(
  //         TAG,
  //         "Scheduled time is invalid; Scheduled: %llu; Now: %llu",
  //         tdcSchedStartTimeUS,
  //         timeNowUS);
  //     reportStatusLED(ESP_ERR_TIME_SCHEDULE_INVALID);

  //     return ESP_ERR_TIME_SCHEDULE_INVALID;
  //   }

  //   uint64_t sleepTimeUS = (tdcSchedStartTimeUS - timeNowUS);

  //   ESP_LOGI(
  //       TAG,
  //       "Scheduled time: %llu; Time now: %llu; Sleep time: %llu",
  //       tdcSchedStartTimeUS,
  //       timeNowUS,
  //       sleepTimeUS);

  //   vTaskDelay(pdMS_TO_TICKS(sleepTimeUS / 1000));

  //   tdcScheduled = false;
  // }

  sensor_payload_t payload;

  strncpy(payload.fields[0].name, "sens_id", SENSOR_FIELD_NAME_LEN - 1);
  payload.fields[0].name[SENSOR_FIELD_NAME_LEN - 1] = '\0';
  payload.fields[0].type = SENSOR_FIELD_DATATYPE_UINT;

  strncpy(payload.fields[1].name, "gas_idx", SENSOR_FIELD_NAME_LEN - 1);
  payload.fields[1].name[SENSOR_FIELD_NAME_LEN - 1] = '\0';
  payload.fields[1].type = SENSOR_FIELD_DATATYPE_UINT;

  strncpy(payload.fields[2].name, "gas_res", SENSOR_FIELD_NAME_LEN - 1);
  payload.fields[2].name[SENSOR_FIELD_NAME_LEN - 1] = '\0';
  payload.fields[2].type = SENSOR_FIELD_DATATYPE_FLOAT;

  payload.field_count = 3U;

  for (;;) {
    while (bme6xx_manager.scheduleSensor()) {
      BMESensorData sensor_data;
      esp_err_t ret = bme6xx_manager.collectData(sensor_data);
      if (ret != ESP_OK) {
        // ESP_LOGE(TAG, "Error collecting bme6xxManager data, code:%u", ret);
        break;
      }

      strncpy(payload.sensor, sensor_data.type, SENSOR_NAME_MAX_LEN - 1); // Dynamic
      payload.sensor[SENSOR_NAME_MAX_LEN - 1] = '\0';

      for (uint8_t i = 0; i < sensor_data.dataLen; ++i) {
        payload.timestamp = (uint64_t)(sensor_data.data[i].meas_timestamp + boot_to_utc_offset_us);

        payload.fields[0].value.u = sensor_data.sensorId;               /* sens_id */
        payload.fields[1].value.u = sensor_data.data[i].gas_index;      /* gas index */
        payload.fields[2].value.f = sensor_data.data[i].gas_resistance; /* gas res */

        if (xQueueSend(data_aggregation_queue_handle, &payload, pdMS_TO_TICKS(TASK_QUEUE_SEND_TIMEOUT_MS)) == pdTRUE) {
          // ESP_LOGD(TAG, "Sent data from BME sampling task");
        } else {
          ESP_LOGE(TAG, "Failed sending from BME sampling task");
        }
      }
      // vTaskDelay(pdMS_TO_TICKS(35));
    }
    vTaskDelay(pdMS_TO_TICKS(65));
  }
}
void
task_fall_detection(void *arg) {
  alert_config alert_cfg = {
      .alert_id = ALERT_FALL,
      .alert_notify_mask = ALERT_NOTIFY_SET | ALERT_NOTIFY_EXPIRED,
      .auto_expire = true,
      .expiry_time_ms = ALERT_FALL_EXPIRY_MS,
      .state_change_handler = alert_handler,
  };
  strncpy(alert_cfg.name, "fall", ALERT_MODULE_NAME_MAX_LEN - 1);
  alert_cfg.name[ALERT_MODULE_NAME_MAX_LEN - 1] = '\0';
  alert_module_register_alert(alert_module, &alert_cfg);

  uint64_t prev_trigger_time_us = esp_timer_get_time();

  for (;;) {
    int x, y, z;
    adxl345.readAccel(&x, &y, &z);
    // ESP_LOGD(TAG, "Accel, x:%d y:%d z:%d", x, y, z);
    if (((uint64_t)esp_timer_get_time() - prev_trigger_time_us) >= 12000000U) {
      alert_module_set_alert(alert_module, ALERT_FALL);
      prev_trigger_time_us = (uint64_t)esp_timer_get_time();
    }

    vTaskDelay(pdMS_TO_TICKS(TASK_FALL_DETECTION_SAMPLE_PERIOD_MS));
  }
}
void
task_gnss_sampling(void *arg) {
  sensor_payload_t payload;

  strncpy(payload.sensor, "GNSS", SENSOR_NAME_MAX_LEN - 1);
  payload.sensor[SENSOR_NAME_MAX_LEN - 1] = '\0';

  payload.field_count = 2U;

  strncpy(payload.fields[0].name, "lat", SENSOR_FIELD_NAME_LEN - 1);
  payload.fields[0].name[SENSOR_FIELD_NAME_LEN - 1] = '\0';
  payload.fields[0].type = SENSOR_FIELD_DATATYPE_FLOAT;

  strncpy(payload.fields[1].name, "lon", SENSOR_FIELD_NAME_LEN - 1);
  payload.fields[1].name[SENSOR_FIELD_NAME_LEN - 1] = '\0';
  payload.fields[1].type = SENSOR_FIELD_DATATYPE_FLOAT;

  // Getting a reliable fix from the GNSS module is problematic
  // Therefore a set of predetermined coordinates is used
  float latitudes[] = {
      59.422597f, 59.422790f, 59.422980f, 59.423212f, 59.423221f, 59.423208f, 59.422705f,
      59.422361f, 59.421739f, 59.421851f, 59.422329f, 59.422806f, 59.422705f, 59.422567f,
  };
  float longitudes[] = {
      17.830827f, 17.830010f, 17.829397f, 17.829352f, 17.828007f, 17.826988f, 17.826722f,
      17.826582f, 17.827448f, 17.828842f, 17.829555f, 17.829541f, 17.830358f, 17.830867f,
  };
  uint8_t num_points = sizeof(latitudes) / sizeof(float);

  struct gnss_module_sentence sentence;

  // Track the coordinate pairs
  uint16_t counter_points = 0U;

  // Track the interpolation process
  uint16_t counter_interpolation = 1U;
  uint16_t interpolation_points = 6U;
  for (;;) {
    payload.timestamp = (uint64_t)esp_timer_get_time() + boot_to_utc_offset_us;

    float lambda = (float)counter_interpolation / (float)interpolation_points;

    uint16_t next_point = (counter_points + 1 < num_points) ? counter_points + 1 : 0U;
    float lat = (1 - lambda) * latitudes[counter_points] + lambda * latitudes[next_point];
    float lon = (1 - lambda) * longitudes[counter_points] + lambda * longitudes[next_point];

    payload.fields[0].value.f = lat;
    payload.fields[1].value.f = lon;

    if (counter_interpolation % interpolation_points == 0) {
      counter_interpolation = 1U;
      counter_points = next_point;
    } else {
      counter_interpolation++;
    }

    if (xQueueSend(data_aggregation_queue_handle, &payload, pdMS_TO_TICKS(TASK_QUEUE_SEND_TIMEOUT_MS)) == pdTRUE) {
      // ESP_LOGD(TAG, "Sent data from GNSS sampling task");
    } else {
      ESP_LOGE(TAG, "Failed sending from GNSS sampling task");
    }

    vTaskDelay(pdMS_TO_TICKS(TASK_GNSS_SAMPLING_REPORT_PERIOD_MS));
  }
}
void
gnss_data_event_handler(gnss_module_sentence sentence) {
  if (xQueueSend(gnss_event_data_queue_handle, &sentence, pdMS_TO_TICKS(1000)) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to send gnss event data to a queue");
  }
}

void
task_alert(void *arg) {
  alert_descriptor alert;

  sensor_payload_t payload;

  strncpy(payload.sensor, "alerts", SENSOR_NAME_MAX_LEN - 1);
  payload.sensor[SENSOR_NAME_MAX_LEN - 1] = '\0';

  for (;;) {
    // Shrink the waiting window from ALERT_AGGREGATION_PERIOD_MS down to 0
    uint64_t start_time_us = (uint64_t)esp_timer_get_time();
    uint64_t elapsed_ms = 0U;
    uint64_t time_to_wait_ms = ALERT_AGGREGATION_PERIOD_MS;
    uint8_t counter = 0U;

    payload.timestamp = (uint64_t)(start_time_us + boot_to_utc_offset_us);
    while (elapsed_ms < ALERT_AGGREGATION_PERIOD_MS) {
      if (xQueueReceive(alert_data_queue_handle, &alert, pdMS_TO_TICKS(time_to_wait_ms)) != pdTRUE) {
        break;
      }

      strncpy(payload.fields[counter].name, alert.name, SENSOR_FIELD_NAME_LEN - 1);
      payload.fields[counter].name[SENSOR_FIELD_NAME_LEN - 1] = '\0';
      payload.fields[counter].type = SENSOR_FIELD_DATATYPE_UINT;
      payload.fields[counter].value.u = alert.state == ALERT_STATE_SET ? 1U : 0U;
      counter++;

      if (counter >= SENSOR_MAX_FIELDS) {
        break;
      }

      elapsed_ms = ((uint64_t)esp_timer_get_time() - start_time_us) / 1000ULL;
      if (elapsed_ms >= ALERT_AGGREGATION_PERIOD_MS) {
        break;
      }
      time_to_wait_ms = ALERT_AGGREGATION_PERIOD_MS - elapsed_ms;
    }

    if (0 == counter)
      continue;

    payload.field_count = counter;

    if (xQueueSend(data_aggregation_queue_handle, &payload, portMAX_DELAY) == pdTRUE) {
      ESP_LOGD(TAG, "Sent data from Alert task");
    } else {
      ESP_LOGE(TAG, "Failed sending from Alert task");
    }

    counter = 0U;
  }
}
bool
alert_handler(const alert_descriptor *alert) {
  if (NULL == alert)
    return false;

  return xQueueSend(alert_data_queue_handle, alert, pdTICKS_TO_MS(ALERT_HANDLER_TIMEOUT_MS)) == pdTRUE;
}

void
task_sensor_data_aggregation(void *arg) {
  for (;;) {
    mqtt_message *msg = NULL;
    if (xQueueReceive(mqtt_free_queue, &msg, portMAX_DELAY) == pdTRUE) {
      uint32_t payloads_received = 0U;

      CborEncoder encoder, map_outer, map_data, sensors_array;

      cbor_encoder_init(&encoder, msg->buffer, sizeof(msg->buffer), 0);
      cbor_encoder_create_map(&encoder, &map_outer, CborIndefiniteLength);

      cbor_encode_text_stringz(&map_outer, "data");

      cbor_encoder_create_map(&map_outer, &map_data, CborIndefiniteLength);
      cbor_encode_text_stringz(&map_data, "deviceId");
      cbor_encode_text_stringz(&map_data, DEVICE_ID);

      cbor_encode_text_stringz(&map_data, "sensor_data");

      cbor_encoder_create_array(&map_data, &sensors_array, DATA_AGGREGATION_MAX_PAYLOADS);

      uint32_t prev_buffer_size = cbor_encoder_get_buffer_size(&sensors_array, msg->buffer);
      while (payloads_received < DATA_AGGREGATION_MAX_PAYLOADS && prev_buffer_size <= MQTT_MAX_MESSAGE_SIZE) {
        sensor_payload_t payload;
        if (xQueueReceive(data_aggregation_queue_handle, &payload, portMAX_DELAY) != pdTRUE) {
          break;
        }

        ESP_RETURN_VOID_ON_ERROR(encode_sensor_payload(&payload, &sensors_array), TAG, "Failed to encode a sensor payload");

        /* Debug start */
        // uint32_t buffer_size = cbor_encoder_get_buffer_size(&sensors_array, msg->buffer);
        // uint32_t encoded_size = buffer_size - prev_buffer_size;
        // prev_buffer_size = buffer_size;
        // ESP_LOGD(TAG, "Encoded a payload, sensor:%s size:%lu", payload.sensor, encoded_size);
        /* Debug end */

        payloads_received++;
      }

      cbor_encoder_close_container(&map_data, &sensors_array);
      cbor_encoder_close_container(&map_outer, &map_data);
      cbor_encoder_close_container(&encoder, &map_outer);

      msg->length = cbor_encoder_get_buffer_size(&encoder, msg->buffer);

      // ESP_LOGD(TAG, "Encoded an mqtt message, length:%lu, sensor payloads:%lu", msg->length, payloads_received);

      if (xQueueSend(mqtt_filled_queue, &msg, pdMS_TO_TICKS(1000U)) != pdTRUE) {
        ESP_LOGE(TAG, "Filled queue full, dropping msg");
        memset(msg->buffer, 0, sizeof(msg->buffer));
        xQueueSend(mqtt_free_queue, &msg, 0);
      }
    }
  }
}

void
task_mqtt_sending(void *arg) {
  esp_err_t ret = ESP_OK;
  mqtt_message *msg = NULL;
  for (;;) {
    if (xQueueReceive(mqtt_filled_queue, &msg, portMAX_DELAY) == pdTRUE) {
      ret = mqtt_module_publish(mqtt_module, MQTT_DATA_OUT_TOPIC, (char *)msg->buffer, msg->length, 0, 0);
      if (ESP_ERR_INVALID_STATE == ret) {
        mqtt_module_reconnect(mqtt_module, (uint32_t)portMAX_DELAY);
      } else {
        ESP_LOGD(TAG, "Sent an MQTT message via MQTT, size:%lu", msg->length);
      }

      memset(msg->buffer, 0, sizeof(msg->buffer));

      xQueueSend(mqtt_free_queue, &msg, 0);
    }
  }
}
esp_err_t
mqtt_data_event_handler(mqtt_module_handle mqtt_module, int32_t event_id, esp_mqtt_event_handle_t event_handle) {
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
  esp_err_t ret = ESP_OK;
  int msg_id = event_handle->msg_id;

  if (msg_id < 0) {
    ESP_LOGE(TAG, "Error receiving a message on %s; Error: %i", event_handle->topic_len ? event_handle->topic : "None", msg_id);
    return ret;
  }

  ESP_LOGD(TAG, "Received on %.*s; Content: %.*s", event_handle->topic_len, event_handle->topic, event_handle->data_len,
           event_handle->data);
  return ret;
}

void
task_error_handling(void *arg) {
}

void
print_heap_caps_info_debug() {
  multi_heap_info_t info;
  heap_caps_get_info(&info, MALLOC_CAP_DEFAULT);

  ESP_LOGI("", "");
  ESP_LOGI("HEAP", "Total free bytes:       %u", (unsigned)info.total_free_bytes);
  ESP_LOGI("HEAP", "Total allocated bytes:  %u", (unsigned)info.total_allocated_bytes);
  ESP_LOGI("HEAP", "Largest free block:     %u", (unsigned)info.largest_free_block);
  ESP_LOGI("HEAP", "Minimum free bytes:     %u", (unsigned)info.minimum_free_bytes);
  ESP_LOGI("HEAP", "Free blocks:            %u", (unsigned)info.free_blocks);
  ESP_LOGI("HEAP", "Allocated blocks:       %u", (unsigned)info.allocated_blocks);
  ESP_LOGI("HEAP", "Total blocks:           %u", (unsigned)info.total_blocks);
  ESP_LOGI("", "");
}
