#include "float.h"
#include "math.h"
#include "stdio.h"

#include "sdkconfig.h"

#include "soc/gpio_num.h"

#include "bootloader_random.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_ssd1306.h"
#include "esp_log.h"
#include "esp_private/esp_clk.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "Arduino.h"
#include "Wire.h"
#include "WiFi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "bme69xLibrary.h"

#include "cbor_sensor_encoder.h"

#include "adc_module.h"
#include "mqtt_module.h"
#include "sntp_module.h"

#include "app_errors.h"

#define SENSOR_ID "clock1"

#define I2C_SCL_BUS0  GPIO_NUM_1
#define I2C_SDA_BUS0  GPIO_NUM_2
#define I2C_FREQ_BUS0 400000U

#define I2C_SCL_BUS1  GPIO_NUM_3
#define I2C_SDA_BUS1  GPIO_NUM_4
#define I2C_FREQ_BUS1 400000U

#define BME690_I2C_ADDRESS 0x76U

#define SNTP_SERVER_1         "sth1.ntp.se"
#define SNTP_SYNC_WAIT_MS     40000U
#define SNTP_TASK_PERIOD_MS   15000U
#define SNTP_REPORT_PERIOD_MS 30000U

#define WIFI_SSID          "COMHEM_6734e0"
#define WIFI_PASS          "qdmmdnwm"
#define WIFI_MAXIMUM_RETRY 5U

#define MQTT_SERVER           "192.168.0.124"
#define MQTT_PORT             8884U
#define MQTT_USERNAME         "esp32_1"
#define MQTT_PASSWORD         "Test12345"
#define MQTT_DATA_OUT_TOPIC   "/IoT-Clock-RoomMonitor/DEVICE_OUT/DATA"
#define MQTT_BUFFER_COUNT     3U
#define MQTT_MAX_MESSAGE_SIZE 2048U

#define DATA_AGGREGATION_MAX_PAYLOADS       15U
#define DATA_AGGREGATION_CBOR_OVERHEAD_COEF 1.2f /* Assuming 20% overhead */

#define TASK_QUEUE_SEND_TIMEOUT_MS 1000U

typedef struct {
  uint8_t buffer[(uint32_t)(MQTT_MAX_MESSAGE_SIZE * DATA_AGGREGATION_CBOR_OVERHEAD_COEF)];
  uint32_t length;
} mqtt_message;

extern const uint8_t client_crt_start[] asm("_binary_client_crt_start");
extern const uint8_t client_crt_end[] asm("_binary_client_crt_end");
extern const uint8_t client_key_start[] asm("_binary_client_key_start");
extern const uint8_t client_key_end[] asm("_binary_client_key_end");
extern const uint8_t root_cert_pem_start[] asm("_binary_rootCA_pem_start");
extern const uint8_t root_cert_pem_end[] asm("_binary_rootCA_pem_end");

BME69x bme690;

adc_dev_handle adc_sound_sens;

mqtt_module_handle mqtt_module;

uint64_t boot_to_utc_offset_us;

QueueHandle_t data_aggregation_queue_handle;

QueueHandle_t mqtt_free_queue;
QueueHandle_t mqtt_filled_queue;

mqtt_message mqtt_buffers[MQTT_BUFFER_COUNT];

static const char *TAG = "clock_room_monitor_app";

esp_err_t
init_i2c();

esp_err_t
init_bme690();

esp_err_t
init_wifi();
esp_err_t
init_sntp();
esp_err_t
init_mqtt();

void
task_sensor_data_aggregation(void *arg);

void
task_mqtt_sending(void *arg);
esp_err_t
mqtt_data_event_handler(mqtt_module_handle mqtt_module, int32_t event_id, esp_mqtt_event_handle_t event_handle);

extern "C" void
app_main() {
  // initArduino();
  vTaskDelay(pdMS_TO_TICKS(1000));

  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK_WITHOUT_ABORT(ret);

  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_loop_create_default());

  vTaskDelay(pdMS_TO_TICKS(1000));

  init_i2c();

  init_bme690();

  init_wifi();

  vTaskDelay(pdMS_TO_TICKS(3000));

  init_sntp();
  init_mqtt();

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

  for (;;) {
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
  esp_err_t ret = ESP_OK;

  Wire.end();

  ESP_RETURN_ON_FALSE(Wire.begin(I2C_SDA_BUS0, I2C_SCL_BUS0, I2C_FREQ_BUS0), ESP_ERR_MAIN_APP_I2C_FAIL, TAG,
                      "Failed to initialize I2C bus 0");

  ESP_RETURN_ON_FALSE(Wire1.begin(I2C_SDA_BUS1, I2C_SCL_BUS1, I2C_FREQ_BUS1), ESP_ERR_MAIN_APP_I2C_FAIL, TAG,
                      "Failed to initialize I2C bus 1");

  ESP_LOGI(TAG, "Initialized I2C");
  return ret;
}

esp_err_t
init_bme690() {
  esp_err_t ret = ESP_OK;

  bme690.begin(BME690_I2C_ADDRESS, Wire);
  ESP_RETURN_ON_FALSE(bme690.status == BME69X_OK, ESP_ERR_MAIN_APP_BME690_FAIL, TAG,
                      "Failed to initialize BME690 sensor, code: %i", bme690.status);

  ESP_LOGI(TAG, "Initialized BME690");

  return ret;
}

esp_err_t
init_wifi() {
  return ESP_OK;
}

esp_err_t
init_sntp() {
  esp_err_t ret = ESP_OK;

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

  // Could be made configurable
  setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);
  tzset();

  ESP_LOGI(TAG, "Initialized SNTP module");
  return ret;
}
esp_err_t
init_mqtt() {
  esp_err_t ret = ESP_OK;

  mqtt_module_init(&mqtt_module);
  mqtt_address_config addr_cfg = {
      .hostname = MQTT_SERVER,
      .transport = MQTT_TRANSPORT_OVER_SSL,
      .path = "/",
      .port = MQTT_PORT,
  };

  ESP_ERROR_CHECK_WITHOUT_ABORT(mqtt_module_register_event_handler(mqtt_module, MQTT_MODULE_DATA_EVENT, mqtt_data_event_handler));

  ESP_ERROR_CHECK_WITHOUT_ABORT(mqtt_module_set_address_cfg(mqtt_module, &addr_cfg));
  mqtt_credentials_config cred_cfg = {
      .username = MQTT_USERNAME,
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

  ESP_LOGI(TAG, "Initialized MQTT module");
  return ret;
}

void
task_sensor_data_aggregation(void *arg) {
  sensor_payload_t payloads[DATA_AGGREGATION_MAX_PAYLOADS];

  for (;;) {
    uint32_t prev_buffer_size = 0U;
    uint32_t payloads_received = 0U;
    mqtt_message *msg = NULL;
    if (xQueueReceive(mqtt_free_queue, &msg, portMAX_DELAY) == pdTRUE) {
      CborEncoder encoder, map_outer, map_data, sensors_array;

      cbor_encoder_init(&encoder, msg->buffer, sizeof(msg->buffer), 0);
      cbor_encoder_create_map(&encoder, &map_outer, CborIndefiniteLength);

      cbor_encode_text_stringz(&map_outer, "data");

      cbor_encoder_create_map(&map_outer, &map_data, CborIndefiniteLength);
      cbor_encode_text_stringz(&map_data, "deviceId");
      cbor_encode_text_stringz(&map_data, SENSOR_ID);

      cbor_encode_text_stringz(&map_data, "sensor_data");

      cbor_encoder_create_array(&map_data, &sensors_array, DATA_AGGREGATION_MAX_PAYLOADS);

      prev_buffer_size = cbor_encoder_get_buffer_size(&sensors_array, msg->buffer);
      while (payloads_received < DATA_AGGREGATION_MAX_PAYLOADS &&
             (xQueueReceive(data_aggregation_queue_handle, &payloads[payloads_received], portMAX_DELAY) == pdTRUE)) {
        ESP_RETURN_VOID_ON_ERROR(encode_sensor_payload(&payloads[payloads_received], &sensors_array), TAG,
                                 "Failed to encode a sensor payload");

        uint32_t buffer_size = cbor_encoder_get_buffer_size(&sensors_array, msg->buffer);
        uint32_t encoded_size = buffer_size - prev_buffer_size;
        prev_buffer_size = buffer_size;
        ESP_LOGI(TAG, "Encoded a payload, sensor:%s size:%lu", payloads[payloads_received].sensor, encoded_size);
        payloads_received++;
      }

      cbor_encoder_close_container(&map_data, &sensors_array);
      cbor_encoder_close_container(&map_outer, &map_data);
      cbor_encoder_close_container(&encoder, &map_outer);

      msg->length = cbor_encoder_get_buffer_size(&encoder, msg->buffer);

      ESP_LOGI(TAG, "Encoded an mqtt payload, length:%lu, sensor payloads:%lu", msg->length, payloads_received);

      if (xQueueSend(mqtt_filled_queue, &msg, pdMS_TO_TICKS(1000U)) != pdTRUE) {
        ESP_LOGW(TAG, "Filled queue full, dropping msg");
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

  ESP_LOGI(TAG, "Received on %.*s; Content: %.*s", event_handle->topic_len, event_handle->topic, event_handle->data_len,
           event_handle->data);
  return ret;
}
