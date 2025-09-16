#include "float.h"
#include "math.h"
#include "stdio.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_pm.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "Arduino.h"
#include "Wire.h"

#include "cbor_sensor_encoder.h"

#include "ADS1X15.h"

#include "config.h"
#include "tgs_sample_module.h"

/* Managed by main app*/
static sntp_client_handle sntp_client;
/* Managed by main app*/
static QueueHandle_t out_queue;
/* Managed by main app*/
static TwoWire *wire;

static ADS1115 tgs2602(ADS1115_ADDRESS, wire);
static TaskHandle_t task_tgs2602_sampling_handle;

static const char *TAG = "tgs_sample_module";

static esp_err_t
init_tgs2602();

static void
task_tgs2602_sampling(void *arg);

static void
tgs_sample_module_cleanup();

esp_err_t
tgs_sample_module_init(tgs_sample_module_config *module_cfg) {
  esp_err_t ret = ESP_OK;

  ESP_RETURN_ON_FALSE(module_cfg, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  ESP_RETURN_ON_FALSE(module_cfg->out_queue, ESP_ERR_INVALID_ARG, TAG, "out_queue is invalid");
  ESP_RETURN_ON_FALSE(module_cfg->wire, ESP_ERR_INVALID_ARG, TAG, "wire is invalid");
  ESP_RETURN_ON_FALSE(module_cfg->sntp_client, ESP_ERR_INVALID_ARG, TAG, "sntp_client is invalid");

  out_queue = module_cfg->out_queue;
  sntp_client = module_cfg->sntp_client;
  wire = module_cfg->wire;

  ESP_GOTO_ON_ERROR(init_tgs2602(), err, TAG, "failed to initialize TGS2602 sensor");

  ESP_GOTO_ON_FALSE(
      xTaskCreatePinnedToCore(task_tgs2602_sampling, "tgs2602Tsk", 3072U, NULL, 6, &task_tgs2602_sampling_handle, 0U) == pdPASS,
      ESP_FAIL, err, TAG, "failed to create tgs2602Tsk");

  return ESP_OK;
err:
  tgs_sample_module_cleanup();
  return ret;
}
esp_err_t
tgs_sample_module_del() {
  tgs_sample_module_stop();
  tgs_sample_module_cleanup();

  ESP_LOGI(TAG, "Deleted TGS Sample module");

  return ESP_OK;
}

esp_err_t
tgs_sample_module_start() {
  vTaskResume(task_tgs2602_sampling_handle);

  ESP_LOGI(TAG, "Started TGS Sample module");

  return ESP_OK;
}
esp_err_t
tgs_sample_module_stop() {
  xTaskNotifyGive(task_tgs2602_sampling_handle);

  ESP_LOGI(TAG, "Stopped TGS Sample module");

  return ESP_OK;
}

static esp_err_t
init_tgs2602() {
  ESP_RETURN_ON_FALSE(tgs2602.begin(), ESP_FAIL, TAG, "Failed to initialize TGS2602 sensor");

  ESP_RETURN_ON_FALSE(tgs2602.isConnected(), ESP_FAIL, TAG, "TGS2602 sensor is not connected");

  tgs2602.setDataRate(0U);
  tgs2602.setGain(1U);

  ESP_LOGD(TAG, "Initialized TGS2602 sensor");

  return ESP_OK;
}

static void
task_tgs2602_sampling(void *arg) {
  sensor_payload_t payload;

  strncpy(payload.sensor, "TGS2602", SENSOR_NAME_MAX_LEN - 1);
  payload.sensor[SENSOR_NAME_MAX_LEN - 1] = '\0';

  float voltage_conversion_factor = tgs2602.toVoltage();

  digitalWrite(TGS2602_HEATER_CONTROL_PIN, 1);
  ESP_LOGD(TAG, "Warming up the TGS2602 sensor for 2 minutes");
  vTaskDelay(pdMS_TO_TICKS(TGS2602_HEATER_WARMUP_DURATION_MS));
  ESP_LOGD(TAG, "Ended warming up the TGS2602 sensor");

  for (;;) {
    if (ulTaskNotifyTake(pdTRUE, 0)) {
      vTaskSuspend(NULL);
    }

    if (tgs2602.isReady()) {
      payload.timestamp = (uint64_t)(esp_timer_get_time() + sntp_client_get_boot_posix_time_us(sntp_client));

      int16_t voltage_raw = tgs2602.readADC(0);

      if (voltage_raw > 0) {
        float voltage = voltage_raw * voltage_conversion_factor;

        // R_S = ((V_circuit / V_RL) - 1) * R_L
        // V_circuit ~ 5v
        // R_L ~ 15kOhm
        // V_RL = voltage
        float gas_res = ((5.0f / voltage) - 1.0f) * 15000.0f;

        strncpy(payload.fields[0].name, "gas_res", SENSOR_FIELD_NAME_LEN - 1);
        payload.fields[0].name[SENSOR_FIELD_NAME_LEN - 1] = '\0';
        payload.fields[0].type = SENSOR_FIELD_DATATYPE_FLOAT;
        payload.fields[0].value.f = (gas_res >= 0.0f) ? gas_res : 0.0f;

        payload.field_count = 1U;

        if (xQueueSend(out_queue, &payload, pdMS_TO_TICKS(TASK_QUEUE_SEND_TIMEOUT_MS)) != pdTRUE) {
          ESP_LOGE(TAG, "Failed sending from TGS2602 sampling task");
        }
      }
      tgs2602.requestADC(0U);
    }
    vTaskDelay(pdMS_TO_TICKS(TASK_TGS2602_SAMPLING_SAMPLE_PERIOD_MS));
  }
}

static void
tgs_sample_module_cleanup() {
  if (task_tgs2602_sampling_handle)
    vTaskDelete(task_tgs2602_sampling_handle);

  task_tgs2602_sampling_handle = NULL;
}