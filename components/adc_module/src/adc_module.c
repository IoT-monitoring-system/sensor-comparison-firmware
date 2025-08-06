#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <sys/cdefs.h>

#include "freertos/FreeRTOS.h"

#include "FreeRTOSConfig.h"
#include "freertos/task.h"

#include "portmacro.h"

#include "esp_check.h"
#include "esp_err.h"

#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_oneshot.h"
#include "soc/adc_periph.h"
#include "soc/gpio_num.h"
#include "soc/soc_caps.h"

#include "adc_module.h"
#include "adc_module_defs.h"
#include "private/adc_module_private.h"

typedef struct {
  bool is_enabled;

  adc_channel_t id;
} esp_adc_channel_t;

typedef struct {
  bool is_configured;

  adc_unit_t id;

  enum adc_module_mode mode;
  union {
    adc_oneshot_unit_handle_t oneshot;
    adc_continuous_handle_t cont;
  } handle;
  esp_adc_channel_t channels[SOC_ADC_MAX_CHANNEL_NUM];
} esp_adc_unit_t;

struct adc_dev {
  struct adc_base_dev_config_t cfg_base;
  union {
    struct adc_dev_oneshot_config_t oneshot;
    struct adc_dev_cont_config_t cont;
  } cfg;

  struct {
    bool is_calibrated;
    adc_cali_handle_t handle;
    union {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
      adc_cali_curve_fitting_config_t curve_fitting_cfg;
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
      adc_cali_line_fitting_config_t line_fitting_cfg;
#endif
    } method;
  } adc_cali;

  esp_adc_channel_t *adc_channel;
  esp_adc_unit_t *adc_unit;
};

static const char *TAG = "adc_module";

static esp_adc_unit_t adc_units[SOC_ADC_PERIPH_NUM] = {0}; // allocate dynamically?

static bool
is_cont_adc_configured();

static esp_err_t
init_unit(adc_unit_t unit_id, enum adc_module_mode mode);
static esp_err_t
init_unit_cont(adc_unit_t unit_id);
static esp_err_t
init_unit_oneshot(adc_unit_t unit_id);

static esp_err_t
del_unit(adc_unit_t unit_id, enum adc_module_mode mode);
static esp_err_t
del_unit_cont(adc_unit_t unit_id);
static esp_err_t
del_unit_oneshot(adc_unit_t unit_id);

static esp_err_t
enable_dev(adc_dev_handle adc_device, enum adc_module_mode mode);
static esp_err_t
enable_dev_cont(adc_dev_handle adc_device);
static esp_err_t
enable_dev_oneshot(adc_dev_handle adc_device);

static esp_err_t
disable_dev(adc_dev_handle adc_device, enum adc_module_mode mode);
static esp_err_t
disable_dev_oneshot(adc_dev_handle adc_device);
static esp_err_t
disable_dev_cont(adc_dev_handle adc_device);

esp_err_t
adc_module_enable_dev(adc_dev_handle adc_device) {
  ESP_RETURN_ON_FALSE(adc_device, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  ESP_RETURN_ON_FALSE(adc_device->adc_unit->is_configured, ESP_ERR_INVALID_STATE, TAG,
                      "adc channel enable failed, adc unit is not configured");
  ESP_RETURN_ON_FALSE(!adc_device->adc_channel->is_enabled, ESP_ERR_INVALID_STATE, TAG,
                      "adc channel enable failed, channel is already enabled");

  enable_dev(adc_device, adc_device->adc_unit->mode);

  if (adc_device->cfg_base.perform_calibration) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = adc_device->adc_unit->id,
        .chan = adc_device->adc_channel->id,
        .atten = ESP_ADC_ATTEN,
        .bitwidth = (adc_bitwidth_t)adc_device->cfg_base.data_bit_width,
    };
    ESP_RETURN_ON_ERROR(adc_cali_create_scheme_curve_fitting(&cali_config, &adc_device->adc_cali.handle), TAG,
                        "adc channel curve fitting calibration failed");
    adc_device->adc_cali.method.curve_fitting_cfg = cali_config;
    adc_device->adc_cali.is_calibrated = true;
    ESP_LOGD(TAG, "calibrated a oneshot adc channel, unit_id:%u channel_id:%u, using curve fitting calibration scheme",
             (uint8_t)adc_device->adc_unit->id, (uint8_t)adc_device->adc_channel->id);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = adc_device->adc_unit->id,
        .atten = ESP_ADC_ATTEN,
        .bitwidth = (adc_bitwidth_t)adc_device->cfg_base.data_bit_width,
    };
    ESP_RETURN_ON_ERROR(adc_cali_create_scheme_line_fitting(&cali_config, &adc_device->adc_cali.handle), TAG,
                        "adc channel line fitting calibration failed");
    adc_device->adc_cali.method.line_fitting_cfg = cali_config;
    adc_device->adc_cali.is_calibrated = true;
    ESP_LOGD(TAG, "calibrated a oneshot adc channel, unit_id:%u channel_id:%u, using line fitting calibration scheme",
             (uint8_t)adc_device->adc_unit->id, (uint8_t)adc_device->adc_channel->id);
#endif
  }
  return ESP_OK;
}

esp_err_t
adc_module_disable_dev(adc_dev_handle adc_device) {
  ESP_RETURN_ON_FALSE(adc_device, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  ESP_RETURN_ON_FALSE(adc_device->adc_unit->is_configured, ESP_ERR_INVALID_STATE, TAG,
                      "adc channel disable failed, adc unit is not configured");
  ESP_RETURN_ON_FALSE(adc_device->adc_channel->is_enabled, ESP_ERR_INVALID_STATE, TAG,
                      "adc channel disable failed, channel is not enabled");
  disable_dev(adc_device, adc_device->adc_unit->mode);

  if (adc_device->adc_cali.is_calibrated) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_RETURN_ON_ERROR(adc_cali_delete_scheme_curve_fitting(adc_device->adc_cali.handle), TAG,
                        "adc channel curve fitting calibration delete failed");
    adc_device->adc_cali.is_calibrated = false;
    ESP_LOGD(TAG, "deleted a curve fitting calibration scheme for a oneshot adc channel, unit_id:%u channel_id:%u pin:%u",
             (uint8_t)adc_device->adc_unit->id, (uint8_t)adc_device->adc_channel->id, (uint8_t)adc_device->cfg_base.data_pin);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_RETURN_ON_ERROR(adc_cali_delete_scheme_line_fitting(adc_device->adc_cali.handle), TAG,
                        "adc channel line fitting calibration delete failed");
    adc_device->adc_cali.is_calibrated = false;
    ESP_LOGD(TAG, "deleted a line fitting calibration scheme for a oneshot adc channel, unit_id:%u channel_id:%u pin:%u",
             (uint8_t)adc_device->adc_unit->id, (uint8_t)adc_device->adc_channel->id, (uint8_t)adc_device->cfg_base.data_pin);
#endif
  }
  return ESP_OK;
}

/**
 * NOTE: Very inefficient but the driver is ass
 */
esp_err_t
adc_module_get_data(adc_dev_handle adc_device, uint32_t samples_to_read, uint32_t *out_data, uint32_t *samples_read) {
  ESP_RETURN_ON_FALSE(adc_device && out_data && samples_read, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  ESP_RETURN_ON_FALSE(samples_to_read > 0, ESP_ERR_INVALID_ARG, TAG, "zero samples requested");

  esp_err_t ret = ESP_OK;
  uint32_t read_count = 0;

  switch (adc_device->adc_unit->mode) {
  case ADC_MODULE_MODE_ONESHOT: {
    for (uint32_t i = 0; i < samples_to_read; ++i) {
      int raw = 0;
      ret = adc_oneshot_read(adc_device->adc_unit->handle.oneshot, adc_device->adc_channel->id, &raw);
      if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Oneshot read failed at sample %lu", i);
        break;
      }

      int voltage = 0;
      if (adc_device->adc_cali.is_calibrated) {
        ret = adc_cali_raw_to_voltage(adc_device->adc_cali.handle, raw, &voltage);
        if (ret != ESP_OK) {
          ESP_LOGW(TAG, "Calibration failed at sample %lu", i);
          break;
        }
      } else {
        voltage = raw * ((float)ADC_MAX_MV / ((1U << adc_device->cfg_base.data_bit_width) - 1U));
      }

      out_data[i] = (uint32_t)voltage;
      ++read_count;
    }
    break;
  }

  case ADC_MODULE_MODE_CONTINUOUS: {
    const uint32_t dma_buf_bytes = samples_to_read * sizeof(adc_digi_output_data_t);
    uint8_t *dma_buf = (uint8_t *)heap_caps_malloc(dma_buf_bytes, MALLOC_CAP_INTERNAL);
    ESP_RETURN_ON_FALSE(dma_buf, ESP_ERR_NO_MEM, TAG, "failed to allocate DMA buffer");

    uint32_t raw_bytes_read = 0;
    ret = adc_continuous_read(adc_device->adc_unit->handle.cont, dma_buf, dma_buf_bytes, &raw_bytes_read, 0);
    if (ret != ESP_OK || raw_bytes_read == 0) {
      ESP_LOGW(TAG, "No DMA data available");
      free(dma_buf);
      return ESP_ERR_TIMEOUT;
    }

    uint32_t raw_samples = raw_bytes_read / sizeof(adc_digi_output_data_t);
    adc_digi_output_data_t *results = (adc_digi_output_data_t *)dma_buf;

    for (uint32_t i = 0; i < raw_samples && read_count < samples_to_read; ++i) {
      adc_digi_output_data_t res = results[i];

      if (res.type2.channel > SOC_ADC_CHANNEL_NUM(adc_device->adc_unit->id))
        continue;
      if (res.type2.unit != adc_device->adc_unit->id)
        continue;
      if (res.type2.channel != adc_device->adc_channel->id)
        continue;

      uint16_t raw = res.type2.data;
      int voltage = 0;

      if (adc_device->adc_cali.is_calibrated) {
        ret = adc_cali_raw_to_voltage(adc_device->adc_cali.handle, raw, &voltage);
        if (ret != ESP_OK) {
          ESP_LOGW(TAG, "Calibration failed at sample %lu", i);
          continue;
        }
      } else {
        voltage = raw * ((float)ADC_MAX_MV / ((1U << adc_device->cfg_base.data_bit_width) - 1U));
      }

      out_data[read_count++] = (uint32_t)voltage;
    }

    free(dma_buf);
    break;
  }

  default:
    ESP_LOGE(TAG, "Invalid ADC mode");
    return ESP_ERR_INVALID_ARG;
  }

  *samples_read = read_count;
  return ESP_OK;
}

esp_err_t
adc_module_del_dev(adc_dev_handle adc_device) {
  ESP_RETURN_ON_FALSE(adc_device, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  ESP_RETURN_ON_FALSE(!adc_device->adc_channel->is_enabled, ESP_ERR_INVALID_STATE, TAG,
                      "delete device failed, make sure it is disabled");
  ESP_RETURN_ON_FALSE(!adc_device->adc_cali.is_calibrated, ESP_ERR_INVALID_STATE, TAG,
                      "delete device failed, make sure to delete the calibration scheme");

  free(adc_device);
  return ESP_OK;
}

esp_err_t
adc_module_del() {
  for (uint8_t id = 0; id < SOC_ADC_PERIPH_NUM; ++id) {
    if (!adc_units[id].is_configured)
      continue;
    ESP_RETURN_ON_ERROR(del_unit((adc_unit_t)id, adc_units[id].mode), TAG, "failed to del an adc unit");
  }
  return ESP_OK;
}

esp_err_t
adc_module_init_dev_oneshot(const struct adc_base_dev_config_t *adc_base_cfg, const struct adc_dev_oneshot_config_t *adc_dev_cfg,
                            adc_dev_handle *ret_device) {
  esp_err_t ret = ESP_OK;

  ESP_RETURN_ON_FALSE(adc_dev_cfg && adc_base_cfg && ret_device, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  ESP_RETURN_ON_FALSE(!is_cont_adc_configured(), ESP_ERR_INVALID_STATE, TAG, "only one device permitted in continuous mode");

  gpio_num_t pin = adc_base_cfg->data_pin;
  adc_unit_t unit = ADC_UNIT_1;
  adc_channel_t channel = -1;

  for (uint8_t unit_idx = 0; unit_idx < SOC_ADC_PERIPH_NUM; ++unit_idx) {
#if CONFIG_IDF_TARGET_ESP32S3
    if (unit_idx == ADC_UNIT_2)
      continue;
#endif
    for (uint8_t ch_idx = 0; ch_idx < SOC_ADC_CHANNEL_NUM(unit_idx); ++ch_idx) {
      if (!adc_units[unit_idx].channels[ch_idx].is_enabled && ADC_GET_IO_NUM(unit_idx, ch_idx) == pin) {
        unit = (adc_unit_t)unit_idx;
        channel = (adc_channel_t)ch_idx;
        break;
      }
    }
    if (channel != -1)
      break;
  }
  ESP_RETURN_ON_FALSE(channel != -1, ESP_ERR_NOT_FOUND, TAG, "no suitable adc channel found");
  ESP_RETURN_ON_ERROR(init_unit(unit, ADC_MODULE_MODE_ONESHOT), TAG, "unit init failed");

  struct adc_dev *adc_device = calloc(1, sizeof(struct adc_dev));
  ESP_GOTO_ON_FALSE(adc_device, ESP_ERR_NO_MEM, err, TAG, "allocation failed");

  adc_device->adc_unit = &adc_units[unit];
  adc_device->adc_channel = &adc_units[unit].channels[channel];
  adc_device->adc_channel->id = channel;
  adc_device->adc_unit->mode = ADC_MODULE_MODE_ONESHOT;

  adc_device->cfg_base.data_bit_width = adc_base_cfg->data_bit_width;
  adc_device->cfg_base.data_pin = adc_base_cfg->data_pin;
  adc_device->cfg_base.perform_calibration = adc_base_cfg->perform_calibration;

  *ret_device = adc_device;
  return ESP_OK;

err:
  if (adc_units[unit].is_configured) {
    del_unit(unit, ADC_MODULE_MODE_ONESHOT);
  }
  if (adc_device) {
    free(adc_device);
  }
  return ret;
}
esp_err_t
adc_module_init_dev_continuous(const struct adc_base_dev_config_t *adc_base_cfg, const struct adc_dev_cont_config_t *adc_dev_cfg,
                               adc_dev_handle *ret_device) {
  esp_err_t ret = ESP_OK;
  ESP_RETURN_ON_FALSE(adc_dev_cfg && ret_device, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  ESP_RETURN_ON_FALSE(!is_cont_adc_configured(), ESP_ERR_INVALID_STATE, TAG, "only one device permitted in continuous mode");

  gpio_num_t pin = adc_base_cfg->data_pin;
  adc_unit_t unit = ADC_UNIT_1;
  adc_channel_t channel = -1;

  for (uint8_t unit_idx = 0; unit_idx < SOC_ADC_PERIPH_NUM; ++unit_idx) {
#if CONFIG_IDF_TARGET_ESP32S3
    if (unit_idx == ADC_UNIT_2)
      continue;
#endif
    for (uint8_t ch_idx = 0; ch_idx < SOC_ADC_CHANNEL_NUM(unit_idx); ++ch_idx) {
      if (!adc_units[unit_idx].channels[ch_idx].is_enabled && ADC_GET_IO_NUM(unit_idx, ch_idx) == pin) {
        unit = (adc_unit_t)unit_idx;
        channel = (adc_channel_t)ch_idx;
        break;
      }
    }
    if (channel != -1)
      break;
  }

  ESP_RETURN_ON_FALSE(channel != -1, ESP_ERR_NOT_FOUND, TAG, "no suitable adc channel found");
  ESP_RETURN_ON_ERROR(init_unit(unit, ADC_MODULE_MODE_CONTINUOUS), TAG, "unit init failed");

  adc_digi_pattern_config_t pattern = {
      .atten = ESP_ADC_ATTEN,
      .channel = channel,
      .unit = unit,
      .bit_width = adc_base_cfg->data_bit_width,
  };

  adc_continuous_config_t cont_cfg = {
      .sample_freq_hz = adc_dev_cfg->sample_freq_hz,
      .conv_mode = ADC_CONV_SINGLE_UNIT_1,
      .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
      .pattern_num = 1,
      .adc_pattern = &pattern,
  };

  ESP_RETURN_ON_ERROR(adc_continuous_config(adc_units[unit].handle.cont, &cont_cfg), TAG, "config cont unit failed");

  struct adc_dev *adc_device = calloc(1, sizeof(struct adc_dev));
  ESP_GOTO_ON_FALSE(adc_device, ESP_ERR_NO_MEM, err, TAG, "calloc failed");

  adc_device->adc_unit = &adc_units[unit];
  adc_device->adc_channel = &adc_units[unit].channels[channel];
  adc_device->adc_channel->id = channel;
  adc_device->adc_unit->mode = ADC_MODULE_MODE_CONTINUOUS;

  adc_device->cfg_base.data_bit_width = adc_base_cfg->data_bit_width;
  adc_device->cfg_base.data_pin = adc_base_cfg->data_pin;
  adc_device->cfg_base.perform_calibration = adc_base_cfg->perform_calibration;
  adc_device->cfg.cont.sample_freq_hz = adc_dev_cfg->sample_freq_hz;

  *ret_device = adc_device;
  return ESP_OK;

err:
  if (adc_units[unit].is_configured) {
    del_unit(unit, ADC_MODULE_MODE_CONTINUOUS);
  }
  if (adc_device) {
    free(adc_device);
  }
  return ret;
}

static bool
is_cont_adc_configured() {
  for (uint8_t i = 0; i < SOC_ADC_PERIPH_NUM; ++i) {
    if (adc_units[i].mode == ADC_MODULE_MODE_CONTINUOUS && adc_units[i].is_configured) {
      return true;
    }
  }
  return false;
}

static esp_err_t
init_unit(adc_unit_t unit_id, enum adc_module_mode mode) {
  uint8_t id = (uint8_t)unit_id;
  ESP_RETURN_ON_FALSE(id < SOC_ADC_PERIPH_NUM, ESP_ERR_INVALID_ARG, TAG, "invalid adc unit id");

  esp_err_t ret = ESP_OK;

  switch (mode) {
  case ADC_MODULE_MODE_CONTINUOUS:
    ret = init_unit_cont(unit_id);
    break;
  case ADC_MODULE_MODE_ONESHOT:
    ret = init_unit_oneshot(unit_id);
    break;
  default:
    ESP_LOGE(TAG, "invalid mode requested");
    return ESP_ERR_INVALID_ARG;
  }

  ESP_RETURN_ON_ERROR(ret, TAG, "failed to init adc");

  adc_units[id].id = unit_id;
  adc_units[id].mode = mode;
  adc_units[id].is_configured = true;
  return ret;
}
static esp_err_t
init_unit_oneshot(adc_unit_t unit_id) {
  uint8_t id = (uint8_t)unit_id;
  adc_oneshot_unit_init_cfg_t init_config = {
      .unit_id = unit_id,
  };
  ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&init_config, &adc_units[id].handle.oneshot), TAG,
                      "create new adc oneshot unit failed");
  ESP_LOGD(TAG, "initialized oneshot adc unit %u", id);

  return ESP_OK;
}
static esp_err_t
init_unit_cont(adc_unit_t unit_id) {
  uint8_t id = (uint8_t)unit_id;

#if CONFIG_IDF_TARGET_ESP32S3
  ESP_RETURN_ON_FALSE(unit_id != ADC_UNIT_2, ESP_ERR_INVALID_ARG, TAG, "adc2 doesn't support DMA on S3");
#endif
  adc_continuous_handle_cfg_t cont_handle_cfg = {
      .max_store_buf_size = ADC_CONT_MAX_FRAMES_LENGTH,
      .conv_frame_size = ADC_CONT_CONV_FRAME_SIZE,
      .flags.flush_pool = ADC_CONT_FLUSH_POOL_IF_FULL,
  };
  ESP_RETURN_ON_ERROR(adc_continuous_new_handle(&cont_handle_cfg, &adc_units[id].handle.cont), TAG,
                      "create continuous adc handle failed");
  ESP_LOGD(TAG, "initialized continuous adc unit %u", id);

  return ESP_OK;
}

static esp_err_t
del_unit(adc_unit_t unit_id, enum adc_module_mode mode) {
  uint8_t id = (uint8_t)unit_id;
  ESP_RETURN_ON_FALSE(id < SOC_ADC_PERIPH_NUM, ESP_ERR_INVALID_ARG, TAG, "invalid adc unit id");

  esp_err_t ret = ESP_OK;

  switch (mode) {
  case ADC_MODULE_MODE_CONTINUOUS:
    ret = del_unit_cont(unit_id);
    break;
  case ADC_MODULE_MODE_ONESHOT:
    ret = del_unit_oneshot(unit_id);
    break;
  default:
    ESP_LOGE(TAG, "invalid mode requested");
    return ESP_ERR_INVALID_ARG;
  }

  ESP_RETURN_ON_ERROR(ret, TAG, "failed to del adc");

  adc_units[id].mode = ADC_MODULE_MODE_DEFAULT;
  adc_units[id].is_configured = false;
  return ret;
}
static esp_err_t
del_unit_oneshot(adc_unit_t unit_id) {
  return adc_oneshot_del_unit(adc_units[unit_id].handle.oneshot);
}
static esp_err_t
del_unit_cont(adc_unit_t unit_id) {
  return adc_continuous_deinit(adc_units[unit_id].handle.cont);
}

static esp_err_t
enable_dev(adc_dev_handle adc_device, enum adc_module_mode mode) {
  esp_err_t ret = ESP_OK;

  switch (mode) {
  case ADC_MODULE_MODE_CONTINUOUS:
    ret = enable_dev_cont(adc_device);
    break;
  case ADC_MODULE_MODE_ONESHOT:
    ret = enable_dev_oneshot(adc_device);
    break;
  default:
    ESP_LOGE(TAG, "invalid mode requested");
    return ESP_ERR_INVALID_ARG;
  }
  ESP_RETURN_ON_ERROR(ret, TAG, "failed to enable an adc device");

  adc_device->adc_channel->is_enabled = true;

  ESP_LOGD(TAG, "enabled a oneshot adc channel, unit_id:%u channel_id:%u pin:%u bitwidth:%u attenuation:%u ",
           (uint8_t)adc_device->adc_unit->id, (uint8_t)adc_device->adc_channel->id, (uint8_t)adc_device->cfg_base.data_pin,
           adc_device->cfg_base.data_bit_width, (uint8_t)ESP_ADC_ATTEN);

  return ret;
}
static esp_err_t
enable_dev_cont(adc_dev_handle adc_device) {
  return adc_continuous_start(adc_device->adc_unit->handle.cont);
}
static esp_err_t
enable_dev_oneshot(adc_dev_handle adc_device) {
  adc_oneshot_chan_cfg_t config = {
      .atten = ESP_ADC_ATTEN,
      .bitwidth = (adc_bitwidth_t)adc_device->cfg_base.data_bit_width,
  };
  return adc_oneshot_config_channel(adc_device->adc_unit->handle.oneshot, adc_device->adc_channel->id, &config);
}

static esp_err_t
disable_dev(adc_dev_handle adc_device, enum adc_module_mode mode) {
  esp_err_t ret = ESP_OK;

  switch (mode) {
  case ADC_MODULE_MODE_CONTINUOUS:
    ret = disable_dev_cont(adc_device);
    break;
  case ADC_MODULE_MODE_ONESHOT:
    ret = disable_dev_oneshot(adc_device);
    break;
  default:
    ESP_LOGE(TAG, "invalid mode requested");
    return ESP_ERR_INVALID_ARG;
  }
  ESP_RETURN_ON_ERROR(ret, TAG, "failed to disable an adc device");

  adc_device->adc_channel->is_enabled = false;

  ESP_LOGD(TAG, "disabled a oneshot adc channel, unit_id:%u channel_id:%u pin:%u", (uint8_t)adc_device->adc_unit->id,
           (uint8_t)adc_device->adc_channel->id, (uint8_t)adc_device->cfg_base.data_pin);

  return ret;
}
static esp_err_t
disable_dev_oneshot(adc_dev_handle adc_device) {
  return gpio_reset_pin((gpio_num_t)adc_device->cfg_base.data_pin);
}
static esp_err_t
disable_dev_cont(adc_dev_handle adc_device) {
  return adc_continuous_stop(adc_device->adc_unit->handle.cont);
}
