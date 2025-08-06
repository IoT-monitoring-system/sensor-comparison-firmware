#pragma once
#ifndef ADC_MODULE_DEFS_H
#define ADC_MODULE_DEFS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum adc_module_mode {
  ADC_MODULE_MODE_DEFAULT = 0,
  ADC_MODULE_MODE_ONESHOT,
  ADC_MODULE_MODE_CONTINUOUS,
};

struct adc_base_dev_config_t {
  uint8_t data_pin;
  uint8_t data_bit_width;

  bool perform_calibration;
};

struct adc_dev_oneshot_config_t {};

struct adc_dev_cont_config_t {
  uint32_t sample_freq_hz;
};

#ifdef __cplusplus
}
#endif

#endif