#pragma once
#ifndef ADC_MODULE_H
#define ADC_MODULE_H

#include "esp_err.h"

#include "adc_module_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct adc_dev *adc_dev_handle;

esp_err_t
adc_module_enable_dev(adc_dev_handle device);

esp_err_t
adc_module_disable_dev(adc_dev_handle device);

esp_err_t
adc_module_del_dev(adc_dev_handle device);

esp_err_t
adc_module_get_data(adc_dev_handle adc_device, uint32_t samples_to_read, uint32_t *out_data, uint32_t *samples_read);

// esp_err_t
// adc_module_init_dev(const struct adc_dev_config_t *adc_dev_cfg, adc_dev_handle *ret_device);

esp_err_t
adc_module_init_dev_oneshot(const struct adc_base_dev_config_t *adc_base_cfg, const struct adc_dev_oneshot_config_t *adc_dev_cfg,
                            adc_dev_handle *ret_device);

esp_err_t
adc_module_init_dev_continuous(const struct adc_base_dev_config_t *adc_base_cfg, const struct adc_dev_cont_config_t *adc_dev_cfg,
                               adc_dev_handle *ret_device);

// esp_err_t
// adc_module_init();

esp_err_t
adc_module_del();

#ifdef __cplusplus
}
#endif

#endif