#pragma once
#ifndef ALERT_MODULE_H
#define ALERT_MODULE_H

#include "esp_err.h"

#include "alert_module_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t
alert_module_init(const struct alert_module_config *module_cfg, alert_module_handle *out_module);
esp_err_t
alert_module_del(alert_module_handle module);

esp_err_t
alert_module_register_alert(alert_module_handle module, const struct alert_config *alert_cfg);
esp_err_t
alert_module_unregister_alert(alert_module_handle module, uint8_t alert_id);

esp_err_t
alert_module_set_alert(alert_module_handle module, uint8_t alert_id);
esp_err_t
alert_module_reset_alert(alert_module_handle module, uint8_t alert_id);

#ifdef __cplusplus
}
#endif
#endif