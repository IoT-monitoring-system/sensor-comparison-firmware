#pragma once
#ifndef SNTP_MODULE_H
#define SNTP_MODULE_H

#include "esp_err.h"

#include "sntp_module_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t
sntp_module_init(struct sntp_module_config *sntp_cfg);
esp_err_t
sntp_module_del(void);

esp_err_t
sntp_module_start(void);
esp_err_t
sntp_module_stop(void);

enum sntp_module_sync_state
sntp_module_get_state(void);

esp_err_t
sntp_module_push_server(const char *server);
esp_err_t
sntp_module_pop_server(void);

esp_err_t
sntp_module_sync_wait(uint32_t time_to_wait_ms);

#ifdef __cplusplus
}
#endif

#endif