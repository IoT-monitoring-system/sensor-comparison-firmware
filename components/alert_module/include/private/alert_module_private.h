#pragma once
#ifndef ALERT_MODULE_PRIVATE_H
#define ALERT_MODULE_PRIVATE_H

#include "alert_module_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ALERT_MODULE_TASK_NAME             "alertModuleTsk"
#define ALERT_MODULE_ACCESS_MUX_TIMEOUT_MS 10000U

#define ALERT_MODULE_MAX_ALERTS 10

struct alert_internal {
  uint8_t alert_id;
  char name[ALERT_MODULE_NAME_MAX_LEN];

  SemaphoreHandle_t access_mux;

  bool auto_expire;
  uint64_t expiry_time_ms;

  uint32_t alert_notify_mask;

  enum alert_state state;
  alert_state_change_handler state_change_handler;

  uint64_t state_change_time_us;
};

#ifdef __cplusplus
}
#endif
#endif