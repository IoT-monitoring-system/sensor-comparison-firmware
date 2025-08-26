#pragma once
#ifndef ALERT_MODULE_DEFS_H
#define ALERT_MODULE_DEFS_H

#include "stdbool.h"
#include "stdint.h"

#include "esp_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ALERT_MODULE_NAME_MAX_LEN 8U

enum alert_state {
  ALERT_STATE_INVALID = 0,
  ALERT_STATE_NORMAL,
  ALERT_STATE_SET,
  ALERT_STATE_EXPIRED,
};

#define ALERT_NOTIFY_NORMAL  (1U << ALERT_STATE_NORMAL)
#define ALERT_NOTIFY_SET     (1U << ALERT_STATE_SET)
#define ALERT_NOTIFY_EXPIRED (1U << ALERT_STATE_EXPIRED)

struct alert_descriptor {
  uint8_t alert_id;
  char name[ALERT_MODULE_NAME_MAX_LEN];

  uint64_t timestamp_us;

  enum alert_state state;
};

typedef bool (*alert_state_change_handler)(const struct alert_descriptor *);

struct alert_config {
  uint8_t alert_id;
  char name[ALERT_MODULE_NAME_MAX_LEN];

  uint32_t alert_notify_mask;

  bool auto_expire;
  uint64_t expiry_time_ms;

  alert_state_change_handler state_change_handler;
};

typedef struct alert_module_instance *alert_module_handle;

struct alert_module_config {
  uint32_t task_stack_size;
  uint8_t task_priority;
  uint8_t task_core_affinity;
  uint64_t task_period_ms;
};

#ifdef __cplusplus
}
#endif
#endif