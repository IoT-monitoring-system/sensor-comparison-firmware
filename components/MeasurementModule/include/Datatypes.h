#pragma once
#ifndef DATATYPES_H
#define DATATYPES_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_types.h"

/* #region ERROR CODES */
#define ESP_ERR_MEASUREMENT_TASK_BASE 0x7000U

#define ESP_ERR_INVALID_POLLING_RATE (ESP_ERR_MEASUREMENT_TASK_BASE + 1U)
#define ESP_ERR_INVALID_STACK_DEPTH (ESP_ERR_MEASUREMENT_TASK_BASE + 2U)
#define ESP_ERR_INVALID_CORE_ID (ESP_ERR_MEASUREMENT_TASK_BASE + 3U)
#define ESP_ERR_INVALID_PRIORITY (ESP_ERR_MEASUREMENT_TASK_BASE + 4U)
#define ESP_ERR_INVALID_PC_NAME (ESP_ERR_MEASUREMENT_TASK_BASE + 5U)
#define ESP_ERR_INVALID_MEASUREMENT_TASK (ESP_ERR_MEASUREMENT_TASK_BASE + 6U)

#define ESP_ERR_INIT_FAIL (ESP_ERR_MEASUREMENT_TASK_BASE + 7U)
#define ESP_ERR_START_FAIL (ESP_ERR_MEASUREMENT_TASK_BASE + 8U)
#define ESP_ERR_STOP_FAIL (ESP_ERR_MEASUREMENT_TASK_BASE + 9U)
#define ESP_ERR_RESET_FAIL (ESP_ERR_MEASUREMENT_TASK_BASE + 10U)

#define ESP_ERR_MEASUREMENT_FAIL (ESP_ERR_MEASUREMENT_TASK_BASE + 10U)

#define ESP_ERR_NOT_CONFIGURED (ESP_ERR_MEASUREMENT_TASK_BASE + 11U)

#define ESP_ERR_TASK_CREATE_FAIL (ESP_ERR_MEASUREMENT_TASK_BASE + 13U)
#define ESP_ERR_IS_RUNNING (ESP_ERR_MEASUREMENT_TASK_BASE + 14U)
#define ESP_ERR_NOT_RUNNING (ESP_ERR_MEASUREMENT_TASK_BASE + 15U)
/* #endregion */

/* #region EVENT IDS */
enum measurement_task_event {
  MEASUREMENT_TASK_DATA_UPDATE_EVENT = 0,

  MEASUREMENT_TASK_TOTAL_EVENTS /* Should always be at the end */
};

/* #endregion */

/* #region CONFIGURATION STRUCTS */
struct measurement_task_config {
  uint32_t pollingRate; /* Hertz */

  uint32_t usStackDepth;
  BaseType_t xCoreID;
  UBaseType_t uxPriority;

  char *pcName;
};
struct measurement_task_eventloop_config {
  TickType_t sendWaitTicks;
  TickType_t accessWaitTicks;
  TickType_t receiveWaitTicks; /* How long do we wait for all handlers to
                                  process the event, 0 - no waiting*/
};
/* #endregion */

/* #region DEFAULT CONFIGURATIONS */
#define DEFAULT_MEASUREMENT_TASK_CONFIG                                        \
  measurement_task_config {                                                    \
    pdMS_TO_TICKS(1000), 1024U, tskNO_AFFINITY, tskIDLE_PRIORITY,              \
        (char *)"MeasurementTask"                                              \
  }

#define DEFAULT_EVENTLOOP_ARGS                                                 \
  esp_event_loop_args_t {                                                      \
    4U, (char *)"EventLoop", tskIDLE_PRIORITY, 1024U, tskNO_AFFINITY           \
  }

#define DEFAULT_EVENTLOOP_CONFIG                                               \
  measurement_task_eventloop_config {                                          \
    pdMS_TO_TICKS(1000), pdMS_TO_TICKS(1000), pdMS_TO_TICKS(1000)              \
  }
/* #endregion */

#endif