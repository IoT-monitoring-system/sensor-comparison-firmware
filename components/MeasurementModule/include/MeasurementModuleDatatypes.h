#pragma once
#ifndef DATATYPES_H
#define DATATYPES_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_types.h"

/* #region EVENT IDS */
enum MeasurementTaskEvent {
  MEASUREMENT_TASK_DATA_UPDATE_EVENT = 0,

  MEASUREMENT_TASK_TOTAL_EVENTS /* Should always be at the end */
};

enum MeasurementTaskLifecycleHook {
  MT_PRE_CONFIGURE_HOOK = 0,
  MT_POST_CONFIGURE_HOOK,

  MT_PRE_START_HOOK,
  MT_POST_START_HOOK,

  MT_PRE_STOP_HOOK,
  MT_POST_STOP_HOOK,

  MT_TOTAL_HOOKS
};

/* #endregion */

/* #region CONFIGURATION STRUCTS */
struct MeasurementTaskConfig {
  float sampleRate; /* Hertz */

  uint32_t usStackDepth;
  BaseType_t xCoreID;
  UBaseType_t uxPriority;

  char *pcName;

  TickType_t sendWaitTicks;
  TickType_t accessWaitTicks;
  TickType_t receiveWaitTicks; /* How long do we wait for all handlers to
                                  process the event, 0 - no waiting*/
};

/* #endregion */

/* #region DEFAULT CONFIGURATIONS */
#define DEFAULT_MEASUREMENT_TASK_CONFIG                                        \
  MeasurementTaskConfig {                                                      \
    pdMS_TO_TICKS(1000), 1024U, tskNO_AFFINITY, tskIDLE_PRIORITY,              \
        (char *)"MeasurementTask", pdMS_TO_TICKS(1000), pdMS_TO_TICKS(1000),   \
        pdMS_TO_TICKS(1000)                                                    \
  }

/* #endregion */

#endif