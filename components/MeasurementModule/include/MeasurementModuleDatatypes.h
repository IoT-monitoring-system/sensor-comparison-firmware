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

/* #endregion */

/* #region CONFIGURATION STRUCTS */
struct MeasurementTaskConfig {
  uint32_t pollingRate; /* Hertz */

  uint32_t usStackDepth;
  BaseType_t xCoreID;
  UBaseType_t uxPriority;

  char *pcName;
};
struct MeasurementTaskEventloopConfig {
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
        (char *)"MeasurementTask"                                              \
  }

#define DEFAULT_EVENTLOOP_CONFIG                                               \
  MeasurementTaskEventloopConfig {                                             \
    pdMS_TO_TICKS(1000), pdMS_TO_TICKS(1000), pdMS_TO_TICKS(1000)              \
  }
/* #endregion */

#endif