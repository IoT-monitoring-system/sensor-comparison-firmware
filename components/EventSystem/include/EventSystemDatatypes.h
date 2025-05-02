#pragma once
#ifndef EVENTDATATYPES_H
#define EVENTDATATYPES_H

#include <freertos/FreeRTOS.h>

#include "esp_types.h"

struct ESWaitsConfig {
  TickType_t receiveWaitTicks;
  TickType_t sendWaitTicks;
  TickType_t accessWaitTicks;
};

#define DEFAULT_ESWAITS_CONFIG                                                 \
  ESWaitsConfig { portMAX_DELAY, pdMS_TO_TICKS(1000), pdMS_TO_TICKS(1000) }

template <typename EventType> struct EventDescriptor {
  EventType event;
  void *data;
  uint32_t dataSize;
};

#endif