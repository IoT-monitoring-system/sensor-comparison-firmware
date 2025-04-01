#pragma once
#ifndef EVENTDATATYPES_H
#define EVENTDATATYPES_H

#include <sys/_stdint.h>

#define ESP_ERR_EVENT_SYSTEM_BASE 0x7500U

#define ESP_ERR_MAX_EVENT_HANDLERS (ESP_ERR_EVENT_SYSTEM_BASE + 1U)
#define ESP_ERR_NO_EVENT_HANDLERS (ESP_ERR_EVENT_SYSTEM_BASE + 2U)
#define ESP_ERR_EVENT_HANDLER_NOT_FOUND (ESP_ERR_EVENT_SYSTEM_BASE + 3U)

template <typename EventType> struct EventDescriptor {
  EventType event;
  void *data;
  uint32_t dataSize;
};

#endif