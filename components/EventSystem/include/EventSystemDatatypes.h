#pragma once
#ifndef EVENTDATATYPES_H
#define EVENTDATATYPES_H

#include "esp_types.h"

template <typename EventType> struct EventDescriptor {
  EventType event;
  void *data;
  uint32_t dataSize;
};

#endif