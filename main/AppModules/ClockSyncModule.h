#pragma once
#ifndef CLOCK_SYNC_MODULE_H
#define CLOCK_SYNC_MODULE_H

#include "esp_attr.h"
#include "esp_err.h"

#include "Arduino.h"

#define SYNC_PERIOD_US 10000ULL // 1 Hz = 1,000,000 us


void IRAM_ATTR onSyncPulse();

extern volatile int64_t interDeviceOffset;

typedef void (*DockingTriggerHandler)();

class CLKSyncModule {
  public:
    esp_err_t setupSyncInput(uint8_t pin);
    esp_err_t setupSyncOutput(uint8_t pin);

    esp_err_t setupDockingInput(uint8_t pin);
    esp_err_t setupDockingOutput(uint8_t pin, DockingTriggerHandler handler);
};

#endif