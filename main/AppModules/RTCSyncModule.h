#pragma once
#ifndef RTC_SYNC_MODULE_H
#define RTC_SYNC_MODULE_H

#include <Arduino.h>
#include <Wire.h>

#include "RTClib.h"

/**
 * @note Currently supports only RTC_DS1307
 */
class RTCSyncModule {
public:
  esp_err_t begin(TwoWire *wireobj = &Wire);

  bool isRunning();

  time_t getRTCUnixTime();

private:
  TwoWire *wireobj;
  RTC_DS1307 rtc;
};

#endif