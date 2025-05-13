#include "esp_log.h"

#include "RTCSyncModule.h"

const char *TAG_RTC_SYNC_MODULE = "RTCSyncModule";

bool RTCSyncModule::isRunning() { return rtc.isrunning(); }

esp_err_t RTCSyncModule::begin(TwoWire *wireobj) {
  if (!wireobj)
    return ESP_FAIL;

  if (!rtc.begin(wireobj))
    return ESP_FAIL;

  this->wireobj = wireobj;

  return ESP_OK;
}

time_t RTCSyncModule::getRTCUnixTime() { return rtc.now().unixtime(); }
