#include "esp_timer.h"

#include "ClockSyncModule.h"

volatile uint64_t lastPulseESP = 0;
volatile uint64_t firstPulseESP = 0;
volatile uint32_t pulseCount = 0;
volatile int64_t interDeviceOffset = 0;

// Filter constants
const float ALPHA = 0.1;
volatile int64_t offsetEMA = 0;

void IRAM_ATTR onSyncPulse() {
  uint64_t now = esp_timer_get_time();

  if (pulseCount == 0) {
    firstPulseESP = now;
    offsetEMA = 0;
  } else {
    uint64_t expected = firstPulseESP + (pulseCount * SYNC_PERIOD_US);
    int64_t drift = now - expected;

    offsetEMA = (1.0 - ALPHA) * offsetEMA + ALPHA * drift;

    interDeviceOffset = -offsetEMA;
  }

  lastPulseESP = now;
  pulseCount++;
}

esp_err_t CLKSyncModule::setupSyncInput(uint8_t pin) {
  pinMode(pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin), onSyncPulse, RISING);

  return ESP_OK;
}
