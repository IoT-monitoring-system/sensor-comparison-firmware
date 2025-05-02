#pragma once
#ifndef EVENTSYSTEM_H
#define EVENTSYSTEM_H

#include <algorithm>
#include <array>
#include <functional>
#include <numeric>
#include <utility>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/ringbuf.h>
#include <freertos/semphr.h>

#include "esp_log.h"

#include "EventSystemDatatypes.h"
#include "EventSystemErrors.h"

static const char *TAG_EVENT_SYSTEM = "EventSystem";

static int findFirstSetBit(uint32_t num);

template <typename EventType> class EventProducer;

template <typename EventType> class EventConsumer {
  friend class EventProducer<EventType>;

public:
  EventConsumer(
      size_t bufferSize,
      RingbufferType_t bufferType,
      ESWaitsConfig waitsConfig) {
    this->_ringBuffer = xRingbufferCreate(bufferSize, bufferType);
    if (!this->_ringBuffer) {
      ESP_LOGE(TAG_EVENT_SYSTEM, "Mutex allocation failed");
      return;
    }

    this->_bufferSize = bufferSize;
    this->_bufferType = bufferType;

    // NOTE - May fail
    this->_accessMux = xSemaphoreCreateMutex();
    this->_waitsConfig = waitsConfig;

    if (!this->_accessMux) {
      ESP_LOGE(TAG_EVENT_SYSTEM, "Mutex allocation failed");
      return;
    }
  }

  /* Blocking event receive operation.
    pvBuffer and pvBuffer->data must be freed.
    Could be improved using smart pointers.*/
  esp_err_t listenForEvents(
      size_t itemSize,
      void *pvBuffer,
      int64_t receiveWaitTicks = -1,
      int64_t accessWaitTicks = -1) {
    esp_err_t result = ESP_OK;

    if (this->_bufferType != RINGBUF_TYPE_NOSPLIT)
      return ESP_ERR_NOT_SUPPORTED;
    if (itemSize > this->_bufferSize)
      return ESP_ERR_INVALID_SIZE;
    if (!pvBuffer)
      return ESP_ERR_INVALID_ARG;

    if (receiveWaitTicks < 0)
      receiveWaitTicks = this->_waitsConfig.receiveWaitTicks;
    if (accessWaitTicks < 0)
      accessWaitTicks = this->_waitsConfig.accessWaitTicks;

    ESP_LOGI(TAG_EVENT_SYSTEM, "About to block on ring buffer");
    void *item =
        xRingbufferReceive(this->_ringBuffer, &itemSize, receiveWaitTicks);
    ESP_LOGI(TAG_EVENT_SYSTEM, "All good");

    if (!item) {
      result = ESP_ERR_RECEIVE_WAIT_TIMEOUT;
    } else {
      auto newItem = new EventDescriptor<EventType>();

      memcpy(newItem, item, sizeof(EventDescriptor<EventType>));

      if (newItem->dataSize > 0) {
        uint8_t *newData = new uint8_t[newItem->dataSize];
        memcpy(
            newData,
            ((EventDescriptor<EventType> *)item)->data,
            newItem->dataSize);
        newItem->data = newData;
      } else {
        newItem->data = nullptr;
      }

      *(EventDescriptor<EventType> **)pvBuffer = newItem;

      vRingbufferReturnItem(this->_ringBuffer, item);
    }

    ESP_LOGI(TAG_EVENT_SYSTEM, "Setting bits");
    xEventGroupSetBits(this->_eventGroupHandle, this->_bitToSet);
    ESP_LOGI(TAG_EVENT_SYSTEM, "Finished setting bits");

    return result;
  }

  esp_err_t flushBuffer() {

    if (xSemaphoreTake(this->_accessMux, this->_waitsConfig.accessWaitTicks) !=
        pdPASS)
      return ESP_ERR_ACCESS_WAIT_TIMEOUT;
    vRingbufferDelete(this->_ringBuffer);
    this->_ringBuffer = xRingbufferCreate(this->_bufferSize, this->_bufferType);

    if (!this->_ringBuffer)
      return ESP_ERR_RING_BUFF_CREATE_FAIL;

    xSemaphoreGive(this->_accessMux);

    return ESP_OK;
  }

  size_t getBufferFreeSpace() {
    if (xSemaphoreTake(this->_accessMux, this->_waitsConfig.accessWaitTicks) !=
        pdPASS)
      return ESP_ERR_ACCESS_WAIT_TIMEOUT;
    size_t size = xRingbufferGetCurFreeSize(this->_ringBuffer);
    xSemaphoreGive(this->_accessMux);

    return size;
  }

protected:
  size_t _bufferSize;
  RingbufferType_t _bufferType;

  RingbufHandle_t _ringBuffer;

  ESWaitsConfig _waitsConfig;
  SemaphoreHandle_t _accessMux;

  /* These will be modified by the MeasurementTask whenever we use pass it to
   * subscribe to an event*/
  EventBits_t _bitToSet;
  EventGroupHandle_t _eventGroupHandle;

  /* Defined this way because it shouldn't make an impression to be a part of
   * the public API*/
  esp_err_t _receiveEvent(
      const void *pvItem,
      size_t xItemSize,
      int64_t sendWaitTicks = -1,
      int64_t accessWaitTicks = -1) {

    esp_err_t err = ESP_OK;

    if (accessWaitTicks < 0)
      accessWaitTicks = this->_waitsConfig.accessWaitTicks;
    if (sendWaitTicks < 0)
      sendWaitTicks = this->_waitsConfig.sendWaitTicks;

    if (xSemaphoreTake(this->_accessMux, accessWaitTicks) != pdPASS)
      return ESP_ERR_ACCESS_WAIT_TIMEOUT;

    if (xRingbufferSend(this->_ringBuffer, pvItem, xItemSize, sendWaitTicks) !=
        pdTRUE) {
      err = ESP_ERR_RING_BUFF_SEND_FAIL;
    }
    xSemaphoreGive(this->_accessMux);

    return err;
  }
};

template <typename EventType> class EventProducer {
public:
  EventProducer(uint8_t numEvents, ESWaitsConfig waitsConfig) {
    this->eventConsumers.resize(numEvents);

    for (uint8_t i = 0; i < numEvents; i++) {
      this->eventConsumers[i].first = xEventGroupCreate();
    }

    this->_waitsConfig = waitsConfig;
  }

  esp_err_t registerEventConsumer(
      EventConsumer<EventType> *eventConsumer,
      EventType event) {

    if (this->eventConsumers[event].second.size() == 32)
      return ESP_ERR_MAX_EVENT_HANDLERS;

    uint32_t possibleBits = 0xFFFFFFFF;

    possibleBits = std::accumulate(
        this->eventConsumers[event].second.begin(),
        this->eventConsumers[event].second.end(),
        possibleBits,
        [](uint32_t acc, const auto &eC) { return acc ^ eC->_bitToSet; });

    int bit = findFirstSetBit(possibleBits);

    if (bit < 0)
      return ESP_ERR_MAX_EVENT_HANDLERS;

    eventConsumer->_bitToSet = (1 << bit);
    eventConsumer->_eventGroupHandle = this->eventConsumers[event].first;

    // FIXME
    // Assuming the operation succeeds which won't always be the case.
    esp_err_t error = ESP_OK;
    this->eventConsumers[event].second.push_back(eventConsumer);

    return error;
  }

  esp_err_t unregisterEventConsumer(
      EventConsumer<EventType> *eventConsumer,
      EventType event) {
    if (!eventConsumer) {
      return ESP_ERR_INVALID_ARG;
    }

    auto res = std::find(
        this->eventConsumers[event].second.begin(),
        this->eventConsumers[event].second.end(),
        eventConsumer);

    if (res == this->eventConsumers[event].second.end()) {
      return ESP_ERR_EVENT_HANDLER_NOT_FOUND;
    }

    this->eventConsumers[event].second.erase(res);

    return ESP_OK;
  }

  esp_err_t produceEvent(
      void *data,
      size_t dataSize,
      EventType event,
      int64_t receiveWaitTicks = -1,
      int64_t sendWaitTicks = -1,
      int64_t accessWaitTicks = -1) {
    esp_err_t err = ESP_OK;

    if (!data)
      return ESP_ERR_INVALID_ARG;

    if (receiveWaitTicks < 0)
      receiveWaitTicks = this->_waitsConfig.receiveWaitTicks;
    if (accessWaitTicks < 0)
      accessWaitTicks = this->_waitsConfig.accessWaitTicks;
    if (sendWaitTicks < 0)
      sendWaitTicks = this->_waitsConfig.sendWaitTicks;

    EventDescriptor<EventType> newDataEvent = {
        .event = event,
        .data = data,
        .dataSize = dataSize,
    };

    EventBits_t bitsToWait = 0;
    for (auto eventConsumer : this->eventConsumers[event].second) {
      if (eventConsumer->_bitToSet == 0) {
        ESP_LOGE(TAG_EVENT_SYSTEM, "Invalid _bitToSet for event consumer");
        continue;
      }

      bitsToWait |= eventConsumer->_bitToSet;
      err = eventConsumer->_receiveEvent(
          &newDataEvent, sizeof(newDataEvent), sendWaitTicks, accessWaitTicks);

      ESP_LOGI(
          TAG_EVENT_SYSTEM,
          "Waiting: %lu, bitToSet: %lu",
          bitsToWait,
          eventConsumer->_bitToSet);

      if (err != ESP_OK)
        return err;
    }

    xEventGroupWaitBits(
        this->eventConsumers[event].first,
        bitsToWait,
        pdFALSE,
        pdTRUE,
        portMAX_DELAY); // receiveWaitTicks

    EventBits_t bitsCleared =
        xEventGroupClearBits(this->eventConsumers[event].first, bitsToWait);

    if ((bitsCleared & bitsToWait) != bitsToWait) {
      ESP_LOGW(
          TAG_EVENT_SYSTEM,
          "Waiting: %lu, Cleared: %lu, Comparison: %i",
          bitsToWait,
          bitsCleared,
          (bitsCleared & bitsToWait) != bitsToWait);
      return ESP_ERR_RECEIVE_WAIT_TIMEOUT;
    }

    return ESP_OK;
  }

  const ESWaitsConfig &getESWaitsCfg() { return this->_waitsConfig; }

protected:
  std::vector<
      std::pair<EventGroupHandle_t, std::vector<EventConsumer<EventType> *>>>
      eventConsumers;
  ESWaitsConfig _waitsConfig;
};

static int findFirstSetBit(uint32_t num) {
  for (int i = 0; i < 32; ++i) {
    if (num & (1 << i))
      return i;
  }
  return -1;
}

#endif