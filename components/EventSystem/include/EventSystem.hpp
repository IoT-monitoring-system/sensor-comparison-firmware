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

#include "EventDatatypes.h"

static int findFirstSetBit(uint32_t num);

template <typename EventType> class EventProducer;

template <typename EventType> class EventConsumer {
  friend class EventProducer<EventType>;

public:
  EventConsumer(
      size_t bufferSize,
      RingbufferType_t bufferType,
      TickType_t ticksToWaitAccess) {
    this->_ringBuffer = xRingbufferCreate(bufferSize, bufferType);

    this->_bufferSize = bufferSize;
    this->_bufferType = bufferType;

    // NOTE - May fail
    this->_accessMux = xSemaphoreCreateMutex();
    this->_ticksToWaitAccess = ticksToWaitAccess;

    if (this->_accessMux == NULL)
      ESP_LOGE((char *)"EventSystem", "Access mutex null%s", "");
  }

  /* Blocking event receive operation.
    pvBuffer and pvBuffer->data must be freed.
    Could be improved using smart pointers.*/
  esp_err_t receiveEvent(
      size_t itemSize,
      void *pvBuffer,
      TickType_t ticksToWait) {
    esp_err_t result = ESP_OK;

    if (this->_bufferType != RINGBUF_TYPE_NOSPLIT)
      return ESP_FAIL;
    if (itemSize > this->_bufferSize)
      return ESP_ERR_INVALID_SIZE;

    void *item = xRingbufferReceive(this->_ringBuffer, &itemSize, ticksToWait);

    if (item == nullptr) {
      result = ESP_FAIL;
    } else {
      EventDescriptor<EventType> *newItem =
          (EventDescriptor<EventType> *)malloc(
              sizeof(EventDescriptor<EventType>));

      if (!newItem) {
        result = ESP_ERR_NO_MEM;
      } else {
        memcpy(newItem, item, sizeof(EventDescriptor<EventType>));

        if (newItem->dataSize > 0) {
          newItem->data = malloc(newItem->dataSize);
          if (newItem->data) {
            memcpy(
                newItem->data,
                ((EventDescriptor<EventType> *)item)->data,
                newItem->dataSize);
          } else {
            free(newItem);
            result = ESP_ERR_NO_MEM;
          }
        }

        *(EventDescriptor<EventType> **)pvBuffer = newItem;
      }

      vRingbufferReturnItem(this->_ringBuffer, item);
    }

    xEventGroupSetBits(this->_eventGroupHandle, this->_bitToSet);

    return result;
  }

  esp_err_t flushBuffer() {
    xSemaphoreTake(this->_accessMux, this->_ticksToWaitAccess);
    vRingbufferDelete(this->_ringBuffer);
    this->_ringBuffer = xRingbufferCreate(this->_bufferSize, this->_bufferType);
    xSemaphoreGive(this->_accessMux);

    return ESP_OK;
  }

  size_t getBufferFreeSpace() {
    xSemaphoreTake(this->_accessMux, this->_ticksToWaitAccess);
    size_t size = xRingbufferGetCurFreeSize(this->_ringBuffer);
    xSemaphoreGive(this->_accessMux);

    return size;
  }

protected:
  size_t _bufferSize;
  RingbufferType_t _bufferType;

  RingbufHandle_t _ringBuffer;

  TickType_t _ticksToWaitAccess;
  SemaphoreHandle_t _accessMux;

  /* These will be modified by the MeasurementTask whenever we use pass it to
   * subscribe to an event*/
  EventBits_t _bitToSet;
  EventGroupHandle_t _eventGroupHandle;

  /* Defined this way because it shouldn't make an impression to be a part of
   * the public API*/
  esp_err_t _acceptEvent(
      const void *pvItem,
      size_t xItemSize,
      TickType_t ticksToWaitSend,
      TickType_t ticksToWaitAccess) {

    xSemaphoreTake(this->_accessMux, ticksToWaitAccess);
    BaseType_t res =
        xRingbufferSend(this->_ringBuffer, pvItem, xItemSize, ticksToWaitSend);
    xSemaphoreGive(this->_accessMux);

    if (res == pdFAIL)
      return ESP_FAIL;

    return ESP_OK;
  }
};

template <typename EventType> class EventProducer {
public:
  EventProducer(uint8_t numEvents) {
    this->eventConsumers.resize(numEvents);

    for (uint8_t i = 0; i < numEvents; i++) {
      this->eventConsumers[i].first = xEventGroupCreate();
    }
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
        [](uint32_t acc, const auto &eH) { return acc ^ eH->_bitToSet; });

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
      TickType_t sendWaitTicks,
      TickType_t accessWaitTicks,
      TickType_t receiveWaitTicks) {

    EventDescriptor<EventType> newDataEvent = {
        .event = event,
        .data = data,
        .dataSize = dataSize,
    };

    EventBits_t bitsToWait = 0;
    for (auto eventConsumer : this->eventConsumers[event].second) {
      bitsToWait |= eventConsumer->_bitToSet;
      eventConsumer->_acceptEvent(
          &newDataEvent, sizeof(newDataEvent), sendWaitTicks, accessWaitTicks);
    }

    xEventGroupWaitBits(
        this->eventConsumers[event].first,
        bitsToWait,
        pdFALSE,
        pdTRUE,
        receiveWaitTicks);

    EventBits_t bitsCleared =
        xEventGroupClearBits(this->eventConsumers[event].first, bitsToWait);

    if ((bitsCleared && bitsToWait) != bitsToWait) {
      // TODO
    }
    /* TODO Error handling*/
    return ESP_OK;
  }

protected:
  std::vector<
      std::pair<EventGroupHandle_t, std::vector<EventConsumer<EventType> *>>>
      eventConsumers;
};

static int findFirstSetBit(uint32_t num) {
  for (int i = 0; i < 32; ++i) {
    if (num & (1 << i))
      return i;
  }
  return -1;
}

#endif