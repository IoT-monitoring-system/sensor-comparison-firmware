#pragma once
#ifndef STATEMANAGEMENT_H
#define STATEMANAGEMENT_H

#if ESP_PLATFORM
#include "esp_err.h"
#include "esp_log.h"
#else
typedef int esp_err_t;
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "AppDatatypes.hpp"

#define STATE_MANAGEMENT_STATE_POLL_FREQ 2
#define STATE_MANAGEMENT_STATE_TRANSITION_ERROR_POLL_FREQ 0.2
#define STATE_MANAGEMENT_TRANSITION_MUX_TIMEOUT 10000
#define STATE_MANAGEMENT_INSTANCE_MUX_TIMEOUT 10000
#define STATE_MANAGEMENT_STATE_TRANSITION_ERROR_TASK_PRIO 20
#define STATE_MANAGEMENT_TASK_PRIO 19

typedef esp_err_t (*StateFunc)(void);

struct StateHandler {
  AppState state;
  StateFunc onEnter;
  StateFunc onExit;
};

class StateManager {
private:
  static StateManager *instance_p;
  static StaticSemaphore_t instance_static_mux;
  static SemaphoreHandle_t instance_mux;

  AppState defaultState;

  StateHandler stateHandlers[STATE_COUNT];

  esp_err_t exitStateError = ESP_OK;
  esp_err_t enterStateError = ESP_OK;

  AppState nextState = STATE_IDLE;
  AppState currentState = STATE_IDLE;
  SemaphoreHandle_t stateTransitionMutex;

  TaskHandle_t stateTransitionErrorTaskHandle;
  TaskHandle_t smTaskHandle;

  void runSmTask(void *pvParameter);
  void runStateTransitionErrorTask(void *pvParameter);

  static void stateTransitionErrorTask(void *pvParameter);
  static void smTask(void *pvParameter);

protected:
  StateManager() {
    this->currentState = STATE_IDLE;
    this->nextState = STATE_IDLE;
  };
  ~StateManager() {}

public:
  StateManager(StateManager &otherSM) = delete;
  void operator=(const StateManager &) = delete;

  static StateManager *getInstance();

  void registerState(AppState state, StateFunc enter, StateFunc exit);
  void requestTransition(AppState state);

  void run();
};

#endif