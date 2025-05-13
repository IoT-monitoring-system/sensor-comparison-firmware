#pragma once
#ifndef ALERTMODULE_H
#define ALERTMODULE_H

#include <stdio.h>
#include <string>

// Take in measurements (battery charge, acceleration values, gas resistance
// values, temperature, etc)
// All of these are floats, integers, etc (numbers)
// The system should be responsible for different types of alerts
// Each alert should be described by thresholds and hold times (derivatives?)
// VERY VERY similar to just an ordinary event system
// The existing EventSystem is mainly for task - taks event sharing
// This one is more like an abstraction for alert detection algorithms

enum class ALERT_TYPE : uint8_t {

};

std::string alertTypeToString(ALERT_TYPE alert);

class Alert {
public:
  Alert();
  // Should store a bunch of thresholds
  // Add an addThreshold function
  // Add check function
  // Add Alert handlers

private:
  ALERT_TYPE alertType;
};

class DerivativeAlert : public Alert {};

class AbsoluteAlert : public Alert {};

class DurationAlert : public Alert {};

#endif