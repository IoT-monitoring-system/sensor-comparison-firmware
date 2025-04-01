#pragma once
#ifndef MEASUREMENTDATATYPES_H
#define MEASUREMENTDATATYPES_H

#include <bitset>
#include <vector>

// Types of sensor measurements.
// -----------------------------------
enum MeasurementType {
  UNKNOWN = 0,

  Voltage,
  Current,
  Power,
  Temperature,
  Position,
  Elevation,
  Speed,
  Humidity,
  VOC,
  VSC,
  Time,
  Acceleration,
  Rotation,
  // CtrlData,

  NUM_MEASUREMENTS,

  DEF = 255
};
//-----------------------------------

// Structures for each of the sensor reading types.
// -----------------------------------
struct DefaultData {
  uint8_t def = 0xFF;
};
struct VoltageData {
  float voltage = 0.0f;
};
struct CurrentData {
  float current = 0.0f;
};
struct PowerData {
  float power = 0.0f;
};
struct TemperatureData {
  float temperature = 0.0f;
};
struct PositionData {
  float lat = 0.0f;
  float lon = 0.0f;
};
struct ElevationData {
  float elevation = 0.0f;
};
struct SpeedData {
  float speed = 0.0f;
};
struct HumidityData {
  float humidity = 0.0f;
};
struct VOCData {
  float VOC = 0.0f;
};
struct VSCData {
  float VSC = 0.0f;
};
struct TimeData {
  uint64_t time = 0U;
};
struct AccelerationData {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};
struct RotationData {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};
// -----------------------------------

// A composite datatype that tries to help the user derive the datatypes that
// are included.
// -----------------------------------
struct MeasurementBase {
#if CONFIG_MEASUREMENT_TASK_DYNAMIC_CAST
  virtual ~MeasurementBase() = default;
#endif
  std::bitset<NUM_MEASUREMENTS> included_types;
  size_t size;

  std::vector<MeasurementType> get_types() {
    std::vector<MeasurementType> types{0};
    for (uint8_t k = 0; k < NUM_MEASUREMENTS; k++) {
      if (this->included_types.test(k))
        types.push_back((MeasurementType)k);
    }
    return types;
  }
};

template <typename... Components>
struct MeasurementTaskData : public MeasurementBase, public Components... {
  MeasurementTaskData() {
    this->included_types.reset();
    add_types<Components...>();
    this->size = sizeof(*this);
  }

private:
  /**
   * @brief
   * @warning Recursive
   *
   */
  template <typename U, typename... Rest> void add_types() {
    set_flag<U>();
    if constexpr (sizeof...(Rest) > 0) {
      add_types<Rest...>();
    }
  }

  /**
   * @brief Set the flag for the datatypes that are included.
   *
   */
  template <typename U> void set_flag() {
    if constexpr (std::is_same_v<U, VoltageData>) {
      included_types.set(Voltage);
    } else if constexpr (std::is_same_v<U, CurrentData>) {
      included_types.set(Current);
    } else if constexpr (std::is_same_v<U, PowerData>) {
      included_types.set(Power);
    } else if constexpr (std::is_same_v<U, TemperatureData>) {
      included_types.set(Temperature);
    } else if constexpr (std::is_same_v<U, PositionData>) {
      included_types.set(Position);
    } else if constexpr (std::is_same_v<U, ElevationData>) {
      included_types.set(Elevation);
    } else if constexpr (std::is_same_v<U, SpeedData>) {
      included_types.set(Speed);
    } else if constexpr (std::is_same_v<U, HumidityData>) {
      included_types.set(Humidity);
    } else if constexpr (std::is_same_v<U, VOCData>) {
      included_types.set(VOC);
    } else if constexpr (std::is_same_v<U, VSCData>) {
      included_types.set(VSC);
    } else if constexpr (std::is_same_v<U, TimeData>) {
      included_types.set(Time);
    } else if constexpr (std::is_same_v<U, AccelerationData>) {
      included_types.set(Acceleration);
    } else if constexpr (std::is_same_v<U, RotationData>) {
      included_types.set(Rotation);
    } else if constexpr (std::is_same_v<U, DefaultData>) {
      included_types.set(DEF);
    } else {
      included_types.set(UNKNOWN);
    }
  }
};
// -----------------------------------

#endif