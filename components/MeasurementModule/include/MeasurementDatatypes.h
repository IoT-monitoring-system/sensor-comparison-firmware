#pragma once
#ifndef MEASUREMENTDATATYPES_H
#define MEASUREMENTDATATYPES_H

#include <bitset>
#include <vector>

// Types of sensor measurements.
// -----------------------------------
enum measurement_type {
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
struct default_data {
  uint8_t def = 0xFF;
};
struct voltage_data {
  float voltage = 0.0f;
};
struct current_data {
  float current = 0.0f;
};
struct power_data {
  float power = 0.0f;
};
struct temperature_data {
  float temperature = 0.0f;
};
struct position_data {
  float lat = 0.0f;
  float lon = 0.0f;
};
struct elevation_data {
  float elevation = 0.0f;
};
struct speed_data {
  float speed = 0.0f;
};
struct humidity_data {
  float humidity = 0.0f;
};
struct VOC_data {
  float VOC = 0.0f;
};
struct VSC_data {
  float VSC = 0.0f;
};
struct time_data {
  uint64_t time = 0U;
};
struct acceleration_data {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};
struct rotation_data {
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

  std::vector<measurement_type> get_types() {
    std::vector<measurement_type> types{0};
    for (uint8_t k = 0; k < NUM_MEASUREMENTS; k++) {
      if (this->included_types.test(k))
        types.push_back((measurement_type)k);
    }
    return types;
  }
};

template <typename... Components>
struct measurement_task_data_t : public MeasurementBase, public Components... {
  measurement_task_data_t() {
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
    if constexpr (std::is_same_v<U, voltage_data>) {
      included_types.set(Voltage);
    } else if constexpr (std::is_same_v<U, current_data>) {
      included_types.set(Current);
    } else if constexpr (std::is_same_v<U, power_data>) {
      included_types.set(Power);
    } else if constexpr (std::is_same_v<U, temperature_data>) {
      included_types.set(Temperature);
    } else if constexpr (std::is_same_v<U, position_data>) {
      included_types.set(Position);
    } else if constexpr (std::is_same_v<U, elevation_data>) {
      included_types.set(Elevation);
    } else if constexpr (std::is_same_v<U, speed_data>) {
      included_types.set(Speed);
    } else if constexpr (std::is_same_v<U, humidity_data>) {
      included_types.set(Humidity);
    } else if constexpr (std::is_same_v<U, VOC_data>) {
      included_types.set(VOC);
    } else if constexpr (std::is_same_v<U, VSC_data>) {
      included_types.set(VSC);
    } else if constexpr (std::is_same_v<U, time_data>) {
      included_types.set(Time);
    } else if constexpr (std::is_same_v<U, acceleration_data>) {
      included_types.set(Acceleration);
    } else if constexpr (std::is_same_v<U, rotation_data>) {
      included_types.set(Rotation);
    } else if constexpr (std::is_same_v<U, default_data>) {
      included_types.set(DEF);
    } else {
      included_types.set(UNKNOWN);
    }
  }
};
// -----------------------------------

#endif