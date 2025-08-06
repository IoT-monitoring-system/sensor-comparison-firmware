#pragma once
#ifndef CBOR_SENSOR_ENCODER_H
#define CBOR_SENSOR_ENCODER_H

#include "esp_err.h"

#include "cbor.h"

#include "cbor_sensor_encoder_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Attaches a map with sensor data to sensors_array.
 * @warning Not thread safe!
 */
esp_err_t
encode_sensor_payload(const sensor_payload_t *payload, CborEncoder *sensors_array);

#ifdef __cplusplus
}
#endif
#endif