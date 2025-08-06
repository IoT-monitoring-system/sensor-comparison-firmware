#pragma once
#ifndef SENSOR_DATATYPES_H
#define SENSOR_DATATYPES_H

#include "stdbool.h"
#include "stddef.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENSOR_NAME_MAX_LEN   16
#define SENSOR_FIELD_NAME_LEN 16
#define SENSOR_MAX_FIELDS     5

typedef enum {
  SENSOR_FIELD_DATATYPE_INVALID = 0,
  SENSOR_FIELD_DATATYPE_FLOAT,
  SENSOR_FIELD_DATATYPE_INT,
  SENSOR_FIELD_DATATYPE_LONG_INT,
  SENSOR_FIELD_DATATYPE_UINT,
  SENSOR_FIELD_DATATYPE_LONG_UINT,
  SENSOR_FIELD_DATATYPE_BOOL,
} sensor_field_datatype_t;

typedef struct {
  char name[SENSOR_FIELD_NAME_LEN];
  sensor_field_datatype_t type;

  union {
    float f;
    int64_t i;
    uint64_t u;
    bool b;
  } value;
} sensor_field_t;

typedef struct {
  char sensor[SENSOR_NAME_MAX_LEN];
  uint64_t timestamp;

  sensor_field_t fields[SENSOR_MAX_FIELDS];
  size_t field_count;
} sensor_payload_t;

#ifdef __cplusplus
}
#endif
#endif
