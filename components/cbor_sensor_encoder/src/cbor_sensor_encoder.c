#include "cbor_sensor_encoder.h"

esp_err_t
encode_sensor_payload(const sensor_payload_t *payload, CborEncoder *sensors_array) {
  CborEncoder map, fields_map;

  CborError err = cbor_encoder_create_map(sensors_array, &map, CborIndefiniteLength);
  if (err != CborNoError)
    return ESP_FAIL;

  cbor_encode_text_stringz(&map, "sensor");
  cbor_encode_text_stringz(&map, payload->sensor);

  cbor_encode_text_stringz(&map, "timestamp");
  cbor_encode_uint(&map, payload->timestamp);

  cbor_encode_text_stringz(&map, "fields");
  cbor_encoder_create_map(&map, &fields_map, CborIndefiniteLength);

  for (size_t i = 0; i < payload->field_count; i++) {
    const sensor_field_t *field = &payload->fields[i];

    // CborEncoder field_map;
    // cbor_encoder_create_map(&fields_map, &field_map, 2);
    //
    cbor_encode_text_stringz(&fields_map, field->name);

    // cbor_encode_text_stringz(&field_map, "name");
    // cbor_encode_text_stringz(&field_map, "value");

    switch (field->type) {
    case SENSOR_FIELD_DATATYPE_FLOAT:
      cbor_encode_float(&fields_map, field->value.f);
      break;
    case SENSOR_FIELD_DATATYPE_INT:
      cbor_encode_int(&fields_map, field->value.i);
      break;
    case SENSOR_FIELD_DATATYPE_LONG_INT:
      cbor_encode_int(&fields_map, field->value.i);
      break;
    case SENSOR_FIELD_DATATYPE_UINT:
      cbor_encode_uint(&fields_map, field->value.u);
      break;
    case SENSOR_FIELD_DATATYPE_LONG_UINT:
      cbor_encode_uint(&fields_map, field->value.u);
      break;
    case SENSOR_FIELD_DATATYPE_BOOL:
      cbor_encode_boolean(&fields_map, field->value.b);
      break;
    default:
      cbor_encode_null(&fields_map);
      break;
    }

    // cbor_encoder_close_container(&fields_map, &field_map);
  }

  cbor_encoder_close_container(&map, &fields_map);
  cbor_encoder_close_container(sensors_array, &map);

  return ESP_OK;
}
