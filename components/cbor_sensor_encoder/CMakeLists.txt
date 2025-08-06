file(GLOB_RECURSE CBOR_SENSOR_ENCODER_SRC
  "src/*.*"
)

list(APPEND INCLUDE_DIRS
  "include"
)

if(ESP_PLATFORM)
  idf_component_register(SRCS ${CBOR_SENSOR_ENCODER_SRC}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    REQUIRES cbor)

endif()

project(cbor_sensor_encoder)
