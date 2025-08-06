file(GLOB_RECURSE MQTT_CLIENT_SRC
  "src/*.*"
)

list(APPEND INCLUDE_DIRS
  "include"
)


if(ESP_PLATFORM)
  idf_component_register(
    SRCS ${MQTT_CLIENT_SRC}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    REQUIRES esp_event mqtt
  )

  return()
endif()

project(MQTTClient)
