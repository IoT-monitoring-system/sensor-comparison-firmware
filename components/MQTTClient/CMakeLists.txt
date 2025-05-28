cmake_minimum_required(VERSION 3.13)

if(ESP_PLATFORM)
  file(GLOB_RECURSE MQTTClient
    "src/*.*"
  )

  idf_component_register(
    SRCS ${MQTTClient}
    INCLUDE_DIRS "include"
    REQUIRES esp_event mqtt
  )
  
  return()
endif()

