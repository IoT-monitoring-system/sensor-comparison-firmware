cmake_minimum_required(VERSION 3.13)

if(ESP_PLATFORM)
  file(GLOB_RECURSE BME6xxManager
    "src/*.*"
  )

  idf_component_register(
    SRCS ${BME6xxManager}
    INCLUDE_DIRS "include"
    REQUIRES arduino BME6xxSensor bblanchon__arduinojson FSManager
  )
  
  return()
endif()

