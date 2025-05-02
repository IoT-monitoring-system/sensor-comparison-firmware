cmake_minimum_required(VERSION 3.13)

if(ESP_PLATFORM)
  file(GLOB_RECURSE BME6xxSensor
    "*.cpp"
  )

  idf_component_register(
    SRCS ${BME6xxSensor}
    INCLUDE_DIRS "include"
    REQUIRES arduino BME68xLibrary BME69xLibrary
  )

  return()
endif()

