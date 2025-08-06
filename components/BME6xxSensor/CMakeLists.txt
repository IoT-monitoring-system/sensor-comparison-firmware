file(GLOB_RECURSE BME6XXSENSOR_SRC
  "include"
)

list(APPEND INCLUDE_DIRS
  "include"
)

if(ESP_PLATFORM)

  idf_component_register(
    SRCS ${BME6XXSENSOR_SRC}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    REQUIRES
  )
  
  return()
endif()

project(BME6xxSensor)
