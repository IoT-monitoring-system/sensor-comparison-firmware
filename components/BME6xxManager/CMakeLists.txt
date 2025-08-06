file(GLOB_RECURSE BME6XXMANAGER_SRC
  "src/*.*"
)

list(APPEND INCLUDE_DIRS
  "include"
)

if(ESP_PLATFORM)

  idf_component_register(
    SRCS ${BME6XXMANAGER_SRC}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    REQUIRES arduino-esp32 bblanchon__arduinojson FSManager BME6xxSensor
  )
  
  return()
endif()

project(BME6xxManager)
