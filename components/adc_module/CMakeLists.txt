cmake_minimum_required(VERSION 3.16)

file(GLOB_RECURSE ADC_MODULE_SRC
  "src/*.*"
)

list(APPEND INCLUDE_DIRS
  "include"
)

if(ESP_PLATFORM)
  idf_component_register(
    SRCS ${ADC_MODULE_SRC}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    PRIV_REQUIRES esp_adc esp_driver_gpio
  )
endif()
