cmake_minimum_required(VERSION 3.13)

if(ESP_PLATFORM)
  file(GLOB_RECURSE FSManager
    "*.*"
  )

  idf_component_register(
    SRCS ${FSManager}
    INCLUDE_DIRS "include"
    REQUIRES arduino bblanchon__arduinojson
  )

  return()
endif()

