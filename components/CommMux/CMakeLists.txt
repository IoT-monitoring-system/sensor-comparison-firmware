cmake_minimum_required(VERSION 3.13)

if(ESP_PLATFORM)
  file(GLOB_RECURSE CommMux
    "src/*.*"
  )

  idf_component_register(
    SRCS ${CommMux}
    INCLUDE_DIRS "include"
    REQUIRES arduino
  )
  
  return()
endif()

