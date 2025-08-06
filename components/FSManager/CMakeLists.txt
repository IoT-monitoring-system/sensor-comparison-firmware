file(GLOB_RECURSE FSMANAGER_SRC
  "src/*.*"
)

list(APPEND INCLUDE_DIRS
  "include"
)

if(ESP_PLATFORM)
  idf_component_register(
    SRCS ${FSMANAGER_SRC}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    REQUIRES arduino-esp32 bblanchon__arduinojson
  )

  return()
endif()

project(FSManager)