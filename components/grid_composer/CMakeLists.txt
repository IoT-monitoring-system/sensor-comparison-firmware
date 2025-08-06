file(GLOB_RECURSE GRID_COMPOSER_SRC
  "src/*.*"
)

list(APPEND INCLUDE_DIRS
  "include"
  "interface"
)

if(ESP_PLATFORM)
  list(APPEND INCLUDE_DIRS
    "platform/esp-arduino"
  )
  list(APPEND GRID_COMPOSER_SRC
    "platform/esp-arduino/adafruit_renderer.cpp"
  )

  idf_component_register(
    SRCS ${GRID_COMPOSER_SRC}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    PRIV_REQUIRES arduino-esp32 Adafruit-GFX-Library Adafruit_SH110x U8g2_for_Adafruit_GFX
  )
endif()
