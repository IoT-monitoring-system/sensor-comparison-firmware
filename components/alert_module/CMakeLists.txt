file(GLOB_RECURSE ALERT_MODULE_SRC
  "src/*.*"
)

list(APPEND INCLUDE_DIRS
  "include"
)

if(ESP_PLATFORM)
  idf_component_register(
    SRCS ${ALERT_MODULE_SRC}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    PRIV_REQUIRES
  )
endif()
