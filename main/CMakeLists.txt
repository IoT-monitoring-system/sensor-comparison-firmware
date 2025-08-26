file(GLOB_RECURSE APP_SRCS
  "*.*"
)

list(APPEND INCLUDE_DIRS
  "."
)

idf_component_register(
  SRCS ${APP_SRCS}
  INCLUDE_DIRS ${INCLUDE_DIRS}
  REQUIRES
)

littlefs_create_partition_image(littlefs littlefs FLASH_IN_PROJECT)