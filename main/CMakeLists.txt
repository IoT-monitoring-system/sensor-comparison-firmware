FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/main/*.*)

idf_component_register(SRCS ${app_sources} INCLUDE_DIRS "." REQUIRES RTClib MeasurementModule EventSystem BME6xxManager FSManager BME68xLibrary BME69xLibrary  BSEC2Library pubsubclient bblanchon__arduinojson)

littlefs_create_partition_image(littlefs littlefs FLASH_IN_PROJECT)
