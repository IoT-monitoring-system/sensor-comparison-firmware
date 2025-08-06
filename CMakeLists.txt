cmake_minimum_required(VERSION 3.15)

set(ENV{ARDUINO_SKIP_TICK_CHECK} 1)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(node-firmware)

target_compile_definitions(${CMAKE_PROJECT_NAME}.elf PRIVATE
  ARDUINO_USB_MODE=0
  ARDUINO_USB_CDC_ON_BOOT=0
)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "main/certs/client.crt" TEXT)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "main/certs/client.key" TEXT)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "main/certs/rootCA.pem" TEXT)
