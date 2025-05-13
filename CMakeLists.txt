# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(ENV{ARDUINO_SKIP_TICK_CHECK} 1)
add_definitions(
-DARDUINO_USB_MODE=0
-DARDUINO_USB_CDC_ON_BOOT=0
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(node-firmware)