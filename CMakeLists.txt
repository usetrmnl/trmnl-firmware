cmake_minimum_required(VERSION 3.16.0)
# sdkconfig.defaults lives under sdkconfigs/ (per-target configs are redirected there
# via board_build.esp-idf.sdkconfig_path in platformio.ini). Must be set before project().
set(SDKCONFIG_DEFAULTS "${CMAKE_SOURCE_DIR}/sdkconfigs/sdkconfig.defaults")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(trmnl-firmware)
