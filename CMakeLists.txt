cmake_minimum_required(VERSION 3.16.0)
# sdkconfig.defaults lives under sdkconfigs/ (per-target configs are redirected there
# via board_build.esp-idf.sdkconfig_path in platformio.ini). Must be set before project().
# Only set the default when an env hasn't supplied its own SDKCONFIG_DEFAULTS chain
# (via board_build.cmake_extra_args), so board-specific overrides can layer on top.
if(NOT DEFINED SDKCONFIG_DEFAULTS)
    set(SDKCONFIG_DEFAULTS "${CMAKE_SOURCE_DIR}/sdkconfigs/sdkconfig.defaults")
endif()
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(DEPENDENCIES_LOCK "dependencies.lock.${IDF_TARGET}")
project(trmnl-firmware)
