cmake_minimum_required(VERSION 3.16.0)

# USB Host components (espressif__usb, cdc_ecm_host) conflict with the built-in
# 'usb' component in IDF 4.4.7 (used for esp32c3). They live in components_usb_host/
# and are only added for targets with USB OTG hardware.
if(NOT "${IDF_TARGET}" STREQUAL "esp32c3")
    list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/components_usb_host")
endif()

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
