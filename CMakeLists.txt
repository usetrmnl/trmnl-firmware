cmake_minimum_required(VERSION 3.16.0)

# USB Host components (espressif__usb, cdc_ecm_host) conflict with the built-in
# 'usb' component in IDF 4.4.7 (used for esp32c3). They live in components_usb_host/
# and are only added for targets with USB OTG hardware.
if(NOT "${IDF_TARGET}" STREQUAL "esp32c3")
    list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/components_usb_host")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(trmnl-firmware)
