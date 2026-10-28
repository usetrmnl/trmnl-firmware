FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

set(app_requirements driver soc ulp)
idf_build_get_property(app_ble_pairing TRMNL_ENABLE_BLE_PAIRING)
if(app_ble_pairing)
    list(APPEND app_requirements protocomm bt qrcode)
endif()
idf_component_register(SRCS ${app_sources} REQUIRES ${app_requirements})

# The pinned Espressif component bundles Nayuki's segment encoder but exports
# only its higher-level include directory. Use the bundled implementation.
if(app_ble_pairing)
    idf_component_get_property(qrcode_component_dir espressif__qrcode COMPONENT_DIR)
    target_include_directories(${COMPONENT_LIB} PRIVATE "${qrcode_component_dir}")
endif()
