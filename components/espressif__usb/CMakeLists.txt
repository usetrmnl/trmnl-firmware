idf_build_get_property(target IDF_TARGET)

if(${target} STREQUAL "linux")
    return() # This component is not supported by the POSIX/Linux simulator
endif()

set(srcs)
set(include)
set(priv_includes)
# As CONFIG_SOC_USB_OTG_SUPPORTED comes from Kconfig, it is not evaluated yet
# when components are being registered.
# Thus, always add the (private) requirements, regardless of Kconfig
set(priv_requires esp_mm)

# Explicitly add psram component for esp32p4, as the USB-DWC internal DMA can access PSRAM on esp32p4
if(${target} STREQUAL "esp32p4")
    list(APPEND priv_requires esp_psram)
endif()

if(${IDF_VERSION_MAJOR} LESS 6)
    # usb_phy driver is present in USB component only in IDF 5.x, it relies on gpio driver API
    list(APPEND priv_requires esp_driver_gpio)
else()
    # For IDF 6.x and later, usb_phy driver is moved to esp_hw_support and HAL driver is moved to esp_hal_usb
    list(APPEND priv_requires esp_hal_usb esp_hw_support)
endif()

if(CONFIG_SOC_USB_OTG_SUPPORTED)
    list(APPEND srcs "src/hcd_dwc.c"
                     "src/enum.c"
                     "src/hub.c"
                     "src/usb_helpers.c"
                     "src/usb_host.c"
                     "src/usb_private.c"
                     "src/usbh.c"
                     )
    list(APPEND include "include")
    list(APPEND priv_includes "private_include")

    if(${IDF_VERSION_MAJOR} LESS 6)
        # usb_phy driver is present in USB component only in IDF 5.x
        # If we want to override usb component from esp-idf with managed usb component from esp-usb, we must provide
        # usb_phy driver for the managed usb component separately by taking src usb_phy.c and
        # include esp_private/usb_phy.h from the native usb component in esp-idf

        # Get path to usb component inside esp-idf
        idf_component_get_property(esp_idf_usb_dir usb COMPONENT_OVERRIDEN_DIR)
        if(NOT esp_idf_usb_dir)
            idf_component_get_property(esp_idf_usb_dir usb COMPONENT_DIR)
        endif()

        # Explicitly add usb_phy.c and usb_phy.h from overridden usb component in esp-idf
        list(APPEND srcs "${esp_idf_usb_dir}/usb_phy.c")

        # This append must be under the first "include" append,
        # so usb_host.h (and the rest of the files in /include/usb/) are used from esp-usb, not from esp-idf
        list(APPEND include "${esp_idf_usb_dir}/include")
    endif()
endif()

if(CONFIG_USB_HOST_HUBS_SUPPORTED)
    list(APPEND srcs "src/ext_hub.c"
                     "src/ext_port.c")
endif()

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS ${include}
                       PRIV_INCLUDE_DIRS ${priv_includes}
                       PRIV_REQUIRES ${priv_requires}
                       )

if(CONFIG_COMPILER_STATIC_ANALYZER AND CMAKE_C_COMPILER_ID STREQUAL "GNU") # TODO GCC-366 (segfault)
    set_property(SOURCE usb_host.c PROPERTY COMPILE_FLAGS -fno-analyzer)
endif()

# Create public symbol representing remote wakeup changes availability based on IDF version
# Remote wakeup HAL changes are present:
# On IDF 5.4.x from IDF 5.4.4
# On IDF 5.5.x from IDF 5.5.3
# On IDF 6.0.x from IDF 6.0.0
set(REMOTE_WAKE_HAL_SUPPORTED OFF)
set(IDF_VERSION "${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}.${IDF_VERSION_PATCH}")

# IDF 5.4.x from 5.4.4
if(IDF_VERSION VERSION_GREATER_EQUAL "5.4.4" AND IDF_VERSION VERSION_LESS "5.5.0")
    set(REMOTE_WAKE_HAL_SUPPORTED ON)

# IDF 5.5.x from 5.5.3 and all 6.x+
elseif(IDF_VERSION VERSION_GREATER_EQUAL "5.5.3")
    set(REMOTE_WAKE_HAL_SUPPORTED ON)
endif()

if(REMOTE_WAKE_HAL_SUPPORTED)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC REMOTE_WAKE_HAL_SUPPORTED)
endif()
