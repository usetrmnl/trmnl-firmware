cmake_minimum_required(VERSION 3.16.0)
# sdkconfig.defaults lives under sdkconfigs/ (per-target configs are redirected there
# via board_build.esp-idf.sdkconfig_path in platformio.ini). Must be set before project().
# Only set the default when an env hasn't supplied its own SDKCONFIG_DEFAULTS chain
# (via board_build.cmake_extra_args), so board-specific overrides can layer on top.
if(NOT DEFINED SDKCONFIG_DEFAULTS)
    set(SDKCONFIG_DEFAULTS "${CMAKE_SOURCE_DIR}/sdkconfigs/sdkconfig.defaults")
endif()
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(trmnl-firmware)

# libwebp is pulled in via FetchContent from src/CMakeLists.txt. Its static archives
# are empty under this ESP-IDF/CMake build, so link the decoder object targets directly.
set(_trmnl_project_elf "${CMAKE_PROJECT_NAME}.elf")
if(TARGET "${_trmnl_project_elf}" AND TARGET webpdecode AND TARGET webpdspdecode AND TARGET webputilsdecode)
    target_link_libraries("${_trmnl_project_elf}" PRIVATE webpdecode webpdspdecode webputilsdecode)
endif()
