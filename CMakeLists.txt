cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(trmnl-firmware)

# libwebp is pulled in via FetchContent from src/CMakeLists.txt. Its static archives
# are empty under this ESP-IDF/CMake build, so link the decoder object targets directly.
set(_trmnl_project_elf "${CMAKE_PROJECT_NAME}.elf")
if(TARGET "${_trmnl_project_elf}" AND TARGET webpdecode AND TARGET webpdspdecode AND TARGET webputilsdecode)
    target_link_libraries("${_trmnl_project_elf}" PRIVATE webpdecode webpdspdecode webputilsdecode)
endif()
