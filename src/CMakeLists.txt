# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

# ESP-IDF runs this file again with CMAKE_BUILD_EARLY_EXPANSION=1 to collect
# component requirements; FetchContent is not allowed in that phase (CMake 4.x).
if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    include(FetchContent)
    set(WEBP_BUILD_ANIM_UTILS OFF CACHE BOOL "" FORCE)
    set(WEBP_BUILD_CWEBP OFF CACHE BOOL "" FORCE)
    set(WEBP_BUILD_DWEBP OFF CACHE BOOL "" FORCE)
    set(WEBP_BUILD_GIF2WEBP OFF CACHE BOOL "" FORCE)
    set(WEBP_BUILD_IMG2WEBP OFF CACHE BOOL "" FORCE)
    set(WEBP_BUILD_VWEBP OFF CACHE BOOL "" FORCE)
    set(WEBP_BUILD_WEBPINFO OFF CACHE BOOL "" FORCE)
    set(WEBP_BUILD_LIBWEBPMUX OFF CACHE BOOL "" FORCE)
    set(WEBP_BUILD_WEBPMUX OFF CACHE BOOL "" FORCE)
    set(WEBP_BUILD_EXTRAS OFF CACHE BOOL "" FORCE)
    set(WEBP_BUILD_WEBP_JS OFF CACHE BOOL "" FORCE)
    set(WEBP_USE_THREAD OFF CACHE BOOL "" FORCE)
    set(WEBP_ENABLE_SIMD OFF CACHE BOOL "" FORCE)
    set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

    FetchContent_Declare(
        trmnl_libwebp
        GIT_REPOSITORY https://github.com/webmproject/libwebp.git
        GIT_TAG        v1.4.0
        GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(trmnl_libwebp)
endif()

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(SRCS ${app_sources})

if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    # Linking webp to ${COMPONENT_LIB} does not reach firmware.elf: ESP-IDF links the ELF
    # only to component targets, not transitive CMake deps. See root CMakeLists.txt.
    target_include_directories(${COMPONENT_LIB} PRIVATE
        "${trmnl_libwebp_SOURCE_DIR}/src"
        "${trmnl_libwebp_BINARY_DIR}/src"
    )
endif()
