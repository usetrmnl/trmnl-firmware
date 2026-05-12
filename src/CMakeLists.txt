FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(SRCS ${app_sources})
target_compile_options(${COMPONENT_LIB} PRIVATE -Wformat)
