cmake_minimum_required(VERSION 3.5.0)

set(TARGET lib_util)

set(SOURCE_FILES
  timer.cc
)

add_library(${TARGET} STATIC ${SOURCE_FILES})

target_include_directories(${TARGET} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

set_target_properties(${TARGET} PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)

target_compile_options(${TARGET} PRIVATE 
  -Wfatal-errors
)
