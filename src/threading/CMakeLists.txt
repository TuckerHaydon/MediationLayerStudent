cmake_minimum_required(VERSION 2.8.3)

set(TARGET lib_threading)


add_library(${TARGET} INTERFACE)
target_include_directories(${TARGET} INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(${TARGET} INTERFACE pthread)
