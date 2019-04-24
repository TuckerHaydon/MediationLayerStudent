cmake_minimum_required(VERSION 3.5.0)

set(TARGET lib_graph)

add_library(${TARGET} INTERFACE)

target_include_directories(${TARGET} INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${TARGET} INTERFACE Eigen3::Eigen)
