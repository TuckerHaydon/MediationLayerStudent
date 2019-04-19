cmake_minimum_required(VERSION 2.8.3)

set(TARGET lib_mediation_layer)

find_package (Eigen3 3.3 NO_MODULE)

add_library(${TARGET} INTERFACE)
target_include_directories(${TARGET} INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/potential
  ../util
)
target_link_libraries(${TARGET} INTERFACE 
  lib_integration
  Eigen3::Eigen)
