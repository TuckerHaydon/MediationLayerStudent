cmake_minimum_required(VERSION 2.8.3)

set(TARGET lib_geometry)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_library(${TARGET} INTERFACE)
target_include_directories(${TARGET} INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ../util
)
target_link_libraries(${TARGET} INTERFACE yaml-cpp Eigen3::Eigen)
