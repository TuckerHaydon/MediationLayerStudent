cmake_minimum_required(VERSION 2.8.3)

set(TARGET geometry)

find_package (Eigen3 3.3 NO_MODULE)

set(SOURCES
  line2d.cc
  polygon.cc
)

add_library(${TARGET} SHARED ${SOURCES})
target_include_directories(${TARGET} PUBLIC ../util .)
target_link_libraries(${TARGET} Eigen3::Eigen)

