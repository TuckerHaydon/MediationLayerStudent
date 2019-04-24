cmake_minimum_required(VERSION 3.5.0)

set(TARGET lib_geometry)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

set(SOURCE_FILES
  line2d.cc
  line3d.cc
  plane3d.cc
  polygon.cc
  polyhedron.cc
)

add_library(${TARGET} STATIC ${SOURCE_FILES})

target_include_directories(${TARGET} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${TARGET} PUBLIC
  lib_util
  yaml-cpp
  Eigen3::Eigen
)

set_target_properties(${TARGET} PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)

target_compile_options(${TARGET} PRIVATE 
  -Wfatal-errors
)
