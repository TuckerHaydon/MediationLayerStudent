cmake_minimum_required(VERSION 3.5.0)

set(TARGET lib_environment)

set(SOURCE_FILES
  map2d.cc
  map3d.cc
  occupancy_grid2d.cc
)

add_library(${TARGET} STATIC ${SOURCE_FILES})

target_include_directories(${TARGET} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${TARGET} PUBLIC
  lib_geometry 
  lib_graph)

set_target_properties(${TARGET} PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)

target_compile_options(${TARGET} PRIVATE 
  -Wfatal-errors
)
