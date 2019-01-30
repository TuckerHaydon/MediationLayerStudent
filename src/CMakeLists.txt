cmake_minimum_required(VERSION 3.1)

set(TARGET path_planning)

set(SOURCES 
  node.cc
  graph.cc
  occupancy_grid.cc
  dijkstra.cc
  a_star.cc
  gnu_visualizer.cc
  )


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../CppPlot/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_library(${TARGET} ${SOURCES})
target_link_libraries(${TARGET} rt)
