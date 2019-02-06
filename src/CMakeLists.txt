cmake_minimum_required(VERSION 3.1)

set(TARGET path_planning)

find_package (Eigen3 3.3 NO_MODULE)
find_package(Boost 1.40.0 COMPONENTS filesystem system iostreams REQUIRED)

set(SOURCES 
  graph/graph.cc
  algorithm/a_star.cc
  algorithm/dijkstra.cc
  algorithm/safe_flight_corridor2d.cc
  environment/occupancy_grid2d.cc
  gui/gnu_visualizer.cc
  gui/gui2d.cc
  )

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  algorithm
  environment
  graph
  gui
  constraint
)

add_library(${TARGET} ${SOURCES})

# Why do I have to link rt,eigen?
target_link_libraries(${TARGET} 
  cpp_plot 
  geometry 
  rt 
  Eigen3::Eigen
  ${Boost_LIBRARIES})
