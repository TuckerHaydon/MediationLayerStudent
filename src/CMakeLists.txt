cmake_minimum_required(VERSION 3.1)

set(TARGET path_planning)

find_package (Eigen3 3.3 NO_MODULE)
find_package(Boost 1.40.0 COMPONENTS filesystem system iostreams REQUIRED)

add_subdirectory(ros)
add_subdirectory(geometry)
add_subdirectory(integration)
add_subdirectory(graph)

set(SOURCES 
  # graph/graph.cc
  algorithm/a_star.cc
  algorithm/dijkstra.cc
  algorithm/safe_flight_corridor2d.cc
  environment/occupancy_grid2d.cc
  gui/gui2d.cc
  mediation_layer/mediation_layer2d.cc
  )

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  algorithm
  environment
  # graph
  gui
  constraint
  mediation_layer
  mediation_layer/potential
  util
  ros
)
add_library(${TARGET} ${SOURCES})

target_link_libraries(${TARGET} 
  ros2d_lib
  lib_geometry
  lib_integration
  lib_graph
  Eigen3::Eigen
  ${Boost_LIBRARIES})
