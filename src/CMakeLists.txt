cmake_minimum_required(VERSION 3.1)

set(TARGET lib_mediation_layer)

find_package (Eigen3 3.3 NO_MODULE)
find_package(Boost 1.40.0 COMPONENTS filesystem system iostreams REQUIRED)

# Build libraries
add_subdirectory(lib)
add_subdirectory(util)
add_subdirectory(integration)
add_subdirectory(geometry)
add_subdirectory(graph)
add_subdirectory(environment)
add_subdirectory(ros)
add_subdirectory(P4)

set(SOURCES 
  algorithm/safe_flight_corridor2d.cc
  gui/gui2d.cc
  mediation_layer/mediation_layer2d.cc
  )

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  algorithm
  gui
  mediation_layer
  mediation_layer/potential
  ros/node
  ros/view
)
add_library(${TARGET} ${SOURCES})

target_link_libraries(${TARGET} 
  yaml-cpp
  lib_util
  lib_integration
  lib_geometry
  lib_graph
  lib_environment
  lib_ros_view
  lib_ros_node
  Eigen3::Eigen
  ${Boost_LIBRARIES})

# Build the final executables
add_subdirectory(exe)
