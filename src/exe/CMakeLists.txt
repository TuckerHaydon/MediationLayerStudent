cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

add_executable(mediation_layer mediation_layer.cc)
target_include_directories(mediation_layer PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(mediation_layer
  lib_geometry
  lib_environment
  lib_util
  lib_mediation_layer
  lib_ros_node
  lib_ros_view
  yaml-cpp
  ${catkin_LIBRARIES})

set_target_properties(mediation_layer
  PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin/exe"
)

add_executable(ap_blue ap_blue.cc)
target_include_directories(ap_blue PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(ap_blue
  lib_geometry
  lib_environment
  lib_util
  lib_mediation_layer
  lib_ros_node
  lib_ros_view
  lib_automation_protocol
  yaml-cpp
  ${catkin_LIBRARIES})

set_target_properties(ap_blue
  PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin/exe"
)
