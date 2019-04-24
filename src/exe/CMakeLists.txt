cmake_minimum_required(VERSION 3.5.0)

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
  ${catkin_LIBRARIES}
)

set_target_properties(mediation_layer
  PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
  RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
)

add_executable(physics_simulator physics_simulator.cc)
target_include_directories(physics_simulator PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(physics_simulator
  lib_geometry
  lib_environment
  lib_util
  lib_mediation_layer
  lib_ros_node
  lib_ros_view
  lib_autonomy_protocol
  yaml-cpp
  ${catkin_LIBRARIES})
set_target_properties(physics_simulator
  PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
  RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
)

add_executable(example_autonomy_protocol example_autonomy_protocol.cc)
target_include_directories(example_autonomy_protocol PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(example_autonomy_protocol
  lib_geometry
  lib_environment
  lib_util
  lib_mediation_layer
  lib_ros_node
  lib_ros_view
  lib_autonomy_protocol
  yaml-cpp
  ${catkin_LIBRARIES})
set_target_properties(example_autonomy_protocol
  PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
  RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
)

add_executable(student_autonomy_protocol student_autonomy_protocol.cc)
target_include_directories(student_autonomy_protocol PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(student_autonomy_protocol
  lib_geometry
  lib_environment
  lib_util
  lib_mediation_layer
  lib_ros_node
  lib_ros_view
  lib_autonomy_protocol
  yaml-cpp
  ${catkin_LIBRARIES})
set_target_properties(student_autonomy_protocol
  PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
  RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
)
