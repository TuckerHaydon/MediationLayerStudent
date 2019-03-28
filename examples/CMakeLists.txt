cmake_minimum_required(VERSION 3.1)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(path_planning_file_example file_example.cc)
target_include_directories(path_planning_file_example PUBLIC 
  ../src/graph
  ../src/algorithm
  ../src/gui
  ../src/environment
  ../src/geometry
  )
target_link_libraries(path_planning_file_example PUBLIC 
  lib_geometry
  lib_graph
  lib_environment
  Eigen3::Eigen)



add_executable(path_planning_map_example map_example.cc)
target_include_directories(path_planning_map_example PUBLIC 
  ../src/graph
  ../src/algorithm
  ../src/gui
  ../src/environment
  ../src/geometry
  ../src/util
  )
target_link_libraries(path_planning_map_example PUBLIC 
  lib_geometry
  lib_graph
  lib_environment
  lib_util
  Eigen3::Eigen)



add_executable(path_planning_sfc2d_example sfc2d_example.cc)
target_include_directories(path_planning_sfc2d_example PUBLIC 
  ../src/graph
  ../src/algorithm
  ../src/gui
  ../src/environment
  ../src/geometry
  ../src/util
  )
target_link_libraries(path_planning_sfc2d_example PUBLIC 
  lib_geometry
  lib_graph
  lib_environment
  lib_util
  Eigen3::Eigen)



add_executable(path_planning_map2sfc_example map2sfc_example.cc)
target_include_directories(path_planning_map2sfc_example PUBLIC 
  ../src/graph
  ../src/algorithm
  ../src/gui
  ../src/environment
  ../src/geometry
  ../src/util
  )
target_link_libraries(path_planning_map2sfc_example PUBLIC 
  lib_geometry
  lib_graph
  lib_environment
  lib_util
  Eigen3::Eigen)



add_executable(path_planning_mediation_layer_example mediation_layer_example.cc)
target_include_directories(path_planning_mediation_layer_example PUBLIC 
  ../src/graph
  ../src/algorithm
  ../src/gui
  ../src/environment
  ../src/geometry
  ../src/util
  ../src/mediation_layer
  ../src/mediation_layer/potential
  ../src/integration
  )
target_link_libraries(path_planning_mediation_layer_example PUBLIC 
  lib_geometry
  lib_graph
  lib_environment
  lib_util
  lib_integration
  Eigen3::Eigen)



add_executable(path_planning_ros2d_example ros2d_example.cc)
target_include_directories(path_planning_ros2d_example PUBLIC 
  ../src/graph
  ../src/algorithm
  ../src/gui
  ../src/environment
  ../src/geometry
  ../src/util
  ../src/mediation_layer
  ../src/mediation_layer/potential
  ../src/integration
  ../src/ros/node
  ../src/ros/view
  )
target_link_libraries(path_planning_ros2d_example PUBLIC 
  lib_geometry
  lib_graph
  lib_environment
  lib_util
  lib_integration
  lib_ros_view
  lib_ros_node
  Eigen3::Eigen)

