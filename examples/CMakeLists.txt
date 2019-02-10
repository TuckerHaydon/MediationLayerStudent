cmake_minimum_required(VERSION 3.1)

find_package (Eigen3 3.3 NO_MODULE)

add_executable(path_planning_file_example file_example.cc)
target_include_directories(path_planning_file_example PUBLIC 
  ../src/graph
  ../src/algorithm
  ../src/gui
  ../src/environment
  ../src/geometry
  )
target_link_libraries(path_planning_file_example PUBLIC path_planning Eigen3::Eigen)

add_executable(path_planning_map_example map_example.cc)
target_include_directories(path_planning_map_example PUBLIC 
  ../src/graph
  ../src/algorithm
  ../src/gui
  ../src/environment
  ../src/geometry
  ../src/util
  )
target_link_libraries(path_planning_map_example PUBLIC path_planning Eigen3::Eigen)

add_executable(path_planning_sfc2d_example sfc2d_example.cc)
target_include_directories(path_planning_sfc2d_example PUBLIC 
  ../src/graph
  ../src/algorithm
  ../src/gui
  ../src/environment
  ../src/geometry
  ../src/util
  )
target_link_libraries(path_planning_sfc2d_example PUBLIC path_planning Eigen3::Eigen)

add_executable(path_planning_map2sfc_example map2sfc_example.cc)
target_include_directories(path_planning_map2sfc_example PUBLIC 
  ../src/graph
  ../src/algorithm
  ../src/gui
  ../src/environment
  ../src/geometry
  ../src/util
  )
target_link_libraries(path_planning_map2sfc_example PUBLIC path_planning Eigen3::Eigen)

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
target_link_libraries(path_planning_mediation_layer_example PUBLIC path_planning Eigen3::Eigen)
