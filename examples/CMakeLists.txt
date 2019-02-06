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
  )
target_link_libraries(path_planning_map_example PUBLIC path_planning Eigen3::Eigen)

add_executable(path_planning_sfc2d_example sfc2d_example.cc)
target_include_directories(path_planning_sfc2d_example PUBLIC 
  ../src/graph
  ../src/algorithm
  ../src/gui
  ../src/environment
  ../src/geometry
  )
target_link_libraries(path_planning_sfc2d_example PUBLIC path_planning Eigen3::Eigen)
