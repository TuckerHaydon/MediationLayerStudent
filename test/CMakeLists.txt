cmake_minimum_required(VERSION 3.1)

find_package (Eigen3 3.3 NO_MODULE)

add_executable(path_planning_tests path_planning_tests.cc)
target_include_directories(path_planning_tests PUBLIC 
  ../src/graph
  ../src/algorithm
  ../src/gui
  ../src/environment
  ../src/constraint
  ../src/geometry
  )
target_link_libraries(path_planning_tests PUBLIC path_planning Eigen3::Eigen)

add_executable(geometry_tests geometry_tests.cc)
target_include_directories(geometry_tests PUBLIC 
  ../src/geometry
)
target_link_libraries(geometry_tests PUBLIC path_planning Eigen3::Eigen)

