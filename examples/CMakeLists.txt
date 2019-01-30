cmake_minimum_required(VERSION 3.1)

set(TARGET path_planning_example)

find_package (Eigen3 3.3 NO_MODULE)

add_executable(${TARGET} main.cc)

target_include_directories(${TARGET} PUBLIC 
  ../src/graph
  ../src/algorithm
  ../src/gui
  ../src/environment
  ../Geometry/src
  )

target_link_libraries(${TARGET} PUBLIC path_planning Eigen3::Eigen)

