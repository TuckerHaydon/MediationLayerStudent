cmake_minimum_required(VERSION 3.1)

set(TARGET path_planning_tests)

find_package (Eigen3 3.3 NO_MODULE)

add_executable(${TARGET} main.cc)

target_include_directories(${TARGET} PUBLIC 
  ../src/graph
  ../src/algorithm
  ../src/gui
  ../src/environment
  ../src/constraint
  )

target_link_libraries(${TARGET} PUBLIC path_planning geometry Eigen3::Eigen)

