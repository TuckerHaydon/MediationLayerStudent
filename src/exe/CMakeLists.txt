cmake_minimum_required(VERSION 2.8.3)

set(TARGET mediation_layer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

add_executable(${TARGET} main.cc)
target_include_directories(${TARGET} PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${TARGET} 
  yaml-cpp
  lib_mediation_layer
  ${catkin_LIBRARIES})
