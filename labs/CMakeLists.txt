cmake_minimum_required(VERSION 2.8.3)

add_executable(lab4 
  lab4.cc 
  dijkstra2d.h 
  dijkstra2d.cc)
target_link_libraries(lab4 lib_graph lib_environment lib_util)

add_executable(examples
  examples.cc)
target_link_libraries(examples lib_graph lib_environment lib_util)
