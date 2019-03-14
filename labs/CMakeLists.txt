cmake_minimum_required(VERSION 2.8.3)

add_executable(lab4 
  lab4.cc 
  dijkstra_student.h 
  dijkstra_student.cc)
target_link_libraries(lab4 lib_graph lib_util)
