cmake_minimum_required(VERSION 2.8.3)

find_package(Boost 1.40.0 COMPONENTS filesystem system iostreams REQUIRED)

add_executable(main
  main.cc 
  dijkstra2d.h 
  dijkstra2d.cc
  depth_first_search2d.h 
  depth_first_search2d.cc
  a_star2d.h 
  a_star2d.cc
  gnuplot-iostream.h
  gui2d.h
  gui2d.cc
  )
target_link_libraries(main 
  lib_graph 
  lib_environment 
  lib_util
  ${Boost_LIBRARIES})

add_executable(examples
  examples.cc)
target_link_libraries(examples lib_graph lib_environment lib_util)
