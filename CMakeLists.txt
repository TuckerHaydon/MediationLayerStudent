cmake_minimum_required(VERSION 3.1)
project(path_planning)
set(CMAKE_CXX_STANDARD 11)
add_definitions(-std=c++11)

add_subdirectory(CppPlot)
 
include_directories(src)
 
file(GLOB SOURCES "src/*.cc")
 
add_executable(path_planning ${SOURCES})
target_link_libraries(path_planning cpp_plot)
