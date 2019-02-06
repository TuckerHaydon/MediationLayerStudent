cmake_minimum_required(VERSION 3.1)
project(PathPlanning)
set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wextra")

add_subdirectory(Geometry)

add_subdirectory(src)
add_subdirectory(examples)
add_subdirectory(test) 
