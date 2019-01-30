cmake_minimum_required(VERSION 3.1)

set(TARGET run_examples)

include_directories("../src")
add_executable(${TARGET} main.cc)
target_link_libraries(${TARGET} PUBLIC path_planning)

