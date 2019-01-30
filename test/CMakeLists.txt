cmake_minimum_required(VERSION 3.1)

set(TARGET path_planning_tests)

find_package (Eigen3 3.3 NO_MODULE)

add_executable(${TARGET} main.cc)
target_include_directories(${TARGET} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../src)
target_link_libraries(${TARGET} PUBLIC path_planning Eigen3::Eigen)

