cmake_minimum_required(VERSION 2.8.3)

set(TARGET mediation_layer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

add_executable(${TARGET} main.cc)
target_include_directories(${TARGET} PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${TARGET} 
  lib_geometry
  lib_environment
  lib_util
  lib_mediation_layer
  yaml-cpp
  ${catkin_LIBRARIES})

set_target_properties( ${TARGET}
  PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin/exe"
)
