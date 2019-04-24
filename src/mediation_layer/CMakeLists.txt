cmake_minimum_required(VERSION 3.5.0)

set(TARGET lib_mediation_layer)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(SOURCE_FILES
  mediation_layer.cc
  physics_simulator.cc
  quad_state.cc
  quad_state_dispatcher.cc
  quad_state_guard.cc
  quad_state_warden.cc
  trajectory.cc
  trajectory_dispatcher.cc
  trajectory_warden.cc
)

add_library(${TARGET} STATIC ${SOURCE_FILES})

target_include_directories(${TARGET} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${TARGET} PUBLIC
  lib_geometry
  lib_integration
  lib_util
  lib_ros_node
  Eigen3::Eigen
)

set_target_properties(${TARGET} PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)

target_compile_options(${TARGET} PRIVATE 
  -Wfatal-errors
)
