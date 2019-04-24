set(TARGET lib_ros_view)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  visualization_msgs
  geometry_msgs
)


set(SOURCE_FILES
  balloon_view.cc
  polyhedron_view.cc
  view_manager.cc
  plane3d_view.cc
  quad_view.cc
)

add_library(${TARGET} STATIC ${SOURCE_FILES})

target_include_directories(${TARGET} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${TARGET} PUBLIC
  lib_mediation_layer
  lib_environment
  lib_geometry
  Eigen3::Eigen
  ${catkin_LIBRARIES}
)

add_dependencies(${TARGET} visualization_msgs_generate_messages_cpp)
add_dependencies(${TARGET} geometry_msgs_generate_messages_cpp)

set_target_properties(${TARGET} PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)

target_compile_options(${TARGET} PRIVATE 
  -Wfatal-errors
)
