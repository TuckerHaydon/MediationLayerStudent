cmake_minimum_required(VERSION 2.8.3)
# project(snap_cam)
# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -Wno-format-security -Wno-literal-suffix")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  visualization_msgs
  jsk_rviz_plugins
  geometry_msgs
)

# catkin_package(
#   CATKIN_DEPENDS visualization_msgs
# )

add_library(ros2d_lib 
  trajectory2d_view.h
  potential2d_view.h
  point2d_potential_view.h
  # marker_publisher_node.h
  # marker_publisher_node.cc 
  # polygon_publisher_node.h
  # polygon_publisher_node.cc 
  tmp.cc
  )
target_include_directories(ros2d_lib PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(ros2d_lib ${catkin_LIBRARIES})

add_dependencies(ros2d_lib visualization_msgs_generate_messages_cpp)
add_dependencies(ros2d_lib geometry_msgs_generate_messages_cpp)
