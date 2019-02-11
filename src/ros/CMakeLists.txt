cmake_minimum_required(VERSION 2.8.3)
# project(snap_cam)
# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -Wno-format-security -Wno-literal-suffix")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  # message_generation
  nav_msgs
)

# catkin_package(
#   CATKIN_DEPENDS nav_msgs message_runtime
# )

# include_directories(
# 	${catkin_INCLUDE_DIRS}
# )

# file(GLOB_RECURSE SOURCE_FILES "*.cc")
add_library(ros2d_lib marker_publisher_node.cc marker_publisher_node.h)
target_include_directories(ros2d_lib PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(ros2d_lib ${catkin_LIBRARIES})
# add_dependencies(${PROJECT_NAME} nav_msgs_gencpp nav_msgs_generate_messages_cpp)
