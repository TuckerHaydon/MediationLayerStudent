set(TARGET lib_ros_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  visualization_msgs
  geometry_msgs
)

add_library(${TARGET}
  marker_publisher_node.cc
  marker_publisher_node.h
  publisher_guard.h
  state2d_dispatcher.h
  )
target_include_directories(${TARGET} PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${TARGET} ${catkin_LIBRARIES})

add_dependencies(${TARGET} visualization_msgs_generate_messages_cpp)
add_dependencies(${TARGET} geometry_msgs_generate_messages_cpp)
