set(TARGET lib_ros_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  visualization_msgs
  geometry_msgs
  mg_msgs
)

add_library(${TARGET} INTERFACE)
target_include_directories(${TARGET} INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${TARGET} INTERFACE ${catkin_LIBRARIES})

add_dependencies(${TARGET} std_msgs_generate_messages_cpp)
add_dependencies(${TARGET} visualization_msgs_generate_messages_cpp)
add_dependencies(${TARGET} geometry_msgs_generate_messages_cpp)
add_dependencies(${TARGET} mg_msgs_generate_messages_cpp)
