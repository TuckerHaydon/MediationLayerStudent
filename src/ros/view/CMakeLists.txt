set(TARGET lib_ros_view)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  visualization_msgs
  geometry_msgs
)

add_library(${TARGET} INTERFACE)
target_include_directories(${TARGET} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR} ${catkin_INCLUDE_DIRS})
target_link_libraries(${TARGET} INTERFACE ${catkin_LIBRARIES})

add_dependencies(${TARGET} visualization_msgs_generate_messages_cpp)
add_dependencies(${TARGET} geometry_msgs_generate_messages_cpp)
