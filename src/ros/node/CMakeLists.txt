find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  visualization_msgs
  geometry_msgs
)

add_library(node 
  trajectory2d_view.h
  potential_view.h
  point2d_potential_view.h
  )
target_include_directories(node PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(node ${catkin_LIBRARIES})

add_dependencies(node visualization_msgs_generate_messages_cpp)
add_dependencies(node geometry_msgs_generate_messages_cpp)
