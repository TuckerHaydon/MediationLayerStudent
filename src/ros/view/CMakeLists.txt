find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  visualization_msgs
  geometry_msgs
)

add_library(lib_view 
  trajectory2d_view.h
  potential_view.h
  point2d_potential_view.h
  )
target_include_directories(lib_view PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(lib_view ${catkin_LIBRARIES})

add_dependencies(lib_view visualization_msgs_generate_messages_cpp)
add_dependencies(lib_view geometry_msgs_generate_messages_cpp)
