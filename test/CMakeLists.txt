cmake_minimum_required(VERSION 3.5.0)

find_package (Eigen3 3.3 NO_MODULE)

# Geometry library tests
add_executable(geometry_tests geometry_tests.cc)
target_link_libraries(geometry_tests PUBLIC lib_geometry)
set_target_properties(geometry_tests PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)
target_compile_options(geometry_tests PRIVATE 
  -Wfatal-errors
)


# Graph library tests
add_executable(graph_tests graph_tests.cc)
target_link_libraries(graph_tests PUBLIC lib_graph)
set_target_properties(graph_tests PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)
target_compile_options(graph_tests PRIVATE 
  -Wfatal-errors
)


# Environment directory unit tests
add_executable(environment_tests environment_tests.cc)
target_link_libraries(environment_tests PUBLIC lib_environment)
set_target_properties(environment_tests PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)
target_compile_options(environment_tests PRIVATE 
  -Wfatal-errors
)


# Integration library tests
add_executable(integration_tests integration_tests.cc)
target_link_libraries(integration_tests PUBLIC 
  lib_integration 
  Eigen3::Eigen)
set_target_properties(integration_tests PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)
target_compile_options(integration_tests PRIVATE 
  -Wfatal-errors
)


add_executable(mediation_layer_tests mediation_layer_tests.cc)
target_include_directories(mediation_layer_tests PUBLIC 
)
target_link_libraries(mediation_layer_tests PUBLIC 
  lib_geometry
  lib_mediation_layer
  Eigen3::Eigen)
set_target_properties(mediation_layer_tests PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)
target_compile_options(mediation_layer_tests PRIVATE 
  -Wfatal-errors
)
