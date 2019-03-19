cmake_minimum_required(VERSION 3.1)

find_package (Eigen3 3.3 NO_MODULE)

# Geometry library tests
add_executable(geometry_tests geometry_tests.cc)
target_link_libraries(geometry_tests PUBLIC lib_geometry)


# Graph library tests
add_executable(graph_tests graph_tests.cc)
target_link_libraries(graph_tests PUBLIC lib_graph)


# Environment directory unit tests
add_executable(environment_tests environment_tests.cc)
target_link_libraries(environment_tests PUBLIC lib_environment)


# Integration library tests
add_executable(integration_tests integration_tests.cc)
target_link_libraries(integration_tests PUBLIC 
  lib_integration 
  Eigen3::Eigen)


add_executable(mediation_layer_tests mediation_layer_tests.cc)
target_include_directories(mediation_layer_tests PUBLIC 
  ../src/mediation_layer
  ../src/mediation_layer/potential
  ../src/geometry
)
target_link_libraries(mediation_layer_tests PUBLIC 
  lib_mediation_layer
  Eigen3::Eigen)

