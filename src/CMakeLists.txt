cmake_minimum_required(VERSION 3.1)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost 1.40.0 COMPONENTS filesystem system iostreams REQUIRED)

# Build pre-existing libraries
add_subdirectory(P4)
add_subdirectory(mg_msgs)
add_subdirectory(lib)

# Build custom libraries
add_subdirectory(util)
add_subdirectory(integration)
add_subdirectory(geometry)
add_subdirectory(graph)
add_subdirectory(environment)
add_subdirectory(ros)
add_subdirectory(mediation_layer)

# Build the final executables
add_subdirectory(exe)
