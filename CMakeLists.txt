cmake_minimum_required(VERSION 3.10)
project(AStarOSM)

set(CMAKE_CXX_STANDARD 17)

# Osmium and its dependencies
include_directories(${CMAKE_SOURCE_DIR}/../libosmium/include)
include_directories(${CMAKE_SOURCE_DIR}/../protozero/include) 

# Add executable
add_executable(a_star_osm
    src/main.cpp
    src/graph.cpp
)

# Link system libraries required by Osmium
target_link_libraries(a_star_osm
    bz2
    z
    expat
)
