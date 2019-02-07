cmake_minimum_required(VERSION 3.7)
enable_testing()

project(dijkstra)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# choose debug / release
if(NOT CMAKE_BUILD_TYPE)
    #set(CMAKE_BUILD_TYPE Release)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# set compiler optimisations
set(CMAKE_CXX_FLAGS_DEBUG "-O3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(SOURCE_FILES
    src/sim.cpp
    src/list.hpp
    src/graph.hpp
    src/edge.hpp
    )

# create test executable
add_executable(dijkstra ${SOURCE_FILES})
