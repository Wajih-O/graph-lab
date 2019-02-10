cmake_minimum_required(VERSION 3.7)
enable_testing()

project(dijkstra)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake-modules")

# choose debug / release
if(NOT CMAKE_BUILD_TYPE)
    #set(CMAKE_BUILD_TYPE Release)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# set compiler optimisations
set(CMAKE_CXX_FLAGS_DEBUG "-O3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(DEPENDENCIES_DIR /home/wo/deps)

# include directories
set(SRC_ROOT ./)
set(TESTS_ROOT ./tests)

include_directories(${SRC_ROOT})
include_directories(${TESTS_ROOT})
include_directories(${DEPENDENCIES_DIR}/include)

set(SRC_FILES
    src/list.hpp
    src/graph.hpp
    src/edge.hpp
    )

file (GLOB_RECURSE UNIT_TESTS_SRC
    ${TESTS_ROOT}/*.h
    ${TESTS_ROOT}/*.tpp
    ${TESTS_ROOT}/*.cpp
)

link_directories(/usr/lib/x86_64-linux-gnu ${DEPENDENCIES_DIR}/lib)

# create test executable
add_executable(dijkstra src/dijkstra.cpp ${SRC_FILES})


# Testing libraries
find_package(fakeit REQUIRED)
find_package(gtest REQUIRED)
set(TEST_LIBRARIES  gtest gmock gtest_main pthread)
set(TEST_SRC ${UNIT_TESTS_SRC} ${SRC_FILES})

# Create test executable
add_executable(tests ${TEST_SRC})
target_link_libraries(tests  ${TEST_LIBRARIES} )
set_target_properties(tests PROPERTIES OUTPUT_NAME "graph_tests")
