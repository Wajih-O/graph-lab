cmake_minimum_required(VERSION 3.7)
enable_testing()

include("./gate/cmake/HunterGate.cmake")

HunterGate(
    URL "https://github.com/ruslo/hunter/archive/v0.23.198.tar.gz"
    SHA1 "f44b698691eedfe9d1ee6763a6fb9b4af7756c24"
)
project("graph-lab")


set(CMAKE_CXX_STANDARD 11)
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
set(DEPENDENCIES_DIR /home/wo/deps)

# include directories
set(SRC_ROOT ./src)
set(TESTS_ROOT ./tests)

include_directories(${SRC_ROOT})
include_directories(${TESTS_ROOT})


file (GLOB_RECURSE HEX_SRC_FILES
    ${SRC_ROOT}/hex/*.h
    ${SRC_ROOT}/hex/*.hpp
    ${SRC_ROOT}/hex/*.tpp
    ${SRC_ROOT}/hex/*.cpp
)

set(SRC_FILES
    src/list.hpp
    src/graph.hpp
    src/edge.hpp
    ${HEX_SRC_FILES}
    )


file (GLOB_RECURSE UNIT_TESTS_SRC
    ${TESTS_ROOT}/*.h
    ${TESTS_ROOT}/*.hpp
    ${TESTS_ROOT}/*.tpp
    ${TESTS_ROOT}/*.cpp
)

link_directories(/usr/lib/x86_64-linux-gnu $ENV{HOME}/.hunter) #  /usr/local/lib /opt/local/lib)

# Boost
hunter_add_package(Boost COMPONENTS thread system serialization filesystem)
find_package(Boost CONFIG REQUIRED thread system serialization filesystem)
set(BOOST_LIBRARIES Boost::system Boost::thread Boost::serialization Boost::filesystem)


# create test executable
add_executable(dijkstra src/dijkstra.cpp ${SRC_FILES})
target_link_libraries(dijkstra  ${BOOST_LIBRARIES})

add_executable(mst src/mst.cpp ${SRC_FILES})
target_link_libraries(mst  ${BOOST_LIBRARIES})

# Testing libraries

#FakeIt
hunter_add_package(FakeIt)
find_package(FakeIt CONFIG REQUIRED)
# GTest
hunter_add_package(GTest)
find_package(GTest CONFIG REQUIRED)
find_package(GMock CONFIG REQUIRED)

set(TEST_LIBRARIES  ${BOOST_LIBRARIES} FakeIt::FakeIt GTest::main GMock::gmock pthread)
set(TEST_SRC ${UNIT_TESTS_SRC} ${SRC_FILES})

# Create test executable
add_executable(tests ${TEST_SRC})
target_link_libraries(tests  ${TEST_LIBRARIES} )
set_target_properties(tests PROPERTIES OUTPUT_NAME "graph_tests")
