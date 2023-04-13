cmake_minimum_required(VERSION 3.10.2)
project(frenet_optimal_trajectory)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(include)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
file(GLOB_RECURSE curve1d ${PROJECT_SOURCE_DIR}/curve1d/*.cpp)
add_library(Curve1d SHARED ${curve1d})

file(GLOB_RECURSE TOOLS ${PROJECT_SOURCE_DIR}/src/*.cpp)
add_executable(frenet_optimal_trajectory frenet_optimal_trajectory.cpp ${TOOLS})
target_link_libraries(frenet_optimal_trajectory ${PYTHON_LIBRARIES})