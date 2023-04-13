cmake_minimum_required(VERSION 3.5)
project(routing)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB_RECURSE TOOLS ${PROJECT_SOURCE_DIR}/src/*.cpp)
add_executable(main demo.cpp ${TOOLS})
target_link_libraries(main ${PCL_LIBRARIES})
