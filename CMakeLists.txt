cmake_minimum_required(VERSION 3.0)

add_definitions(-std=c++11)

project(MPCDPS)
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR})

if(MSVC)
message("MSVC")
add_definitions(-D_USE_MATH_DEFINES)
endif()

add_subdirectory(Core)
add_subdirectory(PointCloud)
