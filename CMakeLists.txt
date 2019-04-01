cmake_minimum_required(VERSION 3.0)
project(MPCDPS)

if(MSVC)
message("MSVC")
add_definitions(-D_USE_MATH_DEFINES)
endif()

add_subdirectory(Core)
add_subdirectory(PointCloud)