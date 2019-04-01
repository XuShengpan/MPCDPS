
file(GLOB LOC_INC_LIST "./include/*.h")
file(GLOB LOC_SRC_LIST "./src/*.cpp")

include_directories(../Core/include)
include_directories(../Index/include)
include_directories(./include)

SET(LIBRARY_OUTPUT_PATH  ${CMAKE_SOURCE_DIR}/lib/)

add_library(mpcdps_pointcloud ${LOC_INC_LIST} ${LOC_SRC_LIST})

target_link_libraries(mpcdps_pointcloud mpcdps_core)