
file(GLOB INC_LIST   "./include/*")
file(GLOB SRC_LIST   "./src/*.cpp")

source_group(Memory FILES
./include/RefManager.h
./src/RefManager.cpp
./include/SmartArray.h
./include/SmartArray.inl
./include/SmartArray2D.h
./include/SmartArray2D.inl
./include/SmartArrayReal2D.h
./include/SmartArrayReal2D.inl
./include/SmartPointer.h
./include/SmartPointer.inl
)

source_group(Math FILES
./include/Tuple.h
./include/VectorK.h
./include/VectorX.h
./include/Point2.h
./include/Point3.h
./include/Determinate.h
./include/Matrix.h
./include/Quaternion.h
./include/Quaternion.inl
./include/Transform.h
./include/Histogram.h
)

source_group(Geometry FILES
./include/Line2.h
./include/LineSegment2.h
./include/LineSegment3.h
./include/Rect2.h
./include/RRect2.h
./include/Box3.h
./include/Circle.h
./include/Sphere.h
./include/Plane.h
./include/FastConvexHull2D.h
./include/OBBox.h
./include/Line3.h
./include/OBRect.h
./include/Spline2.h
./include/Rect3.h
./include/Polygon.h
./include/Sector.h
./include/AABB.h
)

source_group(Public FILES
./include/PublicFunc.h
./include/PublicInfo.h
./include/FixedSizeMap.h
)

include_directories(./include/)

add_definitions(-DMPCDPS_EXPORT)

SET(LIBRARY_OUTPUT_PATH  ${CMAKE_SOURCE_DIR}/lib/)
add_library(mpcdps_core SHARED
${INC_LIST}
${SRC_LIST}
)
