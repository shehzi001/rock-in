set(V4R_RECOGNIZER_3D 1 CACHE BOOL "SIFT based 3D pose estimation of objects." )
if(V4R_RECOGNIZER_3D)

FIND_PACKAGE (OpenCV QUIET)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})


# Recognizer3D lib ---------------------------
project(v4rRecognizer3D)
set(SOURCE_CPP
        ODetect3D.cc
        CodebookEntry.cc
        Object3D.cc
        PoseCv.cc
        ModelObject3D.cc
        Keypoint.cc
        KeypointDescriptor.cc
        Definitions.cc
        DetectGPUSIFT.cc
        DetectSIFT.cc
        Vector2.cc
        SDraw.cc
        SPolygon.cc
        Geometry.cc
        Except.cc
        Math.cc
        Config.cc
        ConfigFile.cc
        Recognizer3D.cpp)

set(SOURCE_H
        Array.hh
        CodebookEntry.hh
        ConfigFile.hh
        Config.hh
        cvgeometry.h
        Definitions.hh
        DetectGPUSIFT.hh
        DetectSIFT.hh
        Except.hh
        Geometry.hh
        headers.h
        Homography.hh
        KeyClusterPair.hh
        KeypointDescriptor.hh
        Keypoint.hh
        KeypointPair.hh
        Math.hh
        ModelObject3D.hh
        Object3D.hh
        ODetect3D.hh
        PNamespace.hh
        PoseCv.hh
        Pose.hh
        Recognizer3D.h
        SDraw.hh
        SMatrix.hh
        SPolygon.hh
        Vector2.hh
        Vector3.hh
        Array.ic
        CodebookEntry.ic
        Geometry.ic
        Homography.ic
        Math.ic
        SMatrix.ic
        Vector2.ic
        Vector3.ic)

add_library(${PROJECT_NAME} SHARED ${SOURCE_CPP} ${SOURCE_H})
v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")
target_link_libraries(${PROJECT_NAME} ${CGAL_LIBRARIES} ${OPENCV_LIBRARIES} siftgpu v4rTomGine)

endif(V4R_RECOGNIZER_3D)
