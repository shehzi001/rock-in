SET(V4R_TRACKER 1 CACHE BOOL "Model-based 6-dof object tracker. (depends: TomGine)" )
IF(V4R_TRACKER)
project(v4rTracker)

add_definitions(-DLINUX)
add_definitions(-DGL_GLEXT_PROTOTYPES)

SET(	SOURCE_H
	Distribution.h
	EdgeTracker.h
	Filter.h
	headers.h
	ModelEntry.h
	ModelLoader.h
	myPredictor.h
	Noise.h
	Particle.h
	ply.h
	PlyStructure.h
	Predictor.h
	Resources.h	
	TextureTracker.h
	Timer.h
	TrackerAPI.h
	Tracker.h
	TrackerModel.h)

SET(SOURCE_CPP
	Distribution.cpp
	EdgeTracker.cpp
	ModelEntry.cpp
	ModelLoader.cpp
	myPredictor.cpp
	Noise.cpp
	Particle.cpp
	ply.c
	Predictor.cpp
	Resources.cpp
	TextureTracker.cpp
	Timer.cpp
	Tracker.cpp
	TrackerModel.cpp)

add_library(${PROJECT_NAME} SHARED  ${SOURCE_CPP})
target_link_libraries(${PROJECT_NAME} GL GLU GLEW)
target_link_libraries(${PROJECT_NAME} v4rTomGine v4rTomGineIP)
target_link_libraries(${PROJECT_NAME} ${OPENCV_LIBRARIES})
v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

#add_subdirectory(example)

ENDIF(V4R_TRACKER)
