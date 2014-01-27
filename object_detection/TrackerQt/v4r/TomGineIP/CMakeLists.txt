cmake_minimum_required(VERSION 2.8)

set(V4R_TOMGINE_IP 1 CACHE BOOL "TomGine GPU image processing library" )
IF(V4R_TOMGINE_IP)
project(v4rTomGineIP)

add_definitions(-DLINUX)
add_definitions(-DGL_GLEXT_PROTOTYPES)

# create tomgine include directory name
string(REGEX REPLACE "v4r" "" TOMGINE_IP_INCLUDE_NAME ${PROJECT_NAME})

# define shader path
set(TOMGINE_IP_SHADER ${CMAKE_INSTALL_PREFIX}/include/v4r/${TOMGINE_IP_INCLUDE_NAME}/shader/)

add_definitions(-DTOMGINE_IP_SHADER="${TOMGINE_IP_SHADER}")
message( STATUS "TOMGINE_IP_SHADER is ${TOMGINE_IP_SHADER}")

set(SOURCE_CPP 
	tgHistDesc2D.cpp
	tgHistogram.cpp
	tgImageProcessor.cpp
	tgNurbsCurve.cpp
	tgNurbsSurface.cpp
	tgNurbsVolume.cpp
	tgPlotIP.cpp
	tgShader.cpp
	)

set(SOURCE_H
	tgHistDesc2D.h
	tgHistogram.h
	tgImageProcessor.h
	tgNurbsCurve.h
	tgNurbsSurface.h
	tgNurbsVolume.h
	tgPlotIP.h
	tgShader.h
	)
	
set(SOURCE_SHADER
  shader/color.frag
  shader/coxdeboor.c
  shader/edgetest.frag
  shader/edgetest.vert
  shader/ipAdd.frag
  shader/ipDotConvolution.frag
  shader/ipDilate.frag
  shader/ipGauss.frag
  shader/ipGradientsX.frag
  shader/ipGradientsY.frag
  shader/ipInvert.frag
  shader/ipMultiply.frag
  shader/ipParam2Polar.frag
  shader/ipRGB2HSV.frag
  shader/ipSobel.frag
  shader/ipSobelNormal.frag
  shader/ipSobelAdaptive.frag
  shader/ipSpreading.frag
  shader/ipThinning.frag
  shader/ipThreshold.frag
  shader/mmConfidence.frag
  shader/mmConfidence.vert
  shader/nurbscurve.vert
  shader/nurbssurface.vert
  shader/nurbsvolume.vert
  shader/texColorTest.frag
  shader/texColorTest.vert
  shader/texDepthTest.frag
  shader/texDepthTest.vert
  shader/texEdgeTest.frag
  shader/texEdgeTest.vert
  shader/texturing.frag
  shader/texturing.vert
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_CPP})
target_link_libraries(${PROJECT_NAME} v4rTomGine)

if(${V4R_BUILD})

	message(STATUS "TomGineIP: Building within v4r.")
	v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

else(${V4R_BUILD})

  install(DIRECTORY DESTINATION include/v4r/${TOMGINE_IP_INCLUDE_NAME})
  install(FILES ${SOURCE_H} DESTINATION include/v4r/${TOMGINE_IP_INCLUDE_NAME})
  install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

endif(${V4R_BUILD})

install(DIRECTORY DESTINATION include/v4r/${TOMGINE_IP_INCLUDE_NAME}/shader)
install(FILES ${SOURCE_SHADER} DESTINATION include/v4r/${TOMGINE_IP_INCLUDE_NAME}/shader)

ENDIF(V4R_TOMGINE_IP)
