cmake_minimum_required ( VERSION 2.6 )
PROJECT(v4r)

include(opencv.cmake)
include(v4rutils.cmake)

SET(LIBRARY_OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib")
SET(EXECUTABLE_OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/bin")

set(V4R_BUILD 1)

add_definitions(-DLINUX)
add_definitions("-Wall -Wl,--no-undefined -fno-strict-aliasing -pthread")

include_directories(.)

add_subdirectory(apps)
add_subdirectory(v4r)
add_subdirectory(v4rexternal)

message(STATUS "V4R_LIBRARIES: " ${V4R_LIBRARIES})
message(STATUS "V4R_LIBRARIES_PC: " ${V4R_LIBRARIES_PC})

# install target
configure_file(v4r.pc.in "${PROJECT_BINARY_DIR}/v4r.pc" @ONLY)
INSTALL(FILES ${PROJECT_BINARY_DIR}/v4r.pc DESTINATION lib/pkgconfig)

# uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)
add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
