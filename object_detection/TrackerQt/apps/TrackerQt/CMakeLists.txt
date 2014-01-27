set(APPS_TRACKER_QT 1 CACHE BOOL "Qt GUI for object tracker (v4rTracker)" )
if(APPS_TRACKER_QT)

project(trackerQt)


find_package(Qt4 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})


set(QT_USE_QTOPENGL TRUE)
include(${QT_USE_FILE})

add_definitions(${QT_DEFINITIONS})
add_definitions(-Dlinux -D__x86_64__)

include_directories(${QT_INCLUDE_DIR})
include_directories(${QT_INCLUDE_DIR}/QtOpenGL)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

message(STATUS "QT_INCLUDE_DIR: ${QT_INCLUDE_DIR}")
message(STATUS "QT_USE_FILE: ${QT_USE_FILE}")
message(STATUS "QT_DEFINITIONS: ${QT_DEFINITIONS}")
message(STATUS "QT_LIBRARIES: ${QT_LIBRARIES}")


set(SOURCES
        gltracker.cpp
        main.cpp
        mainwindow.cpp
        params.cpp
        recognizer.cpp
        sensor.cpp
)

set(HEADERS
        gltracker.h
        mainwindow.h
        params.h
        recognizer.h
        sensor.h
)

set(FORMS
        mainwindow.ui
        params.ui
)

QT4_WRAP_CPP(HEADERS_MOC ${HEADERS})
QT4_WRAP_UI(FORMS_MOC ${FORMS})

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS_MOC} ${FORMS_MOC})
target_link_libraries(${PROJECT_NAME} ${OPENCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME} v4rTracker v4rRecognizer3D)
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})

endif(APPS_TRACKER_QT)
