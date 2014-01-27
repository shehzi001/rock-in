#-------------------------------------------------
#
# Project created by QtCreator 2013-11-19T14:37:43
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = tracker-gui
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    glwidget.cpp \
    sensor.cpp \
    params.cpp

HEADERS  += mainwindow.h \
    glwidget.h \
    sensor.h \
    params.h

FORMS    += mainwindow.ui \
    params.ui

unix:!symbian {


QMAKE_LFLAGS += -Wl,-rpath=.

LIBS += -L$$V4R_DIR/lib \
        -lv4rTracker \
        -lv4rTomGine \
        -lopencv_core

INCLUDEPATH += /usr/local/include \
               $$V4R_DIR/

}

unix:!mac:!symbian {

    DEFINES += linux \
               LINUX

}
