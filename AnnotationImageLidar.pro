#-------------------------------------------------
#
# Project created by QtCreator 2017-04-24T14:33:12
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = AnnotationImageLidar
TEMPLATE = app


SOURCES += main.cpp\
    realobject.cpp \
    context.cpp \
    xmlmanager.cpp \
    mainwindow.cpp \
    realobjectinformationwidget.cpp

HEADERS  += realobject.h \
    poses.h \
    context.h \
    xmlmanager.h \
    mainwindow.h \
    realobjectinformationwidget.h

FORMS    += mainwindow.ui

DISTFILES += \
    CMakeLists.txt

OTHER_FILES += \
    CMakeLists.txt

RESOURCES += \
    resourcesSQL.qrc
