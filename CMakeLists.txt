cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

cmake_policy(SET CMP0020 NEW)

project      (AnnotationImageLidar)
#find_package (Qt4 REQUIRED COMPONENTS QtMain QtCore QtGui QtSql QtXml QtNetwork)
find_package (Qt4 REQUIRED)
include(${QT_USE_FILE})
#find_package (VTK REQUIRED)
#include(${VTK_USE_FILE})
find_package (PCL 1.8 REQUIRED)


include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

set  (project_SOURCES main.cpp  realobject.cpp context.cpp xmlmanager.cpp mainwindow.cpp realobjectinformationwidget.cpp)
set  (project_HEADERS poses.h realobject.h context.h xmlmanager.h mainwindow.h realobjectinformationwidget.h)
#set  (project_FORMS   mainwindow.ui)
#set  (project_SOURCES main.cpp loader.cpp imagevisualizer.cpp)
#set  (project_HEADERS loader.h imagevisualizer.h)
#set  (project_FORMS  )

SET(CMAKE_CXX_FLAGS "-std=c++0x")

QT4_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT4_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

INCLUDE         (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})

ADD_EXECUTABLE  (AnnotationImageLidar ${project_SOURCES}
                                ${project_FORMS_HEADERS}
                                ${project_HEADERS_MOC})

TARGET_LINK_LIBRARIES (AnnotationImageLidar ${QT_LIBRARIES} ${PCL_LIBRARIES} Qt4::QtSql Qt4::QtXml)# QVTK vtkRendering Qt4::QtSql)# ${VTK_LIBRARIES})
#TARGET_LINK_LIBRARIES (AnnotationImageLidar ${QT_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES})

