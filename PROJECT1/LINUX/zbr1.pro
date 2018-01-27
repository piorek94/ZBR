#-------------------------------------------------
#
# Project created by QtCreator 2015-03-13T10:39:55
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = zbr1
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    kinematics.cpp \
    trajectory.cpp \
    trajectorydialog.cpp \
    qcustomplot.cpp \
    infodialog.cpp

HEADERS  += mainwindow.h \
    kinematics.h \
    trajectory.h \
    typedefs.h \
    trajectorydialog.h \
    qcustomplot.h \
    infodialog.h

FORMS    += mainwindow.ui \
    trajectorydialog.ui \
    infodialog.ui
