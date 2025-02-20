TEMPLATE = app
TARGET = 2708prg
DESTDIR = .

QT += core gui widgets serialport

DEPENDPATH  += . ./GeneratedFiles
MOC_DIR     += ./GeneratedFiles
UI_DIR      += ./GeneratedFiles
RCC_DIR     += ./GeneratedFiles

CONFIG += qt warn_off thread sdk_no_version_check c++11
CONFIG(debug, debug|release) {
    DEFINES += _DEBUG
    OBJECTS_DIR = debug
    CONFIG -= release
    DESTDIR = debug
}
CONFIG(release, debug|release) {
    DEFINES     += QT_NO_WARNING_OUTPUT
    OBJECTS_DIR = release
    CONFIG -= debug
    DESTDIR = release
}

DEFINES += QT_GUI_LIB QT_WIDGETS_LIB

INCLUDEPATH += . \
    ./GeneratedFiles \
    $(QTDIR64)/include \
    $(QTDIR64)/include/QtCore \
    $(QTDIR64)/include/QtGui \
    $(QTDIR64)/include/QtSerialPort \
    $(QTDIR64)/include/QtWidgets

HEADERS += \
    hexFile.h \
    guiMainWindow.h \
    qLedWidget.h \
    senderthread.h

SOURCES += \
    hexFile.cpp \
    guiMainWindow.cpp \
    main.cpp \
    qLedWidget.cpp \
    senderthread.cpp

FORMS += \
    guiMainWindow.ui

