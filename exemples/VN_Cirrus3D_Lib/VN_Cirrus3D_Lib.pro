QT       -= core gui
TARGET = VnCirrus3DLib
TEMPLATE = lib
#CONFIG += staticlib

win32 {
LIBS +=  -lws2_32
}

INCLUDEPATH += ../../src

SOURCES += \
        ../../src/VN_SDK_CloudSavingFunctions.c \
        ../../src/VN_SDK_CommandFunctions.c \
        ../../src/VN_SDK_CommunicationFunctions.c

HEADERS += \
        ../../src/VN_SDK_Constant.h \
        ../../src/VN_SDK_CloudSavingFunctions.h \
        ../../src/VN_SDK_CommandFunctions.h \
        ../../src/VN_SDK_CommunicationFunctions.h

