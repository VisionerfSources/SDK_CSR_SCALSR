QT       -= core gui
TARGET = VnCirrus3DLib
TEMPLATE = lib
#CONFIG += staticlib

win32 {
LIBS +=  -lws2_32
}

INCLUDEPATH += ../../Source/

SOURCES += \
        ../../Source/VN_SDK_CloudSavingFunctions.c \
        ../../Source/VN_SDK_CommandFunctions.c \
        ../../Source/VN_SDK_CommunicationFunctions.c

HEADERS += \
        ../../Source/VN_SDK_Constant.h \
        ../../Source/VN_SDK_CloudSavingFunctions.h \
        ../../Source/VN_SDK_CommandFunctions.h \
        ../../Source/VN_SDK_CommunicationFunctions.h

