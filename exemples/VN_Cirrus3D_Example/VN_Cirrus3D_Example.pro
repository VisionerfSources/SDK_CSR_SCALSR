TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

win32 {
LIBS +=  -lws2_32
}

LIBS += -L../../lib/Ubuntu/ -lVnCirrus3DLib
#INCLUDEPATH =/home/vn/VNroot/VNSource/App/CirrusSDK/VN_Cirrus3D_Lib/Helper

#PRE_TARGETDEPS += /home/vn/VNroot/VNSource/App/CirrusSDK/build-VN_Cirrus3D_Lib-Desktop_Qt_5_15_2_GCC_64bit-Release/libVnCirrus3DLib.a

INCLUDEPATH +=../../lib/API/

HEADERS += \
    ../../lib/API/VN_Cirrus3D_DataId.h \
    ../../lib/API/VN_Cirrus3D.API.h\

SOURCES += \
    main.c \


