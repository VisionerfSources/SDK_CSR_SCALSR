TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

win32 {
LIBS +=  -lws2_32
LIBS += -L../../lib/Win/ -lVnCirrus3DLib
}

unix {
LIBS += -L../../lib/Ubuntu/ -lVnCirrus3DLib
}

INCLUDEPATH +=../../lib/API/

HEADERS += \
    ../../lib/API/VN_Cirrus3D_DataId.h \
    ../../lib/API/VN_Cirrus3D.API.h\

SOURCES += \
    main.c \


