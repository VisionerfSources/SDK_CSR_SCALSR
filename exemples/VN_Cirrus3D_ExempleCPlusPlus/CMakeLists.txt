cmake_minimum_required(VERSION 3.5)

project(VN_Cirrus3D_ExempleCPlusPlus LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(../../src)

add_executable(VN_Cirrus3D_ExempleCPlusPlus main.cpp
    Cirrus3D.h Cirrus3D.cpp
    ../../src/VN_SDK_CloudSavingFunctions.c ../../src/VN_SDK_CloudSavingFunctions.h
    ../../src/VN_SDK_CommandFunctions.c ../../src/VN_SDK_CommandFunctions.h
    ../../src/VN_SDK_CommunicationFunctions.c ../../src/VN_SDK_CommunicationFunctions.h
    ../../src/VN_SDK_Constant.h
    ../../lib/API/VN_Cirrus3D_DataId.h
)


if (win32)
    target_link_libraries(VN_Cirrus3D_ExempleCPlusPlus wsock32 ws2_32)
endif()


include(GNUInstallDirs)
install(TARGETS VN_Cirrus3D_ExempleCPlusPlus
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

