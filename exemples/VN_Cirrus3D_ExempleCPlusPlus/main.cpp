#include <iostream>
#include "Cirrus3D.h"

extern "C"
{
#include "VN_SDK_CommunicationFunctions.h"
#include "VN_SDK_CommandFunctions.h"
#include "VN_SDK_CloudSavingFunctions.h"
}


using namespace std;

void VN_PrintError(VN_tERR_Code err)
{
    switch (err) {
    case VN_eERR_NoError:
        printf("The request succeeded. (0)");
        break;
    case VN_eERR_UndefinedError:
        printf("The sensor has encountered a situation it does not know how to handle. (1)");
        break;
    case VN_eERR_WSAStartup:
        printf("The function WSAStartup returned an error. The WSAStartup function initiates use of the Winsock DLL by a process. (2)");
        break;
    case VN_eERR_WSACleanup:
        printf("The function WSACleanup returned an error. The WSACleanup function terminates use of the Winsock 2 DLL (Ws2_32.dll). (3)");
        break;
    case VN_eERR_SocketError:
        printf("The socket is invalid. (4)");
            break;
    case VN_eERR_SocketConnectError:
        printf("The connection to the sensor failed. (5)");
            break;
    case VN_eERR_SocketSendError:
        printf("Sending data to the sensor failed. (6)");
            break;
    case VN_eERR_SocketCloseError:
        printf("Closing the connection with the sensor failed. (7)");
            break;
    case VN_eERR_SocketRecvError:
        printf("Data was not received. (8)");
            break;
    case VN_eERR_SocketIncompleteCom:
        printf("The number of data received is not as expected. (9)");
            break;
    case VN_eERR_Failure:
        printf("The sensor has encountered a situation it does not know how to handle. (10)");
            break;
    case VN_eERR_UnknownCommand:
        printf("The request method is not supported by the Cirrus3D. (11)");
            break;
    case VN_eERR_InsufficientMemory:
        printf("Insufficient memory to create the requested buffer or the received cloud of points exceeds the maximum allowable size. (12)");
            break;
    case VN_eERR_Nullpointer:
        printf("A null pointer detected. (13)");
            break;
    case VN_eERR_BadHeader:
        printf("The server cannot or will not process the request due to something that is perceived to be a client error. (14)");
            break;
    case VN_eERR_FileAccessError:
        printf("Unable to access the file (15)");
            break;
    case VN_eERR_FileWriteError:
        printf("Unable to write in the file (16)");
            break;
    case VN_eERR_BadStatus:
        printf("Bad Status (17)");
            break;
    case VN_eERR_BadNetworkConfig:
        printf("Bad Network Config (18)");
            break;
    case VN_eERR_UnknownData:
        printf("The requested data is not known by the Cirrus3D (19)");
            break;
    case VN_eERR_ValueOutOfBounds:
        printf("The value of the data is out of Ranges (20)");
            break;
    case VN_eERR_UnknownDataType:
        printf("The requested data type is not known by the Cirrus3D (21)");
            break;
    case VN_eERR_IncompatibleProtocolVersion:
        printf("The SDK protocol version is incompatible with the Cirrus3D protocol version (22)");
            break;
    case VN_eERR_IncompatibleHeaderVersion:
        printf("The SDK protocol header version is incompatible with the Cirrus3D protocol header version (22)");
            break;
    case VN_eERR_IncompatibleDeviceVersion:
        printf("The SDK protocol device version is incompatible with the Cirrus3D protocol device version (22)");
    default:
        break;
    }
}
int main()
{
    VN_tERR_Code err;
    Cirrus3D myCirrus;
    VN_Point_XYZRGB* pMatrix_XYZRGB= new(VN_Point_XYZRGB[VN_cMaxCOPSize]);
    int matrixRows=0;//number of rows in case the cloud of points format is a matrix
    int matrixColumns=0;//number of columns in case the cloud of points format is a matrix
    int copSize=0;//current number of points

    err=VN_CirrusCom_Init();
    if(err!=VN_eERR_NoError)
    {
        VN_PrintError(err);
        return err;
    }

    err=myCirrus.init("192.168.5.222",
                      1);
    if(err!=VN_eERR_NoError)
    {
        VN_PrintError(err);
        return err;
    }

    err=myCirrus.getParameters();
    if(err!=VN_eERR_NoError)
    {
        VN_PrintError(err);
        return err;
    }

    // 0 <= Sensitivity <= 100
    err=myCirrus.setSensitivity(50);
    if(err!=VN_eERR_NoError)
    {
        VN_PrintError(err);
        return err;
    }

    // 0= VeryLow
    // 1= Low,
    // 2= Medium,
    // 3= High,
    // 4= Max,
    err=myCirrus.setDensity(1);
    if(err!=VN_eERR_NoError)
    {
        VN_PrintError(err);
        return err;
    }

    // 0= disabled
    // 10=  max filter
    err=myCirrus.setFalsePointsFilter(7);
    if(err!=VN_eERR_NoError)
    {
        VN_PrintError(err);
        return err;
    }

    // 0= Refuse points issued from saturated pixels
    // 1= Accept points issued from saturated pixels
    err=myCirrus.setAcceptSaturatedPixels(1);
    if(err!=VN_eERR_NoError)
    {
        VN_PrintError(err);
        return err;
    }

    // Depends on the Cirrus3D type
    err=myCirrus.setROI_CenterX_SensorFrame(0);
    if(err!=VN_eERR_NoError)
    {
        VN_PrintError(err);
        return err;
    }

    // Depends on the Cirrus3D type
    err=myCirrus.setROI_CenterY_SensorFrame(0);
    if(err!=VN_eERR_NoError)
    {
        VN_PrintError(err);
        return err;
    }

    // Depends on the Cirrus3D type
    err=myCirrus.setROI_CenterZ_SensorFrame(1250);
    if(err!=VN_eERR_NoError)
    {
        VN_PrintError(err);
        return err;
    }

    // Depends on the Cirrus3D type
    err=myCirrus.setROI_DimX_SensorFrame(800);
    if(err!=VN_eERR_NoError)
    {
        VN_PrintError(err);
        return err;
    }

    // Depends on the Cirrus3D type
    err=myCirrus.setROI_DimY_SensorFrame(600);
    if(err!=VN_eERR_NoError)
    {
        VN_PrintError(err);
        return err;
    }

    // Depends on the Cirrus3D type
    err=myCirrus.setROI_DimZ_SensorFrame(500);
    if(err!=VN_eERR_NoError)
    {
        VN_PrintError(err);
        return err;
    }

    // 0= mm
    // 1= m
    err=myCirrus.setUnitOption(1);
    if(err!=VN_eERR_NoError)
    {
        VN_PrintError(err);
        return err;
    }

    // 0= FactoryZinFrame
    // 1= SensorFrame
    // 2= FactoryZoutFrame
    err=myCirrus.setFrameOption(1);
    if(err!=VN_eERR_NoError)
    {
        VN_PrintError(err);
        return err;
    }

    err=myCirrus.capture(pMatrix_XYZRGB,
                         matrixRows,
                         matrixColumns,
                         copSize);
    if(err!=VN_eERR_NoError)
    {
        VN_PrintError(err);
        return err;
    }

    std::string fileName= "./PointCloud_config_1.bmp";
    err=VN_SaveXYZRGBCloudAsDepthMapBmp(pMatrix_XYZRGB,
                                           matrixRows,
                                           matrixColumns,
                                           fileName.c_str());
    if(err!=VN_eERR_NoError)
    {
        VN_PrintError(err);
        return err;
    }


    // 0= VeryLow
    // 1= Low,
    // 2= Medium,
    // 3= High,
    // 4= Max,
    err=myCirrus.setDensity(4);
    if(err!=VN_eERR_NoError)
    {
        VN_PrintError(err);
        return err;
    }


    err=myCirrus.capture(pMatrix_XYZRGB,
                         matrixRows,
                         matrixColumns,
                         copSize);
    if(err!=VN_eERR_NoError)
    {
        VN_PrintError(err);
        return err;
    }

    fileName= "./PointCloud_config_1.bmp";
    err=VN_SaveXYZRGBCloudAsDepthMapBmp(pMatrix_XYZRGB,
                                           matrixRows,
                                           matrixColumns,
                                           fileName.c_str());
    if(err!=VN_eERR_NoError)
    {
        VN_PrintError(err);
        return err;
    }

    err=VN_CirrusCom_End();
    if(err!=VN_eERR_NoError)
    {
        VN_PrintError(err);
        return err;
    }

    delete[] pMatrix_XYZRGB;

    return 0;
}
