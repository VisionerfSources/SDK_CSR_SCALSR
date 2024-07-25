/*******************************************************************
 *  This document contains a code example for using the functions
 *   provided by Visionerf to communicate with a Cirrus3D.
 *  All the code is written in C language for maximal compatibility,
 *  and is designed to be compatible with Linux and Windows
 *******************************************************************
 *  History:
 * 12/05/2023  Original version................................ P.H
 * 20/06/2023  Completed version............................... F.R
 * 04/07/2023  CLEAN : replaced the remaining pError by printf. F.R
 * 16/02/2023  NEW new example: exampleWithTwoSensors.......... P.H
 *******************************************************************
 *
 * This document is the property of Visionerf and may not be reproduced or communicated without written permission
 *
 */
#include "VN_Cirrus3D.API.h"

#include "VN_Cirrus3D_DataId.h"

#include "stdio.h" //for NULL

/********************************** USER DATA **********************************
 * Data to be modified by the user
 *******************************************************************************/
#define ipAddressCirrus1 "192.168.5.222" //IP address of the Cirrus3DSensor
#define ipAddressCirrus2 "192.168.5.222" //IP address of the Cirrus3DSensor

VN_INT32 exampleSimple(VN_Point_XYZI8 *pMatrix_XYZRGB);
VN_INT32 exampleWithTwoSensors(VN_Point_XYZI8 *pMatrix_XYZI8);


VN_INT32 main()
{
    VN_tERR_Code err = VN_eERR_NoError;
    int nbCirrus3DAvailable=0;
    char ipCirrus3DAvailable[VN_cMaxNbCirrus3D][VN_cSizeDataString];

    //WARNING: Remember to free the buffer in your code
    VN_Point_XYZI8* pMatrix_XYZI8= (VN_Point_XYZI8*)malloc(sizeof(VN_Point_XYZI8)*VN_cMaxCOPSize);

    // Enable connection protocol for Windows OS.
    // Call this function once before using the functions available in the SDK
    err = VN_CirrusCom_Init();
    if (err!=VN_eERR_NoError) return err;

    err=VN_GetCirrus3DAvailable(&nbCirrus3DAvailable, ipCirrus3DAvailable);
    for(int i=0; i<nbCirrus3DAvailable; i++)
        printf("Product detected with IP address: %s\n", ipCirrus3DAvailable[i]);
    fflush(stdout);

    err=exampleSimple(pMatrix_XYZI8);
    if (err!=VN_eERR_NoError) return err;

    err=exampleWithTwoSensors(pMatrix_XYZI8);
    if (err!=VN_eERR_NoError) return err;

    // Cleanly end connection protocol for Windows OS.
    err = VN_CirrusCom_End();
    if (err!=VN_eERR_NoError) return err;
}

VN_INT32 exampleSimple(VN_Point_XYZI8* pMatrix_XYZI8)
{
    VN_tERR_Code err = VN_eERR_NoError;
    VN_Cirrus3DHandler sensor1;
    VN_tDataInt32 sensor1_FrameOption;
    int copSize=0;//current number of points
    int matrixRows=0;//number of rows in case the cloud of points format is a matrix
    int matrixColumns=0;//number of columns in case the cloud of points format is a matrix

    //**** Test if the Cirrus3D is available ****//
    int status=0;
    err = VN_ExecuteCommandSTS(ipAddressCirrus1, &status);
    if (err!=VN_eERR_NoError || status!=0)
    {
        printf("\nCirrus3D at address %s is NOT ready\n\n",ipAddressCirrus1);
        return err;
    }
    else
        printf("\nCirrus3D at address %s is ready\n\n",ipAddressCirrus1);

    //****  Initialize the VN_Cirrus3DHandler structure used by the get/set data functions ****//
    err=VN_Cirrus3DHandler_Initialize(ipAddressCirrus1,
                                        &sensor1);
    if (err!=VN_eERR_NoError)
    {
        printf("\nVN_Cirrus3DHandler_Initialize function return error code = %d \n\n", err);

        if (err==VN_eERR_IncompatibleProtocolVersion) {
            printf("ERROR incompatible protocol version. Camera protocol is %d and SDK version is %d", sensor1.ProtocolVersion, VN_Cirrus3DHandler_getProtocolVersion());
        }
        return err;
    }

    //**** Get data structure of FrameOption ****//
    {
        //Get FrameOption structure
        err=VN_Cirrus3DHandler_GetDataInt32(&sensor1,
                                            VN_Id_FrameOption_Int32,
                                            &sensor1_FrameOption);
        if (err!=VN_eERR_NoError)
        {
            printf("\nIn get FrameOption structure, VN_Cirrus3DHandler_GetDataInt32 function return error code = %d \n\n", err);
            return err;
        }


        printf("Name %s - ",sensor1_FrameOption.dataInfo.name);
        printf("Value %d\n",sensor1_FrameOption.value);
    }

    //**** Set FrameOptiony ****//
    {
        //Set FrameOption value
        sensor1_FrameOption.value=1; //SensorFrame
        err=VN_Cirrus3DHandler_SetDataInt32Value(&sensor1,
                                                 &sensor1_FrameOption);
        if (err!=VN_eERR_NoError)
        {
            printf("\nIn set FrameOption value, VN_Cirrus3DHandler_SetDataInt32Value function return error code = %d \n\n", err);
            return err;
        }

        //Get FrameOption value
        err=VN_Cirrus3DHandler_GetDataInt32Value(&sensor1,
                                                   &sensor1_FrameOption);
        if (err!=VN_eERR_NoError)
        {
            printf("\nIn get FrameOption value, VN_Cirrus3DHandler_GetDataInt32Value function return error code = %d \n\n", err);
            return err;
        }

        printf("Name %s - ",sensor1_FrameOption.dataInfo.name);
        printf("Value %d\n",sensor1_FrameOption.value);
    }

    //**** Execute a scan with the currently configuration, where the expected resulting ****//
    err = VN_ExecuteSCAN_CameraCOP_XYZI8(ipAddressCirrus1,
                                       VN_cMaxCOPSize,
                                       pMatrix_XYZI8,
                                       &copSize,
                                       &matrixColumns,
                                       &matrixRows,
                                       (VN_BOOL)0);
    if (err!=VN_eERR_NoError)
        printf("Cirrus scan FAILED\n");
    else
        printf("Cirrus scan sucessfull with %d points\n",copSize);
    //**** Save the resulting cloud of points as a depthmap bmp image ****//
    err = VN_SaveXYZI8CloudAsDepthMapBmp(pMatrix_XYZI8,
                                  matrixRows,
                                  matrixColumns,
                                  "./PointCloud_1.bmp");
    if (err!=VN_eERR_NoError)
        printf("Cirrus scan image save FAILED in file %s\n","./PointCloud_config1.bmp");
    else
        printf("Cirrus scan image saved in file %s\n\n","./PointCloud_config1.bmp");


    //**** Execute a scan with the currently configuration, where the expected resulting ****//
    err = VN_ExecuteSCAN_CameraCOP_XYZI8(ipAddressCirrus1,
                                       VN_cMaxCOPSize,
                                       pMatrix_XYZI8,
                                       &copSize,
                                       &matrixColumns,
                                       &matrixRows,
                                       (VN_BOOL)1);
    if (err!=VN_eERR_NoError)
        printf("Cirrus scan FAILED\n");
    else
        printf("Cirrus scan sucessfull with %d points\n",copSize);
    //**** Save the resulting cloud of points as a depthmap bmp image ****//
    err = VN_SaveXYZI8CloudAsDepthMapBmp(pMatrix_XYZI8,
                                  matrixRows,
                                  matrixColumns,
                                  "./PointCloud_2.bmp");
    if (err!=VN_eERR_NoError)
        printf("Cirrus scan image save FAILED in file %s\n","./PointCloud_config1.bmp");
    else
        printf("Cirrus scan image saved in file %s\n\n","./PointCloud_config1.bmp");


    //**** Execute a scan with the currently configuration, where the expected resulting ****//
    err = VN_ExecuteSCAN_CameraCOP_XYZI8_sampling(ipAddressCirrus1,
                                           VN_cMaxCOPSize,
                                           pMatrix_XYZI8,
                                           &copSize,
                                           &matrixColumns,
                                           &matrixRows,
                                           (VN_BOOL)0,
                                                 5);
    if (err!=VN_eERR_NoError)
        printf("Cirrus scan FAILED\n");
    else
        printf("Cirrus scan sucessfull with %d points\n",copSize);
    //**** Save the resulting cloud of points as a depthmap bmp image ****//
    err = VN_SaveXYZI8CloudAsDepthMapBmp(pMatrix_XYZI8,
                                  matrixRows,
                                  matrixColumns,
                                  "./PointCloud_3.bmp");
    if (err!=VN_eERR_NoError)
        printf("Cirrus scan image save FAILED in file %s\n","./PointCloud_config1.bmp");
    else
        printf("Cirrus scan image saved in file %s\n\n","./PointCloud_config1.bmp");

    return err;
}

VN_INT32 exampleWithTwoSensors(VN_Point_XYZI8* pMatrix_XYZI8)
{
    VN_tERR_Code err = VN_eERR_NoError;
    VN_Cirrus3DHandler sensor1;
    VN_tDataInt32 sensor1_FrameOption;
    VN_Cirrus3DHandler sensor2;
    VN_tDataInt32 sensor2_FrameOption;
    int copSize=0;//current number of points
    int matrixRows=0;//number of rows in case the cloud of points format is a matrix
    int matrixColumns=0;//number of columns in case the cloud of points format is a matrix

    //**** Test if the Cirrus3D is available ****//
    int status=0;
    err = VN_ExecuteCommandSTS(ipAddressCirrus1, &status);
    if (err!=VN_eERR_NoError || status!=0)
    {
        printf("\nCirrus3D at address %s is NOT ready\n\n",ipAddressCirrus1);
        return err;
    }
    else
        printf("\nCirrus3D at address %s is ready\n\n",ipAddressCirrus1);

    err = VN_ExecuteCommandSTS(ipAddressCirrus2, &status);
    if (err!=VN_eERR_NoError || status!=0)
    {
        printf("\nCirrus3D at address %s is NOT ready\n\n",ipAddressCirrus2);
        return err;
    }
    else
        printf("\nCirrus3D at address %s is ready\n\n",ipAddressCirrus2);

    //****  Initialize the VN_Cirrus3DHandler structure used by the get/set data functions ****//
    err=VN_Cirrus3DHandler_Initialize(ipAddressCirrus1,
                                        &sensor1);
    if (err!=VN_eERR_NoError)
    {
        printf("\nVN_Cirrus3DHandler_Initialize function return error code = %d \n\n", err);
        return err;
    }

    //****  Initialize the VN_Cirrus3DHandler structure used by the get/set data functions ****//
    err=VN_Cirrus3DHandler_Initialize(ipAddressCirrus2,
                                        &sensor2);
    if (err!=VN_eERR_NoError)
    {
        printf("\nVN_Cirrus3DHandler_Initialize function return error code = %d \n\n", err);
        return err;
    }

    //**** Get data structure of FrameOption ****//
    {
        //Get FrameOption structure
        err=VN_Cirrus3DHandler_GetDataInt32(&sensor1,
                                            VN_Id_FrameOption_Int32,
                                            &sensor1_FrameOption);
        if (err!=VN_eERR_NoError)
        {
            printf("\nIn get FrameOption structure, VN_Cirrus3DHandler_GetDataInt32 function return error code = %d \n\n", err);
            return err;
        }


        printf("Name %s - ",sensor1_FrameOption.dataInfo.name);
        printf("Value %d\n",sensor1_FrameOption.value);
    }

    //**** Get data structure of FrameOption ****//
    {
        //Get FrameOption structure
        err=VN_Cirrus3DHandler_GetDataInt32(&sensor2,
                                            VN_Id_FrameOption_Int32,
                                            &sensor2_FrameOption);
        if (err!=VN_eERR_NoError)
        {
            printf("\nIn get FrameOption structure, VN_Cirrus3DHandler_GetDataInt32 function return error code = %d \n\n", err);
            return err;
        }


        printf("Name %s - ",sensor2_FrameOption.dataInfo.name);
        printf("Value %d\n",sensor2_FrameOption.value);
    }

    //**** Set FrameOptiony ****//
    {
        //Set FrameOption value
        sensor1_FrameOption.value=1; //SensorFrame
        err=VN_Cirrus3DHandler_SetDataInt32Value(&sensor1,
                                                 &sensor1_FrameOption);
        if (err!=VN_eERR_NoError)
        {
            printf("\nIn set FrameOption value, VN_Cirrus3DHandler_SetDataInt32Value function return error code = %d \n\n", err);
            return err;
        }

        //Get FrameOption value
        err=VN_Cirrus3DHandler_GetDataInt32Value(&sensor1,
                                                   &sensor1_FrameOption);
        if (err!=VN_eERR_NoError)
        {
            printf("\nIn get FrameOption value, VN_Cirrus3DHandler_GetDataInt32Value function return error code = %d \n\n", err);
            return err;
        }

        printf("Name %s - ",sensor1_FrameOption.dataInfo.name);
        printf("Value %d\n",sensor1_FrameOption.value);
    }

    //**** Execute a scan with the currently configuration, where the expected resulting ****//
    err = VN_ExecuteSCAN_CameraCOP_XYZI8(ipAddressCirrus2,
                                         VN_cMaxCOPSize,
                                         pMatrix_XYZI8,
                                         &copSize,
                                         &matrixColumns,
                                         &matrixRows,
                                         (VN_BOOL)1);
    if (err!=VN_eERR_NoError)
        printf("Cirrus scan FAILED\n");
    else
        printf("Cirrus scan sucessfull with %d points\n",copSize);

    //**** Save the resulting cloud of points as a depthmap bmp image ****//
    err = VN_SaveXYZI8CloudAsDepthMapBmp(pMatrix_XYZI8,
                                         matrixRows,
                                         matrixColumns,
                                          "./PointCloud_config1.pcd");
    if (err!=VN_eERR_NoError)
        printf("Cirrus scan image save FAILED in file %s\n","./PointCloud_config1_Sensor1.pcd");
    else
        printf("Cirrus scan image saved in file %s\n\n","./PointCloud_config1_Sensor1.pcd");



    //**** Set FrameOptiony ****//
    {
        //Set FrameOption value
        sensor1_FrameOption.value=1; //SensorFrame
        err=VN_Cirrus3DHandler_SetDataInt32Value(&sensor2,
                                                 &sensor2_FrameOption);
        if (err!=VN_eERR_NoError)
        {
            printf("\nIn set FrameOption value, VN_Cirrus3DHandler_SetDataInt32Value function return error code = %d \n\n", err);
            return err;
        }

        //Get FrameOption value
        err=VN_Cirrus3DHandler_GetDataInt32Value(&sensor2,
                                                   &sensor2_FrameOption);
        if (err!=VN_eERR_NoError)
        {
            printf("\nIn get FrameOption value, VN_Cirrus3DHandler_GetDataInt32Value function return error code = %d \n\n", err);
            return err;
        }

        printf("Name %s - ",sensor2_FrameOption.dataInfo.name);
        printf("Value %d\n",sensor2_FrameOption.value);
    }

    //**** Execute a scan with the currently configuration, where the expected resulting ****//
    err = VN_ExecuteSCAN_CameraCOP_XYZI8(ipAddressCirrus2,
                                         VN_cMaxCOPSize,
                                         pMatrix_XYZI8,
                                         &copSize,
                                         &matrixColumns,
                                         &matrixRows,
                                         (VN_BOOL)1);
    if (err!=VN_eERR_NoError)
        printf("Cirrus scan FAILED\n");
    else
        printf("Cirrus scan sucessfull with %d points\n",copSize);

    //**** Save the resulting cloud of points as a depthmap bmp image ****//
    err = VN_SaveXYZI8CloudAsDepthMapBmp(pMatrix_XYZI8,
                                         matrixRows,
                                         matrixColumns,
                                         "./PointCloud_config1.pcd");
    if (err!=VN_eERR_NoError)
        printf("Cirrus scan image save FAILED in file %s\n","./PointCloud_config1_Sensor2.pcd");
    else
        printf("Cirrus scan image saved in file %s\n\n","./PointCloud_config1_Sensor2.pcd");

    return err;
}
