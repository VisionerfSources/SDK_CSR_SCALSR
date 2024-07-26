#include "Cirrus3D.h"

extern "C"
{
#include "VN_SDK_CommunicationFunctions.h"
#include "VN_SDK_CommandFunctions.h"
#include "VN_SDK_CloudSavingFunctions.h"
}
#include "../../lib/API/VN_Cirrus3D_DataId.h"

Cirrus3D::Cirrus3D()
{

}

VN_tERR_Code Cirrus3D::init(std::string ipAddress,
                            VN_UINT32 versionDevice)
{
    VN_tERR_Code err;
    //****  Initialize the VN_Cirrus3DHandler structure used by the get/set data functions ****//
    err=VN_Cirrus3DHandler_Initialize((char*)ipAddress.c_str(),
                                      &m_Cirrus3DHandler);
    if (err!=VN_eERR_NoError)
    {
        printf("\nVN_Cirrus3DHandler_Initialize function return error code = %d \n\n", err);

        if (err==VN_eERR_IncompatibleProtocolVersion) {
            printf("ERROR incompatible protocol version. Camera protocol is %d and SDK version is %d", m_Cirrus3DHandler.ProtocolVersion, VN_Cirrus3DHandler_getProtocolVersion());
        }
        return err;
    }

    if(versionDevice!=0) //if 0 use default device version available in Cirrus3D
    {
        err=VN_Cirrus3DHandler_ForceVersionDevice(&m_Cirrus3DHandler,
                                                  versionDevice);
        if (err!=VN_eERR_NoError)
        {
            printf("\nVN_Cirrus3DHandler_ForceVersionDevice function return error code = %d \n\n", err);

            if (err==VN_eERR_IncompatibleDeviceVersion) {
                printf("ERROR incompatible device version. Camera device version don't use the version device = %d", versionDevice);
            }
            return err;
        }
    }

    return err;
}
VN_tERR_Code Cirrus3D::capture(VN_Point_XYZRGB* pMatrix_XYZRGB,
                               int matrixRows,
                               int matrixColumns,
                               int copSize)
{
    //**** Execute a scan with the currently configuration, where the expected resulting ****//
    return VN_ExecuteSCAN_Matrix_XYZRGB(m_Cirrus3DHandler.IPAddress,
                                       VN_cMaxCOPSize,
                                       pMatrix_XYZRGB,
                                       &copSize,
                                       &matrixColumns,
                                       &matrixRows);

}
VN_tERR_Code Cirrus3D::getParameters()
{
    VN_tERR_Code err;

    if(m_Cirrus3DHandler.DeviceVersion==1)
    {
        //Get Sensibility data
        err=VN_Cirrus3DHandler_GetDataInt32(&m_Cirrus3DHandler,
                                            VN_Id_Sensitivity_Int32,
                                            &m_Sensitivity);
        if (err!=VN_eERR_NoError)
        {
            printf("\nVN_Cirrus3DHandler_GetDataInt32 function return error code = %d \n\n", err);
            return err;
        }

        //Get Density data
        err=VN_Cirrus3DHandler_GetDataInt32(&m_Cirrus3DHandler,
                                              VN_Id_Density_Int32,
                                              &m_Density);
        if (err!=VN_eERR_NoError)
        {
            printf("\nVN_Cirrus3DHandler_GetDataInt32 function return error code = %d \n\n", err);
            return err;
        }

        //Get FalsePointsFilter data
        err=VN_Cirrus3DHandler_GetDataInt32(&m_Cirrus3DHandler,
                                              VN_Id_FalsePointsFilter_Int32,
                                              &m_FalsePointsFilter);
        if (err!=VN_eERR_NoError)
        {
            printf("\nVN_Cirrus3DHandler_GetDataInt32 function return error code = %d \n\n", err);
            return err;
        }

        //Get AcceptSaturatedPixels data
        err=VN_Cirrus3DHandler_GetDataInt32(&m_Cirrus3DHandler,
                                              VN_Id_AcceptSaturatedPixels_Int32,
                                              &m_AcceptSaturatedPixels);
        if (err!=VN_eERR_NoError)
        {
            printf("\nVN_Cirrus3DHandler_GetDataInt32 function return error code = %d \n\n", err);
            return err;
        }

        //Get ROI_CenterX_SensorFrame data
        err=VN_Cirrus3DHandler_GetDataReal32(&m_Cirrus3DHandler,
                                              VN_Id_ROI_CenterX_SensorFrame_Real32,
                                              &m_ROI_CenterX_SensorFrame);
        if (err!=VN_eERR_NoError)
        {
            printf("\nVN_Cirrus3DHandler_GetDataInt32 function return error code = %d \n\n", err);
            return err;
        }

        //Get ROI_CenterY_SensorFrame data
        err=VN_Cirrus3DHandler_GetDataReal32(&m_Cirrus3DHandler,
                                            VN_Id_ROI_CenterY_SensorFrame_Real32,
                                            &m_ROI_CenterY_SensorFrame);
        if (err!=VN_eERR_NoError)
        {
            printf("\nVN_Cirrus3DHandler_GetDataInt32 function return error code = %d \n\n", err);
            return err;
        }

        //Get ROI_CenterZ_SensorFrame data
        err=VN_Cirrus3DHandler_GetDataReal32(&m_Cirrus3DHandler,
                                            VN_Id_ROI_CenterZ_SensorFrame_Real32,
                                            &m_ROI_CenterZ_SensorFrame);
        if (err!=VN_eERR_NoError)
        {
            printf("\nVN_Cirrus3DHandler_GetDataInt32 function return error code = %d \n\n", err);
            return err;
        }
        //Get ROI_DimX_SensorFrame data
        err=VN_Cirrus3DHandler_GetDataReal32(&m_Cirrus3DHandler,
                                              VN_Id_ROI_DimX_SensorFrame_Real32,
                                              &m_ROI_DimX_SensorFrame);
        if (err!=VN_eERR_NoError)
        {
            printf("\nVN_Cirrus3DHandler_GetDataInt32 function return error code = %d \n\n", err);
            return err;
        }
        //Get ROI_DimY_SensorFrame data
        err=VN_Cirrus3DHandler_GetDataReal32(&m_Cirrus3DHandler,
                                              VN_Id_ROI_DimY_SensorFrame_Real32,
                                              &m_ROI_DimY_SensorFrame);
        if (err!=VN_eERR_NoError)
        {
            printf("\nVN_Cirrus3DHandler_GetDataInt32 function return error code = %d \n\n", err);
            return err;
        }
        //Get ROI_DimZ_SensorFrame data
        err=VN_Cirrus3DHandler_GetDataReal32(&m_Cirrus3DHandler,
                                              VN_Id_ROI_DimZ_SensorFrame_Real32,
                                              &m_ROI_DimZ_SensorFrame);
        if (err!=VN_eERR_NoError)
        {
            printf("\nVN_Cirrus3DHandler_GetDataInt32 function return error code = %d \n\n", err);
            return err;
        }

        //Get UnitOption data
        err=VN_Cirrus3DHandler_GetDataInt32(&m_Cirrus3DHandler,
                                              VN_Id_UnitOption_Int32,
                                              &m_UnitOption);
        if (err!=VN_eERR_NoError)
        {
            printf("\nVN_Cirrus3DHandler_GetDataInt32 function return error code = %d \n\n", err);
            return err;
        }


        //Get FrameOption data
        err=VN_Cirrus3DHandler_GetDataInt32(&m_Cirrus3DHandler,
                                              VN_Id_FrameOption_Int32,
                                              &m_FrameOption);
        if (err!=VN_eERR_NoError)
        {
            printf("\nVN_Cirrus3DHandler_GetDataInt32 function return error code = %d \n\n", err);
            return err;
        }
    }
    else
    {
        printf("\nVersionDevice %d not supported in this class\n\n", m_Cirrus3DHandler.DeviceVersion);
        return VN_eERR_Failure;
    }

    return VN_eERR_NoError;
}

VN_tERR_Code Cirrus3D::setSensitivity(VN_INT32 newSensitivity)
{
    m_Sensitivity.value = newSensitivity;

    //Set sensitivity value
    return VN_Cirrus3DHandler_SetDataInt32Value(&m_Cirrus3DHandler,
                                                &m_Sensitivity);
}


VN_tERR_Code Cirrus3D::setDensity(VN_INT32 newDensity)
{
    m_Density.value = newDensity;

    //Set Density value
    return VN_Cirrus3DHandler_SetDataInt32Value(&m_Cirrus3DHandler,
                                                &m_Density);
}

VN_tERR_Code Cirrus3D::setFalsePointsFilter(VN_INT32 newFalsePointsFilter)
{
    m_FalsePointsFilter.value = newFalsePointsFilter;

    //Set FalsePointsFilter value
    return VN_Cirrus3DHandler_SetDataInt32Value(&m_Cirrus3DHandler,
                                                &m_FalsePointsFilter);
}


VN_tERR_Code Cirrus3D::setAcceptSaturatedPixels(VN_INT32 newAcceptSaturatedPixels)
{
    m_AcceptSaturatedPixels.value = newAcceptSaturatedPixels;

    //Set AcceptSaturatedPixels value
    return VN_Cirrus3DHandler_SetDataInt32Value(&m_Cirrus3DHandler,
                                                &m_AcceptSaturatedPixels);
}

VN_tERR_Code Cirrus3D::setROI_CenterX_SensorFrame(VN_REAL32 newROI_CenterX_SensorFrame)
{
    m_ROI_CenterX_SensorFrame.value = newROI_CenterX_SensorFrame;

    //Set ROI_CenterX_SensorFrame value
    return VN_Cirrus3DHandler_SetDataReal32Value(&m_Cirrus3DHandler,
                                                &m_ROI_CenterX_SensorFrame);
}

VN_tERR_Code Cirrus3D::setROI_CenterY_SensorFrame(VN_REAL32 newROI_CenterY_SensorFrame)
{
    m_ROI_CenterY_SensorFrame.value = newROI_CenterY_SensorFrame;

    //Set ROI_CenterY_SensorFrame value
    return VN_Cirrus3DHandler_SetDataReal32Value(&m_Cirrus3DHandler,
                                                &m_ROI_CenterY_SensorFrame);
}

VN_tERR_Code Cirrus3D::setROI_CenterZ_SensorFrame(VN_REAL32 newROI_CenterZ_SensorFrame)
{
    m_ROI_CenterZ_SensorFrame.value = newROI_CenterZ_SensorFrame;

    //Set ROI_CenterZ_SensorFrame value
    return VN_Cirrus3DHandler_SetDataReal32Value(&m_Cirrus3DHandler,
                                                &m_ROI_CenterZ_SensorFrame);
}

VN_tERR_Code Cirrus3D::setROI_DimX_SensorFrame(VN_REAL32 newROI_DimX_SensorFrame)
{
    m_ROI_DimX_SensorFrame.value = newROI_DimX_SensorFrame;

    //Set ROI_DimX_SensorFrame value
    return VN_Cirrus3DHandler_SetDataReal32Value(&m_Cirrus3DHandler,
                                                &m_ROI_DimX_SensorFrame);
}

VN_tERR_Code Cirrus3D::setROI_DimY_SensorFrame(VN_REAL32 newROI_DimY_SensorFrame)
{
    m_ROI_DimY_SensorFrame.value = newROI_DimY_SensorFrame;

    //Set ROI_DimY_SensorFrame value
    return VN_Cirrus3DHandler_SetDataReal32Value(&m_Cirrus3DHandler,
                                                &m_ROI_DimY_SensorFrame);
}

VN_tERR_Code Cirrus3D::setROI_DimZ_SensorFrame(VN_REAL32 newROI_DimZ_SensorFrame)
{
    m_ROI_DimZ_SensorFrame.value = newROI_DimZ_SensorFrame;

    //Set ROI_DimZ_SensorFrame value
    return VN_Cirrus3DHandler_SetDataReal32Value(&m_Cirrus3DHandler,
                                                &m_ROI_DimZ_SensorFrame);
}

VN_tERR_Code Cirrus3D::setUnitOption(VN_INT32 newUnitOption)
{
    m_UnitOption.value = newUnitOption;

    //Set UnitOption value
    return VN_Cirrus3DHandler_SetDataInt32Value(&m_Cirrus3DHandler,
                                                &m_UnitOption);
}

VN_tERR_Code Cirrus3D::setFrameOption(VN_INT32 newFrameOption)
{
    m_FrameOption.value = newFrameOption;

    //Set FrameOption value
    return VN_Cirrus3DHandler_SetDataInt32Value(&m_Cirrus3DHandler,
                                                &m_FrameOption);
}
