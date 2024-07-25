#ifndef CIRRUS3D_H
#define CIRRUS3D_H

extern "C"
{
#include "VN_SDK_CommunicationFunctions.h"
#include "VN_SDK_CommandFunctions.h"
#include "VN_SDK_CloudSavingFunctions.h"
}


#include <string>

class Cirrus3D
{
public:
    Cirrus3D();
    VN_tERR_Code init(std::string ipAddress,
                      VN_UINT32 versionDevice=0);
    VN_tERR_Code capture(VN_Point_XYZRGB* pMatrix_XYZRGB,
                         int matrixRows,
                         int matrixColumns,
                         int copSize);

    VN_tERR_Code getParameters();

    VN_tERR_Code setSensitivity(VN_INT32 newSensitivity);
    VN_tERR_Code setDensity(VN_INT32 newDensity);
    VN_tERR_Code setFalsePointsFilter(VN_INT32 newFalsePointsFilter);
    VN_tERR_Code setAcceptSaturatedPixels(VN_INT32 newAcceptSaturatedPixels);
    VN_tERR_Code setROI_CenterX_SensorFrame(VN_REAL32 newROI_CenterX_SensorFrame);
    VN_tERR_Code setROI_CenterY_SensorFrame(VN_REAL32 newROI_CenterY_SensorFrame);
    VN_tERR_Code setROI_CenterZ_SensorFrame(VN_REAL32 newROI_CenterZ_SensorFrame);
    VN_tERR_Code setROI_DimX_SensorFrame(VN_REAL32 newROI_DimX_SensorFrame);
    VN_tERR_Code setROI_DimY_SensorFrame(VN_REAL32 newROI_DimY_SensorFrame);
    VN_tERR_Code setROI_DimZ_SensorFrame(VN_REAL32 newROI_DimZ_SensorFrame);
    VN_tERR_Code setUnitOption(VN_INT32 newUnitOption);
    VN_tERR_Code setFrameOption(VN_INT32 newFrameOption);

private:
    VN_Cirrus3DHandler m_Cirrus3DHandler;

    VN_tDataInt32 m_Sensitivity;
    VN_tDataInt32 m_Density;
    VN_tDataInt32 m_FalsePointsFilter;
    VN_tDataInt32 m_AcceptSaturatedPixels;
    VN_tDataReal32 m_ROI_CenterX_SensorFrame;
    VN_tDataReal32 m_ROI_CenterY_SensorFrame;
    VN_tDataReal32 m_ROI_CenterZ_SensorFrame;
    VN_tDataReal32 m_ROI_DimX_SensorFrame;
    VN_tDataReal32 m_ROI_DimY_SensorFrame;
    VN_tDataReal32 m_ROI_DimZ_SensorFrame;
    VN_tDataInt32 m_UnitOption;
    VN_tDataInt32 m_FrameOption;
};

#endif // CIRRUS3D_H
