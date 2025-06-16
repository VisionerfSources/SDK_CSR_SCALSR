/*------------------------------------------------------------------------------
 *  History:
 * 22/01/2022 Original version.............................................. P.H
 * 11/04/2024 NEW: new parameters ROI_*_CameraLeftFrame..................... P.H
 * 24/05/2024 NEW VN_Id_Unknown............................................. P.H
 * 16/06/2025 ADDED new parameters (device version 4):
 * 	VN_Id_AcceptPartlyHiddenPoint_Int32
 *	VN_Id_HighDynamicMode_Int32............................................. P.H
 *------------------------------------------------------------------------------
 */


#ifndef VN_CIRRUS3D_DATAID_H
#define VN_CIRRUS3D_DATAID_H

////Para
#define VN_Id_Unknown -1
#define VN_Id_Sensitivity_Int32  1
#define VN_Id_Density_Int32  2
#define VN_Id_FalsePointsFilter_Int32 3
#define VN_Id_AcceptSaturatedPixels_Int32 4
#define VN_Id_ROI_CenterX_SensorFrame_Real32 5
#define VN_Id_ROI_CenterY_SensorFrame_Real32 6
#define VN_Id_ROI_CenterZ_SensorFrame_Real32 7
#define VN_Id_ROI_DimX_SensorFrame_Real32 8
#define VN_Id_ROI_DimY_SensorFrame_Real32 9
#define VN_Id_ROI_DimZ_SensorFrame_Real32 10
#define VN_Id_UnitOption_Int32 11
#define VN_Id_FrameOption_Int32 12
#define VN_Id_ROI_CenterX_FactoryZinFrame_Real32 13
#define VN_Id_ROI_CenterY_FactoryZinFrame_Real32 14
#define VN_Id_ROI_CenterZ_FactoryZinFrame_Real32 15
#define VN_Id_ROI_DimX_FactoryZinFrame_Real32 16
#define VN_Id_ROI_DimY_FactoryZinFrame_Real32 17
#define VN_Id_ROI_DimZ_FactoryZinFrame_Real32 18
#define VN_Id_ROI_CenterX_FactoryZoutFrame_Real32 19
#define VN_Id_ROI_CenterY_FactoryZoutFrame_Real32 20
#define VN_Id_ROI_CenterZ_FactoryZoutFrame_Real32 21
#define VN_Id_ROI_DimX_FactoryZoutFrame_Real32 22
#define VN_Id_ROI_DimY_FactoryZoutFrame_Real32 23
#define VN_Id_ROI_DimZ_FactoryZoutFrame_Real32 24
#define VN_Id_ROI_CenterX_CameraLeftFrame_Real32 25
#define VN_Id_ROI_CenterY_CameraLeftFrame_Real32 26
#define VN_Id_ROI_CenterZ_CameraLeftFrame_Real32 27
#define VN_Id_ROI_DimX_CameraLeftFrame_Real32 28
#define VN_Id_ROI_DimY_CameraLeftFrame_Real32 29
#define VN_Id_ROI_DimZ_CameraLeftFrame_Real32 30
#define VN_Id_AcceptPartlyHiddenPoint_Int32 31
#define VN_Id_HighDynamicMode_Int32 56
#define VN_IDMax_Data 57

#define VN_Sensitivity_ReadOnly  0
#define VN_Density_ReadOnly  0
#define VN_FalsePointsFilter_ReadOnly  0
#define VN_AcceptSaturatedPixels_ReadOnly  0
#define VN_ROI_CenterX_SensorFrame_ReadOnly  0
#define VN_ROI_CenterY_SensorFrame_ReadOnly  0
#define VN_ROI_CenterZ_SensorFrame_ReadOnly  0
#define VN_ROI_DimX_SensorFrame_ReadOnly  0
#define VN_ROI_DimY_SensorFrame_ReadOnly  0
#define VN_ROI_DimZ_SensorFrame_ReadOnly  0
#define VN_UnitOption_ReadOnly  0
#define VN_FrameOption_ReadOnly  0
#define VN_ROI_CenterX_FactoryZinFrame_ReadOnly  0
#define VN_ROI_CenterY_FactoryZinFrame_ReadOnly  0
#define VN_ROI_CenterZ_FactoryZinFrame_ReadOnly  0
#define VN_ROI_DimX_FactoryZinFrame_ReadOnly  0
#define VN_ROI_DimY_FactoryZinFrame_ReadOnly  0
#define VN_ROI_DimZ_FactoryZinFrame_ReadOnly  0
#define VN_ROI_CenterX_FactoryZoutFrame_ReadOnly  0
#define VN_ROI_CenterY_FactoryZoutFrame_ReadOnly  0
#define VN_ROI_CenterZ_FactoryZoutFrame_ReadOnly  0
#define VN_ROI_DimX_FactoryZoutFrame_ReadOnly  0
#define VN_ROI_DimY_FactoryZoutFrame_ReadOnly  0
#define VN_ROI_DimZ_FactoryZoutFrame_ReadOnly  0
#define VN_ROI_CenterX_CameraLeftFrame_ReadOnly  0
#define VN_ROI_CenterY_CameraLeftFrame_ReadOnly  0
#define VN_ROI_CenterZ_CameraLeftFrame_ReadOnly  0
#define VN_ROI_DimX_CameraLeftFrame_ReadOnly  0
#define VN_ROI_DimY_CameraLeftFrame_ReadOnly  0
#define VN_ROI_DimZ_CameraLeftFrame_ReadOnly  0
#define VN_AcceptPartlyHiddenPoint_ReadOnly 0
#define VN_HighDynamicMode_ReadOnly 0


#define VN_Sensitivity_isSupported 0
#define VN_Density_isSupported 0
#define VN_FalsePointsFilter_isSupported 0
#define VN_AcceptSaturatedPixels_isSupported 0
#define VN_ROI_CenterX_SensorFrame_isSupported 0
#define VN_ROI_CenterY_SensorFrame_isSupported 0
#define VN_ROI_CenterZ_SensorFrame_isSupported 0
#define VN_ROI_DimX_SensorFrame_isSupported 0
#define VN_ROI_DimY_SensorFrame_isSupported 0
#define VN_ROI_DimZ_SensorFrame_isSupported 0
#define VN_UnitOption_isSupported 0
#define VN_FrameOption_isSupported 0
#define VN_ROI_CenterX_FactoryZinFrame_isSupported 0
#define VN_ROI_CenterY_FactoryZinFrame_isSupported 0
#define VN_ROI_CenterZ_FactoryZinFrame_isSupported 0
#define VN_ROI_DimX_FactoryZinFrame_isSupported 0
#define VN_ROI_DimY_FactoryZinFrame_isSupported 0
#define VN_ROI_DimZ_FactoryZinFrame_isSupported 0
#define VN_ROI_CenterX_FactoryZoutFrame_isSupported 0
#define VN_ROI_CenterY_FactoryZoutFrame_isSupported 0
#define VN_ROI_CenterZ_FactoryZoutFrame_isSupported 0
#define VN_ROI_DimX_FactoryZoutFrame_isSupported 0
#define VN_ROI_DimY_FactoryZoutFrame_isSupported 0
#define VN_ROI_DimZ_FactoryZoutFrame_isSupported 0
#define VN_ROI_CenterX_CameraLeftFrame_isSupported 0
#define VN_ROI_CenterY_CameraLeftFrame_isSupported 0
#define VN_ROI_CenterZ_CameraLeftFrame_isSupported 0
#define VN_ROI_DimX_CameraLeftFrame_isSupported 0
#define VN_ROI_DimY_CameraLeftFrame_isSupported 0
#define VN_ROI_DimZ_CameraLeftFrame_isSupported 0
#define VN_AcceptPartlyHiddenPoint_isSupported 0
#define VN_HighDynamicMode_isSupported 0

#define VN_Sensitivity_Name "Sensitivity"
#define VN_Density_Name "Density"
#define VN_FalsePointsFilter_Name "FalsePointsFilter"
#define VN_AcceptSaturatedPixels_Name "AcceptSaturatedPixels"
#define VN_ROI_CenterX_SensorFrame_Name "ROI_CenterX_SensorFrame"
#define VN_ROI_CenterY_SensorFrame_Name "ROI_CenterY_SensorFrame"
#define VN_ROI_CenterZ_SensorFrame_Name "ROI_CenterZ_SensorFrame"
#define VN_ROI_DimX_SensorFrame_Name "ROI_DimX_SensorFrame"
#define VN_ROI_DimY_SensorFrame_Name "ROI_DimY_SensorFrame"
#define VN_ROI_DimZ_SensorFrame_Name "ROI_DimZ_SensorFrame"
#define VN_ROI_CenterX_FactoryZinFrame_Name "ROI_CenterX_FactoryZinFrame"
#define VN_ROI_CenterY_FactoryZinFrame_Name "ROI_CenterY_FactoryZinFrame"
#define VN_ROI_CenterZ_FactoryZinFrame_Name "ROI_CenterZ_FactoryZinFrame"
#define VN_ROI_DimX_FactoryZinFrame_Name "ROI_DimX_FactoryZinFrame"
#define VN_ROI_DimY_FactoryZinFrame_Name "ROI_DimY_FactoryZinFrame"
#define VN_ROI_DimZ_FactoryZinFrame_Name "ROI_DimZ_FactoryZinFrame"
#define VN_ROI_CenterX_FactoryZoutFrame_Name "ROI_CenterX_FactoryZoutFrame"
#define VN_ROI_CenterY_FactoryZoutFrame_Name "ROI_CenterY_FactoryZoutFrame"
#define VN_ROI_CenterZ_FactoryZoutFrame_Name "ROI_CenterZ_FactoryZoutFrame"
#define VN_ROI_DimX_FactoryZoutFrame_Name "ROI_DimX_FactoryZoutFrame"
#define VN_ROI_DimY_FactoryZoutFrame_Name "ROI_DimY_FactoryZoutFrame"
#define VN_ROI_DimZ_FactoryZoutFrame_Name "ROI_DimZ_FactoryZoutFrame"
#define VN_ROI_CenterX_CameraLeftFrame_Name "ROI_CenterX_CameraLeft"
#define VN_ROI_CenterY_CameraLeftFrame_Name "ROI_CenterY_CameraLeft"
#define VN_ROI_CenterZ_CameraLeftFrame_Name "ROI_CenterZ_CameraLeft"
#define VN_ROI_DimX_CameraLeftFrame_Name "ROI_DimX_CameraLeft"
#define VN_ROI_DimY_CameraLeftFrame_Name "ROI_DimY_CameraLeft"
#define VN_ROI_DimZ_CameraLeftFrame_Name "ROI_DimZ_CameraLeft"
#define VN_UnitOption_Name "UnitOption"
#define VN_FrameOption_Name "FrameOption"
#define VN_AcceptPartlyHiddenPoint_Name "AcceptPartlyHiddenPoint"
#define VN_HighDynamicMode_Name "HighDynamicMode"

#define VN_Sensitivity_Description "Sensitivity parameter to adapt the sensor to the current scene."
#define VN_Density_Description "Density parameter to manage the density of the cloud of points"
#define VN_FalsePointsFilter_Description "FalsePointsFilter parameter manages the behaviour of several filters applied to the cloud of points that are used to eliminate wrong 3d points."
#define VN_AcceptSaturatedPixels_Description "AcceptSaturatedPixels parameter controls whether or not pixels are considered saturated."
#define VN_ROI_CenterX_SensorFrame_Description "X coordinate of ROI's center in SensorFrame"
#define VN_ROI_CenterY_SensorFrame_Description "Y coordinate of ROI's center in SensorFrame"
#define VN_ROI_CenterZ_SensorFrame_Description "Z coordinate of ROI's center in SensorFrame"
#define VN_ROI_DimX_SensorFrame_Description "Dimension according to the X axis in SensorFrame"
#define VN_ROI_DimY_SensorFrame_Description "Dimension according to the Y axis in SensorFrame"
#define VN_ROI_DimZ_SensorFrame_Description "Dimension according to the Z axis in SensorFrame"
#define VN_ROI_CenterX_FactoryZinFrame_Description "X coordinate of ROI's center in FactoryZinFrame"
#define VN_ROI_CenterY_FactoryZinFrame_Description "Y coordinate of ROI's center in FactoryZinFrame"
#define VN_ROI_CenterZ_FactoryZinFrame_Description "Z coordinate of ROI's center in FactoryZinFrame"
#define VN_ROI_DimX_FactoryZinFrame_Description "Dimension according to the X axis in FactoryZinFrame"
#define VN_ROI_DimY_FactoryZinFrame_Description "Dimension according to the Y axis in FactoryZinFrame"
#define VN_ROI_DimZ_FactoryZinFrame_Description "Dimension according to the Z axis in FactoryZinFrame"
#define VN_ROI_CenterX_FactoryZoutFrame_Description "X coordinate of ROI's center in FactoryZoutFrame"
#define VN_ROI_CenterY_FactoryZoutFrame_Description "Y coordinate of ROI's center in FactoryZoutFrame"
#define VN_ROI_CenterZ_FactoryZoutFrame_Description "Z coordinate of ROI's center in FactoryZoutFrame"
#define VN_ROI_DimX_FactoryZoutFrame_Description "Dimension according to the X axis in FactoryZoutFrame"
#define VN_ROI_DimY_FactoryZoutFrame_Description "Dimension according to the Y axis in FactoryZoutFrame"
#define VN_ROI_DimZ_FactoryZoutFrame_Description "Dimension according to the Z axis in FactoryZoutFrame"
#define VN_ROI_CenterX_CameraLeftFrame_Description "X coordinate of ROI's center in CameraLeft"
#define VN_ROI_CenterY_CameraLeftFrame_Description "Y coordinate of ROI's center in CameraLeft"
#define VN_ROI_CenterZ_CameraLeftFrame_Description "Z coordinate of ROI's center in CameraLeft"
#define VN_ROI_DimX_CameraLeftFrame_Description "Dimension according to the X axis in CameraLeft"
#define VN_ROI_DimY_CameraLeftFrame_Description "Dimension according to the Y axis in CameraLeft"
#define VN_ROI_DimZ_CameraLeftFrame_Description "Dimension according to the Z axis in CameraLeft"
#define VN_UnitOption_Description "UnitOption parameter to manage 3D coordinates unit (mm or m)"
#define VN_FrameOption_Description "FrameOption parameter to manage cloud of points coordinate system"
#define VN_AcceptPartlyHiddenPoint_Description "Accept 3d points generated using data from a single camera in addition to 3d points using data from both cameras. Can provide a more complete point coverage, but the added points will be less accurate"
#define VN_HighDynamicMode_Description "This parameter configures the optical dynamic range of the scan. In practice, increasing this parameters allows for scanning brighter/more reflective objects, at the cost of an increased scan time. It is recommended to increase this parameter until none of the objects under observataion in the scene are saturated."

#define VN_Sensitivity_ToolTip "This parameter to adapt the sensor to the current scene."
#define VN_Density_ToolTip "This parameter to manage the density of the cloud of points"
#define VN_FalsePointsFilter_ToolTip "This parameter to manage the behaviour of several filters applied to the cloud of points that are used to eliminate wrong 3d points."
#define VN_AcceptSaturatedPixels_ToolTip "This parameter to manage whether or not pixels are considered saturated."
#define VN_ROI_CenterX_SensorFrame_ToolTip "X coordinate of ROI's center in SensorFrame"
#define VN_ROI_CenterY_SensorFrame_ToolTip "Y coordinate of ROI's center in SensorFrame"
#define VN_ROI_CenterZ_SensorFrame_ToolTip "Z coordinate of ROI's center in SensorFrame"
#define VN_ROI_DimX_SensorFrame_ToolTip "Dimension according to the X axis in SensorFrame"
#define VN_ROI_DimY_SensorFrame_ToolTip "Dimension according to the Y axis in SensorFrame"
#define VN_ROI_DimZ_SensorFrame_ToolTip "Dimension according to the Z axis in SensorFrame"
#define VN_ROI_CenterX_FactoryZinFrame_ToolTip "X coordinate of ROI's center in FactoryZinFrame"
#define VN_ROI_CenterY_FactoryZinFrame_ToolTip "Y coordinate of ROI's center in FactoryZinFrame"
#define VN_ROI_CenterZ_FactoryZinFrame_ToolTip "Z coordinate of ROI's center in FactoryZinFrame"
#define VN_ROI_DimX_FactoryZinFrame_ToolTip "Dimension according to the X axis in FactoryZinFrame"
#define VN_ROI_DimY_FactoryZinFrame_ToolTip "Dimension according to the Y axis in FactoryZinFrame"
#define VN_ROI_DimZ_FactoryZinFrame_ToolTip "Dimension according to the Z axis in FactoryZinFrame"
#define VN_ROI_CenterX_FactoryZoutFrame_ToolTip "X coordinate of ROI's center in FactoryZoutFrame"
#define VN_ROI_CenterY_FactoryZoutFrame_ToolTip "Y coordinate of ROI's center in FactoryZoutFrame"
#define VN_ROI_CenterZ_FactoryZoutFrame_ToolTip "Z coordinate of ROI's center in FactoryZoutFrame"
#define VN_ROI_DimX_FactoryZoutFrame_ToolTip "Dimension according to the X axis in FactoryZoutFrame"
#define VN_ROI_DimY_FactoryZoutFrame_ToolTip "Dimension according to the Y axis in FactoryZoutFrame"
#define VN_ROI_DimZ_FactoryZoutFrame_ToolTip "Dimension according to the Z axis in FactoryZoutFrame"
#define VN_ROI_CenterX_CameraLeftFrame_ToolTip "X coordinate of ROI's center in CameraLeft"
#define VN_ROI_CenterY_CameraLeftFrame_ToolTip "Y coordinate of ROI's center in CameraLeft"
#define VN_ROI_CenterZ_CameraLeftFrame_ToolTip "Z coordinate of ROI's center in CameraLeft"
#define VN_ROI_DimX_CameraLeftFrame_ToolTip "Dimension according to the X axis in CameraLeft"
#define VN_ROI_DimY_CameraLeftFrame_ToolTip "Dimension according to the Y axis in CameraLeft"
#define VN_ROI_DimZ_CameraLeftFrame_ToolTip "Dimension according to the Z axis in CameraLeft"
#define VN_UnitOption_ToolTip "3D coordinate unit (mm or m)"
#define VN_FrameOption_ToolTip "Cloud of points coordinate system"
#define VN_AcceptPartlyHiddenPoint_ToolTip "Accept 3d points generated using data from a single camera in addition to 3d points using data from both cameras"
#define VN_HighDynamicMode_ToolTip "Configures the optical dynamic range of the scan"

#define VN_Sensitivity_Units " "
#define VN_Density_Units " "
#define VN_FalsePointsFilter_Units " "
#define VN_AcceptSaturatedPixels_Units " "
#define VN_ROI_CenterX_SensorFrame_Units "mm"
#define VN_ROI_CenterY_SensorFrame_Units "mm"
#define VN_ROI_CenterZ_SensorFrame_Units "mm"
#define VN_ROI_DimX_SensorFrame_Units "mm"
#define VN_ROI_DimY_SensorFrame_Units "mm"
#define VN_ROI_DimZ_SensorFrame_Units "mm"
#define VN_ROI_CenterX_FactoryZinFrame_Units "mm"
#define VN_ROI_CenterY_FactoryZinFrame_Units "mm"
#define VN_ROI_CenterZ_FactoryZinFrame_Units "mm"
#define VN_ROI_DimX_FactoryZinFrame_Units "mm"
#define VN_ROI_DimY_FactoryZinFrame_Units "mm"
#define VN_ROI_DimZ_FactoryZinFrame_Units "mm"
#define VN_ROI_CenterX_FactoryZoutFrame_Units "mm"
#define VN_ROI_CenterY_FactoryZoutFrame_Units "mm"
#define VN_ROI_CenterZ_FactoryZoutFrame_Units "mm"
#define VN_ROI_DimX_FactoryZoutFrame_Units "mm"
#define VN_ROI_DimY_FactoryZoutFrame_Units "mm"
#define VN_ROI_DimZ_FactoryZoutFrame_Units "mm"
#define VN_ROI_CenterX_CameraLeftFrame_Units "mm"
#define VN_ROI_CenterY_CameraLeftFrame_Units "mm"
#define VN_ROI_CenterZ_CameraLeftFrame_Units "mm"
#define VN_ROI_DimX_CameraLeftFrame_Units "mm"
#define VN_ROI_DimY_CameraLeftFrame_Units "mm"
#define VN_ROI_DimZ_CameraLeftFrame_Units "mm"
#define VN_UnitOption_Units " "
#define VN_FrameOption_Units " "
#define VN_AcceptPartlyHiddenPoint_Units " "
#define VN_HighDynamicMode_Units ""

#endif // VN_CIRRUS3D_DATAID_H
