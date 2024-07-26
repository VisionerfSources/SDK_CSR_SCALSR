#ifndef COMMANDFUNCTIONS_H
#define COMMANDFUNCTIONS_H

/*------------------------------------------------------------------------------
 *  History:
 * 16/02/2024: NEW: new function VN_Cirrus3DHandler_ResetData............... P.H
 * 16/02/2024: NEW: new function VN_GetCirrus3DAvailable.................... P.H
 * 11/04/2024 NEW: new function VN_ExecuteSCAN_CameraCOP_XYZRGB............. P.H
 * 11/04/2024 NEW: new function VN_Cirrus3DHandler_Get...................... P.H
 * 11/04/2024 CHANGED VN_ExecuteSCAN_CameraCOP_XYZRGB added param MiddleCam. F.R
 * 16/05/2024 NEW: VN_ExecuteSCAN_readHeader
 *                 VN_ExecuteSCAN_CameraCOP_XYZI8
 *                 VN_ExecuteSCAN_CameraCOP_XYZI8_sampling
 *                 VN_ExecuteSCAN_Matrix_XYZRGB_readHeader
 *            REMOVE: VN_ExecuteSCAN_CameraCOP_XYZRGB....................... P.H
 * 17/05/2024 NEW function VN_ExecuteSCAN_CameraCOP_MiddleCam2_XYZI......... F.R
 * 24/05/2024 NEW: VN_ExecuteSCAN_CameraCOP_XYZI8_s
 *                 VN_ExecuteSCAN_CameraCOP_XYZI8_sampling_s................ P.H
 * 07/06/2024 COMMENT: Adding comment in Header function.................... P.H
 * 24/07/2024 NEW VN_ExecuteSCAN_DepthMap function.......................... P.H
 *------------------------------------------------------------------------------
 */

#include "VN_SDK_Constant.h"
/***************************** API FUNCTIONS ******************************/
/** @ingroup CloudOfPoint
 * \brief Get status of Cirrus3D (ready or not)
 * 
 * Function available for CirrusSensor version 1.0.0 and higher
 * \param[in] IPAddress : IP address of the Cirrus3D
 * \param[out] pStatus  : Status of Cirrus3D (= 0 if ready)
 * \retval VN_eERR_NoError if success (i.e. Cirrus ready)
 * \retval Error code otherwise
 */
VN_tERR_Code VN_ExecuteCommandSTS(const char *pIpAddress,
                                  int *pStatus);

/** @ingroup CloudOfPoint
 * \brief Request its information from the Cirrus3D
 *
 * Function available for CirrusSensor version 1.0.0 and higher
 * \param[in] IPAddress: IP address of the Cirrus3D
 * \param[in] InfoType: 1 to request Cirrus3D type/size, 2 to request Cirrus3D serial number
 * \param[out] pResult: Char array of size at least 64 to store the requested info
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise

 */
VN_tERR_Code VN_ExecuteCommandINFO(const char *IPAddress,
                                   int InfoType,
                                   char *pResult);


/** @ingroup CloudOfPoint
 * \brief Command the Cirrus3D to load the specified scan configuration
 *
 * Function available for CirrusSensor version 1.0.0 and higher
 * \param[in] IPAddress : IP address of the Cirrus3D
 * \param[in] ConfigId  : ID of the config to load
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_ExecuteCommandRECI(const char *IPAddress,
                                   int ConfigId);



/** @ingroup CloudOfPoint
 * \brief Read header receive on the socket sock after a command SCAN
 * \param[in] sock : socket used to send the SCAN command
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_ExecuteSCAN_readHeader(VN_SOCKET sock);

/** @ingroup CloudOfPoint
 * \brief Request the Cirrus3D to scan and get the scan result as a XYZ cloud
 *
 * Function available for CirrusSensor version 1.0.0 and higher
 * \param[in] IPAddress     : IP address of the Cirrus3D
 * \param[in] MaxCloudSize  : Max number of points that can be stored in the cloud of points buffer
 * \param[out] pCloud_XYZ   : Pointer to a buffer able to store the resulting cloud of points
 * \param[out] pCloudSize   : Will contain the number of points in the cloud
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise

 */
VN_tERR_Code VN_ExecuteSCAN_XYZ(const char *IPAddress, const int MaxCloudSize, char *pCloud_XYZ, int *pCloudSize);

/** @ingroup CloudOfPoint
 * \brief Request the Cirrus3D to scan and get the scan result as a XYZI cloud
 *
 * Function available for CirrusSensor version 1.0.0 and higher
 * \param[in] IPAddress     : IP address of the Cirrus3D
 * \param[in] MaxCloudSize  : Max number of points that can be stored in the cloud of points buffer
 * \param[out] pCloud_XYZI  : Pointer to a buffer able to store the resulting cloud of points
 * \param[out] pCloudSize   : Will contain the number of points in the cloud
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise

 */
VN_tERR_Code VN_ExecuteSCAN_XYZI16(const char *IPAddress, const int MaxCloudSize, VN_Point_XYZI16 *pCloud_XYZI16, int *pCloudSize);

/** @ingroup CloudOfPoint
 * \brief Request the Cirrus3D to scan and get the scan result as matrix of XYZi points
 *
 * Function available for CirrusSensor version 1.0.0 and higher
 * \param[in] IPAddress         : IP address of the Cirrus3D
 * \param[in] MaxCloudSize      : Max number of points that can be stored in the cloud of points buffer
 * \param[out] pCloud_XYZi      : Pointer to a buffer able to store the resulting cloud of points
 * \param[out] pCloudSize       : Will contain the number of non-zero points in the matrix
 * \param[out] pMatrixColumns   : Will contain the number of columns in the matrix
 * \param[out] pMatrixRows      : Will contain the number of rows in the matrix
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise

 */
VN_tERR_Code VN_ExecuteSCAN_Matrix_XYZI8(const char *IPAddress, const int MaxCloudSize, VN_Point_XYZI8 *pCloud_XYZI8, int *pCloudSize, int *pMatrixColumns, int *pMatrixRows);

/** @ingroup CloudOfPoint
 * \brief request the Cirrus3D to scan and get the scan result as matrix of XYZRGB points
 *  (more accurately pcl::PointXYZRGB from the PCL library) ;
 *  in this format the empty elements of the matrix have NaN coordinates, and the
 *  rgb float is actually a rgb int casted as float (where all r,g,b fields are
 *  identical since the Cirrus only generate black-and-white pixel data)
 *
 * Function available for device version greater than or equal to 1. (CirrusSensor version 3.0.0 and higher)
 * \param[in] IPAddress         : IP address of the Cirrus3D
 * \param[in] MaxCloudSize      : Max number of points that can be stored in the cloud of points buffer
 * \param[out] pCloud_XYZRGB    : pointer to a buffer able to store the resulting cloud of points
 * \param[out] pCloudSize       : Will contain the number of non-zero points in the matrix
 * \param[out] pMatrixColumns   : Will contain the number of columns in the matrix
 * \param[out] pMatrixRows      : Will contain the number of rows in the matrix
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise

 */
VN_tERR_Code VN_ExecuteSCAN_Matrix_XYZRGB(const char *IPAddress, const int MaxCloudSize, VN_Point_XYZRGB *pCloud_XYZRGB, int *pCloudSize, int *pMatrixColumns, int *pMatrixRows);

/** @ingroup CloudOfPoint
 * \brief request the Cirrus3D to scan and get the scan result as matrix of XYZI8 points
 *  The matrix corresponds to a rectified image from the left camera.
 *  In this format the empty elements of the matrix have NaN coordinates.
 *
 * Function available for device version greater than or equal to 2. (CirrusSensor version 3.1.0 and higher)
 * \param[in] IPAddress         : IP address of the Cirrus3D
 * \param[in] MaxCloudSize      : Max number of points that can be stored in the cloud of points buffer
 * \param[out] pCloud_XYZI8     : pointer to a buffer able to store the resulting cloud of points
 * \param[out] pCloudSize       : Will contain the number of non-zero points in the matrix
 * \param[out] pMatrixColumns   : Will contain the number of columns in the matrix
 * \param[out] pMatrixRows      : Will contain the number of rows in the matrix
 * \param[out] fromMiddleCamPOV : 1->the point of view of a virtual ideal middle camera, 0-> left camera
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_ExecuteSCAN_CameraCOP_XYZI8(const char *pIpAddress,
                                             const int MaxCloudSize,
                                             VN_Point_XYZI8 *pCloud_XYZI8,
                                             int *pCloudSize, int *pMatrixColumns, int *pMatrixRows,
                                             VN_BOOL fromMiddleCamPOV);


/** @ingroup CloudOfPoint
 * \brief request the Cirrus3D to scan and get the scan result as matrix of XYZI points
 *  The matrix corresponds to a rectified image from the left camera or a virtual middle camera.
 *  In this format the empty elements of the matrix have NaN coordinates.
 *  Function with version compatibility check before connection to the Cirrus3D.
 *
 * Function available for device version greater than or equal to 2. (CirrusSensor version 3.1.0 and higher)
 * \param[in] pCirrus3DHandler  : Cirrus3DHandler
 * \param[in] MaxCloudSize      : Max number of points that can be stored in the cloud of points buffer
 * \param[out] pCloud_XYZI8    : pointer to a buffer able to store the resulting cloud of points
 * \param[out] pCloudSize       : Will contain the number of non-zero points in the matrix
 * \param[out] pMatrixColumns   : Will contain the number of columns in the matrix
 * \param[out] pMatrixRows      : Will contain the number of rows in the matrix
 * \param[out] fromMiddleCamPOV : 1->the point of view of a virtual ideal middle camera, 0-> left camera
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_ExecuteSCAN_CameraCOP_XYZI8_s(const VN_Cirrus3DHandler *pCirrus3DHandler,
                                              const int MaxCloudSize,
                                              VN_Point_XYZI8 *pCloud_XYZI8,
                                              int *pCloudSize, int *pMatrixColumns, int *pMatrixRows,
                                              VN_BOOL fromMiddleCamPOV);

/** @ingroup CloudOfPoint
 * \brief request the Cirrus3D to scan and get the scan result as matrix of XYZI points
 *  The matrix corresponds to a rectified image from the left camera or a virtual middle camera.
 *  In this format the empty elements of the matrix have NaN coordinates.
 *  Available only for CirrusSensor version 3.0.0 and higher.
 *
 * Function available for device version greater than or equal to 2. (CirrusSensor version 3.1.0 and higher)
 * \param[in] IPAddress         : IP address of the Cirrus3D
 * \param[in] MaxCloudSize      : Max number of points that can be stored in the cloud of points buffer
 * \param[out] pCloud_XYZI8     : pointer to a buffer able to store the resulting cloud of points
 * \param[out] pCloudSize       : Will contain the number of non-zero points in the matrix
 * \param[out] pMatrixColumns   : Will contain the number of columns in the matrix
 * \param[out] pMatrixRows      : Will contain the number of rows in the matrix
 * \param[out] samplingFactor   : This parameter corresponds to the sampling factor applied during the conversion from the raw unordered cloud of point to MatrixXYZI8. The default value is 1.2
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_ExecuteSCAN_CameraCOP_XYZI8_sampling(const char *pIpAddress,
                                                     const int MaxCloudSize,
                                                     VN_Point_XYZI8 *pCloud_XYZI8,
                                                     int *pCloudSize,
                                                     int *pMatrixColumns,
                                                     int *pMatrixRows,
                                                     VN_BOOL fromMiddleCamPOV,
                                                     VN_REAL32 samplingFactor);
/** @ingroup CloudOfPoint
 * \brief request the Cirrus3D to scan and get the scan result as matrix of XYZI points
 *  The matrix corresponds to a rectified image from the left camera or a virtual middle camera.
 *  In this format the empty elements of the matrix have NaN coordinates.
 *  Function with version compatibility check before connection to the Cirrus3D.
 *
 * Function available for device version greater than or equal to 2. (CirrusSensor version 3.1.0 and higher)
 * \param[in] pCirrus3DHandler  : Cirrus3DHandler
 * \param[in] MaxCloudSize      : Max number of points that can be stored in the cloud of points buffer
 * \param[out] pCloud_XYZI8     : pointer to a buffer able to store the resulting cloud of points
 * \param[out] pCloudSize       : Will contain the number of non-zero points in the matrix
 * \param[out] pMatrixColumns   : Will contain the number of columns in the matrix
 * \param[out] pMatrixRows      : Will contain the number of rows in the matrix
 * \param[out] samplingFactor   : This parameter corresponds to the sampling factor applied during the conversion from the raw unordered cloud of point to MatrixXYZI8. The default value is 1.2
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_ExecuteSCAN_CameraCOP_XYZI8_sampling_s(const VN_Cirrus3DHandler *pCirrus3DHandler,
                                                       const int MaxCloudSize,
                                                       VN_Point_XYZI8 *pCloud_XYZI8,
                                                       int *pCloudSize,
                                                       int *pMatrixColumns,
                                                       int *pMatrixRows,
                                                       VN_BOOL fromMiddleCamPOV,
                                                       VN_REAL32 samplingFactor);

/** @ingroup CloudOfPoint
 * \brief request the Cirrus3D to scan and get the scan result as matrix of XYZI points
 *  The points are organized in a structured cloud, sorted by their projection on
 *   a virtual camera centered on the Cirrus. The matrix size (number of rows & columns)
 *   vary depending on the selected density, the selected ROI, and the Cirrus calibration.
 *  The scan result is transfered during the scan instead of after to minize the
 *   total time needed for the operation.
 *  In this format the empty elements of the matrix have NaN coordinates.
 *
 * Function available for device version greater than or equal to 2. (CirrusSensor version 3.1.0 and higher)
 * \param[in] IPAddress         : IP address of the Cirrus3D
 * \param[in] MaxCloudSize      : Max number of points that can be stored in the cloud of points buffer
 * \param[out] pCloud_XYZI8     : pointer to a buffer able to store the resulting cloud of points
 * \param[out] pMatrixColumns   : Will contain the number of columns in the matrix
 * \param[out] pMatrixRows      : Will contain the number of rows in the matrix
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_ExecuteSCAN_CameraCOP_MiddleCam2_XYZI(const char *pIpAddress,
                                                      const int MaxCloudSize,
                                                      VN_Point_XYZI8 *pCloud_XYZI8,
                                                      int *pMatrixColumns,
                                                      int *pMatrixRows);


/** @ingroup CloudOfPoint
 * \brief request the Cirrus3D to scan and get the scan result as rectified image
 *  The "Rectified" format is a format where only the z coordinate is transferred, and 'Rectified',
 *  meaning that the points are sorted in a grid with a fixed interval. The coordinates are expressed as 16-bits unsigned.
 *  Presented differently, this format is a depth map: an image built of 16-bits pixel,
 *  where the darkest pixels correspond to points closer to the Cirrus. The 3d coordinates x,y,z of the 3d points can
 *  then be extracted from the u,v pixel coordinates and their corresponding pixVal :
 *       x = x_scale * u + x_offset
 *       y = y_scale * v + y_offset
 *       z = z_scale * pixVal + z_offset
 *  With, in this case, x_scale = y_scale.
 *  Invalid pixels (the elements of the rectified image that do not contain valid data) are set to a user-selected value (generally 0 or 65535).
 *  This format was designed to be compatible with a wide variety of applications, including those who have yet to expand
 *  in the "3D processing" territory and are designed for use with 2d images.
 *  However, this format has some drawbacks :
 *     - since the x & y coordinates are implicit, there is a loss of accuracy (the raw 3d points are rarely
 *      generated at the exact center of the pixel)
 *     - the Cirrus natively generates a denser coverage of 3d points on closer objects than on farther objects.
 *      In order to minimize the number of "holes" in the RectifiedC image, the image is under-sampled, losing about 20% resolution compared to the CalibratedABC_Grid format.
 *     - the Cirrus natively  generates 3d points from a perspective view, meaning that if the Cirrus is suspended
 *      above a bin it is able to (at least partially) scan the sides of the bin, and generates 3d points sharing (roughly)
 *      the same x,y, coordinates but with a different z. With this RectifiedC format, those points are lost : in case of multiple
 *      3d points projecting to the same pixel, we only keep the one closest to the Cirrus.
 *  The resolution of the image (number of lines & columns) vary only depending on the selected scan density ; this is an arbitrary choice
 *   for easier integration with applications where the image resolution is hard-coded.
 *
 * Function available for device version greater than or equal to 2. (CirrusSensor version 5.0.0 and higher)
 * \param[in] IPAddress         : IP address of the Cirrus3D
 * \param[in] MaxCloudSize      : Max number of points that can be stored in the cloud of points buffer
 * \param[out] pCloud_DepthMap  : pointer to a buffer able to store the resulting cloud of points
 * \param[out] pMatrixColumns   : Will contain the number of columns in the matrix
 * \param[out] pMatrixRows      : Will contain the number of rows in the matrix
 * \param[out] pInvalidVal      : Will contain the invalid value
 * \param[out] pXOffset         : Will contain the XOffset
 * \param[out] pXScale          : Will contain the XScale
 * \param[out] pYOffset         : Will contain the YOffset
 * \param[out] pYScale          : Will contain the YScale
 * \param[out] pZOffset         : Will contain the ZOffset
 * \param[out] pZScale          : Will contain the ZScale
 * \param[out] samplingFactor   : Will contain the sampling factor
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_ExecuteSCAN_DepthMap(const char *pIpAddress,
                                         const int MaxCloudSize,
                                         VN_UINT16 *pDepthMap,
                                         VN_UINT32 *pMatrixColumns,
                                         VN_UINT32 *pMatrixRows,
                                         VN_UINT16 *pInvalidVal,
                                         float *pXOffset, float *pXScale,
                                         float *pYOffset, float *pYScale,
                                         float *pZOffset, float *pZScale,
                                         float samplingFactor);

/** @ingroup CloudOfPoint
 * \brief Read header receive on the socket sock after a command SCAN Matrix_XYZRGB
 * \param[in] sock : socket used to send the SCAN command
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_ExecuteSCAN_Matrix_XYZRGB_readHeader(VN_SOCKET sock,
                                                     VN_INT32 *pStatus,
                                                     VN_INT32 *pCloudBytesSize,
                                                     VN_INT32 *pCloudPoints,
                                                     VN_INT32 *pRowCount,
                                                     VN_INT32 *pColumnCount);

/**
 * \brief get status / keepAlive response from the Cirrus.
 *  Available only for Cirrus version 2.0.0 and higher.
 *  Can be requested even during a scan (contrary to
 *  the STS command)
 * \param[in] IPaddress : IP address of the Cirrus to command
 * \retval VN_eERR_NoError if success (i.e. Cirrus connected no reported error)
 * \retval error code otherwise
 */
VN_tERR_Code VN_ExecuteCommandKAST(const char *IPaddress);

/** 
 * \brief This function is used to get Cirrus3D available
 *  Available only for Cirrus version 2.0.0 and higher.
 * \param[in,out] pNbCirrus3DAvailable                         : Number
 * \param[in,out] ipCirrus3DAvailable[20][VN_cSizeDataString]  : IPAddress of cirrus available
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_GetCirrus3DAvailable(VN_INT32 *pNbCirrus3DAvailable, char ipCirrus3DAvailable[VN_cMaxNbCirrus3D][VN_cSizeDataString]);

enum
{
    VN_CmdId_InitializeProductHandler,
    VN_CmdId_GetData,
    VN_CmdId_GetDataList,
    VN_CmdId_GetDataValue,
    VN_CmdId_SetDataValue,
    VN_CmdId_ResetProductHandler,
    VN_CmdId_ForceDeviceVersion,
    VN_CmdId_GetProductHandler
};

/** @ingroup General
 * \brief This function is used to initialize the VN_Cirrus3DHandler structure used by the get/set data functions
 *  Available only for Cirrus version 3.0.0 and higher.
 * \param[in]  IPAddress        : IP address of the Cirrus3D
 * \param[out] pCirrus3DHandler : Will contain the VN_Cirrus3DHandler structure initialized for the Cirrus3D with ip address=IPAddress
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_Cirrus3DHandler_Initialize(char *IPAddress,
                                           VN_Cirrus3DHandler *pCirrus3DHandler);


/** @ingroup Config
 * \brief This function get the VN_Cirrus3DHandler structure used by the get/set data functions without initilize data in the CirrusSensor
 * \param[in]  IPAddress        : IP address of the Cirrus3DSensor
 * \param[out] pCirrus3DHandler : Will contain the VN_Cirrus3DHandler structure initialized for the Cirrus3DSensor with ip address=IPAddress
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_Cirrus3DHandler_Get(char* pIpAddress,
                                    VN_Cirrus3DHandler *pCirrus3DHandler);

/** @ingroup General
 * \brief This function is used to force device version
 *  Available only for Cirrus version 3.0.0 and higher.
 * \param[in] pCirrus3DHandler : Data previously initialized with the VN_Cirrus3DHandler_initialize() function
 * \param[in] NewDeviceVersion : Device version used to configure the cirrus3D.
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_Cirrus3DHandler_ForceVersionDevice(VN_Cirrus3DHandler *pCirrus3DHandler,
                                                   VN_UINT32 NewDeviceVersion);

/** @ingroup Config
 * \brief This function is used to reset the VN_Cirrus3DHandler structure used by the get/set data functions
 * Function available for device version greater than or equal to 2. (CirrusSensor version 3.1.0 and higher)
 * \param[in] pCirrus3DHandler : Will contain the VN_Cirrus3DHandler structure initialized for the Cirrus3DSensor with ip address=IPAddress
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_Cirrus3DHandler_ResetData(VN_Cirrus3DHandler *pCirrus3DHandler);

/** @ingroup Config
 * \brief This function is used to get the list of configuration datas
 *  Available only for Cirrus version 3.0.0 and higher. 
 * \param[in]  pCirrus3DHandler : Data previously initialized with the VN_Cirrus3DHandler_initialize() function
 * \param[out] pNbParamInList   : Number of data available
 * \param[out] DataList         : List of data available
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_Cirrus3DHandler_GetDataRootList(const VN_Cirrus3DHandler *pCirrus3DHandler,
                                                VN_INT32 *pNbParamInList,
                                                VN_tDataRoot DataList[VN_cSizeList]);

/** @ingroup Config
 * \brief This function is used to get the data
 *  Available only for Cirrus version 3.0.0 and higher.
 * \snippet main.c VN_Cirrus3DHandler_GetData example
 * \param[in] pCirrus3DHandler : Data previously initialized with the VN_Cirrus3DHandler_initialize() function
 * \param[in] DataId           : Data Index
 * \param[in] DataType         : Data Type
 * \param[out] pData           : Data Structure of DataType type.
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_Cirrus3DHandler_GetData(const VN_Cirrus3DHandler *pCirrus3DHandler,
                                        VN_UINT32 DataId,
                                        VN_eDataType DataType,
                                        VN_tDataRoot* pData);

/** @ingroup Config
 * \brief This function is used to get the data of type INT32
 *  Available only for Cirrus version 3.0.0 and higher. 
 * \param[in] pCirrus3DHandler : Data previously initialized with the VN_Cirrus3DHandler_initialize() function
 * \param[in] DataId           : Data Index
 * \param[out] pDataInt32      : Structure contains the data of type INT32
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
static inline VN_tERR_Code VN_Cirrus3DHandler_GetDataInt32(const VN_Cirrus3DHandler *pCirrus3DHandler,
                                                           VN_UINT32 DataId,
                                                           VN_tDataInt32* pDataInt32)
{
    return VN_Cirrus3DHandler_GetData(pCirrus3DHandler,
                                      DataId,
                                      VN_eTypeInt32,
                                      (VN_tDataRoot*)pDataInt32);
};

/** @ingroup Config
 * \brief This function is used to get the data of type REAL32
 *  Available only for Cirrus version 3.0.0 and higher. 
 * \param[in] pCirrus3DHandler : Data previously initialized with the VN_Cirrus3DHandler_initialize() function
 * \param[in] DataId           : Data Index
 * \param[out] pDataReal32     : Structure contains the data of type REAL32
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
static inline VN_tERR_Code VN_Cirrus3DHandler_GetDataReal32(const VN_Cirrus3DHandler *pCirrus3DHandler,
                                                            VN_UINT32 DataId,
                                                            VN_tDataReal32* pDataReal32)
{
    return VN_Cirrus3DHandler_GetData(pCirrus3DHandler,
                                      DataId,
                                      VN_eTypeReal32,
                                      (VN_tDataRoot*)pDataReal32);
};

/** @ingroup Config
 * \brief This function is used to get the data of type char[]
 *  Available only for Cirrus version 3.0.0 and higher. 
 * \param[in] pCirrus3DHandler : Data previously initialized with the VN_Cirrus3DHandler_initialize() function
 * \param[in] DataId           : Data Index
 * \param[out] pDataString     : Structure contains the data of type char[]
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
static inline VN_tERR_Code VN_Cirrus3DHandler_GetDataString(const VN_Cirrus3DHandler *pCirrus3DHandler,
                                                            VN_UINT32 DataId,
                                                            VN_tDataString* pDataString)
{
    return VN_Cirrus3DHandler_GetData(pCirrus3DHandler,
                                      DataId,
                                      VN_eTypeString,
                                      (VN_tDataRoot*)pDataString);
};

/** @ingroup Config
 * \brief This function is used to get the data value
 *  Available only for Cirrus version 3.0.0 and higher. 
 * \param[in] pCirrus3DHandler : Data previously initialized with the VN_Cirrus3DHandler_initialize() function
 * \param[in,out] pDataInt32   : Get the value of the data. The structure was initialized by calling VN_Cirrus3DHandler_GetData(), VN_Cirrus3DHandler_GetDataInt32(), VN_Cirrus3DHandler_GetDataReal32() or VN_Cirrus3DHandler_GetDataString() function
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_Cirrus3DHandler_GetDataValue(const VN_Cirrus3DHandler *pCirrus3DHandler,
                                             VN_tDataRoot* pData);

/** @ingroup Config
 * \brief This function is used to get the data value of type INT32
 *  Available only for Cirrus version 3.0.0 and higher. 
 * \param[in] pCirrus3DHandler : Data previously initialized with the VN_Cirrus3DHandler_initialize() function
 * \param[in,out] pDataInt32   : Get the value of the data. The structure was initialized by calling VN_Cirrus3DHandler_GetData(), VN_Cirrus3DHandler_GetDataInt32() function
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
static inline VN_tERR_Code VN_Cirrus3DHandler_GetDataInt32Value(const VN_Cirrus3DHandler *pCirrus3DHandler,
                                                                VN_tDataInt32* pDataInt32)
{
    return VN_Cirrus3DHandler_GetDataValue(pCirrus3DHandler,
                                           (VN_tDataRoot*)pDataInt32);
};

/** @ingroup Config
 * \brief This function is used to get the data value of type REAL32
 *  Available only for Cirrus version 3.0.0 and higher. 
 * \param[in] pCirrus3DHandler : Data previously initialized with the VN_Cirrus3DHandler_initialize() function
 * \param[in,out] pDataReal32  : Get the value of the data. The structure was initialized by calling VN_Cirrus3DHandler_GetData(), VN_Cirrus3DHandler_GetDataInt32(), VN_Cirrus3DHandler_GetDataReal32() function
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
static inline VN_tERR_Code VN_Cirrus3DHandler_GetDataReal32Value(const VN_Cirrus3DHandler *pCirrus3DHandler,
                                                                 VN_tDataReal32* pDataReal32)
{
    return VN_Cirrus3DHandler_GetDataValue(pCirrus3DHandler,
                                           (VN_tDataRoot*)pDataReal32);
};

/** @ingroup Config
 * \brief This function is used to get the data value of type char[]
 *  Available only for Cirrus version 3.0.0 and higher. 
 * \param[in] pCirrus3DHandler : Data previously initialized with the VN_Cirrus3DHandler_initialize() function
 * \param[in,out] pDataString  : Get the data value. The structure was initialized by calling VN_Cirrus3DHandler_GetData() or VN_Cirrus3DHandler_GetDataString() function
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
static inline VN_tERR_Code VN_Cirrus3DHandler_GetDataStringValue(const VN_Cirrus3DHandler *pCirrus3DHandler,
                                                                 VN_tDataString* pDataString)
{
    return VN_Cirrus3DHandler_GetDataValue(pCirrus3DHandler,
                                           (VN_tDataRoot*)pDataString);
};

/** @ingroup Config
 * \brief This function is used to set the data value
 *  Available only for Cirrus version 3.0.0 and higher. 
 * \param[in] pCirrus3DHandler : Data previously initialized with the VN_Cirrus3DHandler_initialize() function
 * \param[in,out] pData        : Get the data value. The structure was initialized by calling VN_Cirrus3DHandler_GetData(), VN_Cirrus3DHandler_GetDataInt32(), VN_Cirrus3DHandler_GetDataReal32() or VN_Cirrus3DHandler_GetDataString() function
 * \param[in] value: new value of the data in the product.
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise

 */
VN_tERR_Code VN_Cirrus3DHandler_SetDataValue(const VN_Cirrus3DHandler *pCirrus3DHandler,
                                             VN_tDataRoot* pData);

/** @ingroup Config
 * \brief This function is used to get the data value
 *  Available only for Cirrus version 3.0.0 and higher. 
 * \param[in] pCirrus3DHandler : Data previously initialized with the VN_Cirrus3DHandler_initialize() function
 * \param[in,out] pDataInt32   : Get the data value. The structure was initialized by calling VN_Cirrus3DHandler_GetData(), VN_Cirrus3DHandler_GetDataInt32() function
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise

 */
static inline VN_tERR_Code VN_Cirrus3DHandler_SetDataInt32Value(const VN_Cirrus3DHandler *pCirrus3DHandler,
                                                                VN_tDataInt32* pDataInt32)
{
    return VN_Cirrus3DHandler_SetDataValue(pCirrus3DHandler,
                                           (VN_tDataRoot*)pDataInt32);
};
/** @ingroup Config
 * \brief This function is used to get the data value
 *  Available only for Cirrus version 3.0.0 and higher. 
 * \param[in] pCirrus3DHandler : Data previously initialized with the VN_Cirrus3DHandler_initialize() function
 * \param[in,out] pDataReal32  : Get the data value. The structure was initialized by calling VN_Cirrus3DHandler_GetData() or VN_Cirrus3DHandler_GetDataReal32() function
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
static inline VN_tERR_Code VN_Cirrus3DHandler_SetDataReal32Value(const VN_Cirrus3DHandler *pCirrus3DHandler,
                                                                 VN_tDataReal32* pDataReal32)
{
    return VN_Cirrus3DHandler_SetDataValue(pCirrus3DHandler,
                                           (VN_tDataRoot*)pDataReal32);
};

/** @ingroup Config
 * \brief This function is used to get the data value
 *  Available only for Cirrus version 3.0.0 and higher. 
 * \param[in] pCirrus3DHandler : Data previously initialized with the VN_Cirrus3DHandler_initialize() function
 * \param[in,out] pDataString  : Struct contain the new value of the data. The structure was initialized by calling VN_Cirrus3DHandler_GetData() or VN_Cirrus3DHandler_GetDataString() function
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
static inline VN_tERR_Code VN_Cirrus3DHandler_SetDataStringValue(const VN_Cirrus3DHandler *pCirrus3DHandler,
                                                                 VN_tDataString* pDataString)
{
    return VN_Cirrus3DHandler_SetDataValue(pCirrus3DHandler,
                                           (VN_tDataRoot*)pDataString);
};

/**************************************************************************/
#define VN_cMagicNumber 123456

typedef struct _VN_HeaderCmdSDK
{
    VN_UINT32 protocolVersion;
    VN_UINT32 headerVersion;
    VN_UINT32 CmdId;
    VN_UINT32 sizeData;
}VN_HeaderCmdSDK;

typedef struct _VN_HeaderAckSDK
{
    VN_UINT32 protocolVersion;
    VN_UINT32 headerVersion;
    VN_UINT32 deviceVersion;
    VN_UINT32 CmdId;
    VN_UINT32 Ackstatus;
    VN_UINT32 sizeData;
}VN_HeaderAckSDK;

VN_tERR_Code VN_COM_HeaderCmd_send(VN_SOCKET sock,
                                   VN_UINT32 protocolVersion,
                                   VN_UINT32 headerVersion,
                                   VN_UINT32 CmdId,
                                   VN_UINT32 sizeData);

VN_tERR_Code VN_COM_HeaderAck_send(VN_SOCKET sock,
                                   VN_UINT32 protocolVersion,
                                   VN_UINT32 headerVersion,
                                   VN_UINT32 deviceVersion,
                                   VN_UINT32 CmdId,
                                   VN_UINT32 Ackstatus,
                                   VN_UINT32 sizeData);

VN_tERR_Code VN_COM_HeaderAck_receive(VN_SOCKET sock,
                                      VN_UINT32 *protocolVersion,
                                      VN_UINT32 *headerVersion,
                                      VN_UINT32 *deviceVersion,
                                      VN_UINT32 *Ackstatus,
                                      VN_UINT32 *sizeData);

VN_tERR_Code VN_COM_HeaderCmd_receive(VN_SOCKET sock,
                                      VN_UINT32 *protocolVersion,
                                      VN_UINT32 *headerVersion,
                                      VN_UINT32 *CmdId,
                                      VN_UINT32 *sizeData);

#endif // COMMANDFUNCTIONS_H
