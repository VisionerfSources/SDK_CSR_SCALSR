/**
 *  @file VN_Cirrus3D.API.h
 *  This file contains all functions, structures and constants to communicate with the Cirrus3D SCALSR in using the visionerf librairy
 *  All the code is written in C language for maximal compatibility, and is designed to be compatible with Linux and Windows
 *  @author Visionerf
 *  @date 2023/03/08
 *  @version 2.3
 */
/*------------------------------------------------------------------------------
 *  History:
 *  16/02/2024: CHANGED: new value for VN_cSizeName 16->56.................. P.H
 *  16/02/2024: NEW: new function VN_GetCirrus3DAvailable................... P.H
 *  16/02/2024: NEW: new function VN_Cirrus3DHandler_ResetData.............. P.H
 *  16/02/2024: NEW: new function VN_SaveMatrixXYZRGBAsDepthMapBmp.......... P.H
 *  06/03/2024: NEW: new function VN_Cirrus3DHandler_ResetData.............. P.H
 *              CHANGE: VN_cSizeName 16 -> 56............................... P.H
 *  16/05/2024: NEW: new function VN_ExecuteSCAN_CameraCOP_XYZI8
 *                                VN_ExecuteSCAN_MiddleCameraCOP_XYZI8
 *                                VN_ExecuteSCAN_LeftCameraCOP_XYZI8
 *                                VN_ExecuteSCAN_MiddleCameraCOP_XYZI8_Sampling
 *                                VN_ExecuteSCAN_LeftCameraCOP_XYZI8
 *                                VN_ExecuteSCAN_CameraCOP_XYZI8_sampling
 *              CHANGE: VN_SaveCloudAsBinFile->VN_SaveXYZCloudAsBinFile
 *                      VN_SaveCloudAsTxtFile->VN_SaveXYZCloudAsTxtFile
 *                      VN_SaveAsPcd->VN_SaveXYZI16CloudAsPcdFile
 *                      VN_SaveAsDepthMapBmp->VN_SaveXYZI8CloudAsDepthMapBmp P.H
 * 24/07/2024 NEW VN_ExecuteSCAN_DepthMap function.......................... P.H
 *------------------------------------------------------------------------------
 */

#ifndef VN_CIRRUS3D_API_H
#define VN_CIRRUS3D_API_H
/********************************** INCLUDE **********************************/
/** @defgroup CommunicationTool General communication functions, variables...
 *  This group contain general functions and variables available to communicate with a Cirrus3D
 */

/** @defgroup CloudOfPoint Get and save a cloud of points
 *  This group contain functions and variables available to get and save a cloud of points with a Cirrus3D
 */

/** @defgroup Config Get and set Cirrus3D configuration
 *  This group contain functions and variables available to get and set Cirrus3D configuration
 */

/** @defgroup GeneralSetting Get general Cirrus3D settings
 *  This group contain functions and variables available to get general Cirrus3D settings
 */

/** @defgroup GeneralConst General constants
 *  This group contain constants used by the SDK
 */

typedef unsigned char VN_UINT8;
typedef unsigned short VN_UINT16;
typedef unsigned int VN_UINT32;
typedef int VN_INT32;
typedef int VN_BOOL;
typedef float VN_REAL32;

/********************************** GENERAL GLOBALS **********************************/
/** @ingroup GeneralConst
 *  @def VN_cSizeDataString
 *  @brief Maximum number of characters in a data of type char[]
 */
#define VN_cSizeDataString 256

/** @ingroup GeneralConst
 *  @def VN_cMaxNbCirrus3D
 *  @brief Maximum number of Cirrus3D
 */
#define VN_cMaxNbCirrus3D 20


/*******************************************************************************************/

/********************************** COMMUNICATION GLOBALS **********************************/

/** @ingroup CommunicationTool
*   @enum VN_tERR_Code
*   @brief List error code returned by the SDK function.
*/
typedef enum _VN_tERR_Code{
    VN_eERR_NoError=0, /*!< @ingroup CommunicationTool*//*!< @brief The request succeeded. (0) */
    VN_eERR_UndefinedError, /*!< @ingroup CommunicationTool*//*!< @brief The sensor has encountered a situation it does not know how to handle. (1) */
    VN_eERR_WSAStartup,/*!< @ingroup CommunicationTool *//*!< @brief The function WSAStartup returned an error. The WSAStartup function initiates use of the Winsock DLL by a process. (2) */
    VN_eERR_WSACleanup,/*!< @ingroup CommunicationTool *//*!< @brief The function WSACleanup returned an error. The WSACleanup function terminates use of the Winsock 2 DLL (Ws2_32.dll). (3) */
    VN_eERR_SocketError,/*!< @ingroup CommunicationTool *//*!< @brief The socket is invalid. (4)*/
    VN_eERR_SocketConnectError,/*!< @ingroup CommunicationTool *//*!< @brief The connection to the sensor failed. (5)*/
    VN_eERR_SocketSendError,/*!< @ingroup CommunicationTool *//*!< @brief Sending data to the sensor failed. (6)*/
    VN_eERR_SocketCloseError,/*!< @ingroup CommunicationTool *//*!< @brief Closing the connection with the sensor failed. (7)*/
    VN_eERR_SocketRecvError,/*!< @ingroup CommunicationTool *//*!< @brief Data was not received. (8)*/
    VN_eERR_SocketIncompleteCom,/*!< @ingroup CommunicationTool *//*!< @brief The number of data received is not as expected. (9)*/
    VN_eERR_Failure,/*!< @ingroup CommunicationTool *//*!< @brief The sensor has encountered a situation it does not know how to handle. (10)*/
    VN_eERR_UnknownCommand,/*!< @ingroup CommunicationTool *//*!< @brief The request method is not supported by the Cirrus3D. (11)*/
    VN_eERR_InsufficientMemory,/*!< @ingroup CommunicationTool *//*!< @brief Insufficient memory to create the requested buffer or the received cloud of points exceeds the maximum allowable size. (12)*/
    VN_eERR_Nullpointer,/*!< @ingroup CommunicationTool *//*!< @brief A null pointer detected. (13)*/
    VN_eERR_BadHeader,/*!< @ingroup CommunicationTool *//*!< @brief The server cannot or will not process the request due to something that is perceived to be a client error. (14)*/
    VN_eERR_FileAccessError,/*!< @ingroup CommunicationTool *//*!< @brief Unable to access the file (15)*/
    VN_eERR_FileWriteError,/*!< @ingroup CommunicationTool *//*!< @brief Unable to write in the file (16)*/
    VN_eERR_BadStatus,/*!< @ingroup CommunicationTool *//*!< @brief Bad Status (17)*/
    VN_eERR_BadNetworkConfig,/*!< @ingroup CommunicationTool *//*!< @brief Bad Network Config (18)*/
    VN_eERR_UnknownData, /*!< @ingroup CommunicationTool *//*!< @brief The requested data is not known by the Cirrus3D (19)*/
    VN_eERR_ValueOutOfRanges, /*!< @ingroup CommunicationTool *//*!< @brief The value of the data is out of Ranges. (20)*/
    VN_eERR_UnknownDataType, /*!< @ingroup CommunicationTool *//*!< @brief The requested data type is not known by the Cirrus3D (21)*/
    VN_eERR_IncompatibleProtocolVersion, /*!< @brief The SDK protocol version is incompatible with the Cirrus3D protocol version (22)*/
    VN_eERR_IncompatibleHeaderVersion, /*!< @brief The SDK protocol header version is incompatible with the Cirrus3D protocol header version (22)*/
    VN_eERR_IncompatibleDeviceVersion, /*!< @brief The SDK protocol device version is incompatible with the Cirrus3D protocol device version (22)*/
}VN_tERR_Code;

/**
 *  @struct VN_Cirrus3DHandler
 *  @brief Read-Only Structure containing the properties of the Cirrus3D. This structure is initialized with VN_Cirrus3DHandler_initialize function.
  * @param IPAddress: Cirrus3D IPAddress
  * @param SerialNumber: Cirrus3D serialNumber
  * @param ProtocolVersion: Protocol version used between the SDK and the Cirrus3D.
  * @param HeaderVersion: Header version used by the protocol between the SDK and the Cirrus3D.
  * @param DeviceVersion: Cirrus3D configuration version
 */
typedef struct _VN_Cirrus3DHandler
{
    char IPAddress[VN_cSizeDataString]; /*!< @brief [Read only] */
    char SerialNumber[VN_cSizeDataString]; /*!< @brief [Read only] */
    VN_UINT32 ProtocolVersion; /*!< @brief [Read only] */
    VN_UINT32 HeaderVersion; /*!< @brief [Read only] */
    VN_UINT32 DeviceVersion; /*!< @brief [Read only] */
}VN_Cirrus3DHandler;

/*********************** END COMMUNICATION GLOBALS **************************/

/***************************** COMMAND STRUCTURES ******************************/
/**
 *  @def VN_cMaxCOPSize
 *  @brief Max credible size of a cloud of points
 */
#define VN_cMaxCOPSize 6000000

/** @ingroup CloudOfPoint
  * @struct VN_Point_XYZI16
  * @brief Structure containing a point of type XYZI16
  * @param x: coordinate x (32-bit in single-precision IEEE-754 binary floatingpoint format)
  * @param y: coordinate y (32-bit in single-precision IEEE-754 binary floatingpoint format)
  * @param z: coordinate z (32-bit in single-precision IEEE-754 binary floatingpoint format)
  * @param i: intensity (16-bit unsigned integer)
  */
typedef struct _VN_Point_XYZI16
{
    VN_REAL32 x;
    VN_REAL32 y;
    VN_REAL32 z;
    VN_UINT16 i;
}VN_Point_XYZI16;

/** @ingroup CloudOfPoint
  * @struct VN_Point_XYZI8
  * @brief Structure containing a point of type XYZI8
  * @param x: coordinate x (32-bit in single-precision IEEE-754 binary floating-point format)
  * @param y: coordinate y (32-bit in single-precision IEEE-754 binary floating-point format)
  * @param z: coordinate z (32-bit in single-precision IEEE-754 binary floating-point format)
  * @param i: intensity (8-bit unsigned integer)
  */
typedef struct _VN_Point_XYZI8
{
    VN_REAL32 x;
    VN_REAL32 y;
    VN_REAL32 z;
    VN_UINT8 i;
}VN_Point_XYZI8;

/** @ingroup CloudOfPoint
  * @struct VN_Point_XYZRGB
  * @brief Structure containing a point of type pcl::PointXYZRGB Struct Reference
  * @param x: coordinate x (32-bit in single-precision IEEE-754 binary floating-point format)
  * @param y: coordinate y (32-bit in single-precision IEEE-754 binary floating-point format)
  * @param z: coordinate z (32-bit in single-precision IEEE-754 binary floating-point format)
  * @param rgb: RGB color information is packed into an integer and casted to a float.(32-bit in single-precision IEEE-754 binary floating-point format)
  */
typedef struct _VN_Point_XYZRGB
{
    VN_REAL32 x;
    VN_REAL32 y;
    VN_REAL32 z;
    VN_REAL32 rgb;
}VN_Point_XYZRGB;
/*********************** END COMMAND STRUCTURES **************************/

/*********************** CONFIGURATION STRUCTURES AND CONSTANTS **************************/
/** @ingroup Config
 *  @def VN_cSizeName
 *  @brief Maximum number of characters in data names
 */
#define VN_cSizeName 56

/** @ingroup Config
 *  @def VN_cSizeDescription
 *  @brief Maximum number of characters in data description
 */
#define VN_cSizeDescription 1024

/** @ingroup Config
 *  @def VN_cSizeToolTip
 *  @brief Maximum number of characters in data ToolTip
 */
#define VN_cSizeToolTip 256

/** @ingroup Config
 *  @def VN_cSizeUnits
 *  @brief Maximum number of characters in data units
 */
#define VN_cSizeUnits 8

/** @ingroup Config
 *  @def VN_cSizeList
 *  @brief Maximum number of datas available.
 */
#define VN_cSizeList 256

/** @ingroup Config
 *  @enum VN_eDataType
 *  @brief Data type used in the SDK.
 */
typedef enum _VN_eDataType{
    VN_eTypeInt32, /*!< @ingroup Config *//*!< @brief int*/
    VN_eTypeReal32, /*!< @ingroup Config *//*!< @brief float*/
    VN_eTypeString /*!< @ingroup Config *//*!< @brief char[]*/
}VN_eDataType;


/** @ingroup Config
 *  @enum VN_eAvailableStatus
 *  @brief Data status in the Cirrus3D
 */
typedef enum _VN_eAvailableStatus{
    VN_eAvailable, /*!< @brief Data available*/
    VN_eDeprecated, /*!<  @brief Data will be removed in the futur version*/
    VN_eObsolete, /*!< @brief Data obsolete*/
}VN_eAvailableStatus;

/** @ingroup Config
 *  @struct VN_tDataRoot
 *  @brief Read-Only Structure contains the properties of the data. The structure is initialized by calling VN_Cirrus3DHandler_GetData(), VN_Cirrus3DHandler_GetDataInt32(), VN_Cirrus3DHandler_GetDataReal32() or VN_Cirrus3DHandler_GetDataString() function.
  * @param magicNumber: [Read only] Used by visionerf protocol
  * @param paramId: [Read only] Data index
  * @param paramtype: [Read only] Data type
  * @param isSupported: [Read only] Data status
  * @param isReadOnly: [Read only] Data propreties in read/write
 */
typedef struct _VN_tDataRoot{
    VN_INT32 magicNumber;/*!< @brief [Read only] */
    VN_INT32 paramId;/*!< @brief [Read only] */
    VN_eDataType paramtype;/*!< @brief [Read only] */
    VN_eAvailableStatus isSupported;/*!< @brief [Read only] */
    VN_BOOL isReadOnly;/*!< @brief [Read only] */
}VN_tDataRoot;


/** @ingroup Config
 *  @struct VN_tDataInfo
 *  @brief Read-Only Structure contains information of the data. The structure is initialized by calling VN_Cirrus3DHandler_GetData(), VN_Cirrus3DHandler_GetDataInt32(), VN_Cirrus3DHandler_GetDataReal32() or VN_Cirrus3DHandler_GetDataString() function.
  * @param dataRoot: [Read only] Properties data
  * @param name: [Read only] Data name
  * @param description: [Read only] Data description
  * @param toolTip: [Read only] Data toolTip
 */
typedef struct _VN_tDataInfo{
    VN_tDataRoot dataRoot;/*!< @brief [Read only] */
    char name[VN_cSizeName];/*!< @brief [Read only] */
    char description[VN_cSizeDescription];/*!< @brief [Read only] */
    char toolTip[VN_cSizeToolTip];/*!< @brief [Read only] */
}VN_tDataInfo;

/** @ingroup Config
 *  @struct VN_tDataInt32
 *  @brief Structure contains the value and the information data of type VN_INT32. The structure is initialized by calling VN_Cirrus3DHandler_GetDataRootList() or VN_Cirrus3DHandler_GetDataInt32() function.
 * @param dataInfo: [Read only] Information data
 * @param units: [Read only] Data units
 * @param minValue: [Read only] Minimal value
 * @param maxValue: [Read only] Maximal value
 * @param stepValue: [Read only] Step between two values
 * @param value: [read/Write] Current value
 */
typedef struct _VN_tDataInt32{
    VN_tDataInfo dataInfo;/*!< @brief [Read only] */
    char units[VN_cSizeUnits];/*!< @brief [Read only] */
    VN_INT32 minValue;/*!< @brief [Read only] */
    VN_INT32 maxValue;/*!< @brief [Read only] */
    VN_INT32 stepValue;/*!< @brief [Read only] */
    VN_INT32 value;/*!< @brief [read/write] */
}VN_tDataInt32;

/** @ingroup Config
 *  @struct VN_tDataReal32
 *  @brief Structure contains the value and the information data of type VN_REAL32. The structure is initialized by calling VN_Cirrus3DHandler_GetDataRootList() or VN_Cirrus3DHandler_GetDataReal32() function.
 * @param dataInfo: [Read only] Information data
 * @param units: [Read only] Data units
 * @param minValue: [Read only] Minimal value
 * @param maxValue: [Read only] Maximal value
 * @param stepValue: [Read only] Step between two values
 * @param value: [read/Write] Current value
 */
typedef struct _VN_tDataReal32{
    VN_tDataInfo dataInfo;/*!< @brief [Read only] */
    char units[VN_cSizeUnits];/*!< @brief [Read only] */
    float minValue;/*!< @brief [Read only] */
    float maxValue;/*!< @brief [Read only] */
    float stepValue;/*!< @brief [Read only] */
    float value;/*!< @brief [read/write] */
}VN_tDataReal32;


/** @ingroup Config
 *  @struct VN_tDataString
 *  @brief Structure contains the value and the information data of type VN_REAL32. The structure is initialized by calling VN_Cirrus3DHandler_GetDataRootList() or VN_Cirrus3DHandler_GetDataString() function.
 * @param dataInfo: [Read only] Information data
 * @param value: [read/Write] Current value
 */
typedef struct _VN_tDataString{
    VN_tDataInfo dataInfo; /*!< @brief [Read only] */
    char value[VN_cSizeDataString];/*!< @brief [read/write] */
}VN_tDataString;

/*********************** END CONFIGURATION STRUCTURES AND CONSTANTS **************************/


/************************* COMMUNICATION FUNCTIONS ****************************/
/** @ingroup CommunicationTool
 * \brief Enable connection protocol for Windows OS
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_CirrusCom_Init(void);

/** @ingroup CommunicationTool
 * \brief Cleanly end connection protocol for Windows OS
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_CirrusCom_End(void);

/*********************** END COMMUNICATION FUNCTIONS **************************/




/***************************** COMMAND FUNCTIONS ******************************/
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


/*************************** END COMMAND FUNCTIONS ****************************/




/************************** CLOUD SAVING FUNCTIONS ****************************/

/**
 * \brief This function can be used to save a cloud of points (XYZ format) as a .bin file
 *
 * \param[in] pCloud_XYZ    : pointer to a buffer storing the XYZ cloud
 * \param[in] cloudSize     : number of points in the cloud
 * \param[in] pFileName     : path of the .bin result file
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code VN_SaveXYZCloudAsBinFile(float *pCloud_XYZ, int cloudSize, const char *pFileName);

/**
 * \brief This function can be used to save a cloud of points (XYZ format) as a .bin file
 *
 * \param[in] pCloud_XYZ    : pointer to a buffer storing the XYZ cloud
 * \param[in] cloudSize     : number of points in the cloud
 * \param[in] pFileName     : path of the .txt result file
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code VN_SaveXYZCloudAsTxtFile(float *pCloud_XYZ, int cloudSize, const char *pFileName);

/**
 * \brief This function can be used to save a cloud of points (XYZI16 format in this example) as a .pcd file
 *
 * \param[in] pCloud_XYZI6    : pointer to a buffer storing the XYZI16 cloud
 * \param[in] rowCount      : number of rows in the matrix
 * \param[in] columnCount   : number of columns in the matrix
 * \param[in] pFileName     : path of the .pcd result file
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code VN_SaveXYZI16CloudAsPcdFile(VN_Point_XYZI16 *pCloud_XYZI16,
                                         int rowsNumber,
                                         int columnsNumber,
                                         const char *pFileName);

/**
 * \brief This function can be used to save a matrix_XYZi cloud of points as a depth map bmp image
 *
 * \param[in] pCloud_XYZi   : pointer to a buffer storing the matrix
 * \param[in] rowCount      : number of rows in the matrix
 * \param[in] columnCount   : number of columns in the matrix
 * \param[in] pFileName     : path of the .bmp result file
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code VN_SaveXYZI8CloudAsDepthMapBmp(VN_Point_XYZI8 *pCloud_XYZI8, int rowCount, int columnCount, const char *pFileName);

/**
 * \brief This function can be used to save a matrix_XYZi cloud of points as a depth map bmp image
 *
 * \param[in] pCloud_XYZi   : pointer to a buffer storing the matrix
 * \param[in] rowCount      : number of rows in the matrix
 * \param[in] columnCount   : number of columns in the matrix
 * \param[in] pFileName     : path of the .bmp result file
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code VN_SaveXYZI8CloudAsDepthMapBmp(VN_Point_XYZI8 *pCloud_XYZI8, int rowCount, int columnCount, const char *pFileName);

/**
 * \brief This function can be used to save a matrix_XYZI8 cloud of points as an artifical image (using lum data only...)
 *
 * \param[in] pCloud_XYZI8   : pointer to a buffer storing the matrix
 * \param[in] rowCount         : number of rows in the matrix
 * \param[in] columnCount      : number of columns in the matrix
 * \param[in] pFileName        : path of the .bmp result file
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code VN_SaveXYZI8CloudAsLumBmp(VN_Point_XYZI8 *pCloud_XYZI8, int rowCount, int columnCount, const char *pFileName);

/**
 * \brief This function can be used to save a cloud of points (XYZI8 format) as a .pcd file,
 *  in this case a file with organized rows & columnds and the point data stored in binary format.
 *
 * \param[in] pCloud_XYZI8  : pointer to a buffer storing the XYZI8 cloud
 * \param[in] cloudSize     : number of points in the cloud
 * \param[in] pFileName     : path of the .pcd result file
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code VN_SaveXYZI8CloudAsPcdFile(VN_Point_XYZI8 *pCloud_XYZI8,
                                        int rowsNumber,
                                        int columnsNumber,
                                        const char *pFileName);

/**
 * \brief This function can be used to save a matrix_XYZRGB cloud of points as a depth map bmp image
 *
 * \param[in] pCloud_XYZRGB   : pointer to a buffer storing the matrix
 * \param[in] rowCount         : number of rows in the matrix
 * \param[in] columnCount      : number of columns in the matrix
 * \param[in] pFileName        : path of the .bmp result file
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code VN_SaveXYZRGBCloudAsDepthMapBmp(VN_Point_XYZRGB *pCloud_XYZRGB, int rowCount, int columnCount, const char *pFileName);


/**
 * \brief This function can be used to save a matrix_XYZRGB cloud of points as an artifical image (using lum data only...)
 *
 * \param[in] pCloud_XYZRGB   : pointer to a buffer storing the matrix
 * \param[in] rowCount         : number of rows in the matrix
 * \param[in] columnCount      : number of columns in the matrix
 * \param[in] pFileName        : path of the .bmp result file
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code VN_SaveXYZRGBCloudAsLumBmp(VN_Point_XYZRGB *pCloud_XYZRGB, int rowCount, int columnCount, const char *pFileName);


/**
 * \brief This function can be used to save a cloud of points (XYZRGB format) as a .pcd file,
 *  in this case a file with organized rows & columnds and the point data stored in binary format.
 *
 * \param[in] pCloud_XYZRGB  : pointer to a buffer storing the XYZRGB cloud
 * \param[in] rowCount         : number of rows in the matrix
 * \param[in] columnCount      : number of columns in the matrix
 * \param[in] pFileName     : path of the .pcd result file
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code VN_SaveXYZRGBCloudAsPcdFile(VN_Point_XYZRGB *pCloud_XYZRGB,
                                     int rowsNumber,
                                     int columnsNumber,
                                     const char *pFileName);
/************************ END CLOUD SAVING FUNCTIONS **************************/


/************************** CONFIGURATIONS FUNCTIONS ****************************/
/** @ingroup GeneralSetting
 * \brief This function is used to get Cirrus3D version protocol
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_UINT32 VN_Cirrus3DHandler_getProtocolVersion();

/** @ingroup GeneralSetting
 * \brief This function is used to initialize the VN_Cirrus3DHandler structure used by the get/set data functions and reset the data in
 * the CirrusSensor
 * \param[in]  IPAddress        : IP address of the Cirrus3D
 * \param[out] pCirrus3DHandler : Will contain the VN_Cirrus3DHandler structure initialized for the Cirrus3D with ip address=IPAddress
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_Cirrus3DHandler_Initialize(char *IPAddress,
                                           VN_Cirrus3DHandler *pCirrus3DHandler);

/** @ingroup GeneralSetting
 * \brief This function is used to force device version
 * \param[in] pCirrus3DHandler : Data previously initialized with the VN_Cirrus3DHandler_initialize() function
 * \param[in] NewDeviceVersion : Device version used to configure the cirrus3D.
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_Cirrus3DHandler_ForceVersionDevice(VN_Cirrus3DHandler *pCirrus3DHandler,
                                                   VN_UINT32 NewDeviceVersion);

/** @ingroup Config
 * \brief This function is used to reset the data in the CirrusSensor
 * \param[in] pCirrus3DHandler : Data previously initialized with the VN_Cirrus3DHandler_initialize() function
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_Cirrus3DHandler_ResetData(VN_Cirrus3DHandler *pCirrus3DHandler);

/** @ingroup Config
 * \brief This function is used to get the list of configuration datas
 * \param[in]  pCirrus3DHandler : Data previously initialized with the VN_Cirrus3DHandler_initialize() function
 * \param[out] pNbParamInList   : Number of data available
 * \param[out] DataList         : List of data available
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 * \brief Example :
 * \snippet exampleInDoc.c VN_Cirrus3DHandler_GetDataRootList example
 */
VN_tERR_Code VN_Cirrus3DHandler_GetDataRootList(const VN_Cirrus3DHandler *pCirrus3DHandler,
                                                VN_INT32 *pNbParamInList,
                                                VN_tDataRoot DataList[VN_cSizeList]);

/** @ingroup Config
 * \brief This function is used to get the data
 * \param[in] pCirrus3DHandler : Data previously initialized with the VN_Cirrus3DHandler_initialize() function
 * \param[in] DataId           : Data Index
 * \param[in] DataType         : Data Type
 * \param[out] pData           : Data Structure of DataType type.
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 * \brief Example :
 * \snippet exampleInDoc.c VN_Cirrus3DHandler_GetData example
 */
VN_tERR_Code VN_Cirrus3DHandler_GetData(const VN_Cirrus3DHandler *pCirrus3DHandler,
                                        VN_UINT32 DataId,
                                        VN_eDataType DataType,
                                        VN_tDataRoot* pData);

/** @ingroup Config
 * \brief This function is used to get the data of type INT32
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
 * \param[in] pCirrus3DHandler : Data previously initialized with the VN_Cirrus3DHandler_initialize() function
 * \param[in,out] pData        : Get the value of the data. The structure was initialized by calling VN_Cirrus3DHandler_GetData(), VN_Cirrus3DHandler_GetDataInt32(), VN_Cirrus3DHandler_GetDataReal32() or VN_Cirrus3DHandler_GetDataString() function
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 * \brief Example :
 * \snippet exampleInDoc.c VN_Cirrus3DHandler_GetDataValue example
 */
VN_tERR_Code VN_Cirrus3DHandler_GetDataValue(const VN_Cirrus3DHandler *pCirrus3DHandler,
                                             VN_tDataRoot* pData);

/** @ingroup Config
 * \brief This function is used to get the data value of type INT32
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
 * \param[in] pCirrus3DHandler : Data previously initialized with the VN_Cirrus3DHandler_initialize() function
 * \param[in,out] pData        : Get the data value. The structure was initialized by calling VN_Cirrus3DHandler_GetData(), VN_Cirrus3DHandler_GetDataInt32(), VN_Cirrus3DHandler_GetDataReal32() or VN_Cirrus3DHandler_GetDataString() function
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_Cirrus3DHandler_SetDataValue(const VN_Cirrus3DHandler *pCirrus3DHandler,
                                             VN_tDataRoot* pData);

/** @ingroup Config
 * \brief This function is used to get the data value
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

/** @ingroup GeneralSetting
 * \brief This function is used to get Cirrus3D available
 * \param[in,out] pNbCirrus3DAvailable : Number
 * \param[in,out] ipCirrus3DAvailable  : NULL pointer,
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_GetCirrus3DAvailable(VN_INT32 *pNbCirrus3DAvailable, char ipCirrus3DAvailable[VN_cMaxNbCirrus3D][VN_cSizeDataString]);

#endif //VN_CIRRUS3D_API_H

