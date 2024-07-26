/*
 *  History:
 * 16/05/2024 ADDED: VN_cSocketTimeout enum................................. P.H
 * 16/05/2024 CHANGED: VN_Point_XYZI8 is declared without padding
 *            CHANGED: VN_Point_XYZI16 is declared without padding.......... P.H
 * 04/07/2024 ADDED: VN_cSmallSocketTimeout................................. P.H
 *------------------------------------------------------------------------------
 */

#ifndef CONSTANT_H
#define CONSTANT_H

#include <stdio.h>
#include <stdlib.h> //for atoi

typedef unsigned char VN_UINT8;
typedef unsigned short VN_UINT16;
typedef unsigned int VN_UINT32;
typedef int VN_INT32;
typedef int VN_BOOL;
typedef float VN_REAL32;
typedef int VN_SOCKET;

//For communication protocol :
#ifdef WIN32
#include "winsock.h"
#define closeVNSocket(s) closesocket(s)
#elif  WIN64
#include "winsock.h"
#define closeVNSocket(s) closesocket(s)
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <netdb.h>
#include <string.h>
typedef struct sockaddr_in SOCKADDR_IN;
typedef struct sockaddr SOCKADDR;
typedef struct in_addr IN_ADDR;
#define closeVNSocket(s) close(s)
#endif

#define INVALID_VN_SOCKET -1
#define VN_SOCKET_ERROR -1
#define cPortConfiguration 20002

/********************************** GENERAL GLOBALS **********************************/
/** @ingroup GeneralConst
 *  @def VN_cProtocolVersion
 *  @brief Protocol communication version
 */
#define VN_cProtocolVersion 2

/** @ingroup GeneralConst
 *  @def VN_cHeaderVersion
 *  @brief Header version
 */
#define VN_cHeaderVersion 1


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


/** @ingroup GeneralConst
 *  @def VN_cSocketTimeout
 *  @brief VN_Socket timeout value of socket VN_Socket used by recv function. Value in second.
 */
#define VN_cSocketTimeout 20

/** @ingroup GeneralConst
 *  @def VN_cSmallSocketTimeout
 *  @brief VN_Socket timeout value of socket VN_Socket used by recv function. Value in second.
 */
#define VN_cSmallSocketTimeout 2
/*******************************************************************************************/

/********************************** COMMUNICATION GLOBALS **********************************/
/** @ingroup CommunicationTool
*   @enum VN_tERR_Code
*   @brief List error code returned by the SDK function.
*/
typedef enum _VN_tERR_Code{
    VN_eERR_NoError=0, /*!< @brief The request succeeded. (0) */
    VN_eERR_UndefinedError, /*!< @brief The sensor has encountered a situation it does not know how to handle. (1) */
    VN_eERR_WSAStartup,/*!< @brief The function WSAStartup returned an error. The WSAStartup function initiates use of the Winsock DLL by a process. (2) */
    VN_eERR_WSACleanup,/*!< @brief The function WSACleanup returned an error. The WSACleanup function terminates use of the Winsock 2 DLL (Ws2_32.dll). (3) */
    VN_eERR_SocketError,/*!< @brief The socket is invalid. (4)*/
    VN_eERR_SocketConnectError,/*!< @brief The connection to the sensor failed. (5)*/
    VN_eERR_SocketSendError,/*!< @brief Sending data to the sensor failed. (6)*/
    VN_eERR_SocketCloseError,/*!< @brief Closing the connection with the sensor failed. (7)*/
    VN_eERR_SocketRecvError,/*!< @brief Data was not received. (8)*/
    VN_eERR_SocketIncompleteCom,/*!< @brief The number of data received is not as expected. (9)*/
    VN_eERR_Failure,/*!< @brief The sensor has encountered a situation it does not know how to handle. (10)*/
    VN_eERR_UnknownCommand,/*!< @brief The request method is not supported by the Cirrus3D. (11)*/
    VN_eERR_InsufficientMemory,/*!< @brief Insufficient memory to create the requested buffer or the received cloud of points exceeds the maximum allowable size. (12)*/
    VN_eERR_Nullpointer,/*!< @brief A null pointer detected. (13)*/
    VN_eERR_BadHeader,/*!< @brief The server cannot or will not process the request due to something that is perceived to be a client error. (14)*/
    VN_eERR_FileAccessError,/*!< @brief Unable to access the file (15)*/
    VN_eERR_FileWriteError,/*!< @brief Unable to write in the file (16)*/
    VN_eERR_BadStatus,/*!< @brief Bad Status (17)*/
    VN_eERR_BadNetworkConfig,/*!< @brief Bad Network Config (18)*/
    VN_eERR_UnknownData, /*!< @brief The requested data is not known by the Cirrus3D (19)*/
    VN_eERR_ValueOutOfBounds, /*!< @brief The value of the data is out of Ranges (20)*/
    VN_eERR_UnknownDataType, /*!< @brief The requested data type is not known by the Cirrus3D (21)*/
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
//Cloud of points structures
#define VN_cMaxCOPSize 6000000//max credible size of a cloud of points for led or laser Cirrus scanner

/** @ingroup CloudOfPoint
  * @struct VN_Point_XYZI16
  * @brief Structure containing a point of type XYZI16
  * @param x: coordinate x (32-bit in single-precision IEEE-754 binary floatingpoint format)
  * @param y: coordinate y (32-bit in single-precision IEEE-754 binary floatingpoint format)
  * @param z: coordinate z (32-bit in single-precision IEEE-754 binary floatingpoint format)
  * @param i: intensity (16-bit unsigned integer)
  */
#pragma pack(push,1) // Pushes the current alignment setting on an internal stack and then sets the new alignment.
                     // Use the flag WITH_PADDING if #pragma directives is not compatible with your compiler.
typedef struct _VN_Point_XYZI16
{
    VN_REAL32 x;
    VN_REAL32 y;
    VN_REAL32 z;
    VN_UINT16 i;
}VN_Point_XYZI16;
#pragma pack(pop) //restores the alignment setting to the one saved at the top of the internal stack (and removes that stack entry).

/** @ingroup CloudOfPoint
  * @struct VN_Point_XYZI8
  * @brief Structure containing a point of type XYZI8
  * @param x: coordinate x (32-bit in single-precision IEEE-754 binary floating-point format)
  * @param y: coordinate y (32-bit in single-precision IEEE-754 binary floating-point format)
  * @param z: coordinate z (32-bit in single-precision IEEE-754 binary floating-point format)
  * @param i: intensity (8-bit unsigned integer)
  */
#pragma pack(push,1) // Pushes the current alignment setting on an internal stack and then sets the new alignment.
                     // Use the flag WITH_PADDING if #pragma directives is not compatible with your compiler.
typedef struct _VN_Point_XYZI8
{
    VN_REAL32 x;
    VN_REAL32 y;
    VN_REAL32 z;
    VN_UINT8 i;
}VN_Point_XYZI8;
#pragma pack(pop) //restores the alignment setting to the one saved at the top of the internal stack (and removes that stack entry).

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
    VN_eTypeInt32, /*!< @brief int*/
    VN_eTypeReal32, /*!< @brief float*/
    VN_eTypeString /*!< @brief char[]*/
}VN_eDataType;


/** @ingroup Config
 *  @enum VN_eAvailableStatus
 *  @brief Data status in the Cirrus3D
 */
typedef enum _VN_eAvailableStatus{
    VN_eAvailable=0, /*!< @brief Data available*/
    VN_eDeprecated, /*!< @brief Data will be removed in the futur version*/
    VN_eObsolete, /*!< @brief Data obsolete*/
}VN_eAvailableStatus;

/** @ingroup Config
 *  @struct VN_tDataRoot
 *  @brief Read-Only Structure contains the properties of the data. The structure is initialized by calling VN_Cirrus3DHandler_getData(), VN_Cirrus3DHandler_getDataInt32(), VN_Cirrus3DHandler_getDataReal32() or VN_Cirrus3DHandler_getDataString() function.
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
 *  @brief Read-Only Structure contains information of the data. The structure is initialized by calling VN_Cirrus3DHandler_getData(), VN_Cirrus3DHandler_getDataInt32(), VN_Cirrus3DHandler_getDataReal32() or VN_Cirrus3DHandler_getDataString() function.
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
 *  @brief Structure contains the value and the information data of type VN_INT32. The structure is initialized by calling VN_Cirrus3DHandler_getDataRootList() or VN_Cirrus3DHandler_getDataInt32() function.
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
 *  @brief Structure contains the value and the information data of type VN_REAL32. The structure is initialized by calling VN_Cirrus3DHandler_getDataRootList() or VN_Cirrus3DHandler_getDataReal32() function.
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
 *  @brief Structure contains the value and the information data of type VN_REAL32. The structure is initialized by calling VN_Cirrus3DHandler_getDataRootList() or VN_Cirrus3DHandler_getDataString() function.
 * @param dataInfo: [Read only] Information data
 * @param value: [read/Write] Current value
 */
typedef struct _VN_tDataString{
    VN_tDataInfo dataInfo; /*!< @brief [Read only] */
    char value[VN_cSizeDataString];/*!< @brief [read/write] */
}VN_tDataString;

/*********************** END CONFIGURATION STRUCTURES AND CONSTANTS **************************/

#endif // CONSTANT_H
