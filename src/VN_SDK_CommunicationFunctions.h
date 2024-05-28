#ifndef VN_SDK_COMMUNICATIONFUNCTIONS_H
#define VN_SDK_COMMUNICATIONFUNCTIONS_H
/*------------------------------------------------------------------------------
 *  History:
 *  16/02/2024: NEW: new function CirrusCom_ReceiveBufferofSizeN............ P.H
 *  16/02/2024: CHANGED: remove CirrusCom_ReceiveCommandWithSize function... P.H
 *------------------------------------------------------------------------------
 */

#include "VN_SDK_Constant.h"

/***************************** API FUNCTIONS ******************************/
 /** @ingroup General
 * \brief This function is used to get Cirrus3D version protocol
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_UINT32 VN_Cirrus3DHandler_getProtocolVersion(void);

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

/**************************************************************************/

/**
 * \brief start connection with the Cirrus
 *
 * \param[in] address    : Cirrus addressIP
 * \param[in] pSock      : Cirrus connection VN_SOCKET.
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code CirrusCom_connect(const char *address,
                               VN_SOCKET* pSock);

/**
 * \brief start connection with the Cirrus
 *
 * \param[in] address    : Cirrus addressIP
 * \param[in] port       : Cirrus port
 * \param[in] pSock      : Cirrus connection VN_SOCKET.
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code CirrusCom_connectToPort(const char *address,
                                     int port,
                                     VN_SOCKET* pSock);

/**
 * \brief start connection with the Cirrus on port dedicated to KAST command
 *
 * \param[in] address    : Cirrus addressIP
 * \param[in] pSock      : Cirrus connection VN_SOCKET.
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code CirrusCom_KAST_connect(const char *address,
                                    VN_SOCKET* pSock);

/**
 * \brief end connection with the Cirrus
 *
 * \param[in] Sock      : Cirrus connection VN_SOCKET.
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code CirrusCom_End_connection(VN_SOCKET sock);


/**
 * \brief send command to Cirrus
 *
 * \param[in] Sock      : Cirrus connection VN_SOCKET.
 * \param[in] buffer    : Cirrus command.
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code CirrusCom_SendCommand(VN_SOCKET sock, const char *buffer);


/**
 * \brief send command to Cirrus
 *
 * \param[in] Sock      : Cirrus connection VN_SOCKET.
 * \param[in] buffer    : Cirrus command.
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code CirrusCom_SendCommandWithSize(VN_SOCKET sock, const char *buffer, int sizeBuffer);

/**
 * \brief CirrusCom_SendAckCommandWithSize
 *
 * \param[in] Sock      : Cirrus connection VN_SOCKET.
 * \param[in] buffer    : Cirrus command.
 * \param[in] sizeBuffer: buffer size.
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code CirrusCom_SendAckCommandWithSize(VN_SOCKET sock, const char *buffer, int sizeBuffer);

/**
 * \brief CirrusCom_ReceiveBufferofSizeN
 *
 * \param[in] Sock      : Cirrus connection VN_SOCKET.
 * \param[in] buffer    : Cirrus command.
 * \param[in] sizeBuffer: buffer size to read.
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code CirrusCom_ReceiveBufferofSizeN(VN_SOCKET sock, char *buffer, int sizeBuffer);

#endif // VN_SDK_COMMUNICATIONFUNCTIONS_H
