#ifndef VN_SDK_COMMUNICATIONFUNCTIONS_H
#define VN_SDK_COMMUNICATIONFUNCTIONS_H
/*------------------------------------------------------------------------------
 *  History:
 *  16/02/2024: NEW: new function CirrusCom_ReceiveBufferofSizeN............ P.H
 *  16/02/2024: CHANGED: remove CirrusCom_ReceiveCommandWithSize function... P.H
 *  16/05/2024: NEW: new function VN_Cirrus3DHandler_getHeaderVersion
 *                                CirrusCom_setTimeout...................... P.H
 *  28/11/2024: NEW: new function CirrusCom_Receive......................... P.H
 *  26/03/2025 ADDED: Export the functions available when using the library. P.H
 *------------------------------------------------------------------------------
 */

#include "VN_SDK_Constant.h"
#include <errno.h>


/***************************** API FUNCTIONS ******************************/
 /** @ingroup General
 * \brief This function is used to get Cirrus3D version protocol
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
EXPORT VN_UINT32 VN_Cirrus3DHandler_getProtocolVersion(void);

/** @ingroup General
 * \brief This function is used to get Cirrus3D version header
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
EXPORT VN_UINT32 VN_Cirrus3DHandler_getHeaderVersion(void);

/** @ingroup CommunicationTool
 * \brief Enable connection protocol for Windows OS
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
EXPORT VN_tERR_Code VN_CirrusCom_Init(void);

/** @ingroup CommunicationTool
 * \brief Cleanly end connection protocol for Windows OS
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
EXPORT VN_tERR_Code VN_CirrusCom_End(void);

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
 * \brief define a timeout on the VN_SOCKET, to make sure we can't get stuck in an infinite loop
 *
 * \param[in] sock    : Cirrus connection VN_SOCKET.
 * \param[in] timeout : VN_SOCKET timeout in second.
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code CirrusCom_setTimeout(VN_SOCKET sock,
                                  VN_INT32 timeout);

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


/**
 * \brief CirrusCom_Receive
 *
 * \param[in] Sock      : Cirrus connection VN_SOCKET.
 * \param[in] pBuffer   : Cirrus command.
 * \param[in] bufferLen : buffer max size to read.
 * \param[in] flags     : recv command flags.
 * \retval number of bytes received if success
 * \retval -1 if an error occurred
 */
static inline int CirrusCom_Receive(VN_SOCKET socket, void *pBuffer, size_t bufferLen, int flags)
{
#ifdef _WIN32
    return recv(socket,(char*)(pBuffer), bufferLen,flags);
#else
    int rval=0;
    do{
        rval = recv(socket,pBuffer,bufferLen,flags);
    }while (rval==-1 && errno==EINTR);//i.e. if recv interrupted by "system call interrupt" resume automatically
    return rval;
#endif
}

#endif // VN_SDK_COMMUNICATIONFUNCTIONS_H
