#include "VN_SDK_CommunicationFunctions.h"

/*------------------------------------------------------------------------------
 *  History:
 *  16/02/2024: NEW: new function CirrusCom_ReceiveBufferofSizeN............ P.H
 *  16/02/2024: CHANGED: remove CirrusCom_ReceiveCommandWithSize function... P.H
 *  16/05/2024: NEW: new function VN_Cirrus3DHandler_getHeaderVersion
 *                                CirrusCom_setTimeout...................... P.H
 *------------------------------------------------------------------------------
 */

/***************************** API FUNCTIONS ******************************/
/** @ingroup General
 * \brief This function is used to get Cirrus3D version protocol
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_UINT32 VN_Cirrus3DHandler_getProtocolVersion()
{
    return VN_cProtocolVersion;
}

/** @ingroup General
 * \brief This function is used to get Cirrus3D version header
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_UINT32 VN_Cirrus3DHandler_getHeaderVersion(void)
{
    return VN_cHeaderVersion;
}

/** @ingroup CommunicationTool
 * \brief Enable connection protocol for Windows OS
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_CirrusCom_Init(void)
{
#ifdef WIN32
    WSADATA WSAData;
    int err=WSAStartup(MAKEWORD(2,2), &WSAData);
    if(err != 0)
    {
        printf("WSAStartup failed !\n");
        return VN_eERR_WSAStartup;
    }
#endif
    return VN_eERR_NoError;
}

/** @ingroup CommunicationTool
 * \brief Cleanly end connection protocol for Windows OS
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_CirrusCom_End(void)
{
#ifdef WIN32
    int err=WSACleanup();
    if(err != 0)
    {
        printf("WSACleanup failed !\n");
        return VN_eERR_WSACleanup;
    }
#endif
    return VN_eERR_NoError;
}

/**************************************************************************/

/**
 * \brief start connection with the Cirrus
 *
 * \param[in] address    : Cirrus addressIP
 * \param[in] pSock      : Cirrus connection VN_SOCKET.
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code CirrusCom_connect(const char *pIpAddress,
                               VN_SOCKET* pSock)
{

#ifndef  _APPLICATION_CSA

    *pSock = socket(PF_INET, SOCK_STREAM, 0);
    SOCKADDR_IN sin = { 0 };
    struct hostent *hostinfo;

    if(*pSock == INVALID_VN_SOCKET)
    {
        printf("Error : invalid VN_SOCKET()\n");
        return VN_eERR_SocketError;
    }

    hostinfo = gethostbyname(pIpAddress);
    if (hostinfo == NULL)
    {
        fprintf (stderr, "Unknown host %s.\n", pIpAddress);
        return VN_eERR_SocketError;
    }

    sin.sin_addr = *(IN_ADDR *) hostinfo->h_addr;
    sin.sin_port = htons(20001);
    sin.sin_family = PF_INET;

    //printf("Trying connection from IPaddress %s on port 20001\n", pIpAddress);
    if(connect(*pSock,(SOCKADDR *) &sin, sizeof(SOCKADDR)) == VN_SOCKET_ERROR)
    {
        printf("Connection from IPaddress %s on port 20001 failed\n",pIpAddress);
        return VN_eERR_SocketConnectError;
    }
    //printf("Connection successful\n");

    return VN_eERR_NoError;
#else
    return VN_eERR_Failure;
#endif
}

/**
 * \brief define a timeout on the VN_SOCKET, to make sure we can't get stuck in an infinite loop
 *
 * \param[in] sock    : Cirrus connection VN_SOCKET.
 * \param[in] timeout : VN_SOCKET timeout in second.
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code CirrusCom_setTimeout(VN_SOCKET sock,
                                  VN_INT32 timeout)
{
#ifdef WIN32
    int timeout=30000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout,sizeof(timeout));
#else
    struct timeval tv;
    tv.tv_sec=timeout;
    tv.tv_usec=0;

    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval));
#endif

    return VN_eERR_NoError;
}

/**
 * \brief start connection with the Cirrus
 *
 * \param[in] address    : Cirrus addressIP
 * \param[in] port       : Cirrus port
 * \param[in] pSock      : Cirrus connection VN_SOCKET.
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code CirrusCom_connectToPort(const char *pIpAddress,
                                     int port,
                                     VN_SOCKET* pSock)
{
#ifndef  _APPLICATION_CSA
    *pSock = socket(PF_INET, SOCK_STREAM, 0);
    SOCKADDR_IN sin = { 0 };
    struct hostent *hostinfo;

    if(*pSock == INVALID_VN_SOCKET)
    {
        printf("Error : invalid VN_SOCKET()\n");
        return VN_eERR_SocketError;
    }

    hostinfo = gethostbyname(pIpAddress);
    if (hostinfo == NULL)
    {
        fprintf (stderr, "Unknown host %s.\n", pIpAddress);
        return VN_eERR_SocketError;
    }

    sin.sin_addr = *(IN_ADDR *) hostinfo->h_addr;
    sin.sin_port = htons(port);
    sin.sin_family = PF_INET;

    //printf("Trying connection from IPaddress %s on port %d\n", address, port);
    if(connect(*pSock,(SOCKADDR *) &sin, sizeof(SOCKADDR)) == VN_SOCKET_ERROR)
    {
        printf("Connection from IPaddress %s on port %d failed\n",pIpAddress, port);
        return VN_eERR_SocketConnectError;
    }
    //printf("Connection successful\n");

    return VN_eERR_NoError;
#else
    return VN_eERR_Failure;
#endif
}

/**
 * \brief start connection with the Cirrus on port dedicated to KAST command
 *
 * \param[in] address    : Cirrus addressIP
 * \param[in] pSock      : Cirrus connection VN_SOCKET.
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code CirrusCom_KAST_connect(const char *address,
                                    VN_SOCKET* pSock)
{
#ifndef  _APPLICATION_CSA
    *pSock = socket(AF_INET, SOCK_STREAM, 0);
    SOCKADDR_IN sin = { 0 };
    struct hostent *hostinfo;

    if(*pSock == INVALID_VN_SOCKET)
    {
        printf("Error : invalid VN_SOCKET()\n");
        return VN_eERR_SocketError;
    }

    hostinfo = gethostbyname(address);
    if (hostinfo == NULL)
    {
        fprintf (stderr, "Unknown host %s.\n", address);
        return VN_eERR_SocketError;
    }

    sin.sin_addr = *(IN_ADDR *) hostinfo->h_addr;
    sin.sin_port = htons(23001);
    sin.sin_family = AF_INET;

    //printf("Trying connection from IPaddress %s on port 20001\n", address);
    if(connect(*pSock,(SOCKADDR *) &sin, sizeof(SOCKADDR)) == VN_SOCKET_ERROR)
    {
        printf("Connection from IPaddress %s on port 20001 failed\n",address);
        return VN_eERR_SocketConnectError;
    }
    //printf("Connection successful\n");

    return VN_eERR_NoError;
#else
    return VN_eERR_Failure;
#endif
}

/**
 * \brief end connection with the Cirrus
 *
 * \param[in] Sock      : Cirrus connection VN_SOCKET.
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code CirrusCom_End_connection(VN_SOCKET sock)
{
    int error=closeVNSocket(sock);

    if(error != 0)
    {
        printf("Error when closing VN_SOCKET()\n");
        return VN_eERR_SocketCloseError;
    }
    return VN_eERR_NoError;
}


/**
 * \brief send command to Cirrus
 *
 * \param[in] Sock      : Cirrus connection VN_SOCKET.
 * \param[in] buffer    : Cirrus command.
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code CirrusCom_SendCommand(VN_SOCKET sock, const char *buffer)
{
    if(send(sock, buffer, strlen(buffer), 0) == -1)
    {
        printf("Error in CirrusCom_SendCommand when trying to send command %s\n",buffer);
        return VN_eERR_SocketSendError;
    }
    //each command is supposed to end with CR
    //    const char CR[2]="\r\n";
    //    if(send(sock, CR, strlen(CR), 0) == -1)
    //    {
    //        printf("Error in CirrusCom_SendCommand when trying to send trailing CR\n");
    //        return VN_eERR_SocketSendError;
    //    }
    return VN_eERR_NoError;
}


/**
 * \brief send command to Cirrus
 *
 * \param[in] Sock      : Cirrus connection VN_SOCKET.
 * \param[in] buffer    : Cirrus command.
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code CirrusCom_SendCommandWithSize(VN_SOCKET sock, const char *buffer, int sizeBuffer)
{
    if(send(sock, buffer, sizeBuffer, 0) == -1)
    {
        printf("Error in CirrusCom_SendCommand when trying to send command %s\n",buffer);
        return VN_eERR_SocketSendError;
    }
    //each command is supposed to end with CR
    //    const char CR[2]="\r\n";
    //    if(send(sock, CR, strlen(CR), 0) == -1)
    //    {
    //        printf("Error in CirrusCom_SendCommand when trying to send trailing CR\n");
    //        return VN_eERR_SocketSendError;
    //    }
    return VN_eERR_NoError;
}

/**
 * \brief CirrusCom_ReceiveCommandWithSize
 *
 * \param[in] Sock      : Cirrus connection VN_SOCKET.
 * \param[in] buffer    : Cirrus command.
 * \param[in] sizeBuffer: buffer size.
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code CirrusCom_SendAckCommandWithSize(VN_SOCKET sock, const char *buffer, int sizeBuffer)
{
    if(send(sock, buffer, sizeBuffer, 0) == -1)
    {
        printf("Error in CirrusCom_SendCommand when trying to send command %s\n",buffer);
        return VN_eERR_SocketSendError;
    }

    return VN_eERR_NoError;
}

/**
 * \brief CirrusCom_ReceiveBufferofSizeN
 *
 * \param[in] Sock      : Cirrus connection VN_SOCKET.
 * \param[in] buffer    : Cirrus command.
 * \param[in] sizeBuffer: buffer size to read.
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code CirrusCom_ReceiveBufferofSizeN(VN_SOCKET sock, char *buffer, int sizeBuffer)
{
    int bytesRead=0;
    while (bytesRead < sizeBuffer)
    {
        //The expected response from the Cirrus is the command code followed by a binary status then the cloud of points
        int rVal = recv(sock, buffer, sizeBuffer-bytesRead, 0);
        if (rVal==-1)
        {//a return value of -1 indicate a VN_SOCKET error
            printf("Error in CirrusCom_ReceiveBufferofSizeN command : VN_SOCKET error when receiving command result\n");
            CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
            return VN_eERR_SocketRecvError;
        }

        if (rVal != 0)
        {
            bytesRead += rVal;
            buffer += rVal;
        }
        else
        {
            //with the large timeout defined, not receiving any bytes here means
            // a com failure...
            printf("Error in CirrusCom_ReceiveBufferofSizeN function : incomplete com\n");
            CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
            return VN_eERR_SocketIncompleteCom;
        }
    }

    return VN_eERR_NoError;
}
