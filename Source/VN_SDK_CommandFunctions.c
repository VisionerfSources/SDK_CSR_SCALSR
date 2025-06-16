/*-------------------------------------------------------------------------------
 *  History:
 * 24/01/2023 FIX: comparison of integer expressons of different signedness.. P.H
 * 16/02/2024: NEW: new function VN_Cirrus3DHandler_ResetData................ P.H
 * 16/02/2024: NEW: new VN_GetCirrus3DAvailable.............................. P.H
 * 16/02/2024: CHANGED added output parameter(status) in VN_ExecuteCommandSTS P.H
 * 16/02/2024: FIXED in VN_Cirrus3DHandler_Initialize serialNumber is 
 *             not initialized............................................... P.H
 * 16/02/2024: NEW: new function VN_GetCirrus3DAvailable..................... P.H
 * 12/03/2024 FIXED wrong KAST temperature printed in VN_ExecuteCommandKAST.. F.R
 * 11/04/2024 NEW: new function VN_ExecuteSCAN_CameraCOP_XYZRGB.............. P.H
 * 11/04/2024 NEW: new function VN_Cirrus3DHandler_Get....................... P.H
 * 11/04/2024 CHANGED VN_ExecuteSCAN_CameraCOP_XYZRGB added param MiddleCam.. F.R
 * 16/05/2024 NEW: VN_ExecuteSCAN_readHeader
 *                 VN_ExecuteSCAN_CameraCOP_XYZI8
 *                 VN_ExecuteSCAN_CameraCOP_XYZI8_sampling
 *                 VN_ExecuteSCAN_Matrix_XYZRGB_readHeader
 *            CHANGED: Multiple code improvement changes (removal of mallocs,
 *  better use of timeouts when receiving...)
 *            REMOVE: VN_ExecuteSCAN_CameraCOP_XYZRGB....................... P.H
 * 17/05/2024 NEW function VN_ExecuteSCAN_CameraCOP_MiddleCam2_XYZI......... F.R
 * 24/05/2024 NEW: VN_ExecuteSCAN_CameraCOP_XYZI8_s
 *                 VN_ExecuteSCAN_CameraCOP_XYZI8_sampling_s................ P.H
 * 04/06/2024 CHANGED Compatibility of different versions CSA<->SDK: Improved
 *   error handling......................................................... P.H
 * 24/07/2024 NEW VN_ExecuteSCAN_DepthMap function.......................... P.H
 * 28/11/2024 CHANGED: use CirrusCom_Receive function instead of recv....... P.H
 * 16/06/2025 FIXED: In VN_Cirrus3DHandler_GetData and
 *   VN_Cirrus3DHandler_SetDataValue functions, properly close the socket in
 *   case of an error....................................................... P.H
 *-------------------------------------------------------------------------------
 */

#include "VN_SDK_CommandFunctions.h"
#include "VN_SDK_CommunicationFunctions.h"

/***************************** API FUNCTIONS ******************************/
/** @ingroup CloudOfPoint
 * \brief Get status of Cirrus3D (ready or not)
 * \param[in] IPAddress : IP address of the Cirrus3D
 * \param[out] pStatus  : Status of Cirrus3D (= 0 if ready)
 * \retval VN_eERR_NoError if success (i.e. Cirrus ready)
 * \retval Error code otherwise
 */
VN_tERR_Code VN_ExecuteCommandSTS(const char *pIPAddress,
                                  int *pStatus)
{
    VN_SOCKET sock;
    VN_tERR_Code err=VN_eERR_NoError;
    char STSCommand[10];

    printf("Starting STS command at address %s\n",pIPAddress);

    //Open a VN_SOCKET
    err=CirrusCom_connect(pIPAddress,&sock);
    if (err!=VN_eERR_NoError)
        return err;

    err=CirrusCom_setTimeout(sock,
                             VN_cSocketTimeout);
    if (err!=VN_eERR_NoError)
        return err;

    //send the STS command to the Cirrus
    sprintf(STSCommand, "STS 0%c%c",13,10);
    err=CirrusCom_SendCommand(sock, STSCommand);
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

    //get and interpret the response from the Cirrus
    {
        int nbBytesRead;
        char  command[4];

        //The expected response from the Cirrus is the command code followed by the status
        //Note : there can be some loading time depending on the selected configuration, max 500ms...
        nbBytesRead=CirrusCom_Receive(sock, command, 3, 0);
        if(nbBytesRead == -1)
        {//a return value of -1 indicate a VN_SOCKET error
            printf("VN_SOCKET error during STS command recv()\n");
            //end the communication properly even if the commands fails
            CirrusCom_End_connection(sock);
            return VN_eERR_SocketRecvError;
        }
        else if (nbBytesRead != 3)
        {//we are supposed to read exactly 3 bytes here
            printf("Error during STS command recv : insufficient number of bytes (wanted %d, received %d)\n",3,nbBytesRead);
            CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
            return VN_eERR_SocketIncompleteCom;
        }

        if(command[0]!='S' || command[1]!='T' || command[2]!='S')
        {
            if(command[0]=='E' && command[1]=='R' && command[2]=='R')
            {
                printf("Recv: ERR -> Unknown command");
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_UnknownCommand;
            }
            else
            {
                command[3]='\0';
                printf("Recv: %s -> communication error", command);
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_Failure;
            }
        }
        else
        {
            const int nbBytesToRead=1024;
            char buffer[1024];

            //read the rest of the response : all the data sent by the Cirrus until it closes the VN_SOCKET
            nbBytesRead=CirrusCom_Receive(sock, (char*)buffer, sizeof(buffer), 0);
            if(nbBytesRead == -1)
            {//a return value of -1 indicate a VN_SOCKET error
                printf("Error in STS command : VN_SOCKET error when receiving command result\n");
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_SocketRecvError;
            }

            if (nbBytesRead<nbBytesToRead)
                buffer[nbBytesRead]='\0';
            else
                buffer[nbBytesToRead-1]='\0';

            *pStatus = atoi(strtok(buffer, ","));
            if (*pStatus!=0)
            {//expected status is 0 if the command was executed properly, error code otherwise
                printf("Failed STS command with status %d",*pStatus);
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_Failure;
            }
        }
    }

    //if reached here : command successful, close VN_SOCKET properly
    err=CirrusCom_End_connection(sock);
    if (err!=VN_eERR_NoError)
        return err;

    return VN_eERR_NoError;
}


/** @ingroup GeneralSetting
 * \brief Request its information from the Cirrus3DSensor
 * \param[in] IPAddress: IP address of the Cirrus3DSensor
 * \param[in] InfoType: 1 to request Cirrus3DSensor type/size, 2 to request Cirrus3DSensor serial number
 * \param[out] pResult: Char array of size at least 64 to store the requested info
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_ExecuteCommandINFO(const char *pIpAddress,
                                   int InfoType,
                                   char *pResult)
{
    VN_SOCKET sock;
    VN_tERR_Code err=VN_eERR_NoError;
    char infoCommand[10];

    //Open a VN_SOCKET
    err=CirrusCom_connect(pIpAddress,&sock);
    if (err!=VN_eERR_NoError)
        return err;

    err=CirrusCom_setTimeout(sock,
                             VN_cSocketTimeout);
    if (err!=VN_eERR_NoError)
        return err;

    //send the STS command to the Cirrus
    sprintf(infoCommand, "INFO %d%c%c", InfoType, 13, 10);
    err=CirrusCom_SendCommand(sock, infoCommand);
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

    //get and interpret the response from the Cirrus
    {
        int nbBytesRead;
        char  command[5];

        //The expected response from the Cirrus is the command code followed by the status and the result string
        nbBytesRead=CirrusCom_Receive(sock, command, 4, 0);
        if(nbBytesRead == -1)
        {//a return value of -1 indicate a VN_SOCKET error
            printf("VN_SOCKET error during INFO command recv()\n");
            //end the communication properly even if the commands fails
            CirrusCom_End_connection(sock);
            return VN_eERR_SocketRecvError;
        }
        else if (nbBytesRead != 4)
        {//we are supposed to read exactly 4 bytes here (sent as a single packet)
            printf("Error during INFO command recv : insufficient number of bytes (wanted %d, received %d)\n",4,nbBytesRead);
            CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
            return VN_eERR_SocketIncompleteCom;
        }

        if(command[0]!='I' || command[1]!='N' || command[2]!='F' || command[3]!='O')
        {
            if(command[0]=='E' && command[1]=='R' && command[2]=='R')
            {
                printf("Recv: ERR -> Unknown command");
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_UnknownCommand;
            }
            else
            {
                command[4]='\0';
                printf("Recv: %s -> communication error", command);
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_Failure;
            }
        }
        else
        {
            const int nbBytesToRead=1024;
            char buffer[1024];

            //read the rest of the response : all the data sent by the Cirrus until it closes the VN_SOCKET
            nbBytesRead=CirrusCom_Receive(sock, (char*)buffer, sizeof(buffer), 0);
            if(nbBytesRead == -1)
            {//a return value of -1 indicate a VN_SOCKET error
                printf("Error in INFO command : VN_SOCKET error when receiving command result\n");
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_SocketRecvError;
            }

            if (nbBytesRead<nbBytesToRead)
                buffer[nbBytesRead]='\0';
            else
                buffer[nbBytesToRead-1]='\0';

            int status=atoi(strtok(buffer, ","));
            if (status!=0)
            {//expected status is 0 if the command was executed properly, error code otherwise
                printf("Failed INFO command with status %d",status);
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_Failure;
            }
            else
            {//extract string response from returned data
                char *pCh=strtok(NULL, "\r\n");//each command end with a carriage return
                memcpy(pResult,pCh,64);
                pResult[63]='\0';//just in case
            }
        }
    }

    //if reached here : command successful, close VN_SOCKET properly
    err=CirrusCom_End_connection(sock);
    if (err!=VN_eERR_NoError)
        return err;

    return VN_eERR_NoError;
}

/** @ingroup CloudOfPoint
 * \brief Command the Cirrus3DSensor to load the specified scan configuration
 * \param[in] IPAddress : IP address of the Cirrus3DSensor
 * \param[in] ConfigId  : ID of the config to load
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_ExecuteCommandRECI(const char *pIpAddress,
                                   int configId)
{
    VN_SOCKET sock;
    VN_tERR_Code err=VN_eERR_NoError;
    char reciCommand[15];

    printf("Starting RECI command with config %d at address %s\n",configId,pIpAddress);

    //Open a VN_SOCKET
    err=CirrusCom_connect(pIpAddress,&sock);
    if (err!=VN_eERR_NoError)
        return err;

    err=CirrusCom_setTimeout(sock,
                             VN_cSocketTimeout);
    if (err!=VN_eERR_NoError)
        return err;

    //send the RECI command to the Cirrus
    sprintf(reciCommand, "RECI %d%c%c", configId, 13, 10);
    err=CirrusCom_SendCommand(sock, reciCommand);
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

    //get and interpret the response from the Cirrus
    {
        int nbBytesRead;
        char  command[5];

        //The expected response from the Cirrus is the command code followed by the status
        //Note : there can be some loading time depending on the selected configuration, max 500ms...
        nbBytesRead=CirrusCom_Receive(sock, command, 4, 0);
        if(nbBytesRead == -1)
        {//a return value of -1 indicate a VN_SOCKET error
            printf("VN_SOCKET error during RECI command recv()\n");
            //end the communication properly even if the commands fails
            CirrusCom_End_connection(sock);
            return VN_eERR_SocketRecvError;
        }
        else if (nbBytesRead != 4)
        {//we are supposed to read exactly 4 bytes here
            printf("Error during RECI command recv : insufficient number of bytes (wanted %d, received %d)\n",4,nbBytesRead);
            CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
            return VN_eERR_SocketIncompleteCom;
        }

        if(command[0]!='R' || command[1]!='E' || command[2]!='C' || command[3]!='I')
        {
            if(command[0]=='E' && command[1]=='R' && command[2]=='R')
            {
                printf("Recv: ERR -> Unknown command");
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_UnknownCommand;
            }
            else
            {
                command[4]='\0';
                printf("Recv: %s -> communication error", command);
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_Failure;
            }
        }
        else
        {
            const int nbBytesToRead=1024;
            char buffer[1024];

            //read the rest of the response : all the data sent by the Cirrus until it closes the VN_SOCKET
            nbBytesRead=CirrusCom_Receive(sock, (char*)buffer, sizeof(buffer), 0);
            if(nbBytesRead == -1)
            {//a return value of -1 indicate a VN_SOCKET error
                printf("Error INFO STS command : VN_SOCKET error when receiving command result\n");
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_SocketRecvError;
            }

            if (nbBytesRead<nbBytesToRead)
                buffer[nbBytesRead]='\0';
            else
                buffer[nbBytesToRead-1]='\0';

            int status=atoi(strtok(buffer, ","));
            if (status!=0)
            {//expected status is 0 if the command was executed properly, error code otherwise
                printf("Failed RECI command with status %d",status);
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_Failure;
            }
        }
    }

    //if reached here : command successful, close VN_SOCKET properly
    err=CirrusCom_End_connection(sock);
    if (err!=VN_eERR_NoError)
        return err;
    printf("RECI command with config %d at address %s executed successfully\n",configId,pIpAddress);

    return VN_eERR_NoError;
}

/** @ingroup CloudOfPoint
 * \brief Request the Cirrus3DSensor to scan and get the scan result as a XYZ cloud
 * \param[in] IPAddress     : IP address of the Cirrus3DSensor
 * \param[in] MaxCloudSize  : Max number of points that can be stored in the cloud of points buffer
 * \param[out] pCloud_XYZ   : Pointer to a buffer able to store the resulting cloud of points
 * \param[out] pCloudSize   : Will contain the number of points in the cloud
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise

 */
VN_tERR_Code VN_ExecuteSCAN_XYZ(const char *pIpAddress,
                                const int MaxCloudSize,
                                char *pCloud_XYZ,
                                int *pCloudSize)
{
    VN_SOCKET sock;
    VN_tERR_Code err=VN_eERR_NoError;
    char ScanCommand[15];
    int nbBytesRead;
    int header[3];
    int status, cloudBytesSize, cloudPoints;

    printf("Starting SCAN command with XYZ format at address %s\n",pIpAddress);

    //Open a VN_SOCKET
    err=CirrusCom_connect(pIpAddress,&sock);
    if (err!=VN_eERR_NoError)
        return err;

    err=CirrusCom_setTimeout(sock,
                             VN_cSocketTimeout);
    if (err!=VN_eERR_NoError)
        return err;

    //send the SCAN command to the Cirrus
    sprintf(ScanCommand, "SCAN %d,%d%c%c",1,0,13,10);//second parameter 0 to request XYZ cloud format
    err=CirrusCom_SendCommand(sock, ScanCommand);
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

   //get and interpret the response from the Cirrus
   err=VN_ExecuteSCAN_readHeader(sock);
   if (err!=VN_eERR_NoError)
   {
       printf("Scan failed with error %d\n",err);
       return err;
   }

    //read the header of the response
    nbBytesRead=CirrusCom_Receive(sock, (char*)header, sizeof(header), 0);
    if(nbBytesRead == -1)
    {//a return value of -1 indicate a VN_SOCKET error
        printf("VN_SOCKET error when trying to read scan header\n");
        CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
        return VN_eERR_SocketRecvError;
    }

    //in case of incomplete read of the header (possible since status is sent separately from the rest) try again
    if (nbBytesRead < (int)sizeof(header))
        nbBytesRead += CirrusCom_Receive(sock, ((char*)header) + nbBytesRead , sizeof(header)-nbBytesRead, 0);

    if (nbBytesRead!=sizeof(header))
    {//incomplete com
        printf("Incomplete scan header\n");
        printf("Header=%d; %d; %d\n", header[0],header[1],header[2]);
        fflush(stdout);
        CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
        return VN_eERR_SocketIncompleteCom;
    }
    status=header[0];
    cloudBytesSize=header[1];
    cloudPoints=header[2];

    //check that the status indicates scan sucess
    if (status!=0)
    {
        printf("Scan failed with error status %d\n",header[0]);
        CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
        return VN_eERR_SocketRecvError;
    }

    //check that there is no incoherency in the cloud of points size
    if (cloudBytesSize != (cloudPoints*3*4))
    {
        printf("ERROR : Scan result header is incoherent : for %d points, we expected a buffer size of %d bytes, not %d\n",
               cloudPoints,cloudPoints*3*4,cloudBytesSize);
        CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
        return VN_eERR_BadHeader;
    }

    //check that the buffer passed as a parameter is big enough for this cloud of points
    if (cloudPoints > MaxCloudSize)
    {
        printf("ERROR : cloud of points buffer is insufficient to accept the incoming points (buffer size %d points, cloud size %d points)\n",
               MaxCloudSize,cloudPoints);
        CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
        return VN_eERR_InsufficientMemory;
    }

    //read the cloud of points
    //Note : because of the size of the data to read, we might need several calls to recv to properly receive all the data
    int bytesRead=0;
    while (bytesRead < cloudBytesSize)
    {
        int rVal=CirrusCom_Receive(sock, pCloud_XYZ, cloudBytesSize-bytesRead, 0);
        if (rVal==-1)
        {//a return value of -1 indicate a VN_SOCKET error
            printf("Error in SCAN_XYZ command : VN_SOCKET error when receiving command result\n");
            CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
            return VN_eERR_SocketRecvError;
        }

        if (rVal != 0)
        {
            bytesRead += rVal;
            pCloud_XYZ += rVal;
        }
        else
        {
            //with the large timeout defined, not receiving any bytes here means
            // a com failure...
            printf("Error in SCA_XYZ command : incomplete com\n");
            CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
            return VN_eERR_SocketIncompleteCom;
        }
    }
    //if reached here : cloud of points received properly ; update cloud size
    *pCloudSize=cloudPoints;


    //if reached here : command successful, close VN_SOCKET properly
    err=CirrusCom_End_connection(sock);
    if (err!=VN_eERR_NoError)
        return err;

    printf("SCAN command successfull ; an XYZ cloud of %d points was received.\n",*pCloudSize);

    return VN_eERR_NoError;
}

/** @ingroup CloudOfPoint
 * \brief Request the Cirrus3DSensor to scan and get the scan result as a XYZI cloud
 * \param[in] IPAddress     : IP address of the Cirrus3DSensor
 * \param[in] MaxCloudSize  : Max number of points that can be stored in the cloud of points buffer
 * \param[out] pCloud_XYZI  : Pointer to a buffer able to store the resulting cloud of points
 * \param[out] pCloudSize   : Will contain the number of points in the cloud
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_ExecuteSCAN_XYZI16(const char *pIpAddress, const int MaxCloudSize, VN_Point_XYZI16 *pCloud_XYZI16, int *pCloudSize)
{
    VN_SOCKET sock;
    VN_tERR_Code err=VN_eERR_NoError;
    char ScanCommand[15];
    int nbBytesRead;
    int header[3];
    int status, cloudBytesSize, cloudPoints;

    printf("Starting SCAN command with XYZi format at address %s\n",pIpAddress);

    //Open a VN_SOCKET
    err=CirrusCom_connect(pIpAddress,&sock);
    if (err!=VN_eERR_NoError)
        return err;

    err=CirrusCom_setTimeout(sock,
                             VN_cSocketTimeout);
    if (err!=VN_eERR_NoError)
        return err;

    //send the SCAN command to the Cirrus
    sprintf(ScanCommand, "SCAN %d,%d%c%c",1,1,13,10);//second parameter 1 to request XYZi cloud format
    err=CirrusCom_SendCommand(sock, ScanCommand);
    if(err!=VN_eERR_NoError)
    {
        printf("Failed to send scan command\n");
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

    //get and interpret the response from the Cirrus
    err=VN_ExecuteSCAN_readHeader(sock);
    if (err!=VN_eERR_NoError)
    {
        printf("Scan failed with error %d\n",err);
        return err;
    }

    {

        //read the header of the response
        nbBytesRead=CirrusCom_Receive(sock, (char*)header, sizeof(header), 0);
        if(nbBytesRead == -1)
        {//a return value of -1 indicate a VN_SOCKET error
            printf("VN_SOCKET error when trying to read header\n");
            CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
            return VN_eERR_SocketRecvError;
        }
        //in case of incomplete read of the header (possible since status is sent separately from the rest) try again
        if (nbBytesRead < (int)sizeof(header))
            nbBytesRead += CirrusCom_Receive(sock, ((char*)header) + nbBytesRead , sizeof(header)-nbBytesRead, 0);

        if (nbBytesRead!=sizeof(header))
        {//incomplete com
            printf("Incomplete header error\n");
            CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
            return VN_eERR_SocketIncompleteCom;
        }
        status=header[0];
        cloudBytesSize=header[1];
        cloudPoints=header[2];

        //check that the status indicates scan sucess
        if (status!=0)
        {
            printf("Scan failed with error status %d\n",header[0]);
            CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
            return VN_eERR_SocketRecvError;
        }

        //check that there is no incoherency in the cloud of points size
        const int cPointSize=3*4+2;//each point is 3 4-bytes VN_REAL32 and one 2-byte uint16
        if (cloudBytesSize != (cloudPoints*cPointSize))
        {
            printf("ERROR : Scan result header is incoherent : for %d points, we expected a buffer size of %d bytes, not %d\n",
                   cloudPoints,cloudPoints*cPointSize,cloudBytesSize);
            CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
            return VN_eERR_BadHeader;
        }

        //check that the buffer passed as a parameter is big enough for this cloud of points
        if (cloudPoints > MaxCloudSize)
        {
            printf("ERROR : cloud of points buffer is insufficient to accept the incoming points (buffer size %d points, cloud size %d points)\n",
                   MaxCloudSize,cloudPoints);
            CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
            return VN_eERR_InsufficientMemory;
        }

#ifdef WITH_PADDING
        //read the cloud of points
        //Note : because of the size of the data to read, we might need several calls to recv to properly receive all the data
        /*Warning : the Point_XYZI struct used here tends to be automatically padded by the compiler when used in arrays
         * because unaligned memory access tends to be slower.
         *However, that means that we can't just transfer directly the binary stream from the Cirrus to the buffer, we need
         * to go through an intermediary buffer then store its contents properly into the Point_XYZI array.
         */
        char *pBuffer=(char*)malloc(cloudBytesSize);
        if (pBuffer == NULL)
        {
            printf("Error : failed to allocate enough memory for the image intermediary buffer\n");
            CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
            return VN_eERR_InsufficientMemory;
        }
#else
        char *pBuffer=(char*)pCloud_XYZI16;
#endif

        int bytesRead=0;
        char *pBufferPtr=pBuffer;
        err=CirrusCom_ReceiveBufferofSizeN(sock, pBufferPtr, cloudBytesSize-bytesRead);
        if(err!=VN_eERR_NoError)
        {
            //end the communication properly even if the commands fails
            CirrusCom_End_connection(sock);
            return err;
        }

#ifdef WITH_PADDING
        //extracts points from the intermediary buffer...
        pBufferPtr=pBuffer;
        for(int pt=0 ; pt<cloudPoints ; pt++)
        {
            VN_REAL32 x=*((VN_REAL32*)pBufferPtr);
            pBufferPtr += sizeof(VN_REAL32);
            VN_REAL32 y=*((VN_REAL32*)pBufferPtr);
            pBufferPtr += sizeof(VN_REAL32);
            VN_REAL32 z=*((VN_REAL32*)pBufferPtr);
            pBufferPtr += sizeof(VN_REAL32);
            VN_UINT16 I=*((VN_UINT16*)pBufferPtr);
            pBufferPtr += sizeof(VN_UINT16);
            pCloud_XYZI16[pt].x=x;
            pCloud_XYZI16[pt].y=y;
            pCloud_XYZI16[pt].z=z;
            pCloud_XYZI16[pt].i=I;
        }

        free(pBuffer);
#endif

        //if reached here : cloud of points received properly ; update cloud size
        *pCloudSize=cloudPoints;
    }

    //if reached here : command successful, close VN_SOCKET properly
    err=CirrusCom_End_connection(sock);
    if (err!=VN_eERR_NoError)
        return err;

    printf("SCAN command successfull ; an XYZI cloud of %d points was received.\n",*pCloudSize);

    return VN_eERR_NoError;
}


/** @ingroup CloudOfPoint
 * \brief Request the Cirrus3DSensor to scan and get the scan result as matrix of XYZi points
 * \param[in] IPAddress         : IP address of the Cirrus3DSensor
 * \param[in] MaxCloudSize      : Max number of points that can be stored in the cloud of points buffer
 * \param[out] pCloud_XYZi      : Pointer to a buffer able to store the resulting cloud of points
 * \param[out] pCloudSize       : Will contain the number of non-zero points in the matrix
 * \param[out] pMatrixColumns   : Will contain the number of columns in the matrix
 * \param[out] pMatrixRows      : Will contain the number of rows in the matrix
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise

 */
VN_tERR_Code VN_ExecuteSCAN_Matrix_XYZI8(const char *pIpAddress, const int MaxCloudSize, VN_Point_XYZI8 *pCloud_XYZI8, int *pCloudSize, int *pMatrixColumns, int *pMatrixRows)
{
    VN_SOCKET sock;
    VN_tERR_Code err=VN_eERR_NoError;
    char ScanCommand[15];

    printf("Starting SCAN command with XYZi format at address %s\n",pIpAddress);

    //Open a VN_SOCKET
    err=CirrusCom_connect(pIpAddress,&sock);
    if (err!=VN_eERR_NoError)
        return err;

    err=CirrusCom_setTimeout(sock,
                             VN_cSocketTimeout);
    if (err!=VN_eERR_NoError)
        return err;

    //send the SCAN command to the Cirrus
    sprintf(ScanCommand, "SCAN %d,%d%c%c",1,3,13,10);//second parameter 3 to request matrix_XYZi cloud format
    err=CirrusCom_SendCommand(sock, ScanCommand);
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

    //get and interpret the response from the Cirrus
    {
        int nbBytesRead;
        int header[5];

        err=VN_ExecuteSCAN_readHeader(sock);
        if (err!=VN_eERR_NoError)
        {
            printf("Scan failed with error %d\n",err);
            return err;
        }

        {
            int status, cloudBytesSize, cloudPoints, rowCount, columnCount;

            //read the header of the response
            nbBytesRead=CirrusCom_Receive(sock, (char*)header, sizeof(header), 0);
            if(nbBytesRead == -1)
            {//a return value of -1 indicate a VN_SOCKET error
                printf("VN_SOCKET error when trying to get scan_MATRIX_XYZi header\n");
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_SocketRecvError;
            }
            //in case of incomplete read of the header (possible here twice because rowcount & columnCount are sent in a separate packet from the rest of the header...)
            if (nbBytesRead < (int)sizeof(header))
                nbBytesRead += CirrusCom_Receive(sock, ((char*)header) + nbBytesRead , sizeof(header)-nbBytesRead, 0);
            if (nbBytesRead < (int)sizeof(header))
                nbBytesRead += CirrusCom_Receive(sock, ((char*)header) + nbBytesRead , sizeof(header)-nbBytesRead, 0);

            if (nbBytesRead!=sizeof(header))
            {//incomplete com
                printf("Recv error when trying to get scan_MATRIX_XYZi header\n");
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_SocketIncompleteCom;
            }
            status=header[0];
            cloudBytesSize=header[1];
            cloudPoints=header[2];
            rowCount=header[3];
            columnCount=header[4];

            //check that the status indicates scan sucess
            if (status!=0)
            {
                printf("Scan failed with error status %d\n",header[0]);
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_SocketRecvError;
            }

            //check that there is no incoherency in the cloud of points size
            const int cPointSize=3*4+1;//each point is 3 4-bytes VN_REAL32 and one 1-byte uint8
            int expectedCloudSize=2*4 + rowCount*columnCount*cPointSize;//in this format the cloud is transfered as a matrix,
            // and for compatibility with the other scan formats the rowCount & columnCount are considered to be part of
            // the cloud of points data when computing the buffer size

            //with each case of the matrix containing either a point or 0
            if (cloudBytesSize != expectedCloudSize)
            {
                printf("ERROR : Scan result header is incoherent : for %d points, we expected a buffer size of %d bytes, not %d\n",
                       cloudPoints,expectedCloudSize,cloudBytesSize);
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_BadHeader;
            }

            //check that the buffer passed as a parameter is big enough for this cloud of points
            if (rowCount*columnCount > MaxCloudSize)
            {
                printf("ERROR : cloud of points buffer is insufficient to accept the incoming points (buffer size %d points, cloud matrix size %d points)\n",
                       MaxCloudSize,rowCount*columnCount);
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_InsufficientMemory;
            }

#ifdef WITH_PADDING
            //read the cloud of points
            //Note : because of the size of the data to read, we might need several calls to recv to properly receive all the data
            /*Warning : the Point_XYZI struct used here tends to be automatically padded by the compiler when used in arrays
             * because unaligned memory access tends to be slower.
             *However, that means that we can't just transfer directly the binary stream from the Cirrus to the buffer, we need
             * to go through an intermediary buffer then store its contents properly into the Point_XYZI array.
             */
            char *pBuffer=(char*)malloc(cloudBytesSize);
            if (pBuffer == NULL)
            {
                printf("Error : failed to allocate enough memory for the image intermediary buffer\n");
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_InsufficientMemory;
            }
#else
            char *pBuffer=(char*)pCloud_XYZI8;
#endif
            int bytesRead=2*4;//doesn't start at 0 here, because we already read rowCount & columnCount...
            char *pBufferPtr=pBuffer;
            err=CirrusCom_ReceiveBufferofSizeN(sock, pBufferPtr, cloudBytesSize-bytesRead);
            if(err!=VN_eERR_NoError)
            {
                //end the communication properly even if the commands fails
                CirrusCom_End_connection(sock);
                return err;
            }

#ifdef WITH_PADDING
            //extracts points from the intermediary buffer...
            pBufferPtr=pBuffer;
            for(int pt=0 ; pt<rowCount*columnCount ; pt++)
            {
                VN_REAL32 x=*((VN_REAL32*)pBufferPtr);
                pBufferPtr += sizeof(VN_REAL32);
                VN_REAL32 y=*((VN_REAL32*)pBufferPtr);
                pBufferPtr += sizeof(VN_REAL32);
                VN_REAL32 z=*((VN_REAL32*)pBufferPtr);
                pBufferPtr += sizeof(VN_REAL32);
                VN_UINT8 I=*((VN_UINT8*)pBufferPtr);
                pBufferPtr += sizeof(VN_UINT8);
                pCloud_XYZI8[pt].x=x;
                pCloud_XYZI8[pt].y=y;
                pCloud_XYZI8[pt].z=z;
                pCloud_XYZI8[pt].i=I;
            }

            free(pBuffer);
#endif

            //if reached here : cloud of points received properly ; update cloud size
            *pCloudSize=cloudPoints;
            //also update matrix size
            *pMatrixColumns=columnCount;
            *pMatrixRows=rowCount;
        }
    }

    //if reached here : command successful, close VN_SOCKET properly
    err=CirrusCom_End_connection(sock);
    if (err!=VN_eERR_NoError)
        return err;

    printf("SCAN command successfull ; a matrix cloud of %d points was received.\n",*pCloudSize);

    return VN_eERR_NoError;
}

/** @ingroup CloudOfPoint
 * \brief request the Cirrus3DSensor to scan and get the scan result as matrix of XYZRGB points
 *  (more accurately pcl::PointXYZRGB from the PCL library) ;
 *  in this format the empty elements of the matrix have NaN coordinates, and the
 *  rgb float is actually a rgb int casted as float (where all r,g,b fields are
 *  identical since the Cirrus only generate black-and-white pixel data)
 * \param[in] IPAddress         : IP address of the Cirrus3DSensor
 * \param[in] MaxCloudSize      : Max number of points that can be stored in the cloud of points buffer
 * \param[out] pCloud_XYZRGB    : pointer to a buffer able to store the resulting cloud of points
 * \param[out] pCloudSize       : Will contain the number of non-zero points in the matrix
 * \param[out] pMatrixColumns   : Will contain the number of columns in the matrix
 * \param[out] pMatrixRows      : Will contain the number of rows in the matrix
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_ExecuteSCAN_Matrix_XYZRGB(const char *pIpAddress,
                                          const int MaxCloudSize,
                                          VN_Point_XYZRGB *pCloud_XYZRGB,
                                          int *pCloudSize,
                                          int *pMatrixColumns,
                                          int *pMatrixRows)
{
    VN_SOCKET sock;
    VN_tERR_Code err=VN_eERR_NoError;
    char ScanCommand[15];
    int status, cloudBytesSize, cloudPoints, rowCount, columnCount;

    printf("Starting SCAN command with matrix XYZRGB format at address %s\n",pIpAddress);

    //Open a VN_SOCKET
    err=CirrusCom_connect(pIpAddress,&sock);
    if (err!=VN_eERR_NoError)
        return err;

    err=CirrusCom_setTimeout(sock,
                             VN_cSocketTimeout);
    if (err!=VN_eERR_NoError)
        return err;

    //send the SCAN command to the Cirrus
    sprintf(ScanCommand, "SCAN %d,%d%c%c",1,4,13,10);//second parameter 4 to request matrix_XYZRGB cloud format
    err=CirrusCom_SendCommand(sock, ScanCommand);
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }


    err=VN_ExecuteSCAN_Matrix_XYZRGB_readHeader(sock,
                                                &status,
                                                &cloudBytesSize,
                                                &cloudPoints,
                                                &rowCount,
                                                &columnCount);
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }


    //check that the buffer passed as a parameter is big enough for this cloud of points
    if (rowCount*columnCount > MaxCloudSize)
    {
        printf("ERROR : cloud of points buffer is insufficient to accept the incoming points (buffer size %d points, cloud matrix size %d points)\n",
               MaxCloudSize,rowCount*columnCount);
        CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
        return VN_eERR_InsufficientMemory;
    }

    int bytesRead=2*4;//doesn't start at 0 here, because we already read rowCount & columnCount...
    char *pCloud=(char*)pCloud_XYZRGB;//need the cast because no guarantee that each call to recv will result in an integer number of Point_XYZRGB...

    err=CirrusCom_ReceiveBufferofSizeN(sock, pCloud, cloudBytesSize-bytesRead);
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

    //if reached here : cloud of points received properly ; update cloud size
    *pCloudSize=cloudPoints;
    //also update matrix size
    *pMatrixColumns=columnCount;
    *pMatrixRows=rowCount;

    //if reached here : command successful, close VN_SOCKET properly
    err=CirrusCom_End_connection(sock);
    if (err!=VN_eERR_NoError)
        return err;

    printf("SCAN command successfull ; a matrix cloud of %d XYZRGB points was received.\n",*pCloudSize);

    return VN_eERR_NoError;
}



VN_tERR_Code VN_ExecuteSCAN_readHeader(VN_SOCKET sock)
{

    //get and interpret the response from the Cirrus
    int nbBytesRead;
    char  command[5];

    //The expected response from the Cirrus is the command code followed by a binary status then the cloud of points
    nbBytesRead=CirrusCom_Receive(sock, command, 4, 0);
    if(nbBytesRead == -1)
    {//a return value of -1 indicate a VN_SOCKET error
        printf("VN_SOCKET error when trying to get scan result fom SCAN_Matrix_XYZRGB command\n");
        //end the communication properly even if the commands fails
        return VN_eERR_SocketRecvError;
    }
    else if (nbBytesRead != 4)
    {//we are supposed to read exactly 4 bytes here
        printf("Recv error when trying to get scan result\n");
        return VN_eERR_SocketIncompleteCom;
    }

    if(command[1]!='C' || command[2]!='A' || command[3]!='N')
    {
        if(command[0]=='E' && command[1]=='R' && command[2]=='R')
        {
            printf("Recv: ERR -> Unknown command");
            return VN_eERR_UnknownCommand;
        }
        else
        {
            command[4]='\0';
            printf("Recv: %s -> communication error", command);
            return VN_eERR_Failure;
        }
    }

    return VN_eERR_NoError;

}


VN_tERR_Code VN_ExecuteSCAN_Matrix_XYZRGB_readHeader(VN_SOCKET sock,
                                                     VN_INT32 *pStatus,
                                                     VN_INT32 *pCloudBytesSize,
                                                     VN_INT32 *pCloudPoints,
                                                     VN_INT32 *pRowCount,
                                                     VN_INT32 *pColumnCount)
{
    //get and interpret the response from the Cirrus
    int nbBytesRead;
    VN_tERR_Code err=VN_eERR_NoError;
    int header[5];

    err=VN_ExecuteSCAN_readHeader(sock);
    if (err!=VN_eERR_NoError)
    {
        printf("Scan failed with error %d\n",err);
        return err;
    }

    //read the header of the response
    nbBytesRead=CirrusCom_Receive(sock, (char*)header, sizeof(header), 0);
    if(nbBytesRead == -1)
    {//a return value of -1 indicate a VN_SOCKET error
        printf("VN_SOCKET error when trying to get scan_MATRIX_XYZRGB header\n");
        return VN_eERR_SocketRecvError;
    }
    //in case of incomplete read of the header (possible here twice because rowcount & columnCount are sent in a separate packet from the rest of the header...)
    if (nbBytesRead < (int)sizeof(header))
        nbBytesRead += CirrusCom_Receive(sock, ((char*)header) + nbBytesRead , sizeof(header)-nbBytesRead, 0);
    if (nbBytesRead < (int)sizeof(header))
        nbBytesRead += CirrusCom_Receive(sock, ((char*)header) + nbBytesRead , sizeof(header)-nbBytesRead, 0);

    if (nbBytesRead!=sizeof(header))
    {//incomplete com
        printf("Recv error when trying to get scan_MATRIX_XYZRGB header\n");
        return VN_eERR_SocketIncompleteCom;
    }
    *pStatus=header[0];
    *pCloudBytesSize=header[1];
    *pCloudPoints=header[2];
    *pRowCount=header[3];
    *pColumnCount=header[4];

    //check that the status indicates scan sucess
    if (*pStatus!=0)
    {
        printf("Scan failed with error status %d\n",*pStatus);
        return *pStatus;
    }

    //check that there is no incoherency in the cloud of points size
    const int cPointSize=3*4+4;//each point is 3 4-bytes VN_REAL32 and one 4-byte VN_REAL32
    int expectedCloudSize=2*4 + (*pRowCount)*(*pColumnCount)*cPointSize;//in this format the cloud is transfered as a matrix,
    // and for compatibility with the other scan formats the rowCount & columnCount are considered to be part of
    // the cloud of points data when computing the buffer size

    //with each case of the matrix containing either a point or 0
    if (*pCloudBytesSize != expectedCloudSize)
    {
        printf("ERROR : Scan result header is incoherent : for %d points, we expected a buffer size of %d bytes, not %d\n",
               *pCloudPoints,expectedCloudSize,*pCloudBytesSize);
        return VN_eERR_BadHeader;
    }

    printf("Receiving a matrix cloud of size %d rows & %d columns\n",*pRowCount,*pColumnCount);

    return VN_eERR_NoError;
}

VN_tERR_Code VN_ExecuteSCAN_CameraCOP_XYZI8_readHeader(VN_SOCKET sock,
                                                       VN_INT32 *pStatus,
                                                       VN_INT32 *pCloudBytesSize,
                                                       VN_INT32 *pCloudPoints,
                                                       VN_INT32 *pRowCount,
                                                       VN_INT32 *pColumnCount)
{
    VN_tERR_Code err;
    int header[5];
    int nbBytesRead;

    //get and interpret the response from the Cirrus
    err=VN_ExecuteSCAN_readHeader(sock);
    if (err!=VN_eERR_NoError)
    {
        printf("Scan failed with error %d\n",err);
        return err;
    }

    //read the header of the response
    nbBytesRead=CirrusCom_Receive(sock, (char*)header, sizeof(header), 0);
    if(nbBytesRead == -1)
    {//a return value of -1 indicate a VN_SOCKET error
        printf("VN_SOCKET error when trying to get scan_MATRIX_XYZI8 header\n");
        CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
        return VN_eERR_SocketRecvError;
    }
    //in case of incomplete read of the header (possible here twice because rowcount & columnCount are sent in a separate packet from the rest of the header...)
    if (nbBytesRead < (int)sizeof(header))
        nbBytesRead += CirrusCom_Receive(sock, ((char*)header) + nbBytesRead , sizeof(header)-nbBytesRead, 0);
    if (nbBytesRead < (int)sizeof(header))
        nbBytesRead += CirrusCom_Receive(sock, ((char*)header) + nbBytesRead , sizeof(header)-nbBytesRead, 0);

    if (nbBytesRead!=sizeof(header))
    {//incomplete com
        printf("Recv error when trying to get scan_MATRIX_XYZI8 header\n");
        CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
        return VN_eERR_SocketIncompleteCom;
    }
    *pStatus=header[0];
    *pCloudBytesSize=header[1];
    *pCloudPoints=header[2];
    *pRowCount=header[3];
    *pColumnCount=header[4];

    //check that the status indicates scan sucess
    if (*pStatus!=0)
    {
        printf("Scan failed with error status %d\n",*pStatus);
        return VN_eERR_SocketRecvError;
    }

    //check that there is no incoherency in the cloud of points size
    const int cPointSize=3*4+1;//each point is 3 4-bytes VN_REAL32 and one 1-byte VN_UCHAR
    int expectedCloudSize=2*4 + (*pRowCount)*(*pColumnCount)*cPointSize;//in this format the cloud is transfered as a matrix,
    // and for compatibility with the other scan formats the rowCount & columnCount are considered to be part of
    // the cloud of points data when computing the buffer size

    //with each case of the matrix containing either a point or 0
    if (*pCloudBytesSize != expectedCloudSize)
    {
        printf("ERROR : Scan result header is incoherent : for %d points, we expected a buffer size of %d bytes, not %d\n",
               *pCloudPoints,expectedCloudSize,*pCloudBytesSize);
        return VN_eERR_BadHeader;
    }

    return VN_eERR_NoError;
}

/** @ingroup CloudOfPoint
 * \brief request the Cirrus3D to scan and get the scan result as matrix of XYZI points
 *  The matrix corresponds to a rectified image from the left camera or a virtual middle camera.
 *  In this format the empty elements of the matrix have NaN coordinates.
 *  Function with version compatibility check before connection to the Cirrus3D.
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
                                              VN_BOOL fromMiddleCamPOV)
{
    if( pCirrus3DHandler->DeviceVersion>=2)
    return VN_ExecuteSCAN_CameraCOP_XYZI8(pCirrus3DHandler->IPAddress,
                                          MaxCloudSize,
                                          pCloud_XYZI8,
                                          pCloudSize,
                                          pMatrixColumns,
                                          pMatrixRows,
                                          fromMiddleCamPOV);
    else
        return VN_eERR_IncompatibleDeviceVersion;
}

/** @ingroup CloudOfPoint
 * \brief request the Cirrus3D to scan and get the scan result as matrix of XYZI points
 *  The matrix corresponds to a rectified image from the left camera or a virtual middle camera.
 *  In this format the empty elements of the matrix have NaN coordinates.
 * \param[in] IPAddress         : IP address of the Cirrus3D
 * \param[in] MaxCloudSize      : Max number of points that can be stored in the cloud of points buffer
 * \param[out] pCloud_XYZI8    : pointer to a buffer able to store the resulting cloud of points
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
                                             VN_BOOL fromMiddleCamPOV)
{
    VN_SOCKET sock;
    VN_tERR_Code err=VN_eERR_NoError;
    char ScanCommand[15];
    int status, cloudBytesSize, cloudPoints, rowCount, columnCount;

    //Open a VN_SOCKET
    err=CirrusCom_connect(pIpAddress,&sock);
    if (err!=VN_eERR_NoError)
        return err;

    err=CirrusCom_setTimeout(sock,
                             VN_cSocketTimeout);
    if (err!=VN_eERR_NoError)
        return err;

    //send the SCAN command to the Cirrus
    if (fromMiddleCamPOV==0)
        sprintf(ScanCommand, "SCAN %d,%d%c%c",1,5,13,10);//second parameter 5 to request Camera_COP cloud format from the perspective of the left camera
    else
        sprintf(ScanCommand, "SCAN %d,%d%c%c",1,6,13,10);//second parameter 6 to request Camera_COP cloud format from the perspective of a virtual middle camera

    err=CirrusCom_SendCommand(sock, ScanCommand);
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }


    err= VN_ExecuteSCAN_CameraCOP_XYZI8_readHeader(sock,
                                                   &status,
                                                   &cloudBytesSize,
                                                   &cloudPoints,
                                                   &rowCount,
                                                   &columnCount);
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

    //check that the buffer passed as a parameter is big enough for this cloud of points
    if (rowCount*columnCount > MaxCloudSize)
    {
        printf("ERROR : cloud of points buffer is insufficient to accept the incoming points (buffer size %d points, cloud matrix size %d points)\n",
               MaxCloudSize,rowCount*columnCount);
        CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
        return VN_eERR_InsufficientMemory;
    }

    printf("Receiving a matrix cloud of size %d rows & %d columns\n",rowCount,columnCount);

#ifdef WITH_PADDING
    //read the cloud of points
    //Note : because of the size of the data to read, we might need several calls to recv to properly receive all the data
    /*Warning : the Point_XYZI struct used here tends to be automatically padded by the compiler when used in arrays
     * because unaligned memory access tends to be slower.
     *However, that means that we can't just transfer directly the binary stream from the Cirrus to the buffer, we need
     * to go through an intermediary buffer then store its contents properly into the Point_XYZI array.
     */
    char *pBuffer=(char*)malloc(cloudBytesSize);
    if (pBuffer == NULL)
    {
        printf("Error : failed to allocate enough memory for the image intermediary buffer\n");
        CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
        return VN_eERR_InsufficientMemory;
    }
#else
    char *pBuffer=(char*)pCloud_XYZI8;
#endif

    int bytesRead=2*4;
    err=CirrusCom_ReceiveBufferofSizeN(sock, pBuffer, cloudBytesSize-bytesRead);
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

#ifdef WITH_PADDING
    //extracts points from the intermediary buffer...
    pBufferPtr=pBuffer;
    for(int pt=0 ; pt<rowCount*columnCount ; pt++)
    {
        VN_REAL32 x=*((VN_REAL32*)pBufferPtr);
        pBufferPtr += sizeof(VN_REAL32);
        VN_REAL32 y=*((VN_REAL32*)pBufferPtr);
        pBufferPtr += sizeof(VN_REAL32);
        VN_REAL32 z=*((VN_REAL32*)pBufferPtr);
        pBufferPtr += sizeof(VN_REAL32);
        VN_UINT8 I=*((VN_UINT8*)pBufferPtr);
        pBufferPtr += sizeof(VN_UINT8);
        pCloud_XYZI8[pt].x=x;
        pCloud_XYZI8[pt].y=y;
        pCloud_XYZI8[pt].z=z;
        pCloud_XYZI8[pt].i=I;
    }

    free(pBuffer);
#endif

    //if reached here : cloud of points received properly ; update cloud size
    *pCloudSize=cloudPoints;
    //also update matrix size
    *pMatrixColumns=columnCount;
    *pMatrixRows=rowCount;

    //if reached here : command successful, close VN_SOCKET properly
    err=CirrusCom_End_connection(sock);
    if (err!=VN_eERR_NoError)
        return err;

    printf("SCAN command successfull ; a matrix cloud of %d points was received.\n",*pCloudSize);

    return VN_eERR_NoError;
}

/** @ingroup CloudOfPoint
 * \brief request the Cirrus3D to scan and get the scan result as matrix of XYZI points
 *  The matrix corresponds to a rectified image from the left camera or a virtual middle camera.
 *  In this format the empty elements of the matrix have NaN coordinates.
 *  Available only for CirrusSensor version 3.0.0 and higher.
 *  Function with version compatibility check before connection to the Cirrus3D.
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
                                                       VN_REAL32 samplingFactor)
{

    if( pCirrus3DHandler->DeviceVersion>=2)
        return VN_ExecuteSCAN_CameraCOP_XYZI8_sampling(pCirrus3DHandler->IPAddress,
                                                       MaxCloudSize,
                                                       pCloud_XYZI8,
                                                       pCloudSize,
                                                       pMatrixColumns,
                                                       pMatrixRows,
                                                       fromMiddleCamPOV,
                                                       samplingFactor);
    else
        return VN_eERR_IncompatibleDeviceVersion;

}

/** @ingroup CloudOfPoint
 * \brief request the Cirrus3D to scan and get the scan result as matrix of XYZI points
 *  The matrix corresponds to a rectified image from the left camera or a virtual middle camera.
 *  In this format the empty elements of the matrix have NaN coordinates.
 * \param[in] IPAddress         : IP address of the Cirrus3D
 * \param[in] MaxCloudSize      : Max number of points that can be stored in the cloud of points buffer
 * \param[out] pCloud_XYZI8    : pointer to a buffer able to store the resulting cloud of points
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
                                                     int *pCloudSize, int *pMatrixColumns, int *pMatrixRows,
                                                     VN_BOOL fromMiddleCamPOV,
                                                     VN_REAL32 samplingFactor)
{
    VN_SOCKET sock;
    VN_tERR_Code err=VN_eERR_NoError;
    char ScanCommand[16];
    int status, cloudBytesSize, cloudPoints, rowCount, columnCount;

    //Open a VN_SOCKET
    err=CirrusCom_connect(pIpAddress,&sock);
    if (err!=VN_eERR_NoError)
        return err;

    err=CirrusCom_setTimeout(sock,
                             VN_cSocketTimeout);
    if (err!=VN_eERR_NoError)
        return err;

    //send the SCAN command to the Cirrus
    if (fromMiddleCamPOV==0)
        sprintf(ScanCommand, "SCAN %d,%d,%d.%d%c%c",1,5,(int)round(samplingFactor),(int)round((samplingFactor-1.0)*100),//note : using round to make sure that sampling factor will be expressed with a '.' instead of a ',' separator
                13,10);//second parameter 5 to request Camera_COP cloud format from the perspective of the left camera
    else
        sprintf(ScanCommand, "SCAN %d,%d,%d.%d%c%c",1,6,(int)round(samplingFactor),(int)round((samplingFactor-1.0)*100),//note : using round to make sure that sampling factor will be expressed with a '.' instead of a ',' separator
                13,10);//second parameter 6 to request Camera_COP cloud format from the perspective of a virtual middle camera

    err=CirrusCom_SendCommand(sock, ScanCommand);
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

    err= VN_ExecuteSCAN_CameraCOP_XYZI8_readHeader(sock,
                                                   &status,
                                                   &cloudBytesSize,
                                                   &cloudPoints,
                                                   &rowCount,
                                                   &columnCount);
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

    //check that the buffer passed as a parameter is big enough for this cloud of points
    if (rowCount*columnCount > MaxCloudSize)
    {
        printf("ERROR : cloud of points buffer is insufficient to accept the incoming points (buffer size %d points, cloud matrix size %d points)\n",
               MaxCloudSize,rowCount*columnCount);
        CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
        return VN_eERR_InsufficientMemory;
    }

    printf("Receiving a matrix cloud of size %d rows & %d columns\n",rowCount,columnCount);

#ifdef WITH_PADDING
    //read the cloud of points
    //Note : because of the size of the data to read, we might need several calls to recv to properly receive all the data
    /*Warning : the Point_XYZI struct used here tends to be automatically padded by the compiler when used in arrays
     * because unaligned memory access tends to be slower.
     *However, that means that we can't just transfer directly the binary stream from the Cirrus to the buffer, we need
     * to go through an intermediary buffer then store its contents properly into the Point_XYZI array.
     */
    char *pBuffer=(char*)malloc(cloudBytesSize);
    if (pBuffer == NULL)
    {
        printf("Error : failed to allocate enough memory for the image intermediary buffer\n");
        CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
        return VN_eERR_InsufficientMemory;
    }
#else
    char *pBuffer=(char*)pCloud_XYZI8;
#endif

    int bytesRead=2*4;
    err=CirrusCom_ReceiveBufferofSizeN(sock, pBuffer, cloudBytesSize-bytesRead);
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

#ifdef WITH_PADDING
    //extracts points from the intermediary buffer...
    pBufferPtr=pBuffer;
    for(int pt=0 ; pt<rowCount*columnCount ; pt++)
    {
        VN_REAL32 x=*((VN_REAL32*)pBufferPtr);
        pBufferPtr += sizeof(VN_REAL32);
        VN_REAL32 y=*((VN_REAL32*)pBufferPtr);
        pBufferPtr += sizeof(VN_REAL32);
        VN_REAL32 z=*((VN_REAL32*)pBufferPtr);
        pBufferPtr += sizeof(VN_REAL32);
        VN_UINT8 I=*((VN_UINT8*)pBufferPtr);
        pBufferPtr += sizeof(VN_UINT8);
        pCloud_XYZI8[pt].x=x;
        pCloud_XYZI8[pt].y=y;
        pCloud_XYZI8[pt].z=z;
        pCloud_XYZI8[pt].i=I;
    }

    free(pBuffer);
#endif

    //if reached here : cloud of points received properly ; update cloud size
    *pCloudSize=cloudPoints;
    //also update matrix size
    *pMatrixColumns=columnCount;
    *pMatrixRows=rowCount;


    //if reached here : command successful, close VN_SOCKET properly
    err=CirrusCom_End_connection(sock);
    if (err!=VN_eERR_NoError)
        return err;

    printf("SCAN command successfull ; a matrix cloud of %d points was received.\n",*pCloudSize);

    return VN_eERR_NoError;
}

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
                                                     int *pMatrixColumns, int *pMatrixRows)
{
    VN_SOCKET sock;
    VN_tERR_Code err=VN_eERR_NoError;
    char ScanCommand[20];

    //Open a VN_SOCKET
    err=CirrusCom_connect(pIpAddress,&sock);
    if (err!=VN_eERR_NoError)
        return err;

    //send the SCAN command to the Cirrus
    sprintf(ScanCommand, "SCAN %d,%d%c%c",1,7,
            13,10);//second parameter 6 to request Camera_COP cloud format from the perspective of a virtual middle camera

    err=CirrusCom_SendCommand(sock, ScanCommand);
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

    //get and interpret the response from the Cirrus
    {
        int nbBytesRead;
        char  command[5];

        //The expected response from the Cirrus is the command code followed by the cloud of points
        nbBytesRead=CirrusCom_Receive(sock, command, 4, 0);
        if(nbBytesRead == -1)
        {//a return value of -1 indicate a VN_SOCKET error
            printf("VN_SOCKET error when trying to get scan result fom SCAN_Matrix_XYZRGB command\n");
            //end the communication properly even if the commands fails
            CirrusCom_End_connection(sock);
            return VN_eERR_SocketRecvError;
        }
        else if (nbBytesRead != 4)
        {//we are supposed to read exactly 4 bytes here
            printf("Recv error when trying to get scan result\n");
            CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
            return VN_eERR_SocketIncompleteCom;
        }

        if(command[0]!='S' || command[1]!='C' || command[2]!='A' || command[3]!='N')
        {
            if(command[0]=='E' && command[1]=='R' && command[2]=='R')
            {
                printf("Recv: ERR -> Unknown command");
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_UnknownCommand;
            }
            else
            {
                command[4]='\0';
                printf("Recv: %s -> communication error", command);
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_Failure;
            }
        }
        else
        {
            int header[3];
            int cloudBytesSize, rowCount, columnCount;

            //read the header of the response
            nbBytesRead=CirrusCom_Receive(sock, (char*)header, sizeof(header), 0);
            if(nbBytesRead == -1)
            {//a return value of -1 indicate a VN_SOCKET error
                printf("VN_SOCKET error when trying to get scan_MATRIX_XYZRGB header\n");
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_SocketRecvError;
            }

            if (nbBytesRead!=sizeof(header))
            {//incomplete com
                printf("Recv error when trying to get CameraCloud header\n");
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_SocketIncompleteCom;
            }
            cloudBytesSize=header[0];
            rowCount=header[1];
            columnCount=header[2];

            //check that there is no incoherency in the cloud of points size
            const int cPointSize=3*4+1;//each point is 3 4-bytes VN_REAL32 + uint8
            int expectedCloudSize=rowCount*columnCount*cPointSize;//in this format the cloud is transfered as a matrix,

            //with each case of the matrix containing either a point or nan
            if (cloudBytesSize != expectedCloudSize)
            {
                printf("ERROR : Scan result header is incoherent : for a %d x %d matrix we expected a buffer size of %d bytes, not %d\n",
                       rowCount,columnCount,expectedCloudSize,cloudBytesSize);
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_BadHeader;
            }

            //check that the buffer passed as a parameter is big enough for this cloud of points
            if (rowCount*columnCount > MaxCloudSize)
            {
                printf("ERROR : cloud of points buffer is insufficient to accept the incoming points (buffer size %d points, cloud matrix size %d points)\n",
                       MaxCloudSize,rowCount*columnCount);
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_InsufficientMemory;
            }

            printf("Receiving a matrix cloud of size %d rows & %d columns\n",rowCount,columnCount);

//read the cloud of points
//Note : because of the size of the data to read, we might need several calls to recv to properly receive all the data
//Note2 : this point data structure is aligned on four bytes, so we can receive it directly

//define a timeout on the VN_SOCKET, to make sure we can't get stuck in an infinite loop
#ifdef WIN32
            int timeout=30000;
            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout,sizeof(timeout));
#else
            struct timeval tv;
            tv.tv_sec=10;  /* 10 Secs Timeout */
            tv.tv_usec=1;  // Not init'ing this can cause strange errors

            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval));
#endif
            int bytesRead=0;
            char *pCloud=(char*)pCloud_XYZI8;
            while (bytesRead < cloudBytesSize)
            {
                int rVal=CirrusCom_Receive((int)sock, (void*)pCloud, (size_t)(cloudBytesSize-bytesRead), (int)0);
                if (rVal==-1)
                {//a return value of -1 indicate a VN_SOCKET error
                    printf("VN_SOCKET error when trying to get scan data (received %d of %d bytes expected)\n",bytesRead,cloudBytesSize);
                    CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                    return VN_eERR_SocketRecvError;
                }

                if (rVal != 0)
                {
                    bytesRead += rVal;
                    pCloud += rVal;
                }
                else
                {
                    //with the large timeout defined, not receiving any bytes here means
                    // a com failure...
                    printf("Timeout error when trying to get scan data (received %d of %d bytes expected)\n",bytesRead,cloudBytesSize);
                    CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                    return VN_eERR_SocketIncompleteCom;
                }
            }            
            //also update matrix size
            *pMatrixColumns=columnCount;
            *pMatrixRows=rowCount;
        }
    }

    //if reached here : command successful, close VN_SOCKET properly
    err=CirrusCom_End_connection(sock);
    if (err!=VN_eERR_NoError)
        return err;

    printf("SCAN command successfull ; a matrix cloud of size %d x %d of XYZi points was received.\n",*pMatrixRows,*pMatrixColumns);

    return VN_eERR_NoError;
}

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
                                         float samplingFactor)
{
    VN_SOCKET sock;
    VN_tERR_Code err=VN_eERR_NoError;
    char ScanCommand[20];

    //Open a VN_SOCKET
    err=CirrusCom_connect(pIpAddress,&sock);
    if (err!=VN_eERR_NoError)
        return err;

    //send the SCAN command to the Cirrus
    sprintf(ScanCommand, "SCAN %d,%d,%d.%d%c%c",1,8,//second parameter 8 to request depthmap
            (int)round(samplingFactor-0.5),(int)(100*samplingFactor - 100*(int)round(samplingFactor-0.5)),
            13,10);

    err=CirrusCom_SendCommand(sock, ScanCommand);
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

    //get and interpret the response from the Cirrus
    {
        int nbBytesRead;
        char  command[5];

        //The expected response from the Cirrus is the command code followed by the deptmap
        nbBytesRead=CirrusCom_Receive(sock, command, 4, 0);
        if(nbBytesRead == -1)
        {//a return value of -1 indicate a VN_SOCKET error
            printf("VN_SOCKET error when trying to get scan result fom SCAN_Depthmap command\n");
            //end the communication properly even if the commands fails
            CirrusCom_End_connection(sock);
            return VN_eERR_SocketRecvError;
        }
        else if (nbBytesRead != 4)
        {//we are supposed to read exactly 4 bytes here
            printf("Recv error when trying to get scan result\n");
            CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
            return VN_eERR_SocketIncompleteCom;
        }

        if(command[0]!='S' || command[1]!='C' || command[2]!='A' || command[3]!='N')
        {
            if(command[0]=='E' && command[1]=='R' && command[2]=='R')
            {
                printf("Recv: ERR -> Unknown command");
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_UnknownCommand;
            }
            else
            {
                command[4]='\0';
                printf("Recv: %s -> communication error", command);
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_Failure;
            }
        }
        else
        {
            int header[7+5];
            int cloudBytesSize, rowCount, columnCount,status/*, nbPoints*/;

            //read the header of the response
            nbBytesRead=CirrusCom_Receive(sock, (char*)header, sizeof(header), 0);
            if(nbBytesRead == -1)
            {//a return value of -1 indicate a VN_SOCKET error
                printf("VN_SOCKET error when trying to get depthmap scan header\n");
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_SocketRecvError;
            }
            //in case of incomplete read of the header (possible here twice because rowcount & columnCount are sent in a separate packet from the rest of the header...)
            for(int k=0 ; k<20 ; k++)
            {
                if (nbBytesRead < (int)sizeof(header))
                    nbBytesRead += CirrusCom_Receive(sock, ((char*)header) + nbBytesRead , sizeof(header)-nbBytesRead, 0);
            }

            if (nbBytesRead!=sizeof(header))
            {//incomplete com
                printf("Recv error when trying to get depthmap header\n");
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_SocketIncompleteCom;
            }
            status=header[0];
            if (status!=VN_eERR_NoError)
            {//incomplete com
                printf("status error returned %d\n", status);
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return status;
            }

            cloudBytesSize=header[1];
//            nbPoints=header[2];

            *pInvalidVal = (VN_UINT16)header[3];
            *pXOffset = *(float*)(&header[4]);
            *pXScale = *(float*)(&header[5]);
            *pYOffset = *(float*)(&header[6]);
            *pYScale = *(float*)(&header[7]);
            *pZOffset = *(float*)(&header[8]);
            *pZScale = *(float*)(&header[9]);

            rowCount=header[10];
            columnCount=header[11];

            //check that there is no incoherency in the cloud of points size
            const int cPointSize=2;//each point is 2-bytes pixel
            int expectedCloudSize=rowCount*columnCount*cPointSize;//in this format the cloud is transfered as a matrix,

            //with each case of the matrix containing either a point or nan
            if (cloudBytesSize != expectedCloudSize)
            {
                printf("ERROR : Scan result header is incoherent : for a %d x %d depthmap image we expected a buffer size of %d bytes, not %d\n",
                       rowCount,columnCount,expectedCloudSize,cloudBytesSize);
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_BadHeader;
            }

            //check that the buffer passed as a parameter is big enough for this cloud of points
            if (rowCount*columnCount > MaxCloudSize)
            {
                printf("ERROR : cloud of points buffer is insufficient to accept the incoming points (buffer size %d points, cloud matrix size %d points)\n",
                       MaxCloudSize,rowCount*columnCount);
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_InsufficientMemory;
            }

            printf("Receiving a depthmap image of size %d rows & %d columns\n",rowCount,columnCount);

//read the cloud of points
//Note : because of the size of the data to read, we might need several calls to recv to properly receive all the data
//Note2 : this point data structure is aligned on four bytes, so we can receive it directly

//define a timeout on the VN_SOCKET, to make sure we can't get stuck in an infinite loop
#ifdef WIN32
            int timeout=30000;
            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout,sizeof(timeout));
#else
            struct timeval tv;
            tv.tv_sec=10;  /* 10 Secs Timeout */
            tv.tv_usec=1;  // Not init'ing this can cause strange errors

            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval));
#endif
            int bytesRead=0;
            char *pCloud=(char*)pDepthMap;
            while (bytesRead < cloudBytesSize)
            {
                int rVal=CirrusCom_Receive(sock, pCloud, cloudBytesSize-bytesRead, 0);
                if (rVal==-1)
                {//a return value of -1 indicate a VN_SOCKET error
                    printf("VN_SOCKET error when trying to get scan data (received %d of %d bytes expected)\n",bytesRead,cloudBytesSize);
                    CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                    return VN_eERR_SocketRecvError;
                }

                if (rVal != 0)
                {
                    bytesRead += rVal;
                    pCloud += rVal;
                }
                else
                {
                    //with the large timeout defined, not receiving any bytes here means
                    // a com failure...
                    printf("Timeout error when trying to get scan data (received %d of %d bytes expected)\n",bytesRead,cloudBytesSize);
                    CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                    return VN_eERR_SocketIncompleteCom;
                }
            }
            //also update matrix size
            *pMatrixColumns=columnCount;
            *pMatrixRows=rowCount;
        }
    }

    //if reached here : command successful, close VN_SOCKET properly
    err=CirrusCom_End_connection(sock);
    if (err!=VN_eERR_NoError)
        return err;

    printf("SCAN command successfull ; a matrix cloud of size %d x %d of XYZi points was received.\n",*pMatrixRows,*pMatrixColumns);

    return VN_eERR_NoError;
}

/**
 * \brief get status / keepAlive response from the Cirrus.
 *  Available only for Cirrus version 2.0.0 and higher.
 *  Can be requested even during a scan (contrary to
 *  the STS command)
 * \param[in] IPaddress : IP address of the Cirrus to command
 * \retval VN_eERR_NoError if success (i.e. Cirrus connected no reported error)
 * \retval error code otherwise
 */
VN_tERR_Code VN_ExecuteCommandKAST(const char *IPaddress)
{
    VN_SOCKET sock;
    VN_tERR_Code err=VN_eERR_NoError;
    char KASTCommand[16];

    printf("Starting KAST command at address %s\n",IPaddress);

    //Open a VN_SOCKET on the dedicated port for this command
    err=CirrusCom_KAST_connect(IPaddress,&sock);
    if (err!=VN_eERR_NoError)
    {
        printf("Failed to open socket for KAST command\n");
        return err;
    }

    //send the KAST command to the Cirrus
    // param : 1 indicating that the latest KAST command version this function
    // supports is 1
    sprintf(KASTCommand, "KAST 1%c%c",13,10);
    err=CirrusCom_SendCommand(sock, KASTCommand);
    if(err!=VN_eERR_NoError)
    {
        printf("Failed to send KAST command\n");
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

    //get and interpret the response from the Cirrus
    //Note : for this command, the Cirrus build and sends the response as a single packet
    {
        int nbBytesRead;
        char  response[4];

        //The expected response is 'KAST' in char format, then a binary uint32 status code (0 for no problem),
        //then the uint32 version of the KAST message returned by the Cirrus (at most the version supported by
        //the command, in this case 1 as specified. There is no version 0. )
        nbBytesRead=CirrusCom_Receive(sock, response, 4, 0);

        if(nbBytesRead == -1)
        {//a return value of -1 indicate a VN_SOCKET error
            printf("VN_SOCKET error during KAST command recv()\n");
            //end the communication properly even if the commands fails
            CirrusCom_End_connection(sock);
            return VN_eERR_SocketRecvError;
        }
        else if (nbBytesRead != 4)
        {//we are supposed to read exactly 4 bytes here
            printf("Error during KAST command recv : insufficient number of bytes (wanted %d, received %d)\n",4,nbBytesRead);
            CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
            return VN_eERR_SocketIncompleteCom;
        }

        if(response[0]!='K' || response[1]!='A' || response[2]!='S' || response[3]!='T')
        {
            if(response[0]=='E' && response[1]=='R' && response[2]=='R')
            {
                printf("Recv: ERR -> Unknown command");
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_UnknownCommand;
            }
            else
            {
                response[3]='\0';
                printf("Recv: %s -> communication error", response);
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_Failure;
            }
        }
        else
        {
            //read the uint32 status and uint32 version response
            char buffer[8];
            nbBytesRead=CirrusCom_Receive(sock, (char*)buffer, sizeof(buffer), 0);
            if(nbBytesRead == -1)
            {//a return value of -1 indicate a VN_SOCKET error
                printf("Error in KAST command : VN_SOCKET error when receiving command result\n");
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_SocketRecvError;
            }
            else if (nbBytesRead != sizeof(buffer))
            {//we are supposed to read exactly 8 bytes here
                printf("Error during KAST command recv : insufficient number of bytes (wanted %d, received %d)\n",8,nbBytesRead);
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_SocketIncompleteCom;
            }

            VN_UINT32 status=*(int*)&buffer[0];
            VN_UINT32 version=*(int*)&buffer[4];
            enum statusFlags
            {
                cStatus_NoProblem=0,
                cStatus_NoFan=1<<0,
                cStatus_HighFPGATemp=1<<1,
                cStatus_HighCPUBoardTemp=1<<2,
            };

            if (status!=cStatus_NoProblem)
            {//expected status is 0 if the cirrus has no urgent problem
                printf("KAST command returned with status %d",status);
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_BadStatus;
            }

            if (version==1)
            {
                char miscData[12];
                nbBytesRead=CirrusCom_Receive(sock, (char*)miscData, sizeof(miscData), 0);
                CirrusCom_End_connection(sock);//end the communication properly (this was the last of the received data)
                if(nbBytesRead == -1)
                {//a return value of -1 indicate a VN_SOCKET error
                    printf("Error in KAST command : VN_SOCKET error when receiving command result\n");
                    return VN_eERR_SocketRecvError;
                }
                else if (nbBytesRead != sizeof(miscData))
                {//we are supposed to read exactly 8 bytes here
                    printf("Error during KAST command recv : insufficient number of bytes (wanted %d, received %d)\n",8,nbBytesRead);
                    return VN_eERR_SocketIncompleteCom;
                }

                int networkSpeed=*(int*)&miscData[0];
                if (networkSpeed<1000)
                {
                    printf("Error during KAST command result : network speed detected on the Cirrus end lower than expected (%d MBps < 1 GBps)\n",networkSpeed);
                    return VN_eERR_BadNetworkConfig;
                }

                float CirrusTemperature=*(float*)&miscData[4];
                printf("KAST command info : Cirrus reported temperature is %.2f°C\n",CirrusTemperature);
                int timeSincePowerUp=*(int*)&miscData[8];
                printf("KAST command info : Cirrus has been started for %d seconds\n",timeSincePowerUp);
            }
            else
            {
                printf("KAST command returned with unknown version %d\n",version);
                CirrusCom_End_connection(sock);//end the communication properly even if the commands fails
                return VN_eERR_BadHeader;
            }
        }
    }

    return VN_eERR_NoError;
}
/** @ingroup Config
 * \brief This function is used to initialize the VN_Cirrus3DHandler structure used by the get/set data functions
 * \param[in]  IPAddress        : IP address of the Cirrus3DSensor
 * \param[out] pCirrus3DHandler : Will contain the VN_Cirrus3DHandler structure initialized for the Cirrus3DSensor with ip address=IPAddress
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_Cirrus3DHandler_Initialize(char* pIpAddress,
                                           VN_Cirrus3DHandler *pCirrus3DHandler)
{
    VN_tERR_Code err=VN_eERR_NoError;
    VN_SOCKET sock;
    VN_UINT32 ackstatus;
    VN_UINT32 sizeData;
    int status;

    memset((char*)pCirrus3DHandler->IPAddress, '\0', VN_cSizeDataString);
    memset((char*)pCirrus3DHandler->SerialNumber, '\0', VN_cSizeDataString);
#ifdef WIN32
    memcpy_s((void*)pCirrus3DHandler->IPAddress,VN_cSizeDataString,pIpAddress,strlen(pIpAddress));
#else
    memcpy((void*)pCirrus3DHandler->IPAddress,pIpAddress,strlen(pIpAddress));
#endif

    *(VN_UINT32*)(&pCirrus3DHandler->ProtocolVersion)=VN_cProtocolVersion;
    *(VN_UINT32*)(&pCirrus3DHandler->HeaderVersion)=VN_cHeaderVersion;

    err=CirrusCom_connectToPort(pCirrus3DHandler->IPAddress,
                                  20002,
                                  &sock);
    if (err!=VN_eERR_NoError)
    {
        if ( VN_ExecuteCommandSTS((char*)pCirrus3DHandler->IPAddress, &status)==VN_eERR_NoError)
            return VN_eERR_IncompatibleProtocolVersion;
        else
            return err;
    }

    err=CirrusCom_setTimeout(sock,
                             VN_cSocketTimeout);
    if (err!=VN_eERR_NoError)
        return err;

    err=VN_COM_HeaderCmd_send(sock,
                              pCirrus3DHandler->ProtocolVersion,
                              pCirrus3DHandler->HeaderVersion,
                              VN_CmdId_InitializeProductHandler,
                              0);
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

    err=VN_COM_HeaderAck_receive(sock,
                                 (VN_UINT32*)(&pCirrus3DHandler->ProtocolVersion),
                                 (VN_UINT32*)(&pCirrus3DHandler->HeaderVersion),
                                 (VN_UINT32*)(&pCirrus3DHandler->DeviceVersion),
                                  &ackstatus,
                                  &sizeData);
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

    if(ackstatus!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return ackstatus;
    }

    int size= VN_cSizeDataString;
    if((int)sizeData==size)
    {
        err=CirrusCom_ReceiveBufferofSizeN(sock,
                                           (char*)pCirrus3DHandler->SerialNumber,
                                           size);
        if(err!=VN_eERR_NoError)
        {
            CirrusCom_End_connection(sock);
            return err;
        }
    }
    else
    {
        if((int)sizeData!=0)
        {
            CirrusCom_End_connection(sock);
            return VN_eERR_Failure;
        }
        else
            sprintf(pCirrus3DHandler->SerialNumber, "Unread");
    }

    //if reached here : command successful, close VN_SOCKET properly
    err = CirrusCom_End_connection(sock);
    if (err!=VN_eERR_NoError)
        return err;

    return VN_eERR_NoError;
}


/** @ingroup Config
 * \brief This function get the VN_Cirrus3DHandler structure used by the get/set data functions without initilize data in the CirrusSensor
 * \param[in]  IPAddress        : IP address of the Cirrus3DSensor
 * \param[out] pCirrus3DHandler : Will contain the VN_Cirrus3DHandler structure initialized for the Cirrus3DSensor with ip address=IPAddress
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_Cirrus3DHandler_Get(char* pIpAddress,
                                    VN_Cirrus3DHandler *pCirrus3DHandler)
{
    VN_tERR_Code err=VN_eERR_NoError;
    VN_SOCKET sock;
    VN_UINT32 ackstatus;
    VN_UINT32 sizeData;

    memset((char*)pCirrus3DHandler->IPAddress, '\0', VN_cSizeDataString);
#ifdef WIN32
    memcpy_s((void*)pCirrus3DHandler->IPAddress,VN_cSizeDataString,pIpAddress,strlen(pIpAddress));
#else
    memcpy((void*)pCirrus3DHandler->IPAddress,pIpAddress,strlen(pIpAddress));
#endif

    *(VN_UINT32*)(&pCirrus3DHandler->ProtocolVersion)=VN_cProtocolVersion;
    *(VN_UINT32*)(&pCirrus3DHandler->HeaderVersion)=VN_cHeaderVersion;

    err=CirrusCom_connectToPort(pCirrus3DHandler->IPAddress,
                                  20002,
                                  &sock);
    if (err!=VN_eERR_NoError)
        return err;

    err=CirrusCom_setTimeout(sock,
                             VN_cSmallSocketTimeout);
    if (err!=VN_eERR_NoError)
        return err;

    err=VN_COM_HeaderCmd_send(sock,
                              pCirrus3DHandler->ProtocolVersion,
                              pCirrus3DHandler->HeaderVersion,
                              VN_CmdId_GetProductHandler,
                              0);
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

    err=VN_COM_HeaderAck_receive(sock,
                                 (VN_UINT32*)(&pCirrus3DHandler->ProtocolVersion),
                                 (VN_UINT32*)(&pCirrus3DHandler->HeaderVersion),
                                 (VN_UINT32*)(&pCirrus3DHandler->DeviceVersion),
                                  &ackstatus,
                                  &sizeData);
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

    if(ackstatus!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return ackstatus;
    }

    int size= VN_cSizeDataString;
    if((int)sizeData!=size)
    {
        CirrusCom_End_connection(sock);
        return VN_eERR_Failure;
    }

    err=CirrusCom_ReceiveBufferofSizeN(sock,
                                       (char*)pCirrus3DHandler->SerialNumber,
                                       size);
    if(err!=VN_eERR_NoError)
    {
        CirrusCom_End_connection(sock);
        return err;
    }

    //if reached here : command successful, close VN_SOCKET properly
    err = CirrusCom_End_connection(sock);
    if (err!=VN_eERR_NoError)
        return err;

    return VN_eERR_NoError;
}


/** @ingroup Config
 * \brief This function is used to force device version
 * \param[in] pCirrus3DHandler : Data previously initialized with the VN_Cirrus3DHandler_initialize() function
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_Cirrus3DHandler_ForceVersionDevice(VN_Cirrus3DHandler *pCirrus3DHandler,
                                                   VN_UINT32 NewDeviceVersion)
{
    VN_SOCKET sock;
    VN_tERR_Code err = VN_eERR_NoError;
    VN_UINT32 ackstatus;
    VN_UINT32 sizeData;
    VN_UINT32 protocolVersion;
    VN_UINT32 headerVersion;
    VN_UINT32 deviceVersion;

    err = CirrusCom_connectToPort(pCirrus3DHandler->IPAddress,
                                  cPortConfiguration,
                                  &sock);
    if (err!=VN_eERR_NoError)
        return err;

    err=CirrusCom_setTimeout(sock,
                             VN_cSmallSocketTimeout);
    if (err!=VN_eERR_NoError)
        return err;

    err=VN_COM_HeaderCmd_send(sock,
                                pCirrus3DHandler->ProtocolVersion,
                                pCirrus3DHandler->HeaderVersion,
                                VN_CmdId_ForceDeviceVersion,
                                sizeof(int));
    if(err!=VN_eERR_NoError)
    {
        CirrusCom_End_connection(sock);
        return err;
    }

    err=CirrusCom_SendCommandWithSize(sock,
                                        (char*)&NewDeviceVersion,
                                        sizeof(NewDeviceVersion));
    if(err!=VN_eERR_NoError)
    {
        CirrusCom_End_connection(sock);
        return err;
    }

    err=VN_COM_HeaderAck_receive(sock,
                                   &protocolVersion,
                                   &headerVersion,
                                   &deviceVersion,
                                   &ackstatus,
                                   &sizeData);
    if(err!=VN_eERR_NoError)
    {
        CirrusCom_End_connection(sock);
        return err;
    }

    //if reached here : command successful, close VN_SOCKET properly
    err = CirrusCom_End_connection(sock);
    if (err!=VN_eERR_NoError)
        return err;

    if(pCirrus3DHandler->ProtocolVersion!=protocolVersion) return VN_eERR_Failure;
    if(pCirrus3DHandler->HeaderVersion!=headerVersion) return VN_eERR_Failure;
    if(ackstatus!=VN_eERR_NoError)
    {
        if(ackstatus==VN_eERR_UnknownCommand)
            return VN_eERR_IncompatibleDeviceVersion;
        else
            return ackstatus;
    }

    if(deviceVersion!=NewDeviceVersion) return VN_eERR_Failure;
    pCirrus3DHandler->DeviceVersion=NewDeviceVersion;


    return VN_eERR_NoError;
}


/** @ingroup Config
 * \brief This function is used to reset the data in the CirrusSensor
 * \param[in] pCirrus3DHandler : Will contain the VN_Cirrus3DHandler structure initialized for the Cirrus3DSensor with ip address=IPAddress
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_Cirrus3DHandler_ResetData(VN_Cirrus3DHandler *pCirrus3DHandler)
{
    VN_SOCKET sock;
    VN_tERR_Code err = VN_eERR_NoError;
    VN_UINT32 ackstatus;
    VN_UINT32 sizeData;
    VN_UINT32 protocolVersion;
    VN_UINT32 headerVersion;
    VN_UINT32 deviceVersion;

    err = CirrusCom_connectToPort(pCirrus3DHandler->IPAddress,
                                  cPortConfiguration,
                                  &sock);
    if (err!=VN_eERR_NoError)
        return err;

    err=CirrusCom_setTimeout(sock,
                             VN_cSmallSocketTimeout);
    if (err!=VN_eERR_NoError)
        return err;

    err=VN_COM_HeaderCmd_send(sock,
                                pCirrus3DHandler->ProtocolVersion,
                                pCirrus3DHandler->HeaderVersion,
                                VN_CmdId_ResetProductHandler,
                                sizeof(int));
    if(err!=VN_eERR_NoError)
    {
        CirrusCom_End_connection(sock);
        return err;
    }

    err=VN_COM_HeaderAck_receive(sock,
                                   &protocolVersion,
                                   &headerVersion,
                                   &deviceVersion,
                                   &ackstatus,
                                   &sizeData);
    if(err!=VN_eERR_NoError)
    {
        CirrusCom_End_connection(sock);
        return err;
    }

    if(pCirrus3DHandler->ProtocolVersion!=protocolVersion) return VN_eERR_Failure;
    if(pCirrus3DHandler->HeaderVersion!=headerVersion) return VN_eERR_Failure;
    if(pCirrus3DHandler->DeviceVersion!=deviceVersion) return VN_eERR_Failure;
    if(ackstatus!=VN_eERR_NoError) return ackstatus;

    //if reached here : command successful, close VN_SOCKET properly
    err = CirrusCom_End_connection(sock);
    if (err!=VN_eERR_NoError)
        return err;

    return VN_eERR_NoError;
}


/** @ingroup Config
 * \brief This function is used to get the list of configuration datas
 * \param[in]  pCirrus3DHandler : Data previously initialized with the VN_Cirrus3DHandler_Initialize() function
 * \param[out] pNbParamInList   : Number of data available
 * \param[out] DataList         : List of data available
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_Cirrus3DHandler_GetDataRootList(const VN_Cirrus3DHandler *pCirrus3DHandler,
                                                VN_INT32 *pNbParamInList,
                                                VN_tDataRoot DataList[VN_cSizeList])
{
    VN_SOCKET sock;
    VN_tERR_Code err=VN_eERR_NoError;
    VN_UINT32 ackstatus;
    VN_UINT32 sizeData;
    VN_UINT32 protocolVersion;
    VN_UINT32 headerVersion;
    VN_UINT32 deviceVersion;

    err=CirrusCom_connectToPort(pCirrus3DHandler->IPAddress,
                                  cPortConfiguration,
                                  &sock);
    if (err!=VN_eERR_NoError)
        return err;

    err=CirrusCom_setTimeout(sock,
                             VN_cSmallSocketTimeout);
    if (err!=VN_eERR_NoError)
        return err;

    err=VN_COM_HeaderCmd_send(sock,
                                pCirrus3DHandler->ProtocolVersion,
                                pCirrus3DHandler->HeaderVersion,
                                VN_CmdId_GetDataList,
                                0);
    if(err!=VN_eERR_NoError)
    {
        CirrusCom_End_connection(sock);
        return err;
    }

    err=VN_COM_HeaderAck_receive(sock,
                                   &protocolVersion,
                                   &headerVersion,
                                   &deviceVersion,
                                   &ackstatus,
                                   &sizeData);
    if(err!=VN_eERR_NoError)
    {
        CirrusCom_End_connection(sock);
        return err;
    }

    if(pCirrus3DHandler->ProtocolVersion!=protocolVersion) return VN_eERR_Failure;
    if(pCirrus3DHandler->HeaderVersion!=headerVersion) return VN_eERR_Failure;
    //if(pCirrus3DHandler->deviceVersion!=deviceVersion) return VN_eERR_Failure;

    int size=sizeof(int);
    err=CirrusCom_ReceiveBufferofSizeN(sock,
                                       (char*)pNbParamInList,
                                        size);
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }
    size=sizeof(VN_tDataRoot);
    if(sizeData!=sizeof(VN_tDataRoot)*(*pNbParamInList)+sizeof(int)) return VN_eERR_Failure;

    size=sizeof(VN_tDataRoot)*(*pNbParamInList);
    err=CirrusCom_ReceiveBufferofSizeN(sock,
                                       (char*)DataList,
                                       size);
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

    err=CirrusCom_End_connection(sock);
    if (err!=VN_eERR_NoError)
        return err;

    return VN_eERR_NoError;
}

/** @ingroup Config
 * \brief This function is used to get the data
 * \param[in] pCirrus3DHandler : Data previously initialized with the VN_Cirrus3DHandler_Initialize() function
 * \param[in] DataId           : Data Index
 * \param[in] DataType         : Data Type
 * \param[out] pData           : Data Structure of DataType type.
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 * \brief Example :
 * \snippet main.c VN_Cirrus3DHandler_GetData
 */
VN_tERR_Code VN_Cirrus3DHandler_GetData(const VN_Cirrus3DHandler *pCirrus3DHandler,
                                        VN_UINT32 dataId,
                                        VN_eDataType dataType,
                                        VN_tDataRoot* pData)
{
    VN_SOCKET sock;
    VN_tERR_Code err=VN_eERR_NoError;
    VN_UINT32 ackstatus;
    VN_UINT32 sizeData;
    VN_UINT32 protocolVersion;
    VN_UINT32 headerVersion;
    VN_UINT32 deviceVersion;

    err=CirrusCom_connectToPort(pCirrus3DHandler->IPAddress,
                                  cPortConfiguration,
                                  &sock);
    if (err!=VN_eERR_NoError)
        return err;

    err=CirrusCom_setTimeout(sock,
                             VN_cSmallSocketTimeout);
    if (err!=VN_eERR_NoError)
        return err;

    err=VN_COM_HeaderCmd_send(sock,
                                pCirrus3DHandler->ProtocolVersion,
                                pCirrus3DHandler->HeaderVersion,
                                VN_CmdId_GetData,
                                sizeof(int));
    if(err!=VN_eERR_NoError)
    {
        CirrusCom_End_connection(sock);
        return err;
    }

    err=CirrusCom_SendCommandWithSize(sock,
                                        (char*)&dataId,
                                        sizeof(int));
    if(err!=VN_eERR_NoError)
    {
        CirrusCom_End_connection(sock);
        return err;
    }

    err=VN_COM_HeaderAck_receive(sock,
                                   &protocolVersion,
                                   &headerVersion,
                                   &deviceVersion,
                                   &ackstatus,
                                   &sizeData);
    if(err!=VN_eERR_NoError)
    {
        CirrusCom_End_connection(sock);
        return err;
    }

    if(pCirrus3DHandler->ProtocolVersion!=protocolVersion) err=VN_eERR_Failure;
    if(pCirrus3DHandler->HeaderVersion!=headerVersion) err=VN_eERR_Failure;
    if(pCirrus3DHandler->DeviceVersion!=deviceVersion) err=VN_eERR_Failure;
    if(ackstatus!=VN_eERR_NoError) err=ackstatus;

    if(err!=VN_eERR_NoError)
    {
        CirrusCom_End_connection(sock);
        return err;
    }

    int size= 0;

    switch (dataType) {
    case VN_eTypeInt32:
        size=sizeof(VN_tDataInt32);
        break;
    case VN_eTypeReal32:
        size=sizeof(VN_tDataReal32);
        break;
    case VN_eTypeString:
        size=sizeof(VN_tDataString);
        break;
    default:
        break;
    }
    if(size==0) return VN_eERR_Failure;

    err=CirrusCom_ReceiveBufferofSizeN(sock,
                                       (char*)pData,
                                       size);
    if(err!=VN_eERR_NoError)
    {
        CirrusCom_End_connection(sock);
        return err;
    }
    if((int)sizeData!=size)
    {
        CirrusCom_End_connection(sock);
        return VN_eERR_Failure;
    }

    err=CirrusCom_End_connection(sock);
    if (err!=VN_eERR_NoError)
        return err;

    return VN_eERR_NoError;

}


/** @ingroup Config
 * \brief This function is used to get the data value
 * \param[in] pCirrus3DHandler : Data previously initialized with the VN_Cirrus3DHandler_Initialize() function
 * \param[in,out] pDataInt32   : Get the value of the data. The structure was initialized by calling VN_Cirrus3DHandler_GetData(), VN_Cirrus3DHandler_GetDataInt32(), VN_Cirrus3DHandler_GetDataReal32() or VN_Cirrus3DHandler_GetDataString() function
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_Cirrus3DHandler_GetDataValue(const VN_Cirrus3DHandler *pCirrus3DHandler,
                                             VN_tDataRoot* pData)
{
    VN_SOCKET sock;
    VN_tERR_Code err=VN_eERR_NoError;
    VN_UINT32 ackstatus;
    VN_UINT32 sizeData;
    VN_UINT32 protocolVersion;
    VN_UINT32 headerVersion;
    VN_UINT32 deviceVersion;
    int size;

    if(pData->magicNumber!=VN_cMagicNumber) return VN_eERR_Failure;

    err = CirrusCom_connectToPort(pCirrus3DHandler->IPAddress,
                                  cPortConfiguration,
                                  &sock);
    if (err!=VN_eERR_NoError)
        return err;

    err=CirrusCom_setTimeout(sock,
                             VN_cSmallSocketTimeout);
    if (err!=VN_eERR_NoError)
        return err;

    err=VN_COM_HeaderCmd_send(sock,
                                pCirrus3DHandler->ProtocolVersion,
                                pCirrus3DHandler->HeaderVersion,
                                VN_CmdId_GetDataValue,
                                sizeof(int));
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

    err=CirrusCom_SendCommandWithSize(sock,
                                        (char*)&pData->paramId,
                                        sizeof(int));
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

    err=VN_COM_HeaderAck_receive(sock,
                                   &protocolVersion,
                                   &headerVersion,
                                   &deviceVersion,
                                   &ackstatus,
                                   &sizeData);
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

    if(pCirrus3DHandler->ProtocolVersion!=protocolVersion) err=VN_eERR_Failure;
    if(pCirrus3DHandler->HeaderVersion!=headerVersion) err=VN_eERR_Failure;
    if(pCirrus3DHandler->DeviceVersion!=deviceVersion) err=VN_eERR_Failure;
    if(ackstatus!=VN_eERR_NoError) err=ackstatus;
    if(sizeData!=sizeof(VN_INT32)) err=VN_eERR_Failure;
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

    size=sizeData;
    char* value;
    switch (pData->paramtype) {
    case VN_eTypeInt32:
        value=(char*)&(((VN_tDataInt32*)pData)->value);
        break;
    case VN_eTypeReal32:
        value=(char*)&(((VN_tDataReal32*)pData)->value);
        break;
    case VN_eTypeString:
        value=(char*)(((VN_tDataString*)pData)->value);
        break;
    default:
        CirrusCom_End_connection(sock);
        return VN_eERR_UnknownData;
        break;
    }
    err=CirrusCom_ReceiveBufferofSizeN(sock,
                                       value,
                                       size);
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

    //if reached here : command successful, close VN_SOCKET properly
    err=CirrusCom_End_connection(sock);
    if (err!=VN_eERR_NoError)
        return err;

    return VN_eERR_NoError;

}


/** @ingroup Config
 * \brief This function is used to set the data value
 * \param[in] pCirrus3DHandler : Data previously initialized with the VN_Cirrus3DHandler_Initialize() function
 * \param[in,out] pData        : Get the data value. The structure was initialized by calling VN_Cirrus3DHandler_GetData(), VN_Cirrus3DHandler_GetDataInt32(), VN_Cirrus3DHandler_GetDataReal32() or VN_Cirrus3DHandler_GetDataString() function
 * \param[in] value            : new value of the data in the product.
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_Cirrus3DHandler_SetDataValue(const VN_Cirrus3DHandler *pCirrus3DHandler,
                                             VN_tDataRoot* pData)
{
    VN_SOCKET sock;
    VN_tERR_Code err=VN_eERR_NoError;
    VN_UINT32 ackstatus;
    VN_UINT32 sizeData;
    VN_UINT32 protocolVersion;
    VN_UINT32 headerVersion;
    VN_UINT32 deviceVersion;

    if(pData->magicNumber!=VN_cMagicNumber) return VN_eERR_Failure;

    err = CirrusCom_connectToPort(pCirrus3DHandler->IPAddress,
                                  cPortConfiguration,
                                  &sock);
    if (err!=VN_eERR_NoError)
        return err;

    err=CirrusCom_setTimeout(sock,
                             VN_cSmallSocketTimeout);
    if (err!=VN_eERR_NoError)
        return err;

    err=VN_COM_HeaderCmd_send(sock,
                                pCirrus3DHandler->ProtocolVersion,
                                pCirrus3DHandler->HeaderVersion,
                                VN_CmdId_SetDataValue,
                                sizeof(int)*2);
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

    err=CirrusCom_SendCommandWithSize(sock,
                                        (char*)&pData->paramId,
                                        sizeof(int));
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

    char* value;
    int sizeValue;
    switch (pData->paramtype) {
    case VN_eTypeInt32:
        value=(char*)&(((VN_tDataInt32*)pData)->value);
        sizeValue=sizeof(((VN_tDataInt32*)pData)->value);
        break;
    case VN_eTypeReal32:
        value=(char*)&(((VN_tDataReal32*)pData)->value);
        sizeValue=sizeof(((VN_tDataReal32*)pData)->value);
        break;
    case VN_eTypeString:
        value=(char*)&(((VN_tDataString*)pData)->value);
        sizeValue=sizeof(((VN_tDataString*)pData)->value);
        break;
    default:
        CirrusCom_End_connection(sock);
        return VN_eERR_Failure;
        break;
    }
    err=CirrusCom_SendCommandWithSize(sock,
                                        value,
                                        sizeValue);
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

    err=VN_COM_HeaderAck_receive(sock,
                                   &protocolVersion,
                                   &headerVersion,
                                   &deviceVersion,
                                   &ackstatus,
                                   &sizeData);
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

    if(pCirrus3DHandler->ProtocolVersion!=protocolVersion) err=VN_eERR_BadHeader;
    if(pCirrus3DHandler->HeaderVersion!=headerVersion) err=VN_eERR_BadHeader;
    if(pCirrus3DHandler->DeviceVersion!=deviceVersion) err=VN_eERR_BadHeader;
    if(ackstatus!=VN_eERR_NoError) err=ackstatus;
    if(sizeData!=0) err=VN_eERR_BadHeader;

    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

    err=CirrusCom_End_connection(sock);
    if (err!=VN_eERR_NoError)
        return err;

    return VN_eERR_NoError;
}




/** @ingroup GeneralSetting
 * \brief This function is used to get Cirrus3D available
 * \param[in,out] pNbCirrus3DAvailable                         : Number
 * \param[in,out] ipCirrus3DAvailable[20][VN_cSizeDataString]  : IPAddress of cirrus available
 * \retval VN_eERR_NoError if success
 * \retval Error code otherwise
 */
VN_tERR_Code VN_GetCirrus3DAvailable(VN_INT32 *pNbCirrus3DAvailable, char ipCirrus3DAvailable[VN_cMaxNbCirrus3D][VN_cSizeDataString])
{
    memset(ipCirrus3DAvailable, '\0', VN_cMaxNbCirrus3D*VN_cSizeDataString*sizeof(char));

    VN_tERR_Code err=VN_eERR_NoError;
    VN_tERR_Code errCommand=VN_eERR_NoError;
    const int MAXBUF=50;
    int sock, status, sinlen;
    int option=1;
    char Recvbuffer[50];
    const int buflen=MAXBUF;
    const int SOCKETERROR=-1;
    /* Port for the protocol UDP */
    int portBroadcast=60100;
    const char header[15]="Vn_CirrusSensor";

    *pNbCirrus3DAvailable=0;

    struct sockaddr_in sock_in;
    sinlen=sizeof(struct sockaddr_in);
    memset(&sock_in, 0, sinlen);

    struct timeval read_timeout;
    read_timeout.tv_sec=1;
    read_timeout.tv_usec=0;

    sock=socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(sock==SOCKETERROR) return VN_eERR_SocketError;

    status=setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (const char*)(&option), sizeof(option));
    if(status==SOCKETERROR) return VN_eERR_SocketError;

    status=setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)(&read_timeout), sizeof(read_timeout));
    if(status==SOCKETERROR) return VN_eERR_SocketError;

    sock_in.sin_addr.s_addr=htonl(-1); /* send message to 255.255.255.255 */
    sock_in.sin_port=htons(portBroadcast); /* port number */
    sock_in.sin_family=PF_INET;

    status=sendto(sock, header, sizeof(header), 0, (struct sockaddr *)&sock_in, sinlen);
    if(status==SOCKETERROR) return VN_eERR_SocketError;

    while (status != -1)
    {
        memset(Recvbuffer, 0, buflen);
#if _LINUX_COMPILATION_
        socklen_t socklent;
        status=recvfrom(sock, Recvbuffer, buflen, 0, (struct sockaddr *)&sock_in, &socklent);

#else
        status=recvfrom(sock, Recvbuffer, buflen, 0, (struct sockaddr *)&sock_in, &sinlen);

#endif

        if ((status != -1))
        {
            char *pch=strtok(&Recvbuffer[0], ",");
            if(pch==NULL || strncmp(pch,header,15))
            {
                //printf("Product detected is not a cirrusSensor");
            }
            else
            {
                pch=strtok(NULL,",");

                if(pch==NULL || atoi(pch)!=0)
                {
                    printf( "Error in the detection cirrusSensor");
                }
                else
                {
                    pch=strtok(NULL,",");
                    if(pch!=NULL)
                    {
                        if(strlen(pch)<VN_cSizeDataString)
                        {
                            memcpy(ipCirrus3DAvailable[*pNbCirrus3DAvailable],pch, strlen(pch));
                            (*pNbCirrus3DAvailable)++;
                        }
                        else
                        {
                            printf("Error in reading IPAddress");
                            errCommand=VN_eERR_Failure;
                        }
                    }
                    else
                    {
                        printf("Error in reading IPAddress");
                        errCommand=VN_eERR_Failure;

                    }
                }
            }
        }
    }

    shutdown(sock, 2);

    err=CirrusCom_End_connection(sock);
    if (err!=VN_eERR_NoError)
        return err;

    return errCommand;
}


/**************************************************************************/


VN_tERR_Code VN_COM_HeaderCmd_send(VN_SOCKET sock,
                                   VN_UINT32 protocolVersion,
                                   VN_UINT32 headerVersion,
                                   VN_UINT32 CmdId,
                                   VN_UINT32 sizeData)
{
    VN_tERR_Code err;
    VN_HeaderCmdSDK HeaderCommandSDK;

    HeaderCommandSDK.protocolVersion =protocolVersion;
    HeaderCommandSDK.headerVersion =headerVersion;
    HeaderCommandSDK.CmdId =CmdId;
    HeaderCommandSDK.sizeData =sizeData;

    err=CirrusCom_SendCommandWithSize(sock,
                                        (char*)&HeaderCommandSDK,
                                        sizeof(HeaderCommandSDK));
    if(err!=VN_eERR_NoError)
    {
        //end the communication properly even if the commands fails
        CirrusCom_End_connection(sock);
        return err;
    }

    return VN_eERR_NoError;
}

VN_tERR_Code VN_COM_HeaderAck_send(VN_SOCKET sock,
                                   VN_UINT32 protocolVersion,
                                   VN_UINT32 headerVersion,
                                   VN_UINT32 deviceVersion,
                                   VN_UINT32 CmdId,
                                   VN_UINT32 Ackstatus,
                                   VN_UINT32 sizeData)
{
    VN_tERR_Code vn_err=VN_eERR_NoError;
    VN_HeaderAckSDK HeaderAckSDK;

    HeaderAckSDK.protocolVersion =protocolVersion;
    HeaderAckSDK.headerVersion =headerVersion;
    HeaderAckSDK.deviceVersion =deviceVersion;
    HeaderAckSDK.CmdId =CmdId;
    HeaderAckSDK.Ackstatus =Ackstatus;
    HeaderAckSDK.sizeData =sizeData;

    vn_err=CirrusCom_SendCommandWithSize(sock,
                                         (char*)&HeaderAckSDK,
                                         sizeof(HeaderAckSDK));

    return vn_err;
}

VN_tERR_Code VN_COM_HeaderAck_receive(VN_SOCKET sock,
                                      VN_UINT32 *protocolVersion,
                                      VN_UINT32 *headerVersion,
                                      VN_UINT32 *deviceVersion,
                                      VN_UINT32 *Ackstatus,
                                      VN_UINT32 *sizeData)
{
    VN_tERR_Code err;
    VN_HeaderAckSDK HeaderAckSDK;
    int size= sizeof(HeaderAckSDK);

    err=CirrusCom_ReceiveBufferofSizeN(sock,
                                       (char*)&HeaderAckSDK,
                                        size);
    if(err!=VN_eERR_NoError)
    {
        CirrusCom_End_connection(sock);
        return err;
    }

    *protocolVersion=HeaderAckSDK.protocolVersion;
    *headerVersion=HeaderAckSDK.headerVersion;
    *deviceVersion=HeaderAckSDK.deviceVersion;
    *Ackstatus=HeaderAckSDK.Ackstatus;
    *sizeData=HeaderAckSDK.sizeData;

    return VN_eERR_NoError;
}


VN_tERR_Code VN_COM_HeaderCmd_receive(VN_SOCKET sock,
                                      VN_UINT32 *protocolVersion,
                                      VN_UINT32 *headerVersion,
                                      VN_UINT32 *CmdId,
                                      VN_UINT32 *sizeData)
{
    VN_tERR_Code err;
    VN_HeaderCmdSDK HeaderCommandSDK;
    int size=sizeof(VN_HeaderCmdSDK);

    err=CirrusCom_ReceiveBufferofSizeN(sock,
                                       (char*)&HeaderCommandSDK,
                                        size);
    if(err!=VN_eERR_NoError)
    {
        CirrusCom_End_connection(sock);
        return err;
    }

    *protocolVersion=HeaderCommandSDK.protocolVersion;
    *headerVersion=HeaderCommandSDK.headerVersion;
    *CmdId=HeaderCommandSDK.CmdId;
    *sizeData=HeaderCommandSDK.sizeData;

    return VN_eERR_NoError;
}
