#include "VN_SDK_CloudSavingFunctions.h"
#include <math.h> //include NAN

/*------------------------------------------------------------------------------
 *  History:
 *  16/02/2024: NEW: new function VN_SaveMatrixXYZRGBAsDepthMapBmp.......... P.H
 *  16/05/2024: CHANGE: VN_SaveCloudAsBinFile-> VN_SaveXYZCloudAsBinFile
 *                      VN_SaveCloudAsTxtFile-> VN_SaveXYZCloudAsTxtFile
 *                      VN_SaveAsPcd->VN_SaveXYZI16CloudAsPcdFile
 *                      VN_SaveAsDepthMapBmp->VN_SaveXYZI8CloudAsDepthMapBmp
 *                      VN_SaveMatrixCloudAsPCD->VN_SaveXYZI8CloudAsPcdFile
 *                      VN_SaveMatrixXYZRGBAsDepthMapBmp->VN_SaveXYZRGBCloudAsPcdFile
 *              NEW : VN_SaveXYZI8CloudAsDepthMapBmp function
 *                    VN_SaveXYZI8CloudAsLumBmp
 *                    VN_SaveXYZRGBCloudAsDepthMapBmp
 *                    VN_SaveXYZRGBCloudAsLumBmp............................ P.H
 *  24/08/2024: CHANGE: in VN_SaveXYZI8CloudAsDepthMapBmp, the color indicating
 *                  no valid point changed from black to white (to better see
 *                  the limits of the image)................................ F.R
 *------------------------------------------------------------------------------
 */

/***************************** API FUNCTIONS ******************************/
/**************************************************************************/

/**
 * \brief This function can be used to save a cloud of points (XYZ format) as a .bin file
 *
 * \param[in] pCloud_XYZ    : pointer to a buffer storing the XYZ cloud
 * \param[in] cloudSize     : number of points in the cloud
 * \param[in] pFileName     : path of the .bin result file
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code VN_SaveXYZCloudAsBinFile(float *pCloud_XYZ, int cloudSize, const char *pFileName)
{
    FILE *fptr;

    //open file to be written to
    fptr = fopen(pFileName,"wb");//"wb" : open file for writing (or overwriting if already exists) in binary mode
    if (fptr == NULL)
    {
        printf("Failed to create file %s\n",pFileName);
        return VN_eERR_FileAccessError;
    }

    //write the number of points (as a binary int32)
    int nbItemsWritten = fwrite(&cloudSize,sizeof(int),1,fptr);
    if (nbItemsWritten!=1)
    {
        printf("Error when writing file %s\n",pFileName);
        fclose(fptr);
        return VN_eERR_FileWriteError;
    }

    //write the list of points (each point as 3 32-bits float X,Y,Z)
    nbItemsWritten = fwrite(pCloud_XYZ,3*4,cloudSize,fptr);
    if (nbItemsWritten!=cloudSize)
    {
        printf("Error when writing file %s : only %d points instead of the expected %d\n",pFileName,nbItemsWritten,cloudSize);
        fclose(fptr);
        return VN_eERR_FileWriteError;
    }

    //flush and close file
    if (fclose(fptr)!=0)
        return VN_eERR_FileWriteError;

    return VN_eERR_NoError;
}
/**
 * \brief This function can be used to save a cloud of points (XYZ format) as a .bin file
 *
 * \param[in] pCloud_XYZ    : pointer to a buffer storing the XYZ cloud
 * \param[in] cloudSize     : number of points in the cloud
 * \param[in] pFileName     : path of the .txt result file
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code VN_SaveXYZCloudAsTxtFile(float *pCloud_XYZ, int cloudSize, const char *pFileName)
{
    FILE *fptr;

    //open file to be written to
    fptr = fopen(pFileName,"w");//"wb" : open file for writing (or overwriting if already exists) in ascii mode
    if (fptr == NULL)
    {
        printf("Failed to create file %s\n",pFileName);
        return VN_eERR_FileAccessError;
    }

    //Write 3d points
    for(int pt=0 ; pt<cloudSize ; pt++)
    {
        int offset=pt*3;
        fprintf(fptr,"%f %f %f\n",*(pCloud_XYZ+offset),*(pCloud_XYZ+offset+1),*(pCloud_XYZ+offset+2));
    }

    //flush and close file
    if (fclose(fptr)!=0)
        return VN_eERR_FileWriteError;

    return VN_eERR_NoError;
}

/**
 * \brief This function can be used to save a cloud of points (XYZI16 format in this example) as a .pcd file
 *
 * \param[in] pCloud_XYZI6    : pointer to a buffer storing the XYZI16 cloud
 * \param[in] cloudSize     : number of points in the cloud
 * \param[in] pFileName     : path of the .pcd result file
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code VN_SaveXYZI16CloudAsPcdFile(VN_Point_XYZI16 *pCloud_XYZI16,
                                     int rowsNumber,
                                     int columnsNumber,
                                     const char *pFileName)
{
    /*This function is designed to save the scan result (3d points + luminance)
     * as a .pcd (Point Cloud Data) ascii file. This is a non-optimized universal 3D file format,
     * which can be useful for compatibility with other softwares
     * The filePath parameter is supposed to link to a .pcd file.
     */

    FILE *fptr;

    //open file to be written to
    fptr = fopen(pFileName,"w");//"w" : open file for writing (or overwriting if already exists) in ascii mode
    if (fptr == NULL)
    {
        printf("Failed to create file %s\n",pFileName);
        return VN_eERR_FileAccessError;
    }

    //Write some context
    fprintf(fptr,"#This is a XYZi cloud of points saved with file format .PCD version 0.7\n");
    //write fields specified in .pcd format
    fprintf(fptr,"VERSION 0.7\n");
    fprintf(fptr,"FIELDS x y z rgb\n");
    fprintf(fptr,"SIZE 4 4 4 4\n");//4 bytes per datum
    fprintf(fptr,"TYPE F F F U\n");//datum format : float for the points coord, uint32 for the rgb info
    fprintf(fptr,"COUNT 1 1 1 1\n");//1 dimension per data
    fprintf(fptr,"WIDTH %d\n",columnsNumber);//number of columns of the matrix of points
    fprintf(fptr,"HEIGHT %d\n",rowsNumber);//number of rows of the matrix of points
    fprintf(fptr,"VIEWPOINT 0 0 0 1 0 0 0\n");//Default viewpoint
    fprintf(fptr,"POINTS %d\n",columnsNumber*rowsNumber);//the number of points
    fprintf(fptr,"DATA ascii\n");//data stored as ascii here ; could also be stored in binary format for a more compressed file

    //Write 3d points
    for(int pt=0 ; pt<columnsNumber*rowsNumber ; pt++)
    {
        VN_Point_XYZI16 point = pCloud_XYZI16[pt];
        //Compute rgb value corresponding with i luminance value :
        VN_UINT32 I = (VN_UINT32) point.i;//first convert from uint16 to uint32 so that the bitshift can work correctly
        I = I>>8;//to simplify conversion to rgb, only use the 8 MSB of the luminance...
        VN_UINT32 rgb = 0xff000000 | (I<<16) | (I<<8) | (I);

        //write the line corresponding to the 3d point
        fprintf(fptr,"%f %f %f %u\n",point.x,point.y,point.z,rgb);
    }

    //flush and close file
    if (fclose(fptr)!=0)
        return VN_eERR_FileWriteError;

    return VN_eERR_NoError;
}

/**
 * \brief This function can be used to save a cloud of points (XYZI8 format) as a .pcd file,
 *  in this case a file with organized rows & columnds and the point data stored in binary format.
 *
 * \param[in] pCloud_XYZI8  : pointer to a buffer storing the XYZI8 cloud
 * \param[in] rowCount      : number of rows in the matrix
 * \param[in] columnCount   : number of columns in the matrix
 * \param[in] pFileName     : path of the .pcd result file
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code VN_SaveXYZI8CloudAsPcdFile(VN_Point_XYZI8 *pCloud_XYZI8,
                               int rowsNumber,
                               int columnsNumber,
                               const char *pFileName)
{
    /*This function is designed to save the scan result (3d points + luminance)
     * as a .pcd (Point Cloud Data) ascii file. This is a non-optimized universal 3D file format,
     * which can be useful for compatibility with other softwares
     * The filePath parameter is supposed to link to a .pcd file.
     */

    FILE *fptr;

    //open file to be written to
    fptr = fopen(pFileName,"w");//"w" : open file for writing (or overwriting if already exists) in ascii mode
    if (fptr == NULL)
    {
        printf("Failed to create file %s\n",pFileName);
        return VN_eERR_FileAccessError;
    }

    //Write some context
    fprintf(fptr,"#This is a XYZi cloud of points saved with file format .PCD version 0.7\n");
    //write fields specified in .pcd format
    fprintf(fptr,"VERSION 0.7\n");
    fprintf(fptr,"FIELDS x y z rgb\n");
    fprintf(fptr,"SIZE 4 4 4 4\n");//4 bytes per datum
    fprintf(fptr,"TYPE F F F U\n");//datum format : float for the points coord, uint32 for the rgb info
    fprintf(fptr,"COUNT 1 1 1 1\n");//1 dimension per data
    fprintf(fptr,"WIDTH %d\n",columnsNumber);//number of columns of the matrix of points
    fprintf(fptr,"HEIGHT %d\n",rowsNumber);//number of rows of the matrix of points
    fprintf(fptr,"VIEWPOINT 0 0 0 1 0 0 0\n");//Default viewpoint
    fprintf(fptr,"POINTS %d\n",columnsNumber*rowsNumber);//the number of points
    fprintf(fptr,"DATA ascii\n");//data stored as ascii here ; could also be stored in binary format for a more compressed file

    //Write 3d points
    for(int pt=0 ; pt<columnsNumber*rowsNumber ; pt++)
    {
        VN_Point_XYZI8 point = pCloud_XYZI8[pt];
        //Compute rgb value corresponding with i luminance value :
        VN_UINT32 I = (VN_UINT32) point.i;//first convert from uint8 to uint32 so that the bitshift can work correctly
        I = I>>8;//to simplify conversion to rgb, only use the 8 MSB of the luminance...
        VN_UINT32 rgb = 0xff000000 | (I<<16) | (I<<8) | (I);

        //write the line corresponding to the 3d point
        fprintf(fptr,"%f %f %f %u\n",point.x,point.y,point.z,rgb);
    }

    //flush and close file
    if (fclose(fptr)!=0)
        return VN_eERR_FileWriteError;

    return VN_eERR_NoError;
}


/**
 * \brief This function can be used to save a matrix_XYZi cloud of points as a depth map bmp image
 *
 * \param[in] pCloud_XYZI8   : pointer to a buffer storing the matrix
 * \param[in] rowCount      : number of rows in the matrix
 * \param[in] columnCount   : number of columns in the matrix
 * \param[in] pFileName     : path of the .bmp result file
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code VN_SaveXYZI8CloudAsDepthMapBmp(VN_Point_XYZI8 *pCloud_XYZI8, int rowCount, int columnCount, const char *pFileName)
{
    /*A depth map image is an image where the color of the pixel corresponds to the height
     *  of the 3d point it represents. */

    FILE *fptr;

    //open file to be written to
    fptr = fopen(pFileName,"wb");//"wb" : open file for writing (or overwriting if already exists) in binary mode
    if (fptr == NULL)
    {
        printf("Failed to create file %s\n",pFileName);
        return VN_eERR_FileAccessError;
    }

    //write the header of the bmp file (according to the standard bmp format... use dedicated image processing libraries
    // if you have them, it is easier)
    int h=rowCount;//height of the image
    int w=columnCount;//width of the image
    int filesize = 54 + 3*w*h;  //w is your image width, h is image height, both int

    unsigned char bmpfileheader[14] = {'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0};
    unsigned char bmpinfoheader[40] = {40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 24,0};
    unsigned char bmppad[3] = {0,0,0};

    bmpfileheader[ 2] = (unsigned char)(filesize    );
    bmpfileheader[ 3] = (unsigned char)(filesize>> 8);
    bmpfileheader[ 4] = (unsigned char)(filesize>>16);
    bmpfileheader[ 5] = (unsigned char)(filesize>>24);

    bmpinfoheader[ 4] = (unsigned char)(       w    );
    bmpinfoheader[ 5] = (unsigned char)(       w>> 8);
    bmpinfoheader[ 6] = (unsigned char)(       w>>16);
    bmpinfoheader[ 7] = (unsigned char)(       w>>24);
    bmpinfoheader[ 8] = (unsigned char)(       h    );
    bmpinfoheader[ 9] = (unsigned char)(       h>> 8);
    bmpinfoheader[10] = (unsigned char)(       h>>16);
    bmpinfoheader[11] = (unsigned char)(       h>>24);

    fwrite(bmpfileheader,1,14,fptr);
    fwrite(bmpinfoheader,1,40,fptr);


    //write the actual image data

    //first step : get the scale of the cloud of points (i.e. zmin & zmax) ;
    // here it is dynamic (i.e. it varies depending on each cloud of points)
    // but if you want to be able to analyse & compare images it would
    // be better to use a fixed scale adapted to the Cirrus size
    float zmin=0;
    float zmax=1;
    char initDone=0;
    for(int ptIndex=0 ; ptIndex<(rowCount*columnCount) ; ptIndex++)
    {
        VN_Point_XYZI8 pt = pCloud_XYZI8[ptIndex];
        if (pt.i!=0)//i.e. if the point is non-zero, because we can't generate a point if there is no light on it...
        {
            float z=pt.z;
            if (initDone == 0)
            {//this is the first point found in the matrix, use it to initialize min & max
                zmin=zmax=z;
                initDone=1;
            }
            else
            {
                if (z<zmin) zmin=z;
                if (z>zmax) zmax=z;
            }
        }
    }

    //now that we have a scale : for each point, compute the corresponding rgb values
    //note : the algorithm below is not optimized...
    VN_UINT8 *pImgData = (VN_UINT8 *)malloc(3*w*h);
    if (pImgData == NULL)
    {
        printf("Error : failed to allocate enough memory for the image data\n");
        fclose(fptr);
        return VN_eERR_InsufficientMemory;
    }

    VN_UINT8 r,g,b;
    if (zmax==zmin) zmin=zmax-1;//to avoid a division by zero error in the unlikely scenario where all points have strictly the same height...
    float zFactor = 1/(zmax-zmin);
    for(int v=0 ; v<rowCount ; v++)
        for(int u=0 ; u<columnCount ; u++)
        {
            int ptIndex = v*columnCount+u;
            VN_Point_XYZI8 pt = pCloud_XYZI8[ptIndex];
            if (pt.i!=0)//i.e. point is non-zero
            {
                float z=pt.z;

                float gray_buff = (z-zmin)*zFactor;
                float red,green,blue;

                if (gray_buff<(1.0/3))
                {
                    blue= 1;
                    green= gray_buff*3;
                    red= 0;
                }
                else if (gray_buff<(2.0/3))
                {
                    blue= 1-((gray_buff-(1.0/3))*3);
                    green= 1;
                    red= 0;
                }
                else
                {
                    blue= 0;
                    green= 1;
                    red= (gray_buff-(2.0/3))*3;
                }

                r = (VN_UINT8)(255.0*red);
                g = (VN_UINT8)(255.0*green);
                b = (VN_UINT8)(255.0*blue);
            }
            else
            {
                r=g=b=255;
            }
            pImgData[3*ptIndex+0]=r;
            pImgData[3*ptIndex+1]=g;
            pImgData[3*ptIndex+2]=b;
        }

    //write image data in bmp file (including padding if needed... the bmp format requires each binary line length to start at a 4-byte aligned address)
    for(int i=0; i<h; i++)
    {
        fwrite(pImgData+(w*(h-i-1)*3),3,w,fptr);//write line
        fwrite(bmppad,1,(4-(w*3)%4)%4,fptr);//write padding (if any)
    }

    //free image buffer
    free(pImgData);
    //flush and close file
    if (fclose(fptr)!=0)
        return VN_eERR_FileWriteError;

    return VN_eERR_NoError;
}

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
VN_tERR_Code VN_SaveXYZRGBCloudAsDepthMapBmp(VN_Point_XYZRGB *pCloud_XYZRGB, int rowCount, int columnCount, const char *pFileName)
{
    /*A depth map image is an image where the color of the pixel corresponds to the height
     *  of the 3d point it represents. */

    FILE *fptr;

    //open file to be written to
    fptr = fopen(pFileName,"wb");//"wb" : open file for writing (or overwriting if already exists) in binary mode
    if (fptr == NULL)
    {
        printf("Failed to create file %s\n",pFileName);
        return VN_eERR_FileAccessError;
    }

    //write the header of the bmp file (according to the standard bmp format... use dedicated image processing libraries
    // if you have them, it is easier)
    int h=rowCount;//height of the image
    int w=columnCount;//width of the image
    int filesize = 54 + 3*w*h;  //w is your image width, h is image height, both int

    unsigned char bmpfileheader[14] = {'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0};
    unsigned char bmpinfoheader[40] = {40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 24,0};
    unsigned char bmppad[3] = {0,0,0};

    bmpfileheader[ 2] = (unsigned char)(filesize    );
    bmpfileheader[ 3] = (unsigned char)(filesize>> 8);
    bmpfileheader[ 4] = (unsigned char)(filesize>>16);
    bmpfileheader[ 5] = (unsigned char)(filesize>>24);

    bmpinfoheader[ 4] = (unsigned char)(       w    );
    bmpinfoheader[ 5] = (unsigned char)(       w>> 8);
    bmpinfoheader[ 6] = (unsigned char)(       w>>16);
    bmpinfoheader[ 7] = (unsigned char)(       w>>24);
    bmpinfoheader[ 8] = (unsigned char)(       h    );
    bmpinfoheader[ 9] = (unsigned char)(       h>> 8);
    bmpinfoheader[10] = (unsigned char)(       h>>16);
    bmpinfoheader[11] = (unsigned char)(       h>>24);

    fwrite(bmpfileheader,1,14,fptr);
    fwrite(bmpinfoheader,1,40,fptr);


    //write the actual image data

    //first step : get the scale of the cloud of points (i.e. zmin & zmax) ;
    // here it is dynamic (i.e. it varies depending on each cloud of points)
    // but if you want to be able to analyse & compare images it would
    // be better to use a fixed scale adapted to the Cirrus size
    float zmin=0;
    float zmax=1;
    char initDone=0;
    for(int ptIndex=0 ; ptIndex<(rowCount*columnCount) ; ptIndex++)
    {
        VN_Point_XYZRGB pt = pCloud_XYZRGB[ptIndex];
        if (!isnan(pt.x))//i.e. if the point is non-zero
        {
            float z=pt.z;
            if (initDone == 0)
            {//this is the first point found in the matrix, use it to initialize min & max
                zmin=zmax=z;
                initDone=1;
            }
            else
            {
                if (z<zmin) zmin=z;
                if (z>zmax) zmax=z;
            }
        }
    }

    //now that we have a scale : for each point, compute the corresponding rgb values
    //note : the algorithm below is not optimized...
    VN_UINT8 *pImgData = (VN_UINT8 *)malloc(3*w*h);
    if (pImgData == NULL)
    {
        printf("Error : failed to allocate enough memory for the image data\n");
        fclose(fptr);
        return VN_eERR_InsufficientMemory;
    }

    VN_UINT8 r,g,b;
    if (zmax==zmin) zmin=zmax-1;//to avoid a division by zero error in the unlikely scenario where all points have strictly the same height...
    float zFactor = 1/(zmax-zmin);
    for(int v=0 ; v<rowCount ; v++)
        for(int u=0 ; u<columnCount ; u++)
        {
            int ptIndex = v*columnCount+u;
            VN_Point_XYZRGB pt = pCloud_XYZRGB[ptIndex];
            if (!isnan(pt.x))//i.e. if the point is non-zero
            {
                float z=pt.z;

                float gray_buff = (z-zmin)*zFactor;
                float red,green,blue;

                if (gray_buff<(1.0/3))
                {
                    blue= 1;
                    green= gray_buff*3;
                    red= 0;
                }
                else if (gray_buff<(2.0/3))
                {
                    blue= 1-((gray_buff-(1.0/3))*3);
                    green= 1;
                    red= 0;
                }
                else
                {
                    blue= 0;
                    green= 1;
                    red= (gray_buff-(2.0/3))*3;
                }

                r = (VN_UINT8)(255.0*red);
                g = (VN_UINT8)(255.0*green);
                b = (VN_UINT8)(255.0*blue);
            }
            else
            {
                r=g=b=0;
            }
            pImgData[3*ptIndex+0]=r;
            pImgData[3*ptIndex+1]=g;
            pImgData[3*ptIndex+2]=b;
        }

    //write image data in bmp file (including padding if needed... the bmp format requires each binary line length to start at a 4-byte aligned address)
    for(int i=0; i<h; i++)
    {
        fwrite(pImgData+(w*(h-i-1)*3),3,w,fptr);//write line
        fwrite(bmppad,1,(4-(w*3)%4)%4,fptr);//write padding (if any)
    }

    //free image buffer
    free(pImgData);
    //flush and close file
    if (fclose(fptr)!=0)
        return VN_eERR_FileWriteError;

    return VN_eERR_NoError;
}


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
VN_tERR_Code VN_SaveXYZRGBCloudAsLumBmp(VN_Point_XYZRGB *pCloud_XYZRGB, int rowCount, int columnCount, const char *pFileName)
{
    /*A depth map image is an image where the color of the pixel corresponds to the height
     *  of the 3d point it represents. */

    FILE *fptr;

    //open file to be written to
    fptr = fopen(pFileName,"wb");//"wb" : open file for writing (or overwriting if already exists) in binary mode
    if (fptr == NULL)
    {
        printf("Failed to create file %s\n",pFileName);
        return VN_eERR_FileAccessError;
    }

    //write the header of the bmp file (according to the standard bmp format... use dedicated image processing libraries
    // if you have them, it is easier)
    int h=rowCount;//height of the image
    int w=columnCount;//width of the image
    int filesize = 54 + 3*w*h;  //w is your image width, h is image height, both int

    unsigned char bmpfileheader[14] = {'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0};
    unsigned char bmpinfoheader[40] = {40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 24,0};
    unsigned char bmppad[3] = {0,0,0};

    bmpfileheader[ 2] = (unsigned char)(filesize    );
    bmpfileheader[ 3] = (unsigned char)(filesize>> 8);
    bmpfileheader[ 4] = (unsigned char)(filesize>>16);
    bmpfileheader[ 5] = (unsigned char)(filesize>>24);

    bmpinfoheader[ 4] = (unsigned char)(       w    );
    bmpinfoheader[ 5] = (unsigned char)(       w>> 8);
    bmpinfoheader[ 6] = (unsigned char)(       w>>16);
    bmpinfoheader[ 7] = (unsigned char)(       w>>24);
    bmpinfoheader[ 8] = (unsigned char)(       h    );
    bmpinfoheader[ 9] = (unsigned char)(       h>> 8);
    bmpinfoheader[10] = (unsigned char)(       h>>16);
    bmpinfoheader[11] = (unsigned char)(       h>>24);

    fwrite(bmpfileheader,1,14,fptr);
    fwrite(bmpinfoheader,1,40,fptr);


    //write the actual image data
    //note : the algorithm below is not optimized...
    VN_UINT8 *pImgData = (VN_UINT8 *)malloc(3*w*h);
    if (pImgData == NULL)
    {
        printf("Error : failed to allocate enough memory for the image data\n");
        fclose(fptr);
        return VN_eERR_InsufficientMemory;
    }

    for(int v=0 ; v<rowCount ; v++)
        for(int u=0 ; u<columnCount ; u++)
        {
            VN_UINT8 lum;

            int ptIndex = v*columnCount+u;
            VN_Point_XYZRGB pt = pCloud_XYZRGB[ptIndex];

            if (!isnan(pt.rgb))//i.e. if the point is non-zero
            {
                float rgb = pt.rgb;
                int rgb_int = *(int*)(&rgb);
                lum = (rgb_int>>8)&0xff;
            }
            else
                lum = 0;

            pImgData[3*ptIndex+0]=lum;
            pImgData[3*ptIndex+1]=lum;
            pImgData[3*ptIndex+2]=lum;
        }

    //write image data in bmp file (including padding if needed... the bmp format requires each binary line length to start at a 4-byte aligned address)
    for(int i=0; i<h; i++)
    {
        fwrite(pImgData+(w*(h-i-1)*3),3,w,fptr);//write line
        fwrite(bmppad,1,(4-(w*3)%4)%4,fptr);//write padding (if any)
    }

    //free image buffer
    free(pImgData);
    //flush and close file
    if (fclose(fptr)!=0)
        return VN_eERR_FileWriteError;

    return VN_eERR_NoError;
}

/**
 * \brief This function can be used to save a cloud of points (XYZRGB format) as a .pcd file,
 *  in this case a file with organized rows & columnds and the point data stored in binary format.
 *
 * \param[in] pCloud_XYZRGB : pointer to a buffer storing the XYZRGB cloud
 * \param[in] cloudSize     : number of points in the cloud
 * \param[in] pFileName     : path of the .pcd result file
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code VN_SaveXYZRGBCloudAsPcdFile(VN_Point_XYZRGB *pCloud_XYZRGB,
                                     int rowsNumber,
                                     int columnsNumber,
                                     const char *pFileName)
{
    /*This function is designed to save the scan result (3d points + regb data)
     * as a .pcd (Point Cloud Data) binary file. */

    FILE *fptr;

    //open file to be written to
    fptr = fopen(pFileName,"wb");//"wb" : open file for writing (or overwriting if already exists) in binary mode
    if (fptr == NULL)
    {
        printf("Failed to create file %s\n",pFileName);
        return VN_eERR_FileAccessError;
    }

    //Write some context
    fprintf(fptr,"#This is a XYZi cloud of points saved with file format .PCD version 0.7\n");
    //write fields specified in .pcd format
    fprintf(fptr,"VERSION 0.7\n");
    fprintf(fptr,"FIELDS x y z rgb\n");
    fprintf(fptr,"SIZE 4 4 4 4\n");//4 bytes per datum
    fprintf(fptr,"TYPE F F F F\n");//datum format : float for the points coord, float for the rgb info
    fprintf(fptr,"COUNT 1 1 1 1\n");//1 dimension per data
    fprintf(fptr,"WIDTH %d\n",columnsNumber);//number of columns of the matrix of points
    fprintf(fptr,"HEIGHT %d\n",rowsNumber);//number of rows of the matrix of points
    fprintf(fptr,"VIEWPOINT 0 0 0 1 0 0 0\n");//Default viewpoint
    fprintf(fptr,"POINTS %d\n",columnsNumber*rowsNumber);//the number of points that will be stored in this file (including empty points, with nan coordinates)
    fprintf(fptr,"DATA binary\n");//data stored as binary here

    //Write 3d points data (which is already in the correct format)
    size_t writtenBlocks = fwrite((void*)pCloud_XYZRGB,sizeof(VN_Point_XYZRGB),rowsNumber*columnsNumber,fptr);
    if (writtenBlocks != (size_t)rowsNumber*columnsNumber)
    {
        printf("Write error when writing in file %s\n",pFileName);
        fclose(fptr);
        return VN_eERR_FileAccessError;
    }

    //flush and close file
    if (fclose(fptr)!=0)
        return VN_eERR_FileWriteError;

    return VN_eERR_NoError;
}

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
VN_tERR_Code VN_SaveXYZI8CloudAsLumBmp(VN_Point_XYZI8 *pCloud_XYZI8, int rowCount, int columnCount, const char *pFileName)
{
    /*A depth map image is an image where the color of the pixel corresponds to the height
     *  of the 3d point it represents. */

    FILE *fptr;

    //open file to be written to
    fptr = fopen(pFileName,"wb");//"wb" : open file for writing (or overwriting if already exists) in binary mode
    if (fptr == NULL)
    {
        printf("Failed to create file %s\n",pFileName);
        return VN_eERR_FileAccessError;
    }

    //write the header of the bmp file (according to the standard bmp format... use dedicated image processing libraries
    // if you have them, it is easier)
    int h=rowCount;//height of the image
    int w=columnCount;//width of the image
    int filesize = 54 + 3*w*h;  //w is your image width, h is image height, both int

    unsigned char bmpfileheader[14] = {'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0};
    unsigned char bmpinfoheader[40] = {40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 24,0};
    unsigned char bmppad[3] = {0,0,0};

    bmpfileheader[ 2] = (unsigned char)(filesize    );
    bmpfileheader[ 3] = (unsigned char)(filesize>> 8);
    bmpfileheader[ 4] = (unsigned char)(filesize>>16);
    bmpfileheader[ 5] = (unsigned char)(filesize>>24);

    bmpinfoheader[ 4] = (unsigned char)(       w    );
    bmpinfoheader[ 5] = (unsigned char)(       w>> 8);
    bmpinfoheader[ 6] = (unsigned char)(       w>>16);
    bmpinfoheader[ 7] = (unsigned char)(       w>>24);
    bmpinfoheader[ 8] = (unsigned char)(       h    );
    bmpinfoheader[ 9] = (unsigned char)(       h>> 8);
    bmpinfoheader[10] = (unsigned char)(       h>>16);
    bmpinfoheader[11] = (unsigned char)(       h>>24);

    fwrite(bmpfileheader,1,14,fptr);
    fwrite(bmpinfoheader,1,40,fptr);


    //write the actual image data
    //note : the algorithm below is not optimized...
    VN_UINT8 *pImgData = (VN_UINT8 *)malloc(3*w*h);
    if (pImgData == NULL)
    {
        printf("Error : failed to allocate enough memory for the image data\n");
        fclose(fptr);
        return VN_eERR_InsufficientMemory;
    }

    for(int v=0 ; v<rowCount ; v++)
        for(int u=0 ; u<columnCount ; u++)
        {
            int ptIndex = v*columnCount+u;
            VN_Point_XYZI8 pt = pCloud_XYZI8[ptIndex];

            pImgData[3*ptIndex+0]=pt.i;
            pImgData[3*ptIndex+1]=pt.i;
            pImgData[3*ptIndex+2]=pt.i;
        }

    //write image data in bmp file (including padding if needed... the bmp format requires each binary line length to start at a 4-byte aligned address)
    for(int i=0; i<h; i++)
    {
        fwrite(pImgData+(w*(h-i-1)*3),3,w,fptr);//write line
        fwrite(bmppad,1,(4-(w*3)%4)%4,fptr);//write padding (if any)
    }

    //free image buffer
    free(pImgData);
    //flush and close file
    if (fclose(fptr)!=0)
        return VN_eERR_FileWriteError;

    return VN_eERR_NoError;
}
