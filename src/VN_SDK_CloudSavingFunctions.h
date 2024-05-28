#ifndef VN_SDK_COMMANDFUNCTIONS_H
#define VN_SDK_COMMANDFUNCTIONS_H

/*------------------------------------------------------------------------------
 *  History:
 *  16/02/2024: NEW: new function VN_SaveMatrixXYZRGBAsDepthMapBmp.......... P.H
 *------------------------------------------------------------------------------
 */

#include "VN_SDK_Constant.h"

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
VN_tERR_Code VN_SaveCloudAsBinFile(float *pCloud_XYZ, int cloudSize, const char *pFileName);

/**
 * \brief This function can be used to save a cloud of points (XYZ format) as a .bin file
 *
 * \param[in] pCloud_XYZ    : pointer to a buffer storing the XYZ cloud
 * \param[in] cloudSize     : number of points in the cloud
 * \param[in] pFileName     : path of the .txt result file
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code VN_SaveCloudAsTxtFile(float *pCloud_XYZ, int cloudSize, const char *pFileName);

/**
 * \brief This function can be used to save a cloud of points (XYZI format in this example) as a .pcd file
 *
 * \param[in] pCloud_XYZI16    : pointer to a buffer storing the XYZI cloud
 * \param[in] cloudSize     : number of points in the cloud
 * \param[in] pFileName     : path of the .pcd result file
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code VN_SaveAsPcd(VN_Point_XYZI16 *pCloud_XYZI16, int cloudSize, const char *pFileName);

/**
 * \brief This function can be used to save a matrix_XYZI8 cloud of points as a depth map bmp image
 *
 * \param[in] pMatrix_XYZI8   : pointer to a buffer storing the matrix
 * \param[in] rowCount      : number of rows in the matrix
 * \param[in] columnCount   : number of columns in the matrix
 * \param[in] pFileName     : path of the .bmp result file
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code VN_SaveAsDepthMapBmp(VN_Point_XYZI8 *pMatrix_XYZI8, int rowCount, int columnCount, const char *pFileName);

/**
 * \brief This function can be used to save a cloud of points (XYZRGB format in this example) as a .pcd file,
 *  in this case a file with organized rows & columnds and the point data stored in binary format.
 *
 * \param[in] pCloud_XYZRGB    : pointer to a buffer storing the XYZI cloud
 * \param[in] cloudSize     : number of points in the cloud
 * \param[in] pFileName     : path of the .pcd result file
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code VN_SaveMatrixCloudAsPCD(VN_Point_XYZRGB *pCloud_XYZRGB, int rowsNumber, int columnsNumber, const char *pFileName);


/**
 * \brief This function can be used to save a matrix_XYZIRGB cloud of points as a depth map bmp image
 *
 * \param[in] pMatrix_XYZIRGB : pointer to a buffer storing the matrix
 * \param[in] rowCount        : number of rows in the matrix
 * \param[in] columnCount     : number of columns in the matrix
 * \param[in] pFileName       : path of the .bmp result file
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
VN_tERR_Code VN_SaveMatrixXYZRGBAsDepthMapBmp(VN_Point_XYZRGB *pMatrix_XYZRGB, int rowCount, int columnCount, const char *pFileName);
#endif // VN_SDK_COMMANDFUNCTIONS_H
