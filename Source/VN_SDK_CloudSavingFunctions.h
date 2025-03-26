#ifndef VN_SDK_COMMANDFUNCTIONS_H
#define VN_SDK_COMMANDFUNCTIONS_H

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
 *  26/03/2025 ADDED: Export the functions available when using the library. P.H
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
EXPORT VN_tERR_Code VN_SaveXYZCloudAsBinFile(float *pCloud_XYZ, int cloudSize, const char *pFileName);

/**
 * \brief This function can be used to save a cloud of points (XYZ format) as a .bin file
 *
 * \param[in] pCloud_XYZ    : pointer to a buffer storing the XYZ cloud
 * \param[in] cloudSize     : number of points in the cloud
 * \param[in] pFileName     : path of the .txt result file
 * \retval VN_eERR_NoError if success
 * \retval error code otherwise
 */
EXPORT VN_tERR_Code VN_SaveXYZCloudAsTxtFile(float *pCloud_XYZ, int cloudSize, const char *pFileName);

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
EXPORT VN_tERR_Code VN_SaveXYZI16CloudAsPcdFile(VN_Point_XYZI16 *pCloud_XYZI16,
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
EXPORT VN_tERR_Code VN_SaveXYZI8CloudAsDepthMapBmp(VN_Point_XYZI8 *pCloud_XYZI8, int rowCount, int columnCount, const char *pFileName);

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
EXPORT VN_tERR_Code VN_SaveXYZI8CloudAsLumBmp(VN_Point_XYZI8 *pCloud_XYZI8, int rowCount, int columnCount, const char *pFileName);

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
EXPORT VN_tERR_Code VN_SaveXYZI8CloudAsPcdFile(VN_Point_XYZI8 *pCloud_XYZI8,
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
EXPORT VN_tERR_Code VN_SaveXYZRGBCloudAsDepthMapBmp(VN_Point_XYZRGB *pCloud_XYZRGB, int rowCount, int columnCount, const char *pFileName);


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
EXPORT VN_tERR_Code VN_SaveXYZRGBCloudAsLumBmp(VN_Point_XYZRGB *pCloud_XYZRGB, int rowCount, int columnCount, const char *pFileName);


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
EXPORT VN_tERR_Code VN_SaveXYZRGBCloudAsPcdFile(VN_Point_XYZRGB *pCloud_XYZRGB,
                                                int rowsNumber,
                                                int columnsNumber,
                                                const char *pFileName);
#endif // VN_SDK_COMMANDFUNCTIONS_H
