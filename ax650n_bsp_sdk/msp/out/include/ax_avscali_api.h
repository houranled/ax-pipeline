/**************************************************************************************************
 *
 * Copyright (c) 2019-2024 Axera Semiconductor Co., Ltd. All Rights Reserved.
 *
 * This source file is the property of Axera Semiconductor Co., Ltd. and
 * may not be copied or distributed in any isomorphic form without the prior
 * written consent of Axera Semiconductor Co., Ltd.
 *
 **************************************************************************************************/

#ifndef __AX_AVSCALI_API_H__
#define __AX_AVSCALI_API_H__

#include <string.h>
#include "ax_base_type.h"
#include "ax_avs_api.h"
#include "ax_isp_3a_api.h"

#ifdef __cplusplus
    extern "C" {
#endif

#define AX_AVSCALI_MAX_PATH_LEN    (512)
#define AX_AVSCALI_IP_ADDR_LEN     (15)
#define AX_AVSCALI_ON_SITE_CALI_DATA_FOLDER    "onsite"

/* error code */
#define AX_AVSCALI_SUCC                        (0)
#define AX_ERR_AVSCALI_NULL_PTR                AX_DEF_ERR(AX_ID_AVSCALI, 1, AX_ERR_NULL_PTR)
#define AX_ERR_AVSCALI_ILLEGAL_PARAM           AX_DEF_ERR(AX_ID_AVSCALI, 1, AX_ERR_ILLEGAL_PARAM)
#define AX_ERR_AVSCALI_NOT_INIT                AX_DEF_ERR(AX_ID_AVSCALI, 1, AX_ERR_NOT_INIT)
#define AX_ERR_AVSCALI_FILE_UNEXIST            AX_DEF_ERR(AX_ID_AVSCALI, 1, AX_ERR_UNEXIST)
#define AX_ERR_AVSCALI_TIMEOUT                 AX_DEF_ERR(AX_ID_AVSCALI, 1, AX_ERR_TIMED_OUT)
#define AX_ERR_AVSCALI_SYS_NOTREADY            AX_DEF_ERR(AX_ID_AVSCALI, 1, AX_ERR_SYS_NOTREADY)
#define AX_ERR_AVSCALI_UNKNOWN                 AX_DEF_ERR(AX_ID_AVSCALI, 1, AX_ERR_UNKNOWN)
#define AX_ERR_AVSCALI_NOT_SUPPORT             AX_DEF_ERR(AX_ID_AVSCALI, 1, AX_ERR_NOT_SUPPORT)
#define AX_ERR_AVSCALI_INITED                  AX_DEF_ERR(AX_ID_AVSCALI, 1, AX_ERR_EXIST)
#define AX_ERR_AVSCALI_NOMEM                   AX_DEF_ERR(AX_ID_AVSCALI, 1, AX_ERR_NOMEM)
#define AX_ERR_AVSCALI_CALI_FAIL               AX_DEF_ERR(AX_ID_AVSCALI, 1, 0x80)
#define AX_ERR_AVSCALI_GEO_CALI_FAIL           AX_DEF_ERR(AX_ID_AVSCALI, 1, 0x81)
#define AX_ERR_AVSCALI_DATA_IMCOMPATIBLE       AX_DEF_ERR(AX_ID_AVSCALI, 1, 0x82)
#define AX_ERR_AVSCALI_EMPTY_FILE              AX_DEF_ERR(AX_ID_AVSCALI, 1, 0x83)
#define AX_ERR_AVSCALI_DATA_BROKEN             AX_DEF_ERR(AX_ID_AVSCALI, 1, 0x84)

typedef struct axAVSCALI_PIPE_SEQ_INFO_T {
    AX_U8   nPipeNum;                     // Pipe number
    AX_U8   arrPipeSeq[AX_AVS_PIPE_NUM];  // Pipe ids sequence: form left to right
} AX_AVSCALI_PIPE_SEQ_INFO_T, *AX_AVSCALI_PIPE_SEQ_INFO_PTR;

typedef struct axAVSCALI_PIPE_INFO_T {
    AX_AVSCALI_PIPE_SEQ_INFO_T stPipeSeqInfo;
    AX_U8   nChn;                                   // ISP chn
    AX_U32  nImgWidth;                              // Pipe out imgage width
    AX_U32  nImgHeight;                             // Pipe out imgage height
} AX_AVSCALI_PIPE_INFO_T, *AX_AVSCALI_PIPE_INFO_PTR;

typedef struct axAVSCALI_AEAWB_SYNC_RATIO_DATA_T
{
    AX_U8 nPipeId;
    AX_ISP_IQ_AE_SYNC_RATIO_T  stAeSyncRatio;
    AX_ISP_IQ_AWB_SYNC_RATIO_T stAwbSyncRatio;
} AX_AVSCALI_AEAWB_SYNC_RATIO_DATA_T, *AX_AVSCALI_AEAWB_SYNC_RATIO_DATA_PTR;

typedef struct axAVSCALI_AEAWB_SYNC_RATIO_INFO_T
{
    AX_AVSCALI_PIPE_SEQ_INFO_T stPipeSeqInfo;
    AX_AVSCALI_AEAWB_SYNC_RATIO_DATA_T  stAeAwbSyncRatioData[AX_AVS_PIPE_NUM];
} AX_AVSCALI_AEAWB_SYNC_RATIO_INFO_T, *AX_AVSCALI_AEAWB_SYNC_RATIO_INFO_PTR;

typedef struct axAVSCALI_GEO_CALIDONE_RESULT_T {
    AX_S32    nResult;
    AX_BOOL   bUpdateOverlap;
    AX_VOID*  pPrivData;
} AX_AVSCALI_GEO_CALIDONE_RESULT_T, *AX_AVSCALI_GEO_CALIDONE_RESULT_PTR;

typedef struct axAVSCALI_ALL_CALIDONE_RESULT_T {
    AX_S32    nResult;
    AX_BOOL   bLscCaliIgnore;
    AX_BOOL   bGeoCaliIgnore;
    AX_BOOL   bAeAwbCaliIgnore;
    AX_BOOL   bIsOnSiteCali;
    AX_VOID*  pPrivData;
} AX_AVSCALI_ALL_CALIDONE_RESULT_T, *AX_AVSCALI_ALL_CALIDONE_RESULT_PTR;

typedef AX_VOID (*AX_AVSCALI_GeoCaliDone)(const AX_AVSCALI_GEO_CALIDONE_RESULT_T* pCaliDoneResult);
typedef AX_VOID (*AX_AVSCALI_AllCaliDone)(const AX_AVSCALI_ALL_CALIDONE_RESULT_T* pCaliDoneResult);

typedef struct axAVSCALI_CALLBACK_T
{
    AX_AVSCALI_GeoCaliDone GeoCaliDoneCb;
    AX_AVSCALI_AllCaliDone AllCaliDoneCb;
} AX_AVSCALI_CALLBACK_T, *AX_AVSCALI_CALLBACK_PTR;

typedef struct axAVSCALI_INIT_PARAM_T {
    AX_AVSCALI_PIPE_INFO_T  tPipeInfo;
    AX_AVSCALI_CALLBACK_T   tCallbacks;
    AX_CHAR                 strCaliDataPath[AX_AVSCALI_MAX_PATH_LEN]; // calibration data base path
    AX_VOID*                pPrivData;
} AX_AVSCALI_INIT_PARAM_T, *AX_AVSCALI_INIT_PARAM_PTR;

typedef struct axAVSCALI_STITCH_OVERLAP_DATA_T
{
    AX_U8 nPipeId;
    AX_ISP_OVERLAP_INFO_T stIspOverlapInfo[AX_VIN_OVERLAP_NUM];
} AX_AVSCALI_STITCH_OVERLAP_DATA_T, *AX_AVSCALI_STITCH_OVERLAP_DATA_PTR;

typedef struct axAVSCALI_STITCH_OVERLAP_INFO_T
{
    AX_BOOL bPrioOnsiteCali;                  // prioritize handle onsite calibration data
    AX_AVSCALI_PIPE_SEQ_INFO_T stPipeSeqInfo;
    AX_AVSCALI_STITCH_OVERLAP_DATA_T stIspOverlapData[AX_AVS_PIPE_NUM];
} AX_AVSCALI_STITCH_OVERLAP_INFO_T, *AX_AVSCALI_STITCH_OVERLAP_INFO_PTR;

typedef struct axAVSCALI_AVS_GRP_ATTR_T
{
    AX_BOOL bPrioOnsiteCali;                  // prioritize handle onsite calibration data
    AX_AVSCALI_PIPE_SEQ_INFO_T stPipeSeqInfo;
    AX_AVS_GRP_ATTR_T stAVSGrpAttr;
} AX_AVSCALI_AVS_GRP_ATTR_T, *AX_AVSCALI_AVS_GRP_ATTR_PTR;

//////////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize
///
/// @param pParams  [I]: initialize parameter
///
/// @return 0 if success, otherwise failure
//////////////////////////////////////////////////////////////////////////////////////
AX_S32 AX_AVSCALI_Init(AX_AVSCALI_INIT_PARAM_T* pParams);

//////////////////////////////////////////////////////////////////////////////////////
/// @brief uninitialize
///
/// @param NA
///
/// @return 0 if success, otherwise failure
//////////////////////////////////////////////////////////////////////////////////////
AX_S32 AX_AVSCALI_DeInit();

//////////////////////////////////////////////////////////////////////////////////////
/// @brief Start calibrate
///
/// @param pServerIP  [I]: server IP address
/// @param nPort      [I]: server port
///
/// @return 0 if success, otherwise failure
//////////////////////////////////////////////////////////////////////////////////////
AX_S32 AX_AVSCALI_Start(const AX_CHAR* pServerIP, const AX_U16 nPort);

//////////////////////////////////////////////////////////////////////////////////////
/// @brief Stop calibrate
///
/// @param NA
///
/// @return 0 if success, otherwise failure
//////////////////////////////////////////////////////////////////////////////////////
AX_S32 AX_AVSCALI_Stop();

//////////////////////////////////////////////////////////////////////////////////////
/// @brief Load AVS parameter
///
/// @param pParamPath   [I ]: path of avs parameter data.
///                           If pass nullptr, get current cali avs data after AX_AVSCALI_Init
/// @param pAVSGrpAttr  [IO]: avs group attribution
/// @param pCalibrated  [ O]: whether cali data is calibrated
///
/// @return 0 if success, otherwise failure
//////////////////////////////////////////////////////////////////////////////////////
AX_S32 AX_AVSCALI_LoadAvsParam(const AX_CHAR* pParamPath, AX_AVSCALI_AVS_GRP_ATTR_T* pAVSGrpAttr, AX_BOOL* pCalibrated);

//////////////////////////////////////////////////////////////////////////////////////
/// @brief Load AE/AWB sync ratio
///
/// @param pParamPath          [I ]: path of AE/AWB parameter data.
///                                  If pass nullptr, get current cali ae/awb ratio after AX_AVSCALI_Init
/// @param pAeAwbSyncRatioInfo [IO]: AeAwb sync ratios
///
/// @return 0 if success, otherwise failure
//////////////////////////////////////////////////////////////////////////////////////
AX_S32 AX_AVSCALI_LoadAeAwbSyncRatio(const AX_CHAR* pParamPath, AX_AVSCALI_AEAWB_SYNC_RATIO_INFO_T* pAeAwbSyncRatioInfo);

//////////////////////////////////////////////////////////////////////////////////////
/// @brief Load stitch overlap info
///
/// @param pParamPath         [I ]: path of stitch overlap data.
///                                 If pass nullptr, get current stitch overlap info after AX_AVSCALI_Init
/// @param pStitchOverlapInfo [IO]: stitch overlap info
///
/// @return 0 if success, otherwise failure
//////////////////////////////////////////////////////////////////////////////////////
AX_S32 AX_AVSCALI_LoadStitchOverlapInfo(const AX_CHAR* pParamPath, AX_AVSCALI_STITCH_OVERLAP_INFO_T* pStitchOverlapInfo);

//////////////////////////////////////////////////////////////////////////////////////
/// @brief Load lsc params
///
/// @param pParamPath         [I ]: path of lsc params.
///                                 If pass nullptr, get current lsc params after AX_AVSCALI_Init
/// @param nPipeId            [I ]: pipe id.
/// @param pLscParam          [ O]: lsc params
///
/// @return 0 if success, otherwise failure
//////////////////////////////////////////////////////////////////////////////////////
AX_S32 AX_AVSCALI_LoadLscParams(const AX_CHAR* pParamPath, const AX_U8 nPipeId, AX_ISP_IQ_LSC_PARAM_T* pLscParams);

#ifdef __cplusplus
}
#endif

#endif