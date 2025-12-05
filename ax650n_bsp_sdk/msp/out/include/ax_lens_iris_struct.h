/**************************************************************************************************
 *
 * Copyright (c) 2019-2024 Axera Semiconductor Co., Ltd. All Rights Reserved.
 *
 * This source file is the property of Axera Semiconductor Co., Ltd. and
 * may not be copied or distributed in any isomorphic form without the prior
 * written consent of Axera Semiconductor Co., Ltd.
 *
 **************************************************************************************************/

#ifndef _AX_LEN_IRIS_STRUCT_H_
#define _AX_LEN_IRIS_STRUCT_H_

#include "ax_base_type.h"
#include "ax_lens_af_struct.h"

#define ACTUATOR_MAX_NUM 8
#define AX_IRIS_MAX_STEP_FNO_NUM (1024)

typedef enum _AX_IRIS_TYPE_E_ {
    AX_IRIS_FIXED_TYPE   = 0,
    AX_IRIS_P_TYPE       = 1,
    AX_IRIS_DC_TYPE      = 2,
} AX_IRIS_TYPE_E;

typedef enum _AX_IRIS_F_NO_E_ {
    AX_IRIS_F_NO_32_0 = 0,
    AX_IRIS_F_NO_22_0,
    AX_IRIS_F_NO_16_0,
    AX_IRIS_F_NO_11_0,
    AX_IRIS_F_NO_8_0,
    AX_IRIS_F_NO_5_6,
    AX_IRIS_F_NO_4_0,
    AX_IRIS_F_NO_2_8,
    AX_IRIS_F_NO_2_0,
    AX_IRIS_F_NO_1_4,
    AX_IRIS_F_NO_1_0,
} AX_IRIS_F_NO_E;


typedef struct _AX_MANUAL_ATTR_T_T_ {
    AX_U32         nHoldValueForDcIris;   /* Automatic aperture correction value, used for DC-Iris debugging */
    AX_IRIS_F_NO_E eIrisFNOForPIris;      /* Manual aperture size, distinguished by aperture F-value, supports only P-Iris, does not support DC-Iris */
} AX_MANUAL_ATTR_T;


typedef struct _AX_IRIS_PARAMS_T_ {
    AX_U8 nEnable;                 /* Auto aperture enable 1: Auto aperture 0: Manual aperture */
    AX_IRIS_TYPE_E eIrisType;      /* Aperture type, fixed, DC-Iris, or P-Iris */
    AX_MANUAL_ATTR_T tManualAttr;  /* Manual aperture attribute settings structure */
} AX_IRIS_PARAMS_T;

typedef struct _AX_DCIRIS_PARAMS_T_ {
    AX_S32 kp;            /* Proportional gain */
    AX_S32 ki;            /* Integral gain */
    AX_S32 kd;            /* Derivative gain */
    AX_U32 nMinPwmDuty;   /* Minimum PWM duty cycle */
    AX_U32 nMaxPwmDuty;   /* Maximum PWM duty cycle */
    AX_U32 nOpenPwmDuty;  /* PWM duty cycle when the aperture is open */
} AX_DCIRIS_PARAMS_T;

typedef struct _AX_PIRIS_PARAMS_T_ {
    AX_U8 NStepFNOTableChange;                        /* Flag indicating whether the P-Iris stepper motor position to aperture F-value mapping table has been updated */
    AX_U8 NZeroIsMax;                                 /* Flag indicating whether Step 0 of the P-Iris stepper motor corresponds to the maximum aperture position */
    AX_U32 nTotalStep;                                /* Total steps of the P-Iris stepper motor */
    AX_U32 nStepCount;                                /* Available steps of the P-Iris stepper motor */
    AX_U32 nStepFNOTable[AX_IRIS_MAX_STEP_FNO_NUM];   /* P-Iris stepper motor position to F-value mapping table */
    AX_IRIS_F_NO_E eMaxIrisFNOTarget;                 /* Maximum aperture target value */
    AX_IRIS_F_NO_E eMinIrisFNOTarget;                 /* Minimum aperture target value */
    AX_U8 nFNOExValid;                                /* Flag indicating whether a higher precision aperture F-value equivalent gain is used in the AE allocation route when interfacing with P-Iris */
    AX_U32 nMaxIrisFNOGainTarget;                     /* Maximum aperture F-value equivalent gain target value */
    AX_U32 nMinIrisFNOGainTarget;                     /* Minimum aperture F-value equivalent gain target value */
} AX_PIRIS_PARAMS_T;


typedef struct _AX_LENS_ACTUATOR_IRIS_FUNC_T_ {
    AX_S32 (*pfn_iris_init)(AX_U8 nPipeId, AxAfMotorsType_s motorstype);
    AX_S32 (*pfn_iris_set_param)(AX_U8 nPipeId, AX_IRIS_PARAMS_T *pIrisParam);
    AX_S32 (*pfn_iris_get_param)(AX_U8 nPipeId, AX_IRIS_PARAMS_T *pIrisParam);
    AX_S32 (*pfn_dciris_set_param)(AX_U8 nPipeId, AX_DCIRIS_PARAMS_T *pDcIrisParam);
    AX_S32 (*pfn_dciris_get_param)(AX_U8 nPipeId, AX_DCIRIS_PARAMS_T *pDcIrisParam);
    AX_S32 (*pfn_piris_set_aperture_pos)(AX_U8 nPipeId, AX_S32 nPos);
    AX_S32 (*pfn_piris_get_aperture_pos)(AX_U8 nPipeId, AX_S32 *pPos);
    AX_S32 (*pfn_piris_set_param)(AX_U8 nPipeId, AX_PIRIS_PARAMS_T *pPIrisParam);
    AX_S32 (*pfn_piris_get_param)(AX_U8 nPipeId, AX_PIRIS_PARAMS_T *pPIrisParam);
    AX_S32 (*pfn_iris_exit)(AX_U8 nPipeId);
    AX_S32 (*pfn_dciris_set_duty)(AX_U8 nPipeId, AX_F32 nDuty);
    AX_S32 (*pfn_dciris_get_duty)(AX_U8 nPipeId, AX_F32 *pDuty);
} AX_LENS_ACTUATOR_IRIS_FUNC_T;

#endif

