/****************************************************************************
* The confidential and proprietary information contained in this file may    *
* only be used by a person authorized under and to the extent permitted      *
* by a subsisting licensing agreement from ARM Limited or its affiliates.    *
*   (C) COPYRIGHT [2001-2017] ARM Limited or its affiliates.         *
*       ALL RIGHTS RESERVED                          *
* This entire notice must be reproduced on all copies of this file           *
* and copies of this file may only be made by a person if such person is     *
* permitted to do so under the terms of a subsisting license agreement       *
* from ARM Limited or its affiliates.                        *
*****************************************************************************/


/************* Include Files ****************/
#include "mbedtls_cc_mng.h"
#include "mbedtls_cc_mng_error.h"
#include "mbedtls_cc_mng_int.h"
#include "cc_pal_types.h"
#include "cc_pal_mem.h"
#include "cc_otp_defs.h"
#include "dx_id_registers.h"
#include "dx_crys_kernel.h"
#include "driver_defs.h"
#include "cc_pal_abort.h"
#include "cc_pal_mutex.h"
#include "cc_util_pm.h"

#include "da16200_map.h"
#include "cc_hal_plat.h"
#include "dx_reg_base_host.h"


extern CC_PalMutex CCSymCryptoMutex;
extern CC_PalMutex CCAsymCryptoMutex;
extern CC_PalMutex CCGenVecMutex;
extern CC_PalMutex CCApbFilteringRegMutex;

/************* Auxiliary API's *************/

static CCError_t setHwKeyToShadowReg(mbedtls_mng_keytype keyType, uint32_t *pHwKey, size_t keySize)
{
    CCError_t rc = CC_OK;
    uint32_t  wordIdx;
    uint32_t  regAddr = 0;
    uint32_t  regVal = 0;
    uint32_t  shft = 0;

    /* check input variables */
    if (pHwKey == NULL) {
        return CC_MNG_ILLEGAL_INPUT_PARAM_ERR;
    }

        switch (keyType) {
    case CC_MNG_HUK_KEY:
        if (AES_256_BIT_KEY_SIZE != keySize) {
            return CC_MNG_ILLEGAL_HUK_SIZE_ERR;
        }
        regAddr = DX_HOST_SHADOW_KDR_REG_REG_OFFSET;
                break;
    case CC_MNG_PROV_KEY:
        if (AES_128_BIT_KEY_SIZE != keySize) {
            return CC_MNG_ILLEGAL_HW_KEY_SIZE_ERR;
        }
        shft    = CC_MNG_HOST_KCP_LOCK_BIT_SHFT;
        regAddr = DX_HOST_SHADOW_KCP_REG_REG_OFFSET;
                break;
    case CC_MNG_CE_KEY:
        if (AES_128_BIT_KEY_SIZE != keySize) {
            return CC_MNG_ILLEGAL_HW_KEY_SIZE_ERR;
        }
        shft    = CC_MNG_HOST_KCE_LOCK_BIT_SHFT;
        regAddr = DX_HOST_SHADOW_KCE_REG_REG_OFFSET;
                break;
    case CC_MNG_ICV_PROV_KEY:
        if (AES_128_BIT_KEY_SIZE != keySize) {
            return CC_MNG_ILLEGAL_HW_KEY_SIZE_ERR;
        }
        shft    = CC_MNG_HOST_KPICV_LOCK_BIT_SHFT;
        regAddr = DX_HOST_SHADOW_KPICV_REG_REG_OFFSET;
                break;
    case CC_MNG_ICV_CE_KEY:
        if (AES_128_BIT_KEY_SIZE != keySize) {
            return CC_MNG_ILLEGAL_HW_KEY_SIZE_ERR;
        }
        shft    = CC_MNG_HOST_KCEICV_LOCK_BIT_SHFT;
        regAddr = DX_HOST_SHADOW_KCEICV_REG_REG_OFFSET;
                break;
    default:
        rc = CC_MNG_INVALID_KEY_TYPE_ERROR;
                break;
        }

        if ((CC_OK == rc) && (CC_MNG_HUK_KEY != keyType)) {
        /* read APB slave accesses control register */
        regVal = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, HOST_AO_LOCK_BITS));
        /* check if relevant HW key is already locked */
        if(CC_TRUE == ((regVal>>shft)&0x1)) {
            rc = CC_MNG_HW_KEY_IS_LOCKED_ERR;
        }
        }

        /* Load HW key (HUK / KCP / KCE / KPICV / KCEICV) */
    if (CC_OK == rc) {
        /* Set HW Key Value in the Shadow register */
        for (wordIdx = 0; wordIdx < (keySize>>2); wordIdx++) {
            CC_HAL_WRITE_REGISTER(regAddr, pHwKey[wordIdx]);
        }
    }

        return rc;
}

/***************** API's *******************/

/* The function reads the OEM flags OTP word (see Table 41: OEM-Programmed Flags (word 0x21 in OTP memory))
   and returns whether the RMA status is pending or not. */

int mbedtls_mng_pending_rma_status_get(uint32_t *isPendingRMA)
{
    int       rc        = CC_OK;
    uint32_t  regVal    = CC_MNG_INVALID_REG_VAL;
    uint32_t  lcsVal    = 0;
    uint32_t  isHbkFull = 0;
    CCError_t error     = CC_OK;

    *isPendingRMA = CC_FALSE;

    /* Read LCS Register */
    rc = mbedtls_mng_lcsGet(&lcsVal);
    if (CC_OK != rc) {
        return rc;
    }

    /* Check LCS value */
    if ((CC_MNG_LCS_DM != lcsVal) && (CC_MNG_LCS_SEC_ENABLED != lcsVal)) {
        return CC_MNG_ILLEGAL_OPERATION_ERR;
    }

    /* Read OTP Word #21 */
    rc = mbedtls_mng_otpWordRead(CC_OTP_OEM_FLAG_OFFSET, &regVal);
    if (CC_OK != rc) {
        return rc;
    }

    /* get HBK configuration */
    CC_IS_HBK_FULL(isHbkFull, error);
    if (CC_OK != error) {
        return error;
    }

    /* if device supports full Hbk, return with error */
    if (CC_TRUE == isHbkFull) {
        return CC_MNG_ILLEGAL_OPERATION_ERR;
    }

    regVal >>= CC_MNG_OEM_RMA_SHFT;
    regVal &= CC_MNG_OEM_RMA_MSK;

    switch (regVal) {
    case CC_MNG_NON_RMA:
        *isPendingRMA = CC_FALSE;
        break;
    case CC_MNG_PENDING_RMA:
        *isPendingRMA = CC_TRUE;
        break;
    case CC_MNG_ILLEGAL_STATE:
        rc = CC_MNG_RMA_ILLEGAL_STATE_ERR;
        *isPendingRMA = CC_FALSE;
        break;
    case CC_MNG_RMA:
        *isPendingRMA = CC_FALSE;
        break;
    default:
        break;
    }

    return rc;
}

int mbedtls_mng_hw_version_get(uint32_t *partNumber, uint32_t *revision)
{
    uint32_t pidReg[CC_MNG_PID_SIZE_WORDS] = {0};
    uint32_t cidReg[CC_MNG_CID_SIZE_WORDS] = {0};
    uint32_t pidVal[CC_MNG_PID_SIZE_WORDS] = {CC_MNG_PID_0_VAL, CC_MNG_PID_1_VAL, CC_MNG_PID_2_VAL, CC_MNG_PID_3_VAL, CC_MNG_PID_4_VAL};
    uint32_t cidVal[CC_MNG_CID_SIZE_WORDS] = {CC_MNG_CID_0_VAL, CC_MNG_CID_1_VAL, CC_MNG_CID_2_VAL, CC_MNG_CID_3_VAL};

    /* verify peripheral ID (PIDR) */
    pidReg[0] = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, PERIPHERAL_ID_0));
    pidReg[1] = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, PERIPHERAL_ID_1));
    pidReg[2] = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, PERIPHERAL_ID_2));
    pidReg[3] = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, PERIPHERAL_ID_3));
    pidReg[4] = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, PERIPHERAL_ID_4));

    if (0 != CC_PalMemCmp((uint8_t*)pidVal, (uint8_t*)pidReg, sizeof(pidVal))){
        return CC_MNG_ILLEGAL_PIDR_ERR;
    }

    /* verify component ID (CIDR) */
    cidReg[0] = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, COMPONENT_ID_0));
    cidReg[1] = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, COMPONENT_ID_1));
    cidReg[2] = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, COMPONENT_ID_2));
    cidReg[3] = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, COMPONENT_ID_3));

    if (0 != CC_PalMemCmp((uint8_t*)cidVal, (uint8_t*)cidReg, sizeof(cidVal))){
        return CC_MNG_ILLEGAL_CIDR_ERR;
    }

    *partNumber = (pidReg[0] & 0xFF) | ((pidReg[1] & 0x0F) << 8);
    *revision   = (pidReg[2] >> 4) & 0x0F;

    return CC_OK;
}

int mbedtls_mng_cc_sec_mode_set(CCBool_t isSecAccessMode, CCBool_t isSecModeLock)
{
    uint32_t rc     = CC_OK;
    uint32_t regVal = 0, tempVal;

    /* verify input parameters */
    if((CC_TRUE != isSecAccessMode) && (CC_FALSE != isSecAccessMode)) {
        return CC_MNG_ILLEGAL_INPUT_PARAM_ERR;
    }

    if((CC_TRUE != isSecModeLock) && (CC_FALSE != isSecModeLock)) {
        return CC_MNG_ILLEGAL_INPUT_PARAM_ERR;
    }

    /* Lock mutexes for all hw operation */
    if (CC_PalMutexLock(&CCSymCryptoMutex, CC_INFINITE) != 0) {
        CC_PalAbort("Fail to acquire 'CCSymCryptoMutex'\n");
    }

    if (CC_PalMutexLock(&CCAsymCryptoMutex, CC_INFINITE) != 0) {
        CC_PalAbort("Fail to acquire 'CCAsymCryptoMutex'\n");
    }

    if (CC_PalMutexLock(&CCGenVecMutex, CC_INFINITE) != 0) {
        CC_PalAbort("Fail to acquire 'CCGenVecMutex'\n");
    }

    if (CC_PalMutexLock(&CCApbFilteringRegMutex, CC_INFINITE) != 0) {
        CC_PalAbort("Fail to acquire 'CCApbFilteringRegMutex'\n");
    }

    /* read APB slave accesses control register */
    regVal = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, AO_APB_FILTERING));

    /* check if security mode is already locked */
    if(CC_TRUE == CC_REG_FLD_GET(0, AO_APB_FILTERING, ONLY_SEC_ACCESS_ALLOW_LOCK, regVal)) {
        rc = CC_MNG_APB_SECURE_IS_LOCKED_ERR;
        goto mbedtls_mng_setCCSecMode_END;
    }

    /* set security mode */
    CC_REG_FLD_SET(0, AO_APB_FILTERING, ONLY_SEC_ACCESS_ALLOW, regVal, isSecAccessMode);
    CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, AO_APB_FILTERING), regVal);
    tempVal = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, AO_APB_FILTERING));
    if(tempVal != regVal) {
        rc = CC_MNG_AO_WRITE_FAILED_ERR;
        goto mbedtls_mng_setCCSecMode_END;
    }

    /* set security lock */
    CC_REG_FLD_SET(0, AO_APB_FILTERING, ONLY_SEC_ACCESS_ALLOW_LOCK, regVal, isSecModeLock);
    CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, AO_APB_FILTERING), regVal);
    tempVal = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, AO_APB_FILTERING));
    if(tempVal != regVal) {
        rc = CC_MNG_AO_WRITE_FAILED_ERR;
        goto mbedtls_mng_setCCSecMode_END;
    }

mbedtls_mng_setCCSecMode_END:
    /* Release mutexes */
    if (CC_PalMutexUnlock(&CCApbFilteringRegMutex) != 0) {
        CC_PalAbort("Fail to release 'CCApbFilteringRegMutex'\n");
    }

    if (CC_PalMutexUnlock(&CCGenVecMutex) != 0) {
        CC_PalAbort("Fail to release 'CCGenVecMutex'\n");
    }

    if (CC_PalMutexUnlock(&CCAsymCryptoMutex) != 0) {
        CC_PalAbort("Fail to release 'CCAsymCryptoMutex'\n");
    }

    if (CC_PalMutexUnlock(&CCSymCryptoMutex) != 0) {
        CC_PalAbort("Fail to release 'CCSymCryptoMutex'\n");
    }

    return rc;
}

int mbedtls_mng_cc_priv_mode_set(CCBool_t isPrivAccessMode, CCBool_t isPrivModeLock)
{
    uint32_t rc     = CC_OK;
    uint32_t regVal = 0, tempVal = 0;

    /* verify input parameters */
    if((CC_TRUE != isPrivAccessMode) && (CC_FALSE != isPrivAccessMode)) {
        return CC_MNG_ILLEGAL_INPUT_PARAM_ERR;
    }

    if((CC_TRUE != isPrivModeLock) && (CC_FALSE != isPrivModeLock)) {
        return CC_MNG_ILLEGAL_INPUT_PARAM_ERR;
    }

    /* Lock mutexes for all hw operation */
    if (CC_PalMutexLock(&CCSymCryptoMutex, CC_INFINITE) != 0) {
        CC_PalAbort("Fail to acquire 'CCSymCryptoMutex'\n");
    }

    if (CC_PalMutexLock(&CCAsymCryptoMutex, CC_INFINITE) != 0) {
        CC_PalAbort("Fail to acquire 'CCAsymCryptoMutex'\n");
    }

    if (CC_PalMutexLock(&CCGenVecMutex, CC_INFINITE) != 0) {
        CC_PalAbort("Fail to acquire 'CCGenVecMutex'\n");
    }

    if (CC_PalMutexLock(&CCApbFilteringRegMutex, CC_INFINITE) != 0) {
        CC_PalAbort("Fail to acquire 'CCApbFilteringRegMutex'\n");
    }

    /* read APB slave accesses control register */
    regVal = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, AO_APB_FILTERING));

    /* check if privileged mode is already locked */
    if( CC_REG_FLD_GET(0, AO_APB_FILTERING, ONLY_PRIV_ACCESS_ALLOW_LOCK, regVal) == CC_TRUE) {
        rc = CC_MNG_APB_PRIVILEGE_IS_LOCKED_ERR;
        goto mbedtls_mng_setCCPrivMode_END;
    }

    /* set privileged mode */
    CC_REG_FLD_SET(0, AO_APB_FILTERING, ONLY_PRIV_ACCESS_ALLOW, regVal, isPrivAccessMode);
    CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, AO_APB_FILTERING), regVal);
    tempVal = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, AO_APB_FILTERING));
    if(tempVal != regVal) {
        rc = CC_MNG_AO_WRITE_FAILED_ERR;
        goto mbedtls_mng_setCCPrivMode_END;
    }

    /* set privileged lock */
    CC_REG_FLD_SET(0, AO_APB_FILTERING, ONLY_PRIV_ACCESS_ALLOW_LOCK, regVal, isPrivModeLock);
    CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, AO_APB_FILTERING), regVal);
    tempVal = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, AO_APB_FILTERING));
    if(tempVal != regVal) {
        rc = CC_MNG_AO_WRITE_FAILED_ERR;
        goto mbedtls_mng_setCCPrivMode_END;
    }

mbedtls_mng_setCCPrivMode_END:
    /* Release mutexes */
    if (CC_PalMutexUnlock(&CCApbFilteringRegMutex) != 0) {
        CC_PalAbort("Fail to release 'CCApbFilteringRegMutex'\n");
    }

    if (CC_PalMutexUnlock(&CCGenVecMutex) != 0) {
        CC_PalAbort("Fail to release 'CCGenVecMutex'\n");
    }

    if (CC_PalMutexUnlock(&CCAsymCryptoMutex) != 0) {
        CC_PalAbort("Fail to release 'CCAsymCryptoMutex'\n");
    }

    if (CC_PalMutexUnlock(&CCSymCryptoMutex) != 0) {
        CC_PalAbort("Fail to release 'CCSymCryptoMutex'\n");
    }

    return rc;
}

int mbedtls_mng_debug_key_set(mbedtls_mng_keytype keyType, uint32_t *pHwKey, size_t keySize)
{
    int      rc     = CC_OK;
    uint32_t lcsVal = 0;

    /* Lock mutex for all hw operation */
    if (CC_PalMutexLock(&CCSymCryptoMutex, CC_INFINITE) != 0) {
        CC_PalAbort("Fail to acquire mutex\n");
    }

    /* Read LCS Register */
    rc = mbedtls_mng_lcsGet(&lcsVal);
    if (CC_OK != rc) {
        goto mbedtls_mng_setDebugKey_END;
    }

    /* Check LCS value */
    if ((CC_MNG_LCS_DM != lcsVal) && (CC_MNG_LCS_CM != lcsVal)) {
        rc = CC_MNG_ILLEGAL_OPERATION_ERR;
        goto mbedtls_mng_setDebugKey_END;
    }

    rc = setHwKeyToShadowReg(keyType, pHwKey, keySize);

mbedtls_mng_setDebugKey_END:
    /* Release mutex */
    if (CC_PalMutexUnlock(&CCSymCryptoMutex) != 0) {
        CC_PalAbort("Fail to release mutex\n");
    }

    return rc;
}

int mbedtls_mng_gen_config_get(uint32_t *pOtpWord)
{
    int rc = 0;

    rc = mbedtls_mng_otpWordRead(CC_OTP_ICV_GENERAL_PURPOSE_FLAG_OFFSET, pOtpWord);
    if (CC_OK != rc) {
        return rc;
    }

    return CC_OK;
}

int mbedtls_mng_oem_key_lock(CCBool_t kcpLock, CCBool_t kceLock)
{
    int      rc     = CC_OK;
    uint32_t lcsVal = 0;
    uint32_t regVal = 0, tempVal = 0;

    /* Lock mutex for all hw operation */
    if (CC_PalMutexLock(&CCSymCryptoMutex, CC_INFINITE) != 0) {
        CC_PalAbort("Fail to acquire mutex\n");
    }

    /* Read LCS Register */
    rc = mbedtls_mng_lcsGet(&lcsVal);
    if (CC_OK != rc)
    {
        goto mbedtls_mng_lockOemKey_END;
    }

    /* Check LCS value */
    if ((CC_MNG_LCS_SEC_ENABLED != lcsVal) && (CC_MNG_LCS_RMA != lcsVal))
    {
        rc = CC_MNG_ILLEGAL_OPERATION_ERR;
        goto mbedtls_mng_lockOemKey_END;
    }

    /* Verify input parameters */
    if((CC_TRUE != kcpLock) && (CC_FALSE != kcpLock)) {
        rc = CC_MNG_ILLEGAL_INPUT_PARAM_ERR;
        goto mbedtls_mng_lockOemKey_END;
    }

    if((CC_TRUE != kceLock)  && (CC_FALSE != kceLock)) {
        rc = CC_MNG_ILLEGAL_INPUT_PARAM_ERR;
        goto mbedtls_mng_lockOemKey_END;
    }

    /* read AO Lock Bits register */
    regVal = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, HOST_AO_LOCK_BITS));

    /* check if Kcp is already locked and the user wants to lock it */
    if((CC_TRUE == CC_REG_FLD_GET(0, HOST_AO_LOCK_BITS, HOST_KCP_LOCK, regVal)) &&
       (CC_TRUE == kcpLock)) {
        rc = CC_MNG_KCP_IS_LOCKED_ERR;
        goto mbedtls_mng_lockOemKey_END;
    }

    /* check if Kce is already locked and the user wants to lock it */
    if((CC_TRUE == CC_REG_FLD_GET(0, HOST_AO_LOCK_BITS, HOST_KCE_LOCK, regVal)) &&
       (CC_TRUE == kceLock)) {
        rc = CC_MNG_KCE_IS_LOCKED_ERR;
        goto mbedtls_mng_lockOemKey_END;
    }

    /* Set lock only - do not unlock */
    if (CC_TRUE == kcpLock) {
        /* set Kcp lock */
        CC_REG_FLD_SET(0, HOST_AO_LOCK_BITS, HOST_KCP_LOCK, regVal, kcpLock);
    }

    /* Set lock only - do not unlock */
    if (CC_TRUE == kceLock) {
        /* set Kce lock */
        CC_REG_FLD_SET(0, HOST_AO_LOCK_BITS, HOST_KCE_LOCK, regVal, kceLock);
    }

    /* write APB slave accesses control register */
    CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, HOST_AO_LOCK_BITS), regVal);
    tempVal = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, HOST_AO_LOCK_BITS));
    if(tempVal != regVal) {
        rc = CC_MNG_AO_WRITE_FAILED_ERR;
        goto mbedtls_mng_lockOemKey_END;
    }

mbedtls_mng_lockOemKey_END:
    /* Release mutex */
    if (CC_PalMutexUnlock(&CCSymCryptoMutex) != 0) {
        CC_PalAbort("Fail to release mutex\n");
    }

    return rc;
}

int mbedtls_mng_apbc_config_set(mbedtls_mng_apbcconfig apbcConfig)
{
    uint32_t rc = CC_OK;
    uint32_t regVal = 0, tempVal = 0;

    /* Lock mutex for all hw operation */
    if (CC_PalMutexLock(&CCApbFilteringRegMutex, CC_INFINITE) != 0)
    {
        CC_PalAbort("Fail to acquire 'CCApbFilteringRegMutex'\n");
    }

    /* read APB slave accesses control register */
    regVal = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, AO_APB_FILTERING));

    /* check if APBC security mode is already locked */
    if(CC_TRUE == CC_REG_FLD_GET(0, AO_APB_FILTERING, APBC_ONLY_SEC_ACCESS_ALLOW_LOCK, regVal))
    {
        rc = CC_MNG_APBC_SECURE_IS_LOCKED_ERR;
        goto mbedtls_mng_setApbcConfig_END;
    }

    /* check if APBC privilege mode is already locked */
    if(CC_TRUE == CC_REG_FLD_GET(0, AO_APB_FILTERING, APBC_ONLY_PRIV_ACCESS_ALLOW_LOCK, regVal))
    {
        rc = CC_MNG_APBC_PRIVILEGE_IS_LOCKED_ERR;
        goto mbedtls_mng_setApbcConfig_END;
    }

    /* check if APBC Instruction mode is already locked */
    if(CC_TRUE == CC_REG_FLD_GET(0, AO_APB_FILTERING, APBC_ONLY_INST_ACCESS_ALLOW_LOCK, regVal))
    {
        rc = CC_MNG_APBC_INSTRUCTION_IS_LOCKED_ERR;
        goto mbedtls_mng_setApbcConfig_END;
    }

    /* set APBC security mode */
    CC_REG_FLD_SET(0, AO_APB_FILTERING, APBC_ONLY_SEC_ACCESS_ALLOW, regVal, apbcConfig.apbcbits.isApbcSecAccessMode);
    CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, AO_APB_FILTERING), regVal);
    tempVal = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, AO_APB_FILTERING));
    if(tempVal != regVal) {
        rc = CC_MNG_AO_WRITE_FAILED_ERR;
        goto mbedtls_mng_setApbcConfig_END;
    }

    /* set APBC security lock */
    CC_REG_FLD_SET(0, AO_APB_FILTERING, APBC_ONLY_SEC_ACCESS_ALLOW_LOCK, regVal, apbcConfig.apbcbits.isApbcSecModeLock);
    CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, AO_APB_FILTERING), regVal);
    tempVal = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, AO_APB_FILTERING));
    if(tempVal != regVal) {
        rc = CC_MNG_AO_WRITE_FAILED_ERR;
        goto mbedtls_mng_setApbcConfig_END;
    }

    /* set APBC privilege mode */
    CC_REG_FLD_SET(0, AO_APB_FILTERING, APBC_ONLY_PRIV_ACCESS_ALLOW, regVal, apbcConfig.apbcbits.isApbcPrivAccessMode);
    CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, AO_APB_FILTERING), regVal);
    tempVal = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, AO_APB_FILTERING));
    if(tempVal != regVal) {
        rc = CC_MNG_AO_WRITE_FAILED_ERR;
        goto mbedtls_mng_setApbcConfig_END;
    }

    /* set APBC privilege lock */
    CC_REG_FLD_SET(0, AO_APB_FILTERING, APBC_ONLY_PRIV_ACCESS_ALLOW_LOCK, regVal, apbcConfig.apbcbits.isApbcPrivModeLock);
    CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, AO_APB_FILTERING), regVal);
    tempVal = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, AO_APB_FILTERING));
    if(tempVal != regVal) {
        rc = CC_MNG_AO_WRITE_FAILED_ERR;
        goto mbedtls_mng_setApbcConfig_END;
    }

    /* set APBC Instruction mode */
    CC_REG_FLD_SET(0, AO_APB_FILTERING, APBC_ONLY_INST_ACCESS_ALLOW, regVal, apbcConfig.apbcbits.isApbcInstAccessMode);
    CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, AO_APB_FILTERING), regVal);
    tempVal = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, AO_APB_FILTERING));
    if(tempVal != regVal) {
        rc = CC_MNG_AO_WRITE_FAILED_ERR;
        goto mbedtls_mng_setApbcConfig_END;
    }

    /* set APBC Instruction lock */
    CC_REG_FLD_SET(0, AO_APB_FILTERING, APBC_ONLY_INST_ACCESS_ALLOW_LOCK, regVal, apbcConfig.apbcbits.isApbcInstModeLock);
    CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, AO_APB_FILTERING), regVal);
    tempVal = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, AO_APB_FILTERING));
    if(tempVal != regVal) {
        rc = CC_MNG_AO_WRITE_FAILED_ERR;
        goto mbedtls_mng_setApbcConfig_END;
    }

mbedtls_mng_setApbcConfig_END:
    /* Release mutex */
    if (CC_PalMutexUnlock(&CCApbFilteringRegMutex) != 0)
    {
        CC_PalAbort("Fail to release 'CCApbFilteringRegMutex'\n");
    }

    return rc;
}

int mbedtls_mng_apbc_access(CCBool_t isApbcAccessUsed)
{
    int rc = CC_OK;

    if ((isApbcAccessUsed != CC_TRUE) && (isApbcAccessUsed != CC_FALSE)) {
        return CC_MNG_APBC_ACCESS_FAILED_ERR;
    }

    if (isApbcAccessUsed == CC_TRUE) {
           /* increase CC counter at the beginning of each operation */
        rc = CC_IS_WAKE;
        if (rc != CC_OK) {
            CC_PalAbort("Fail to increase PM counter\n");
        }
    } else {
        /* decrease CC counter at the end of each operation */
        rc = CC_IS_IDLE;
        if (rc != CC_OK) {
            CC_PalAbort("Fail to decrease PM counter\n");
        }
    }

    return rc;
}

int mbedtls_mng_suspend(uint8_t *pBackupBuffer, size_t backupSize)
{
    int rc = CC_OK;
    uint32_t regVal = 0;

    /* check input parameters */
    if ((pBackupBuffer != NULL) &&
             (backupSize <= 0) ){
            return CC_MNG_ILLEGAL_INPUT_PARAM_ERR;
    }

    /* lock mutexes for all HW operation */
    if (CC_PalMutexLock(&CCSymCryptoMutex, CC_INFINITE) != 0) {
        CC_PalAbort("Fail to acquire 'CCSymCryptoMutex'\n");
    }

    if (CC_PalMutexLock(&CCAsymCryptoMutex, CC_INFINITE) != 0) {
        CC_PalAbort("Fail to acquire 'CCAsymCryptoMutex'\n");
    }

    if (CC_PalMutexLock(&CCGenVecMutex, CC_INFINITE) != 0) {
        CC_PalAbort("Fail to acquire 'CCGenVecMutex'\n");
    }

    if (CC_PalMutexLock(&CCApbFilteringRegMutex, CC_INFINITE) != 0) {
        CC_PalAbort("Fail to acquire 'CCApbFilteringRegMutex'\n");
    }

    /* verify that the FW has no pending tasks */
    if(CC_STATUS_GET != 0){
        rc = CC_MNG_PM_SUSPEND_RESUME_FAILED_ERR;
        goto mbedtls_mng_suspend_END;
    }

    /* verify HW is idle */
    regVal = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, HOST_CC_IS_IDLE));
    if(CC_TRUE != CC_REG_FLD_GET(0, HOST_CC_IS_IDLE, HOST_CC_IS_IDLE, regVal)) {
        rc = CC_MNG_PM_SUSPEND_RESUME_FAILED_ERR;
        goto mbedtls_mng_suspend_END;
    }

    /* do backup operations if needed... */
    // TBD!!!!

    /* set HW register to notify about the coming event */
    CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, HOST_POWERDOWN), CC_TRUE);

    return CC_OK;

mbedtls_mng_suspend_END:
    /* release mutexes */
    if (CC_PalMutexUnlock(&CCApbFilteringRegMutex) != 0) {
        CC_PalAbort("Fail to release 'CCApbFilteringRegMutex'\n");
    }

    if (CC_PalMutexUnlock(&CCGenVecMutex) != 0) {
        CC_PalAbort("Fail to release 'CCGenVecMutex'\n");
    }

    if (CC_PalMutexUnlock(&CCAsymCryptoMutex) != 0) {
        CC_PalAbort("Fail to release 'CCAsymCryptoMutex'\n");
    }

    if (CC_PalMutexUnlock(&CCSymCryptoMutex) != 0) {
        CC_PalAbort("Fail to release 'CCSymCryptoMutex'\n");
    }

    return rc;
}

int mbedtls_mng_resume(uint8_t *pBackupBuffer, size_t backupSize)
{
    int rc = CC_OK;

    /* check input parameters */
    if ((pBackupBuffer != NULL) &&
             (backupSize <= 0) ){
            return CC_MNG_ILLEGAL_INPUT_PARAM_ERR;
    }

    /* do restore operations if needed... */
    // TBD!!!!

        /* set DMA endianess */
#ifdef BIG__ENDIAN
        CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, HOST_ENDIAN) , 0xCCUL);
#else
        CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, HOST_ENDIAN) , 0x00UL);
#endif

    /* release mutexes */
    if (CC_PalMutexUnlock(&CCApbFilteringRegMutex) != 0) {
        CC_PalAbort("Fail to release 'CCApbFilteringRegMutex'\n");
    }

    if (CC_PalMutexUnlock(&CCGenVecMutex) != 0) {
        CC_PalAbort("Fail to release 'CCGenVecMutex'\n");
    }

    if (CC_PalMutexUnlock(&CCAsymCryptoMutex) != 0) {
        CC_PalAbort("Fail to release 'CCAsymCryptoMutex'\n");
    }

    if (CC_PalMutexUnlock(&CCSymCryptoMutex) != 0) {
        CC_PalAbort("Fail to release 'CCSymCryptoMutex'\n");
    }

    return rc;
}


