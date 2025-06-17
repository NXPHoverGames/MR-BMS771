/*
 * Copyright 2016 - 2019, 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*!
 * @file bcc_tpl.h
 *
 * This file provides access to the functions for SPI communication of BCC
 * driver in TPL mode.
 */

#ifndef __BCC_TPL_H
#define __BCC_TPL_H

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "bcc_communication.h"

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @addtogroup function_group
 * @{
 */

/*!
 * @brief This function reads a value from addressed register of selected
 * Battery Cell Controller device. Intended for TPL mode only.
 *
 * In case of simultaneous read of more registers, address is incremented
 * in ascending manner.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param regAddr Register address. See BCC header file with register map for
 *                possible values.
 * @param regCnt Number of registers to read.
 * @param regVal Pointer to memory where content of selected 16 bit registers
 *               is stored.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Reg_ReadTpl(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    uint8_t regAddr, uint8_t regCnt, uint16_t* regVal);

/*!
 * @brief This function writes a value to addressed register of selected
 * Battery Cell Controller device. Intended for TPL mode only.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param regAddr Register address. See BCC header file with register map for
 *                possible values.
 * @param regVal New value of selected register.
 * @param retReg Automatic response of BCC, which contains updated register.
 *               You can pass NULL when you do not care about the response.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Reg_WriteTpl(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    uint8_t regAddr, uint16_t regVal, uint16_t* retReg);

/*!
 * @brief This function writes a value to addressed register of all configured
 * BCC devices. Intended for TPL mode only.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param regAddr Register address. See BCC header file with register map for
 *                possible values.
 * @param regVal New value of selected register.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Reg_WriteGlobalTpl(bcc_drv_config_t* const drvConfig,
    uint8_t regAddr, uint16_t regVal);

/*!
 * @brief This function uses No Operation command of BCC to verify communication
 * without performing any operation. Intended for TPL mode only.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_VerifyComTpl(bcc_drv_config_t* const drvConfig, bcc_cid_t cid);
/*! @} */

#endif /* __BCC_TPL_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
