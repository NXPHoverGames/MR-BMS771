/*
 * Copyright 2016 - 2019, 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*!
 * @file bcc_spi.h
 *
 * This file provides access to the functions for SPI communication of BCC driver
 * in SPI mode.
 */

#ifndef __BCC_SPI_H
#define __BCC_SPI_H

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
 * Battery Cell Controller device. Intended for SPI mode only.
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
bcc_status_t BCC_Reg_ReadSpi(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    uint8_t regAddr, uint8_t regCnt, uint16_t* regVal);

/*!
 * @brief This function writes a value to addressed register of selected
 * Battery Cell Controller device. Intended for SPI mode only.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param regAddr Register address. See BCC header file with register map for
 *                possible values.
 * @param regVal New value of selected register.
 * @param retReg Automatic response of BCC, which contains register addressed
 *               in previous access. You can pass NULL when you do not care
 *               about the response.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Reg_WriteSpi(const bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid, uint8_t regAddr, uint16_t regVal, uint16_t* retReg);

/*!
 * @brief This function uses No Operation command of BCC to verify communication
 * without performing any operation. Intended for SPI mode only.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_VerifyComSpi(const bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid);
/*! @} */

#endif /* __BCC_SPI_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
