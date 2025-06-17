/*
 * Copyright 2016-2022, 2024-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*!
 * File: bcc_peripheries.h
 *
 * This file implements functions for LPSPI and GPIO operations required by BCC
 * driver. Adapted from BCC SW example code version 1.1.
 */

#ifndef BCC_PERIPHERIES_H_
#define BCC_PERIPHERIES_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "bcc.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @brief This function performs one 40b transfer via SPI bus. Intended for SPI
 * mode only. This function needs to be implemented for specified MCU by the
 * user.
 *
 * The byte order of buffers is given by BCC_MSG_BIGEND macro (in bcc.h).
 *
 * @param drvInstance Instance of BCC driver.
 * @param transBuf Pointer to 40b data buffer to be sent.
 * @param recvBuf Pointer to 40b data buffer for received data.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_MCU_TransferSpi(uint8_t drvInstance, uint8_t transBuf[],
    uint8_t recvBuf[]);

/*!
 * @brief This function will wake the BCC
 *
 * @return  int If successful, the pthread_mutex_init() function shall return zero;
 *          otherwise, an error number shall be returned to indicate the error.
 */
int BCC_wakeUpBCC(bcc_drv_config_t* const drvConfig, bcc_cid_t cid);

/*!
 *      MODIFIED. NOT USED BUT NEEDED TO COMPILE BCC SW LIBRARY.
 */
bcc_status_t BCC_MCU_TransferTpl(uint8_t drvInstance, uint8_t transBuf[],
    uint8_t recvBuf[], uint16_t recvTrCnt);

/*!
 * @brief User implementation of assert.
 *
 * @param x - True if everything is OK.
 */
void BCC_MCU_Assert(bool x);

/*!
 * @brief Writes logic 0 or 1 to the CSB pin (or CSB_TX in case of TPL mode).
 * This function needs to be implemented by the user.
 *
 * @param drvInstance Instance of BCC driver.
 * @param value - Zero or one to be set to CSB (CSB_TX) pin.
 */
void BCC_MCU_WriteCsbPin(uint8_t drvInstance, uint8_t value);

/*!
 * @brief Writes logic 0 or 1 to the RST pin.
 *
 * @param drvInstance Instance of BCC driver.
 * @param value - Zero or one to be set to RST pin.
 */
void BCC_MCU_WriteRstPin(uint8_t drvInstance, uint8_t value);

/*!
 *      MODIFIED. NOT USED BUT NEEDED TO COMPILE BCC SW LIBRARY.
 */
void BCC_MCU_WriteEnPin(uint8_t drvInstance, uint8_t value);

/*!
 *      MODIFIED. NOT USED BUT NEEDED TO COMPILE BCC SW LIBRARY.
 */
uint32_t BCC_MCU_ReadIntbPin(uint8_t drvInstance);

#endif /* BCC_PERIPHERIES_H_ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
