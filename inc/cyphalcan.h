/****************************************************************************
 * nxp_bms/BMS_v1/inc/cyphalcan.h
 *
 * Copyright 2020-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ** ###################################################################
 **     Filename    : cyphalcan.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date        : 2020-05-23
 **     Abstract    :
 **        cyphalcan module.
 **        This module contains all functions needed for cyphal can
 **
 ** ###################################################################*/
/*!
 ** @file cyphalcan.h
 **
 ** @version 01.00
 **
 ** @brief
 **        cyphalcan module. this module contains the functions for cyphal can
 **
 */
#ifndef CYPHALCAN_H_
#define CYPHALCAN_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include "cli.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

#if BOARD_NAME == MR_BMS771_BOARD
#   define CYPHALCAN_RUNNING_LED_OFF_TIME_MS   250
#   define CYPHALCAN_RUNNING_LED_ON_TIME_MS    250
#   define CYPHALCAN_READY_LED_OFF_TIME_MS     250
#   define CYPHALCAN_READY_LED_ON_TIME_MS      2500
#endif

/*******************************************************************************
 * Types
 ******************************************************************************/

/*******************************************************************************
 * public functions
 ******************************************************************************/
/*!
 * @brief   this function initializes the CYPHAL CAN part
 *
 *          It will create the task to check and update the data
 *
 * @param   none
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error:
 *
 */
int cyphalcan_initialize(void);

/*!
 * @brief   this function will increase the semaphore so the CYPHALCAN task will send the BMS status using CAN
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error:
 *
 */
int cyphalcan_sendBMSStatus(void);

/*!
 * @brief   this function will flush the can TX
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error:
 *
 */
int cyphalcan_flushtx(void);

/*!
 * @brief   this function will turn ON or OFF the CAN PHY LED sequence
 *
 * @param   LedOn if this is true, it will turn on the LED sequence, otherwise it will stop it.
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 * indicate the error:
 *
 */
int cyphalcan_setLed(bool LedOn);

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* CYPHALCAN_H_ */
