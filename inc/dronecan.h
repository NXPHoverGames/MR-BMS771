/****************************************************************************
 * nxp_bms/BMS_v1/inc/dronecan.h
 *
 * Copyright 2020-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ** ###################################################################
 **     Filename    : dronecan.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date        : 2022-02-01
 **     Abstract    :
 **        dronecan module.
 **        This module contains all functions needed for dronecan
 **
 ** ###################################################################*/
/*!
 ** @file dronecan.h
 **
 ** @version 01.00
 **
 ** @brief
 **        dronecan module. this module contains the functions for dronecan
 **
 */
#ifndef DRONECAN_H_
#define DRONECAN_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include "cli.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

#if BOARD_NAME == MR_BMS771_BOARD
#   define DRONECAN_RUNNING_LED_OFF_TIME_MS    500
#   define DRONECAN_RUNNING_LED_ON_TIME_MS     500
#   define DRONECAN_READY_LED_OFF_TIME_MS      500
#   define DRONECAN_READY_LED_ON_TIME_MS       5000
#endif

/*******************************************************************************
 * Types
 ******************************************************************************/

/*******************************************************************************
 * public functions
 ******************************************************************************/
/*!
 * @brief   this function initializes the DRONECAN part
 *
 *          It will create the task to check and update the data
 *
 * @param   none
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 * indicate the error:
 *
 */
int dronecan_initialize(void);

/*!
 * @brief   this function will increase the semaphore so the DRONECAN task will send the BMS status using
 * DRONECAN
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 * indicate the error:
 *
 */
int dronecan_sendBMSStatus(void);

/*!
 * @brief   this function will flush the can TX
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 * indicate the error:
 *
 */
int dronecan_flushtx(void);

/*!
 * @brief   this function will turn ON or OFF the CAN PHY LED sequence
 *
 * @param   LedOn if this is true, it will turn on the LED sequence, otherwise it will stop it.
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 * indicate the error:
 *
 */
int dronecan_setLed(bool LedOn);
/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* DRONECAN_H_ */
