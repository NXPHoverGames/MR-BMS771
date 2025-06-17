/****************************************************************************
 * nxp_bms/BMS_v1/inc/display.h
 *
 * Copyright 2021-2022, 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ** ###################################################################
 **     Filename    : display.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date        : 2021-05-19
 **     Abstract    :
 **        display module.
 **        This module contains all functions needed for MCU display
 **
 ** ###################################################################*/
/*!
 ** @file display.h
 **
 ** @version 01.00
 **
 ** @brief
 **        display module. this module contains the functions for MCU display
 **
 */
#ifndef DISPLAY_H
#define DISPLAY_H

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include "data.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

/*******************************************************************************
 * Types
 ******************************************************************************/

/*******************************************************************************
 * public functions
 ******************************************************************************/
/*!
 * @brief   This function is used to initialize the display part
 *
 * @param   skipSelfTest if this is true it will skip the self-test
 *
 * @return  0 if ok, -1 if there is an error
 */
int display_initialize(bool skipSelfTest);

/*!
 * @brief   This function is used to uninitialize the display part
 *
 * @return  0 if ok, -1 if there is an error
 */
int display_uninitialize(void);

/*!
 * @brief   This function is used to turn on or off the power to the display
 *
 * @param   on If true, it will turn on the display, false otherwise.
 *
 * @return  0 if ok, -1 if there is an error
 */
int display_setPower(bool on);

/*!
 * @brief   This function is used to get the state of the display, on or off.
 *
 * @param   none.
 *
 * @return  true if on, false otherwise.
 */
bool display_getPower(void);

/*!
 * @brief   This function is used to update the values of the display with the actual information
 *
 * @param   pCommonBatteryVariables pointer to the commonBatteryVariables_t to update the information
 * @param   pCalcBatteryVariables pointer to the calcBatteryVariables_t to update the information
 *
 * @return  0 If successful, otherwise an error will indicate the error
 */
int display_updateValues(
    commonBatteryVariables_t *pCommonBatteryVariables, calcBatteryVariables_t *pCalcBatteryVariables);

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* DISPLAY_H */
