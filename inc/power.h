/****************************************************************************
 * nxp_bms/BMS_v1/inc/power.h
 *
 * Copyright 2021-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ** ###################################################################
 **     Filename    : power.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date        : 2021-05-19
 **     Abstract    :
 **        power module.
 **        This module contains all functions needed for MCU power
 **
 ** ###################################################################*/
/*!
 ** @file power.h
 **
 ** @version 01.00
 **
 ** @brief
 **        power module. this module contains the functions for MCU power
 **
 */
#ifndef POWER_H
#define POWER_H

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>

/*******************************************************************************
 * Defines
 ******************************************************************************/

//! @brief This define can be used to not assert the PM functions from this application
//#define DISABLE_PM

/*******************************************************************************
 * Types
 ******************************************************************************/
//! @brief this enum could be used to set or get the MCU power mode.
typedef enum
{
    RUN_MODE,     // In this mode the MCU will be in RUN mode at 80MHz with all the peripherals enabled
    STANDBY_MODE, // In this mode the MCU will be in VLPR mode at 2MHz with the SPI and CLI enabled
    VLPR_MODE,    // In this mode the MCU will be in VLPR mode at 2MHz with the CLI enabled
    ERROR_VALUE   // This will be returned if there is an error.
} mcuPowerModes_t;

/*******************************************************************************
 * public functions
 ******************************************************************************/
/*!
 * @brief   This function is used to initialize the power part
 *
 * @return  0 if ok, -1 if there is an error
 */
int power_initialize(void);

/*!
 * @brief   Function that will set or get the MCU power mode
 * @note    Could be called from multiple threads
 *
 * @param   setNotGet if true it is used to set the power mode, false to get it
 * @param   newValue if setNotGet is true, this is the new power mode, could be ERROR_VALUE otherwise
 *
 * @return  the MCU power mode from the mcuPowerModes_t enum, ERROR_VALUE if error
 */
mcuPowerModes_t power_setNGetMcuPowerMode(bool setNotGet, mcuPowerModes_t newValue);

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* POWER_H */
