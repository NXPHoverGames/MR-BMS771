/****************************************************************************
 * nxp_bms/BMS_v1/inc/sbc.h
 *
 * Copyright 2020-2021, 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ** ###################################################################
 **     Filename    : sbc.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date        : 2020-06-24
 **     Abstract    :
 **        sbc module.
 **        This module contains all functions needed for using sbc to configure the power
 **
 ** ###################################################################*/
/*!
 ** @file sbc.h
 **
 ** @version 01.00
 **
 ** @brief
 **        sbc module. this module contains the functions to control the power of the sbc
 **
 */
#ifndef SBC_H_
#define SBC_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include <string.h>

/*******************************************************************************
 * defines
 ******************************************************************************/
#define SLOW_WATCHDOG true
#define FAST_WATCHDOG false

/*******************************************************************************
 * types
 ******************************************************************************/
//! this enum is used to set the mode of the SBC
typedef enum
{
    SBC_SLEEP   = 1,    //!< in this mode the SBC will turn off both the CAN tranceiver and the power supply (to MCU)
    SBC_STANDBY = 2,    //!< in this mode the SBC will turn off the CAN tranceiver, V1 is on
    SBC_NORMAL  = 3     //!< in this mode the SBC will have the power supply and the CAN tranceicer on
}sbc_mode_t;

//! @brief An enumeration to set the watchdog in a specific mode
typedef enum {
    WD_AUTONOMOUS = 0,  //!< Watchdog off in software development mode, if not in that mode WD on in NORMAL and STANDBY(if RXD LOW)
    WD_TIMEOUT = 1,     //!< trigger watchdog before timeout or reset will occur.
    WD_WINDOW = 2       //!< trigger watchdog in second half of the period or reset will occur.
}watchdogMode_t;

/*******************************************************************************
 * public functions
 ******************************************************************************/
/*!
 * @brief   this function is used to initialze the SBC
 *
 * @param   skipSelfTest if this is true it will skip the self-test
 *
 * @return  If successful, the function will return zero (OK). Otherwise -1
 */
int sbc_initialize(bool skipSelfTest);

/*!
 * @brief   This function is used to verify the SBC using SPI
 *          It will check the Device identification register (0x7E)
 *
 * @param   None
 *
 * @return  If successful, the function will return zero (OK). Otherwise -1
 */
int sbc_verifySbc(void);

/*!
 * @brief   this function is used to set the SBC mode
 * @note    Multi-thread protected
 *
 * @param   newMode the new mode from the sbc_mode_t enum
 *
 * @return  If successful, the function will return zero (OK). Otherwise -1
 */
int sbc_setSbcMode(sbc_mode_t newMode);

/*!
 * @brief   this function is used to get the SBC mode
 * @note    Multi-thread protected
 *
 * @param   None
 *
 * @return  If successful, the function will return the new mode from the sbc_mode_t enum.
 *          Otherwise negative (-1 or -2)
 */
int sbc_getSbcMode(void);

/*!
 * @brief   this function is used to set the CAN FD mode on or off
 *
 * @param   on if true, CAN FD will be tollerated, if false it will not
 *
 * @return  If successful, the function will return zero (OK). Otherwise -1
 */
int sbc_setCANFDMode(bool on);

/*!
 * @brief   this function is used to kick the watchdog, which will reset it.
 * @note    Multi-thread protected
 *
 * @param   none
 *
 * @return  If successful, the function will return zero (OK). Otherwise -1
 */
int sbc_kickTheWatchdog(void);

/*!
 * @brief   this function is used to set a new watchdog mode in the SBC.
 * @warning keep in mind that change the watchdog mode means disabling 5V (CAN tranceiver) briefly
 * @note    Multi-thread protected
 *
 * @param   newMode The new watchdog mode from the watchdogModes_t enum.
 * @param   slow True if the watchdog needs to be slow, false otherwise.
 *
 * @return  0 if succeeded, -1 otherwise.
 */
int sbc_setWatchdogMode(watchdogMode_t newMode, bool slow);

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* SBC_H_ */
