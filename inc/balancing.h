/****************************************************************************
 * nxp_bms/BMS_v1/inc/balancing.h
 *
 * Copyright 2022, 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 ** ###################################################################
 **     Filename    : balancing.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date        : 2022-02-18
 **     Abstract    :
 **        balancing module.
 **        This module contains all functions needed for balancing
 **
 ** ###################################################################*/
/*!
 ** @file balancing.h
 **
 ** @version 01.00
 **
 ** @brief
 **        balancing module. this module contains the functions to control the balancing part
 **
 */
#ifndef BALANCING_H_
#define BALANCING_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include "data.h"
#include "bcc.h"

/*******************************************************************************
 * defines
 ******************************************************************************/

/*******************************************************************************
 * types
 ******************************************************************************/
/*!
 *  @brief  Enumeration to control and check the balancing state.
 */
typedef enum
{
    BALANCE_OFF,
    BALANCE_TO_LOWEST_CELL,
    BALANCE_TO_STORAGE,
    BALANCE_ERROR
}balanceState_t;

/*******************************************************************************
 * public functions
 ******************************************************************************/
/*!
 * @brief   This function will initialze the balancing part
 *
 * @param   drvConfig the address the BCC driver configuration
 *
 * @return  0 if ok, -1 if there is an error
 * @example
 *          if(balancing_initialize())
 *          {
 *            // do something with the error
 *          }
 */
int balancing_initialize(bcc_drv_config_t* const drvConfig);

/*!
 * @brief   this function will set the new balance state.
 * @note    It can be used to re-start the balancing sequence as well.
 *
 * @param   newBalanceState The new balance state from the balanceState_t enum.
 *
 * @return  If successful, the function will return zero (OK).
 *          Otherwise, an error number will be returned to indicate the error:
 */
int balancing_setBalanceState(balanceState_t newBalanceState);

/*!
 * @brief   this function will check the balance state.
 *
 * @param   none
 *
 * @return  The current balancing state from the balanceState_t enum.
 *          If an error occurs, it will return BALANCE_ERROR
 */
balanceState_t balancing_getBalanceState(void);

/*!
 * @brief   this function will initiate the balancing and control/check it.
 * @note    Should be called cyclically after a new measurement.
 *
 * @param   pCommonBatteryVariables pointer to the commonBatteryVariables_t
 *          for the battery information.
 * @param   lowestCellVoltage The lowest cell voltage of all cells in V.
 *
 * @return  If successful, the function will return zero (OK).
 *          Otherwise, an error number will be returned to indicate the error:
 */
int balancing_handleCellBalancing(commonBatteryVariables_t *pCommonBatteryVariables,
    float lowestCellVoltage);

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* BALANCING_H_ */
