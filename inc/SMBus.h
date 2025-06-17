/****************************************************************************
 * nxp_bms/BMS_v1/inc/SMBus.h
 *
 * Copyright 2021-2022, 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ** ###################################################################
 **     Filename    : SMBus.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date        : 2021-05-10
 **     Abstract    :
 **        SMBus module.
 **        This module contains all functions needed for using SMBus (smart battery bus)
 **
 ** ###################################################################*/
/*!
 ** @file SMBus.h
 **
 ** @version 01.00
 **
 ** @brief
 **        SMBus module. this module contains the functions to control the SMBus data
 **
 */
#ifndef SMBUS_H_
#define SMBUS_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include "data.h"
/*******************************************************************************
 * defines
 ******************************************************************************/

/*******************************************************************************
 * types
 ******************************************************************************/

/*******************************************************************************
 * public functions
 ******************************************************************************/
/*!
 * @brief   This function will initialze the SMBus (smart battery bus)
 *          it will open the device and start the SMBus task
 *
 * @return  0 if ok, -1 if there is an error
 * @example
 *          if(SMBus_initialize())
 *          {
 *            // do something with the error
 *          }
 */
int SMBus_initialize(void);

/*!
 * @brief   this function will increase the semaphore so the SMBus
 *          task will update the BMS status of the SBS driver (SMBus)
 *
 * @param   resetCurrent If this value is true, the current will be set to 0
 *          This could be used if a low power state is used and the SMBus struct
 *          will not be updated anymore
 * @param   pCommonBatteryVariables pointer to the commonBatteryVariables_t to update the information
 *          May be NULL if resetCurrent is true
 * @param   pCalcBatteryVariables pointer to the calcBatteryVariables_t to update the information
 *          May be NULL if resetCurrent is true
 *
 * @return  If successful, the function will return zero (OK).
 *          Otherwise, an error number will be returned to indicate the error:
 */
int SMBus_updateInformation(bool resetCurrent, commonBatteryVariables_t *pCommonBatteryVariables,
calcBatteryVariables_t *pCalcBatteryVariables);

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* SMBUS_H_ */
