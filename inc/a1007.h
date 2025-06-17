/****************************************************************************
 * nxp_bms/BMS_v1/inc/a1007.h
 *
 * Copyright 2020-2021, 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 ** ###################################################################
 **     Filename    : a1007.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date        : 2020-08-25
 **     Abstract    :
 **        a1007 module.
 **        This module contains all functions needed for using a1007
 **
 ** ###################################################################*/
/*!
 ** @file a1007.h
 **
 ** @version 01.00
 **
 ** @brief
 **        a1007 module. this module contains the functions for the secure element
 **
 */
#ifndef A1007_H_
#define A1007_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>

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
 * @brief   This function will initialze the a1007
 *          it will test the i2C connection with the chip
 *
 * @param   skipSelfTest if this is true it will skip the self-test
 *
 * @return  0 if ok, -1 if there is an error
 * @example
 *          if(a1007_initialize(false))
 *          {
 *            // do something with the error
 *          }
 */
int a1007_initialize(bool skipSelfTest);

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* A1007_H_ */
