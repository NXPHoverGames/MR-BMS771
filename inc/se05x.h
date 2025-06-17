/****************************************************************************
 * nxp_bms/BMS_v1/inc/se05x.h
 *
 * Copyright 2024-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
/*!
 ** @file se05x.h
 **
 ** @version 01.00
 **
 ** @brief
 **        	se05x module. this module contains the functions to control the
 ** 		SE050 or SE051 part (secure element)
 **
 */
#ifndef SE05X
#define SE05X

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
 * @brief   This function will initialze the SE050 or SE051
 *          it will test the i2C connection with the chip
 *
 * @param   skipSelfTest if this is true it will skip the self-test
 *
 * @return  0 if ok, -1 if there is an error
 * @example
 *          if(se05x_initialize(false))
 *          {
 *            // do something with the error
 *          }
 */
int se05x_initialize(bool skipSelfTest);

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* SE05X */
