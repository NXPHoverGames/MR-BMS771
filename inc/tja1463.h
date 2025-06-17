/****************************************************************************
 * nxp_bms/BMS_v1/inc/tja1463.h
 *
 * Copyright 2024-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
/*!
 ** @file tja1463.h
 **
 ** @version 01.00
 **
 ** @brief
 **        	tja1463 module. this module contains the functions to control the
 ** 		TJA1463 part (CAN transceiver)
 **
 */
#ifndef TJA1463
#define TJA1463

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
 * @brief   This function will initialze the TJA1463
 *          it will test the connection with the chip
 *          And then put it in a lower power mode.
 *
 * @param   skipSelfTest if this is true it will skip the self-test
 *
 * @return  0 if ok, -1 if there is an error
 * @example
 *          if(tja1463_initialize(false))
 *          {
 *            // do something with the error
 *          }
 */
int tja1463_initialize(bool skipSelfTest);


/*!
 * @brief   This function will put the TJA1463 into a low power mode.
 *
 * @param   skipSelfTest if this is true it will skip the self-test
 *
 * @return  0 if ok, -1 if there is an error
 * @example
 *          if(tja1463_put_into_low_power())
 *          {
 *            // do something with the error
 *          }
 */
int tja1463_put_into_low_power(bool skipSelfTest);

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* TJA1463 */
