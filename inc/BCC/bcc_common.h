/*
 * Copyright 2016 - 2021, 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * ###################################################################
 **     Filename    : bcc_common.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K118
 **     Version     : 1.00
 **     Compiler    : GNU C Compiler
 **     Date/Time   : 2020-04-01, 11:00, # CodeGen: 0
 **     Abstract    :
 **         Battery Cell Controller (BCC) module - common functions.
 **         This module contains functions linked to BCC6 chip.
 **
 ** ###################################################################*/
/*!
 ** @file bcc_common.h
 **
 ** @version 01.00
 **
 ** @brief
 **         Battery Cell Controller (BCC) module - common functions.
 **         This module contains functions linked to BCC6 chip. \n
 ** @note
 **         This module was adapted from BCC SW examples by A. Meyer.
 */

#ifndef BCC_COMMON_H_
#define BCC_COMMON_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/

/* Global */
#include "bcc_mc3377x.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Battery type. */
#define BCC_DEMO_BATTERY_TYPE   BCC_BATT_T

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* BCC_COMMON_H_ */
