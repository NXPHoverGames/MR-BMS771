/****************************************************************************
 * nxp_bms/BMS_v1/inc/CAN/timestamp.h
 *
 * Copyright 2022-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ** ###################################################################
 **     Filename    : time.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date   		: 2022-04-22
 **     Abstract    :
 **        CAN timestamp module.
 **
 ** ###################################################################*/
/*!
 ** @file time.h
 **
 ** @version 01.00
 **
 ** @brief
 **        timestamp module
 **
 */

#ifndef CAN_TIME_H_
#define CAN_TIME_H_

#include <sys/types.h>

/****************************************************************************
 * Name: getMonotonicTimestampUSec
 *
 * Description:
 *
 ****************************************************************************/
uint64_t getMonotonicTimestampUSec(void);

#endif // CAN_TIME_H_
