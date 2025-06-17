/****************************************************************************
 * nxp_bms/BMS_v1/inc/CAN/pnp.h
 *
 * Copyright 2020-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ** ###################################################################
 **     Filename    : pnp.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date   		: 2020-10-29
 **     Abstract    :
 **        pnp module.
 **
 ** ###################################################################*/
/*!
 ** @file pnp.h
 **
 ** @version 01.00
 **
 ** @brief
 **        pnp module. this module implements the CAN plug and play protocol
 **
 */

#ifndef CAN_PNP_H_
#define CAN_PNP_H_

#define NUNAVUT_ASSERT
#include <canard.h>


uint32_t initPNPAllocatee(CanardInstance* ins, uint8_t* unique_id);

int32_t PNPAllocRequest(CanardInstance* ins);

int32_t PNPProcess(CanardInstance* ins, CanardTransfer* transfer);

CanardNodeID PNPGetNodeID(void);

CanardPortID PNPGetPortID(CanardInstance* ins);


#endif // CAN_PNP_H_
