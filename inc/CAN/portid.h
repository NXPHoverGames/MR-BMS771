/****************************************************************************
 * nxp_bms/BMS_v1/inc/CAN/portid.h
 *
 * Copyright 2020-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ** ###################################################################
 **     Filename    : portid.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date   		: 2021-01-19
 **     Abstract    :
 **        portid module.
 **
 ** ###################################################################*/
/*!
 ** @file portid.h
 **
 ** @version 01.00
 **
 ** @brief
 **        portid module. this module implements the CAN port id
 **
 */

#ifndef CYPHAL_REGISTER_INTERFACE_H_
#define CYPHAL_REGISTER_INTERFACE_H_

#include <canard.h>

#define NUNAVUT_ASSERT
#define uavcan_primitive_String_1_0_value_ARRAY_CAPACITY_ 64U
#include "uavcan/node/GetInfo_1_0.h"
#include "uavcan/_register/Value_1_0.h"

// No of pre allocated register entries
#ifndef CYPHAL_REGISTER_COUNT
#    define CYPHAL_REGISTER_COUNT 5
#endif

#define CYPHAL_REGISTER_ERROR_SERIALIZATION 1
#define CYPHAL_REGISTER_ERROR_OUT_OF_MEMORY 2

typedef int32_t (*register_access_set_callback)(uavcan_register_Value_1_0* value);
typedef uavcan_register_Value_1_0 (*register_access_get_callback)(void);

typedef struct
{
    /// uavcan.register.Name.1.0 name
    const char*                  name;
    register_access_set_callback cb_set;
    register_access_get_callback cb_get;
} cyphal_register_interface_entry;


int32_t cyphal_register_interface_init(CanardInstance* ins, uavcan_node_GetInfo_Response_1_0* info);

int32_t cyphal_register_interface_add_entry(
    const char* name, register_access_set_callback cb_set, register_access_get_callback cb_get);

// Handler for all PortID registration related messages
int32_t cyphal_register_interface_process(CanardInstance* ins, CanardTransfer* transfer);

// Handler for node.GetInfo which yields a response
int32_t cyphal_register_interface_get_info_response(CanardInstance* ins, CanardTransfer* request);

// Handler for register access interface
int32_t cyphal_register_interface_access_response(CanardInstance* ins, CanardTransfer* request);

// Handler for register list interface
int32_t cyphal_register_interface_list_response(CanardInstance* ins, CanardTransfer* request);

#endif // CYPHAL_REGISTER_INTERFACE_H_
