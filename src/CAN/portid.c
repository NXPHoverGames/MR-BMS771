/****************************************************************************
 * nxp_bms/BMS_v1/src/CAN/portid.c
 *
 * Copyright 2020-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#include <stdio.h>
#include <string.h>

#include "portid.h"

#include "uavcan/_register/Access_1_0.h"
#include "uavcan/_register/List_1_0.h"
#include "uavcan/_register/Name_1_0.h"
#include "uavcan/_register/Value_1_0.h"
#include "pnp.h"
#include "BMS_data_types.h"
#include "timestamp.h"

/****************************************************************************
 * private data
 ****************************************************************************/

uavcan_node_GetInfo_Response_1_0* node_info;

CanardRxSubscription getinfo_subscription;
CanardRxSubscription register_access_subscription;
CanardRxSubscription register_list_subscription;

// TODO register list and data
cyphal_register_interface_entry register_list[CYPHAL_REGISTER_COUNT];
uint32_t                        register_list_size = 0;

/****************************************************************************
 * public functions
 ****************************************************************************/

int32_t cyphal_register_interface_init(CanardInstance* ins, uavcan_node_GetInfo_Response_1_0* info)
{
    node_info = info; // TODO think about retention, copy isntead?

    (void)canardRxSubscribe(ins, CanardTransferKindRequest, uavcan_node_GetInfo_1_0_FIXED_PORT_ID_,
        uavcan_node_GetInfo_Request_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
        CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC, &getinfo_subscription);

    (void)canardRxSubscribe(ins, CanardTransferKindRequest, uavcan_register_Access_1_0_FIXED_PORT_ID_,
        uavcan_register_Access_Request_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
        CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC, &register_access_subscription);

    (void)canardRxSubscribe(ins, CanardTransferKindRequest, uavcan_register_List_1_0_FIXED_PORT_ID_,
        uavcan_register_List_Request_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
        CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC, &register_list_subscription);

    return 0;
}

int32_t cyphal_register_interface_add_entry(
    const char* name, register_access_set_callback cb_set, register_access_get_callback cb_get)
{
    if(register_list_size < CYPHAL_REGISTER_COUNT)
    {
        register_list[register_list_size].name   = name;
        register_list[register_list_size].cb_set = cb_set;
        register_list[register_list_size].cb_get = cb_get;
        register_list_size++;
        return 0;
    }
    else
    {
        return -CYPHAL_REGISTER_ERROR_OUT_OF_MEMORY; // register list full
    }
}

// Handler for all PortID registration related messages
int32_t cyphal_register_interface_process(CanardInstance* ins, CanardTransfer* transfer)
{
    if(transfer->port_id == uavcan_node_GetInfo_1_0_FIXED_PORT_ID_)
    {
        return cyphal_register_interface_get_info_response(ins, transfer);
    }
    else if(transfer->port_id == uavcan_register_Access_1_0_FIXED_PORT_ID_)
    {
        return cyphal_register_interface_access_response(ins, transfer);
    }
    else if(transfer->port_id == uavcan_register_List_1_0_FIXED_PORT_ID_)
    {
        return cyphal_register_interface_list_response(ins, transfer);
    }

    return 0; // Nothing to do
}

// Handler for node.GetInfo which yields a response
int32_t cyphal_register_interface_get_info_response(CanardInstance* ins, CanardTransfer* request)
{
    uavcan_node_GetInfo_Request_1_0 msg;

    if(uavcan_node_GetInfo_Request_1_0_deserialize_(&msg, request->payload, &request->payload_size) < 0)
    {
        // Error deserialize failed
        return -CYPHAL_REGISTER_ERROR_SERIALIZATION;
    }

    // Note so technically the uavcan_node_GetInfo_Request_1_0 is an empty message not sure if the code above
    // is required

    // Setup node.GetInfo response

    uint8_t response_payload_buffer[uavcan_node_GetInfo_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];

    CanardMicrosecond transmission_deadline = getMonotonicTimestampUSec() + 1000 * 10;

    CanardTransfer response = {
        .timestamp_usec = transmission_deadline, // Zero if transmission deadline is not limited.
        .priority       = CanardPriorityNominal,
        .transfer_kind  = CanardTransferKindResponse,
        .port_id        = uavcan_node_GetInfo_1_0_FIXED_PORT_ID_, // This is the subject-ID.
        .remote_node_id = request->remote_node_id,                // Send back to request Node
        .transfer_id    = request->transfer_id,
        .payload_size   = uavcan_node_GetInfo_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
        .payload        = &response_payload_buffer,
    };

    int32_t result = uavcan_node_GetInfo_Response_1_0_serialize_(
        node_info, (uint8_t* const) & response_payload_buffer, &response.payload_size);

    if(result == 0)
    {
        // set the data ready in the buffer and chop if needed
        result = canardTxPush(ins, &response);
    }

    if(result < 0)
    {
        // An error has occurred: either an argument is invalid or we've ran out of memory.
        // It is possible to statically prove that an out-of-memory will never occur for a given application
        // if the heap is sized correctly; for background, refer to the Robson's Proof and the documentation
        // for O1Heap.
        return -CYPHAL_REGISTER_ERROR_SERIALIZATION;
    }
    return 1;
}

// Handler for register access interface
int32_t cyphal_register_interface_access_response(CanardInstance* ins, CanardTransfer* request)
{

    int index;
    {
        uavcan_register_Access_Request_1_0 msg;

        if(uavcan_register_Access_Request_1_0_deserialize_(&msg, request->payload, &request->payload_size) <
            0)
        {
            // Error deserialize failed
            return -CYPHAL_REGISTER_ERROR_SERIALIZATION;
        }

        {
            char register_string[uavcan_primitive_String_1_0_value_ARRAY_CAPACITY_];
            int  reg_str_len;
            for(index = 0; index < register_list_size; index++)
            {
                reg_str_len = snprintf(register_string, sizeof(register_string), "uavcan.pub.udral.%s.0.id",
                    register_list[index].name); // TODO more option then pub (sub rate etc)
                if(strncmp((const char*)msg.name.name.elements, register_string, reg_str_len) == 0)
                {
                    if(msg.value._tag_ != 0)
                    { // Value has been set thus we call set handler
                        if(register_list[index].cb_set(&msg.value) != 0)
                        {
                            // TODO error ocurred check doc for correct response
                        }
                    }
                    break; // We're done exit loop
                }
            }
        }
    }

    {
        uavcan_register_Access_Response_1_0 response_msg;
        uavcan_register_Access_Response_1_0_initialize_(&response_msg);

        if(index < register_list_size)
        { // Index is available
            response_msg.value = register_list[index].cb_get();
        }
        else
        {
            uavcan_register_Value_1_0_initialize_(&response_msg.value);
            uavcan_register_Value_1_0_select_empty_(&response_msg.value);
        }

        uint8_t response_payload_buffer[uavcan_register_Access_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];

        CanardMicrosecond transmission_deadline = getMonotonicTimestampUSec() + 1000 * 10;

        CanardTransfer response = {
            .timestamp_usec = transmission_deadline, // Zero if transmission deadline is not limited.
            .priority       = CanardPriorityNominal,
            .transfer_kind  = CanardTransferKindResponse,
            .port_id        = uavcan_register_Access_1_0_FIXED_PORT_ID_, // This is the subject-ID.
            .remote_node_id = request->remote_node_id,                   // Send back to request Node
            .transfer_id    = request->transfer_id,
            .payload_size   = uavcan_register_Access_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
            .payload        = &response_payload_buffer,
        };

        int32_t result = uavcan_register_Access_Response_1_0_serialize_(
            &response_msg, (uint8_t* const) & response_payload_buffer, &response.payload_size);

        if(result == 0)
        {
            // set the data ready in the buffer and chop if needed
            result = canardTxPush(ins, &response);
        }

        if(result < 0)
        {
            // An error has occurred: either an argument is invalid or we've ran out of memory.
            // It is possible to statically prove that an out-of-memory will never occur for a given
            // application if the heap is sized correctly; for background, refer to the Robson's Proof and the
            // documentation for O1Heap.
            return -CYPHAL_REGISTER_ERROR_SERIALIZATION;
        }

        return 1;
    }
}

// Handler for register list interface
int32_t cyphal_register_interface_list_response(CanardInstance* ins, CanardTransfer* request)
{
    uavcan_register_List_Request_1_0 msg;

    if(uavcan_register_List_Request_1_0_deserialize_(&msg, request->payload, &request->payload_size) < 0)
    {
        // Error deserialize failed
        return -CYPHAL_REGISTER_ERROR_SERIALIZATION;
    }

    // Setup register response

    uint8_t response_payload_buffer
        [uavcan_register_List_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_]; // TODO we know already how big
                                                                              // our response is, don't
                                                                              // overallocate.

    CanardMicrosecond transmission_deadline = getMonotonicTimestampUSec() + 1000 * 10;

    uavcan_register_List_Response_1_0 response_msg;

    // Reponse magic start

    if(msg.index < register_list_size)
    {
        response_msg.name.name.count = snprintf((char*)response_msg.name.name.elements,
            uavcan_primitive_String_1_0_value_ARRAY_CAPACITY_, "uavcan.pub.udral.%s.0.id",
            register_list[msg.index].name);
    }
    // TODO more option then pub (sub rate

    // Response magic end

    CanardTransfer response = {
        .timestamp_usec = transmission_deadline, // Zero if transmission deadline is not limited.
        .priority       = CanardPriorityNominal,
        .transfer_kind  = CanardTransferKindResponse,
        .port_id        = uavcan_register_List_1_0_FIXED_PORT_ID_, // This is the subject-ID.
        .remote_node_id = request->remote_node_id,                 // Send back to request Node
        .transfer_id    = request->transfer_id,
        .payload_size = uavcan_register_List_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_, // See prev TODO
        .payload = &response_payload_buffer,
    };

    int32_t result = uavcan_register_List_Response_1_0_serialize_(
        &response_msg, (uint8_t* const) & response_payload_buffer, &response.payload_size);

    if(result == 0)
    {
        // set the data ready in the buffer and chop if needed
        result = canardTxPush(ins, &response);
    }

    if(result < 0)
    {
        // An error has occurred: either an argument is invalid or we've ran out of memory.
        // It is possible to statically prove that an out-of-memory will never occur for a given application
        // if the heap is sized correctly; for background, refer to the Robson's Proof and the documentation
        // for O1Heap.
        return -CYPHAL_REGISTER_ERROR_SERIALIZATION;
    }
    return 1;
}
