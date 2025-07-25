/*
 * Copyright 2020-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
// This is an AUTO-GENERATED UAVCAN DSDL data type implementation. Curious? See https://uavcan.org.
// You shouldn't attempt to edit this file.
//
// Checking this file under version control is not recommended unless it is used as part of a high-SIL
// safety-critical codebase. The typical usage scenario is to generate it as part of the build process.
//
// To avoid conflicts with definitions given in the source DSDL file, all entities created by the code generator
// are named with an underscore at the end, like foo_bar_().
//
// Generator:     nunavut-1.1.0 (serialization was enabled)
// Source file:   /home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/reg/drone/service/battery/Status.0.2.uavcan
// Generated at:  2021-04-12 07:49:15.890438 UTC
// Is deprecated: no
// Fixed port-ID: None
// Full name:     reg.drone.service.battery.Status
// Version:       0.2
//
// Language Options
//     target_endianness:  any
//     omit_float_serialization_support:  False
//     enable_serialization_asserts:  False
//     enable_override_variable_array_capacity:  True

#ifndef REG_DRONE_SERVICE_BATTERY_STATUS_0_2_INCLUDED_
#define REG_DRONE_SERVICE_BATTERY_STATUS_0_2_INCLUDED_

#include <nunavut/support/serialization.h>
#include <reg/drone/service/battery/Error_0_1.h>
#include <reg/drone/service/common/Heartbeat_0_1.h>
#include <uavcan/si/unit/electric_charge/Scalar_1_0.h>
#include <uavcan/si/unit/temperature/Scalar_1_0.h>
#include <stdint.h>
#include <stdlib.h>

static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_TARGET_ENDIANNESS == 1693710260,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/reg/drone/service/battery/Status.0.2.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_OMIT_FLOAT_SERIALIZATION_SUPPORT == 0,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/reg/drone/service/battery/Status.0.2.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_SERIALIZATION_ASSERTS == 0,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/reg/drone/service/battery/Status.0.2.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_OVERRIDE_VARIABLE_ARRAY_CAPACITY == 1,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/reg/drone/service/battery/Status.0.2.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );

#ifdef __cplusplus
extern "C" {
#endif

/// This type does not have a fixed port-ID. See https://forum.uavcan.org/t/choosing-message-and-service-ids/889
#define reg_drone_service_battery_Status_0_2_HAS_FIXED_PORT_ID_ false

#define reg_drone_service_battery_Status_0_2_FULL_NAME_             "reg.drone.service.battery.Status"
#define reg_drone_service_battery_Status_0_2_FULL_NAME_AND_VERSION_ "reg.drone.service.battery.Status.0.2"

/// Extent is the minimum amount of memory required to hold any serialized representation of any compatible
/// version of the data type; or, on other words, it is the the maximum possible size of received objects of this type.
/// The size is specified in bytes (rather than bits) because by definition, extent is an integer number of bytes long.
/// When allocating a deserialization (RX) buffer for this data type, it should be at least extent bytes large.
/// When allocating a serialization (TX) buffer, it is safe to use the size of the largest serialized representation
/// instead of the extent because it provides a tighter bound of the object size; it is safe because the concrete type
/// is always known during serialization (unlike deserialization). If not sure, use extent everywhere.
#define reg_drone_service_battery_Status_0_2_EXTENT_BYTES_                    600UL
#define reg_drone_service_battery_Status_0_2_SERIALIZATION_BUFFER_SIZE_BYTES_ 534UL
static_assert(reg_drone_service_battery_Status_0_2_EXTENT_BYTES_ >= reg_drone_service_battery_Status_0_2_SERIALIZATION_BUFFER_SIZE_BYTES_,
              "Internal constraint violation");

/// saturated uint8 MAX_CELLS = 255
#define reg_drone_service_battery_Status_0_2_MAX_CELLS (255U)

/// Array metadata for: uavcan.si.unit.temperature.Scalar.1.0[2] temperature_min_max
#define reg_drone_service_battery_Status_0_2_temperature_min_max_ARRAY_CAPACITY_           2U
#define reg_drone_service_battery_Status_0_2_temperature_min_max_ARRAY_IS_VARIABLE_LENGTH_ false
/// Array metadata for: saturated float16[<=255] cell_voltages
#ifndef reg_drone_service_battery_Status_0_2_cell_voltages_ARRAY_CAPACITY_
#define reg_drone_service_battery_Status_0_2_cell_voltages_ARRAY_CAPACITY_           255U
#elif !defined(reg_drone_service_battery_Status_0_2_DISABLE_SERIALIZATION_BUFFER_CHECK_)
#  define reg_drone_service_battery_Status_0_2_DISABLE_SERIALIZATION_BUFFER_CHECK_
#endif
#if reg_drone_service_battery_Status_0_2_cell_voltages_ARRAY_CAPACITY_ > 255U
#  error reg_drone_service_battery_Status_0_2_cell_voltages_ARRAY_CAPACITY_ > 255U
#endif
#define reg_drone_service_battery_Status_0_2_cell_voltages_ARRAY_IS_VARIABLE_LENGTH_ true

typedef struct
{
    /// reg.drone.service.common.Heartbeat.0.1 heartbeat
    reg_drone_service_common_Heartbeat_0_1 heartbeat;

    /// uavcan.si.unit.temperature.Scalar.1.0[2] temperature_min_max
    uavcan_si_unit_temperature_Scalar_1_0 temperature_min_max[2];

    /// uavcan.si.unit.electric_charge.Scalar.1.0 available_charge
    uavcan_si_unit_electric_charge_Scalar_1_0 available_charge;

    /// reg.drone.service.battery.Error.0.1 error
    reg_drone_service_battery_Error_0_1 _error;

    /// saturated float16[<=255] cell_voltages
    struct  /// Array address equivalence guarantee: &elements[0] == &cell_voltages
    {
        float elements[reg_drone_service_battery_Status_0_2_cell_voltages_ARRAY_CAPACITY_];
        size_t count;
    } cell_voltages;
} reg_drone_service_battery_Status_0_2;

/// Serialize an instance into the provided buffer.
/// The lifetime of the resulting serialized representation is independent of the original instance.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original object where possible.
///
/// @param obj      The object to serialize.
///
/// @param buffer   The destination buffer. There are no alignment requirements.
///                 @see reg_drone_service_battery_Status_0_2_SERIALIZATION_BUFFER_SIZE_BYTES_
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the buffer in bytes.
///                                 Upon return this value will be updated with the size of the constructed serialized
///                                 representation (in bytes); this value is then to be passed over to the transport
///                                 layer. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t reg_drone_service_battery_Status_0_2_serialize_(
    const reg_drone_service_battery_Status_0_2* const obj, uint8_t* const buffer,  size_t* const inout_buffer_size_bytes)
{
    if ((obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
#ifndef reg_drone_service_battery_Status_0_2_DISABLE_SERIALIZATION_BUFFER_CHECK_

    if ((8U * (size_t) capacity_bytes) < 4272UL)
    {
        return -NUNAVUT_ERROR_SERIALIZATION_BUFFER_TOO_SMALL;
    }
#endif

    // Notice that fields that are not an integer number of bytes long may overrun the space allocated for them
    // in the serialization buffer up to the next byte boundary. This is by design and is guaranteed to be safe.
    size_t offset_bits = 0U;





    {   // reg.drone.service.common.Heartbeat.0.1 heartbeat
        size_t _size_bytes0_ = 2UL;  // Nested object (max) size, in bytes.
        int8_t _err0_ = reg_drone_service_common_Heartbeat_0_1_serialize_(
            &obj->heartbeat, &buffer[offset_bits / 8U], &_size_bytes0_);
        if (_err0_ < 0)
        {
            return _err0_;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        offset_bits += _size_bytes0_ * 8U;  // Advance by the size of the nested object.
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad0_ = (uint8_t)(8U - offset_bits % 8U);
        const int8_t _err1_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad0_);  // Optimize?
        if (_err1_ < 0)
        {
            return _err1_;
        }
        offset_bits += _pad0_;
    }

    {   // uavcan.si.unit.temperature.Scalar.1.0[2] temperature_min_max
        const size_t _origin0_ = offset_bits;
        {   // Array element #0
            size_t _size_bytes1_ = 4UL;  // Nested object (max) size, in bytes.
            int8_t _err2_ = uavcan_si_unit_temperature_Scalar_1_0_serialize_(
                &obj->temperature_min_max[0], &buffer[offset_bits / 8U], &_size_bytes1_);
            if (_err2_ < 0)
            {
                return _err2_;
            }
            // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
            offset_bits += _size_bytes1_ * 8U;  // Advance by the size of the nested object.
        }
        {   // Array element #1
            size_t _size_bytes2_ = 4UL;  // Nested object (max) size, in bytes.
            int8_t _err3_ = uavcan_si_unit_temperature_Scalar_1_0_serialize_(
                &obj->temperature_min_max[1], &buffer[offset_bits / 8U], &_size_bytes2_);
            if (_err3_ < 0)
            {
                return _err3_;
            }
            // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
            offset_bits += _size_bytes2_ * 8U;  // Advance by the size of the nested object.
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        (void) _origin0_;
    }




    {   // void64
        (void) memset(&buffer[offset_bits / 8U], 0, 8);
        offset_bits += 64UL;
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad1_ = (uint8_t)(8U - offset_bits % 8U);
        const int8_t _err4_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad1_);  // Optimize?
        if (_err4_ < 0)
        {
            return _err4_;
        }
        offset_bits += _pad1_;
    }

    {   // uavcan.si.unit.electric_charge.Scalar.1.0 available_charge
        size_t _size_bytes3_ = 4UL;  // Nested object (max) size, in bytes.
        int8_t _err5_ = uavcan_si_unit_electric_charge_Scalar_1_0_serialize_(
            &obj->available_charge, &buffer[offset_bits / 8U], &_size_bytes3_);
        if (_err5_ < 0)
        {
            return _err5_;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        offset_bits += _size_bytes3_ * 8U;  // Advance by the size of the nested object.
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad2_ = (uint8_t)(8U - offset_bits % 8U);
        const int8_t _err6_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad2_);  // Optimize?
        if (_err6_ < 0)
        {
            return _err6_;
        }
        offset_bits += _pad2_;
    }

    {   // reg.drone.service.battery.Error.0.1 error
        size_t _size_bytes4_ = 1UL;  // Nested object (max) size, in bytes.
        int8_t _err7_ = reg_drone_service_battery_Error_0_1_serialize_(
            &obj->_error, &buffer[offset_bits / 8U], &_size_bytes4_);
        if (_err7_ < 0)
        {
            return _err7_;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        offset_bits += _size_bytes4_ * 8U;  // Advance by the size of the nested object.
    }




    {   // saturated float16[<=255] cell_voltages
        if (obj->cell_voltages.count > 255)
        {
            return -NUNAVUT_ERROR_REPRESENTATION_BAD_ARRAY_LENGTH;
        }
        // Array length prefix: truncated uint8
        buffer[offset_bits / 8U] = (uint8_t)(obj->cell_voltages.count);  // C std, 6.3.1.3 Signed and unsigned integers
        offset_bits += 8U;
        for (size_t _index0_ = 0U; _index0_ < obj->cell_voltages.count; ++_index0_)
        {
            float _sat0_ = obj->cell_voltages.elements[_index0_];
            if (isfinite(_sat0_))
            {
                if (_sat0_ < ((float) -65504.0))
                {
                    _sat0_ = ((float) -65504.0);
                }
                if (_sat0_ > ((float) 65504.0))
                {
                    _sat0_ = ((float) 65504.0);
                }
            }
            const int8_t _err8_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat0_);
            if (_err8_ < 0)
            {
                return _err8_;
            }
            offset_bits += 16U;
        }
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad3_ = (uint8_t)(8U - offset_bits % 8U);
        const int8_t _err9_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad3_);  // Optimize?
        if (_err9_ < 0)
        {
            return _err9_;
        }
        offset_bits += _pad3_;
    }
    // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.





    *inout_buffer_size_bytes = (size_t) (offset_bits / 8U);

    return NUNAVUT_SUCCESS;
}

/// Deserialize an instance from the provided buffer.
/// The lifetime of the resulting object is independent of the original buffer.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original buffer where possible.
///
/// @param obj      The object to update from the provided serialized representation.
///
/// @param buffer   The source buffer containing the serialized representation. There are no alignment requirements.
///                 If the buffer is shorter or longer than expected, it will be implicitly zero-extended or truncated,
///                 respectively; see Specification for "implicit zero extension" and "implicit truncation" rules.
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the supplied serialized
///                                 representation, in bytes. Upon return this value will be updated with the
///                                 size of the consumed fragment of the serialized representation (in bytes),
///                                 which may be smaller due to the implicit truncation rule, but it is guaranteed
///                                 to never exceed the original buffer size even if the implicit zero extension rule
///                                 was activated. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t reg_drone_service_battery_Status_0_2_deserialize_(
    reg_drone_service_battery_Status_0_2* const out_obj, const uint8_t* const buffer, size_t* const inout_buffer_size_bytes)
{
    if ((out_obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
    const size_t capacity_bits = capacity_bytes * (size_t) 8U;
    size_t offset_bits = 0U;





    // reg.drone.service.common.Heartbeat.0.1 heartbeat
    {
        size_t _size_bytes5_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
        const int8_t _err10_ = reg_drone_service_common_Heartbeat_0_1_deserialize_(
            &out_obj->heartbeat, &buffer[offset_bits / 8U], &_size_bytes5_);
        if (_err10_ < 0)
        {
            return _err10_;
        }
        offset_bits += _size_bytes5_ * 8U;  // Advance by the size of the nested serialized representation.
    }


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    // uavcan.si.unit.temperature.Scalar.1.0[2] temperature_min_max
    // Array element #0
    {
        size_t _size_bytes6_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
        const int8_t _err11_ = uavcan_si_unit_temperature_Scalar_1_0_deserialize_(
            &out_obj->temperature_min_max[0], &buffer[offset_bits / 8U], &_size_bytes6_);
        if (_err11_ < 0)
        {
            return _err11_;
        }
        offset_bits += _size_bytes6_ * 8U;  // Advance by the size of the nested serialized representation.
    }
    // Array element #1
    {
        size_t _size_bytes7_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
        const int8_t _err12_ = uavcan_si_unit_temperature_Scalar_1_0_deserialize_(
            &out_obj->temperature_min_max[1], &buffer[offset_bits / 8U], &_size_bytes7_);
        if (_err12_ < 0)
        {
            return _err12_;
        }
        offset_bits += _size_bytes7_ * 8U;  // Advance by the size of the nested serialized representation.
    }




    // void64
    offset_bits += 64;


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    // uavcan.si.unit.electric_charge.Scalar.1.0 available_charge
    {
        size_t _size_bytes8_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
        const int8_t _err13_ = uavcan_si_unit_electric_charge_Scalar_1_0_deserialize_(
            &out_obj->available_charge, &buffer[offset_bits / 8U], &_size_bytes8_);
        if (_err13_ < 0)
        {
            return _err13_;
        }
        offset_bits += _size_bytes8_ * 8U;  // Advance by the size of the nested serialized representation.
    }


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    // reg.drone.service.battery.Error.0.1 error
    {
        size_t _size_bytes9_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
        const int8_t _err14_ = reg_drone_service_battery_Error_0_1_deserialize_(
            &out_obj->_error, &buffer[offset_bits / 8U], &_size_bytes9_);
        if (_err14_ < 0)
        {
            return _err14_;
        }
        offset_bits += _size_bytes9_ * 8U;  // Advance by the size of the nested serialized representation.
    }




    // saturated float16[<=255] cell_voltages
    // Array length prefix: truncated uint8
    if ((offset_bits + 8U) <= capacity_bits)
    {
        out_obj->cell_voltages.count = buffer[offset_bits / 8U] & 255U;
    }
    else
    {
        out_obj->cell_voltages.count = 0U;
    }
    offset_bits += 8U;
    if (out_obj->cell_voltages.count > 255U)
    {
        return -NUNAVUT_ERROR_REPRESENTATION_BAD_ARRAY_LENGTH;
    }
    for (size_t _index1_ = 0U; _index1_ < out_obj->cell_voltages.count; ++_index1_)
    {
        out_obj->cell_voltages.elements[_index1_] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
        offset_bits += 16U;
    }


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    *inout_buffer_size_bytes = (size_t) (nunavutChooseMin(offset_bits, capacity_bits) / 8U);


    return NUNAVUT_SUCCESS;
}

/// Initialize an instance to default values. Does nothing if @param out_obj is NULL.
/// This function intentionally leaves inactive elements uninitialized; for example, members of a variable-length
/// array beyond its length are left uninitialized; aliased union memory that is not used by the first union field
/// is left uninitialized, etc. If full zero-initialization is desired, just use memset(&obj, 0, sizeof(obj)).
static inline void reg_drone_service_battery_Status_0_2_initialize_(reg_drone_service_battery_Status_0_2* const out_obj)
{
    if (out_obj != NULL)
    {
        size_t size_bytes = 0;
        const uint8_t buf = 0;
        const int8_t err = reg_drone_service_battery_Status_0_2_deserialize_(out_obj, &buf, &size_bytes);

        (void) err;
    }
}



#ifdef __cplusplus
}
#endif
#endif // REG_DRONE_SERVICE_BATTERY_STATUS_0_2_INCLUDED_

