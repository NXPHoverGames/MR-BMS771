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
// Generator:     nunavut-1.2.0 (serialization was enabled)
// Source file:   /home/hovergames/src/RDDRONE-BMS772/apps/nxp_bms/BMS_v1/public_regulated_data_types/reg/drone/service/battery/Parameters.0.1.uavcan
// Generated at:  2021-12-29 14:03:55.028522 UTC
// Is deprecated: yes
// Fixed port-ID: None
// Full name:     reg.drone.service.battery.Parameters
// Version:       0.1
//
// Language Options
//     target_endianness:  any
//     omit_float_serialization_support:  False
//     enable_serialization_asserts:  False
//     enable_override_variable_array_capacity:  True

//           _____  ______ _____  _____  ______ _____       _______ ______ _____
//          |  __ `|  ____|  __ `|  __ `|  ____/ ____|   /`|__   __|  ____|  __ `
//          | |  | | |__  | |__) | |__) | |__ | |       /  `  | |  | |__  | |  | |
//          | |  | |  __| |  ___/|  _  /|  __|| |      / /` ` | |  |  __| | |  | |
//          | |__| | |____| |    | | ` `| |___| |____ / ____ `| |  | |____| |__| |
//          |_____/|______|_|    |_|  `_`______`_____/_/    `_`_|  |______|_____/
//
// WARNING: this data type is deprecated and is nearing the end of its life cycle. Seek replacement.

#ifndef REG_DRONE_SERVICE_BATTERY_PARAMETERS_0_1_INCLUDED_
#define REG_DRONE_SERVICE_BATTERY_PARAMETERS_0_1_INCLUDED_

#include <nunavut/support/serialization.h>
#include <reg/drone/service/battery/Technology_0_1.h>
#include <uavcan/si/unit/electric_charge/Scalar_1_0.h>
#include <uavcan/si/unit/electric_current/Scalar_1_0.h>
#include <uavcan/si/unit/mass/Scalar_1_0.h>
#include <uavcan/si/unit/voltage/Scalar_1_0.h>
#include <stdint.h>
#include <stdlib.h>

static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_TARGET_ENDIANNESS == 1693710260,
              "/home/hovergames/src/RDDRONE-BMS772/apps/nxp_bms/BMS_v1/public_regulated_data_types/reg/drone/service/battery/Parameters.0.1.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_OMIT_FLOAT_SERIALIZATION_SUPPORT == 0,
              "/home/hovergames/src/RDDRONE-BMS772/apps/nxp_bms/BMS_v1/public_regulated_data_types/reg/drone/service/battery/Parameters.0.1.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_SERIALIZATION_ASSERTS == 0,
              "/home/hovergames/src/RDDRONE-BMS772/apps/nxp_bms/BMS_v1/public_regulated_data_types/reg/drone/service/battery/Parameters.0.1.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_OVERRIDE_VARIABLE_ARRAY_CAPACITY == 1,
              "/home/hovergames/src/RDDRONE-BMS772/apps/nxp_bms/BMS_v1/public_regulated_data_types/reg/drone/service/battery/Parameters.0.1.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );

#ifdef __cplusplus
extern "C" {
#endif

/// This type does not have a fixed port-ID. See https://forum.uavcan.org/t/choosing-message-and-service-ids/889
#define reg_drone_service_battery_Parameters_0_1_HAS_FIXED_PORT_ID_ false

#define reg_drone_service_battery_Parameters_0_1_FULL_NAME_             "reg.drone.service.battery.Parameters"
#define reg_drone_service_battery_Parameters_0_1_FULL_NAME_AND_VERSION_ "reg.drone.service.battery.Parameters.0.1"

/// Extent is the minimum amount of memory required to hold any serialized representation of any compatible
/// version of the data type; or, on other words, it is the the maximum possible size of received objects of this type.
/// The size is specified in bytes (rather than bits) because by definition, extent is an integer number of bytes long.
/// When allocating a deserialization (RX) buffer for this data type, it should be at least extent bytes large.
/// When allocating a serialization (TX) buffer, it is safe to use the size of the largest serialized representation
/// instead of the extent because it provides a tighter bound of the object size; it is safe because the concrete type
/// is always known during serialization (unlike deserialization). If not sure, use extent everywhere.
#define reg_drone_service_battery_Parameters_0_1_EXTENT_BYTES_                    63UL
#define reg_drone_service_battery_Parameters_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_ 54UL
static_assert(reg_drone_service_battery_Parameters_0_1_EXTENT_BYTES_ >= reg_drone_service_battery_Parameters_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
              "Internal constraint violation");

/// Array metadata for: uavcan.si.unit.voltage.Scalar.1.0[2] design_cell_voltage_min_max
#define reg_drone_service_battery_Parameters_0_1_design_cell_voltage_min_max_ARRAY_CAPACITY_           2U
#define reg_drone_service_battery_Parameters_0_1_design_cell_voltage_min_max_ARRAY_IS_VARIABLE_LENGTH_ false

typedef struct
{
    /// truncated uint64 unique_id
    uint64_t unique_id;

    /// uavcan.si.unit.mass.Scalar.1.0 mass
    uavcan_si_unit_mass_Scalar_1_0 mass;

    /// uavcan.si.unit.electric_charge.Scalar.1.0 design_capacity
    uavcan_si_unit_electric_charge_Scalar_1_0 design_capacity;

    /// uavcan.si.unit.voltage.Scalar.1.0[2] design_cell_voltage_min_max
    uavcan_si_unit_voltage_Scalar_1_0 design_cell_voltage_min_max[2];

    /// uavcan.si.unit.electric_current.Scalar.1.0 discharge_current
    uavcan_si_unit_electric_current_Scalar_1_0 discharge_current;

    /// uavcan.si.unit.electric_current.Scalar.1.0 discharge_current_burst
    uavcan_si_unit_electric_current_Scalar_1_0 discharge_current_burst;

    /// uavcan.si.unit.electric_current.Scalar.1.0 charge_current
    uavcan_si_unit_electric_current_Scalar_1_0 charge_current;

    /// uavcan.si.unit.electric_current.Scalar.1.0 charge_current_fast
    uavcan_si_unit_electric_current_Scalar_1_0 charge_current_fast;

    /// uavcan.si.unit.electric_current.Scalar.1.0 charge_termination_treshold
    uavcan_si_unit_electric_current_Scalar_1_0 charge_termination_treshold;

    /// uavcan.si.unit.voltage.Scalar.1.0 charge_voltage
    uavcan_si_unit_voltage_Scalar_1_0 charge_voltage;

    /// saturated uint16 cycle_count
    uint16_t cycle_count;

    /// saturated uint8 series_cell_count
    uint8_t series_cell_count;

    /// saturated uint7 state_of_health_pct
    uint8_t state_of_health_pct;

    /// reg.drone.service.battery.Technology.0.1 technology
    reg_drone_service_battery_Technology_0_1 technology;
} reg_drone_service_battery_Parameters_0_1;

/// Serialize an instance into the provided buffer.
/// The lifetime of the resulting serialized representation is independent of the original instance.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original object where possible.
///
/// @param obj      The object to serialize.
///
/// @param buffer   The destination buffer. There are no alignment requirements.
///                 @see reg_drone_service_battery_Parameters_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the buffer in bytes.
///                                 Upon return this value will be updated with the size of the constructed serialized
///                                 representation (in bytes); this value is then to be passed over to the transport
///                                 layer. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t reg_drone_service_battery_Parameters_0_1_serialize_(
    const reg_drone_service_battery_Parameters_0_1* const obj, uint8_t* const buffer,  size_t* const inout_buffer_size_bytes)
{
    if ((obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
#ifndef reg_drone_service_battery_Parameters_0_1_DISABLE_SERIALIZATION_BUFFER_CHECK_

    if ((8U * (size_t) capacity_bytes) < 432UL)
    {
        return -NUNAVUT_ERROR_SERIALIZATION_BUFFER_TOO_SMALL;
    }
#endif

    // Notice that fields that are not an integer number of bytes long may overrun the space allocated for them
    // in the serialization buffer up to the next byte boundary. This is by design and is guaranteed to be safe.
    size_t offset_bits = 0U;





    {   // truncated uint64 unique_id
        const int8_t _err0_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, obj->unique_id, 64U);
        if (_err0_ < 0)
        {
            return _err0_;
        }
        offset_bits += 64U;
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

    {   // uavcan.si.unit.mass.Scalar.1.0 mass
        size_t _size_bytes0_ = 4UL;  // Nested object (max) size, in bytes.
        int8_t _err2_ = uavcan_si_unit_mass_Scalar_1_0_serialize_(
            &obj->mass, &buffer[offset_bits / 8U], &_size_bytes0_);
        if (_err2_ < 0)
        {
            return _err2_;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        offset_bits += _size_bytes0_ * 8U;  // Advance by the size of the nested object.
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad1_ = (uint8_t)(8U - offset_bits % 8U);
        const int8_t _err3_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad1_);  // Optimize?
        if (_err3_ < 0)
        {
            return _err3_;
        }
        offset_bits += _pad1_;
    }

    {   // uavcan.si.unit.electric_charge.Scalar.1.0 design_capacity
        size_t _size_bytes1_ = 4UL;  // Nested object (max) size, in bytes.
        int8_t _err4_ = uavcan_si_unit_electric_charge_Scalar_1_0_serialize_(
            &obj->design_capacity, &buffer[offset_bits / 8U], &_size_bytes1_);
        if (_err4_ < 0)
        {
            return _err4_;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        offset_bits += _size_bytes1_ * 8U;  // Advance by the size of the nested object.
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad2_ = (uint8_t)(8U - offset_bits % 8U);
        const int8_t _err5_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad2_);  // Optimize?
        if (_err5_ < 0)
        {
            return _err5_;
        }
        offset_bits += _pad2_;
    }

    {   // uavcan.si.unit.voltage.Scalar.1.0[2] design_cell_voltage_min_max
        const size_t _origin0_ = offset_bits;
        for (size_t _index0_ = 0U; _index0_ < 2UL; ++_index0_)
        {
            size_t _size_bytes2_ = 4UL;  // Nested object (max) size, in bytes.
            int8_t _err6_ = uavcan_si_unit_voltage_Scalar_1_0_serialize_(
                &obj->design_cell_voltage_min_max[_index0_], &buffer[offset_bits / 8U], &_size_bytes2_);
            if (_err6_ < 0)
            {
                return _err6_;
            }
            // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
            offset_bits += _size_bytes2_ * 8U;  // Advance by the size of the nested object.
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        (void) _origin0_;
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad3_ = (uint8_t)(8U - offset_bits % 8U);
        const int8_t _err7_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad3_);  // Optimize?
        if (_err7_ < 0)
        {
            return _err7_;
        }
        offset_bits += _pad3_;
    }

    {   // uavcan.si.unit.electric_current.Scalar.1.0 discharge_current
        size_t _size_bytes3_ = 4UL;  // Nested object (max) size, in bytes.
        int8_t _err8_ = uavcan_si_unit_electric_current_Scalar_1_0_serialize_(
            &obj->discharge_current, &buffer[offset_bits / 8U], &_size_bytes3_);
        if (_err8_ < 0)
        {
            return _err8_;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        offset_bits += _size_bytes3_ * 8U;  // Advance by the size of the nested object.
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad4_ = (uint8_t)(8U - offset_bits % 8U);
        const int8_t _err9_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad4_);  // Optimize?
        if (_err9_ < 0)
        {
            return _err9_;
        }
        offset_bits += _pad4_;
    }

    {   // uavcan.si.unit.electric_current.Scalar.1.0 discharge_current_burst
        size_t _size_bytes4_ = 4UL;  // Nested object (max) size, in bytes.
        int8_t _err10_ = uavcan_si_unit_electric_current_Scalar_1_0_serialize_(
            &obj->discharge_current_burst, &buffer[offset_bits / 8U], &_size_bytes4_);
        if (_err10_ < 0)
        {
            return _err10_;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        offset_bits += _size_bytes4_ * 8U;  // Advance by the size of the nested object.
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad5_ = (uint8_t)(8U - offset_bits % 8U);
        const int8_t _err11_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad5_);  // Optimize?
        if (_err11_ < 0)
        {
            return _err11_;
        }
        offset_bits += _pad5_;
    }

    {   // uavcan.si.unit.electric_current.Scalar.1.0 charge_current
        size_t _size_bytes5_ = 4UL;  // Nested object (max) size, in bytes.
        int8_t _err12_ = uavcan_si_unit_electric_current_Scalar_1_0_serialize_(
            &obj->charge_current, &buffer[offset_bits / 8U], &_size_bytes5_);
        if (_err12_ < 0)
        {
            return _err12_;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        offset_bits += _size_bytes5_ * 8U;  // Advance by the size of the nested object.
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad6_ = (uint8_t)(8U - offset_bits % 8U);
        const int8_t _err13_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad6_);  // Optimize?
        if (_err13_ < 0)
        {
            return _err13_;
        }
        offset_bits += _pad6_;
    }

    {   // uavcan.si.unit.electric_current.Scalar.1.0 charge_current_fast
        size_t _size_bytes6_ = 4UL;  // Nested object (max) size, in bytes.
        int8_t _err14_ = uavcan_si_unit_electric_current_Scalar_1_0_serialize_(
            &obj->charge_current_fast, &buffer[offset_bits / 8U], &_size_bytes6_);
        if (_err14_ < 0)
        {
            return _err14_;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        offset_bits += _size_bytes6_ * 8U;  // Advance by the size of the nested object.
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad7_ = (uint8_t)(8U - offset_bits % 8U);
        const int8_t _err15_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad7_);  // Optimize?
        if (_err15_ < 0)
        {
            return _err15_;
        }
        offset_bits += _pad7_;
    }

    {   // uavcan.si.unit.electric_current.Scalar.1.0 charge_termination_treshold
        size_t _size_bytes7_ = 4UL;  // Nested object (max) size, in bytes.
        int8_t _err16_ = uavcan_si_unit_electric_current_Scalar_1_0_serialize_(
            &obj->charge_termination_treshold, &buffer[offset_bits / 8U], &_size_bytes7_);
        if (_err16_ < 0)
        {
            return _err16_;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        offset_bits += _size_bytes7_ * 8U;  // Advance by the size of the nested object.
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad8_ = (uint8_t)(8U - offset_bits % 8U);
        const int8_t _err17_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad8_);  // Optimize?
        if (_err17_ < 0)
        {
            return _err17_;
        }
        offset_bits += _pad8_;
    }

    {   // uavcan.si.unit.voltage.Scalar.1.0 charge_voltage
        size_t _size_bytes8_ = 4UL;  // Nested object (max) size, in bytes.
        int8_t _err18_ = uavcan_si_unit_voltage_Scalar_1_0_serialize_(
            &obj->charge_voltage, &buffer[offset_bits / 8U], &_size_bytes8_);
        if (_err18_ < 0)
        {
            return _err18_;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        offset_bits += _size_bytes8_ * 8U;  // Advance by the size of the nested object.
    }




    {   // saturated uint16 cycle_count
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err19_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, obj->cycle_count, 16U);
        if (_err19_ < 0)
        {
            return _err19_;
        }
        offset_bits += 16U;
    }




    {   // void8
        buffer[offset_bits / 8U] = 0U;
        offset_bits += 8UL;
    }




    {   // saturated uint8 series_cell_count
        // Saturation code not emitted -- native representation matches the serialized representation.
        buffer[offset_bits / 8U] = (uint8_t)(obj->series_cell_count);  // C std, 6.3.1.3 Signed and unsigned integers
        offset_bits += 8U;
    }




    {   // saturated uint7 state_of_health_pct
        uint8_t _sat0_ = obj->state_of_health_pct;
        if (_sat0_ > 127U)
        {
            _sat0_ = 127U;
        }
        buffer[offset_bits / 8U] = (uint8_t)(_sat0_);  // C std, 6.3.1.3 Signed and unsigned integers
        offset_bits += 7U;
    }




    {   // void1
        const int8_t _err20_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, 1U);  // Optimize?
        if (_err20_ < 0)
        {
            return _err20_;
        }
        offset_bits += 1UL;
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad9_ = (uint8_t)(8U - offset_bits % 8U);
        const int8_t _err21_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad9_);  // Optimize?
        if (_err21_ < 0)
        {
            return _err21_;
        }
        offset_bits += _pad9_;
    }

    {   // reg.drone.service.battery.Technology.0.1 technology
        size_t _size_bytes9_ = 1UL;  // Nested object (max) size, in bytes.
        int8_t _err22_ = reg_drone_service_battery_Technology_0_1_serialize_(
            &obj->technology, &buffer[offset_bits / 8U], &_size_bytes9_);
        if (_err22_ < 0)
        {
            return _err22_;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        offset_bits += _size_bytes9_ * 8U;  // Advance by the size of the nested object.
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad10_ = (uint8_t)(8U - offset_bits % 8U);
        const int8_t _err23_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad10_);  // Optimize?
        if (_err23_ < 0)
        {
            return _err23_;
        }
        offset_bits += _pad10_;
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
static inline int8_t reg_drone_service_battery_Parameters_0_1_deserialize_(
    reg_drone_service_battery_Parameters_0_1* const out_obj, const uint8_t* const buffer, size_t* const inout_buffer_size_bytes)
{
    if ((out_obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
    const size_t capacity_bits = capacity_bytes * (size_t) 8U;
    size_t offset_bits = 0U;





    // truncated uint64 unique_id
    out_obj->unique_id = nunavutGetU64(&buffer[0], capacity_bytes, offset_bits, 64);
    offset_bits += 64U;


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    // uavcan.si.unit.mass.Scalar.1.0 mass
    {
        size_t _size_bytes10_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
        const int8_t _err24_ = uavcan_si_unit_mass_Scalar_1_0_deserialize_(
            &out_obj->mass, &buffer[offset_bits / 8U], &_size_bytes10_);
        if (_err24_ < 0)
        {
            return _err24_;
        }
        offset_bits += _size_bytes10_ * 8U;  // Advance by the size of the nested serialized representation.
    }


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    // uavcan.si.unit.electric_charge.Scalar.1.0 design_capacity
    {
        size_t _size_bytes11_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
        const int8_t _err25_ = uavcan_si_unit_electric_charge_Scalar_1_0_deserialize_(
            &out_obj->design_capacity, &buffer[offset_bits / 8U], &_size_bytes11_);
        if (_err25_ < 0)
        {
            return _err25_;
        }
        offset_bits += _size_bytes11_ * 8U;  // Advance by the size of the nested serialized representation.
    }


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    // uavcan.si.unit.voltage.Scalar.1.0[2] design_cell_voltage_min_max
    for (size_t _index1_ = 0U; _index1_ < 2UL; ++_index1_)
    {
        {
            size_t _size_bytes12_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
            const int8_t _err26_ = uavcan_si_unit_voltage_Scalar_1_0_deserialize_(
                &out_obj->design_cell_voltage_min_max[_index1_], &buffer[offset_bits / 8U], &_size_bytes12_);
            if (_err26_ < 0)
            {
                return _err26_;
            }
            offset_bits += _size_bytes12_ * 8U;  // Advance by the size of the nested serialized representation.
        }
    }


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    // uavcan.si.unit.electric_current.Scalar.1.0 discharge_current
    {
        size_t _size_bytes13_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
        const int8_t _err27_ = uavcan_si_unit_electric_current_Scalar_1_0_deserialize_(
            &out_obj->discharge_current, &buffer[offset_bits / 8U], &_size_bytes13_);
        if (_err27_ < 0)
        {
            return _err27_;
        }
        offset_bits += _size_bytes13_ * 8U;  // Advance by the size of the nested serialized representation.
    }


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    // uavcan.si.unit.electric_current.Scalar.1.0 discharge_current_burst
    {
        size_t _size_bytes14_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
        const int8_t _err28_ = uavcan_si_unit_electric_current_Scalar_1_0_deserialize_(
            &out_obj->discharge_current_burst, &buffer[offset_bits / 8U], &_size_bytes14_);
        if (_err28_ < 0)
        {
            return _err28_;
        }
        offset_bits += _size_bytes14_ * 8U;  // Advance by the size of the nested serialized representation.
    }


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    // uavcan.si.unit.electric_current.Scalar.1.0 charge_current
    {
        size_t _size_bytes15_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
        const int8_t _err29_ = uavcan_si_unit_electric_current_Scalar_1_0_deserialize_(
            &out_obj->charge_current, &buffer[offset_bits / 8U], &_size_bytes15_);
        if (_err29_ < 0)
        {
            return _err29_;
        }
        offset_bits += _size_bytes15_ * 8U;  // Advance by the size of the nested serialized representation.
    }


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    // uavcan.si.unit.electric_current.Scalar.1.0 charge_current_fast
    {
        size_t _size_bytes16_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
        const int8_t _err30_ = uavcan_si_unit_electric_current_Scalar_1_0_deserialize_(
            &out_obj->charge_current_fast, &buffer[offset_bits / 8U], &_size_bytes16_);
        if (_err30_ < 0)
        {
            return _err30_;
        }
        offset_bits += _size_bytes16_ * 8U;  // Advance by the size of the nested serialized representation.
    }


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    // uavcan.si.unit.electric_current.Scalar.1.0 charge_termination_treshold
    {
        size_t _size_bytes17_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
        const int8_t _err31_ = uavcan_si_unit_electric_current_Scalar_1_0_deserialize_(
            &out_obj->charge_termination_treshold, &buffer[offset_bits / 8U], &_size_bytes17_);
        if (_err31_ < 0)
        {
            return _err31_;
        }
        offset_bits += _size_bytes17_ * 8U;  // Advance by the size of the nested serialized representation.
    }


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    // uavcan.si.unit.voltage.Scalar.1.0 charge_voltage
    {
        size_t _size_bytes18_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
        const int8_t _err32_ = uavcan_si_unit_voltage_Scalar_1_0_deserialize_(
            &out_obj->charge_voltage, &buffer[offset_bits / 8U], &_size_bytes18_);
        if (_err32_ < 0)
        {
            return _err32_;
        }
        offset_bits += _size_bytes18_ * 8U;  // Advance by the size of the nested serialized representation.
    }




    // saturated uint16 cycle_count
    out_obj->cycle_count = nunavutGetU16(&buffer[0], capacity_bytes, offset_bits, 16);
    offset_bits += 16U;




    // void8
    offset_bits += 8;




    // saturated uint8 series_cell_count
    if ((offset_bits + 8U) <= capacity_bits)
    {
        out_obj->series_cell_count = buffer[offset_bits / 8U] & 255U;
    }
    else
    {
        out_obj->series_cell_count = 0U;
    }
    offset_bits += 8U;




    // saturated uint7 state_of_health_pct
    if ((offset_bits + 7U) <= capacity_bits)
    {
        out_obj->state_of_health_pct = buffer[offset_bits / 8U] & 127U;
    }
    else
    {
        out_obj->state_of_health_pct = 0U;
    }
    offset_bits += 7U;




    // void1
    offset_bits += 1;


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    // reg.drone.service.battery.Technology.0.1 technology
    {
        size_t _size_bytes19_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
        const int8_t _err33_ = reg_drone_service_battery_Technology_0_1_deserialize_(
            &out_obj->technology, &buffer[offset_bits / 8U], &_size_bytes19_);
        if (_err33_ < 0)
        {
            return _err33_;
        }
        offset_bits += _size_bytes19_ * 8U;  // Advance by the size of the nested serialized representation.
    }


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    *inout_buffer_size_bytes = (size_t) (nunavutChooseMin(offset_bits, capacity_bits) / 8U);


    return NUNAVUT_SUCCESS;
}

/// Initialize an instance to default values. Does nothing if @param out_obj is NULL.
/// This function intentionally leaves inactive elements uninitialized; for example, members of a variable-length
/// array beyond its length are left uninitialized; aliased union memory that is not used by the first union field
/// is left uninitialized, etc. If full zero-initialization is desired, just use memset(&obj, 0, sizeof(obj)).
static inline void reg_drone_service_battery_Parameters_0_1_initialize_(reg_drone_service_battery_Parameters_0_1* const out_obj)
{
    if (out_obj != NULL)
    {
        size_t size_bytes = 0;
        const uint8_t buf = 0;
        const int8_t err = reg_drone_service_battery_Parameters_0_1_deserialize_(out_obj, &buf, &size_bytes);

        (void) err;
    }
}



#ifdef __cplusplus
}
#endif
#endif // REG_DRONE_SERVICE_BATTERY_PARAMETERS_0_1_INCLUDED_

