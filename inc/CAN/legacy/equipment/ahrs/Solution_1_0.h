/*
 * Copyright 2020-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
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
// Source file:   /home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/legacy/equipment/ahrs/Solution.1.0.uavcan
// Generated at:  2021-04-12 07:48:57.610066 UTC
// Is deprecated: no
// Fixed port-ID: None
// Full name:     legacy.equipment.ahrs.Solution
// Version:       1.0
//
// Language Options
//     target_endianness:  any
//     omit_float_serialization_support:  False
//     enable_serialization_asserts:  False
//     enable_override_variable_array_capacity:  True

#ifndef LEGACY_EQUIPMENT_AHRS_SOLUTION_1_0_INCLUDED_
#define LEGACY_EQUIPMENT_AHRS_SOLUTION_1_0_INCLUDED_

#include <nunavut/support/serialization.h>
#include <uavcan/time/SynchronizedTimestamp_1_0.h>
#include <stdlib.h>
#include <string.h>

static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_TARGET_ENDIANNESS == 1693710260,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/legacy/equipment/ahrs/Solution.1.0.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_OMIT_FLOAT_SERIALIZATION_SUPPORT == 0,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/legacy/equipment/ahrs/Solution.1.0.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_SERIALIZATION_ASSERTS == 0,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/legacy/equipment/ahrs/Solution.1.0.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_OVERRIDE_VARIABLE_ARRAY_CAPACITY == 1,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/legacy/equipment/ahrs/Solution.1.0.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );

#ifdef __cplusplus
extern "C" {
#endif

/// This type does not have a fixed port-ID. See https://forum.uavcan.org/t/choosing-message-and-service-ids/889
#define legacy_equipment_ahrs_Solution_1_0_HAS_FIXED_PORT_ID_ false

#define legacy_equipment_ahrs_Solution_1_0_FULL_NAME_             "legacy.equipment.ahrs.Solution"
#define legacy_equipment_ahrs_Solution_1_0_FULL_NAME_AND_VERSION_ "legacy.equipment.ahrs.Solution.1.0"

/// Extent is the minimum amount of memory required to hold any serialized representation of any compatible
/// version of the data type; or, on other words, it is the the maximum possible size of received objects of this type.
/// The size is specified in bytes (rather than bits) because by definition, extent is an integer number of bytes long.
/// When allocating a deserialization (RX) buffer for this data type, it should be at least extent bytes large.
/// When allocating a serialization (TX) buffer, it is safe to use the size of the largest serialized representation
/// instead of the extent because it provides a tighter bound of the object size; it is safe because the concrete type
/// is always known during serialization (unlike deserialization). If not sure, use extent everywhere.
#define legacy_equipment_ahrs_Solution_1_0_EXTENT_BYTES_                    170UL
#define legacy_equipment_ahrs_Solution_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_ 85UL
static_assert(legacy_equipment_ahrs_Solution_1_0_EXTENT_BYTES_ >= legacy_equipment_ahrs_Solution_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
              "Internal constraint violation");

/// Array metadata for: saturated float16[4] orientation_xyzw
#define legacy_equipment_ahrs_Solution_1_0_orientation_xyzw_ARRAY_CAPACITY_           4U
#define legacy_equipment_ahrs_Solution_1_0_orientation_xyzw_ARRAY_IS_VARIABLE_LENGTH_ false
/// Array metadata for: saturated float16[<=9] orientation_covariance
#ifndef legacy_equipment_ahrs_Solution_1_0_orientation_covariance_ARRAY_CAPACITY_
#define legacy_equipment_ahrs_Solution_1_0_orientation_covariance_ARRAY_CAPACITY_           9U
#elif !defined(legacy_equipment_ahrs_Solution_1_0_DISABLE_SERIALIZATION_BUFFER_CHECK_)
#  define legacy_equipment_ahrs_Solution_1_0_DISABLE_SERIALIZATION_BUFFER_CHECK_
#endif
#if legacy_equipment_ahrs_Solution_1_0_orientation_covariance_ARRAY_CAPACITY_ > 9U
#  error legacy_equipment_ahrs_Solution_1_0_orientation_covariance_ARRAY_CAPACITY_ > 9U
#endif
#define legacy_equipment_ahrs_Solution_1_0_orientation_covariance_ARRAY_IS_VARIABLE_LENGTH_ true
/// Array metadata for: saturated float16[3] angular_velocity
#define legacy_equipment_ahrs_Solution_1_0_angular_velocity_ARRAY_CAPACITY_           3U
#define legacy_equipment_ahrs_Solution_1_0_angular_velocity_ARRAY_IS_VARIABLE_LENGTH_ false
/// Array metadata for: saturated float16[<=9] angular_velocity_covariance
#ifndef legacy_equipment_ahrs_Solution_1_0_angular_velocity_covariance_ARRAY_CAPACITY_
#define legacy_equipment_ahrs_Solution_1_0_angular_velocity_covariance_ARRAY_CAPACITY_           9U
#elif !defined(legacy_equipment_ahrs_Solution_1_0_DISABLE_SERIALIZATION_BUFFER_CHECK_)
#  define legacy_equipment_ahrs_Solution_1_0_DISABLE_SERIALIZATION_BUFFER_CHECK_
#endif
#if legacy_equipment_ahrs_Solution_1_0_angular_velocity_covariance_ARRAY_CAPACITY_ > 9U
#  error legacy_equipment_ahrs_Solution_1_0_angular_velocity_covariance_ARRAY_CAPACITY_ > 9U
#endif
#define legacy_equipment_ahrs_Solution_1_0_angular_velocity_covariance_ARRAY_IS_VARIABLE_LENGTH_ true
/// Array metadata for: saturated float16[3] linear_acceleration
#define legacy_equipment_ahrs_Solution_1_0_linear_acceleration_ARRAY_CAPACITY_           3U
#define legacy_equipment_ahrs_Solution_1_0_linear_acceleration_ARRAY_IS_VARIABLE_LENGTH_ false
/// Array metadata for: saturated float16[<=9] linear_acceleration_covariance
#ifndef legacy_equipment_ahrs_Solution_1_0_linear_acceleration_covariance_ARRAY_CAPACITY_
#define legacy_equipment_ahrs_Solution_1_0_linear_acceleration_covariance_ARRAY_CAPACITY_           9U
#elif !defined(legacy_equipment_ahrs_Solution_1_0_DISABLE_SERIALIZATION_BUFFER_CHECK_)
#  define legacy_equipment_ahrs_Solution_1_0_DISABLE_SERIALIZATION_BUFFER_CHECK_
#endif
#if legacy_equipment_ahrs_Solution_1_0_linear_acceleration_covariance_ARRAY_CAPACITY_ > 9U
#  error legacy_equipment_ahrs_Solution_1_0_linear_acceleration_covariance_ARRAY_CAPACITY_ > 9U
#endif
#define legacy_equipment_ahrs_Solution_1_0_linear_acceleration_covariance_ARRAY_IS_VARIABLE_LENGTH_ true

typedef struct
{
    /// uavcan.time.SynchronizedTimestamp.1.0 timestamp
    uavcan_time_SynchronizedTimestamp_1_0 timestamp;

    /// saturated float16[4] orientation_xyzw
    float orientation_xyzw[4];

    /// saturated float16[<=9] orientation_covariance
    struct  /// Array address equivalence guarantee: &elements[0] == &orientation_covariance
    {
        float elements[legacy_equipment_ahrs_Solution_1_0_orientation_covariance_ARRAY_CAPACITY_];
        size_t count;
    } orientation_covariance;

    /// saturated float16[3] angular_velocity
    float angular_velocity[3];

    /// saturated float16[<=9] angular_velocity_covariance
    struct  /// Array address equivalence guarantee: &elements[0] == &angular_velocity_covariance
    {
        float elements[legacy_equipment_ahrs_Solution_1_0_angular_velocity_covariance_ARRAY_CAPACITY_];
        size_t count;
    } angular_velocity_covariance;

    /// saturated float16[3] linear_acceleration
    float linear_acceleration[3];

    /// saturated float16[<=9] linear_acceleration_covariance
    struct  /// Array address equivalence guarantee: &elements[0] == &linear_acceleration_covariance
    {
        float elements[legacy_equipment_ahrs_Solution_1_0_linear_acceleration_covariance_ARRAY_CAPACITY_];
        size_t count;
    } linear_acceleration_covariance;
} legacy_equipment_ahrs_Solution_1_0;

/// Serialize an instance into the provided buffer.
/// The lifetime of the resulting serialized representation is independent of the original instance.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original object where possible.
///
/// @param obj      The object to serialize.
///
/// @param buffer   The destination buffer. There are no alignment requirements.
///                 @see legacy_equipment_ahrs_Solution_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the buffer in bytes.
///                                 Upon return this value will be updated with the size of the constructed serialized
///                                 representation (in bytes); this value is then to be passed over to the transport
///                                 layer. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t legacy_equipment_ahrs_Solution_1_0_serialize_(
    const legacy_equipment_ahrs_Solution_1_0* const obj, uint8_t* const buffer,  size_t* const inout_buffer_size_bytes)
{
    if ((obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
#ifndef legacy_equipment_ahrs_Solution_1_0_DISABLE_SERIALIZATION_BUFFER_CHECK_

    if ((8U * (size_t) capacity_bytes) < 680UL)
    {
        return -NUNAVUT_ERROR_SERIALIZATION_BUFFER_TOO_SMALL;
    }
#endif

    // Notice that fields that are not an integer number of bytes long may overrun the space allocated for them
    // in the serialization buffer up to the next byte boundary. This is by design and is guaranteed to be safe.
    size_t offset_bits = 0U;





    {   // uavcan.time.SynchronizedTimestamp.1.0 timestamp
        size_t _size_bytes0_ = 7UL;  // Nested object (max) size, in bytes.
        int8_t _err0_ = uavcan_time_SynchronizedTimestamp_1_0_serialize_(
            &obj->timestamp, &buffer[offset_bits / 8U], &_size_bytes0_);
        if (_err0_ < 0)
        {
            return _err0_;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        offset_bits += _size_bytes0_ * 8U;  // Advance by the size of the nested object.
    }




    {   // saturated float16[4] orientation_xyzw
        const size_t _origin0_ = offset_bits;
        {   // Array element #0
            float _sat0_ = obj->orientation_xyzw[0];
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
            const int8_t _err1_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat0_);
            if (_err1_ < 0)
            {
                return _err1_;
            }
            offset_bits += 16U;
        }
        {   // Array element #1
            float _sat1_ = obj->orientation_xyzw[1];
            if (isfinite(_sat1_))
            {
                if (_sat1_ < ((float) -65504.0))
                {
                    _sat1_ = ((float) -65504.0);
                }
                if (_sat1_ > ((float) 65504.0))
                {
                    _sat1_ = ((float) 65504.0);
                }
            }
            const int8_t _err2_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat1_);
            if (_err2_ < 0)
            {
                return _err2_;
            }
            offset_bits += 16U;
        }
        {   // Array element #2
            float _sat2_ = obj->orientation_xyzw[2];
            if (isfinite(_sat2_))
            {
                if (_sat2_ < ((float) -65504.0))
                {
                    _sat2_ = ((float) -65504.0);
                }
                if (_sat2_ > ((float) 65504.0))
                {
                    _sat2_ = ((float) 65504.0);
                }
            }
            const int8_t _err3_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat2_);
            if (_err3_ < 0)
            {
                return _err3_;
            }
            offset_bits += 16U;
        }
        {   // Array element #3
            float _sat3_ = obj->orientation_xyzw[3];
            if (isfinite(_sat3_))
            {
                if (_sat3_ < ((float) -65504.0))
                {
                    _sat3_ = ((float) -65504.0);
                }
                if (_sat3_ > ((float) 65504.0))
                {
                    _sat3_ = ((float) 65504.0);
                }
            }
            const int8_t _err4_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat3_);
            if (_err4_ < 0)
            {
                return _err4_;
            }
            offset_bits += 16U;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        (void) _origin0_;
    }




    {   // void4
        buffer[offset_bits / 8U] = 0U;
        offset_bits += 4UL;
    }




    {   // saturated float16[<=9] orientation_covariance
        if (obj->orientation_covariance.count > 9)
        {
            return -NUNAVUT_ERROR_REPRESENTATION_BAD_ARRAY_LENGTH;
        }
        // Array length prefix: truncated uint8
        const int8_t _err5_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, obj->orientation_covariance.count, 8U);
        if (_err5_ < 0)
        {
            return _err5_;
        }
        offset_bits += 8U;
        for (size_t _index0_ = 0U; _index0_ < obj->orientation_covariance.count; ++_index0_)
        {
            float _sat4_ = obj->orientation_covariance.elements[_index0_];
            if (isfinite(_sat4_))
            {
                if (_sat4_ < ((float) -65504.0))
                {
                    _sat4_ = ((float) -65504.0);
                }
                if (_sat4_ > ((float) 65504.0))
                {
                    _sat4_ = ((float) 65504.0);
                }
            }
            const int8_t _err6_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat4_);
            if (_err6_ < 0)
            {
                return _err6_;
            }
            offset_bits += 16U;
        }
    }




    {   // saturated float16[3] angular_velocity
        const size_t _origin1_ = offset_bits;
        {   // Array element #0
            float _sat5_ = obj->angular_velocity[0];
            if (isfinite(_sat5_))
            {
                if (_sat5_ < ((float) -65504.0))
                {
                    _sat5_ = ((float) -65504.0);
                }
                if (_sat5_ > ((float) 65504.0))
                {
                    _sat5_ = ((float) 65504.0);
                }
            }
            const int8_t _err7_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat5_);
            if (_err7_ < 0)
            {
                return _err7_;
            }
            offset_bits += 16U;
        }
        {   // Array element #1
            float _sat6_ = obj->angular_velocity[1];
            if (isfinite(_sat6_))
            {
                if (_sat6_ < ((float) -65504.0))
                {
                    _sat6_ = ((float) -65504.0);
                }
                if (_sat6_ > ((float) 65504.0))
                {
                    _sat6_ = ((float) 65504.0);
                }
            }
            const int8_t _err8_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat6_);
            if (_err8_ < 0)
            {
                return _err8_;
            }
            offset_bits += 16U;
        }
        {   // Array element #2
            float _sat7_ = obj->angular_velocity[2];
            if (isfinite(_sat7_))
            {
                if (_sat7_ < ((float) -65504.0))
                {
                    _sat7_ = ((float) -65504.0);
                }
                if (_sat7_ > ((float) 65504.0))
                {
                    _sat7_ = ((float) 65504.0);
                }
            }
            const int8_t _err9_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat7_);
            if (_err9_ < 0)
            {
                return _err9_;
            }
            offset_bits += 16U;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        (void) _origin1_;
    }




    {   // void4
        const int8_t _err10_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, 4U);  // Optimize?
        if (_err10_ < 0)
        {
            return _err10_;
        }
        offset_bits += 4UL;
    }




    {   // saturated float16[<=9] angular_velocity_covariance
        if (obj->angular_velocity_covariance.count > 9)
        {
            return -NUNAVUT_ERROR_REPRESENTATION_BAD_ARRAY_LENGTH;
        }
        // Array length prefix: truncated uint8
        buffer[offset_bits / 8U] = (uint8_t)(obj->angular_velocity_covariance.count);  // C std, 6.3.1.3 Signed and unsigned integers
        offset_bits += 8U;
        for (size_t _index1_ = 0U; _index1_ < obj->angular_velocity_covariance.count; ++_index1_)
        {
            float _sat8_ = obj->angular_velocity_covariance.elements[_index1_];
            if (isfinite(_sat8_))
            {
                if (_sat8_ < ((float) -65504.0))
                {
                    _sat8_ = ((float) -65504.0);
                }
                if (_sat8_ > ((float) 65504.0))
                {
                    _sat8_ = ((float) 65504.0);
                }
            }
            const int8_t _err11_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat8_);
            if (_err11_ < 0)
            {
                return _err11_;
            }
            offset_bits += 16U;
        }
    }




    {   // saturated float16[3] linear_acceleration
        const size_t _origin2_ = offset_bits;
        {   // Array element #0
            float _sat9_ = obj->linear_acceleration[0];
            if (isfinite(_sat9_))
            {
                if (_sat9_ < ((float) -65504.0))
                {
                    _sat9_ = ((float) -65504.0);
                }
                if (_sat9_ > ((float) 65504.0))
                {
                    _sat9_ = ((float) 65504.0);
                }
            }
            const int8_t _err12_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat9_);
            if (_err12_ < 0)
            {
                return _err12_;
            }
            offset_bits += 16U;
        }
        {   // Array element #1
            float _sat10_ = obj->linear_acceleration[1];
            if (isfinite(_sat10_))
            {
                if (_sat10_ < ((float) -65504.0))
                {
                    _sat10_ = ((float) -65504.0);
                }
                if (_sat10_ > ((float) 65504.0))
                {
                    _sat10_ = ((float) 65504.0);
                }
            }
            const int8_t _err13_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat10_);
            if (_err13_ < 0)
            {
                return _err13_;
            }
            offset_bits += 16U;
        }
        {   // Array element #2
            float _sat11_ = obj->linear_acceleration[2];
            if (isfinite(_sat11_))
            {
                if (_sat11_ < ((float) -65504.0))
                {
                    _sat11_ = ((float) -65504.0);
                }
                if (_sat11_ > ((float) 65504.0))
                {
                    _sat11_ = ((float) 65504.0);
                }
            }
            const int8_t _err14_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat11_);
            if (_err14_ < 0)
            {
                return _err14_;
            }
            offset_bits += 16U;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        (void) _origin2_;
    }




    {   // saturated float16[<=9] linear_acceleration_covariance
        if (obj->linear_acceleration_covariance.count > 9)
        {
            return -NUNAVUT_ERROR_REPRESENTATION_BAD_ARRAY_LENGTH;
        }
        // Array length prefix: truncated uint8
        buffer[offset_bits / 8U] = (uint8_t)(obj->linear_acceleration_covariance.count);  // C std, 6.3.1.3 Signed and unsigned integers
        offset_bits += 8U;
        for (size_t _index2_ = 0U; _index2_ < obj->linear_acceleration_covariance.count; ++_index2_)
        {
            float _sat12_ = obj->linear_acceleration_covariance.elements[_index2_];
            if (isfinite(_sat12_))
            {
                if (_sat12_ < ((float) -65504.0))
                {
                    _sat12_ = ((float) -65504.0);
                }
                if (_sat12_ > ((float) 65504.0))
                {
                    _sat12_ = ((float) 65504.0);
                }
            }
            const int8_t _err15_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat12_);
            if (_err15_ < 0)
            {
                return _err15_;
            }
            offset_bits += 16U;
        }
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad0_ = (uint8_t)(8U - offset_bits % 8U);
        const int8_t _err16_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad0_);  // Optimize?
        if (_err16_ < 0)
        {
            return _err16_;
        }
        offset_bits += _pad0_;
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
static inline int8_t legacy_equipment_ahrs_Solution_1_0_deserialize_(
    legacy_equipment_ahrs_Solution_1_0* const out_obj, const uint8_t* const buffer, size_t* const inout_buffer_size_bytes)
{
    if ((out_obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
    const size_t capacity_bits = capacity_bytes * (size_t) 8U;
    size_t offset_bits = 0U;





    // uavcan.time.SynchronizedTimestamp.1.0 timestamp
    {
        size_t _size_bytes1_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
        const int8_t _err17_ = uavcan_time_SynchronizedTimestamp_1_0_deserialize_(
            &out_obj->timestamp, &buffer[offset_bits / 8U], &_size_bytes1_);
        if (_err17_ < 0)
        {
            return _err17_;
        }
        offset_bits += _size_bytes1_ * 8U;  // Advance by the size of the nested serialized representation.
    }




    // saturated float16[4] orientation_xyzw
    // Array element #0
    out_obj->orientation_xyzw[0] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;
    // Array element #1
    out_obj->orientation_xyzw[1] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;
    // Array element #2
    out_obj->orientation_xyzw[2] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;
    // Array element #3
    out_obj->orientation_xyzw[3] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;




    // void4
    offset_bits += 4;




    // saturated float16[<=9] orientation_covariance
    // Array length prefix: truncated uint8
    out_obj->orientation_covariance.count = nunavutGetU8(&buffer[0], capacity_bytes, offset_bits, 8);
    offset_bits += 8U;
    if (out_obj->orientation_covariance.count > 9U)
    {
        return -NUNAVUT_ERROR_REPRESENTATION_BAD_ARRAY_LENGTH;
    }
    for (size_t _index3_ = 0U; _index3_ < out_obj->orientation_covariance.count; ++_index3_)
    {
        out_obj->orientation_covariance.elements[_index3_] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
        offset_bits += 16U;
    }




    // saturated float16[3] angular_velocity
    // Array element #0
    out_obj->angular_velocity[0] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;
    // Array element #1
    out_obj->angular_velocity[1] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;
    // Array element #2
    out_obj->angular_velocity[2] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;




    // void4
    offset_bits += 4;




    // saturated float16[<=9] angular_velocity_covariance
    // Array length prefix: truncated uint8
    if ((offset_bits + 8U) <= capacity_bits)
    {
        out_obj->angular_velocity_covariance.count = buffer[offset_bits / 8U] & 255U;
    }
    else
    {
        out_obj->angular_velocity_covariance.count = 0U;
    }
    offset_bits += 8U;
    if (out_obj->angular_velocity_covariance.count > 9U)
    {
        return -NUNAVUT_ERROR_REPRESENTATION_BAD_ARRAY_LENGTH;
    }
    for (size_t _index4_ = 0U; _index4_ < out_obj->angular_velocity_covariance.count; ++_index4_)
    {
        out_obj->angular_velocity_covariance.elements[_index4_] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
        offset_bits += 16U;
    }




    // saturated float16[3] linear_acceleration
    // Array element #0
    out_obj->linear_acceleration[0] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;
    // Array element #1
    out_obj->linear_acceleration[1] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;
    // Array element #2
    out_obj->linear_acceleration[2] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;




    // saturated float16[<=9] linear_acceleration_covariance
    // Array length prefix: truncated uint8
    if ((offset_bits + 8U) <= capacity_bits)
    {
        out_obj->linear_acceleration_covariance.count = buffer[offset_bits / 8U] & 255U;
    }
    else
    {
        out_obj->linear_acceleration_covariance.count = 0U;
    }
    offset_bits += 8U;
    if (out_obj->linear_acceleration_covariance.count > 9U)
    {
        return -NUNAVUT_ERROR_REPRESENTATION_BAD_ARRAY_LENGTH;
    }
    for (size_t _index5_ = 0U; _index5_ < out_obj->linear_acceleration_covariance.count; ++_index5_)
    {
        out_obj->linear_acceleration_covariance.elements[_index5_] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
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
static inline void legacy_equipment_ahrs_Solution_1_0_initialize_(legacy_equipment_ahrs_Solution_1_0* const out_obj)
{
    if (out_obj != NULL)
    {
        size_t size_bytes = 0;
        const uint8_t buf = 0;
        const int8_t err = legacy_equipment_ahrs_Solution_1_0_deserialize_(out_obj, &buf, &size_bytes);

        (void) err;
    }
}



#ifdef __cplusplus
}
#endif
#endif // LEGACY_EQUIPMENT_AHRS_SOLUTION_1_0_INCLUDED_

