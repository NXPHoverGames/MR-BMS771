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
// Source file:   /home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/legacy/CoarseOrientation.1.0.uavcan
// Generated at:  2021-04-12 07:48:21.430931 UTC
// Is deprecated: no
// Fixed port-ID: None
// Full name:     legacy.CoarseOrientation
// Version:       1.0
//
// Language Options
//     target_endianness:  any
//     omit_float_serialization_support:  False
//     enable_serialization_asserts:  False
//     enable_override_variable_array_capacity:  True

#ifndef LEGACY_COARSE_ORIENTATION_1_0_INCLUDED_
#define LEGACY_COARSE_ORIENTATION_1_0_INCLUDED_

#include <nunavut/support/serialization.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_TARGET_ENDIANNESS == 1693710260,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/legacy/CoarseOrientation.1.0.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_OMIT_FLOAT_SERIALIZATION_SUPPORT == 0,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/legacy/CoarseOrientation.1.0.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_SERIALIZATION_ASSERTS == 0,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/legacy/CoarseOrientation.1.0.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_OVERRIDE_VARIABLE_ARRAY_CAPACITY == 1,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/legacy/CoarseOrientation.1.0.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );

#ifdef __cplusplus
extern "C" {
#endif

/// This type does not have a fixed port-ID. See https://forum.uavcan.org/t/choosing-message-and-service-ids/889
#define legacy_CoarseOrientation_1_0_HAS_FIXED_PORT_ID_ false

#define legacy_CoarseOrientation_1_0_FULL_NAME_             "legacy.CoarseOrientation"
#define legacy_CoarseOrientation_1_0_FULL_NAME_AND_VERSION_ "legacy.CoarseOrientation.1.0"

/// Extent is the minimum amount of memory required to hold any serialized representation of any compatible
/// version of the data type; or, on other words, it is the the maximum possible size of received objects of this type.
/// The size is specified in bytes (rather than bits) because by definition, extent is an integer number of bytes long.
/// When allocating a deserialization (RX) buffer for this data type, it should be at least extent bytes large.
/// When allocating a serialization (TX) buffer, it is safe to use the size of the largest serialized representation
/// instead of the extent because it provides a tighter bound of the object size; it is safe because the concrete type
/// is always known during serialization (unlike deserialization). If not sure, use extent everywhere.
#define legacy_CoarseOrientation_1_0_EXTENT_BYTES_                    64UL
#define legacy_CoarseOrientation_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_ 2UL
static_assert(legacy_CoarseOrientation_1_0_EXTENT_BYTES_ >= legacy_CoarseOrientation_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
              "Internal constraint violation");

/// saturated float32 ANGLE_MULTIPLIER = 9549296585513721/2000000000000000
#define legacy_CoarseOrientation_1_0_ANGLE_MULTIPLIER (((float) (9549296585513721.0 / 2000000000000000.0)))

/// Array metadata for: saturated int5[3] fixed_axis_roll_pitch_yaw
#define legacy_CoarseOrientation_1_0_fixed_axis_roll_pitch_yaw_ARRAY_CAPACITY_           3U
#define legacy_CoarseOrientation_1_0_fixed_axis_roll_pitch_yaw_ARRAY_IS_VARIABLE_LENGTH_ false

typedef struct
{
    /// saturated int5[3] fixed_axis_roll_pitch_yaw
    int8_t fixed_axis_roll_pitch_yaw[3];

    /// saturated bool orientation_defined
    bool orientation_defined;
} legacy_CoarseOrientation_1_0;

/// Serialize an instance into the provided buffer.
/// The lifetime of the resulting serialized representation is independent of the original instance.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original object where possible.
///
/// @param obj      The object to serialize.
///
/// @param buffer   The destination buffer. There are no alignment requirements.
///                 @see legacy_CoarseOrientation_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the buffer in bytes.
///                                 Upon return this value will be updated with the size of the constructed serialized
///                                 representation (in bytes); this value is then to be passed over to the transport
///                                 layer. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t legacy_CoarseOrientation_1_0_serialize_(
    const legacy_CoarseOrientation_1_0* const obj, uint8_t* const buffer,  size_t* const inout_buffer_size_bytes)
{
    if ((obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
#ifndef legacy_CoarseOrientation_1_0_DISABLE_SERIALIZATION_BUFFER_CHECK_

    if ((8U * (size_t) capacity_bytes) < 16UL)
    {
        return -NUNAVUT_ERROR_SERIALIZATION_BUFFER_TOO_SMALL;
    }
#endif

    // Notice that fields that are not an integer number of bytes long may overrun the space allocated for them
    // in the serialization buffer up to the next byte boundary. This is by design and is guaranteed to be safe.
    size_t offset_bits = 0U;





    {   // saturated int5[3] fixed_axis_roll_pitch_yaw
        const size_t _origin0_ = offset_bits;
        {   // Array element #0
            int8_t _sat0_ = obj->fixed_axis_roll_pitch_yaw[0];
            if (_sat0_ < -16)
            {
                _sat0_ = -16;
            }
            if (_sat0_ > 15)
            {
                _sat0_ = 15;
            }
            buffer[offset_bits / 8U] = (uint8_t)(_sat0_);  // C std, 6.3.1.3 Signed and unsigned integers
            offset_bits += 5U;
        }
        {   // Array element #1
            int8_t _sat1_ = obj->fixed_axis_roll_pitch_yaw[1];
            if (_sat1_ < -16)
            {
                _sat1_ = -16;
            }
            if (_sat1_ > 15)
            {
                _sat1_ = 15;
            }
            const int8_t _err0_ = nunavutSetIxx(&buffer[0], capacity_bytes, offset_bits, _sat1_, 5U);
            if (_err0_ < 0)
            {
                return _err0_;
            }
            offset_bits += 5U;
        }
        {   // Array element #2
            int8_t _sat2_ = obj->fixed_axis_roll_pitch_yaw[2];
            if (_sat2_ < -16)
            {
                _sat2_ = -16;
            }
            if (_sat2_ > 15)
            {
                _sat2_ = 15;
            }
            const int8_t _err1_ = nunavutSetIxx(&buffer[0], capacity_bytes, offset_bits, _sat2_, 5U);
            if (_err1_ < 0)
            {
                return _err1_;
            }
            offset_bits += 5U;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        (void) _origin0_;
    }




    {   // saturated bool orientation_defined
        if (obj->orientation_defined)
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] | (1U << (offset_bits % 8U)));
        }
        else
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] & ~(1U << (offset_bits % 8U)));
        }
        offset_bits += 1U;
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad0_ = (uint8_t)(8U - offset_bits % 8U);
        const int8_t _err2_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad0_);  // Optimize?
        if (_err2_ < 0)
        {
            return _err2_;
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
static inline int8_t legacy_CoarseOrientation_1_0_deserialize_(
    legacy_CoarseOrientation_1_0* const out_obj, const uint8_t* const buffer, size_t* const inout_buffer_size_bytes)
{
    if ((out_obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
    const size_t capacity_bits = capacity_bytes * (size_t) 8U;
    size_t offset_bits = 0U;





    // saturated int5[3] fixed_axis_roll_pitch_yaw
    // Array element #0
    out_obj->fixed_axis_roll_pitch_yaw[0] = nunavutGetI8(&buffer[0], capacity_bytes, offset_bits, 5);
    offset_bits += 5U;
    // Array element #1
    out_obj->fixed_axis_roll_pitch_yaw[1] = nunavutGetI8(&buffer[0], capacity_bytes, offset_bits, 5);
    offset_bits += 5U;
    // Array element #2
    out_obj->fixed_axis_roll_pitch_yaw[2] = nunavutGetI8(&buffer[0], capacity_bytes, offset_bits, 5);
    offset_bits += 5U;




    // saturated bool orientation_defined
    if (offset_bits < capacity_bits)
    {
        out_obj->orientation_defined = (buffer[offset_bits / 8U] & (1U << (offset_bits % 8U))) != 0U;
    }
    else
    {
        out_obj->orientation_defined = false;
    }
    offset_bits += 1U;


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    *inout_buffer_size_bytes = (size_t) (nunavutChooseMin(offset_bits, capacity_bits) / 8U);


    return NUNAVUT_SUCCESS;
}

/// Initialize an instance to default values. Does nothing if @param out_obj is NULL.
/// This function intentionally leaves inactive elements uninitialized; for example, members of a variable-length
/// array beyond its length are left uninitialized; aliased union memory that is not used by the first union field
/// is left uninitialized, etc. If full zero-initialization is desired, just use memset(&obj, 0, sizeof(obj)).
static inline void legacy_CoarseOrientation_1_0_initialize_(legacy_CoarseOrientation_1_0* const out_obj)
{
    if (out_obj != NULL)
    {
        size_t size_bytes = 0;
        const uint8_t buf = 0;
        const int8_t err = legacy_CoarseOrientation_1_0_deserialize_(out_obj, &buf, &size_bytes);

        (void) err;
    }
}



#ifdef __cplusplus
}
#endif
#endif // LEGACY_COARSE_ORIENTATION_1_0_INCLUDED_

