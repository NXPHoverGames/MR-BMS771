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
// Source file:   /home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/legacy/protocol/HardwareVersion.1.0.uavcan
// Generated at:  2021-04-12 07:48:25.189001 UTC
// Is deprecated: no
// Fixed port-ID: None
// Full name:     legacy.protocol.HardwareVersion
// Version:       1.0
//
// Language Options
//     target_endianness:  any
//     omit_float_serialization_support:  False
//     enable_serialization_asserts:  False
//     enable_override_variable_array_capacity:  True

#ifndef LEGACY_PROTOCOL_HARDWARE_VERSION_1_0_INCLUDED_
#define LEGACY_PROTOCOL_HARDWARE_VERSION_1_0_INCLUDED_

#include <nunavut/support/serialization.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_TARGET_ENDIANNESS == 1693710260,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/legacy/protocol/HardwareVersion.1.0.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_OMIT_FLOAT_SERIALIZATION_SUPPORT == 0,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/legacy/protocol/HardwareVersion.1.0.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_SERIALIZATION_ASSERTS == 0,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/legacy/protocol/HardwareVersion.1.0.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_OVERRIDE_VARIABLE_ARRAY_CAPACITY == 1,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/legacy/protocol/HardwareVersion.1.0.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );

#ifdef __cplusplus
extern "C" {
#endif

/// This type does not have a fixed port-ID. See https://forum.uavcan.org/t/choosing-message-and-service-ids/889
#define legacy_protocol_HardwareVersion_1_0_HAS_FIXED_PORT_ID_ false

#define legacy_protocol_HardwareVersion_1_0_FULL_NAME_             "legacy.protocol.HardwareVersion"
#define legacy_protocol_HardwareVersion_1_0_FULL_NAME_AND_VERSION_ "legacy.protocol.HardwareVersion.1.0"

/// Extent is the minimum amount of memory required to hold any serialized representation of any compatible
/// version of the data type; or, on other words, it is the the maximum possible size of received objects of this type.
/// The size is specified in bytes (rather than bits) because by definition, extent is an integer number of bytes long.
/// When allocating a deserialization (RX) buffer for this data type, it should be at least extent bytes large.
/// When allocating a serialization (TX) buffer, it is safe to use the size of the largest serialized representation
/// instead of the extent because it provides a tighter bound of the object size; it is safe because the concrete type
/// is always known during serialization (unlike deserialization). If not sure, use extent everywhere.
#define legacy_protocol_HardwareVersion_1_0_EXTENT_BYTES_                    548UL
#define legacy_protocol_HardwareVersion_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_ 274UL
static_assert(legacy_protocol_HardwareVersion_1_0_EXTENT_BYTES_ >= legacy_protocol_HardwareVersion_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
              "Internal constraint violation");

/// Array metadata for: saturated uint8[16] unique_id
#define legacy_protocol_HardwareVersion_1_0_unique_id_ARRAY_CAPACITY_           16U
#define legacy_protocol_HardwareVersion_1_0_unique_id_ARRAY_IS_VARIABLE_LENGTH_ false
/// Array metadata for: saturated uint8[<=255] certificate_of_authenticity
#ifndef legacy_protocol_HardwareVersion_1_0_certificate_of_authenticity_ARRAY_CAPACITY_
#define legacy_protocol_HardwareVersion_1_0_certificate_of_authenticity_ARRAY_CAPACITY_           255U
#elif !defined(legacy_protocol_HardwareVersion_1_0_DISABLE_SERIALIZATION_BUFFER_CHECK_)
#  define legacy_protocol_HardwareVersion_1_0_DISABLE_SERIALIZATION_BUFFER_CHECK_
#endif
#if legacy_protocol_HardwareVersion_1_0_certificate_of_authenticity_ARRAY_CAPACITY_ > 255U
#  error legacy_protocol_HardwareVersion_1_0_certificate_of_authenticity_ARRAY_CAPACITY_ > 255U
#endif
#define legacy_protocol_HardwareVersion_1_0_certificate_of_authenticity_ARRAY_IS_VARIABLE_LENGTH_ true

typedef struct
{
    /// saturated uint8 major
    uint8_t major;

    /// saturated uint8 minor
    uint8_t minor;

    /// saturated uint8[16] unique_id
    uint8_t unique_id[16];

    /// saturated uint8[<=255] certificate_of_authenticity
    struct  /// Array address equivalence guarantee: &elements[0] == &certificate_of_authenticity
    {
        uint8_t elements[legacy_protocol_HardwareVersion_1_0_certificate_of_authenticity_ARRAY_CAPACITY_];
        size_t count;
    } certificate_of_authenticity;
} legacy_protocol_HardwareVersion_1_0;

/// Serialize an instance into the provided buffer.
/// The lifetime of the resulting serialized representation is independent of the original instance.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original object where possible.
///
/// @param obj      The object to serialize.
///
/// @param buffer   The destination buffer. There are no alignment requirements.
///                 @see legacy_protocol_HardwareVersion_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the buffer in bytes.
///                                 Upon return this value will be updated with the size of the constructed serialized
///                                 representation (in bytes); this value is then to be passed over to the transport
///                                 layer. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t legacy_protocol_HardwareVersion_1_0_serialize_(
    const legacy_protocol_HardwareVersion_1_0* const obj, uint8_t* const buffer,  size_t* const inout_buffer_size_bytes)
{
    if ((obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
#ifndef legacy_protocol_HardwareVersion_1_0_DISABLE_SERIALIZATION_BUFFER_CHECK_

    if ((8U * (size_t) capacity_bytes) < 2192UL)
    {
        return -NUNAVUT_ERROR_SERIALIZATION_BUFFER_TOO_SMALL;
    }
#endif

    // Notice that fields that are not an integer number of bytes long may overrun the space allocated for them
    // in the serialization buffer up to the next byte boundary. This is by design and is guaranteed to be safe.
    size_t offset_bits = 0U;





    {   // saturated uint8 major
        // Saturation code not emitted -- native representation matches the serialized representation.
        buffer[offset_bits / 8U] = (uint8_t)(obj->major);  // C std, 6.3.1.3 Signed and unsigned integers
        offset_bits += 8U;
    }




    {   // saturated uint8 minor
        // Saturation code not emitted -- native representation matches the serialized representation.
        buffer[offset_bits / 8U] = (uint8_t)(obj->minor);  // C std, 6.3.1.3 Signed and unsigned integers
        offset_bits += 8U;
    }




    {   // saturated uint8[16] unique_id
        // Optimization prospect: this item is aligned at the byte boundary, so it is possible to use memmove().
        nunavutCopyBits(&buffer[0], offset_bits, 16UL * 8U, &obj->unique_id[0], 0U);
        offset_bits += 16UL * 8U;
    }




    {   // saturated uint8[<=255] certificate_of_authenticity
        if (obj->certificate_of_authenticity.count > 255)
        {
            return -NUNAVUT_ERROR_REPRESENTATION_BAD_ARRAY_LENGTH;
        }
        // Array length prefix: truncated uint8
        buffer[offset_bits / 8U] = (uint8_t)(obj->certificate_of_authenticity.count);  // C std, 6.3.1.3 Signed and unsigned integers
        offset_bits += 8U;
        // Optimization prospect: this item is aligned at the byte boundary, so it is possible to use memmove().
        nunavutCopyBits(&buffer[0], offset_bits, obj->certificate_of_authenticity.count * 8U, &obj->certificate_of_authenticity.elements[0], 0U);
        offset_bits += obj->certificate_of_authenticity.count * 8U;
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad0_ = (uint8_t)(8U - offset_bits % 8U);
        const int8_t _err0_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad0_);  // Optimize?
        if (_err0_ < 0)
        {
            return _err0_;
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
static inline int8_t legacy_protocol_HardwareVersion_1_0_deserialize_(
    legacy_protocol_HardwareVersion_1_0* const out_obj, const uint8_t* const buffer, size_t* const inout_buffer_size_bytes)
{
    if ((out_obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
    const size_t capacity_bits = capacity_bytes * (size_t) 8U;
    size_t offset_bits = 0U;





    // saturated uint8 major
    if ((offset_bits + 8U) <= capacity_bits)
    {
        out_obj->major = buffer[offset_bits / 8U] & 255U;
    }
    else
    {
        out_obj->major = 0U;
    }
    offset_bits += 8U;




    // saturated uint8 minor
    if ((offset_bits + 8U) <= capacity_bits)
    {
        out_obj->minor = buffer[offset_bits / 8U] & 255U;
    }
    else
    {
        out_obj->minor = 0U;
    }
    offset_bits += 8U;




    // saturated uint8[16] unique_id
    nunavutGetBits(&out_obj->unique_id[0], &buffer[0], capacity_bytes, offset_bits, 16UL * 8U);
    offset_bits += 16UL * 8U;




    // saturated uint8[<=255] certificate_of_authenticity
    // Array length prefix: truncated uint8
    if ((offset_bits + 8U) <= capacity_bits)
    {
        out_obj->certificate_of_authenticity.count = buffer[offset_bits / 8U] & 255U;
    }
    else
    {
        out_obj->certificate_of_authenticity.count = 0U;
    }
    offset_bits += 8U;
    if (out_obj->certificate_of_authenticity.count > 255U)
    {
        return -NUNAVUT_ERROR_REPRESENTATION_BAD_ARRAY_LENGTH;
    }
    nunavutGetBits(&out_obj->certificate_of_authenticity.elements[0], &buffer[0], capacity_bytes, offset_bits, out_obj->certificate_of_authenticity.count * 8U);
    offset_bits += out_obj->certificate_of_authenticity.count * 8U;


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    *inout_buffer_size_bytes = (size_t) (nunavutChooseMin(offset_bits, capacity_bits) / 8U);


    return NUNAVUT_SUCCESS;
}

/// Initialize an instance to default values. Does nothing if @param out_obj is NULL.
/// This function intentionally leaves inactive elements uninitialized; for example, members of a variable-length
/// array beyond its length are left uninitialized; aliased union memory that is not used by the first union field
/// is left uninitialized, etc. If full zero-initialization is desired, just use memset(&obj, 0, sizeof(obj)).
static inline void legacy_protocol_HardwareVersion_1_0_initialize_(legacy_protocol_HardwareVersion_1_0* const out_obj)
{
    if (out_obj != NULL)
    {
        size_t size_bytes = 0;
        const uint8_t buf = 0;
        const int8_t err = legacy_protocol_HardwareVersion_1_0_deserialize_(out_obj, &buf, &size_bytes);

        (void) err;
    }
}



#ifdef __cplusplus
}
#endif
#endif // LEGACY_PROTOCOL_HARDWARE_VERSION_1_0_INCLUDED_

