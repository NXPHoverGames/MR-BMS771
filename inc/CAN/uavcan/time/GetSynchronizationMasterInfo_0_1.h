/*
 *
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
// Source file:   /home/hovergames/src/RDDRONE-BMS772/apps/nxp_bms/BMS_v1/public_regulated_data_types/uavcan/time/510.GetSynchronizationMasterInfo.0.1.uavcan
// Generated at:  2021-12-29 14:04:20.442159 UTC
// Is deprecated: no
// Fixed port-ID: 510
// Full name:     uavcan.time.GetSynchronizationMasterInfo
// Version:       0.1
//
// Language Options
//     target_endianness:  any
//     omit_float_serialization_support:  False
//     enable_serialization_asserts:  False
//     enable_override_variable_array_capacity:  True

#ifndef UAVCAN_TIME_GET_SYNCHRONIZATION_MASTER_INFO_0_1_INCLUDED_
#define UAVCAN_TIME_GET_SYNCHRONIZATION_MASTER_INFO_0_1_INCLUDED_

#include <nunavut/support/serialization.h>
#include <uavcan/time/TAIInfo_0_1.h>
#include <uavcan/time/TimeSystem_0_1.h>
#include <stdlib.h>

static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_TARGET_ENDIANNESS == 1693710260,
              "/home/hovergames/src/RDDRONE-BMS772/apps/nxp_bms/BMS_v1/public_regulated_data_types/uavcan/time/510.GetSynchronizationMasterInfo.0.1.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_OMIT_FLOAT_SERIALIZATION_SUPPORT == 0,
              "/home/hovergames/src/RDDRONE-BMS772/apps/nxp_bms/BMS_v1/public_regulated_data_types/uavcan/time/510.GetSynchronizationMasterInfo.0.1.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_SERIALIZATION_ASSERTS == 0,
              "/home/hovergames/src/RDDRONE-BMS772/apps/nxp_bms/BMS_v1/public_regulated_data_types/uavcan/time/510.GetSynchronizationMasterInfo.0.1.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_OVERRIDE_VARIABLE_ARRAY_CAPACITY == 1,
              "/home/hovergames/src/RDDRONE-BMS772/apps/nxp_bms/BMS_v1/public_regulated_data_types/uavcan/time/510.GetSynchronizationMasterInfo.0.1.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );

#ifdef __cplusplus
extern "C" {
#endif

#define uavcan_time_GetSynchronizationMasterInfo_0_1_HAS_FIXED_PORT_ID_ true
#define uavcan_time_GetSynchronizationMasterInfo_0_1_FIXED_PORT_ID_     510U

#define uavcan_time_GetSynchronizationMasterInfo_0_1_FULL_NAME_             "uavcan.time.GetSynchronizationMasterInfo"
#define uavcan_time_GetSynchronizationMasterInfo_0_1_FULL_NAME_AND_VERSION_ "uavcan.time.GetSynchronizationMasterInfo.0.1"

#define uavcan_time_GetSynchronizationMasterInfo_Request_0_1_FULL_NAME_             "uavcan.time.GetSynchronizationMasterInfo.Request"
#define uavcan_time_GetSynchronizationMasterInfo_Request_0_1_FULL_NAME_AND_VERSION_ "uavcan.time.GetSynchronizationMasterInfo.Request.0.1"

/// Extent is the minimum amount of memory required to hold any serialized representation of any compatible
/// version of the data type; or, on other words, it is the the maximum possible size of received objects of this type.
/// The size is specified in bytes (rather than bits) because by definition, extent is an integer number of bytes long.
/// When allocating a deserialization (RX) buffer for this data type, it should be at least extent bytes large.
/// When allocating a serialization (TX) buffer, it is safe to use the size of the largest serialized representation
/// instead of the extent because it provides a tighter bound of the object size; it is safe because the concrete type
/// is always known during serialization (unlike deserialization). If not sure, use extent everywhere.
#define uavcan_time_GetSynchronizationMasterInfo_Request_0_1_EXTENT_BYTES_                    48UL
#define uavcan_time_GetSynchronizationMasterInfo_Request_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_ 0UL
static_assert(uavcan_time_GetSynchronizationMasterInfo_Request_0_1_EXTENT_BYTES_ >= uavcan_time_GetSynchronizationMasterInfo_Request_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
              "Internal constraint violation");

typedef struct
{
    uint8_t _dummy_;
} uavcan_time_GetSynchronizationMasterInfo_Request_0_1;

/// Serialize an instance into the provided buffer.
/// The lifetime of the resulting serialized representation is independent of the original instance.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original object where possible.
///
/// @param obj      The object to serialize.
///
/// @param buffer   The destination buffer. There are no alignment requirements.
///                 @see uavcan_time_GetSynchronizationMasterInfo_Request_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the buffer in bytes.
///                                 Upon return this value will be updated with the size of the constructed serialized
///                                 representation (in bytes); this value is then to be passed over to the transport
///                                 layer. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t uavcan_time_GetSynchronizationMasterInfo_Request_0_1_serialize_(
    const uavcan_time_GetSynchronizationMasterInfo_Request_0_1* const obj, uint8_t* const buffer,  size_t* const inout_buffer_size_bytes)
{
    if ((obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }

    *inout_buffer_size_bytes = 0U;

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
static inline int8_t uavcan_time_GetSynchronizationMasterInfo_Request_0_1_deserialize_(
    uavcan_time_GetSynchronizationMasterInfo_Request_0_1* const out_obj, const uint8_t* const buffer, size_t* const inout_buffer_size_bytes)
{
    if ((out_obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }

    *inout_buffer_size_bytes = 0U;

    return NUNAVUT_SUCCESS;
}

/// Initialize an instance to default values. Does nothing if @param out_obj is NULL.
/// This function intentionally leaves inactive elements uninitialized; for example, members of a variable-length
/// array beyond its length are left uninitialized; aliased union memory that is not used by the first union field
/// is left uninitialized, etc. If full zero-initialization is desired, just use memset(&obj, 0, sizeof(obj)).
static inline void uavcan_time_GetSynchronizationMasterInfo_Request_0_1_initialize_(uavcan_time_GetSynchronizationMasterInfo_Request_0_1* const out_obj)
{
    if (out_obj != NULL)
    {
        size_t size_bytes = 0;
        const uint8_t buf = 0;
        const int8_t err = uavcan_time_GetSynchronizationMasterInfo_Request_0_1_deserialize_(out_obj, &buf, &size_bytes);

        (void) err;
    }
}



#define uavcan_time_GetSynchronizationMasterInfo_Response_0_1_FULL_NAME_             "uavcan.time.GetSynchronizationMasterInfo.Response"
#define uavcan_time_GetSynchronizationMasterInfo_Response_0_1_FULL_NAME_AND_VERSION_ "uavcan.time.GetSynchronizationMasterInfo.Response.0.1"

/// Extent is the minimum amount of memory required to hold any serialized representation of any compatible
/// version of the data type; or, on other words, it is the the maximum possible size of received objects of this type.
/// The size is specified in bytes (rather than bits) because by definition, extent is an integer number of bytes long.
/// When allocating a deserialization (RX) buffer for this data type, it should be at least extent bytes large.
/// When allocating a serialization (TX) buffer, it is safe to use the size of the largest serialized representation
/// instead of the extent because it provides a tighter bound of the object size; it is safe because the concrete type
/// is always known during serialization (unlike deserialization). If not sure, use extent everywhere.
#define uavcan_time_GetSynchronizationMasterInfo_Response_0_1_EXTENT_BYTES_                    192UL
#define uavcan_time_GetSynchronizationMasterInfo_Response_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_ 7UL
static_assert(uavcan_time_GetSynchronizationMasterInfo_Response_0_1_EXTENT_BYTES_ >= uavcan_time_GetSynchronizationMasterInfo_Response_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
              "Internal constraint violation");

typedef struct
{
    /// saturated float32 error_variance
    float error_variance;

    /// uavcan.time.TimeSystem.0.1 time_system
    uavcan_time_TimeSystem_0_1 time_system;

    /// uavcan.time.TAIInfo.0.1 tai_info
    uavcan_time_TAIInfo_0_1 tai_info;
} uavcan_time_GetSynchronizationMasterInfo_Response_0_1;

/// Serialize an instance into the provided buffer.
/// The lifetime of the resulting serialized representation is independent of the original instance.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original object where possible.
///
/// @param obj      The object to serialize.
///
/// @param buffer   The destination buffer. There are no alignment requirements.
///                 @see uavcan_time_GetSynchronizationMasterInfo_Response_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the buffer in bytes.
///                                 Upon return this value will be updated with the size of the constructed serialized
///                                 representation (in bytes); this value is then to be passed over to the transport
///                                 layer. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t uavcan_time_GetSynchronizationMasterInfo_Response_0_1_serialize_(
    const uavcan_time_GetSynchronizationMasterInfo_Response_0_1* const obj, uint8_t* const buffer,  size_t* const inout_buffer_size_bytes)
{
    if ((obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
#ifndef uavcan_time_GetSynchronizationMasterInfo_Response_0_1_DISABLE_SERIALIZATION_BUFFER_CHECK_

    if ((8U * (size_t) capacity_bytes) < 56UL)
    {
        return -NUNAVUT_ERROR_SERIALIZATION_BUFFER_TOO_SMALL;
    }
#endif

    // Notice that fields that are not an integer number of bytes long may overrun the space allocated for them
    // in the serialization buffer up to the next byte boundary. This is by design and is guaranteed to be safe.
    size_t offset_bits = 0U;





    {   // saturated float32 error_variance
        // Saturation code not emitted -- assume the native representation of float32 is conformant.
        static_assert(NUNAVUT_PLATFORM_IEEE754_FLOAT, "Native IEEE754 binary32 required. TODO: relax constraint");
        const int8_t _err0_ = nunavutSetF32(&buffer[0], capacity_bytes, offset_bits, obj->error_variance);
        if (_err0_ < 0)
        {
            return _err0_;
        }
        offset_bits += 32U;
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

    {   // uavcan.time.TimeSystem.0.1 time_system
        size_t _size_bytes0_ = 1UL;  // Nested object (max) size, in bytes.
        int8_t _err2_ = uavcan_time_TimeSystem_0_1_serialize_(
            &obj->time_system, &buffer[offset_bits / 8U], &_size_bytes0_);
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

    {   // uavcan.time.TAIInfo.0.1 tai_info
        size_t _size_bytes1_ = 2UL;  // Nested object (max) size, in bytes.
        int8_t _err4_ = uavcan_time_TAIInfo_0_1_serialize_(
            &obj->tai_info, &buffer[offset_bits / 8U], &_size_bytes1_);
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
static inline int8_t uavcan_time_GetSynchronizationMasterInfo_Response_0_1_deserialize_(
    uavcan_time_GetSynchronizationMasterInfo_Response_0_1* const out_obj, const uint8_t* const buffer, size_t* const inout_buffer_size_bytes)
{
    if ((out_obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
    const size_t capacity_bits = capacity_bytes * (size_t) 8U;
    size_t offset_bits = 0U;





    // saturated float32 error_variance
    out_obj->error_variance = nunavutGetF32(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 32U;


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    // uavcan.time.TimeSystem.0.1 time_system
    {
        size_t _size_bytes2_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
        const int8_t _err6_ = uavcan_time_TimeSystem_0_1_deserialize_(
            &out_obj->time_system, &buffer[offset_bits / 8U], &_size_bytes2_);
        if (_err6_ < 0)
        {
            return _err6_;
        }
        offset_bits += _size_bytes2_ * 8U;  // Advance by the size of the nested serialized representation.
    }


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    // uavcan.time.TAIInfo.0.1 tai_info
    {
        size_t _size_bytes3_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
        const int8_t _err7_ = uavcan_time_TAIInfo_0_1_deserialize_(
            &out_obj->tai_info, &buffer[offset_bits / 8U], &_size_bytes3_);
        if (_err7_ < 0)
        {
            return _err7_;
        }
        offset_bits += _size_bytes3_ * 8U;  // Advance by the size of the nested serialized representation.
    }


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    *inout_buffer_size_bytes = (size_t) (nunavutChooseMin(offset_bits, capacity_bits) / 8U);


    return NUNAVUT_SUCCESS;
}

/// Initialize an instance to default values. Does nothing if @param out_obj is NULL.
/// This function intentionally leaves inactive elements uninitialized; for example, members of a variable-length
/// array beyond its length are left uninitialized; aliased union memory that is not used by the first union field
/// is left uninitialized, etc. If full zero-initialization is desired, just use memset(&obj, 0, sizeof(obj)).
static inline void uavcan_time_GetSynchronizationMasterInfo_Response_0_1_initialize_(uavcan_time_GetSynchronizationMasterInfo_Response_0_1* const out_obj)
{
    if (out_obj != NULL)
    {
        size_t size_bytes = 0;
        const uint8_t buf = 0;
        const int8_t err = uavcan_time_GetSynchronizationMasterInfo_Response_0_1_deserialize_(out_obj, &buf, &size_bytes);

        (void) err;
    }
}



#ifdef __cplusplus
}
#endif
#endif // UAVCAN_TIME_GET_SYNCHRONIZATION_MASTER_INFO_0_1_INCLUDED_
