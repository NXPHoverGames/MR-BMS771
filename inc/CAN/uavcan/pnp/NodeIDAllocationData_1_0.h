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
// Source file:   /home/hovergames/src/RDDRONE-BMS772/apps/nxp_bms/BMS_v1/public_regulated_data_types/uavcan/pnp/8166.NodeIDAllocationData.1.0.uavcan
// Generated at:  2021-12-29 14:04:17.326123 UTC
// Is deprecated: no
// Fixed port-ID: 8166
// Full name:     uavcan.pnp.NodeIDAllocationData
// Version:       1.0
//
// Language Options
//     target_endianness:  any
//     omit_float_serialization_support:  False
//     enable_serialization_asserts:  False
//     enable_override_variable_array_capacity:  True

#ifndef UAVCAN_PNP_NODE_ID_ALLOCATION_DATA_1_0_INCLUDED_
#define UAVCAN_PNP_NODE_ID_ALLOCATION_DATA_1_0_INCLUDED_

#include <nunavut/support/serialization.h>
#include <uavcan/node/ID_1_0.h>
#include <stdint.h>
#include <stdlib.h>

static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_TARGET_ENDIANNESS == 1693710260,
              "/home/hovergames/src/RDDRONE-BMS772/apps/nxp_bms/BMS_v1/public_regulated_data_types/uavcan/pnp/8166.NodeIDAllocationData.1.0.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_OMIT_FLOAT_SERIALIZATION_SUPPORT == 0,
              "/home/hovergames/src/RDDRONE-BMS772/apps/nxp_bms/BMS_v1/public_regulated_data_types/uavcan/pnp/8166.NodeIDAllocationData.1.0.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_SERIALIZATION_ASSERTS == 0,
              "/home/hovergames/src/RDDRONE-BMS772/apps/nxp_bms/BMS_v1/public_regulated_data_types/uavcan/pnp/8166.NodeIDAllocationData.1.0.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_OVERRIDE_VARIABLE_ARRAY_CAPACITY == 1,
              "/home/hovergames/src/RDDRONE-BMS772/apps/nxp_bms/BMS_v1/public_regulated_data_types/uavcan/pnp/8166.NodeIDAllocationData.1.0.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );

#ifdef __cplusplus
extern "C" {
#endif

#define uavcan_pnp_NodeIDAllocationData_1_0_HAS_FIXED_PORT_ID_ true
#define uavcan_pnp_NodeIDAllocationData_1_0_FIXED_PORT_ID_     8166U

#define uavcan_pnp_NodeIDAllocationData_1_0_FULL_NAME_             "uavcan.pnp.NodeIDAllocationData"
#define uavcan_pnp_NodeIDAllocationData_1_0_FULL_NAME_AND_VERSION_ "uavcan.pnp.NodeIDAllocationData.1.0"

/// Extent is the minimum amount of memory required to hold any serialized representation of any compatible
/// version of the data type; or, on other words, it is the the maximum possible size of received objects of this type.
/// The size is specified in bytes (rather than bits) because by definition, extent is an integer number of bytes long.
/// When allocating a deserialization (RX) buffer for this data type, it should be at least extent bytes large.
/// When allocating a serialization (TX) buffer, it is safe to use the size of the largest serialized representation
/// instead of the extent because it provides a tighter bound of the object size; it is safe because the concrete type
/// is always known during serialization (unlike deserialization). If not sure, use extent everywhere.
#define uavcan_pnp_NodeIDAllocationData_1_0_EXTENT_BYTES_                    9UL
#define uavcan_pnp_NodeIDAllocationData_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_ 9UL
static_assert(uavcan_pnp_NodeIDAllocationData_1_0_EXTENT_BYTES_ >= uavcan_pnp_NodeIDAllocationData_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
              "Internal constraint violation");

/// Array metadata for: uavcan.node.ID.1.0[<=1] allocated_node_id
#ifndef uavcan_pnp_NodeIDAllocationData_1_0_allocated_node_id_ARRAY_CAPACITY_
#define uavcan_pnp_NodeIDAllocationData_1_0_allocated_node_id_ARRAY_CAPACITY_           1U
#elif !defined(uavcan_pnp_NodeIDAllocationData_1_0_DISABLE_SERIALIZATION_BUFFER_CHECK_)
#  define uavcan_pnp_NodeIDAllocationData_1_0_DISABLE_SERIALIZATION_BUFFER_CHECK_
#endif
#if uavcan_pnp_NodeIDAllocationData_1_0_allocated_node_id_ARRAY_CAPACITY_ > 1U
#  error uavcan_pnp_NodeIDAllocationData_1_0_allocated_node_id_ARRAY_CAPACITY_ > 1U
#endif
#define uavcan_pnp_NodeIDAllocationData_1_0_allocated_node_id_ARRAY_IS_VARIABLE_LENGTH_ true

typedef struct
{
    /// truncated uint48 unique_id_hash
    uint64_t unique_id_hash;

    /// uavcan.node.ID.1.0[<=1] allocated_node_id
    struct  /// Array address equivalence guarantee: &elements[0] == &allocated_node_id
    {
        uavcan_node_ID_1_0 elements[uavcan_pnp_NodeIDAllocationData_1_0_allocated_node_id_ARRAY_CAPACITY_];
        size_t count;
    } allocated_node_id;
} uavcan_pnp_NodeIDAllocationData_1_0;

/// Serialize an instance into the provided buffer.
/// The lifetime of the resulting serialized representation is independent of the original instance.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original object where possible.
///
/// @param obj      The object to serialize.
///
/// @param buffer   The destination buffer. There are no alignment requirements.
///                 @see uavcan_pnp_NodeIDAllocationData_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the buffer in bytes.
///                                 Upon return this value will be updated with the size of the constructed serialized
///                                 representation (in bytes); this value is then to be passed over to the transport
///                                 layer. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t uavcan_pnp_NodeIDAllocationData_1_0_serialize_(
    const uavcan_pnp_NodeIDAllocationData_1_0* const obj, uint8_t* const buffer,  size_t* const inout_buffer_size_bytes)
{
    if ((obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
#ifndef uavcan_pnp_NodeIDAllocationData_1_0_DISABLE_SERIALIZATION_BUFFER_CHECK_

    if ((8U * (size_t) capacity_bytes) < 72UL)
    {
        return -NUNAVUT_ERROR_SERIALIZATION_BUFFER_TOO_SMALL;
    }
#endif

    // Notice that fields that are not an integer number of bytes long may overrun the space allocated for them
    // in the serialization buffer up to the next byte boundary. This is by design and is guaranteed to be safe.
    size_t offset_bits = 0U;





    {   // truncated uint48 unique_id_hash
        const int8_t _err0_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, obj->unique_id_hash, 48U);
        if (_err0_ < 0)
        {
            return _err0_;
        }
        offset_bits += 48U;
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

    {   // uavcan.node.ID.1.0[<=1] allocated_node_id
        if (obj->allocated_node_id.count > 1)
        {
            return -NUNAVUT_ERROR_REPRESENTATION_BAD_ARRAY_LENGTH;
        }
        // Array length prefix: truncated uint8
        buffer[offset_bits / 8U] = (uint8_t)(obj->allocated_node_id.count);  // C std, 6.3.1.3 Signed and unsigned integers
        offset_bits += 8U;
        for (size_t _index0_ = 0U; _index0_ < obj->allocated_node_id.count; ++_index0_)
        {
            size_t _size_bytes0_ = 2UL;  // Nested object (max) size, in bytes.
            int8_t _err2_ = uavcan_node_ID_1_0_serialize_(
                &obj->allocated_node_id.elements[_index0_], &buffer[offset_bits / 8U], &_size_bytes0_);
            if (_err2_ < 0)
            {
                return _err2_;
            }
            // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
            offset_bits += _size_bytes0_ * 8U;  // Advance by the size of the nested object.
        }
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
static inline int8_t uavcan_pnp_NodeIDAllocationData_1_0_deserialize_(
    uavcan_pnp_NodeIDAllocationData_1_0* const out_obj, const uint8_t* const buffer, size_t* const inout_buffer_size_bytes)
{
    if ((out_obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
    const size_t capacity_bits = capacity_bytes * (size_t) 8U;
    size_t offset_bits = 0U;





    // truncated uint48 unique_id_hash
    out_obj->unique_id_hash = nunavutGetU64(&buffer[0], capacity_bytes, offset_bits, 48);
    offset_bits += 48U;


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    // uavcan.node.ID.1.0[<=1] allocated_node_id
    // Array length prefix: truncated uint8
    if ((offset_bits + 8U) <= capacity_bits)
    {
        out_obj->allocated_node_id.count = buffer[offset_bits / 8U] & 255U;
    }
    else
    {
        out_obj->allocated_node_id.count = 0U;
    }
    offset_bits += 8U;
    if (out_obj->allocated_node_id.count > 1U)
    {
        return -NUNAVUT_ERROR_REPRESENTATION_BAD_ARRAY_LENGTH;
    }
    for (size_t _index1_ = 0U; _index1_ < out_obj->allocated_node_id.count; ++_index1_)
    {
        {
            size_t _size_bytes1_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
            const int8_t _err4_ = uavcan_node_ID_1_0_deserialize_(
                &out_obj->allocated_node_id.elements[_index1_], &buffer[offset_bits / 8U], &_size_bytes1_);
            if (_err4_ < 0)
            {
                return _err4_;
            }
            offset_bits += _size_bytes1_ * 8U;  // Advance by the size of the nested serialized representation.
        }
    }


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    *inout_buffer_size_bytes = (size_t) (nunavutChooseMin(offset_bits, capacity_bits) / 8U);


    return NUNAVUT_SUCCESS;
}

/// Initialize an instance to default values. Does nothing if @param out_obj is NULL.
/// This function intentionally leaves inactive elements uninitialized; for example, members of a variable-length
/// array beyond its length are left uninitialized; aliased union memory that is not used by the first union field
/// is left uninitialized, etc. If full zero-initialization is desired, just use memset(&obj, 0, sizeof(obj)).
static inline void uavcan_pnp_NodeIDAllocationData_1_0_initialize_(uavcan_pnp_NodeIDAllocationData_1_0* const out_obj)
{
    if (out_obj != NULL)
    {
        size_t size_bytes = 0;
        const uint8_t buf = 0;
        const int8_t err = uavcan_pnp_NodeIDAllocationData_1_0_deserialize_(out_obj, &buf, &size_bytes);

        (void) err;
    }
}



#ifdef __cplusplus
}
#endif
#endif // UAVCAN_PNP_NODE_ID_ALLOCATION_DATA_1_0_INCLUDED_
