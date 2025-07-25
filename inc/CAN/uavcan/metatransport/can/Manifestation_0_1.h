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
// Source file:   /home/hovergames/src/RDDRONE-BMS772/apps/nxp_bms/BMS_v1/public_regulated_data_types/uavcan/metatransport/can/Manifestation.0.1.uavcan
// Generated at:  2021-12-29 14:04:21.417297 UTC
// Is deprecated: yes
// Fixed port-ID: None
// Full name:     uavcan.metatransport.can.Manifestation
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

#ifndef UAVCAN_METATRANSPORT_CAN_MANIFESTATION_0_1_INCLUDED_
#define UAVCAN_METATRANSPORT_CAN_MANIFESTATION_0_1_INCLUDED_

#include <nunavut/support/serialization.h>
#include <uavcan/metatransport/can/DataClassic_0_1.h>
#include <uavcan/metatransport/can/DataFD_0_1.h>
#include <uavcan/metatransport/can/Error_0_1.h>
#include <uavcan/metatransport/can/RTR_0_1.h>
#include <stdint.h>
#include <stdlib.h>

static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_TARGET_ENDIANNESS == 1693710260,
              "/home/hovergames/src/RDDRONE-BMS772/apps/nxp_bms/BMS_v1/public_regulated_data_types/uavcan/metatransport/can/Manifestation.0.1.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_OMIT_FLOAT_SERIALIZATION_SUPPORT == 0,
              "/home/hovergames/src/RDDRONE-BMS772/apps/nxp_bms/BMS_v1/public_regulated_data_types/uavcan/metatransport/can/Manifestation.0.1.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_SERIALIZATION_ASSERTS == 0,
              "/home/hovergames/src/RDDRONE-BMS772/apps/nxp_bms/BMS_v1/public_regulated_data_types/uavcan/metatransport/can/Manifestation.0.1.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_OVERRIDE_VARIABLE_ARRAY_CAPACITY == 1,
              "/home/hovergames/src/RDDRONE-BMS772/apps/nxp_bms/BMS_v1/public_regulated_data_types/uavcan/metatransport/can/Manifestation.0.1.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );

#ifdef __cplusplus
extern "C" {
#endif

/// This type does not have a fixed port-ID. See https://forum.uavcan.org/t/choosing-message-and-service-ids/889
#define uavcan_metatransport_can_Manifestation_0_1_HAS_FIXED_PORT_ID_ false

#define uavcan_metatransport_can_Manifestation_0_1_FULL_NAME_             "uavcan.metatransport.can.Manifestation"
#define uavcan_metatransport_can_Manifestation_0_1_FULL_NAME_AND_VERSION_ "uavcan.metatransport.can.Manifestation.0.1"

/// Extent is the minimum amount of memory required to hold any serialized representation of any compatible
/// version of the data type; or, on other words, it is the the maximum possible size of received objects of this type.
/// The size is specified in bytes (rather than bits) because by definition, extent is an integer number of bytes long.
/// When allocating a deserialization (RX) buffer for this data type, it should be at least extent bytes large.
/// When allocating a serialization (TX) buffer, it is safe to use the size of the largest serialized representation
/// instead of the extent because it provides a tighter bound of the object size; it is safe because the concrete type
/// is always known during serialization (unlike deserialization). If not sure, use extent everywhere.
#define uavcan_metatransport_can_Manifestation_0_1_EXTENT_BYTES_                    71UL
#define uavcan_metatransport_can_Manifestation_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_ 71UL
static_assert(uavcan_metatransport_can_Manifestation_0_1_EXTENT_BYTES_ >= uavcan_metatransport_can_Manifestation_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
              "Internal constraint violation");

/// The number of fields in the union. Valid tag values range from zero to this value minus one, inclusive.
#define uavcan_metatransport_can_Manifestation_0_1_UNION_OPTION_COUNT_ 4U

typedef struct
{
    union  /// The union is placed first to ensure that the active element address equals the struct address.
    {
        /// uavcan.metatransport.can.Error.0.1 error
        uavcan_metatransport_can_Error_0_1 _error;

        /// uavcan.metatransport.can.DataFD.0.1 data_fd
        uavcan_metatransport_can_DataFD_0_1 data_fd;

        /// uavcan.metatransport.can.DataClassic.0.1 data_classic
        uavcan_metatransport_can_DataClassic_0_1 data_classic;

        /// uavcan.metatransport.can.RTR.0.1 remote_transmission_request
        uavcan_metatransport_can_RTR_0_1 remote_transmission_request;
    };
    uint8_t _tag_;
} uavcan_metatransport_can_Manifestation_0_1;

/// Serialize an instance into the provided buffer.
/// The lifetime of the resulting serialized representation is independent of the original instance.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original object where possible.
///
/// @param obj      The object to serialize.
///
/// @param buffer   The destination buffer. There are no alignment requirements.
///                 @see uavcan_metatransport_can_Manifestation_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the buffer in bytes.
///                                 Upon return this value will be updated with the size of the constructed serialized
///                                 representation (in bytes); this value is then to be passed over to the transport
///                                 layer. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t uavcan_metatransport_can_Manifestation_0_1_serialize_(
    const uavcan_metatransport_can_Manifestation_0_1* const obj, uint8_t* const buffer,  size_t* const inout_buffer_size_bytes)
{
    if ((obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
#ifndef uavcan_metatransport_can_Manifestation_0_1_DISABLE_SERIALIZATION_BUFFER_CHECK_

    if ((8U * (size_t) capacity_bytes) < 568UL)
    {
        return -NUNAVUT_ERROR_SERIALIZATION_BUFFER_TOO_SMALL;
    }
#endif

    // Notice that fields that are not an integer number of bytes long may overrun the space allocated for them
    // in the serialization buffer up to the next byte boundary. This is by design and is guaranteed to be safe.
    size_t offset_bits = 0U;

    {   // Union tag field: truncated uint8
        buffer[offset_bits / 8U] = (uint8_t)(obj->_tag_);  // C std, 6.3.1.3 Signed and unsigned integers
        offset_bits += 8U;
    }

    if (0U == obj->_tag_)  // uavcan.metatransport.can.Error.0.1 error
    {
        size_t _size_bytes0_ = 4UL;  // Nested object (max) size, in bytes.
        int8_t _err0_ = uavcan_metatransport_can_Error_0_1_serialize_(
            &obj->_error, &buffer[offset_bits / 8U], &_size_bytes0_);
        if (_err0_ < 0)
        {
            return _err0_;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        offset_bits += _size_bytes0_ * 8U;  // Advance by the size of the nested object.
    }
    else if (1U == obj->_tag_)  // uavcan.metatransport.can.DataFD.0.1 data_fd
    {
        size_t _size_bytes1_ = 70UL;  // Nested object (max) size, in bytes.
        int8_t _err1_ = uavcan_metatransport_can_DataFD_0_1_serialize_(
            &obj->data_fd, &buffer[offset_bits / 8U], &_size_bytes1_);
        if (_err1_ < 0)
        {
            return _err1_;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        offset_bits += _size_bytes1_ * 8U;  // Advance by the size of the nested object.
    }
    else if (2U == obj->_tag_)  // uavcan.metatransport.can.DataClassic.0.1 data_classic
    {
        size_t _size_bytes2_ = 14UL;  // Nested object (max) size, in bytes.
        int8_t _err2_ = uavcan_metatransport_can_DataClassic_0_1_serialize_(
            &obj->data_classic, &buffer[offset_bits / 8U], &_size_bytes2_);
        if (_err2_ < 0)
        {
            return _err2_;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        offset_bits += _size_bytes2_ * 8U;  // Advance by the size of the nested object.
    }
    else if (3U == obj->_tag_)  // uavcan.metatransport.can.RTR.0.1 remote_transmission_request
    {
        size_t _size_bytes3_ = 5UL;  // Nested object (max) size, in bytes.
        int8_t _err3_ = uavcan_metatransport_can_RTR_0_1_serialize_(
            &obj->remote_transmission_request, &buffer[offset_bits / 8U], &_size_bytes3_);
        if (_err3_ < 0)
        {
            return _err3_;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        offset_bits += _size_bytes3_ * 8U;  // Advance by the size of the nested object.
    }
    else
    {
        return -NUNAVUT_ERROR_REPRESENTATION_BAD_UNION_TAG;
    }

    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad0_ = (uint8_t)(8U - offset_bits % 8U);
        const int8_t _err4_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad0_);  // Optimize?
        if (_err4_ < 0)
        {
            return _err4_;
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
static inline int8_t uavcan_metatransport_can_Manifestation_0_1_deserialize_(
    uavcan_metatransport_can_Manifestation_0_1* const out_obj, const uint8_t* const buffer, size_t* const inout_buffer_size_bytes)
{
    if ((out_obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
    const size_t capacity_bits = capacity_bytes * (size_t) 8U;
    size_t offset_bits = 0U;

    // Union tag field: truncated uint8
    if ((offset_bits + 8U) <= capacity_bits)
    {
        out_obj->_tag_ = buffer[offset_bits / 8U] & 255U;
    }
    else
    {
        out_obj->_tag_ = 0U;
    }
    offset_bits += 8U;

    if (0U == out_obj->_tag_)  // uavcan.metatransport.can.Error.0.1 error
    {
        {
            size_t _size_bytes4_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
            const int8_t _err5_ = uavcan_metatransport_can_Error_0_1_deserialize_(
                &out_obj->_error, &buffer[offset_bits / 8U], &_size_bytes4_);
            if (_err5_ < 0)
            {
                return _err5_;
            }
            offset_bits += _size_bytes4_ * 8U;  // Advance by the size of the nested serialized representation.
        }
    }
    else if (1U == out_obj->_tag_)  // uavcan.metatransport.can.DataFD.0.1 data_fd
    {
        {
            size_t _size_bytes5_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
            const int8_t _err6_ = uavcan_metatransport_can_DataFD_0_1_deserialize_(
                &out_obj->data_fd, &buffer[offset_bits / 8U], &_size_bytes5_);
            if (_err6_ < 0)
            {
                return _err6_;
            }
            offset_bits += _size_bytes5_ * 8U;  // Advance by the size of the nested serialized representation.
        }
    }
    else if (2U == out_obj->_tag_)  // uavcan.metatransport.can.DataClassic.0.1 data_classic
    {
        {
            size_t _size_bytes6_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
            const int8_t _err7_ = uavcan_metatransport_can_DataClassic_0_1_deserialize_(
                &out_obj->data_classic, &buffer[offset_bits / 8U], &_size_bytes6_);
            if (_err7_ < 0)
            {
                return _err7_;
            }
            offset_bits += _size_bytes6_ * 8U;  // Advance by the size of the nested serialized representation.
        }
    }
    else if (3U == out_obj->_tag_)  // uavcan.metatransport.can.RTR.0.1 remote_transmission_request
    {
        {
            size_t _size_bytes7_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
            const int8_t _err8_ = uavcan_metatransport_can_RTR_0_1_deserialize_(
                &out_obj->remote_transmission_request, &buffer[offset_bits / 8U], &_size_bytes7_);
            if (_err8_ < 0)
            {
                return _err8_;
            }
            offset_bits += _size_bytes7_ * 8U;  // Advance by the size of the nested serialized representation.
        }
    }
    else
    {
        return -NUNAVUT_ERROR_REPRESENTATION_BAD_UNION_TAG;
    }

    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    *inout_buffer_size_bytes = (size_t) (nunavutChooseMin(offset_bits, capacity_bits) / 8U);


    return NUNAVUT_SUCCESS;
}

/// Initialize an instance to default values. Does nothing if @param out_obj is NULL.
/// This function intentionally leaves inactive elements uninitialized; for example, members of a variable-length
/// array beyond its length are left uninitialized; aliased union memory that is not used by the first union field
/// is left uninitialized, etc. If full zero-initialization is desired, just use memset(&obj, 0, sizeof(obj)).
static inline void uavcan_metatransport_can_Manifestation_0_1_initialize_(uavcan_metatransport_can_Manifestation_0_1* const out_obj)
{
    if (out_obj != NULL)
    {
        size_t size_bytes = 0;
        const uint8_t buf = 0;
        const int8_t err = uavcan_metatransport_can_Manifestation_0_1_deserialize_(out_obj, &buf, &size_bytes);

        (void) err;
    }
}



/// Mark option "error" active without initializing it. Does nothing if @param obj is NULL.
static inline void uavcan_metatransport_can_Manifestation_0_1_select_error_(uavcan_metatransport_can_Manifestation_0_1* const obj)
{
    if (obj != NULL)
    {
        obj->_tag_ = 0;
    }
}

/// Check if option "error" is active. Returns false if @param obj is NULL.
static inline bool uavcan_metatransport_can_Manifestation_0_1_is_error_(const uavcan_metatransport_can_Manifestation_0_1* const obj)
{
    return ((obj != NULL) && (obj->_tag_ == 0));
}

/// Mark option "data_fd" active without initializing it. Does nothing if @param obj is NULL.
static inline void uavcan_metatransport_can_Manifestation_0_1_select_data_fd_(uavcan_metatransport_can_Manifestation_0_1* const obj)
{
    if (obj != NULL)
    {
        obj->_tag_ = 1;
    }
}

/// Check if option "data_fd" is active. Returns false if @param obj is NULL.
static inline bool uavcan_metatransport_can_Manifestation_0_1_is_data_fd_(const uavcan_metatransport_can_Manifestation_0_1* const obj)
{
    return ((obj != NULL) && (obj->_tag_ == 1));
}

/// Mark option "data_classic" active without initializing it. Does nothing if @param obj is NULL.
static inline void uavcan_metatransport_can_Manifestation_0_1_select_data_classic_(uavcan_metatransport_can_Manifestation_0_1* const obj)
{
    if (obj != NULL)
    {
        obj->_tag_ = 2;
    }
}

/// Check if option "data_classic" is active. Returns false if @param obj is NULL.
static inline bool uavcan_metatransport_can_Manifestation_0_1_is_data_classic_(const uavcan_metatransport_can_Manifestation_0_1* const obj)
{
    return ((obj != NULL) && (obj->_tag_ == 2));
}

/// Mark option "remote_transmission_request" active without initializing it. Does nothing if @param obj is NULL.
static inline void uavcan_metatransport_can_Manifestation_0_1_select_remote_transmission_request_(uavcan_metatransport_can_Manifestation_0_1* const obj)
{
    if (obj != NULL)
    {
        obj->_tag_ = 3;
    }
}

/// Check if option "remote_transmission_request" is active. Returns false if @param obj is NULL.
static inline bool uavcan_metatransport_can_Manifestation_0_1_is_remote_transmission_request_(const uavcan_metatransport_can_Manifestation_0_1* const obj)
{
    return ((obj != NULL) && (obj->_tag_ == 3));
}

#ifdef __cplusplus
}
#endif
#endif // UAVCAN_METATRANSPORT_CAN_MANIFESTATION_0_1_INCLUDED_

