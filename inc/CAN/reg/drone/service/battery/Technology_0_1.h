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
// Source file:   /home/hovergames/src/RDDRONE-BMS772/apps/nxp_bms/BMS_v1/public_regulated_data_types/reg/drone/service/battery/Technology.0.1.uavcan
// Generated at:  2021-12-29 14:03:55.141623 UTC
// Is deprecated: no
// Fixed port-ID: None
// Full name:     reg.drone.service.battery.Technology
// Version:       0.1
//
// Language Options
//     target_endianness:  any
//     omit_float_serialization_support:  False
//     enable_serialization_asserts:  False
//     enable_override_variable_array_capacity:  True

#ifndef REG_DRONE_SERVICE_BATTERY_TECHNOLOGY_0_1_INCLUDED_
#define REG_DRONE_SERVICE_BATTERY_TECHNOLOGY_0_1_INCLUDED_

#include <nunavut/support/serialization.h>
#include <stdint.h>
#include <stdlib.h>

static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_TARGET_ENDIANNESS == 1693710260,
              "/home/hovergames/src/RDDRONE-BMS772/apps/nxp_bms/BMS_v1/public_regulated_data_types/reg/drone/service/battery/Technology.0.1.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_OMIT_FLOAT_SERIALIZATION_SUPPORT == 0,
              "/home/hovergames/src/RDDRONE-BMS772/apps/nxp_bms/BMS_v1/public_regulated_data_types/reg/drone/service/battery/Technology.0.1.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_SERIALIZATION_ASSERTS == 0,
              "/home/hovergames/src/RDDRONE-BMS772/apps/nxp_bms/BMS_v1/public_regulated_data_types/reg/drone/service/battery/Technology.0.1.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_OVERRIDE_VARIABLE_ARRAY_CAPACITY == 1,
              "/home/hovergames/src/RDDRONE-BMS772/apps/nxp_bms/BMS_v1/public_regulated_data_types/reg/drone/service/battery/Technology.0.1.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );

#ifdef __cplusplus
extern "C" {
#endif

/// This type does not have a fixed port-ID. See https://forum.uavcan.org/t/choosing-message-and-service-ids/889
#define reg_drone_service_battery_Technology_0_1_HAS_FIXED_PORT_ID_ false

#define reg_drone_service_battery_Technology_0_1_FULL_NAME_             "reg.drone.service.battery.Technology"
#define reg_drone_service_battery_Technology_0_1_FULL_NAME_AND_VERSION_ "reg.drone.service.battery.Technology.0.1"

/// Extent is the minimum amount of memory required to hold any serialized representation of any compatible
/// version of the data type; or, on other words, it is the the maximum possible size of received objects of this type.
/// The size is specified in bytes (rather than bits) because by definition, extent is an integer number of bytes long.
/// When allocating a deserialization (RX) buffer for this data type, it should be at least extent bytes large.
/// When allocating a serialization (TX) buffer, it is safe to use the size of the largest serialized representation
/// instead of the extent because it provides a tighter bound of the object size; it is safe because the concrete type
/// is always known during serialization (unlike deserialization). If not sure, use extent everywhere.
#define reg_drone_service_battery_Technology_0_1_EXTENT_BYTES_                    1UL
#define reg_drone_service_battery_Technology_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_ 1UL
static_assert(reg_drone_service_battery_Technology_0_1_EXTENT_BYTES_ >= reg_drone_service_battery_Technology_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
              "Internal constraint violation");

/// saturated uint8 OTHER = 0
#define reg_drone_service_battery_Technology_0_1_OTHER (0U)
/// saturated uint8 LI_SOCL2 = 10
#define reg_drone_service_battery_Technology_0_1_LI_SOCL2 (10U)
/// saturated uint8 LI_BCX = 11
#define reg_drone_service_battery_Technology_0_1_LI_BCX (11U)
/// saturated uint8 LI_MNO2 = 12
#define reg_drone_service_battery_Technology_0_1_LI_MNO2 (12U)
/// saturated uint8 ZN_O2 = 20
#define reg_drone_service_battery_Technology_0_1_ZN_O2 (20U)
/// saturated uint8 AL_O2 = 21
#define reg_drone_service_battery_Technology_0_1_AL_O2 (21U)
/// saturated uint8 ZN_MNO2_NH4CL = 30
#define reg_drone_service_battery_Technology_0_1_ZN_MNO2_NH4CL (30U)
/// saturated uint8 ZN_MNO2_ZNCL2 = 31
#define reg_drone_service_battery_Technology_0_1_ZN_MNO2_ZNCL2 (31U)
/// saturated uint8 ZN_MNO2_KOH = 32
#define reg_drone_service_battery_Technology_0_1_ZN_MNO2_KOH (32U)
/// saturated uint8 LI_LCO = 100
#define reg_drone_service_battery_Technology_0_1_LI_LCO (100U)
/// saturated uint8 LI_LFP = 101
#define reg_drone_service_battery_Technology_0_1_LI_LFP (101U)
/// saturated uint8 LI_NMC = 102
#define reg_drone_service_battery_Technology_0_1_LI_NMC (102U)
/// saturated uint8 LI_NCA = 103
#define reg_drone_service_battery_Technology_0_1_LI_NCA (103U)
/// saturated uint8 LI_LMO = 104
#define reg_drone_service_battery_Technology_0_1_LI_LMO (104U)
/// saturated uint8 LI_S = 105
#define reg_drone_service_battery_Technology_0_1_LI_S (105U)
/// saturated uint8 LI_LCO_POUCH = 110
#define reg_drone_service_battery_Technology_0_1_LI_LCO_POUCH (110U)
/// saturated uint8 LI_LFP_POUCH = 111
#define reg_drone_service_battery_Technology_0_1_LI_LFP_POUCH (111U)
/// saturated uint8 NI_MH = 120
#define reg_drone_service_battery_Technology_0_1_NI_MH (120U)
/// saturated uint8 NI_CD = 121
#define reg_drone_service_battery_Technology_0_1_NI_CD (121U)
/// saturated uint8 NI_ZN = 122
#define reg_drone_service_battery_Technology_0_1_NI_ZN (122U)
/// saturated uint8 NI_FE = 123
#define reg_drone_service_battery_Technology_0_1_NI_FE (123U)
/// saturated uint8 PB_AC = 130
#define reg_drone_service_battery_Technology_0_1_PB_AC (130U)
/// saturated uint8 PB_AC_SEALED = 131
#define reg_drone_service_battery_Technology_0_1_PB_AC_SEALED (131U)
/// saturated uint8 EDLC = 200
#define reg_drone_service_battery_Technology_0_1_EDLC (200U)

typedef struct
{
    /// saturated uint8 value
    uint8_t value;
} reg_drone_service_battery_Technology_0_1;

/// Serialize an instance into the provided buffer.
/// The lifetime of the resulting serialized representation is independent of the original instance.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original object where possible.
///
/// @param obj      The object to serialize.
///
/// @param buffer   The destination buffer. There are no alignment requirements.
///                 @see reg_drone_service_battery_Technology_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the buffer in bytes.
///                                 Upon return this value will be updated with the size of the constructed serialized
///                                 representation (in bytes); this value is then to be passed over to the transport
///                                 layer. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t reg_drone_service_battery_Technology_0_1_serialize_(
    const reg_drone_service_battery_Technology_0_1* const obj, uint8_t* const buffer,  size_t* const inout_buffer_size_bytes)
{
    if ((obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
#ifndef reg_drone_service_battery_Technology_0_1_DISABLE_SERIALIZATION_BUFFER_CHECK_

    if ((8U * (size_t) capacity_bytes) < 8UL)
    {
        return -NUNAVUT_ERROR_SERIALIZATION_BUFFER_TOO_SMALL;
    }
#endif

    // Notice that fields that are not an integer number of bytes long may overrun the space allocated for them
    // in the serialization buffer up to the next byte boundary. This is by design and is guaranteed to be safe.
    size_t offset_bits = 0U;





    {   // saturated uint8 value
        // Saturation code not emitted -- native representation matches the serialized representation.
        buffer[offset_bits / 8U] = (uint8_t)(obj->value);  // C std, 6.3.1.3 Signed and unsigned integers
        offset_bits += 8U;
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
static inline int8_t reg_drone_service_battery_Technology_0_1_deserialize_(
    reg_drone_service_battery_Technology_0_1* const out_obj, const uint8_t* const buffer, size_t* const inout_buffer_size_bytes)
{
    if ((out_obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
    const size_t capacity_bits = capacity_bytes * (size_t) 8U;
    size_t offset_bits = 0U;





    // saturated uint8 value
    if ((offset_bits + 8U) <= capacity_bits)
    {
        out_obj->value = buffer[offset_bits / 8U] & 255U;
    }
    else
    {
        out_obj->value = 0U;
    }
    offset_bits += 8U;


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    *inout_buffer_size_bytes = (size_t) (nunavutChooseMin(offset_bits, capacity_bits) / 8U);


    return NUNAVUT_SUCCESS;
}

/// Initialize an instance to default values. Does nothing if @param out_obj is NULL.
/// This function intentionally leaves inactive elements uninitialized; for example, members of a variable-length
/// array beyond its length are left uninitialized; aliased union memory that is not used by the first union field
/// is left uninitialized, etc. If full zero-initialization is desired, just use memset(&obj, 0, sizeof(obj)).
static inline void reg_drone_service_battery_Technology_0_1_initialize_(reg_drone_service_battery_Technology_0_1* const out_obj)
{
    if (out_obj != NULL)
    {
        size_t size_bytes = 0;
        const uint8_t buf = 0;
        const int8_t err = reg_drone_service_battery_Technology_0_1_deserialize_(out_obj, &buf, &size_bytes);

        (void) err;
    }
}



#ifdef __cplusplus
}
#endif
#endif // REG_DRONE_SERVICE_BATTERY_TECHNOLOGY_0_1_INCLUDED_
