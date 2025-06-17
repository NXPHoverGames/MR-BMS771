/*
 * Copyright 2019 - 2021, 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ** ###################################################################
 **     Filename    : define.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Compiler    : GNU C Compiler
 **     Date/Time   : 2020-04-01, 11:00, # CodeGen: 0
 **     Abstract    :
 **         Define list.
 **         This file contains all defined values.
 **
 **         N.B.: all '_def' values are default values for global variables.
 **
 ** ###################################################################*/
/*!
 ** @file define.h
 **
 ** @version 01.00
 **
 ** @brief
 **         Define list.
 **         This file contains all defined values.
 **
 ** @b Notes \n
 **         All '_def' values are default values for global variables.
 */

#ifndef BCC_DEFINE_H_
#define BCC_DEFINE_H_


/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "bcc_diagnostics.h"
/*******************************************************************************
 * Communication system
 ******************************************************************************/


/*******************************************************************************
 * Enumerations
 ******************************************************************************/

/* BCC CT filters configuration */

/*!
* @brief CT Filters Components.
*
* Values of external components required for OV & UV functional verification
* and CTx open detection functional verification.
*/
typedef struct
{
    uint32_t rLpf1;        /*!< R_LPF-1 low-pass filter resistor in [Ohm]. */
    uint32_t rLpf2;        /*!< R_LPF-2 low-pass filter resistor in [Ohm]. */
    uint32_t cLpf;         /*!< C_LPF capacitance in [nF]. */
    uint32_t cIn;          /*!< C_IN capacitance in [nF]. */
} ct_filter_t;


/* BCC I-sense configuration */

/*!
* @brief ISENSE Filters Components.
*
* Values of external components required for current measurements and related
* diagnostics.
*/
typedef struct
{
    uint16_t rLpfi;        /*!< R_LPFI resistor (between C_HFI and C_LPFI) in
                                [Ohm]. */
    uint32_t cD;           /*!< C_D capacitor (between ISENSE+ and ISENSE-) in
                                [nF]. */
    uint16_t cLpfi;        /*!< C_LPFI capacitor used to cut off common mode
                                disturbances on ISENSE+/- [nF]. */
    uint32_t rShunt;       /*!< Shunt resistor for ISENSE in [uOhm]. */
    uint16_t iMax;         /*!< Maximum shunt resistor current in [mA]. */
} isense_filter_t;

/* BCC NTC configuration */

/*!
* @brief NTC Configuration.
*
* The device has seven GPIOs which enable temperature measurement.
* NTC thermistor and fixed resistor are external components and must be set
* by the user. These values are used to calculate temperature. Beta parameter
* equation is used to calculate temperature. GPIO port of BCC device must be
* configured as Analog Input to measure temperature.
* This configuration is common for all GPIO ports and all devices (in case of
* daisy chain).
*/
typedef struct
{
    uint32_t beta;         /*!< Beta parameter of NTC thermistor in [K].
                                Admissible range is from 1 to 1000000. */
    uint32_t rntc;         /*!< R_NTC - NTC fixed resistance in [Ohm].
                                Admissible range is from 1 to 1000000. */
    uint32_t refRes;       /*!< NTC Reference Resistance in [Ohm].
                                Admissible range is from 1 to 1000000. */
    uint8_t refTemp;       /*!< NTC Reference Temperature in degrees [Celsius].
                                Admissible range is from 0 to 200. */
} ntc_config_t;



/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* BCC_DEFINE_H_ */
