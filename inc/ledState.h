/****************************************************************************
 * nxp_bms/BMS_v1/inc/ledState.h
 *
 * Copyright 2020-2021, 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ** ###################################################################
 **     Filename    : ledState.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date        : 2020-04-03
 **     Abstract    :
 **        ledState module.
 **        This module contains all functions needed for shared ledState
 **
 ** ###################################################################*/
/*!
 ** @file ledState.h
 **
 ** @version 01.00
 **
 ** @brief
 **        ledState module. this module contains the functions to control the LED
 **
 */
#ifndef LEDSTATE_H_
#define LEDSTATE_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "BMS_data_types.h"

/*******************************************************************************
 * defines
 ******************************************************************************/
#define NORMAL_BLINK_PERIOD_MS          2000    // [ms]
#define LED_BLINK_ON                    NORMAL_BLINK_PERIOD_MS  // use with the ledState_setLedColor function to blink the LED
#define LED_BLINK_OFF                   0       // use with the ledState_setLedColor to not blink the LED

#ifdef LED_INDICATION_DEFINED_WAIT
#define LED_DEFINED_WAIT                2       //<! amount of sec. to wait after the indication
#else
#define LED_PERIOD_INDICATION_S         6       // the period of the state of charge indication
#endif

#define LED_BLINK_TIME_S                1       // the blinking time in s (the time the LED is on and off)
#define LED_BLINK_TIME_MS               500     // the blinking time in ms (the time the LED is on and off), green

#define LED_4_BLINK_PERCENTAGE          80      // the percentage from which (upwards) the BMS will blink 4 times
#define LED_3_BLINK_PERCENTAGE          60      // the percentage from which (upwards) the BMS will blink 4 times
#define LED_2_BLINK_PERCENTAGE          40      // the percentage from which (upwards) the BMS will blink 4 times
#define LED_1_BLINK_PERCENTAGE          0       // the percentage from which (upwards) the BMS will blink 4 times


#ifndef UINT8_MAX
#define UINT8_MAX                       255
#endif

/*******************************************************************************
 * types
 ******************************************************************************/

//! @brief  this enum can be used to set certain LED colors
typedef enum {
    OFF = 0, RED = 1, GREEN = 2, YELLOW = 3, BLUE = 4, MAGENTA = 5, CYAN = 6, WHITE = 7
}LEDColor_t;

/*******************************************************************************
 * public functions
 ******************************************************************************/
/*!
 * @brief   this function initialized the RGB LED and it will set the LED to green blinking
 *
 * @param   startColor the starting color from the LEDColor_t enum
 * @param   skipSelfTest if this is true it will skip the self-test
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error:
 * @example if(ledState_initialize(RED, false))
 *          {
 *              // do something with the error
 *          }
 */
int ledState_initialize(LEDColor_t startColor, bool skipSelfTest);

/*!
 * @brief   This function will set the color of the LED that can be entered from the LEDColor_t enum.
 *          This function can also be used to set the LED off. using the OFF enum.
 *          Using this function the blink period, alternating color can be configured if blinkPeriodms is true (!= 0)
 *          ledState_initialize(); should be called before using this function
 *          This function can be called from multiple threads
 *
 * @param   newColor The new color is should have from the LEDColor_t enum, it will change imidiatly
 * @param   newAltColor The alternating color if blinkPeriodms is true. This could be OFF to blink or an other color
 * @param   blinkPeriodms The blinkperiod of the LED, in ms. If 0 blinking is disabled
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error:
 * @example if(ledState_setLedColor(GREEN, OFF, 2000)) // led from GREEN-OFF will take 2s (period)
 *          {
 *              // do something with the error
 *          }
 */
int ledState_setLedColor(LEDColor_t newColor, LEDColor_t newAltColor, uint16_t blinkPeriodms);

/*!
 * @brief   This function will used to set the blinking indication, the number of blinks
 *
 * @param   newStateOfCharge the new state of charge, value of 0-100.
 *
 * @return  the new state indication number of blinks, UINT8_MAX if error
 */
uint8_t ledState_calcStateIndication(uint8_t newStateOfCharge);

/*!
 * @brief   This function will used to get the blinking indication, the number of blinks
 *
 * @param   none
 *
 * @return  the new state indication number of blinks, UINT8_MAX if error
 */
uint8_t ledState_getStateIndication(void);

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* LEDSTATE_H_ */
