/****************************************************************************
 * nxp_bms/BMS_v1/inc/gpio.h
 *
 * Copyright 2020-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
/*!
 ** @file gpio.h
 **
 ** @version 01.00
 **
 ** @brief
 **        gpio module. this module contains the functions to control the GPIO pins
 **
 */
#ifndef GPIO_H_
#define GPIO_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <sched.h>

#include "BMS_data_limits.h"
/*******************************************************************************
 * defines
 ******************************************************************************/

#if (BOARD_NAME == RDDRONE_BMS772_BOARD)
#   define START_OUTPUT_PIN_NUMBER     0
#   define START_INTERRRUPT_PIN_NUMBER 5
#   define END_INTERRUPT_PIN_NUMBER    11
#elif (BOARD_NAME == S32K144EVBMC33771CEVB_BOARD)
#   define START_OUTPUT_PIN_NUMBER     0
#   define START_INTERRRUPT_PIN_NUMBER 1
#   define END_INTERRUPT_PIN_NUMBER    2
#   define START_NON_IMPLEMENTED_GPIOS 3
#   define END_NON_IMPLEMENTED_GPIOS   6
#elif (BOARD_NAME == MR_BMS771_BOARD)
#   define START_OUTPUT_PIN_NUMBER     0
#   define START_INTERRRUPT_PIN_NUMBER 11
#   define END_INTERRUPT_PIN_NUMBER    18
#else
#error add gpio defs.
#endif

#define NFC_ED_PIN_ACTIVE   0
#define NFC_ED_PIN_INACTIVE 1

#define CAN_LED_OFF         1
#define CAN_LED_ON          0

/*******************************************************************************
 * types
 ******************************************************************************/
/*!
 *   @brief this enum could be used to drive the GPIOs of the BMS and reflect the GPIOs from the
 *          rddrone-bms772.h or mr-bms771 file.
 *          With START_OUTPUT_PIN_NUMBER stating where the GPIO output pins start
 *          and START_INTERRRUPT_PIN_NUMBER stating where the GPIO (input) interrupt pins start
 *   @note  If you change this enum, make sure you change the rddrone-bms772.h or
 *          mr-bms771.h file as well and to the board specific s32k1xx_gpio.c g_gpiopins array.
 */

#if (BOARD_NAME == RDDRONE_BMS772_BOARD)
typedef enum {
    GATE_CTRL_CP    = 0,
    GATE_CTRL_D     = 1,
    BCC_RESET       = 2,
    NFC_HPD         = 3,
    AUTH_WAKE       = 4,
    PTE8            = 5,
    OVERCURRENT     = 6,
    SBC_WAKE        = 7,
    GATE_RS         = 8,
    SBC_LIMP        = 9,
    BCC_FAULT       = 10,
    NFC_ED          = 11
}pinEnum_t;

#elif (BOARD_NAME == S32K144EVBMC33771CEVB_BOARD)

typedef enum {
    BCC_RESET       = 0,
// Configure SW2 as SBC_WAKE (Button press)
    SBC_WAKE        = 1,
    BCC_FAULT       = 2,

    OVERCURRENT     = 3,
    GATE_CTRL_CP    = 4,
    GATE_CTRL_D     = 5,
    PTE8            = 6,
}pinEnum_t;

#elif (BOARD_NAME == MR_BMS771_BOARD)

typedef enum {
    GATE_CTRL_CP    = 0,
    GATE_CTRL_D     = 1,
    BCC_RESET       = 2,
    NFC_HPD         = 3,
    SE_RST_N        = 4,
    SE_EN           = 5,
    LED_CAN0        = 6,
    LED_CAN1        = 7,
    EXT_12V_ON      = 8,
    CAN1_EN         = 9,
    CAN1_STB_N      = 10,

    PTE8            = 11, // PTE8_SW_E_STOP pin
    OVERCURRENT     = 12,
    SBC_WAKE        = 13,
    CAN1_ERR_N      = 14,
    SBC_LIMP        = 15,
    BCC_FAULT       = 16,
    NFC_ED          = 17,
    E_FUSE_FLAG_N   = 18
}pinEnum_t;

#else
# error add GPIO enum
#endif
/*!
 *   @brief this enum could be used to change the GPIO input pintype
 */
typedef enum
{
    INPUT_INTERRUPT,
    INPUT_PULL_UP,
    INPUT_PULL_DOWN,
    INPUT_PIN_CONFIGURATIONS
} inputPinTypes_t;

/*******************************************************************************
 * public functions
 ******************************************************************************/
/*!
 * @brief   This function will initialze the GPIO pins
 *          it will open devices for each pin and the file descriptor will be
 *          saved, to ensure quick acces when reading or writing
 *
 * @param   skipSelfTest if this is true it will skip the self-test
 *
 * @return  0 if there is no error, otherwise the error number will indicate the error
 * @example if(gpio_init(false))
 *          {
 *            // do something with the error
 *          }
 *
 */
int gpio_init(bool skipSelfTest);

/*!
 * @brief   This function will read the state a gpio pin.
 *          it can be used to see if the pin is high or low
 *          the pin needs to be defined in the specific board file (like rrdrone-bms772.h)
 *          and it needs to be added to the array in <chip>_gpio.c
 *
 * @param   pin which pin to read from the pinEnum_t enum.
 *
 * @return  1 if the pin is high, 0 if the pin in low, -1 if there is an error
 * @example lvGpioReadVal = gpio_readPin(GATE_RS);
 *          if(lvGpioReadVal == -1)
 *          {
 *              // do something with the error
 *          }
 *          else if(lvGpioReadVal)
 *          {
 *              // do something
 *          }
 */
int gpio_readPin(pinEnum_t pin);

/*!
 * @brief   This function write a value to a gpio pin.
 *          it can be used to set the pin high or low
 *          the pin needs to be defined in the specific board file (like rrdrone-bms772.h)
 *          and it needs to be added to the array in <chip>_gpio.c
 *
 * @param   pin which pin to set from the pinEnum_t enum.
 * @param   newValue the new value of the pin, 1 is high and 0 is low
 *
 * @return  0 if succesfull, -1 if there is an error
 * @example if(gpio_writePin(GATE_CTRL_CP, 1))
 *          {
 *              // do something with the error
 *          }
 */
int gpio_writePin(pinEnum_t pin, bool newValue);

/*!
 * @brief   This function can be used to register a function as
 *          interrupt service routine (ISR) for multiple pins.
 * @warning This function should be called in a thread that is running (not ended) when the interrupt occurs!
 *          if the thread pid doesn't exist any more, it will not go to the ISR
 * @note    It will use SIGUSR1 for the pin interrupt
 * @note    Multiple pins will have the same ISR.
 *
 * @param   IsrPins Bitfield of the pins which to set the ISR for from the pinEnum_t enum.
 * @param   pinISRHandler The ISR handle function
 *          pinISRHandler = void handler(int signo, FAR siginfo_t *siginfo, FAR void *context);
 *
 * @return  0 if succesfull, otherwise a number to indicate an error
 * @example if(gpio_registerISR((uint32_t) ((1 << SBC_WAKE) + (1 <<BCC_FAULT)), handler);
 *          {
 *            // do something with the error
 *          }
 *
 *          void handler(int signo, siginfo_t *siginfo, void *context)
 *          {
 *            int pinNumber = (*siginfo).si_value.sival_int;
 *            cli_printf("cli_printf("GPIO ISR: sig: %d, pin: %d value %d\n", signo, pinNumber,
 *              gpio_readPin(pinNumber);
 *
 *            // do something with the pin
 *          }
 */
int gpio_registerISR(uint32_t IsrPins, _sa_sigaction_t pinISRHandler);

/*!
 * @brief   This function can be used to register an input pin as a different pin type
 *
 * @param   pin which pin to set from the pinEnum_t enum.
 * @param   newPinType To which the pin type should change from the inputPinTypes_t enum.
 *
 * @return  0 if succesfull, otherwise a number to indicate an error
 * @example if(gpio_changePinType(PTE8, INPUT_PULL_UP);
 *          {
 *            // do something with the error
 *          }
 */
int gpio_changePinType(pinEnum_t pin, inputPinTypes_t newPinType);

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* GPIO_H_ */
