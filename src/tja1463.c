/****************************************************************************
 * nxp_bms/BMS_v1/src/tja1463.c
 *
 * Copyright 2024-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

#include "tja1463.h"
#include "cli.h"
#include "gpio.h"

/****************************************************************************
 * Defines
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/*!
 * @brief   This function will initialze the TJA1463
 *          it will test the connection with the chip
 *          And then put it in a lower power mode.
 *
 * @param   skipSelfTest if this is true it will skip the self-test
 *
 * @return  0 if ok, -1 if there is an error
 * @example
 *          if(tja1463_initialize(false))
 *          {
 *            // do something with the error
 *          }
 */
int tja1463_initialize(bool skipSelfTest)
{
    int ret = 0;

    // Check if the self-test shouldn't be skipped
    if(!skipSelfTest)
    {
        // add a pull up on CAN1_ERR_N
        ret = gpio_changePinType(CAN1_ERR_N, INPUT_PULL_UP);

        // check if it went wrong
        if(ret)
        {
            cli_printfError("TJA1463 ERROR: putting pull-up on CAN1_ERR_N went wrong!\n");

            cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
            ret |= tja1463_put_into_low_power(skipSelfTest);
            return ret;
        }

        // put the TJA1463 in listen-only mode (not standby and not enabled)
        ret = gpio_writePin(CAN1_STB_N, 1);

        // check if it went wrong
        if(ret)
        {
            cli_printfError("TJA1463 ERROR: CAN1_STB_N HIGH went wrong, not in listen-only mode!\n");

            cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
            ret |= tja1463_put_into_low_power(skipSelfTest);
            return ret;
        }

        // put the TJA1463 in listen-only mode (not standby and not enabled)
        ret = gpio_writePin(CAN1_EN, 0);

        // check if it went wrong
        if(ret)
        {
            cli_printfError("TJA1463 ERROR: CAN1_EN LOW went wrong, not in listen-only mode!\n");

            cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
            ret |= tja1463_put_into_low_power(skipSelfTest);
            return ret;
        }

        // turn on the LED to check that and spend some time
        ret = gpio_writePin(LED_CAN1, CAN_LED_ON);

        // check if it went wrong
        if(ret)
        {
            cli_printfError("TJA1463 ERROR: LED_CAN1 ON went wrong!\n");
            cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
        }

        // this printf also works as a wait to let the pin be high if not pulled down
        cli_printf("SELF-TEST TJA1463: START\n");

        // sleep for 1.5ms (t startup)
        usleep(1500);

        // check that CAN1_ERR_N is pulled low
        ret = gpio_readPin(CAN1_ERR_N);
        // pin should be pulled low by TJA1463

        // if pulled high
        if(ret == 1)
        {
            cli_printfError("TJA1463 ERROR: CAN1_ERR_N is not pulled down!\n");
            ret |= tja1463_put_into_low_power(skipSelfTest);
            return ret;
        }
        // if GPIO error
        else if (ret < 0)
        {
            cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
        }

        // go to normal mode
        // put the TJA1463 in normal mode (not standby and enabled)
        // clear some error flags
        ret = gpio_writePin(CAN1_EN, 1);

        // check if it went wrong
        if(ret)
        {
            cli_printfError("TJA1463 ERROR: CAN1_EN HIGH went wrong, not in normal mode!\n");

            cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
            ret |= tja1463_put_into_low_power(skipSelfTest);
            return ret;
        }

        // sleep for 50us to let the device react
        usleep(50);

        // go to listen-only mode
        // put the TJA1463 in listen-only mode (not standby and not enabled)
        // to clear the rest of the errors
        ret = gpio_writePin(CAN1_EN, 0);

        // check if it went wrong
        if(ret)
        {
            cli_printfError("TJA1463 ERROR: CAN1_EN HIGH went wrong, not in normal mode!\n");

            cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
            ret |= tja1463_put_into_low_power(skipSelfTest);

            return ret;
        }

        // sleep for 50us to let the device react
        usleep(50);

        // check that CAN1_ERR_N is pulled high by MCU as the errors a cleared
        ret = gpio_readPin(CAN1_ERR_N);
        // if indeed pulled high (OK)
        if(ret == 1)
        {
            // test OK
            cli_printf("SELF-TEST TJA1463: \e[32mPASS\e[39m\n");

            // make the variable OK
            ret = 0;
        }
        // if pulled low
        else if(ret == 0)
        {
            cli_printfError("TJA1463 ERROR: CAN1_ERR_N is not pulled up!\n");

            // this is not OK
            ret = 1;
        }
        // if GPIO error
        else
        {
            cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
        }
    }

    // put the TJA into low power mode
    ret |= tja1463_put_into_low_power(skipSelfTest);

    // return to the user
    return ret;
}

/*!
 * @brief   This function will put the TJA1463 into a low power mode.
 *
 * @return  0 if ok, -1 if there is an error
 * @example
 *          if(tja1463_put_into_low_power())
 *          {
 *            // do something with the error
 *          }
 */
int tja1463_put_into_low_power(bool skipSelfTest)
{
    int ret = 0;

    // put the TJA1463 in standby mode
    ret = gpio_writePin(CAN1_STB_N, 0);
    if(ret && !skipSelfTest)
    {
        cli_printfError("TJA1463 ERROR: CAN1_STB_N low went wrong, not in standby mode!\n");

        cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
    }

    // put the TJA1463 in sleep mode if possible (depends on wake pin as well)
    ret |= gpio_writePin(CAN1_EN, 1);
    if(ret && !skipSelfTest)
    {
        cli_printfError("TJA1463 ERROR: CAN1_EN high went wrong, not in sleep mode!\n");

        cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
    }

    // turn off the LED
    ret |= gpio_writePin(LED_CAN1, CAN_LED_OFF);
    // check if it went wrong
    if(ret && !skipSelfTest)
    {
        cli_printfError("TJA1463 ERROR: LED_CAN1 OFF went wrong!\n");
        cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
    }

    // change the pintype back to normal
    // remove a pull up on CAN1_ERR_N
    ret |= gpio_changePinType(CAN1_ERR_N, INPUT_INTERRUPT);

    // check if it went wrong
    if(!skipSelfTest && ret && ret != 1)
    {
        cli_printfError("TJA1463 ERROR: removing pull-up from CAN1_ERR_N went wrong!\n");

        cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
    }

    return ret;
}
