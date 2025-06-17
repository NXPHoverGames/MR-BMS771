/****************************************************************************
 * nxp_bms/BMS_v1/src/a1007.c
 *
 * Copyright 2020-2022, 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
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

//#include <nuttx/i2c/i2c_master.h>
#include "a1007.h"
#include "cli.h"
#include "gpio.h"
#include "i2c.h"

/****************************************************************************
 * Defines
 ****************************************************************************/
#define A1007_SLAVE_ADR             0x50
#define SCL_FREQ                    400000

#define STATUS_COMMAND_REG_ADR1     0x09
#define STATUS_COMMAND_REG_ADR2     0x00
#define STATUS_COMMAND_REG_ADR      0x0900

#define STATUS_COMMAND_REG_VAL_MASK 0x0F

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
 * @brief   This function will initialze the a1007
 *          it will test the i2C connection with the chip
 *
 * @param   skipSelfTest if this is true it will skip the self-test
 *
 * @return  0 if ok, -1 if there is an error
 * @example
 *          if(a1007_initialize(false))
 *          {
 *            // do something with the error
 *          }
 */
int a1007_initialize(bool skipSelfTest)
{
    int lvRetValue = 0;
    uint8_t regVal[2] = {0, 0};

    // Check if the self-test shouldn't be skipped
    if(!skipSelfTest)
    {
        cli_printf("SELF-TEST A1007: START\n");

        // wake up the A1007 with the wakeup pin
        // write the pin to high
        lvRetValue = gpio_writePin(AUTH_WAKE, 1);

        // check if it went wrong
        if(lvRetValue)
        {
            cli_printfError("A1007 ERROR: writing AUTH_WAKE high went wrong!\n");

            cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
            return lvRetValue;
        }

        // sleep for 50us for the wakeup pulse
        usleep(50);

        // write the pin to low
        lvRetValue = gpio_writePin(AUTH_WAKE, 0);

        // check if it went wrong
        if(lvRetValue)
        {
            cli_printfError("A1007 ERROR: writing AUTH_WAKE low went wrong!\n");

            cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
            return lvRetValue;
        }

        // get the status command reg val and check for errors
        lvRetValue = i2c_readData(A1007_SLAVE_ADR,
            STATUS_COMMAND_REG_ADR, regVal, 2, true);

        // check for errors
        if(lvRetValue)
        {
            //output to the user
            cli_printfError("A1007 ERROR: Can't do i2c tranfer, error: %d\n", lvRetValue);

            // return to the user
            return lvRetValue;
        }

        // check if there are no wrong status bits
        if((regVal[1] & STATUS_COMMAND_REG_VAL_MASK) != 0)
        {
            // output to the user
            cli_printfError("A1007 ERROR: status command has wrong bits!\n");

            cli_printf("is: %d, should be 0!\n", regVal[1] & STATUS_COMMAND_REG_VAL_MASK);

            cli_printf("Can't verify A1007 chip!\n");

            // set the returnvalue
            lvRetValue = -1;

            return lvRetValue;
        }

        //cli_printf("A1007 I2C communication verified!\n");
        cli_printf("SELF-TEST A1007: \e[32mPASS\e[39m\n");
    }

    // return to the user
    return lvRetValue;
}
