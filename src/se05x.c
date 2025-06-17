/****************************************************************************
 * nxp_bms/BMS_v1/src/se05x.c
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

#include "se05x.h"
#include "cli.h"
#include "gpio.h"
#include "i2c.h"

/****************************************************************************
 * Defines
 ****************************************************************************/
#define SE05X_I2C_ADR               0x48
#define SCL_FREQ                    400000

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
 * @brief   This function will initialze the SE050 or SE051
 *          it will test the i2C connection with the chip
 *
 * @param   skipSelfTest if this is true it will skip the self-test
 *
 * @return  0 if ok, -1 if there is an error
 * @example
 *          if(se05x_initialize(false))
 *          {
 *            // do something with the error
 *          }
 */
int se05x_initialize(bool skipSelfTest)
{
    int lvRetValue = 0;
    // just in case make it 2 bytes of 0
    uint8_t regVal[2] = {0, 0};

    // Check if the self-test shouldn't be skipped
    if(!skipSelfTest)
    {
        cli_printf("SELF-TEST SE05X: START\n");

        // Make sure we can address the SE05X
        // write the not reset pin high
        lvRetValue = gpio_writePin(SE_RST_N, 1);

        // check if it went wrong
        if(lvRetValue)
        {
            cli_printfError("SE05X ERROR: writing SE_RST_N high went wrong!\n");

            cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
            return lvRetValue;
        }

        // wait for a small time to make sure the chip is awake
        usleep(100);

        // write the enable pin high
        lvRetValue = gpio_writePin(SE_EN, 1);

        // check if it went wrong
        if(lvRetValue)
        {
            cli_printfError("SE05X ERROR: writing SE_EN high went wrong!\n");

            cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
            return lvRetValue;
        }

        // wait for a small time to make sure the chip is awake
        usleep(100);

        // write 0 to the device and check for an ack
        lvRetValue = i2c_writeData(SE05X_I2C_ADR,
            0, regVal, 1);

        // check for errors
        if(lvRetValue)
        {
            //output to the user
            cli_printfError("SE05X ERROR: Can't do i2c tranfer, error: %d\n", lvRetValue);

		    // disable the SE05X again as we do not support this yet
        	// write the enable pin low
       	    lvRetValue = gpio_writePin(SE_EN, 0);

        	// check if it went wrong
        	if(lvRetValue)
        	{
            	cli_printfError("SE05X ERROR: writing SE_EN low went wrong!\n");

            	cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
        	}

            cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");

            // return to the user
            return lvRetValue;
        }

        // received ack, so chip is there.

        // disable the SE05X again as we do not support this yet
        // write the enable pin low
        lvRetValue = gpio_writePin(SE_EN, 0);

        // check if it went wrong
        if(lvRetValue)
        {
            cli_printfError("SE05X ERROR: writing SE_EN low went wrong!\n");

            cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
            return lvRetValue;
        }

        //cli_printf("SE05X I2C communication verified!\n");
        cli_printf("SELF-TEST SE05X: \e[32mPASS\e[39m\n");
    }

    // return to the user
    return lvRetValue;
}
