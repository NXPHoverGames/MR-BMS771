/*
 * Copyright 2016 - 2021, 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 * File: bcc_wait.c
 *
 * This file implements functions for busy waiting for BCC driver.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <unistd.h>

#include <stdint.h>
#include <stdbool.h>
#include "BCC/bcc_wait.h"       // Include header file

#define MS_TO_US 1000

/*******************************************************************************
 * Defines
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_WaitMs
 * Description   : Waits for specified amount of milliseconds.
 *
 *END**************************************************************************/
void BCC_MCU_WaitMs(uint16_t delay)
{
    // implement the wait function
    BCC_MCU_WaitUs(delay*MS_TO_US);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_WaitUs
 * Description   : Waits for specified amount of microseconds.
 *
 *END**************************************************************************/
void BCC_MCU_WaitUs(uint32_t delay)
{
    // sleep for an amount of us
    usleep(delay);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
