/*
 * Copyright 2016 - 2020, 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*!
 * File: bcc_wait.h
 *
 * This file implements functions for busy waiting for BCC driver.
 */

#ifndef BCC_WAIT_H_
#define BCC_WAIT_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @brief Waits for specified amount of milliseconds.
 *
 * @param delay - Number of milliseconds to wait.
 */
void BCC_MCU_WaitMs(uint16_t delay);

/*!
 * @brief Waits for specified amount of microseconds.
 *
 * @param delay - Number of microseconds to wait.
 */
void BCC_MCU_WaitUs(uint32_t delay);

#endif /* BCC_WAIT_H_ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
