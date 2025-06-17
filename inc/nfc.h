/****************************************************************************
 * nxp_bms/BMS_v1/inc/nfc.h
 *
 * Copyright 2020-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ** ###################################################################
 **     Filename    : nfc.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date        : 2020-08-25
 **     Abstract    :
 **        nfc module.
 **        This module contains all functions needed for using nfc
 **
 ** ###################################################################*/
/*!
 ** @file nfc.h
 **
 ** @version 01.00
 **
 ** @brief
 **        nfc module. this module contains the functions to control the NTAG5
 **
 */
#ifndef NFC_H_
#define NFC_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include "data.h"

/*******************************************************************************
 * defines
 ******************************************************************************/

/*******************************************************************************
 * types
 ******************************************************************************/

/*******************************************************************************
 * public functions
 ******************************************************************************/
/*!
 * @brief   This function will initialze the NFC
 *          it will test the i2C connection with the chip and read the slave address
 *
 * @param   skipSelfTest if this is true it will skip the self-test
 *
 * @return  0 if ok, -1 if there is an error
 * @example
 *          if(nfc_initialize(false))
 *          {
 *            // do something with the error
 *          }
 */
int nfc_initialize(bool skipSelfTest);

/*!
 * @brief   This function can be used to set the hard power-down (HPD) mode of the NFC
 *
 * @param   HPD if true, the microcontroller will set the NFC chip in hard power-down mode.
 *          if false, it will disable this mode.
 * @return  0 if ok, -1 if there is an error
 * @example
 *          if(nfc_setHPD())
 *          {
 *              // do something with the error
 *          }
 */
int nfc_setHPD(bool HPD);

/*!
 * @brief   This function can be used to update the BMS parameter to the NTAG
 * @note  Blocking
 *
 * @param   setOutdatedText If true it will set the text "outdated, please tap again!",
 *              If false, it will update the NTAG with the latest BMS data.
 * @param   wakingUpMessage If true it will set the waking up text (for sleep mode)
 *              If false, it will set the charge-relaxation string.
 * @param   pCommonBatteryVariables pointer to the commonBatteryVariables_t to update the information
 *              May be NULL if setOutdatedText or wakingUpMessage is true
 * @param   pCalcBatteryVariables pointer to the calcBatteryVariables_t to update the information
 *              May be NULL if setOutdatedText or wakingUpMessage is true
 *
 * @return  0 if ok, -1 if there is an error
 */
int nfc_updateBMSStatus(bool setOutdatedText, bool wakingUpMessage,
    commonBatteryVariables_t *pCommonBatteryVariables, calcBatteryVariables_t *pCalcBatteryVariables);

/*!
 * @brief   This function can be used to disable the NFC.
 *          Calling nfc_updateBMSStatus() will just return 0.
 * @note    It will be placed in HPD mode.
 *
 * @param   disable, true if it needs to be disabled
 * @example
 *          if(nfc_disableNFC(true))
 *          {
 *              // do something with the error
 *          }
 */
int nfc_disableNFC(bool disable);

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* NFC_H_ */
