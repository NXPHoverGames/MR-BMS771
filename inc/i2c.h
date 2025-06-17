/****************************************************************************
 * nxp_bms/BMS_v1/inc/i2c.h
 *
 * Copyright 2021, 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ** ###################################################################
 **     Filename    : i2c.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date        : 2021-04-22
 **     Abstract    :
 **        i2c module.
 **        This module contains all functions needed for i2c
 **
 ** ###################################################################*/
/*!
 ** @file i2c.h
 **
 ** @version 01.00
 **
 ** @brief
 **        i2c module. this module contains the functions for i2c
 **
 */
#ifndef I2C_H_
#define I2C_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>

/*******************************************************************************
 * Defines
 ******************************************************************************/
#define SCL_FREQ                  400000

/*******************************************************************************
 * Types
 ******************************************************************************/

/*******************************************************************************
 * public functions
 ******************************************************************************/
/*!
 * @brief   This function is used to initialize the I2C
 *
 * @return  0 if ok, -1 if there is an error
 */
int i2c_initialize(void);

/*!
 * @brief   This function can be used to read a data via the I2C
 *
 * @param   slaveAdr the slave address of the I2C device
 * @param   regAdr the address of the register to read from (2 bytes)
 * @param   readReg address of the variable to become the read value
 * @param   readBytes the amount of bytes to read max 255
 * @param   useRestart if this is high it will use a restart instead of a stop and start.
 *
 * @return  0 if ok, -1 if there is an error
 */
int i2c_readData(uint8_t slaveAdr, uint16_t regAdr, uint8_t* readReg, uint8_t readBytes,
    bool useRestart);

/*!
 * @brief   This function can be used to write a data via the I2C
 *
 * @param   slaveAdr the slave address of the I2C device
 * @param   regAdr the address of the register to write to (2 bytes)
 * @param   writeReg address of the variable to write
 * @param   writeBytes the amount of bytes to write max 255
 *
 * @return  0 if ok, -1 if there is an error
 */
int i2c_writeData(uint8_t slaveAdr, uint16_t regAdr, uint8_t* writeReg, uint8_t writeBytes);

/*!
 * @brief   This function can be used to read a data byte from the NFC chip session register
 *
 * @param   slaveAdr the slave address of the I2C NFC device
 * @param   regAdr the address of the register to read from (2 bytes)
 * @param   byteNum the numer of byte to read from (0-3)
 * @param   readReg address of the variable to become the read value
 * @param   readBytes the amount of bytes to read max 255
 *
 * @return  0 if ok, -1 if there is an error
 */
int i2c_nfcReadSessionRegByte(uint8_t slaveAdr, uint16_t regAdr, uint8_t byteNum,
    uint8_t* readReg, uint8_t readBytes);

/*!
 * @brief   This function can be used to write data to a byte of the NFC chip session register
 *
 * @param   slaveAdr the slave address of the I2C NFC device
 * @param   regAdr the address of the register to read from (2 bytes)
 * @param   byteNum the numer of byte to write to (0-3)
 * @param   writeReg address of the variable to write
 * @param   mask 8-bit control register bit mask. Only if corresponding control bit is set to 1b,
 *          the register bit will be overwritten.
 * @param   writeBytes the amount of bytes to write max 255
 *
 * @return  0 if ok, -1 if there is an error
 */
int i2c_nfcWriteSessionRegByte(uint8_t slaveAdr, uint16_t regAdr, uint8_t byteNum,
    uint8_t* writeReg, uint8_t mask, uint8_t writeBytes);

/*!
 * @brief   this function will be used to configure if an I2C transmission may be done
 *
 * @param   enable If this is true the I2C transmision of the bus is enabled, disabled otherwise
 *
 * @return  If successful, the function will return zero (OK). Otherwise -1
 * @example if(i2c_enableTransmission(false))
 *          {
 *              // do something with the error
 *          }
 */
int i2c_enableTransmission(bool enable);
/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* I2C_H_ */
