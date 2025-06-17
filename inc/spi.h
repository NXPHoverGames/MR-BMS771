/****************************************************************************
 * nxp_bms/BMS_v1/inc/spi.h
 *
 * Copyright 2020-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ** ###################################################################
 **     Filename    : spi.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date        : 2020-03-17
 **     Abstract    :
 **        spi module.
 **        This module contains all functions needed for spi
 **
 ** ###################################################################*/
/*!
 ** @file spi.h
 **
 ** @version 01.00
 **
 ** @brief
 **        spi module. this module contains the functions for spi
 **
 */
#ifndef SPI_H_
#define SPI_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
//#include "BMS_data_types.h"
#include <stdio.h>
#include "bcc_configuration.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

#ifndef CONFIG_SPI_TYPE_DEFAULT
#define  CONFIG_SPI_TYPE_DEFAULT SPIDEVTYPE_USER
#endif

#ifndef CONFIG_SPI_CSN_DEFAULT
#define  CONFIG_SPI_CSN_DEFAULT         0
#endif

#if BOARD_NAME != S32K144EVBMC33771CEVB_BOARD
#   ifndef CONFIG_BCC_SPI_CSN_DEFAULT
#       define  CONFIG_BCC_SPI_CSN_DEFAULT         0
#   endif
#else
#   ifndef CONFIG_BCC_SPI_CSN_DEFAULT
#       define  CONFIG_BCC_SPI_CSN_DEFAULT         1
#   endif
#endif

#ifndef CONFIG_SPI_MODE_DEFAULT
#define  CONFIG_SPI_MODE_DEFAULT        1 /* CPOL=0, CHPHA=1 */
#endif

#ifndef CONFIG_SPI_WIDTH_DEFAULT
#define  CONFIG_SPI_WIDTH_DEFAULT       16
#endif

#ifndef CONFIG_BCC_SPI_WIDTH_DEFAULT
#   if (BOARD_NAME == RDDRONE_BMS772_BOARD)
#       define  CONFIG_BCC_SPI_WIDTH_DEFAULT   40
#   else
#       define  CONFIG_BCC_SPI_WIDTH_DEFAULT   48
#   endif
#endif

#ifndef CONFIG_SPI_FREQ_DEFAULT
#define  CONFIG_SPI_FREQ_DEFAULT        4000000
#endif

#ifndef CONFIG_SPI_UDELAY_DEFAULT
#define  CONFIG_SPI_UDELAY_DEFAULT      0
#endif

#ifndef CONFIG_SPI_NWORDS_DEFAULT
#define  CONFIG_SPI_NWORDS_DEFAULT      1
#endif


#ifndef CONFIG_SPI_MINBUS_DEFAULT
#define  CONFIG_SPI_MINBUS_DEFAULT      0
#endif

#ifndef CONFIG_SPI_MAXBUS_DEFAULT
#define  CONFIG_SPI_MAXBUS_DEFAULT      1
#endif

#if BOARD_NAME != S32K144EVBMC33771CEVB_BOARD
#   define SBC_SPI_BUS                     0
#   define BCC_SPI_BUS                     1
#   define MAX_SPI_BUS                     BCC_SPI_BUS
#else
#   define SBC_SPI_BUS                     1
#   define BCC_SPI_BUS                     0
#   define MAX_SPI_BUS                     SBC_SPI_BUS
#endif

/*******************************************************************************
 * Types
 ******************************************************************************/
// the struct to configure the SPI
typedef struct
{
    uint32_t    devtype;    /* DevType (see spi_devtype_e)  */
    uint32_t    csn;        /* Chip select number for devtype  */
    uint8_t     mode;       /* Mode to use for transfer         */
    uint8_t     width;      /* is the data width (8 or 16)      */
    uint32_t    freq;       /* SPI frequency                    */
    useconds_t  udelay;     /* Delay in uS after transfer       */
    uint32_t    nwords;     /* No of words to exchange          */
}spiStruct_s;

/*******************************************************************************
 * public functions
 ******************************************************************************/

/*!
 * @brief   this function initializes the SPI mutex
 *
 * @param   none
 *
 * @return  If successful, the function will return zero (OK). Otherwise -1
 * @example if(spi_initialize())
 *          {
 *              // do something with the error
 *          }
 */
int spi_initialize(void);

/*!
 * @brief   this function is the SPI application driver
 *          it will take care of SPI transfers
 *          SPI transmits or receives can be done with this function
 *
 * @param   spiBus Which spi bus to use. this could for example be 0 if LPSPI0 is used. 1 if LPSPI1 is used
 * @param   txDataBuf The transmit data buffer to transmit (max 40 bits / 5 bytes), this may be NULL if not used
 * @param   rxDataBuf The receive data buffer to receive (max 40 bits / 5 bytes), this may be NULL if not used
 *
 * @return  If successful, the function will return zero (OK). Otherwise -1
 * @example for(i = 0; i < transferSize; i++)
 *          {
 *              txDataBuf[i] = i; // set the right data
 *          }
 *          if(spi_BMSTransferData(BCC_SPI_BUS, txDataBuf, rxDataBuf, transferSize))
 *          {
 *              // do something with the error
 *          }
 *
 *          // do something with the rxDataBuf
 */
int spi_BMSTransferData(uint8_t spiBus, uint8_t *txDataBuf, uint8_t *rxDataBuf);

/*!
 * @brief   This function get the current SPI configuration.
 *          it will get the source file global spiStruct_s struct values
 *
 * @param   spiBus Which spi bus to get info from. this could for example be 0 if LPSPI0 is used. 1 if LPSPI1 is used
 * @param   currentSpiConfiguration the spiStruct_s struct to set the current value to
 * @param   BCCconfiguration if this is true it will set the BCC configuration
 *
 * @return  If successful, the function will return zero (OK). Otherwise -1
 * @example if(spi_getCurrentConfiguration(BCC_SPI_BUS, &currentSpiConfiguration))
 *          {
 *              // do something with the error
 *          }
 */
int spi_getCurrentConfiguration(uint8_t spiBus, spiStruct_s *currentSpiConfiguration);

/*!
 * @brief   This function configures the SPI.
 *          it will set the source file global spiStruct_s struct with new values
 *          in startup this is configured with the default values
 *
 * @param   spiBus Which spi bus to get info from. this could for example be 0 if LPSPI0 is used. 1 if LPSPI1 is used
 * @param   newSpiConfiguration the new spiStruct_s struct to set the value
 *
 * @return  If successful, the function will return zero (OK). Otherwise -1
 * @example if(spi_configure(BCC_SPI_BUS, newSpiConfiguration))
 *          {
 *              // do something with the error
 *          }
 */
int spi_configure(uint8_t spiBus, spiStruct_s newSpiConfiguration);

/*!
 * @brief   this function will be used to configure if a spi transmission may be done on a bus
 *
 * @param   spiBus Which spi bus to enable. this could for example be 0 if LPSPI0 is used. 1 if LPSPI1 is used
 * @param   enable If this is true the SPI transmision of the bus is enabled
 *
 * @return  If successful, the function will return zero (OK). Otherwise -1
 * @example if(spi_EnableTransmission(BCC_SPI_BUS, false))
 *          {
 *              // do something with the error
 *          }
 */
int spi_EnableTransmission(uint8_t spiBus, bool enable);

/*!
 * @brief   this function will be used to configure if a spi transmission may be done on a bus
 *
 * @param   spiBus Which spi bus to enable. this could for example be 0 if LPSPI0 is used. 1 if LPSPI1 is used
 * @param   enabled address of the value that will be true if it is enabled.
 *
 * @return  If successful, the function will return zero (OK). Otherwise -1
 * @example if(spi_getEnabledTransmission(BCC_SPI_BUS, &boolVariable))
 *          {
 *              // do something with the error
 *          }
 */
int spi_getEnabledTransmission(uint8_t spiBus, bool *enabled);

/*!
 * @brief   this function will be used to lock or unlock the SPI for multiple SPI transfers
 * @warning Don't forget to unlock the lock!
 * @note    Could be used if you don't want to be interrupted by higher priority tasks
 *
 * @param   lock if this is true, it will lock it and the calling task may use the SPI transfers.
 *
 * @return  If successful, the function will return zero (OK). Otherwise -1
 * @example if(spi_lockNotUnlockBCCSpi(true))
 *          {
 *              // do something with the error
 *          }
 *          // do multiple SPI transfers uninterrupted
 *
 *          // unlock it again
 *          if(spi_lockNotUnlockBCCSpi(false))
 *          {
 *              // do something with the error
 *          }
 */
int spi_lockNotUnlockBCCSpi(bool lock);

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* SPI_H_ */
