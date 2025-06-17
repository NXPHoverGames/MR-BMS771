/*
 * Copyright 2016 - 2021, 2024-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*!
 * File: bcc_peripheries.c
 *
 * This file implements functions for LPSPI and GPIO operations required by BCC
 * driver. Adapted from BCC SW example code version 1.1.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <assert.h>

#include "bcc_peripheries.h"            // Include header file
#include "bcc_configuration.h"
#include "gpio.h"
#include "spi.h"
#include "cli.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* If the old device, define 40bit transfer, 48 bit transfer otherwise */
#if (BOARD_NAME == RDDRONE_BMS772_BOARD)
#   define BCC_SPI_TRANSMISSION_BYTES  5
#else
#   define BCC_SPI_TRANSMISSION_BYTES  6
#endif

/* Number of bytes what 40b needs to be aligned in S32K118 SDK LPSPI driver
 * to. */
#define LPSPI_ALIGNMENT   8

/*******************************************************************************
 * Global variables (constants)
 ******************************************************************************/
pthread_mutex_t gSPIMutex;


/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief This function will initialize the spi mutex for the BCC function
 *
 * @return  int If successful, the pthread_mutex_init() function shall return zero;
 *          otherwise, an error number shall be returned to indicate the error.
 */
int BCC_initialze_spi_mutex(void)
{
    // initialzie the mutex
    return pthread_mutex_init(&gSPIMutex, NULL);
}

/*!
 * @brief This function performs one 40b transfer via SPI bus. Intended for SPI
 * mode only. This function needs to be implemented for specified MCU by the
 * user.
 *
 * The byte order of buffers is given by BCC_MSG_BIGEND macro (in bcc.h).
 *
 * @param drvInstance Instance of BCC driver.
 * @param transBuf Pointer to 40b data buffer to be sent.
 * @param recvBuf Pointer to 40b data buffer for received data.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_MCU_TransferSpi(uint8_t drvInstance, uint8_t transBuf[], uint8_t recvBuf[])
{
    // transfer the spi message with the spi transfer function

    uint8_t tBuf[LPSPI_ALIGNMENT];
    uint8_t rBuf[LPSPI_ALIGNMENT];
    int i;

/* If the BMS772 device, swap it like this */
#if (BOARD_NAME == RDDRONE_BMS772_BOARD)

    // swap the bytes
    for(i = 0; i < BCC_SPI_TRANSMISSION_BYTES; i++)
    {
        tBuf[i] = transBuf[BCC_SPI_TRANSMISSION_BYTES - (i+1)];
    }
#else
    // swap the bytes
    for(i = 0; i < BCC_SPI_TRANSMISSION_BYTES; i++)
    {
        // make it 3, 2, 1, 0, 5, 4
        tBuf[i] = transBuf[((BCC_SPI_TRANSMISSION_BYTES*((int)(i/4)))+4)-(i+1)];
    }
#endif
    // lock the mutex
    pthread_mutex_lock(&gSPIMutex);

    if(spi_BMSTransferData(BCC_SPI_BUS, tBuf, rBuf))
    {
        cli_printfError("BCC ERROR: SPI transfer failed!\n");
    }

    // unlock the mutex
    pthread_mutex_unlock(&gSPIMutex);

/* If the BMS772 device, swap it like this */
#if (BOARD_NAME == RDDRONE_BMS772_BOARD)

    // swap the bytes
    for(i = 0; i < BCC_SPI_TRANSMISSION_BYTES; i++)
    {
        recvBuf[i] = rBuf[BCC_SPI_TRANSMISSION_BYTES - (i+1)];
    }
#else
    // swap the bytes
    for(i = 0; i < BCC_SPI_TRANSMISSION_BYTES; i++)
    {
        recvBuf[i] = rBuf[((BCC_SPI_TRANSMISSION_BYTES*((int)(i/4)))+4)-(i+1)];
    }

#endif

    return BCC_STATUS_SUCCESS;
}

/*!
 * @brief This function will wake the BCC
 *
 * @return  int If successful, the pthread_mutex_init() function shall return zero;
 *          otherwise, an error number shall be returned to indicate the error.
 */
int BCC_wakeUpBCC(bcc_drv_config_t* const drvConfig, bcc_cid_t cid)
{
    spiStruct_s SPIConfig;
    uint32_t oldFreq;
    int ret = 0;

    // lock the BCC SPI
    if(spi_lockNotUnlockBCCSpi(true))
    {
        cli_printfError("BCC_wakeUpBCC ERROR: Couldn't lock spi\n");
        return -1;
    }

    // Get the current BCC SPI config struct
    spi_getCurrentConfiguration(BCC_SPI_BUS, &SPIConfig);

    // save and change the freq to make it slow enough for a wakeup frame
    oldFreq = SPIConfig.freq;
    SPIConfig.freq = (SPIConfig.freq / 8);

    // set the new frequency
    spi_configure(BCC_SPI_BUS, SPIConfig);

    // send a NOP frame that includes the message counter if 771C is used
    ret = (int)BCC_VerifyCom(drvConfig, cid);

    // Change the frequency back to what it was
    SPIConfig.freq = oldFreq;

    // set the original frequency
    spi_configure(BCC_SPI_BUS, SPIConfig);

    // unlock on the BCC SPI
    if(spi_lockNotUnlockBCCSpi(false))
    {
        cli_printfError("BCC_wakeUpBCC ERROR: Couldn't unlock spi\n");
        return -1;
    }

    // wait the wakeup time
    usleep(400);

    return ret;
}

/*FUNCTION**********************************************************************
 *
 *                 MODIFIED. NOT USED BUT NEEDED TO COMPILE BCC SW LIBRARY.
 *
 *END**************************************************************************/
bcc_status_t BCC_MCU_TransferTpl(uint8_t drvInstance, uint8_t transBuf[],
    uint8_t recvBuf[], uint16_t recvTrCnt)
{
    return BCC_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_MCU_Assert
 * Description   : User implementation of assert.
 *
 *END**************************************************************************/
void BCC_MCU_Assert(bool x)
{
    DEBUGASSERT(x);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_MCU_WriteCsbPin
 * Description   : Writes logic 0 or 1 to the CSB pin (or CSB_TX in case of TPL
 *                 mode).
 *
 *END**************************************************************************/
void BCC_MCU_WriteCsbPin(uint8_t drvInstance, uint8_t value)
{
    // nothing
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_MCU_WriteRstPin
 * Description   : Writes logic 0 or 1 to the RST pin.
 *
 *END**************************************************************************/
void BCC_MCU_WriteRstPin(uint8_t drvInstance, uint8_t value)
{
    // write the pin with the value
    gpio_writePin(BCC_RESET, value);
}

/*FUNCTION**********************************************************************
 *
 *                 MODIFIED. NOT USED BUT NEEDED TO COMPILE BCC SW LIBRARY.
 *
 *END**************************************************************************/
void BCC_MCU_WriteEnPin(uint8_t drvInstance, uint8_t value)
{
    // nothing
}

/*FUNCTION**********************************************************************
 *
 *                 MODIFIED. NOT USED BUT NEEDED TO COMPILE BCC SW LIBRARY.
 *
 *END**************************************************************************/
uint32_t BCC_MCU_ReadIntbPin(uint8_t drvInstance)
{
    return 0;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
