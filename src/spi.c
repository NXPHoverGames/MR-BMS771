/****************************************************************************
 * nxp_bms/BMS_v1/src/spi.c
 *
 * Copyright 2020-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi_transfer.h>

#include "spi.h"
#include "cli.h"
#include "power.h"

/****************************************************************************
 * Defines
 ****************************************************************************/
/* Device naming */
#define DEVNAME_FMT    "/dev/spi%d"
#define DEVNAME_FMTLEN (8 + 3 + 1)

#define MAX_BCC_BUFFER_SIZE 8
#define MAX_BUFFER_SIZE     1

/****************************************************************************
 * Types
 ****************************************************************************/

/****************************************************************************
 * private data
 ****************************************************************************/
static char gDevname[DEVNAME_FMTLEN];

// make the struct to configure the SPI transfer for the BCC
static spiStruct_s gBCCSpiStruct =
{
    .devtype    = CONFIG_SPI_TYPE_DEFAULT,
    .csn        = CONFIG_BCC_SPI_CSN_DEFAULT,
    .mode       = CONFIG_SPI_MODE_DEFAULT,
    .width      = CONFIG_BCC_SPI_WIDTH_DEFAULT,
    .freq       = CONFIG_SPI_FREQ_DEFAULT,
    .udelay     = CONFIG_SPI_UDELAY_DEFAULT,
    .nwords     = CONFIG_SPI_NWORDS_DEFAULT
};

// make the struct to configure the SPI transfer for other SPI transfers
static spiStruct_s gSpiStruct =
{
    .devtype    = CONFIG_SPI_TYPE_DEFAULT,
    .csn        = CONFIG_SPI_CSN_DEFAULT,
    .mode       = CONFIG_SPI_MODE_DEFAULT,
    .width      = CONFIG_SPI_WIDTH_DEFAULT,
    .freq       = CONFIG_SPI_FREQ_DEFAULT,
    .udelay     = CONFIG_SPI_UDELAY_DEFAULT,
    .nwords     = CONFIG_SPI_NWORDS_DEFAULT
};

// make a mutex lock for the BCC SPI driver
pthread_mutex_t gSpiBccLock;

// make a mutex lock for the SBC SPI driver
pthread_mutex_t gSpiSbcLock;

/*! @brief  semaphore to lock the bcc spi for longer than 1 transfer */
static sem_t gBccSpiSem;

// this variable is used to determain if the SPI bus may be used
bool gEnableSpiBus[2] = { false, false };

pid_t gSpiLockHolder = -1;
int   gSpiLockCount  = 0;
/****************************************************************************
 * private Functions declerations
 ****************************************************************************/
// insert the number of the SPI bus, will be 0 or 1 for the rddrone-bms772
// it will return the devpath "/dev/spi<bus>"
FAR char *spi_path(int bus);

// this function will open de spi device with O_RDONLY
int spi_open(int bus);

// this function is used to transfer on the SPI bus
// it will use the ioctl function with SPIIOC_TRANSFER
int spi_ioctlTransfer(int fd, FAR struct spi_sequence_s *seq);

/****************************************************************************
 * main
 ****************************************************************************/
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
int spi_initialize(void)
{
    int lvRetValue;

    // initialze the mutex
    lvRetValue = pthread_mutex_init(&gSpiBccLock, NULL);

    // check for errors
    if(lvRetValue)
    {
        cli_printfError("SPI ERROR: Couldn't initialize mutex BCC!\n");
        return lvRetValue;
    }

    // initialze the mutex
    lvRetValue = pthread_mutex_init(&gSpiSbcLock, NULL);

    // check for errors
    if(lvRetValue)
    {
        cli_printfError("SPI ERROR: Couldn't initialize mutex SBC!\n");
        return lvRetValue;
    }

    // initialize the semaphore with already 1
    sem_init(&gBccSpiSem, 0, 1);

    // set the busses enabled
    gEnableSpiBus[0] = true;
    gEnableSpiBus[1] = true;

    return lvRetValue;
}

/*!
 * @brief   this function is the SPI application driver
 *          it will take care of SPI transfers
 *          SPI transmits or receives can be done with this function
 *
 * @param   Which spi bus to use. this could for example be 0 if LPSPI0 is used. 1 if LPSPI1 is used
 * @param   The transmit data buffer to transmit (max 40 bits / 5 bytes), this may be NULL if not used
 * @param   The receive data buffer to receive (max 40 bits / 5 bytes), this may be NULL if not used
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
int spi_BMSTransferData(uint8_t spiBus, uint8_t *txDataBuf, uint8_t *rxDataBuf)
{
    int                   lvRetValue = -1;
    int                   lvFd;
    struct spi_trans_s    lvTrans;
    struct spi_sequence_s lvSeq;
    bool                  slowClockMode = false;
    mcuPowerModes_t       mcuPowerMode;

    // get the MCU power state and check for an error
    mcuPowerMode = power_setNGetMcuPowerMode(false, ERROR_VALUE);
    if(mcuPowerMode == ERROR_VALUE)
    {
        // get the error
        lvRetValue = errno;

        // error
        cli_printfError("SPI ERROR: Could not get MCU power state 1: %d \n", lvRetValue);
    }

    // check if the MCU power mode is SLEEP (SPI1 (BCC_SPI_BUS) is off)
    if(((mcuPowerMode == VLPR_MODE) || (mcuPowerMode == ERROR_VALUE)) && spiBus == BCC_SPI_BUS)
    {
        // error
        cli_printfError("SPI ERROR: SPI%d isn't enabled! mcu: %d == %d, bus%d\n", BCC_SPI_BUS, mcuPowerMode,
            VLPR_MODE, spiBus);

        return lvRetValue;
    }

    // check for slow clock, both busses are enabled, but on a slower clockrate
    if((mcuPowerMode == STANDBY_MODE) || (mcuPowerMode == VLPR_MODE) || (mcuPowerMode == ERROR_VALUE))
    {
        // it is in standby mode
        slowClockMode = true;
    }

    // limit the bus number
    if(spiBus > MAX_SPI_BUS)
    {
        cli_printfError("SPI ERROR: Bus number is incorrect! bus%d\n", spiBus);
        return lvRetValue;
    }

    // make the transmit buffer
    uint8_t txdataBuffer[MAX_BCC_BUFFER_SIZE] =
    {
#if (BOARD_NAME == RDDRONE_BMS772_BOARD)
        0x00, 0x00, 0x00, 0x10, 0x1C
#else
        0x00, 0x00, 0x00, 0x00, 0x10, 0x1C
#endif
    };

    uint8_t *txdata = txdataBuffer;

    // check if write buffer isn't NULL
    if(txDataBuf != NULL)
    {
        txdata = txDataBuf;
    }

    // make the receive buffer
    uint8_t rxdataBuffer[MAX_BCC_BUFFER_SIZE] = { 0x00 };

    uint8_t *rxdata = rxdataBuffer;

    // check if receive buffer isn't null
    if(rxDataBuf != NULL)
    {
        rxdata = rxDataBuf;
    }

    // check which bus it is
    if(spiBus == BCC_SPI_BUS)
    {
        // lock the BCC spi
        if(spi_lockNotUnlockBCCSpi(true))
        {
            cli_printfError("SPI ERROR: Couldn't lock spi\n");
        }

        // lock the BCC mutex
        pthread_mutex_lock(&gSpiBccLock);
    }
    else
    {
        // lock the SBC mutex
        pthread_mutex_lock(&gSpiSbcLock);
    }

    // check if the bus is enabled
    if(!gEnableSpiBus[spiBus])
    {
        cli_printfError("SPI ERROR: Bus%d is not enabled!\n", spiBus);

        // check which bus it is
        if(spiBus == BCC_SPI_BUS)
        {
            // unlock the BCC mutex
            pthread_mutex_unlock(&gSpiBccLock);

            // unlock the BCC spi
            if(spi_lockNotUnlockBCCSpi(false))
            {
                cli_printfError("SPI ERROR: Couldn't unlock spi\n");
            }
        }
        else
        {
            // unlock the SBC mutex
            pthread_mutex_unlock(&gSpiSbcLock);
        }

        return lvRetValue;
    }

    // open the SPI device
    lvFd = spi_open(spiBus);
    if(lvFd < 0)
    {
        // get the error
        lvRetValue = errno;

        // output
        cli_printfError("SPI ERROR: failed to get bus %d, error: %d, fd: %d\n", spiBus, lvRetValue, lvFd);

        // close the device
        close(lvFd);

        // check which bus it is
        if(spiBus == BCC_SPI_BUS)
        {
            // unlock the BCC mutex
            pthread_mutex_unlock(&gSpiBccLock);

            // unlock the BCC spi
            if(spi_lockNotUnlockBCCSpi(false))
            {
                cli_printfError("SPI ERROR: Couldn't unlock spi\n");
            }
        }
        else
        {
            // unlock the SBC mutex
            pthread_mutex_unlock(&gSpiSbcLock);
        }

        return lvRetValue;
    }

    // check which bus it is
    if(spiBus == BCC_SPI_BUS)
    {
        /* Set up the transfer profile */
        lvSeq.dev   = SPIDEV_ID(gBCCSpiStruct.devtype, gBCCSpiStruct.csn);
        lvSeq.mode  = gBCCSpiStruct.mode;
        lvSeq.nbits = gBCCSpiStruct.width;

        // check if it is slowclock mode
        if(slowClockMode)
        {
            // set the frequency
            lvSeq.frequency = gBCCSpiStruct.freq / 4;
        }
        else
        {
            // set the frequency
            lvSeq.frequency = gBCCSpiStruct.freq;
        }

        lvSeq.ntrans = 1;
        lvSeq.trans  = &lvTrans;

        lvTrans.deselect = false; /* De-select after transfer */
        lvTrans.delay    = gBCCSpiStruct.udelay;
        lvTrans.nwords   = gBCCSpiStruct.nwords;
    }
    else
    {
        /* Set up the transfer profile */
        lvSeq.dev   = SPIDEV_ID(gSpiStruct.devtype, gSpiStruct.csn);
        lvSeq.mode  = gSpiStruct.mode;
        lvSeq.nbits = gSpiStruct.width;

        // check if it is slowclock mode
        if(slowClockMode)
        {
            // set the frequency
            lvSeq.frequency = gBCCSpiStruct.freq / 4;
        }
        else
        {
            // set the frequency
            lvSeq.frequency = gBCCSpiStruct.freq;
        }

        lvSeq.ntrans = 1;
        lvSeq.trans  = &lvTrans;

        lvTrans.deselect = false; /* De-select after transfer */
        lvTrans.delay    = gSpiStruct.udelay;
        lvTrans.nwords   = gSpiStruct.nwords;
    }

    // the transmit buffers
    lvTrans.txbuffer = txdata;
    lvTrans.rxbuffer = rxdata;

    // transfer on SPI
    lvRetValue = ioctl(lvFd, SPIIOC_TRANSFER, &lvSeq);
    if(lvRetValue)
    {
        cli_printfError("SPI ERROR: failed to transfer! bus%d error: %d\n", spiBus, lvRetValue);
    }

    // close the device
    close(lvFd);

    // check which bus it is
    if(spiBus == BCC_SPI_BUS)
    {
        // unlock the BCC mutex
        pthread_mutex_unlock(&gSpiBccLock);

        // unlock the BCC spi
        if(spi_lockNotUnlockBCCSpi(false))
        {
            cli_printfError("SPI ERROR: Couldn't unlock spi\n");
        }
    }
    else
    {
        // unlock the SBC mutex
        pthread_mutex_unlock(&gSpiSbcLock);
    }

    // return to the user
    return lvRetValue;
}

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
int spi_getCurrentConfiguration(uint8_t spiBus, spiStruct_s *currentSpiConfiguration)
{
    int lvRetValue = OK;

    DEBUGASSERT(currentSpiConfiguration != NULL);

    // if need the BCC configuration
    if(spiBus == BCC_SPI_BUS)
    {
        // set the struct values
        *currentSpiConfiguration = gBCCSpiStruct;
    }
    else if(spiBus == SBC_SPI_BUS)
    {
        // set the struct values
        *currentSpiConfiguration = gSpiStruct;
    }
    else
    {
        cli_printfError("spi_getCurrentConfiguration ERROR: bus: %d not implemented!\n",
            spiBus);
        lvRetValue = -1;
    }

    return lvRetValue;
}

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
int spi_configure(uint8_t spiBus, spiStruct_s newSpiConfiguration)
{
    int lvRetValue = OK;

    // check which configuration needs to be written
    if(spiBus == BCC_SPI_BUS)
    {
        // set the new struct
        gBCCSpiStruct = newSpiConfiguration;
    }
    else if(spiBus == SBC_SPI_BUS)
    {
        // set the new struct
        gSpiStruct = newSpiConfiguration;
    }
    else
    {
        cli_printfError("spi_configure ERROR: bus: %d not implemented!\n",
            spiBus);
        lvRetValue = -1;
    }

    // return
    return lvRetValue;
}

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
int spi_EnableTransmission(uint8_t spiBus, bool enable)
{
    int lvRetValue = -1;

    // limit the bus number
    if(spiBus > MAX_SPI_BUS)
    {
        cli_printfError("SPI ERROR: Bus number is incorrect!\n");
        return lvRetValue;
    }

    // check which bus it is
    if(spiBus == BCC_SPI_BUS)
    {
        // lock the BCC mutex
        pthread_mutex_lock(&gSpiBccLock);
    }
    else
    {
        // lock the SBC mutex
        pthread_mutex_lock(&gSpiSbcLock);
    }

    // set the variable
    gEnableSpiBus[spiBus] = enable;

    // check which bus it is
    if(spiBus == BCC_SPI_BUS)
    {
        // unlock the BCC mutex
        pthread_mutex_unlock(&gSpiBccLock);
    }
    else
    {
        // unlock the SBC mutex
        pthread_mutex_unlock(&gSpiSbcLock);
    }

    // it went ok
    lvRetValue = 0;

    return lvRetValue;
}

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
int spi_getEnabledTransmission(uint8_t spiBus, bool *enabled)
{
    int lvRetValue = -1;

    // limit the bus number
    if(spiBus > MAX_SPI_BUS)
    {
        cli_printfError("SPI ERROR: Bus number is incorrect!\n");
        return lvRetValue;
    }

    // check for NULL pointer
    if(enabled == NULL)
    {
        cli_printfError("SPI ERROR: NULL pointer!\n");
        return lvRetValue;
    }

    // check which bus it is
    if(spiBus == BCC_SPI_BUS)
    {
        // lock the BCC mutex
        pthread_mutex_lock(&gSpiBccLock);
    }
    else
    {
        // lock the SBC mutex
        pthread_mutex_lock(&gSpiSbcLock);
    }

    // set the variable
    *enabled = gEnableSpiBus[spiBus];

    // check which bus it is
    if(spiBus == BCC_SPI_BUS)
    {
        // unlock the BCC mutex
        pthread_mutex_unlock(&gSpiBccLock);
    }
    else
    {
        // unlock the SBC mutex
        pthread_mutex_unlock(&gSpiSbcLock);
    }

    // it went ok
    lvRetValue = 0;

    return lvRetValue;
}

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
int spi_lockNotUnlockBCCSpi(bool lock)
{
    int lvRetValue = 0;

    // check if to lock or unlock
    if(lock)
    {
        pid_t me = getpid();

        // check if the task already has the lock
        if(gSpiLockHolder == me)
        {
            // just increment the lock count
            gSpiLockCount++;
        }
        else
        {
            // take the semaphore
            lvRetValue = sem_wait(&gBccSpiSem);

            // check if no error
            if(!lvRetValue)
            {
                gSpiLockHolder = me;
                gSpiLockCount  = 1;
            }
            // if error
            else
            {
                lvRetValue = errno;
                cli_printfError("spi_lockNotUnlockBCCSpi ERROR: Couldn't lock sem! %d", lvRetValue);
            }
        }
    }
    // if unlock
    else
    {
        // check if the lock is about the be released
        if(gSpiLockCount == 1)
        {
            // release and don't hold it anymore
            gSpiLockHolder = -1;
            gSpiLockCount  = 0;
            lvRetValue     = sem_post(&gBccSpiSem);

            // check for an error
            if(lvRetValue)
            {
                lvRetValue = errno;
                cli_printfError("spi_lockNotUnlockBCCSpi ERROR: Couldn't unlock sem! %d", lvRetValue);
            }
        }
        else if(gSpiLockCount > 1)
        {
            // We still hold the semaphore. Just decrement the count
            gSpiLockCount--;
        }
    }

    // return
    return lvRetValue;
}

/****************************************************************************
 * private Functions
 ****************************************************************************/

// insert the number of the SPI bus, will be 0 or 1 for the rddrone-bms772
// it will return the devpath "/dev/spi<bus>"
FAR char *spi_path(int bus)
{
    // make the device path
    snprintf(gDevname, DEVNAME_FMTLEN, DEVNAME_FMT, bus);

    // return it
    return gDevname;
}

// this function will open de spi device with O_RDONLY
int spi_open(int bus)
{
    FAR char *devpath;

    // Get the device path
    devpath = spi_path(bus);

    // Open the file for read-only access (we need only IOCTLs)
    return open(devpath, O_RDONLY);
}

// this function is used to transfer on the SPI bus
// it will use the ioctl function with SPIIOC_TRANSFER
int spi_ioctlTransfer(int fd, FAR struct spi_sequence_s *seq)
{
    // Perform the IOCTL
    return ioctl(fd, SPIIOC_TRANSFER, (unsigned long)((uintptr_t)seq));
}

//#endif
