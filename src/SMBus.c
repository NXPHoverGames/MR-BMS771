/****************************************************************************
 * nxp_bms/BMS_v1/src/SMBus.c
 *
 * Copyright 2021-2025 NXP
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
#include <assert.h>

#include <arch/board/smbus_sbd.h>
#include "SMBus.h"
#include "data.h"
#include "cli.h"

/****************************************************************************
 * Defines
 ****************************************************************************/

#define KELVIN_TO_CELCIUS                 273.15
#define MINUTES_PER_HOUR                  60.0

#define MAX_ERROR_VAL                     5

#define MANUFACT_YEAR                     2021
#define MANUFACT_MONTH                    05
#define MANUFACT_DAY                      12

#define MANUFACT_DATA_LENGHT              1

#if (BOARD_NAME == RDDRONE_BMS772_BOARD)
#   define DEVICE_NAME_STRING             "RDDRONE-BMS772"
#else
#   define DEVICE_NAME_STRING             "MR-BMS771"
#endif

/****************************************************************************
 * Private Variables
 ****************************************************************************/
/*! @brief variable to indicate of it is initialized */
static bool gDontDoSMBus                  = false;
const char SMBus_path_sbd[]               = "/dev/smbus-sbd0";

/*! @brief  The manufactrure name*/
static const char manufacture_name[]      = "NXP";

/*! @brief  The device name*/
static const char device_name[]           = DEVICE_NAME_STRING;

/*! @brief  The chemistry name*/
static const char *device_chemistry[]    =
{
    "LiP",
    "LFP",
    "LFY",
    "NMC"
    "NIB"
};

/*! @brief  The array that will be written in the manufacturer_data of the SMBus Smart Battery Data*/
static const uint8_t manufacturer_data[MANUFACT_DATA_LENGHT]  =
{
    0x0,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/*!
 * @brief   This function will initialze the SMBus (smart battery bus)
 *          it will open the device and start the SMBus task
 *
 * @return  0 if ok, -1 if there is an error
 * @example
 *          if(SMBus_initialize())
 *          {
 *            // do something with the error
 *          }
 */
int SMBus_initialize(void)
{
    // return to the user
    return OK;
}

/*!
 * @brief   this function will increase the semaphore so the SMBus
 *          task will update the BMS status of the SBD driver (SMBus)
 *
 * @param   resetCurrent If this value is true, the current will be set to 0
 *          This could be used if a low power state is used and the SMBus struct
 *          will not be updated anymore
 * @param   pCommonBatteryVariables pointer to the commonBatteryVariables_t to update the information
 *          May be NULL if resetCurrent is true
 * @param   pCalcBatteryVariables pointer to the calcBatteryVariables_t to update the information
 *          May be NULL if resetCurrent is true
 *
 * @return  If successful, the function will return zero (OK).
 *          Otherwise, an error number will be returned to indicate the error:
 */
int SMBus_updateInformation(bool resetCurrent, commonBatteryVariables_t *pCommonBatteryVariables,
calcBatteryVariables_t *pCalcBatteryVariables)
{
    int fd, i;
    variableTypes_u variable1;

    // check if SMBus shouldn't be done
    if(gDontDoSMBus)
    {
        // just return without an error
        return 0;
    }

    // check for error
    if(data_getParameter(BATTERY_TYPE,
        &variable1.uint8Var, NULL) == NULL)
    {
        // set the default value
        variable1.uint8Var = BATTERY_TYPE_DEFAULT;

        // error output
        cli_printfError("SMBus ERROR: could not get battery-type!\n");
    }

    // make the struct with the default parameters
    /*! @brief  The structure that will be written in the SMBus SBD driver, this could be retreived with SMBus*/
    struct smbus_sbd_data_s smbus_sbd_data =
    {
        .temperature              = 0,    /* 0.1  K */
        .voltage                  = 0,    /* 1.0 mV */
        .current                  = 0,    /* 1.0 mA */
        .average_current          = 0,    /* 1.0 mA */
        .max_error                = MAX_ERROR_VAL,    /* 1.0  % */
        .relative_state_of_charge = 0,    /* 1.0  % */
        .absolute_state_of_charge = 0,    /* 1.0  % */
        .remaining_capacity       = 0,    /* 1.0 mAh (or 10 mWh?) */
        .full_charge_capacity     = 0,    /* 1.0 mAh (or 10 mWh?) */
        .run_time_to_empty        = 0,    /* 1.0  min */
        .average_time_to_empty    = 0,    /* 1.0  min */

        .cycle_count              = 0,    /* 1.0  cycle */
        .design_capacity          = 0,    /* 1.0 mAh (or 10 mWh?) */
        .design_voltage           = 0,    /* 1.0 mV */

        .manufacture_date         = ((MANUFACT_YEAR - 1980) *
                                    512 + MANUFACT_MONTH *
                                    32 + MANUFACT_DAY),   /* (year - 1980) * 512 + month * 32 + day */
        .serial_number            = 0,
        .manufacturer_name        = manufacture_name,
        .device_name              = device_name,
        .device_chemistry         = device_chemistry[variable1.uint8Var],
        .manufacturer_data        = manufacturer_data,
        .manufacturer_data_length = MANUFACT_DATA_LENGHT,

        .cell1_voltage            = 0,    /* 1.0 mV */
        .cell2_voltage            = 0,    /* 1.0 mV */
        .cell3_voltage            = 0,    /* 1.0 mV */
        .cell4_voltage            = 0,    /* 1.0 mV */
        .cell5_voltage            = 0,    /* 1.0 mV */
        .cell6_voltage            = 0,    /* 1.0 mV */
#if (BOARD_NAME == MR_BMS771_BOARD || BOARD_NAME == S32K144EVBMC33771CEVB_BOARD)
        .cell7_voltage            = 0,    /* 1.0 mV */
        .cell8_voltage            = 0,    /* 1.0 mV */
        .cell9_voltage            = 0,    /* 1.0 mV */
        .cell10_voltage           = 0,    /* 1.0 mV */
        .cell11_voltage           = 0,    /* 1.0 mV */
        .cell12_voltage           = 0,    /* 1.0 mV */
        .cell13_voltage           = 0,    /* 1.0 mV */
        .cell14_voltage           = 0,    /* 1.0 mV */
#endif

    };

    /* Make an array of pointers to loop through the variables */
    uint16_t *cellVoltagePointerArr4_N_CELLS_MAX[N_CELLS_MAX-N_CELLS_MIN_772] =
        {&smbus_sbd_data.cell4_voltage,
        &smbus_sbd_data.cell5_voltage, &smbus_sbd_data.cell6_voltage,
#if (BOARD_NAME == MR_BMS771_BOARD || BOARD_NAME == S32K144EVBMC33771CEVB_BOARD)
        &smbus_sbd_data.cell7_voltage, &smbus_sbd_data.cell8_voltage,
        &smbus_sbd_data.cell9_voltage, &smbus_sbd_data.cell10_voltage,
        &smbus_sbd_data.cell11_voltage, &smbus_sbd_data.cell12_voltage,
        &smbus_sbd_data.cell13_voltage, &smbus_sbd_data.cell14_voltage,
#endif
    };

    // open the device
    fd = open("/dev/smbus-sbd0", O_RDWR);

    if(fd == ERROR)
    {
        /* Something went wrong.  You could try to handle the error here, pass
         * it on to the calling function, or just ignore it.
         */

        cli_printfError("SMBus ERROR: could not open FD: %d\n", fd);

        // don't do SMBus anymore
        gDontDoSMBus = true;

        // error on exit
        variable1.int32Var = -1;
    }
    else
    {
        // check if the current should be updated
        if(resetCurrent)
        {
            variable1.int32Var = read(fd, (void *)&smbus_sbd_data, sizeof(smbus_sbd_data));
            if(variable1.int32Var != sizeof(smbus_sbd_data))
            {
                cli_printfError("SMBus ERROR: could not read sbd data: %d errno: %d\n",
                    variable1.int32Var, errno);

                variable1.int32Var = -1;
            }

            // set the current to 0
            smbus_sbd_data.current = 0;
        }
        else
        {
            // check for NULL pointers in debug mode
            DEBUGASSERT(pCommonBatteryVariables != NULL);
            DEBUGASSERT(pCalcBatteryVariables != NULL);

            // update the struct

            // only set temperature sensor if sensor is enabled
            if(pCommonBatteryVariables->sensor_enable)
            {
                // set the temperature
                smbus_sbd_data.temperature =
                    (uint16_t)((float)(pCommonBatteryVariables->C_batt + KELVIN_TO_CELCIUS) * 10.0);
            }

            // convert battery voltage to mv and place in struct
            smbus_sbd_data.voltage = (uint16_t)((float)(pCommonBatteryVariables->V_batt*1000.0));

            // convert current to mA and place in struct
            smbus_sbd_data.current = (uint16_t)((int32_t)(pCommonBatteryVariables->I_batt*1000.0));

            // convert average_current to mA and place in struct
            smbus_sbd_data.average_current = (uint16_t)((int32_t)(pCommonBatteryVariables->I_batt_avg*1000.0));

            // set both state of charges in %
            smbus_sbd_data.relative_state_of_charge = pCalcBatteryVariables->s_charge;
            smbus_sbd_data.absolute_state_of_charge = pCalcBatteryVariables->s_charge;

            // convert capacity to mAh and place in struct
            smbus_sbd_data.remaining_capacity = (uint16_t)(pCalcBatteryVariables->A_rem*1000.0);

            // convert to mAh and place in struct
            smbus_sbd_data.full_charge_capacity = (uint16_t)(pCalcBatteryVariables->A_full*1000.0);

            // calculate the time to empty ((Ah/A) * minutes per h)
            variable1.int32Var = (int32_t)(((float)(pCalcBatteryVariables->A_rem / pCommonBatteryVariables->I_batt) * MINUTES_PER_HOUR));

            // limit the value
            if(variable1.int32Var < 0)
            {
                // set to 0
                variable1.int32Var = 0;
            }
            // check if it is larger than the max
            else if(variable1.int32Var > UINT16_MAX)
            {
                // set to the max value
                variable1.int32Var = UINT16_MAX;
            }

            // set the time to empty
            smbus_sbd_data.run_time_to_empty = (uint16_t)variable1.int32Var;

            // calculate the time to empty ((Ah/A) * minutes per h)
            variable1.int32Var = (int32_t)((float)(pCalcBatteryVariables->A_rem / pCommonBatteryVariables->I_batt_avg) * MINUTES_PER_HOUR);

            // limit the value
            if(variable1.int32Var < 0)
            {
                // set to 0
                variable1.int32Var = 0;
            }
            // check if it is larger than the max
            else if(variable1.int32Var > UINT16_MAX)
            {
                // set to the max value
                variable1.int32Var = UINT16_MAX;
            }

            // set the average run time to empty
            smbus_sbd_data.average_time_to_empty = (uint16_t)variable1.int32Var;

            // get the cycle count
            if(data_getParameter(N_CHARGES, &variable1.uint16Var, NULL) == NULL)
            {
                // set the default value
                variable1.uint16Var = N_CHARGES_DEFAULT;

                // error output
                cli_printfError("SMBus ERROR: could not get n-charges!\n");
            }

            // set the cycle count
            smbus_sbd_data.cycle_count = variable1.uint16Var;

            // convert to mAh and place in struct
            smbus_sbd_data.design_capacity = (uint16_t)(pCalcBatteryVariables->A_factory*1000.0);

            // get the cell ov as design voltage
            if(data_getParameter(V_CELL_OV, &variable1.floatVar, NULL) == NULL)
            {
                // set the default value
                variable1.floatVar = V_CELL_OV_DEFAULT;

                // error output
                cli_printfError("SMBus ERROR: could not get v-cell-ov!\n");
            }

            // convert to mv and place in struct
            smbus_sbd_data.design_voltage = (uint16_t)(variable1.floatVar*1000.0);

            // set the battery ID
            smbus_sbd_data.serial_number = (uint16_t)pCalcBatteryVariables->batt_id;

            // set the cell voltages in mV
            // first set the minimum 3 cell voltages
            smbus_sbd_data.cell1_voltage = (uint16_t)((pCommonBatteryVariables->V_cellVoltages).value.V_cell1*1000.0);
            smbus_sbd_data.cell2_voltage = (uint16_t)((pCommonBatteryVariables->V_cellVoltages).value.V_cell2*1000.0);
            smbus_sbd_data.cell3_voltage = (uint16_t)((pCommonBatteryVariables->V_cellVoltages).value.V_cell3*1000.0);

            // check for error
            if(data_getParameter(N_CELLS,
                &variable1.uint8Var, NULL) == NULL)
            {
                // set the default value
                variable1.uint8Var = N_CELLS_DEFAULT;

                // error output
                cli_printfError("SMBus ERROR: could not get n-cells!\n");
            }

            // Loop through the rest of the cells
            for(i = 0; i < (variable1.uint8Var-N_CELLS_MIN_772); i++)
            {
                *(cellVoltagePointerArr4_N_CELLS_MAX[i]) = (uint16_t)(((pCommonBatteryVariables->V_cellVoltages).V_cellArr[i+N_CELLS_MIN_772])*1000.0);
            }
        }

        /* Write a prepared smbus_sbd_data_s struct to the SMBus SBD driver.  It needs to be
         * converted to a constant character buffer and the buffer length has to be
         * equal to the size of the smbus_sbd_data_s struct.
         *
         * Note that this write operation can be performed as often as you like.
         * The SMBus SBD driver will always provide the most recent data that it received
         * when it receives a request for battery data.
         */

        variable1.int32Var = write(fd, (const char *)&smbus_sbd_data, sizeof(struct smbus_sbd_data_s));
        if(variable1.int32Var != sizeof(struct smbus_sbd_data_s))
        {
            /* Something went wrong.  You could try to handle the error here, pass
             * it on to the calling function, or just ignore it.
             */

            cli_printfError("SMBus ERROR: could not write new data: %d errno: %d\n",
                variable1.int32Var, errno);

            //return -1;

            variable1.int32Var = -1;
        }
        else
        {
            variable1.int32Var = 0;
        }

        // close the file descriptor
        close(fd);
    }

    // return to user
    return variable1.int32Var;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/
