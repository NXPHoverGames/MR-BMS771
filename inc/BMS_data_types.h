/****************************************************************************
 * nxp_bms/BMS_v1/inc/BMS_data_types.h
 *
 * Copyright 2020-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ** ###################################################################
 **     Filename    : BMS_data_types.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date        : 2020-03-17
 **     Abstract    :
 **        BMS data types module.
 **        This module contains all data types used in shared memory
 **
 ** ###################################################################*/
/*!
 ** @file BMS_data_types.h
 **
 ** @version 01.00
 **
 ** @brief
 **        BMS_data_types module. this module contains the data types for the shared memory
 **
 */
#ifndef BMS_DATA_TYPES_H_
#define BMS_DATA_TYPES_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/

/* Global */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <float.h>
#include <string.h>

#include "BMS_data_limits.h"


/*******************************************************************************
 * Defines
 ******************************************************************************/
#define LIPO_V_CELL_OV_DEFAULT              4.2         //!< [V] the default cell overvoltage level of a LiPo battery cell
#define LIPO_V_CELL_UV_DEFAULT              3           //!< [V] the default cell undervoltage level of a LiPo battery cell
#define LIPO_V_STORAGE_DEFAULT              3.8         //!< [V] the default cell storage voltage level of a LiPo battery cell
#define LIPO_V_CELL_NOMINAL_DEFAULT         3.7         //!< [V] the nominal cell voltage of a LiPo battery cell

#define LIFEPO4_V_CELL_OV_DEFAULT           3.6         //!< [V] the default cell overvoltage level of a LiFePo4 battery cell
#define LIFEPO4_V_CELL_UV_DEFAULT           2.5         //!< [V] the default cell undervoltage level of a LiFePo4 battery cell
#define LIFEPO4_V_STORAGE_DEFAULT           3.3         //!< [V] the default cell storage voltage level of a LiFePo4 battery cell
#define LIFEPO4_V_CELL_NOMINAL_DEFAULT      3.3         //!< [V] the nominal cell voltage of a LiFePo4 battery cell

#define LIFEYPO4_V_CELL_OV_DEFAULT          3.6         //!< [V] the default cell overvoltage level of a LiFeYPo4 battery cell
#define LIFEYPO4_V_CELL_UV_DEFAULT          2.8         //!< [V] the default cell undervoltage level of a LiFeYPo4 battery cell
#define LIFEYPO4_V_STORAGE_DEFAULT          3.2         //!< [V] the default cell storage voltage level of a LiFeYPo4 battery cell
#define LIFEYPO4_V_CELL_NOMINAL_DEFAULT     3.3         //!< [V] the nominal cell voltage of a LiFeYPo4 battery cell

#define SODIUM_ION_V_CELL_OV_DEFAULT        4.1         //!< [V] the default cell overvoltage level of a sodium-ion (Na-ion) battery cell
#define SODIUM_ION_V_CELL_UV_DEFAULT        1.5         //!< [V] the default cell undervoltage level of a sodium-ion (Na-ion), keep minimum system voltage of BMS in mind!
#define SODIUM_ION_V_STORAGE_DEFAULT        SODIUM_ION_V_CELL_UV_DEFAULT        //!< [V] the default cell storage voltage level of a sodium-ion (Na-ion)
#define SODIUM_ION_V_CELL_NOMINAL_DEFAULT   3.1         //!< [V] the nominal cell voltage of a sodium-ion (Na-ion)
// TODO add different OT and UT for sodium-ion battery cells; -10C - 45C charge and -30C - 60C discharge

#define S_HEALTH_UNKNOWN                    127

#define STATUS_ASK_PARS_BIT                 0           //!< There is a change in the extra parameters so these should be asked
#define STATUS_TEMP_ERROR_BIT               1           //!< Battery temperature limit failure, the temperature is either too high or too low
#define STATUS_OVERLOAD_BIT                 2           //!< Safe operating area violation, the controller should look at drawing less current
#define STATUS_BAD_BATTERY_BIT              3           //!< This battery should not be used anymore (e.g. low SoH)
#define STATUS_NEED_SERVICE_BIT             4           //!< This battery requires maintenance (e.g. balancing, full recharge)
#define STATUS_BMS_ERROR_BIT                5           //!< Battery management system/controller error, smart battery interface error
#define STATUS_OPTIONAL1_BIT                6           //!< To be applied to another status
#define STATUS_OPTIONAL2_BIT                7           //!< To be applied to another status
#define STATUS_HIGHEST_BIT                  STATUS_OPTIONAL2_BIT //!< the highest bit of the status flags

#define S_FLAGS_UKNOWN                      255         //!< When the status is unknown

#define CYPHAL_MAX_SUB_ID                   8191        //!< The maximum CYPHAL subject ID
#define CYPHAL_UNSET_SUB_ID                 65535

#define CAN_MODE_OFF                        "OFF\0"     //!< The setting for turning CAN off.
#define CAN_MODE_DRONECAN                   "DRONECAN\0"//!< The setting for setting CAN to DroneCAN.
#define CAN_MODE_CYPHAL                     "CYPHAL\0"  //!< The setting for setting CAN to CyphalCAN.

// default values
#define V_OUT_DEFAULT                       0           //!< [V]
#define V_BATT_DEFAULT                      0           //!< [V]
// if the rddrone-bms772 board with the MC33772B chip
#if (BOARD_NAME == RDDRONE_BMS772_BOARD)
#   define N_CELLS_DEFAULT                  3           //!< [-]
// if the mr-bms771 board with the MC33771C chip
#else
#   define N_CELLS_DEFAULT                  12          //!< [-]
#endif

#define V_CELL1_DEFAULT                     0           //!< [V]
#define V_CELL2_DEFAULT                     0           //!< [V]
#define V_CELL3_DEFAULT                     0           //!< [V]
#define V_CELL4_DEFAULT                     0           //!< [V]
#define V_CELL5_DEFAULT                     0           //!< [V]
#define V_CELL6_DEFAULT                     0           //!< [V]
#define V_CELL7_DEFAULT                     0           //!< [V]
#define V_CELL8_DEFAULT                     0           //!< [V]
#define V_CELL9_DEFAULT                     0           //!< [V]
#define V_CELL10_DEFAULT                    0           //!< [V]
#define V_CELL11_DEFAULT                    0           //!< [V]
#define V_CELL12_DEFAULT                    0           //!< [V]
#define V_CELL13_DEFAULT                    0           //!< [V]
#define V_CELL14_DEFAULT                    0           //!< [V]
#define I_BATT_DEFAULT                      0           //!< [A]
#define I_BATT_AVG_DEFAULT                  0           //!< [A]
#define I_BATT_10S_AVG_DEFAULT              0           //!< [A]
#define SENSOR_ENABLE_DEFAULT               0           //!< [-]
#define C_BATT_DEFAULT                      0           //!< [C]
#define C_AFE_DEFAULT                       0           //!< [C]
#define C_T_DEFAULT                         0           //!< [C]
#define C_R_DEFAULT                         0           //!< [C]

#define P_AVG_DEFAULT                       0           //!< [W]
#define E_USED_DEFAULT                      0           //!< [Wh]
#define T_FULL_DEFAULT                      0           //!< [h]
#define A_REM_DEFAULT                       0           //!< [Ah]
#define A_FULL_DEFAULT                      4.6         //!< [Ah]
#define A_FACTORY_DEFAULT                   A_FULL_DEFAULT  //!< [Ah]
#define S_CHARGE_DEFAULT                    0           //!< [%]
#define S_HEALTH_DEFAULT                    S_HEALTH_UNKNOWN    //!< [%]
#define S_OUT_DEFAULT                       0           //!< [-]
#define S_IN_FLIGHT_DEFAULT                 0           //!< [-]
#define BATT_ID_DEFAULT                     0           //!< [-]

#define V_CELL_OV_DEFAULT                   LIPO_V_CELL_OV_DEFAULT //!< [V]
#define V_CELL_UV_DEFAULT                   LIPO_V_CELL_UV_DEFAULT //!< [V]
#define V_CELL_NOMINAL_DEFAULT              LIPO_V_CELL_NOMINAL_DEFAULT //!<[V]
#define V_STORAGE_DEFAULT                   LIPO_V_STORAGE_DEFAULT //!< [V]
#define V_CELL_MARGIN_DEFAULT               50          //!< [mV]
#define V_RECHARGE_MARGIN_DEFAULT           200         //!< [mV]
#define I_PEAK_MAX_DEFAULT                  200         //!< [A]
#define I_OUT_MAX_DEFAULT                   30          //!< [A]
#define I_OUT_NOMINAL_DEFAULT               I_OUT_MAX_DEFAULT //!< [A]
#define I_FLIGHT_MODE_DEFAULT               5           //!< [A]
#define I_SYSTEM_DEFAULT                    40          //!< [mA]
#define I_CHARGE_MAX_DEFAULT                4.6         //!< [A]
#define I_CHARGE_NOMINAL_DEFAULT            I_CHARGE_MAX_DEFAULT //!< [A]
#define I_CHARGE_FULL_DEFAULT               50          //!< [mA]
#define C_CELL_OT_DEFAULT                   45          //!< [C]
#define C_CELL_UT_DEFAULT                   (-20)       //!< [C]
#define C_PCB_OT_DEFAULT                    45          //!< [C]
#define C_PCB_UT_DEFAULT                    (-20)       //!< [C]
#define C_CELL_OT_CHARGE_DEFAULT            40          //!< [C]
#define C_CELL_UT_CHARGE_DEFAULT            0           //!< [C]
#define N_CHARGES_DEFAULT                   0           //!< [-]
#define N_CHARGES_FULL_DEFAULT              0           //!< [-]
#define OCV_SLOPE_DEFAULT                   5.3         //!< [mV/A.min]
#define BATTERY_TYPE_DEFAULT                3           //!< [-]

#define T_MEAS_DEFAULT                      1000        //!< [ms]
#define T_FTTI_DEFAULT                      1000        //!< [ms]
#define T_BMS_TIMEOUT_DEFAULT               600         //!< [s]
#define T_FAULT_TIMEOUT_DEFAULT             60          //!< [s]
#define T_BCC_SLEEP_CYCLIC_DEFAULT          1           //!< [s]
#define T_SLEEP_TIMEOUT_DEFAULT             12          //!< [h]
#define T_OCV_CYCLIC0_DEFAULT               300         //!< [s]
#define T_OCV_CYCLIC1_DEFAULT               86400       //!< [s]
#define T_CHARGE_DETECT_DEFAULT             1           //!< [s]
#define T_CB_DELAY_DEFAULT                  120         //!< [s]
#define T_CHARGE_RELAX_DEFAULT              300         //!< [s]
#define BATT_EOL_DEFAULT                    80          //!< [%]
#define S_FLAGS_DEFAULT                     S_FLAGS_UKNOWN  //!< [-]
#define SELF_DISCHARGE_ENABLE_DEFAULT       1           //!< [-]
#define FLIGHT_MODE_ENABLE_DEFAULT          0           //!< [-]
#define EMERGENCY_BUTTON_ENABLE_DEFAULT     0           //!< [-]
#define SMBUS_ENABLE_DEFAULT                0           //!< [-]
#define GATE_CHECK_ENABLE_DEFAULT           1           //!< [-]
#define MODEL_ID_DEFAULT                    0           //!< [-]
// if the rddrone-bms772 board with the MC33772B chip
#if (BOARD_NAME == RDDRONE_BMS772_BOARD)
#   define MODEL_NAME_DEFAULT               "BMS772\0"    //!< [-]
#else
// if the mr-bms771 board with the MC33771C chip
#   define MODEL_NAME_DEFAULT               "BMS771\0"    //!< [-]
#endif

#define CYPHAL_NODE_STATIC_ID_DEFAULT   255        //!< [-]
#define CYPHAL_ES_SUB_ID_DEFAULT        4096       //!< [-]
#define CYPHAL_BS_SUB_ID_DEFAULT        4097       //!< [-]
#define CYPHAL_BP_SUB_ID_DEFAULT        4098       //!< [-]
#define CYPHAL_LEGACY_BI_SUB_ID_DEFAULT UINT16_MAX //!< [-]

#define DRONECAN_NODE_STATIC_ID_DEFAULT 255 //!< [-]
#define DRONECAN_BAT_CONTINUOUS_DEFAULT 0   //!< [-]
#define DRONECAN_BAT_PERIODIC_DEFAULT   0   //!< [-]
#define DRONECAN_BAT_CELLS_DEFAULT      0   //!< [-]
#define DRONECAN_BAT_INFO_DEFAULT       1   //!< [-]
#define DRONECAN_BAT_INFO_AUX_DEFAULT   1   //!< [-]

#define CAN_MODE_DEFAULT                    CAN_MODE_OFF  //!< [-]
#define CAN_FD_MODE_DEFAULT                 0           //!< [-]
#define CAN_BITRATE_DEFAULT                 1000000     //!< [bit/s]
#define CAN_FD_BITRATE_DEFAULT              4000000     //!< [bit/s]

// if the rddrone-bms772 board with the MC33772B chip
#if (BOARD_NAME == RDDRONE_BMS772_BOARD)
#   define V_MIN_DEFAULT                    6           //!< [v]
#   define V_MAX_DEFAULT                    26          //!< [v]
#   define I_SLEEP_OC_DISCHARGE_DEFAULT     30          //!< [mA]
#   define I_SLEEP_OC_CHARGE_DEFAULT        30          //!< [mA]
#   define I_SLEEP_OC_WAKEUP_DEFAULT        20          //!< [mA]
#   define I_SHORT_DEFAULT                  500         //!< [A]
#   define I_BAL_DEFAULT                    50          //!< [mA]
#   define R_BAL_DEFAULT                    82          //!< [Ω]
#   define F_V_OUT_DIVIDER_FACTOR_DEFAULT   11.0        //!< [-]
// if the mr-bms771 board with the MC33771C chip
#else
#   define V_MIN_DEFAULT                    15          //!< [v]
#   define V_MAX_DEFAULT                    59          //!< [v]
#   define I_SLEEP_OC_DISCHARGE_DEFAULT     70          //!< [mA]
#   define I_SLEEP_OC_CHARGE_DEFAULT        0           //!< [mA]
#   define I_SLEEP_OC_WAKEUP_DEFAULT        20          //!< [mA]
#   define I_SHORT_DEFAULT                  210         //!< [A]
#   define I_BAL_DEFAULT                    61          //!< [mA]
#   define R_BAL_DEFAULT                    68          //!< [Ω]
#   define F_V_OUT_DIVIDER_FACTOR_DEFAULT   18.857      //!< [-]
#endif
#define I_RANGE_MAX_DEFAULT                 300         //!< [A]
#define I_MAX_DEFAULT                       60          //!< [A]
#define T_SHORT_DEFAULT                     20          //!< [us]
#define M_MASS_DEFAULT                      0           //!< [kg]


#define CAN_MODE_LEN                        10          //!< [-]

/*******************************************************************************
 * Types
 ******************************************************************************/

//! @brief Enumuration for the main states
typedef enum
{
    SELF_TEST,
    INIT,
    NORMAL,
    CHARGE,
    SLEEP,
    OCV,
    FAULT_ON,
    FAULT_OFF,
    SELF_DISCHARGE,
    DEEP_SLEEP,
    NUMBER_OF_MAIN_STATES
} states_t;

//! @brief define the charge state enum
typedef enum
{
    CHARGE_START,
    CHARGE_CB,
    RELAXATION,
    CHARGE_COMPLETE,
    NUMBER_OF_CHARGE_STATES
} charge_states_t;

/*! @brief  this enum consists of each variable name, can be used to get or set variables
 *          if this enum changes, it will automatically change the cli.c gGetSetParameters string array
 *          this define will be used to generate an enum (to get the data) and string values of that enum (for
 *          the cli) this should consist of each of the BMSParameterValues_t value names in capitals
 * @warning if this enum changes, change the s_parameters struct and the data_setDefaultParameters function in
 *          data.c and add the default, min and max value, add it to the parametersTypes array in cli.c and
 *          parameterUnits array in data.c!
 */
typedef enum
{
    V_OUT,                    /*0*/
    V_BATT,
    N_CELLS,
    V_CELL1,
    V_CELL2,
    V_CELL3,
    V_CELL4,
    V_CELL5,
    V_CELL6,
    V_CELL7,
    V_CELL8,                  /*10*/
    V_CELL9,
    V_CELL10,
    V_CELL11,
    V_CELL12,
    V_CELL13,
    V_CELL14,
    I_BATT,
    I_BATT_AVG,
    I_BATT_10S_AVG,
    SENSOR_ENABLE,            /*20*/
    C_BATT,
    C_AFE,
    C_T,
    C_R,

    P_AVG,
    E_USED,
    T_FULL,
    A_REM,
    A_FULL,
    A_FACTORY,                /*30*/
    S_CHARGE,
    S_HEALTH,
    S_OUT,
    S_IN_FLIGHT,
    BATT_ID,

    V_CELL_OV,
    V_CELL_UV,
    V_CELL_NOMINAL,
    V_STORAGE,
    V_CELL_MARGIN,            /*40*/
    V_RECHARGE_MARGIN,
    I_PEAK_MAX,
    I_OUT_MAX,
    I_OUT_NOMINAL,
    I_FLIGHT_MODE,
    I_SLEEP_OC_DISCHARGE,
    I_SLEEP_OC_CHARGE,
    I_SLEEP_OC_WAKEUP,
    I_SYSTEM,
    I_CHARGE_MAX,             /*50*/
    I_CHARGE_NOMINAL,
    I_CHARGE_FULL,
    C_CELL_OT,
    C_CELL_UT,
    C_PCB_OT,
    C_PCB_UT,
    C_CELL_OT_CHARGE,
    C_CELL_UT_CHARGE,
    N_CHARGES,
    N_CHARGES_FULL,           /*60*/
    OCV_SLOPE,
    BATTERY_TYPE,

    T_MEAS,
    T_FTTI,
    T_BMS_TIMEOUT,
    T_FAULT_TIMEOUT,
    T_BCC_SLEEP_CYCLIC,
    T_SLEEP_TIMEOUT,
    T_OCV_CYCLIC0,
    T_OCV_CYCLIC1,            /*70*/
    T_CHARGE_DETECT,
    T_CB_DELAY,
    T_CHARGE_RELAX,
    BATT_EOL,
    S_FLAGS,
    SELF_DISCHARGE_ENABLE,
    FLIGHT_MODE_ENABLE,
    EMERGENCY_BUTTON_ENABLE,
    SMBUS_ENABLE,
    GATE_CHECK_ENABLE,        /*80*/
    MODEL_ID,
    MODEL_NAME,

    CYPHAL_NODE_STATIC_ID,
    CYPHAL_ES_SUB_ID,
    CYPHAL_BS_SUB_ID,
    CYPHAL_BP_SUB_ID,
    CYPHAL_LEGACY_BI_SUB_ID,
    DRONECAN_NODE_STATIC_ID,
    DRONECAN_BAT_CONTINUOUS,
    DRONECAN_BAT_PERIODIC,    /*90*/
    DRONECAN_BAT_CELLS,
    DRONECAN_BAT_INFO,
    DRONECAN_BAT_INFO_AUX,
    CAN_MODE,
    CAN_FD_MODE,
    CAN_BITRATE,
    CAN_FD_BITRATE,

    V_MIN,
    V_MAX,
    I_RANGE_MAX,              /*100*/
    I_MAX,
    I_SHORT,
    T_SHORT,
    I_BAL,
    R_BAL,
    M_MASS,
    F_V_OUT_DIVIDER_FACTOR,   /*107*/
    NONE,                     /*108*/ /* needs to be last! */
} parameterKind_t;

// TODO make output_status with a bool, but this needs to be made in data.c as well

/*! @brief  Union for different variables types.
 */
typedef union
{
    float    floatVar;  //!< The floating point variable type
    uint8_t  uint8Var;  //!< The uint8 variable
    uint16_t uint16Var; //!< The uint16 variable
    int16_t  int16Var;  //!< The int16 variable
    int32_t  int32Var;  //!< The int32 variable
    bool     boolVar;   //!< The bool variable
} variableTypes_u;

/*! @brief  Union to either address the cell voltages via the arrar or
 *          Address the values individially via .value.
 */
typedef union
{
    float               V_cellArr[14];                       //!< [V] the cell voltage (0 = cell 1 through 14)
    struct
    {
        float           V_cell1;                            //!< [V] the voltage of cell 1
        float           V_cell2;                            //!< [V] the voltage of cell 2
        float           V_cell3;                            //!< [V] the voltage of cell 3
        float           V_cell4;                            //!< [V] the voltage of cell 4
        float           V_cell5;                            //!< [V] the voltage of cell 5
        float           V_cell6;                            //!< [V] the voltage of cell 6
        float           V_cell7;                            //!< [V] the voltage of cell 7
        float           V_cell8;                            //!< [V] the voltage of cell 8
        float           V_cell9;                            //!< [V] the voltage of cell 9
        float           V_cell10;                           //!< [V] the voltage of cell 10
        float           V_cell11;                           //!< [V] the voltage of cell 11
        float           V_cell12;                           //!< [V] the voltage of cell 12
        float           V_cell13;                           //!< [V] the voltage of cell 13
        float           V_cell14;                           //!< [V] the voltage of cell 14
    } value;
} cellVoltages_u;

/*! @brief  This struct consists of the common battery variables that are measured
 *          Or the variables are needed for the measurement.
 * @warning if this changes, change FOR_EACH_PARAMETER aswell!!
 */
typedef struct
{
    float          V_out;          //!< [V] the voltage of the BMS output
    float          V_batt;         //!< [V] the voltage of the battery pack
    uint8_t        N_cells;        //!< [-] number of cells used in the BMS board
    cellVoltages_u V_cellVoltages; //!< [V] the cell voltage (0 = cell 1 through 56)
    float          I_batt;         //!< [A] the last recorded current of the battery
    float          I_batt_avg;     //!< [A] the average current since the last measurement (period T_meas (default 1s))
    float          I_batt_10s_avg; //!< [A] the 10s rollling average current, updated each 1s with T_meas 1000 (ms)
    uint8_t        sensor_enable;  //!< [-] This variable is used to enable or disable the battery temperature sensor, 0 is disabled
    float          C_batt;         //!< [C] the temperature of the external battey temperature sensor
    float          C_AFE;          //!< [C] the temperature of the analog front end
    float          C_T;            //!< [C] the temperature of the transitor
    float          C_R;            //!< [C] the temperature of the sense resistor
} commonBatteryVariables_t;

/*! @brief  This struct consists of the calculated battery variables
 * @warning if this changes, change FOR_EACH_PARAMETER aswell!!
 */
typedef struct
{
    float   P_avg;       //!< [W] average power consumption over the last 10 seconds
    float   E_used;      //!< [Wh] power consumption since device boot
    float   t_full;      //!< [h] charging is expected to complete in this time; zero if not charging
    float   A_rem;       //!< [Ah] remaining capacity in the battery
    float   A_full;      //!< [Ah] predicted battery capacity when it is fully charged. falls with aging, full charge capacity
    float   A_factory;   //!< [Ah] battery capacity stated by the factory
    uint8_t s_charge;    //!< [%] state of charge, precent of hte full charge [0, 100]. this field is required.
    uint8_t s_health;    //!< [%] state of health, health of the battery in percentage use S_HEALTH_UNKNOWN = 127 if cannot be estimated
    uint8_t s_out;       //!< [-] this is true if the output power is enabled
    uint8_t s_in_flight; //!< [-] this is true if the system thinks it is in flight (with flight-mode-enable and i-flight-mode)
    uint8_t batt_id;     /*!< [-] identifies the battery within this vehicle, 0 = unused, 1 = 1st battery, 2 = 2nd
                          *   battery for BatteryContinuous slot_id e.g. 0 - primariy battery for BatteryInfoAux
                          *   battery_id */
} calcBatteryVariables_t;


/*! @brief this struct consists of the additional batteries variables
 * @warning if this changes, change FOR_EACH_PARAMETER aswell!!
 */
typedef struct
{
    float    V_cell_ov;             //!< [V] battery maximum allowed voltage for one cell. exceeding this voltage, the battery will go to fault mode.
    float    V_cell_uv;             //!< [V] Battery minimum allowed voltage for one cell. Going below this voltage, the BMS will go to fault (maybe deep_sleep) mode
    float    V_cell_nominal;        //!< [V] Battery nominal voltage for one cell. will be used for energy calculation.
    float    V_storage;             //!< [V] The voltage what is specified as storage voltage for a cell
    uint8_t  V_cell_margin;         //!< [mV] Cell voltage charge margin to decide or not to go through another topping charge cycle
    uint16_t V_recharge_margin;     //!< [mV] Cell voltage charge complete margin to decide or not to do a battery re-charge, to keep the cell voltages at max this much difference with the cell-ov
    float    I_peak_max;            //!< [A] Maximum peak current threshold to open the switch during normal operation, can't be overrulled
    float    I_out_max;             //!< [A] Maximum average current threshold to open the switch during normal operation, if not overrulled
    float    I_out_nominal;         //!< [A] Nominal discharge current (informative only)
    uint8_t  I_flight_mode;         //!< [A] current threshold to not disable the power in flight mode
    uint8_t  I_sleep_oc_discharge;  //!< [mA] overcurrent threshold detection to detect the battery is in use
    uint8_t  I_sleep_oc_charge;     //!< [mA] overcurrent threshold detection to detect the battery is being charged
    uint8_t  I_sleep_oc_wakeup;     //!< [mA] overcurrent threshold detection to detect the battery needs to wake up from the sleep mode
    uint8_t  I_system;              //!< [mA] Current of the BMS board itself, this is measured (as well) during charging, so this needs to be substracted
    float    I_charge_max;          //!< [A] Maximum current threshold to open the switch during charging
    float    I_charge_nominal;      //!< [A] Nominal charge current (informative only)
    uint16_t I_charge_full;         //!< [mA] Current threshold to detect end of charge sequence
    float    C_cell_ot;             //!< [C] Overtemperature threshold for the Cells during discharging. Going over this threshold and the battery will go to FAULT_ON mode
    float    C_cell_ut;             //!< [C] Under temperature threshold for the Cells. Going under this threshold and the battery will go to FAULT_ON mode
    float    C_pcb_ot;              //!< [C] PCB Ambient temperature over temperature threshold
    float    C_pcb_ut;              //!< [C] PCB Ambient temperature under temperature threshold
    float    C_cell_ot_charge;      //!< [C] Overtemperature threshold for the Cells during charging. Going over this hreshold and the battery will go to FAULT_ON mode
    float    C_cell_ut_charge;      //!< [C] Under temperature threshold for the Cells during charging. Going under this threshold during charging and the battery will go to FAULT_ON mode
    uint16_t N_charges;             //!< [-] the number of charges done
    uint16_t N_charges_full;        //!< [-] the number of complete charges
    float    ocv_slope;             //!< [mV/A.min] The slope of the OCV curve
    uint8_t  battery_type;          /*!< [-] The type of battery attached to it. 0 = LiPo, 1 = LiFePo4, 2 = LiFeYPo4, 3 = NMC (LiPo LiNiMnCoO2), 4 = sodium-ion (Na-ion, SIB).
                                        Could be extended. Will change OV, UV, v-storage, OCV/SoC table if changed runtime.*/
} additionalBatteryVariables_t;


/*! @brief  this struct consists of the configuration parameters
 * @warning if this changes, change FOR_EACH_PARAMETER aswell!!
 */
typedef struct
{

    uint16_t t_meas;                       //!< [ms] cycle of the battery to perform a complete battery measurement and SOC estimation can only be 10000 or a whole division of 10000 (For example: 5000, 1000, 500)
    uint16_t t_ftti;                       //!< [ms] cycle of the battery to perform diagnostics (Fault Tolerant Time Interval)
    uint16_t t_bms_timeout;                //!< [s] Timeout for the BMS to go to SLEEP mode when the battery is not used.
    uint16_t t_fault_timeout;              //!< [s] After this timeout, with an undervoltage fault the battery will go to DEEPSLEEP mode to preserve power. 0 sec is disabled.
    uint8_t  t_bcc_sleep_cyclic;           //!< [s] wake up cyclic timing of the AFE (after front end) during sleep mode
    uint8_t  t_sleep_timeout;              //!< [h] When the BMS is in sleep mode for this period it will go to the self discharge mode, 0 if disabled.
    int32_t  t_ocv_cyclic0;                //!< [s] OCV measurement cyclic timer start (timer is increase by 50% at each cycle)
    int32_t  t_ocv_cyclic1;                //!< [s] OCV measurement cyclic timer final (timer is increase by 50% at each cycle)
    uint8_t  t_charge_detect;              //!< [s] During NORMAL mode, is the battery current is positive for more than this time, then the battery will go to CHARGE mode
    uint8_t  t_cb_delay;                   //!< [s] Time for the cell balancing function to start after entering the CHARGE mode
    uint16_t t_charge_relax;               //!< [s] Relaxation after the charge is complete before going to another charge round.
    uint8_t  batt_eol;                     //!< [%] perentage at which the battery is a bad battery and shouldn't be used typical between 90%-50% default is 80%
    uint8_t  s_flags;                      //!< [-] this contains the status flags as discribed in BMS_status_flags_t
    uint8_t  self_discharge_enable;        //!< [-] This variable is used to enable or disable the SELF_DISCHARGE state, 0 is disabled
    uint8_t  flight_mode_enable;           //!< [-] This variable is used to enable or disable flight mode, is used together with i-flight-mode
    uint8_t  emergency_button_enable;      //!< [-] This variable is used to enable or disable the emergency button on PTE8
    uint8_t  smbus_enable;                 //!< [-] This variable is used to enable or disable the SMBus update.
    uint8_t  gate_check_enable;            //!< [-] This variable is used to enable or disable the gate safety check if it can be turned off, based on output voltage.
    uint64_t model_id;                     //!< [-] set to 0 if not applicable
    char     model_name[STRING_MAX_CHARS]; //!< [-] battery model name, model name is a human-radable string that normally should include the vendor name, model name and chemistry
} configurationVariables_t;


/*! @brief  this struct consists of the CAN variables
 * @warning if this changes, change FOR_EACH_PARAMETER aswell!!
 */
typedef struct
{
    uint8_t  Cyphal_node_static_id;      //!< [-] This is the node ID of the CYPHAL node
    uint16_t Cyphal_es_sub_id;           //!< [-] This is the subject ID of the energy source CYPHAL message (1...100Hz)
    uint16_t Cyphal_bs_sub_id;           //!< [-] This is the subject ID of the battery status CYPHAL message (1Hz)
    uint16_t Cyphal_bp_sub_id;           //!< [-] This is the subject ID of the battery parameters CYPHAL message (0.2Hz)
    uint16_t Cyphal_legacy_bi_sub_id;    //!< [-] This is the subject ID of the battery info legacy CYPHAL message (0.2 ~ 1Hz)
    uint8_t  DroneCAN_node_static_id;    //!< [-] This is the node ID of the DRONECAN node, 255 means dynamic node id
    uint8_t  DroneCAN_bat_continuous;    //!< [-] This indicates if the particular DroneCAN topic has to be published
    uint8_t  DroneCAN_bat_periodic;      //!< [-] This indicates if the particular DroneCAN topic has to be published
    uint8_t  DroneCAN_bat_cells;         //!< [-] This indicates if the particular DroneCAN topic has to be published
    uint8_t  DroneCAN_bat_info;          //!< [-] This indicates if the particular DroneCAN topic has to be published
    uint8_t  DroneCAN_bat_info_aux;      //!< [-] This indicates if the particular DroneCAN topic has to be published
    char     can_mode[STRING_MAX_CHARS]; //!< [-] Options "OFF", "DRONECAN" and "CYPHAL". To indicate which is used.
    uint8_t  can_fd_mode;                //!< [-] If true CANFD is used, otherwise classic CAN is used, only during startup check!
    int32_t  can_bitrate;                //!< [bit/s] the bitrate of classical can or CAN FD arbitratration bitrate
    int32_t  can_fd_bitrate;             //!< [bit/s] the bitrate of CAN FD data bitrate
} canVariables_t;


/*! @brief   this struct contains the hardware variables (these are dependent by the hardware)
 * @warning  if this changes, change FOR_EACH_PARAMETER aswell!!
 */
typedef struct
{
    uint8_t  V_min;                  //!< [V] Minimum stack voltage for the BMS board to be fully functional
    uint8_t  V_max;                  //!< [V] Maximum stack voltage allowed by the BMS board
    uint16_t I_range_max;            //!< [A] Maximum current that can be measured by the BMS board
    uint8_t  I_max;                  //!< [A] Maximum DC current allowed in the BMS board (limited by power dissipation in the MOSFETs)
    uint16_t I_short;                //!< [A] short circuit current threshold
    uint8_t  t_short;                //!< [us] Blanking time for the short circuit detection
    uint8_t  I_bal;                  //!< [mA] Cell balancing current under 4.2V with cell balancing resistors of R-bal
    uint8_t  R_bal;                  //!< [Ω] The balance resistor (sum) used for balancing
    float    m_mass;                 //!< [kg] The total mass of the (smart) battery
    float    f_v_out_divider_factor; //!< [-] The factor of the output voltage divider as component tolerances could be different to not result in the theoretical value.
} hardwareVariables_t;

/*! @brief  this struct contains all the variables
 */
typedef struct
{
    commonBatteryVariables_t     commonBatteryVariables; //!< the most common used battery variables
    calcBatteryVariables_t       calcBatteryVariables;   //!< the calculated battery variables plus the ID
    additionalBatteryVariables_t additionalVariables;    //!< the additional battery variables
    configurationVariables_t     configurationVariables; //!< the configuration parameters
    canVariables_t               canVariables;           //!< the variables needed for the (UAV)CAN part
    hardwareVariables_t          hardwareVariables;      //!< the hardware parameters
} BMSParameterValues_t;

/*! @brief      union to be used with the max and min value
    @warning    for integers, INT32_MAX is the maximum value to check on and INT32_MIN the minimum value
 */
typedef union
{
    uint8_t  U8;  //!< the uint8 value
    uint16_t U16; //!< the uint16 value
    int32_t  I32; //!< the int32 value
    // int16_t       FX10; //!< the float x 10  and int16 value
    // int64_t       I64F; //!< the 64 bit value and the float value
    float FLTVAL; //!< the float value

} types_t;

/*! @brief  enum to define what data type it is
 */
typedef enum
{
    UINT8VAL,  //!< it is a uint8
    UINT16VAL, //!< it is a uint16
    INT32VAL,  //!< it is an int32
    UINT64VAL, //!< it is a uint64
    FLOATVAL,  //!< it is a float value
    STRINGVAL  //!< it is a string value (character pointer)
    // BOOLVAL //!< it is a bool
} valueType_t;

/*! @brief  enum to define what data type it is
 */
typedef struct
{
    valueType_t type;          //!< the type of the parameter
    bool        checkMax;      //!< if there is a maximum limit
    bool        checkMin;      //!< if there is a maximum limit
    bool        userReadOnly;  //!< if true, user may not adjust this.
    types_t     max;           //!< the maximum value of the parameter
    types_t     min;           //!< the minimum value of the parameter
    types_t     defaultVal;    //!< the default value
    void *      parameterAdr;  //!< the address of the parameter
    char *      parameterUnit; //!< the unit of the parameter as a string if any, "-" otherwise
    char *      parameterType; //!< the parameter type of the variable in a sting
} BMSparametersInfo_t;


/*! @brief  this struct contains all the parameter values and information of each parameter in an array
 */
typedef struct
{
    BMSparametersInfo_t  parametersInfo[NONE]; //!< the BMS parameters info
    BMSParameterValues_t parameters;           //!< the BMS parameter
} BMSparameters_t;

/*! @brief these are the possible transition commands */
typedef enum
{
    CMD_NONE           = 0,
    CMD_GO_2_SLEEP     = 1,
    CMD_WAKE           = 2,
    CMD_GO_2_DEEPSLEEP = 3,
    CMD_RESET          = 4,
    CMD_ERROR
} stateCommands_t;

/*******************************************************************************
 * Functions
 ******************************************************************************/

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* BMS_DATA_TYPES_H_ */
