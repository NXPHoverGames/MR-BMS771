/****************************************************************************
 * nxp_bms/BMS_v1/src/cli.c
 *
 * Copyright 2020-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <arch/board/board.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sched.h>
#include <ctype.h>
#include <time.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <inttypes.h>
#include <assert.h>

#include "data.h"
#include "power.h"
#include "cli.h"

#include <nuttx/vt100.h>

/****************************************************************************
 * Defines
 ****************************************************************************/

#define HELP_COMMAND        "help"
#define GET_COMMAND         "get"
#define SET_COMMAND         "set"
#define SHOW_COMMAND        "show"
#define RESET_COMMAND       "reset"
#define SLEEP_COMMAND       "sleep"
#define WAKE_COMMAND        "wake"
#define DEEP_SLEEP_COMMAND  "deepsleep"
#define SAVE_COMMAND        "save"
#define LOAD_COMMAND        "load"
#define DEFAULT_COMMAND     "default"
#define TIME_COMMAND        "time"
#define AMOUNT_COMMANDS     12
#define PARAMS_COMMAND      "parameters"
#define SHOW_MEAS_COMMAND   "show-meas"
#define SHOW_CURRENT        "i-batt"
#define SHOW_AVG_CURRENT    "i-avg"
#define SHOW_CELL_VOLTAGE   "v-cell"
#define SHOW_STACK_VOLTAGE  "v-batt"
#define SHOW_BAT_VOLTAGE    "v-out"
#define SHOW_OUTPUT_STATUS  "s-out"
#define SHOW_TEMPERATURE    "c-bms"
#define SHOW_ENGERGY_COMS   "e-used"
#define SHOW_REMAINING_CAP  "a-rem"
#define SHOW_STATE_O_CHARGE "s-charge"
#define SHOW_AVG_POWER      "p-avg"
#define SHOW_STATE          "state"
#define SHOW_ALL            "all"
#define SHOW_TOP            "top"

// because a measured update sequence (bms show top 1 and bms show all 1) takes 42ms, max wait time will be
// 100ms
#define CLI_TIMED_LOCK_WAIT_TIME_MS 500
#define MS_TO_NS_MULT               1000000
#ifndef NSEC_MAX
#    define NSEC_MAX 999999999
#endif

/****************************************************************************
 * Types
 ****************************************************************************/
/*! @brief Enum to print the text in different colors */
typedef enum
{
    RED,
    GREEN,
    YELLOW
} printColor_t;

/*! @brief these are the possible show commands */
typedef enum
{
    CLI_CURRENT         = 0,  //!< the user wants to see the CURRENT after the cyclic measurement
    CLI_AVG_CURRENT     = 1,  //!< the user wants to see the AVG_CURRENT after the cyclic measurement
    CLI_CELL_VOLTAGE    = 2,  //!< the user wants to see the CELL_VOLTAGE after the cyclic measurement
    CLI_STACK_VOLTAGE   = 3,  //!< the user wants to see the STACK_VOLTAGE after the cyclic measurement
    CLI_BAT_VOLTAGE     = 4,  //!< the user wants to see the BAT_VOLTAGE after the cyclic measurement
    CLI_OUTPUT_STATUS   = 5,  //!< the user wants to see the OUTPUT_STATUS after the cyclic measurement
    CLI_TEMPERATURE     = 6,  //!< the user wants to see the TEMPERATURE after the cyclic measurement
    CLI_ENERGY_CONSUMED = 7,  //!< the user wants to see the ENERGY_CONSUMED after the cyclic measurement
    CLI_REMAINING_CAP   = 8,  //!< the user wants to see the REMAINING_CAP after the cyclic measurement
    CLI_STATE_OF_CHARGE = 9,  //!< the user wants to see the STATE_OF_CHARGE after the cyclic measurement
    CLI_AVG_POWER       = 10, //!< the user wants to see the AVG_POWER after the cyclic measurement
    CLI_STATE           = 11, //!< the user wants to see the main state afther the cyclic measurement
    CLI_ALL             = 12, //!< the user want to see all the measurement
    CLI_TOP             = 13, //!< the user wants to see refreshing with all the measurements
    CLI_NONE            = 14
} showCommands_t;

/****************************************************************************
 * private data
 ****************************************************************************/

// the string array for the states
const char *gStatesArray[] =
{
    [SELF_TEST]                            = "SELF_TEST",
    [INIT]                                 = "INIT",
    [NORMAL]                               = "NORMAL",
    [CHARGE]                               = "CHARGE",
    [SLEEP]                                = "SLEEP",
    [OCV]                                  = "OCV",
    [FAULT_ON]                             = "FAULT_ON",
    [FAULT_OFF]                            = "FAULT_OFF",
    [SELF_DISCHARGE]                       = "SELF_DISCHARGE",
    [DEEP_SLEEP]                           = "DEEP_SLEEP"
};

// the string array for the states
const char *gChargeStatesArray[] =
{
    [CHARGE_START]                                  = "CHARGE_START",
    [CHARGE_CB]                                     = "CHARGE_CB",
    [RELAXATION]                                    = "RELAXATION",
    [CHARGE_COMPLETE]                               = "CHARGE_COMPLETE"
};

// the string array for the parameters
const char *gGetSetParameters[PARAMETER_ARRAY_SIZE] =
{
    [V_OUT]                     = "V_OUT",
    [V_BATT]                    = "V_BATT",
    [N_CELLS]                   = "N_CELLS",
    [V_CELL1]                   = "V_CELL1",
    [V_CELL2]                   = "V_CELL2",
    [V_CELL3]                   = "V_CELL3",
    [V_CELL4]                   = "V_CELL4",
    [V_CELL5]                   = "V_CELL5",
    [V_CELL6]                   = "V_CELL6",
    [V_CELL7]                   = "V_CELL7",
    [V_CELL8]                   = "V_CELL8",
    [V_CELL9]                   = "V_CELL9",
    [V_CELL10]                  = "V_CELL10",
    [V_CELL11]                  = "V_CELL11",
    [V_CELL12]                  = "V_CELL12",
    [V_CELL13]                  = "V_CELL13",
    [V_CELL14]                  = "V_CELL14",
    [I_BATT]                    = "I_BATT",
    [I_BATT_AVG]                = "I_BATT_AVG",
    [I_BATT_10S_AVG]            = "I_BATT_10S_AVG",
    [SENSOR_ENABLE]             = "SENSOR_ENABLE",
    [C_BATT]                    = "C_BATT",
    [C_AFE]                     = "C_AFE",
    [C_T]                       = "C_T",
    [C_R]                       = "C_R",

    [P_AVG]                     = "P_AVG",
    [E_USED]                    = "E_USED",
    [T_FULL]                    = "T_FULL",
    [A_REM]                     = "A_REM",
    [A_FULL]                    = "A_FULL",
    [A_FACTORY]                 = "A_FACTORY",
    [S_CHARGE]                  = "S_CHARGE",
    [S_HEALTH]                  = "S_HEALTH",
    [S_OUT]                     = "S_OUT",
    [S_IN_FLIGHT]               = "S_IN_FLIGHT",
    [BATT_ID]                   = "BATT_ID",

    [V_CELL_OV]                 = "V_CELL_OV",
    [V_CELL_UV]                 = "V_CELL_UV",
    [V_CELL_NOMINAL]            = "V_CELL_NOMINAL",
    [V_STORAGE]                 = "V_STORAGE",
    [V_CELL_MARGIN]             = "V_CELL_MARGIN",
    [V_RECHARGE_MARGIN]         = "V_RECHARGE_MARGIN",
    [I_PEAK_MAX]                = "I_PEAK_MAX",
    [I_OUT_MAX]                 = "I_OUT_MAX",
    [I_OUT_NOMINAL]             = "I_OUT_NOMINAL",
    [I_FLIGHT_MODE]             = "I_FLIGHT_MODE",
    [I_SLEEP_OC_DISCHARGE]      = "I_SLEEP_OC_DISCHARGE",
    [I_SLEEP_OC_CHARGE]         = "I_SLEEP_OC_CHARGE",
    [I_SLEEP_OC_WAKEUP]         = "I_SLEEP_OC_WAKEUP",
    [I_SYSTEM]                  = "I_SYSTEM",
    [I_CHARGE_MAX]              = "I_CHARGE_MAX",
    [I_CHARGE_NOMINAL]          = "I_CHARGE_NOMINAL",
    [I_CHARGE_FULL]             = "I_CHARGE_FULL",
    [C_CELL_OT]                 = "C_CELL_OT",
    [C_CELL_UT]                 = "C_CELL_UT",
    [C_PCB_OT]                  = "C_PCB_OT",
    [C_PCB_UT]                  = "C_PCB_UT",
    [C_CELL_OT_CHARGE]          = "C_CELL_OT_CHARGE",
    [C_CELL_UT_CHARGE]          = "C_CELL_UT_CHARGE",
    [N_CHARGES]                 = "N_CHARGES",
    [N_CHARGES_FULL]            = "N_CHARGES_FULL",
    [OCV_SLOPE]                 = "OCV_SLOPE",
    [BATTERY_TYPE]              = "BATTERY_TYPE",

    [T_MEAS]                    = "T_MEAS",
    [T_FTTI]                    = "T_FTTI",
    [T_BMS_TIMEOUT]             = "T_BMS_TIMEOUT",
    [T_FAULT_TIMEOUT]           = "T_FAULT_TIMEOUT",
    [T_BCC_SLEEP_CYCLIC]        = "T_BCC_SLEEP_CYCLIC",
    [T_SLEEP_TIMEOUT]           = "T_SLEEP_TIMEOUT",
    [T_OCV_CYCLIC0]             = "T_OCV_CYCLIC0",
    [T_OCV_CYCLIC1]             = "T_OCV_CYCLIC1",
    [T_CHARGE_DETECT]           = "T_CHARGE_DETECT",
    [T_CB_DELAY]                = "T_CB_DELAY",
    [T_CHARGE_RELAX]            = "T_CHARGE_RELAX",
    [BATT_EOL]                  = "BATT_EOL",
    [S_FLAGS]                   = "S_FLAGS",
    [SELF_DISCHARGE_ENABLE]     = "SELF_DISCHARGE_ENABLE",
    [FLIGHT_MODE_ENABLE]        = "FLIGHT_MODE_ENABLE",
    [EMERGENCY_BUTTON_ENABLE]   = "EMERGENCY_BUTTON_ENABLE",
    [SMBUS_ENABLE]              = "SMBUS_ENABLE",
    [GATE_CHECK_ENABLE]         = "GATE_CHECK_ENABLE",
    [MODEL_ID]                  = "MODEL_ID",
    [MODEL_NAME]                = "MODEL_NAME",

    [CYPHAL_NODE_STATIC_ID]     = "CYPHAL_NODE_STATIC_ID",
    [CYPHAL_ES_SUB_ID]          = "CYPHAL_ES_SUB_ID",
    [CYPHAL_BS_SUB_ID]          = "CYPHAL_BS_SUB_ID",
    [CYPHAL_BP_SUB_ID]          = "CYPHAL_BP_SUB_ID",
    [CYPHAL_LEGACY_BI_SUB_ID]   = "CYPHAL_LEGACY_BI_SUB_ID",
    [DRONECAN_NODE_STATIC_ID]   = "DRONECAN_NODE_STATIC_ID",
    [DRONECAN_BAT_CONTINUOUS]   = "DRONECAN_BAT_CONTINUOUS",
    [DRONECAN_BAT_PERIODIC]     = "DRONECAN_BAT_PERIODIC",
    [DRONECAN_BAT_CELLS]        = "DRONECAN_BAT_CELLS",
    [DRONECAN_BAT_INFO]         = "DRONECAN_BAT_INFO",
    [DRONECAN_BAT_INFO_AUX]     = "DRONECAN_BAT_INFO_AUX",
    [CAN_MODE]                  = "CAN_MODE",
    [CAN_FD_MODE]               = "CAN_FD_MODE",
    [CAN_BITRATE]               = "CAN_BITRATE",
    [CAN_FD_BITRATE]            = "CAN_FD_BITRATE",

    [V_MIN]                     = "V_MIN",
    [V_MAX]                     = "V_MAX",
    [I_RANGE_MAX]               = "I_RANGE_MAX",
    [I_MAX]                     = "I_MAX",
    [I_SHORT]                   = "I_SHORT",
    [T_SHORT]                   = "T_SHORT",
    [I_BAL]                     = "I_BAL",
    [R_BAL]                     = "R_BAL",
    [M_MASS]                    = "M_MASS",
    [F_V_OUT_DIVIDER_FACTOR]    = "F_V_OUT_DIVIDER_FACTOR",
    //[NONE]                      = "NONE",

    // add these 2 strings to it as extra commands
                                  "STATE",
                                  "ALL"
};

/*! @brief this array consists of all the parameters that are written to the BCC and my not be written in
 * sleep mode
 */
const parameterKind_t BCCParameters[] =
{
    N_CELLS,
    SENSOR_ENABLE,
    V_CELL_OV,
    V_CELL_UV,
    I_SLEEP_OC_WAKEUP,
    C_CELL_OT,
    C_CELL_UT,
    C_PCB_OT,
    C_PCB_UT,
    C_CELL_OT_CHARGE,
    C_CELL_UT_CHARGE,
    BATTERY_TYPE,
    T_BCC_SLEEP_CYCLIC,
};

bool                   gCliPrintLockInitialized = false;
static pthread_mutex_t gCliPrintLock;

static char gGetSetParamsLowerString[32];

static uint16_t gShowMeasurements = 0;
static bool     gDoTop            = false;

//! @brief Callback function to handle a command in the main.c
userCommandCallbackBatFuntion gUserCommandCallbackFuntionfp;
/****************************************************************************
 * private Functions
 ****************************************************************************/
//! to print the help
void printHelp(void);

//! this prints all the parameters from gGetSetParameters
void printParameters(void);

//! this prints all the parameters from FOR_EACH_PARAMETER with the value
void printAllParameterValues(void);

int cli_printfColor(printColor_t color, const char *fmt, va_list argp);

/*!
 * @brief   this function can be used to change what is viewed
 *          on the CLI (the new measurements)
 *
 * @param   showCommand the showcommand that the user entered.
 * @param   value the value of that param.
 *
 * @return  none
 */
void setShowMeas(showCommands_t showCommand, bool value);

/****************************************************************************
 * public Functions
 ****************************************************************************/
/*!
 * @brief   this function is needed to get the address of the state and the mutex to the cli
 *
 * @param   p_userCommandCallbackBatFuntion address of the function to be called when a commands needs to be
 *          processed by the main
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error:
 */
int cli_initialize(userCommandCallbackBatFuntion p_userCommandCallbackBatFuntion)
{
    // initialize the mutex
    pthread_mutex_init(&gCliPrintLock, NULL);
    gCliPrintLockInitialized = true;

    DEBUGASSERT((sizeof(gStatesArray) / sizeof(char *)) == NUMBER_OF_MAIN_STATES);
    DEBUGASSERT((sizeof(gChargeStatesArray) / sizeof(char *)) == NUMBER_OF_CHARGE_STATES);
    DEBUGASSERT(((sizeof(gGetSetParameters) / sizeof(char *)) - (EXTRA_GET_AND_SET_PARS)) == NONE);

    // connect the callback function
    gUserCommandCallbackFuntionfp = p_userCommandCallbackBatFuntion;

    // return
    return OK;
}

/*!
 * @brief   this function is the cli function.
 *          it will take care of the commands send to the BMS
 *          like the "help", "set" and "get" command
 *
 * @param   the amount of arguments in char *argv[]
 * @param   the arguments to the function
 *
 * @return  If successful, the function will return zero (OK). Otherwise -1
 * @example (in main.c) cli_processCommands(argc, argv);
 */
int cli_processCommands(int argc, char **argv)
{
    commands_t     lvCommands     = CLI_WRONG;
    showCommands_t lvShowCommands = CLI_NONE;

    FAR char *lvCommandString   = argv[1];
    FAR char *lvParameterString = argv[2];
    FAR char *lvValueString     = argv[3];

    int         lvRetValue         = -1;
    bool        lvFoundParam       = false;
    bool        lvSendParameters   = false;
    bool        lvSendShowCommands = false;
    bool        lvGetAll           = false;
    int         i, j;
    const char *lvCommandArray[AMOUNT_COMMANDS] = { HELP_COMMAND, GET_COMMAND, SET_COMMAND, SHOW_COMMAND,
        RESET_COMMAND, SLEEP_COMMAND, WAKE_COMMAND, DEEP_SLEEP_COMMAND, SAVE_COMMAND, LOAD_COMMAND,
        DEFAULT_COMMAND, TIME_COMMAND };

    const char *lvShowCommandArgArr[] = { SHOW_CURRENT, SHOW_AVG_CURRENT, SHOW_CELL_VOLTAGE,
        SHOW_STACK_VOLTAGE, SHOW_BAT_VOLTAGE, SHOW_OUTPUT_STATUS, SHOW_TEMPERATURE, SHOW_ENGERGY_COMS,
        SHOW_REMAINING_CAP, SHOW_STATE_O_CHARGE, SHOW_AVG_POWER, SHOW_STATE, SHOW_ALL, SHOW_TOP };

    parameterKind_t lvParameter = NONE;
    float           lvFloatVal;
    char *          lvPStringVal;
    int32_t         lvIntVal    = 0;
    uint64_t        lvUint64Val = 0;
    struct timespec currentTime;
    struct timespec sampleTime;

    // variable to check the MCU power state
    mcuPowerModes_t mcuPowerMode;

    // check if initialized
    if(!gCliPrintLockInitialized)
    {
        cli_printfError("CLI ERROR: isn't initialized, please initialize cli\n");
        return lvRetValue;
    }

    // check which command it is
    switch(argc)
    {
        // only one command
        case 2:
            // check for a help command
            if((!strncmp(lvCommandString, lvCommandArray[HELP_INDEX], strlen(lvCommandArray[HELP_INDEX]))))
            {
                // set the command
                lvCommands = CLI_HELP;
            }
            else if((!strncmp(
                        lvCommandString, lvCommandArray[RESET_INDEX], strlen(lvCommandArray[RESET_INDEX]))))
            {
                // set the command
                lvCommands = CLI_RESET;
            }
            else if((!strncmp(
                        lvCommandString, lvCommandArray[SLEEP_INDEX], strlen(lvCommandArray[SLEEP_INDEX]))))
            {
                // set the command
                lvCommands = CLI_SLEEP;
            }
            else if((!strncmp(
                        lvCommandString, lvCommandArray[WAKE_INDEX], strlen(lvCommandArray[WAKE_INDEX]))))
            {
                // set the command
                lvCommands = CLI_WAKE;
            }
            else if((!strncmp(lvCommandString, lvCommandArray[DEEP_SLEEP_INDEX],
                        strlen(lvCommandArray[DEEP_SLEEP_INDEX]))))
            {
                // set the command
                lvCommands = CLI_DEEP_SLEEP;
            }
            else if((!strncmp(
                        lvCommandString, lvCommandArray[SAVE_INDEX], strlen(lvCommandArray[SAVE_INDEX]))))
            {
                // set the command
                lvCommands = CLI_SAVE;
            }
            else if((!strncmp(
                        lvCommandString, lvCommandArray[LOAD_INDEX], strlen(lvCommandArray[LOAD_INDEX]))))
            {
                // set the command
                lvCommands = CLI_LOAD;
            }
            else if((!strncmp(lvCommandString, lvCommandArray[DEFAULT_INDEX],
                        strlen(lvCommandArray[DEFAULT_INDEX]))))
            {
                // set the command
                lvCommands = CLI_DEFAULT;
            }
            else if((!strncmp(
                        lvCommandString, lvCommandArray[TIME_INDEX], strlen(lvCommandArray[TIME_INDEX]))))
            {
                // set the command
                lvCommands = CLI_TIME;
            }

            break;

        // two commands
        case 3:
            // check for a get command
            if((!strncmp(lvCommandString, lvCommandArray[GET_INDEX], strlen(lvCommandArray[GET_INDEX]))))
            {
                // set the command
                lvCommands = CLI_GET;
            }

            // check for help parameters command
            else if((!strncmp(
                        lvCommandString, lvCommandArray[HELP_INDEX], strlen(lvCommandArray[HELP_INDEX]))))
            {
                // set the command
                lvCommands = CLI_HELP;

                if((!strcmp(lvParameterString, PARAMS_COMMAND))) //, strlen(PARAMS_COMMAND))))
                {
                    // set it to true
                    lvSendParameters = true;
                }
                else if((!strcmp(lvParameterString, SHOW_MEAS_COMMAND))) //, strlen(SHOW_MEAS_COMMAND))))
                {
                    // set the variable
                    lvSendShowCommands = true;
                }
                else
                {
                    // if wrong third command
                    // reset the commands
                    lvCommands = CLI_WRONG;
                }
            }

            break;

        // three commands
        case 4:
            // check for a set command
            if((!strncmp(lvCommandString, lvCommandArray[SET_INDEX], strlen(lvCommandArray[SET_INDEX]))))
            {
                // set the command
                lvCommands = CLI_SET;
            }

            // check for the show command
            else if((!strncmp(
                        lvCommandString, lvCommandArray[SHOW_INDEX], strlen(lvCommandArray[SHOW_INDEX]))))
            {
                // set the command
                lvCommands = CLI_SHOW;
            }
            break;

        // too little or too much commands
        default:
            // there is an error
            cli_printfError("Wrong input!\t");
            cli_printf("try \"bms help\"\n");
            return lvRetValue;
            break;
    }

    // do somthing with the different commands
    switch(lvCommands)
    {
        // in case of a wrong input (should've already be filtered out)
        case CLI_WRONG:
            // there is an error
            cli_printfError("Wrong input!\t");
            cli_printf("try \"bms help\"\n");
            break;
        // in case the user wants the help
        case CLI_HELP:

            // print the help
            if(lvSendParameters)
            {
                printParameters();
            }
            else if(lvSendShowCommands)
            {
                // print the show commands
                for(i = 0; i < CLI_NONE; i++)
                {
                    // print them
                    cli_printf("%s \n", lvShowCommandArgArr[i]);
                }
            }
            else
            {
                printHelp();
            }

            lvRetValue = 0;
            break;

        // in case the user wants to get or set a parameter
        case CLI_SET:
        case CLI_GET:

            lvRetValue = 0;
            // go through the string array
            for(i = 0; i < (PARAMETER_ARRAY_SIZE); i++)
            {
                // change the get set parameter to lowercase and convert all underscores to upderscores
                // change each character of the string parameters to lowercase for the user input
                for(j = 0; (gGetSetParameters[i])[j]; j++)
                {
                    // change it to lowercase
                    gGetSetParamsLowerString[j] = tolower((gGetSetParameters[i])[j]);

                    // convert all underscores to upderscores
                    // find the underscores
                    if(((gGetSetParameters[i])[j]) == '_')
                    {
                        // change to upperscore
                        gGetSetParamsLowerString[j] = '-';
                    }
                }

                // add the null terminator to the string
                gGetSetParamsLowerString[j] = '\0';

                // compare the string
                if((strncmp(lvParameterString, gGetSetParamsLowerString, strlen(gGetSetParameters[i])) ==
                       0) &&
                    (strlen(lvParameterString) == strlen(gGetSetParameters[i])))
                {
                    // check if the parameter can be set
                    if(i <= NONE)
                    {
                        // save the parameter
                        lvParameter  = (parameterKind_t)(i);
                        lvFoundParam = true;
                        break;
                    }

                    // it is ALL
                    // check if it is GET
                    if(lvCommands == CLI_GET)
                    {
                        // set the get all parameter true
                        lvGetAll = true;

                        // set the first parameter
                        lvParameter  = 0;
                        lvFoundParam = true;
                        break;
                    }
                }
                else if(i > NONE)
                {
                    // there is an error
                    cli_printf("Wrong parameter input! \ttry \"bms help\"\n");
                }
            }

            // check if get or set
            if(lvCommands == CLI_GET && lvFoundParam)
            {
                if(lvGetAll)
                {
                    // call get all function
                    printAllParameterValues();
                    break;
                }

                // say the standard message
                cli_printf("%s = ", lvParameterString);

                // get the parameter to the user
                if(lvParameter != NONE)
                {
                    // get the parameter from the data sturct
                    switch(data_getType(lvParameter))
                    {
                        // if it is a floating point
                        case FLOATVAL:

                            // get the value
                            if(data_getParameter(lvParameter, &lvFloatVal, NULL) == NULL)
                            {
                                // something went wrong
                                cli_printfError("CLI ERROR: data_getParameter\n");
                            }
                            else
                            {
                                // print it
                                cli_printf("%.3f", lvFloatVal);
                            }
                            break;
                        // if it is a string
                        case STRINGVAL:

                            // get the value
                            lvPStringVal = data_getParameter(lvParameter, NULL, NULL);
                            if(lvPStringVal == NULL)
                            {
                                // something went wrong
                                cli_printfError("CLI ERROR: data_getParameter\n");
                            }
                            else
                            {
                                // print it
                                cli_printf("%s", lvPStringVal);
                            }
                            break;
                        case UINT64VAL:
                            // get the value
                            if(data_getParameter(lvParameter, &lvUint64Val, NULL) == NULL)
                            {
                                // something went wrong
                                cli_printfError("CLI ERROR: data_getParameter\n");
                            }
                            else
                            {
                                // print it
                                cli_printf("%" PRIu64, lvUint64Val);
                                // cli_printf("%"PRIu64"", lvUint64Val);
                            }

                            break;
                        // if it is a integer (max int32_t)
                        default:

                            // get the value
                            if(data_getParameter(lvParameter, &lvIntVal, NULL) == NULL)
                            {
                                // something went wrong
                                cli_printfError("CLI ERROR: data_getParameter\n");
                            }
                            else
                            {
                                // print it
                                cli_printf("%d", lvIntVal);
                            }
                            break;
                    }

                    // check if there is a unit
                    if(*data_getUnit(lvParameter) != '-')
                    {
                        // add the unit
                        cli_printf(" %s\n", data_getUnit(lvParameter));
                    }
                    else
                    {
                        // add the next line
                        cli_printf("\n");
                    }
                }
                else
                // the user wants the state
                {
                    // check if the charge state needs to be outputted as well
                    if(data_getMainState() == CHARGE)
                    {
                        // print the state and the charge state
                        cli_printf("\"%s-%s\"\n", gStatesArray[(int)(data_getMainState())],
                            gChargeStatesArray[(int)(data_getChargeState())]);
                    }
                    else
                    {
                        // output the state
                        cli_printf("\"%s\"\n", gStatesArray[(int)(data_getMainState())]);
                    }
                }
            }
            // if it is a set command
            else if(lvCommands == CLI_SET && lvFoundParam)
            {
                // set the parameter to the user
                if(lvParameter != NONE)
                {
                    // check if the value may be written
                    if(data_getParameterIfUserReadOnly(lvParameter) == 0)
                    {
                        // get the MCU power state and check for an error
                        mcuPowerMode = power_setNGetMcuPowerMode(false, ERROR_VALUE);
                        if(mcuPowerMode == ERROR_VALUE)
                        {
                            // get the error
                            lvRetValue = errno;

                            // error
                            cli_printfError("cli ERROR: Could not get MCU power state 1: %d \n", lvRetValue);
                        }

                        // check if the MCU is in a mode where the BCC SPI(1) is off
                        if((mcuPowerMode == VLPR_MODE) || (mcuPowerMode == ERROR_VALUE))
                        {
                            cli_printf("Some parameters can not be processed in sleep mode\n");
                            cli_printf("Waking up the BMS... \n");

                            // sample the time
                            if(clock_gettime(CLOCK_REALTIME, &sampleTime) == -1)
                            {
                                cli_printfError("CLI ERROR: failed to get sampleTime!\n");
                            }

                            // set the current time to the sample time
                            currentTime.tv_sec = sampleTime.tv_sec;

                            // wake up the BMS
                            gUserCommandCallbackFuntionfp(CLI_WAKE);

                            // get the MCU power state and check for an error
                            mcuPowerMode = power_setNGetMcuPowerMode(false, ERROR_VALUE);
                            if(mcuPowerMode == ERROR_VALUE)
                            {
                                // get the error
                                lvRetValue = errno;

                                // error
                                cli_printfError(
                                    "cli ERROR: Could not get MCU power mode 2: %d \n", lvRetValue);
                            }

                            // wait until the MCU is not in a mode where the BCC spi is off
                            // or a timeout happens (2-3 seconds)
                            while(((mcuPowerMode == VLPR_MODE) || (mcuPowerMode == ERROR_VALUE)) &&
                                ((sampleTime.tv_sec + 3) > currentTime.tv_sec))
                            {
                                // sleep for 1ms
                                usleep(1000);

                                // get the current time
                                if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
                                {
                                    cli_printfError("CLI ERROR: failed to get currentTime!\n");
                                }

                                // get the MCU power state and check for an error
                                mcuPowerMode = power_setNGetMcuPowerMode(false, ERROR_VALUE);
                                if(mcuPowerMode == ERROR_VALUE)
                                {
                                    // get the error
                                    lvRetValue = errno;

                                    // error
                                    cli_printfError(
                                        "cli ERROR: Could not get MCU power mode 3: %d \n", lvRetValue);
                                }
                            }

                            // get the MCU power state and check for an error
                            mcuPowerMode = power_setNGetMcuPowerMode(false, ERROR_VALUE);
                            if(mcuPowerMode == ERROR_VALUE)
                            {
                                // get the error
                                lvRetValue = errno;

                                // error
                                cli_printfError(
                                    "cli ERROR: Could not get MCU power mode 4: %d \n", lvRetValue);
                            }

                            // check if the MCU is in a mode where the BCC SPI(1) is off
                            if((mcuPowerMode == VLPR_MODE) || (mcuPowerMode == ERROR_VALUE))
                            {
                                // inform user and return
                                cli_printfError("CLI ERROR: could not wake up MCU!\n");
                                lvRetValue = -1;

                                // return
                                return lvRetValue;
                            }
                        }

                        // TODO is a password needed?
                        // cli_printf("add password!\n");
                        // check which type the value should be
                        // get the parameter from the data sturct
                        switch(data_getType(lvParameter))
                        {
                            // if it is a floating point
                            case FLOATVAL:
                                // get the value

                                // convert string to float
                                lvFloatVal = strtof(lvValueString, NULL);

                                // inform user
                                cli_printf("setting %s with \"%.3f\"...\n", lvParameterString, lvFloatVal);

                                // check for errors
                                if(lvFloatVal == 0 && (errno == ERANGE || errno == EINVAL))
                                {
                                    // inform user and return
                                    cli_printfError("CLI ERROR: conversion error!\n");
                                    lvRetValue = -1;

                                    // return
                                    return lvRetValue;
                                }

                                // set the parameter and check if failed
                                if(data_setParameter(lvParameter, &lvFloatVal))
                                {
                                    // if failed
                                    cli_printfError("Failed!\tMaybe outside minimum or maximum\n");
                                }
                                else
                                {
                                    // get the parameter
                                    if(data_getParameter(lvParameter, &lvFloatVal, NULL) == NULL)
                                    {
                                        // something went wrong
                                        cli_printfError("CLI ERROR: data_getParameter\n");
                                    }
                                    else
                                    {
                                        // if succeeded
                                        cli_printfGreen("succeeded! %.3f is set!\n", lvFloatVal);
                                    }
                                }

                                break;
                            // if it is a string
                            case STRINGVAL:

                                // inform user
                                cli_printf("setting %s with \"%s\"...\n", lvParameterString, lvValueString);

                                // check if the string lenght is OK
                                if(strlen(lvValueString) > STRING_MAX_CHARS)
                                {
                                    // it is too long
                                    cli_printf(
                                        "input string too long! max characters is %d\n", STRING_MAX_CHARS);
                                    lvRetValue = -1;
                                    break;
                                }

                                // set the parameter and check if failed
                                if(data_setParameter(lvParameter, lvValueString))
                                {
                                    //  if failed
                                    cli_printfError("Failed!\n");
                                }
                                else
                                {
                                    // get the set value
                                    lvPStringVal = data_getParameter(lvParameter, NULL, NULL);

                                    // check for errors
                                    if(lvPStringVal == NULL)
                                    {
                                        // something went wrong
                                        cli_printfError("CLI ERROR: data_getParameter\n");
                                    }
                                    else
                                    {
                                        // output to the user that it worked!
                                        cli_printfGreen("succeeded! %s is set!\n", lvValueString);
                                    }
                                }

                                break;
                            case UINT64VAL:
                                // get the value

                                // check for a negative number
                                if(lvValueString[0] == '-')
                                {
                                    // if failed output to the user
                                    cli_printfError(
                                        "Failed!\tParameter is of type unsigned, no negatives allowed\n");

                                    // break out so the value is not set!
                                    break;
                                }

                                // convert string to float
                                // watch out this returns a long int!
                                lvUint64Val = strtoull(lvValueString, NULL, 10);

                                // inform user
                                cli_printf(
                                    "setting %s with \"%" PRIu64 "\"...\n", lvParameterString, lvUint64Val);

                                // check for errors
                                if(lvUint64Val == 0 && (errno == ERANGE || errno == EINVAL))
                                {
                                    // inform user and return
                                    cli_printfError("CLI ERROR: conversion error!\n");
                                    lvRetValue = -1;

                                    return lvRetValue;
                                }

                                // set the parameter and check if failed
                                if(data_setParameter(lvParameter, &lvUint64Val))
                                {
                                    // if failed
                                    cli_printfError("Failed!\tMaybe outside minimum or maximum\n");
                                }
                                else
                                {
                                    // get the parameter
                                    if(data_getParameter(lvParameter, &lvUint64Val, NULL) == NULL)
                                    {
                                        // something went wrong
                                        cli_printfError("CLI ERROR: data_getParameter\n");
                                    }
                                    else
                                    {
                                        // if succeeded
                                        cli_printfGreen("succeeded! %" PRIu64 " is set!\n", lvUint64Val);
                                    }
                                }

                                break;

                            // if it is a integer (max int32_t)
                            default:
                                // get the value

                                // convert string to int
                                // watch out this returns a long int!
                                lvIntVal = strtol(lvValueString, NULL, 10);

                                // inform user
                                cli_printf("setting %s with \"%d\"...\n", lvParameterString, lvIntVal);

                                // check for errors
                                if(lvIntVal == 0 && (errno == ERANGE || errno == EINVAL))
                                {
                                    // inform user and return
                                    cli_printfError("CLI ERROR: conversion error!\n");
                                    lvRetValue = -1;

                                    return lvRetValue;
                                }

                                // check if it is a UINT8
                                if(data_getType(lvParameter) == UINT8VAL)
                                {
                                    // check for a negative number
                                    if(lvValueString[0] == '-')
                                    {
                                        // if failed output to the user
                                        cli_printfError(
                                            "Failed!\tParameter is of type unsigned, no negatives allowed\n");

                                        // break out so the value is not set!
                                        break;
                                    }

                                    // check if the value is not higher than the max UINT8 value
                                    if(lvIntVal > UINT8_MAX)
                                    {
                                        // if failed output to the user
                                        cli_printfError("Failed!\tParameter does not fit in datatype %d > "
                                                        "UINT8_MAX (%d)\n",
                                            lvIntVal, UINT8_MAX);

                                        // break out so the value is not set!
                                        break;
                                    }
                                }
                                // check if it is a UINT16
                                else if(data_getType(lvParameter) == UINT16VAL)
                                {
                                    // check for a negative number
                                    if(lvValueString[0] == '-')
                                    {
                                        // if failed output to the user
                                        cli_printfError(
                                            "Failed!\tParameter is of type unsigned, no negatives allowed\n");

                                        // break out so the value is not set!
                                        break;
                                    }

                                    // check if the value is not higher than the max UINT8 value
                                    if(lvIntVal > UINT16_MAX)
                                    {
                                        // if failed output to the user
                                        cli_printfError("Failed!\tParameter does not fit in datatype %d > "
                                                        "UINT16_MAX (%d)\n",
                                            lvIntVal, UINT16_MAX);

                                        // break out so the value is not set!
                                        break;
                                    }
                                }

                                // set the parameter and check if failed
                                if(data_setParameter(lvParameter, &lvIntVal))
                                {
                                    // if failed
                                    cli_printfError("Failed!\tMaybe outside minimum or maximum\n");
                                }
                                else
                                {
                                    // get the parameter
                                    if(data_getParameter(lvParameter, &lvIntVal, NULL) == NULL)
                                    {
                                        // something went wrong
                                        cli_printfError("CLI ERROR: data_getParameter\n");
                                    }
                                    else
                                    {
                                        // if succeeded
                                        cli_printfGreen("succeeded! %d is set!\n", lvIntVal);
                                    }
                                }
                                break;
                        }
                    }
                    // if it is one of the parameters that shouldn't be written
                    else
                    {
                        // print that it may not be written
                        cli_printfError("Failed %s may not be written!\n", lvParameterString);
                    }
                }
                // the user wants to set the state (command to go somewhere)
                else
                {
                    // print that it may not be written
                    cli_printfError("Failed %s may not be written!\n", lvParameterString);
                }
            }

            break;

        // in case of time
        case CLI_TIME:
            // get the current time and output it to the user

            // get the current time
            if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
            {
                cli_printfError("CLI ERROR: failed to get currentTime!\n");

                lvRetValue = -1;
                return lvRetValue;
            }

            // output the time to the user
            cli_printf("Time since boot: %ds %dms\n", currentTime.tv_sec, currentTime.tv_nsec / 1000000);

            // it went ok
            lvRetValue = 0;
            break;

        // in case of show
        case CLI_SHOW:

            // check the second parameters
            for(i = 0; i < CLI_NONE; i++)
            {
                // check each
                if(!strncmp(lvParameterString, lvShowCommandArgArr[i], strlen(lvShowCommandArgArr[i])))
                {
                    // found it
                    lvShowCommands = (showCommands_t)i;
                    break;
                }
            }

            // if succeeded
            if(lvShowCommands != CLI_NONE)
            {
                // check the value
                lvIntVal = strtol(lvValueString, NULL, 10);

                // check if the input is OK
                if(lvIntVal == 1 || lvIntVal == 0)
                {
                    // make sure the right things are shown
                    setShowMeas(lvShowCommands, lvIntVal);

                    // output to user
                    if(lvIntVal)
                    {
                        cli_printf("enabled visability for %s\n", lvShowCommandArgArr[lvShowCommands]);
                    }
                    else
                    {
                        cli_printf("disabled visability for %s\n", lvShowCommandArgArr[lvShowCommands]);
                    }
                }
                else
                {
                    cli_printf("wrong value! try \"bms help\"\n");
                }
            }
            else
            {
                cli_printf("wrong show command! try \"bms help show-meas\"\n");
            }

            break;

        // it is an other command
        default:
            // call the callback
            lvRetValue = gUserCommandCallbackFuntionfp(lvCommands);
            break;
    }

    // return
    return lvRetValue;
}

/*!
 * @brief   this function is the same as cli_printf(), but it will make sure it can be used
 *          by multiple threads
 *
 * @param   fmt This is the string that contains the text to be written to stdout.
 *          It can optionally contain embedded format tags that are replaced by
 *          the values specified in subsequent additional arguments and formatted as requested.
 *          Format tags prototype is %[flags][width][.precision][length]specifier
 *
 * @param   the arguments of the function
 *
 * @return  If successful, the total number of characters written is returned.
 *          On failure, a negative number is returned.
 */
int cli_printf(FAR const IPTR char *fmt, ...)
{
    va_list         ap;
    int             lvRetValue;
    struct timespec waitTime;

    // check if mutex is initialzed
    if(gCliPrintLockInitialized)
    {
        // get the time
        if(clock_gettime(CLOCK_REALTIME, &waitTime) == -1)
        {
            cli_printfError("CLI ERROR: failed to get time!\n");
        }

        // add the time
        waitTime.tv_sec += (int)(CLI_TIMED_LOCK_WAIT_TIME_MS / 1000) +
            ((waitTime.tv_nsec + (CLI_TIMED_LOCK_WAIT_TIME_MS % 1000) * MS_TO_NS_MULT) / (NSEC_MAX + 1));
        waitTime.tv_nsec =
            (waitTime.tv_nsec + (CLI_TIMED_LOCK_WAIT_TIME_MS % 1000) * MS_TO_NS_MULT) % (NSEC_MAX + 1);

        // lock the mutex
        lvRetValue = pthread_mutex_timedlock(&gCliPrintLock, &waitTime);

        // check if succesfull
        if(!lvRetValue)
        {
            // initialze variable list ap
            va_start(ap, fmt);

            // do the printf
            lvRetValue = vprintf(fmt, ap);

            // end the variable list
            va_end(ap);

            // unlock the mutex
            pthread_mutex_unlock(&gCliPrintLock);
        }
        else
        {
            // initialze variable list ap
            va_start(ap, fmt);

            // do the printf
            lvRetValue = vprintf(fmt, ap);

            // end the variable list
            va_end(ap);
        }
    }

    else
    {
        // initialze variable list ap
        va_start(ap, fmt);

        // do the printf
        lvRetValue = vprintf(fmt, ap);

        // end the variable list
        va_end(ap);
    }

    // return to the user
    return lvRetValue;
}

/*!
 * @brief   this function is the same as cli_printf(), but it will print the message in red
 *
 * @param   fmt This is the string that contains the text to be written to stdout.
 *          It can optionally contain embedded format tags that are replaced by
 *          the values specified in subsequent additional arguments and formatted as requested.
 *          Format tags prototype is %[flags][width][.precision][length]specifier
 *
 * @param   the arguments of the function
 *
 * @return  If successful, the total number of characters written is returned.
 *          On failure, a negative number is returned.
 */
int cli_printfError(FAR const IPTR char *fmt, ...)
{
    int     lvRetValue;
    va_list ap;

    // initialze variable list ap
    va_start(ap, fmt);

    // do the printf
    lvRetValue = cli_printfColor(RED, fmt, ap);

    // end the variable list
    va_end(ap);

    // return to the user
    return lvRetValue;
}

/*!
 * @brief   this function is the same as cli_printf(), but it will print the message in orange
 *
 * @param   fmt This is the string that contains the text to be written to stdout.
 *          It can optionally contain embedded format tags that are replaced by
 *          the values specified in subsequent additional arguments and formatted as requested.
 *          Format tags prototype is %[flags][width][.precision][length]specifier
 *
 * @param   the arguments of the function
 *
 * @return  If successful, the total number of characters written is returned.
 *          On failure, a negative number is returned.
 */
int cli_printfWarning(FAR const IPTR char *fmt, ...)
{
    int     lvRetValue;
    va_list ap;

    // initialze variable list ap
    va_start(ap, fmt);

    // do the printf
    lvRetValue = cli_printfColor(YELLOW, fmt, ap);

    // end the variable list
    va_end(ap);

    // return to the user
    return lvRetValue;
}

/*!
 * @brief   this function is the same as cli_printf(), but it will print the message in green
 *
 * @param   fmt This is the string that contains the text to be written to stdout.
 *          It can optionally contain embedded format tags that are replaced by
 *          the values specified in subsequent additional arguments and formatted as requested.
 *          Format tags prototype is %[flags][width][.precision][length]specifier
 *
 * @param   the arguments of the function
 *
 * @return  If successful, the total number of characters written is returned.
 *          On failure, a negative number is returned.
 */
int cli_printfGreen(FAR const IPTR char *fmt, ...)
{
    int     lvRetValue;
    va_list ap;

    // initialze variable list ap
    va_start(ap, fmt);

    // do the printf
    lvRetValue = cli_printfColor(GREEN, fmt, ap);

    // end the variable list
    va_end(ap);

    // return to the user
    return lvRetValue;
}

/*!
 * @brief   this function is the same as cli_printf(), but with no mutex lock
 * @warning should be used with cli_printLock()
 *
 * @param   fmt This is the string that contains the text to be written to stdout.
 *          It can optionally contain embedded format tags that are replaced by
 *          the values specified in subsequent additional arguments and formatted as requested.
 *          Format tags prototype is %[flags][width][.precision][length]specifier
 *
 * @param   the arguments of the function
 *
 * @return  If successful, the total number of characters written is returned.
 *          On failure, a negative number is returned.
 */
int cli_printfNoLock(FAR const IPTR char *fmt, ...)
{
    va_list ap;
    int     lvRetValue;

    // initialze variable list ap
    va_start(ap, fmt);

    // do the printf
    lvRetValue = vprintf(fmt, ap);

    // end the variable list
    va_end(ap);

    // return to the user
    return lvRetValue;
}

/*!
 * @brief   this function is the same as cli_printf(), but with a try mutex lock
 * @warning should be used with cli_printLock()
 *
 * @param   fmt This is the string that contains the text to be written to stdout.
 *          It can optionally contain embedded format tags that are replaced by
 *          the values specified in subsequent additional arguments and formatted as requested.
 *          Format tags prototype is %[flags][width][.precision][length]specifier
 *
 * @param   the arguments of the function
 *
 * @return  If successful, the total number of characters written is returned.
 *          On failure, a negative number is returned.
 */
int cli_printfTryLock(FAR const IPTR char *fmt, ...)
{
    va_list ap;
    int     lvRetValue;
    int     mutexState;

    // check if mutex is initialzed
    if(gCliPrintLockInitialized)
    {
        // lock the mutex
        mutexState = pthread_mutex_trylock(&gCliPrintLock);

        // initialze variable list ap
        va_start(ap, fmt);

        // do the printf
        lvRetValue = vprintf(fmt, ap);

        // end the variable list
        va_end(ap);

        // check if the mutex was locked
        if(!mutexState)
        {
            // unlock the mutex
            pthread_mutex_unlock(&gCliPrintLock);
        }
    }
    else
    {
        // initialze variable list ap
        va_start(ap, fmt);

        // do the printf
        lvRetValue = vprintf(fmt, ap);

        // end the variable list
        va_end(ap);
    }

    // return to the user
    return lvRetValue;
}

/*!
 * @brief   this function is used to lock and unlock the cli_print mutex.
 *          it can be used to make sure updating the measured data is done without
 *          writting random things in between.
 *
 * @param   lock when true it will lock, when false it will unlock the cli_print mutex
 *
 * @return  0 If successful, otherwise an error will indicate the error
 */
int cli_printLock(bool lock)
{
    int lvRetValue = -1;
    // int fd;
    // struct termios attributes;

    // check if initialzed
    if(gCliPrintLockInitialized)
    {
        // check if needed to be locked
        if(lock)
        {
            // lock the mutex
            lvRetValue = pthread_mutex_lock(&gCliPrintLock);
        }
        else
        {
            // unlock the mutex
            lvRetValue = pthread_mutex_unlock(&gCliPrintLock);
        }
    }
    else
    {
        cli_printfError("CLI printlock not initialzed!\n");
    }

    return lvRetValue;
}

/*!
 * @brief   this function is used to update the data on the CLI when needed.
 *
 * @param   pCommonBatteryVariables pointer to the commonBatteryVariables_t to update the information
 * @param   pCalcBatteryVariables pointer to the calcBatteryVariables_t to update the information
 *
 * @return  0 If successful, otherwise an error will indicate the error
 */
int cli_updateData(
    commonBatteryVariables_t *pCommonBatteryVariables, calcBatteryVariables_t *pCalcBatteryVariables)
{
    int             lineCounterVal = 0, i;
    variableTypes_u variable1;

    const char gSaveCursor[]    = VT100_SAVECURSOR;
    const char gRestoreCursor[] = VT100_RESTORECURSOR;

    // check for NULL pointer in debug mode
    DEBUGASSERT(pCommonBatteryVariables != NULL);
    DEBUGASSERT(pCalcBatteryVariables != NULL);

    // check if anything needs to be send over the CLI
    if(gShowMeasurements)
    {
        // check if the top command is on
        if(gDoTop)
        {
            // reset the lineCounterVal
            lineCounterVal = 1;

            // Check if the user wants the current
            if(gShowMeasurements & (1 << CLI_CURRENT))
            {
                // save the cursor position
                cli_printf("%s \e[%d;1H \e[2K \ri-batt: \t %8.3f A%s", &gSaveCursor, lineCounterVal,
                    pCommonBatteryVariables->I_batt, &gRestoreCursor);

                // increase the line counter
                lineCounterVal++;
            }

            // Check if the user wants the avg current
            if(gShowMeasurements & (1 << CLI_AVG_CURRENT))
            {
                // print the average current and samples
                cli_printf("%s\e[%d;1H \e[2K \ri-batt-avg: \t %8.3f A%s", &gSaveCursor, lineCounterVal,
                    pCommonBatteryVariables->I_batt_avg, &gRestoreCursor);

                // increase the line counter
                lineCounterVal++;
            }

            // Check if the user wants the cell voltages
            if(gShowMeasurements & (1 << CLI_CELL_VOLTAGE))
            {
                // loop though all the available cells
                for(i = 0; i < pCommonBatteryVariables->N_cells; i++)
                {
                    // get the cell voltage
                    // print the cell voltage
                    cli_printf("%s\e[%d;1H \e[2K \rv-cell%d:\t %8.3f V%s", &gSaveCursor, lineCounterVal,
                        i + 1, (pCommonBatteryVariables->V_cellVoltages).V_cellArr[i], &gRestoreCursor);

                    // increase the line counter
                    lineCounterVal++;
                }
            }

            // Check if the user wants the stack voltage
            if(gShowMeasurements & (1 << CLI_STACK_VOLTAGE))
            {
                // print the battery voltage
                cli_printf("%s\e[%d;1H \e[2K \rv-batt:\t\t %8.3f V%s", &gSaveCursor, lineCounterVal,
                    pCommonBatteryVariables->V_batt, &gRestoreCursor);

                // increase the line counter
                lineCounterVal++;
            }

            // Check if the user wants the battery voltage
            if(gShowMeasurements & (1 << CLI_BAT_VOLTAGE))
            {
                // print the output voltage
                cli_printf("%s\e[%d;1H \e[2K \rv-out: \t\t %8.3f V%s", &gSaveCursor, lineCounterVal,
                    pCommonBatteryVariables->V_out, &gRestoreCursor);

                // increase the line counter
                lineCounterVal++;
            }

            // Check if the user wants the output status
            if(gShowMeasurements & (1 << CLI_OUTPUT_STATUS))
            {
                // print the output status
                cli_printf("%s\e[%d;1H \e[2K \rs-out \t\t\t%d%s", &gSaveCursor, lineCounterVal,
                    pCalcBatteryVariables->s_out, &gRestoreCursor);

                // increase the line counter
                lineCounterVal++;
            }

            // Check if the user wants the temperature
            if(gShowMeasurements & (1 << CLI_TEMPERATURE))
            {
                // Check if the battery sensor is enabled
                if(pCommonBatteryVariables->sensor_enable)
                {
                    // print the battery temperature
                    cli_printf("%s\e[%d;1H \e[2K \rc-batt: \t %8.3f C%s", &gSaveCursor, lineCounterVal,
                        pCommonBatteryVariables->C_batt, &gRestoreCursor);

                    // increase the line counter
                    lineCounterVal++;
                }

                // print the afe temperature
                cli_printf("%s\e[%d;1H \e[2K \rc-afe: \t\t %8.3f C%s", &gSaveCursor, lineCounterVal,
                    pCommonBatteryVariables->C_AFE, &gRestoreCursor);

                // increase the line counter
                lineCounterVal++;

                // print the transistor temperature
                cli_printf("%s\e[%d;1H \e[2K \rc-t: \t\t %8.3f C%s", &gSaveCursor, lineCounterVal,
                    pCommonBatteryVariables->C_T, &gRestoreCursor);

                // increase the line counter
                lineCounterVal++;

                // print the resistor temperature
                cli_printf("%s\e[%d;1H \e[2K \rc-r: \t\t %8.3f C%s", &gSaveCursor, lineCounterVal,
                    pCommonBatteryVariables->C_R, &gRestoreCursor);

                // increase the line counter
                lineCounterVal++;
            }

            // Check if the user wants the energy consumed
            if(gShowMeasurements & (1 << CLI_ENERGY_CONSUMED))
            {
                // print the energy used
                cli_printf("%s\e[%d;1H \e[2K \re-used: \t %8.3f Wh%s", &gSaveCursor, lineCounterVal,
                    pCalcBatteryVariables->E_used, &gRestoreCursor);

                // increase the line counter
                lineCounterVal++;
            }

            // Check if the user wants the remaining capacity
            if(gShowMeasurements & (1 << CLI_REMAINING_CAP))
            {
                // print the remaning capacity
                cli_printf("%s\e[%d;1H \e[2K \ra-rem: \t\t %8.3f Ah%s", &gSaveCursor, lineCounterVal,
                    pCalcBatteryVariables->A_rem, &gRestoreCursor);

                // increase the line counter
                lineCounterVal++;
            }

            // Check if the user wants the state of charge
            if(gShowMeasurements & (1 << CLI_STATE_OF_CHARGE))
            {
                // print the state of charge
                cli_printf("%s\e[%d;1H \e[2K \rs-charge:\t      %3d %%%s", &gSaveCursor, lineCounterVal,
                    pCalcBatteryVariables->s_charge, &gRestoreCursor);

                // increase the line counter
                lineCounterVal++;
            }

            // Check if the user wants the avg power
            if(gShowMeasurements & (1 << CLI_AVG_POWER))
            {
                // print the average power
                cli_printf("%s\e[%d;1H \e[2K \rp-avg: \t\t %8.3f W%s", &gSaveCursor, lineCounterVal,
                    pCalcBatteryVariables->P_avg, &gRestoreCursor);

                // increase the line counter
                lineCounterVal++;
            }

            // check if the user want to state
            if(gShowMeasurements & (1 << CLI_STATE))
            {
                // get the state in a uint8 value
                variable1.uint8Var = (uint8_t)data_getMainState();

                // check if the charge state needs to be outputted as well
                if((states_t)variable1.uint8Var == CHARGE)
                {
                    // get the charge state
                    i = (uint8_t)data_getChargeState();

                    // check if charge relaxation
                    if(i == RELAXATION)
                    {
                        // print the state and the charge state
                        cli_printf("%s\e[%d;1H \e[2K \rstate:  %s_%s%s", &gSaveCursor, lineCounterVal,
                            gStatesArray[variable1.uint8Var], gChargeStatesArray[i], &gRestoreCursor);
                    }
                    // the other states contain the word charge
                    else
                    {
                        // print the charge state
                        cli_printf("%s\e[%d;1H \e[2K \rstate: \t  %*s%s", &gSaveCursor, lineCounterVal, 15,
                            gChargeStatesArray[i], &gRestoreCursor);
                    }
                }
                else
                {
                    // output the state
                    cli_printf("%s\e[%d;1H \e[2K \rstate: \t   %*s%s", &gSaveCursor, lineCounterVal, 14,
                        gStatesArray[variable1.uint8Var], &gRestoreCursor);
                }
            }
        }
        // if top is not on (!gDoTop)
        else
        {
            // print every value on the CLI

            // Check if the user wants the current
            if(gShowMeasurements & (1 << CLI_CURRENT))
            {
                // clear the line and write the value
                cli_printf("\e[2K");

                // print it
                cli_printf("i-batt: \t %8.3f A\n", pCommonBatteryVariables->I_batt);
            }

            // Check if the user wants the avg current
            if(gShowMeasurements & (1 << CLI_AVG_CURRENT))
            {
                // clear the line and write the value
                cli_printf("\e[2K");
                // output the average current if needed
                cli_printf("i-batt-avg: \t %8.3f A\n", pCommonBatteryVariables->I_batt_avg);
            }

            // Check if the user wants the cell voltages
            if(gShowMeasurements & (1 << CLI_CELL_VOLTAGE))
            {
                // loop though all the available cells
                for(i = 0; i < pCommonBatteryVariables->N_cells; i++)
                {
                    // clear the line and write the value
                    cli_printf("\e[2K");

                    // print the cell voltage
                    cli_printf("v-cell%d:\t %8.3f V\n", i + 1,
                        (pCommonBatteryVariables->V_cellVoltages).V_cellArr[i]);
                }
            }

            // Check if the user wants the stack voltage
            if(gShowMeasurements & (1 << CLI_STACK_VOLTAGE))
            {
                // clear the line and write the value
                cli_printf("\e[2K");
                cli_printf("v-batt:\t\t %8.3f V\n", pCommonBatteryVariables->V_batt);
            }

            // Check if the user wants the battery voltage
            if(gShowMeasurements & (1 << CLI_BAT_VOLTAGE))
            {
                // clear the line and write the value
                cli_printf("\e[2K");

                // print it
                cli_printf("v-out: \t\t %8.3f V\n", pCommonBatteryVariables->V_batt);
            }

            // Check if the user wants the output status
            if(gShowMeasurements & (1 << CLI_OUTPUT_STATUS))
            {
                // clear the line and write the value
                cli_printf("\e[2K");
                cli_printf("s-out \t\t\t%d\n", pCalcBatteryVariables->s_out);
            }

            // Check if the user wants the temperature
            if(gShowMeasurements & (1 << CLI_TEMPERATURE))
            {
                // Check if the battery sensor is enabled
                if(pCommonBatteryVariables->sensor_enable)
                {
                    // clear the line and write the value
                    cli_printf("\e[2K");
                    cli_printf("c-batt: \t %8.3f C\n", pCommonBatteryVariables->C_batt);
                }

                // clear the line and write the value
                cli_printf("\e[2K");
                cli_printf("c-afe: \t\t %8.3f C\n", pCommonBatteryVariables->C_AFE);

                // clear the line and write the value
                cli_printf("\e[2K");
                cli_printf("c-t: \t\t %8.3f C\n", pCommonBatteryVariables->C_T);

                // clear the line and write the value
                cli_printf("\e[2K");
                cli_printf("c-r: \t\t %8.3f C\n", pCommonBatteryVariables->C_R);
            }

            // Check if the user wants the energy consumed
            if(gShowMeasurements & (1 << CLI_ENERGY_CONSUMED))
            {
                // clear the line and write the value
                cli_printf("\e[2K");
                cli_printf("e-used: \t %8.3f Wh\n", pCalcBatteryVariables->E_used);
            }

            // Check if the user wants the remaining charge
            if(gShowMeasurements & (1 << CLI_REMAINING_CAP))
            {
                // clear the line and write the value
                cli_printf("\e[2K");
                cli_printf("a-rem: \t\t %8.3f Ah\n", pCalcBatteryVariables->A_rem);
            }

            // Check if the user wants the state of charge
            if(gShowMeasurements & (1 << CLI_STATE_OF_CHARGE))
            {
                // clear the line and write the value
                cli_printf("\e[2K");
                // output the state of charge
                cli_printf("s-charge:\t\t%d %%\n", pCalcBatteryVariables->s_charge);
            }

            // Check if the user wants the average power
            if(gShowMeasurements & (1 << CLI_AVG_POWER))
            {
                // clear the line and write the value
                cli_printf("\e[2K");
                // output the average power
                cli_printf("p-avg: \t\t %8.3f W\n", pCalcBatteryVariables->P_avg);
            }

            // check if the user want to state
            if(gShowMeasurements & (1 << CLI_STATE))
            {
                // get the state in a uint8 value
                variable1.uint8Var = (uint8_t)data_getMainState();

                // check if the charge state needs to be outputted as well
                if((states_t)variable1.uint8Var == CHARGE)
                {
                    // print the state and the charge state
                    cli_printf("state: \t\t    \"%s-%s\"\n", gStatesArray[variable1.uint8Var],
                        gChargeStatesArray[(int)(data_getChargeState())]);
                }
                else
                {
                    // output the state
                    cli_printf("state: \t\t    \"%s\"\n", gStatesArray[variable1.uint8Var]);
                }
            }
        }
    }

    return 0;
}

/*!
 * @brief   this function is used to get the string of a state.
 *
 * @param   getMainState if true, it will get the main state string.
 *          If false, it will get the charge state string.
 * @param   state the state to get the string of, if getMainState is true from the states_t
 *          otherwise from the charge_states_t.
 * @param   dest This is the pointer to the destination array where the content is to be copied.
 *          May be NULL, if NULL, returnvalue is the value of the const string
 *
 * @return  This returns a pointer to the destination string dest or const string.
 */
char *cli_getStateString(bool getMainState, uint8_t state, char *dest)
{
    char *ret;

    // check what kind of state the user want the string of
    // if it is the main state
    if(getMainState)
    {
        // check if the state variable is in the range
        if(state > (uint8_t)DEEP_SLEEP)
        {
            // error
            cli_printfError("cli ERROR: state (%d) > main state enum (%d)\n", state, (uint8_t)DEEP_SLEEP);

            // return the self-test state to indicate an error
            state = (uint8_t)SELF_TEST;
        }

        // check if null pointer
        if(dest != NULL)
        {
            // set the state
            strcpy(dest, gStatesArray[state]);
            ret = dest;
        }
        else
        {
            // give the address of the state array
            ret = (char *)gStatesArray[state];
        }
    }
    // if the user want the charge state string
    else
    {
        // check if the state variable is in the range
        if(state > (uint8_t)CHARGE_COMPLETE)
        {
            // error
            cli_printfError(
                "cli ERROR: state (%d) > charge state enum (%d)\n", state, (uint8_t)CHARGE_COMPLETE);

            // return the relaxation state to indicate an error
            state = (uint8_t)RELAXATION;
        }

        // check if null pointer
        if(dest != NULL)
        {
            // set the state
            strcpy(dest, gChargeStatesArray[state]);
            ret = dest;
        }
        else
        {
            // give the address of the state array
            ret = (char *)gChargeStatesArray[state];
        }
    }

    // return the destination string
    return ret;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

// this prints the help
void printHelp(void)
{
    // print the help
    cli_printf("This is the bms cli (command line interface) help\n");
    cli_printf("These commands can be used with the bms:\n");
    cli_printf("bms help                  --this command shows this help\n");
    cli_printf("bms help parameters       --this command shows the <parameter> list\n");
    cli_printf("bms help show-meas        --this command shows the <show-meas> list\n");
    cli_printf("bms get <parameter>       --this command gets a parameter value.\n");
    cli_printf("                            parameter is the parameter you want\n");
    cli_printf("bms get all               --this command gets all the parameters\n");
    cli_printf("                            including the values\n");
    cli_printf("bms set <parameter> <x>   --this command can be used to set a parameter\n");
    cli_printf("                            WARNING this could lead to unsave operations!\n");
    cli_printf("                            WARNING only use this command in a safe manner!\n");
    cli_printf("                            parameter is the parameter you want to set\n");
    cli_printf("                            x is the new value of the parameter you want to set\n");
    cli_printf("                            to enter a decimal value use \".\" as seperator\n");
    cli_printf("                            to enter a string with spaces use \"input string\"\n");
    cli_printf(
        "bms show <show-meas> <x>  --this command can be used to show the cyclic measurement results\n");
    cli_printf(
        "                            show-meas is the to measurement to enable or disable visibility\n");
    cli_printf("                            if x is 1 the measurement is shown, if 0 it will be disabled\n");
    cli_printf("bms reset                 --this command will reset the fault when in the fault state\n");
    cli_printf("bms sleep                 --with this command it will go to the sleep state\n");
    cli_printf("                            from the normal or the self_discharge state\n");
    cli_printf(
        "                            NOTE: if current is drawn it will transition to the normal state\n");
    cli_printf("bms wake                  --this command will wake the BMS in the sleep state\n");
    cli_printf("bms deepsleep             --with this command it will go to the deep sleep state\n");
    cli_printf("                            from the sleep state or the charge state\n");
    cli_printf(
        "bms save                  --this command will save the current settings (parameters) to flash\n");
    cli_printf(
        "bms load                  --this command will load the saved settings (parameters) from flash\n");
    cli_printf("bms default               --this command will load the default settings\n");
    cli_printf("bms time                  --this command will output the time since boot\n");
    cli_printf("reboot                    --this command will reboot the microcontroller\n");
    cli_printf(
        "                            this command should be used without the word bms in front of it\n\n");
    cli_printf("some parameters have a letter in front of the \"-\", this indicates the type of parameter\n");
    cli_printf("a - capacity\n");
    cli_printf("c - temperature (celcius)\n");
    cli_printf("e - energy\n");
    cli_printf("i - current\n");
    cli_printf("m - mass\n");
    cli_printf("n - number\n");
    cli_printf("s - status\n");
    cli_printf("t - time\n");
    cli_printf("v - voltage\n");
}

// this prints all the parameters from gGetSetparameters
void printParameters(void)
{
    int i, j;

    // standard message
    cli_printf("These are the <parameter> inputs that can be used to get or set a parameter:\n");

    cli_printf("parameter");

    // move the cursor to position
    cli_printf("\r" VT100_FMT_CURSORRT, 22);

    cli_printf("unit");

    // move the cursor to position
    cli_printf("\r" VT100_FMT_CURSORRT, 28);

    cli_printf("RO/RW");

    // move the cursor to position
    cli_printf("\r" VT100_FMT_CURSORRT, 35);

    cli_printf("type\n");

    // loop though the parameters
    for(i = 0; i < PARAMETER_ARRAY_SIZE; i++)
    {
        // change each character of the string parameters to lowercase for the user input
        for(j = 0; (gGetSetParameters[i])[j]; j++)
        {
            // change it to lowercase
            gGetSetParamsLowerString[j] = tolower((gGetSetParameters[i])[j]);

            // find the underscores
            if(((gGetSetParameters[i])[j]) == '_')
            {
                // change to upperscore
                gGetSetParamsLowerString[j] = '-';
            }
        }

        // add the null terminator to the string
        gGetSetParamsLowerString[j] = '\0';

        // print them
        cli_printf("%s", gGetSetParamsLowerString);

        // move the cursor to position
        cli_printf("\r" VT100_FMT_CURSORRT, 22);

        // make sure you stay in the array
        if(i < NONE)
        {
            cli_printf("%s", data_getUnit(i));
        }
        else
        {
            cli_printf("-");
        }

        // move the cursor to position
        cli_printf("\r" VT100_FMT_CURSORRT, 30);

        // check if non writable
        if((data_getParameterIfUserReadOnly(i) == 1) || i >= NONE)
        {
            cli_printf("RO");
        }
        else
        {
            cli_printf("RW");
        }

        // move the cursor to position
        cli_printf("\r" VT100_FMT_CURSORRT, 35);

        // make sure you stay in the array
        if(i < NONE)
        {
            cli_printf("%s\n", data_getTypeString(i));
        }
        else
        {
            cli_printf("-\n");
        }
    }

    // return
    return;
}


//! this prints all the parameters from FOR_EACH_PARAMETER with the value
void printAllParameterValues(void)
{
    int      i, j;
    float    lvFloatVal = 0.0;
    char *   lvPStringVal;
    int32_t  lvIntVal;
    uint64_t lvUint64Val;

    // standard message
    cli_printf("These are the parameters with the values:\n");

    // loop though the parameters
    for(i = 0; i < NONE; i++)
    {
        // reset the intvalues
        lvIntVal    = 0;
        lvUint64Val = 0;

        // change each character of the string parameters to lowercase for the user input
        // convert all underscores to upderscores
        for(j = 0; (gGetSetParameters[i])[j]; j++)
        {
            // change it to lowercase
            gGetSetParamsLowerString[j] = tolower((gGetSetParameters[i])[j]);

            // find the underscores
            if(((gGetSetParameters[i])[j]) == '_')
            {
                // change to upperscore
                gGetSetParamsLowerString[j] = '-';
            }
        }

        // add the null terminator to the string
        gGetSetParamsLowerString[j] = '\0';

        // print them
        cli_printf("%s", gGetSetParamsLowerString); //, strlen(gGetSetParamsLowerString));

        // move the cursor to position
        cli_printf("\r" VT100_FMT_CURSORRT, 24);

        // get the parameter value from the data sturct
        switch(data_getType(i))
        {
            // if it is a floating point
            case FLOATVAL:
                // get the value
                if(data_getParameter(i, &lvFloatVal, NULL) == NULL)
                {
                    // something went wrong
                    cli_printfError("CLI ERROR: data_getParameter\n");
                }
                else
                {
                    cli_printf("%.3f", lvFloatVal);
                }

                break;
            // if it is a string
            case STRINGVAL:
                // get the value
                lvPStringVal = data_getParameter(i, NULL, NULL);

                // check if it went wrong
                if(lvPStringVal == NULL)
                {
                    // something went wrong
                    cli_printfError("CLI ERROR: data_getParameter\n");
                }
                else
                {
                    cli_printf("%s", lvPStringVal);
                }

                break;
            // if it is a uint64_t value
            case UINT64VAL:
                // get the value
                if(data_getParameter(i, &lvUint64Val, NULL) == NULL)
                {
                    // something went wrong
                    cli_printfError("CLI ERROR: data_getParameter\n");
                }
                else
                {
                    cli_printf("%" PRIu64, lvUint64Val);
                }

                break;
            // if it is a integer (max int32_t)
            default:
                // get the value
                if(data_getParameter(i, &lvIntVal, NULL) == NULL)
                {
                    // something went wrong
                    cli_printfError("CLI ERROR: data_getParameter\n");
                }
                else
                {
                    cli_printf("%d", lvIntVal);
                }

                break;
        }

        // move the cursor to position
        cli_printf("\r" VT100_FMT_CURSORRT, 35);

        // output the unit
        // make sure you stay in the array
        if(i < NONE)
        {
            cli_printf("%s\n", data_getUnit(i));
        }
        else
        {
            cli_printf("-\n");
        }
    }

    // return
    return;
}

int cli_printfColor(printColor_t color, const char *fmt, va_list argp)
{
    int lvRetValue;
    // va_list argp;
    struct timespec waitTime;
    char            colorSequence[7];

    // check which color it is
    switch(color)
    {
        case RED:
            // set  the red color string
            sprintf(colorSequence, "\e[31m");
            break;
        case GREEN:
            // set  the green color string
            sprintf(colorSequence, "\e[32m");
            break;
        case YELLOW:
            // set  the yellow color string
            sprintf(colorSequence, "\e[33m");
            break;
        default:
            // set the normal color string
            sprintf(colorSequence, "\e[39m");
            break;
    }

    // check if mutex is initialzed
    if(gCliPrintLockInitialized)
    {
        // get the time
        if(clock_gettime(CLOCK_REALTIME, &waitTime) == -1)
        {
            cli_printf("%sCLI ERROR: failed to get time!\e[39m\n", "\e[31m");
        }

        // add the time
        waitTime.tv_sec += (int)(CLI_TIMED_LOCK_WAIT_TIME_MS / 1000) +
            ((waitTime.tv_nsec + (CLI_TIMED_LOCK_WAIT_TIME_MS % 1000) * MS_TO_NS_MULT) / (NSEC_MAX + 1));
        waitTime.tv_nsec =
            (waitTime.tv_nsec + (CLI_TIMED_LOCK_WAIT_TIME_MS % 1000) * MS_TO_NS_MULT) % (NSEC_MAX + 1);

        // lock the mutex
        lvRetValue = pthread_mutex_timedlock(&gCliPrintLock, &waitTime);

        // check if succesfull
        if(!lvRetValue)
        {
            // print the red color sequence
            lvRetValue |= printf("%s", colorSequence);

            // do the printf
            lvRetValue = vprintf(fmt, argp);

            // print the normal color sequence
            lvRetValue |= printf("\e[39m");

            // unlock the mutex
            pthread_mutex_unlock(&gCliPrintLock);
        }
        else
        {
            // print the red color sequence
            lvRetValue |= printf("%s", colorSequence);

            // do the printf
            lvRetValue = vprintf(fmt, argp);

            // print the normal color sequence
            lvRetValue |= printf("\e[39m");
        }
    }

    else
    {
        // print the red color sequence
        lvRetValue = printf("%s", colorSequence);

        // do the printf
        lvRetValue = vprintf(fmt, argp);

        // print the normal color sequence
        lvRetValue |= printf("\e[39m");
    }

    // return to the user
    return lvRetValue;
}

/*!
 * @brief   this function can be used to change what is viewed
 *          on the CLI (the new measurements)
 *
 * @param   showCommand the showcommand that the user entered.
 * @param   value the value of that param.
 *
 * @return  none
 */
void setShowMeas(showCommands_t showCommand, bool value)
{
    int     i;
    int32_t int32Val;
    int     lineCounterVal = 0;

    // check if a bit is set
    if(showCommand < CLI_ALL)
    {
        // check the value
        if(value)
        {
            // set the bit
            gShowMeasurements |= (1 << showCommand);

            // reset the screen
            if(gDoTop)
            {
                cli_printf("\e[2J");
            }
        }
        else
        {
            // clear the bit
            gShowMeasurements &= ~(1 << showCommand);
        }
    }
    // if it is one of the other commands
    else
    {
        // check if it is show all
        if(showCommand == CLI_ALL)
        {
            // reset the value
            if(!value)
            {
                // reset the global value to not display anything anymore
                gShowMeasurements = 0;

                // turn off top
                gDoTop = 0;
            }
            else
            {
                // set each bit
                for(i = 0; i < CLI_ALL; i++)
                {
                    // set the value
                    gShowMeasurements |= (1 << i);
                }
            }

            // if the top command has been given
            if(gDoTop)
            {
                // clear the screen
                cli_printf("\e[2J");

                // loop though them
                for(i = 0; i < CLI_ALL; i++)
                {
                    // check if they need to be displayed
                    if(gShowMeasurements & (1 << i))
                    {
                        // increase the linecounter value
                        lineCounterVal++;

                        // if it is the cell voltages
                        if(i == CLI_CELL_VOLTAGE)
                        {
                            // get the cell count
                            if(data_getParameter(N_CELLS, &int32Val, NULL) == NULL)
                            {
                                cli_printfError(
                                    "bcc_monitoring_setShowMeas ERROR: getting cell count went wrong!\n");
                            }

                            // add some more lines for the other cells
                            lineCounterVal += ((int32Val & 0xFF) - 1);
                        }
                        // if it is the temperatures
                        else if(i == CLI_TEMPERATURE)
                        {
                            // add some more lines for the other
                            lineCounterVal += 3;
                        }
                    }
                }

                // set the CLI line to after all the occupied lines
                cli_printf("\e[%d;0H", ++lineCounterVal);
            }
        }
        // if it is the top command
        if(showCommand == CLI_TOP)
        {
            // set the topvariable
            gDoTop = value;

            // check if reset is needed
            if(value)
            {
                // clear the screen
                cli_printf("\e[2J");

                // reset the line counter
                lineCounterVal = 0;

                // loop though all the displayable measurements
                for(i = 0; i < CLI_ALL; i++)
                {
                    // Turn on all the measurments
                    gShowMeasurements |= (1 << i);

                    // check if the measuremt is enabled
                    if(gShowMeasurements & (1 << i))
                    {
                        // increase the line counter
                        lineCounterVal++;

                        // check if it is the cell voltages
                        if(i == CLI_CELL_VOLTAGE)
                        {
                            // get the cell count
                            if(data_getParameter(N_CELLS, &int32Val, NULL) == NULL)
                            {
                                cli_printfError(
                                    "bcc_monitoring_setShowMeas ERROR: getting cell count went wrong!\n");
                            }

                            // add some more lines for the other cells
                            lineCounterVal += ((int32Val & 0xFF) - 1);
                        }
                        else if(i == CLI_TEMPERATURE)
                        {
                            // add some more lines for the other
                            lineCounterVal += 3;
                        }
                    }
                }

                // set the CLI line to after all the occupied lines
                cli_printf("\e[%d;0H", ++lineCounterVal);
            }
            // if top needs to be off
            else
            {
                // Turn off all the measurments
                gShowMeasurements = 0;
            }
        }
    }
}
