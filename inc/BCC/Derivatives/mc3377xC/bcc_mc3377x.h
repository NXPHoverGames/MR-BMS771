/*
 * Copyright 2022-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*!
 * @file MC3377x.h
 *
 * Register map for MC33771C device converted to MC33772B defines.
 */

#ifndef BCC_MC3377X_H_
#define BCC_MC3377X_H_


#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "MC33771C.h"
#include "bcc.h"

/* Number of BCC configuration registers set during the initialization. */
#define MC33771C_INIT_CONF_REG_CNT  59U
#define MC33772C_INIT_CONF_REG_CNT  43U

/* Map the bcc_status_t of the MC33772B to MC33771C bcc.h defines */
/*!< Wrong CRC. */
#define BCC_STATUS_CRC              BCC_STATUS_COM_CRC
/*!< Response Rolling Counter (RC) value does not match with expected RC. */
#define BCC_STATUS_COM_RC           BCC_STATUS_COM_MSG_CNT
/*!< Response frame of BCC device is equal to zero */
#define BCC_STATUS_NULL_RESP        BCC_STATUS_COM_NULL
/* Map the old BCC_STATUS_SPI_INIT to the new BCC_STATUS_SPI_FAIL*/
#define BCC_STATUS_SPI_INIT         BCC_STATUS_SPI_FAIL

/******************************************************************************/
/* SYS_CFG1 - System configuration. */
/******************************************************************************/
#define BCC_REG_SYS_CFG1_ADDR        MC33771C_SYS_CFG1_OFFSET

/* Commands the device to DIAG Mode. Re-writing the GO2DIAG bit restarts the DIAG_TIMEOUT. */
#define BCC_DIAG_MODE_DISABLED       (MC33771C_SYS_CFG1_GO2DIAG_EXIT_ENUM_VAL   << MC33771C_SYS_CFG1_GO2DIAG_SHIFT)
#define BCC_DIAG_MODE_ENABLED        (MC33771C_SYS_CFG1_GO2DIAG_ENTER_ENUM_VAL  << MC33771C_SYS_CFG1_GO2DIAG_SHIFT)

/* General enable or disable for all cell balance drivers. */
#define BCC_CB_DRV_DISABLED          (MC33771C_SYS_CFG1_CB_DRVEN_DISABLED_ENUM_VAL  << MC33771C_SYS_CFG1_CB_DRVEN_SHIFT)
/* Each cell balance driver can be individually switched on/off by CB_xx_CFG register. */
#define BCC_CB_DRV_ENABLED           (MC33771C_SYS_CFG1_CB_DRVEN_ENABLED_ENUM_VAL   << MC33771C_SYS_CFG1_CB_DRVEN_SHIFT)

/* Enable for current measurement chain. */
#define BCC_I_MEAS_DISABLED          (MC33771C_SYS_CFG1_I_MEAS_EN_DISABLED_ENUM_VAL << MC33771C_SYS_CFG1_I_MEAS_EN_SHIFT)
#define BCC_I_MEAS_ENABLED           (MC33771C_SYS_CFG1_I_MEAS_EN_ENABLED_ENUM_VAL  << MC33771C_SYS_CFG1_I_MEAS_EN_SHIFT)

/* Controls the off time of the heart beat pulse. */
#define BCC_WAVE_DC_500US            (MC33771C_SYS_CFG1_WAVE_DC_BITX_500US_ENUM_VAL << MC33771C_SYS_CFG1_WAVE_DC_BITX_SHIFT)
#define BCC_WAVE_DC_1MS              (MC33771C_SYS_CFG1_WAVE_DC_BITX_1MS_ENUM_VAL   << MC33771C_SYS_CFG1_WAVE_DC_BITX_SHIFT)
#define BCC_WAVE_DC_10MS             (MC33771C_SYS_CFG1_WAVE_DC_BITX_10MS_ENUM_VAL  << MC33771C_SYS_CFG1_WAVE_DC_BITX_SHIFT)
#define BCC_WAVE_DC_100MS            (MC33771C_SYS_CFG1_WAVE_DC_BITX_100MS_ENUM_VAL << MC33771C_SYS_CFG1_WAVE_DC_BITX_SHIFT)

/* FAULT pin wave form control bit. */
/* Fault pin has high (fault is present) or low (no fault) level behavior. */
#define BCC_FAULT_WAVE_DISABLED      (MC33771C_SYS_CFG1_FAULT_WAVE_DISABLED_ENUM_VAL << MC33771C_SYS_CFG1_FAULT_WAVE_SHIFT)
/* Fault pin has heart beat wave when no fault is present. Pulse high time is
 * fixed at 500us. */
#define BCC_FAULT_WAVE_ENABLED       (MC33771C_SYS_CFG1_FAULT_WAVE_ENABLED_ENUM_VAL << MC33771C_SYS_CFG1_FAULT_WAVE_SHIFT)

/* Software Reset. */
#define BCC_SW_RESET_DISABLED        (MC33771C_SYS_CFG1_SOFT_RST_DISABLED_ENUM_VAL  << MC33771C_SYS_CFG1_SOFT_RST_SHIFT)
#define BCC_SW_RESET_ENABLED         (MC33771C_SYS_CFG1_SOFT_RST_ACTIVE_ENUM_VAL    << MC33771C_SYS_CFG1_SOFT_RST_SHIFT)

/* Cell balancing manual pause. */
/* CB switches can be normally commanded on/off by the dedicated logic functions. */
#define BCC_CB_MAN_PAUSE_DISABLED    (MC33771C_SYS_CFG1_CB_MANUAL_PAUSE_DISABLED_ENUM_VAL   << MC33771C_SYS_CFG1_CB_MANUAL_PAUSE_SHIFT)
/* CB switches are forced off, CB counters are not frozen. */
#define BCC_CB_MAN_PAUSE_ENABLED     (MC33771C_SYS_CFG1_CB_MANUAL_PAUSE_ENABLED_ENUM_VAL    << MC33771C_SYS_CFG1_CB_MANUAL_PAUSE_SHIFT)

/* Commands the device to DIAG Mode. Re-writing the GO2DIAG bit restarts the DIAG_TIMEOUT. */
#define BCC_DIAG_MODE_DISABLED       (MC33771C_SYS_CFG1_GO2DIAG_EXIT_ENUM_VAL   << MC33771C_SYS_CFG1_GO2DIAG_SHIFT)
#define BCC_DIAG_MODE_ENABLED        (MC33771C_SYS_CFG1_GO2DIAG_ENTER_ENUM_VAL  << MC33771C_SYS_CFG1_GO2DIAG_SHIFT)

/* General enable or disable for all cell balance drivers. */
#define BCC_CB_DRV_DISABLED          (MC33771C_SYS_CFG1_CB_DRVEN_DISABLED_ENUM_VAL  << MC33771C_SYS_CFG1_CB_DRVEN_SHIFT)
/* Each cell balance driver can be individually switched on/off by CB_xx_CFG register. */
#define BCC_CB_DRV_ENABLED           (MC33771C_SYS_CFG1_CB_DRVEN_ENABLED_ENUM_VAL   << MC33771C_SYS_CFG1_CB_DRVEN_SHIFT)

/* Disables Cell Balance for ADC1-A and ADC1-B during the conversion cycle. */
//#define BCC_CB_AUTO_PAUSE_DISABLED   (0x00U << BCC_RW_CB_AUTO_PAUSE_SHIFT)
/* CB switches are forced off each time a cyclic measurement is performed, no
 * impact on CB counters. */
//#define BCC_CB_AUTO_PAUSE_ENABLED    (0x01U << BCC_RW_CB_AUTO_PAUSE_SHIFT)

/* DIAG Mode Timeout. Length of time the device is allowed to be in DIAG Mode
 * before being forced to Normal mode. */
#define BCC_DIAG_TIMEOUT_NO_TIMER    (MC33771C_SYS_CFG1_DIAG_TIMEOUT_NO_TIMER_ENUM_VAL << MC33771C_SYS_CFG1_DIAG_TIMEOUT_SHIFT)
#define BCC_DIAG_TIMEOUT_0_05S       (MC33771C_SYS_CFG1_DIAG_TIMEOUT_0_05S_ENUM_VAL << MC33771C_SYS_CFG1_DIAG_TIMEOUT_SHIFT)
#define BCC_DIAG_TIMEOUT_0_1S        (MC33771C_SYS_CFG1_DIAG_TIMEOUT_0_1S_ENUM_VAL  << MC33771C_SYS_CFG1_DIAG_TIMEOUT_SHIFT)
#define BCC_DIAG_TIMEOUT_0_2S        (MC33771C_SYS_CFG1_DIAG_TIMEOUT_0_2S_ENUM_VAL  << MC33771C_SYS_CFG1_DIAG_TIMEOUT_SHIFT)
#define BCC_DIAG_TIMEOUT_1S          (MC33771C_SYS_CFG1_DIAG_TIMEOUT_1S_ENUM_VAL    << MC33771C_SYS_CFG1_DIAG_TIMEOUT_SHIFT)
#define BCC_DIAG_TIMEOUT_2S          (MC33771C_SYS_CFG1_DIAG_TIMEOUT_2S_ENUM_VAL    << MC33771C_SYS_CFG1_DIAG_TIMEOUT_SHIFT)
#define BCC_DIAG_TIMEOUT_4S          (MC33771C_SYS_CFG1_DIAG_TIMEOUT_4S_ENUM_VAL    << MC33771C_SYS_CFG1_DIAG_TIMEOUT_SHIFT)
#define BCC_DIAG_TIMEOUT_8S          (MC33771C_SYS_CFG1_DIAG_TIMEOUT_8S_ENUM_VAL    << MC33771C_SYS_CFG1_DIAG_TIMEOUT_SHIFT)

/* Field CYCLIC_TIMER: Timer to trigger cyclic measurements in normal mode or sleep mode. */
#define BCC_RW_CYCLIC_TIMER_MASK     MC33771C_SYS_CFG1_CYCLIC_TIMER_MASK

/* Timer to trigger cyclic measurements in Normal mode or Sleep mode. */
/* Cyclic timer is disabled, whatever the mode. */
#define BCC_CYCLIC_TIMER_DISABLED    (MC33771C_SYS_CFG1_CYCLIC_TIMER_DISABLED_ENUM_VAL << MC33771C_SYS_CFG1_CYCLIC_TIMER_SHIFT)
/* Continuous measurements. */
#define BCC_CYCLIC_TIMER_CONTINOUS   (MC33771C_SYS_CFG1_CYCLIC_TIMER_CONTINUOUS_ENUM_VAL << MC33771C_SYS_CFG1_CYCLIC_TIMER_SHIFT)
#define BCC_CYCLIC_TIMER_0_1S        (MC33771C_SYS_CFG1_CYCLIC_TIMER_0_1S_ENUM_VAL  << MC33771C_SYS_CFG1_CYCLIC_TIMER_SHIFT)
#define BCC_CYCLIC_TIMER_0_2S        (MC33771C_SYS_CFG1_CYCLIC_TIMER_0_2S_ENUM_VAL  << MC33771C_SYS_CFG1_CYCLIC_TIMER_SHIFT)
#define BCC_CYCLIC_TIMER_1S          (MC33771C_SYS_CFG1_CYCLIC_TIMER_1S_ENUM_VAL    << MC33771C_SYS_CFG1_CYCLIC_TIMER_SHIFT)
#define BCC_CYCLIC_TIMER_2S          (MC33771C_SYS_CFG1_CYCLIC_TIMER_2S_ENUM_VAL    << MC33771C_SYS_CFG1_CYCLIC_TIMER_SHIFT)
#define BCC_CYCLIC_TIMER_4S          (MC33771C_SYS_CFG1_CYCLIC_TIMER_4S_ENUM_VAL    << MC33771C_SYS_CFG1_CYCLIC_TIMER_SHIFT)
#define BCC_CYCLIC_TIMER_8S          (MC33771C_SYS_CFG1_CYCLIC_TIMER_8S_ENUM_VAL    << MC33771C_SYS_CFG1_CYCLIC_TIMER_SHIFT)

/******************************************************************************/
/* SYS_CFG2 - System configuration. */
/******************************************************************************/
#define BCC_REG_SYS_CFG2_ADDR            MC33771C_SYS_CFG2_OFFSET
#define BCC_REG_SYS_CFG2_DEFAULT         MC33771C_SYS_CFG2_POR_VAL

#define BCC_RW_HAMM_ENCOD_MASK           MC33771C_SYS_CFG2_HAMM_ENCOD_MASK
#define BCC_RW_NUMB_ODD_MASK             MC33771C_SYS_CFG2_NUMB_ODD_MASK
//#define BCC_R_VPRE_UV_MASK               0x0004U
#define BCC_RW_TIMEOUT_COMM_MASK         MC33771C_SYS_CFG2_TIMEOUT_COMM_MASK
#define BCC_RW_FLT_RST_CFG_MASK          MC33771C_SYS_CFG2_FLT_RST_CFG_MASK
#define BCC_R_PREVIOUS_STATE_MASK        MC33771C_SYS_CFG2_PREVIOUS_STATE_MASK

#define BCC_RW_HAMM_ENCOD_SHIFT          MC33771C_SYS_CFG2_HAMM_ENCOD_SHIFT
#define BCC_RW_NUMB_ODD_SHIFT            MC33771C_SYS_CFG2_NUMB_ODD_SHIFT
//#define BCC_R_VPRE_UV_SHIFT              0x02U
#define BCC_RW_TIMEOUT_COMM_SHIFT        MC33771C_SYS_CFG2_TIMEOUT_COMM_SHIFT
#define BCC_RW_FLT_RST_CFG_SHIFT         MC33771C_SYS_CFG2_FLT_RST_CFG_SHIFT
#define BCC_R_PREVIOUS_STATE_SHIFT       MC33771C_SYS_CFG2_PREVIOUS_STATE_SHIFT

/* Hamming encoders. */
/* Decode - the DED Hamming decoder performs. */
#define BCC_HAMM_ENCOD_DECODE           (MC33771C_SYS_CFG2_HAMM_ENCOD_DECODE_ENUM_VAL << MC33771C_SYS_CFG2_HAMM_ENCOD_SHIFT)
/* Encode - DED Hamming decoders generate the redundancy bits. */
#define BCC_HAMM_ENCOD_ENCODE           (MC33771C_SYS_CFG2_HAMM_ENCOD_ENCODE_ENUM_VAL << MC33771C_SYS_CFG2_HAMM_ENCOD_SHIFT)

/* Odd number of cells in the cluster (useful for open-load diagnosis). */
#define BCC_EVEN_CELLS                  (MC33771C_SYS_CFG2_NUMB_ODD_EVEN_ENUM_VAL   << MC33771C_SYS_CFG2_NUMB_ODD_SHIFT)
#define BCC_ODD_CELLS                   (MC33771C_SYS_CFG2_NUMB_ODD_ODD_ENUM_VAL    << MC33771C_SYS_CFG2_NUMB_ODD_SHIFT)

/* VPRE undervoltage detection. */
//#define BCC_VPRE_UV_NO_UNDERVOLTAGE     (0x00U << BCC_R_VPRE_UV_SHIFT)
//#define BCC_VPRE_UV_DETECTED            (0x01U << BCC_R_VPRE_UV_SHIFT)

/* No communication timeout - Set FAULT1_STATUS[COMM_LOSS_FLT] bit indicating
 * no valid communication has been received for a period greater than
 * TIMEOUT_COMM while in Normal mode. */
#define BCC_TIMEOUT_COMM_32MS           (MC33771C_SYS_CFG2_TIMEOUT_COMM_32MS_ENUM_VAL   << MC33771C_SYS_CFG2_TIMEOUT_COMM_SHIFT)
#define BCC_TIMEOUT_COMM_64MS           (MC33771C_SYS_CFG2_TIMEOUT_COMM_64MS_ENUM_VAL   << MC33771C_SYS_CFG2_TIMEOUT_COMM_SHIFT)
#define BCC_TIMEOUT_COMM_128MS          (MC33771C_SYS_CFG2_TIMEOUT_COMM_128MS_ENUM_VAL  << MC33771C_SYS_CFG2_TIMEOUT_COMM_SHIFT)
#define BCC_TIMEOUT_COMM_256MS          (MC33771C_SYS_CFG2_TIMEOUT_COMM_256MS_ENUM_VAL  << MC33771C_SYS_CFG2_TIMEOUT_COMM_SHIFT)

/* Fault reset configuration. */
/* Disabled COM timeout (1024 ms) reset and OSC fault monitoring and reset. */
#define BCC_FLT_RST_CFG_RESET_DIS       (MC33771C_SYS_CFG2_FLT_RST_CFG_RESET_DISABLED_ENUM_VAL << MC33771C_SYS_CFG2_FLT_RST_CFG_SHIFT)
/* Enabled OSC fault monitoring. */
#define BCC_FLT_RST_CFG_OSC_MON         (MC33771C_SYS_CFG2_FLT_RST_CFG_OSC_MON_ENUM_VAL << MC33771C_SYS_CFG2_FLT_RST_CFG_SHIFT)
/* Enabled OSC fault monitoring and reset. */
#define BCC_FLT_RST_CFG_OSC             (MC33771C_SYS_CFG2_FLT_RST_CFG_OSC_MON_RESET_ENUM_VAL << MC33771C_SYS_CFG2_FLT_RST_CFG_SHIFT)
/* Enabled COM timeout (1024 ms) reset. */
#define BCC_FLT_RST_CFG_COM             (MC33771C_SYS_CFG2_FLT_RST_CFG_COM_RESET_ENUM_VAL << MC33771C_SYS_CFG2_FLT_RST_CFG_SHIFT)
/* Enabled COM timeout (1024 ms) reset and OSC fault monitoring. */
#define BCC_FLT_RST_CFG_COM_OSC_MON     (MC33771C_SYS_CFG2_FLT_RST_CFG_COM_RESET_OSC_MON_ENUM_VAL << MC33771C_SYS_CFG2_FLT_RST_CFG_SHIFT)
/* Enable COM timeout (1024 ms) reset and OSC fault monitoring and reset (reset value). */
#define BCC_FLT_RST_CFG_COM_OSC         (MC33771C_SYS_CFG2_FLT_RST_CFG_COM_RESET_OSC_MON_RESET_ENUM_VAL << MC33771C_SYS_CFG2_FLT_RST_CFG_SHIFT)

/******************************************************************************/
/* SYS_DIAG - System diagnostic. */
/******************************************************************************/
#define BCC_REG_SYS_DIAG_ADDR       MC33771C_SYS_DIAG_OFFSET
#define BCC_REG_SYS_DIAG_DEFAULT    MC33771C_SYS_DIAG_POR_VAL

/******************************************************************************/
/* ADC_CFG - ADC configuration. */
/******************************************************************************/
#define BCC_REG_ADC_CFG_ADDR      MC33771C_ADC_CFG_OFFSET
#define BCC_REG_ADC_CFG_DEFAULT   MC33771C_ADC_CFG_POR_VAL

#define BCC_W_CC_RST_MASK         MC33771C_ADC_CFG_CC_RST_MASK

/* ADC2 Measurement Resolution. */
#define BCC_ADC2_RES_13BIT        (MC33771C_ADC_CFG_ADC2_DEF_13_BIT_ENUM_VAL << MC33771C_ADC_CFG_ADC2_DEF_SHIFT)
#define BCC_ADC2_RES_14BIT        (MC33771C_ADC_CFG_ADC2_DEF_14_BIT_ENUM_VAL << MC33771C_ADC_CFG_ADC2_DEF_SHIFT)
#define BCC_ADC2_RES_15BIT        (MC33771C_ADC_CFG_ADC2_DEF_15_BIT_ENUM_VAL << MC33771C_ADC_CFG_ADC2_DEF_SHIFT)
#define BCC_ADC2_RES_16BIT        (MC33771C_ADC_CFG_ADC2_DEF_16_BIT_ENUM_VAL << MC33771C_ADC_CFG_ADC2_DEF_SHIFT)

/* ADC1_B Measurement Resolution. */
#define BCC_ADC1_B_RES_13BIT      (MC33771C_ADC_CFG_ADC1_B_DEF_13_BIT_ENUM_VAL << MC33771C_ADC_CFG_ADC1_B_DEF_SHIFT)
#define BCC_ADC1_B_RES_14BIT      (MC33771C_ADC_CFG_ADC1_B_DEF_14_BIT_ENUM_VAL << MC33771C_ADC_CFG_ADC1_B_DEF_SHIFT)
#define BCC_ADC1_B_RES_15BIT      (MC33771C_ADC_CFG_ADC1_B_DEF_15_BIT_ENUM_VAL << MC33771C_ADC_CFG_ADC1_B_DEF_SHIFT)
#define BCC_ADC1_B_RES_16BIT      (MC33771C_ADC_CFG_ADC1_B_DEF_16_BIT_ENUM_VAL << MC33771C_ADC_CFG_ADC1_B_DEF_SHIFT)

/* ADC1_A Measurement Resolution. */
#define BCC_ADC1_A_RES_13BIT      (MC33771C_ADC_CFG_ADC1_A_DEF_13_BIT_ENUM_VAL << MC33771C_ADC_CFG_ADC1_A_DEF_SHIFT)
#define BCC_ADC1_A_RES_14BIT      (MC33771C_ADC_CFG_ADC1_A_DEF_14_BIT_ENUM_VAL << MC33771C_ADC_CFG_ADC1_A_DEF_SHIFT)
#define BCC_ADC1_A_RES_15BIT      (MC33771C_ADC_CFG_ADC1_A_DEF_15_BIT_ENUM_VAL << MC33771C_ADC_CFG_ADC1_A_DEF_SHIFT)
#define BCC_ADC1_A_RES_16BIT      (MC33771C_ADC_CFG_ADC1_A_DEF_16_BIT_ENUM_VAL << MC33771C_ADC_CFG_ADC1_A_DEF_SHIFT)

/* This bit enables or disables a compensation block for pins CT1, CT2. It
 * compensates for the charge transfer from ADC1-A to these pins. If disabled,
 * cell1 and cell2 accuracies may be impacted by a few mV. */
//#define BCC_CHAR_COMP_ENABLED     (0x00U << BCC_RW_DIS_CH_COMP_SHIFT)
//#define BCC_CHAR_COMP_DISABLED    (0x01U << BCC_RW_DIS_CH_COMP_SHIFT)

/* Control bit used to reset the value of the Coulomb counter to 0. */
/* Reset Coulomb counter registers COULOMB_CNT1 and COULOMB_CNT2 and the
 * CC_NB_SAMPLES registers. */
#define BCC_CC_RESET              (MC33771C_ADC_CFG_CC_RST_RESET_ENUM_VAL   << MC33771C_ADC_CFG_CC_RST_SHIFT)

/* Define the gain of the ADC2 Programmable Gain Amplifier. */
#define BCC_ADC2_PGA_4            (MC33771C_ADC_CFG_PGA_GAIN_4_ENUM_VAL     << MC33771C_ADC_CFG_PGA_GAIN_SHIFT)
#define BCC_ADC2_PGA_16           (MC33771C_ADC_CFG_PGA_GAIN_16_ENUM_VAL    << MC33771C_ADC_CFG_PGA_GAIN_SHIFT)
#define BCC_ADC2_PGA_64           (MC33771C_ADC_CFG_PGA_GAIN_64_ENUM_VAL    << MC33771C_ADC_CFG_PGA_GAIN_SHIFT)
#define BCC_ADC2_PGA_256          (MC33771C_ADC_CFG_PGA_GAIN_256_ENUM_VAL   << MC33771C_ADC_CFG_PGA_GAIN_SHIFT)
/* Automatic Gain Selection (internally adjusted). */
#define BCC_ADC2_PGA_AUTO         (MC33771C_ADC_CFG_PGA_GAIN_AUTO_ENUM_VAL  << MC33771C_ADC_CFG_PGA_GAIN_SHIFT)

/* Control bit to command the BCC to initiate a conversion sequence. */
/* Initiate conversion sequence. Bit remains logic 1 during conversion cycle.
 * User may not write logic 0 to this bit. */
#define BCC_INIT_CONV_SEQ         (MC33771C_ADC_CFG_SOC_ENABLED_ENUM_VAL    << MC33771C_ADC_CFG_SOC_SHIFT)

/**
 * The TAG_ID is provided by the system controller during each conversion request.
 * Tag ID should be incremented for each conversion request sent by the system
 * controller. When reading the data for the requested conversion the tag field
 * contains the TAG_ID.
 *
 * @param reg Register to be modified.
 * @param tagID Tag ID provided in conversion [0x00 - 0x0F].
 */
#define BCC_SET_TAG_ID(reg, tagID) \
    BCC_REG_SET_BITS_VALUE(reg, BCC_RW_TAG_ID_MASK, BCC_RW_TAG_ID_SHIFT, tagID, 0x0FU)

/******************************************************************************/
/* GPIO_CFG1 - GPIO configuration. */
/******************************************************************************/
#define BCC_REG_GPIO_CFG1_ADDR     0x1DU
#define BCC_REG_GPIO_CFG1_DEFAULT  0x0000U

/* Represents GPIO[0-6]_CFG mask. */
#define BCC_RW_GPIOX_CFG_MASK(GPIONumber) \
    (0x0003U << ((GPIONumber) * 2))

/* Represents GPIO[1-6]_CFG shift. */
#define BCC_GPIOX_CFG_SHIFT(GPIONumber)   \
    ((GPIONumber) * 2)

/* Register controls the configuration of the GPIO port. */
/* GPIOx configured as Analog Input for Ratiometric Measurement. */
#define BCC_GPIOX_AN_IN_RM_MEAS(gpioNumber)   \
    (0x00U << BCC_GPIOX_CFG_SHIFT(gpioNumber))
/* GPIOx configured as Analog Input for Absolute Measurement. */
#define BCC_GPIOX_AN_IN_ABS_MEAS(gpioNumber)  \
    (0x01U << BCC_GPIOX_CFG_SHIFT(gpioNumber))
/* GPIOx configured as digital input. */
#define BCC_GPIOX_DIG_IN(gpioNumber)          \
    (0x02U << BCC_GPIOX_CFG_SHIFT(gpioNumber))
/* GPIOx configured as digital output. */
#define BCC_GPIOX_DIG_OUT(gpioNumber)         \
    (0x03U << BCC_GPIOX_CFG_SHIFT(gpioNumber))

/* GPIO2 used as ADC1_A/ADC1_B start-of-conversion. Requires GPIO2_CFG = 10. */
/* GPIO2 port ADC Trigger is disabled. */
#define BCC_GPIO2_ADC_TRG_DISABLED  MC33771C_GPIO_CFG2_GPIO2_SOC(MC33771C_GPIO_CFG2_GPIO2_SOC_ADC_TRG_DISABLED_ENUM_VAL)

/* --------------------------------------------------------------------------
 * ADC2_OFFSET_COMP (read-write): Current measurement chain offset compensation.
 * -------------------------------------------------------------------------- */
#define BCC_REG_ADC2_OFFSET_COMP_ADDR MC33771C_ADC2_OFFSET_COMP_OFFSET

#define BCC_R_CC_OVT_MASK           MC33771C_ADC2_OFFSET_COMP_CC_OVT_MASK
#define BCC_R_SAMP_OVF_MASK         MC33771C_ADC2_OFFSET_COMP_SAMP_OVF_MASK
#define BCC_R_CC_N_OVF_MASK         MC33771C_ADC2_OFFSET_COMP_CC_N_OVF_MASK
#define BCC_R_CC_P_OVF_MASK         MC33771C_ADC2_OFFSET_COMP_CC_P_OVF_MASK
#define BCC_RW_FREE_CNT_MASK        MC33771C_ADC2_OFFSET_COMP_FREE_CNT_MASK
#define BCC_RW_CC_RST_CFG_MASK      MC33771C_ADC2_OFFSET_COMP_CC_RST_CFG_MASK

/* GPIO0 wake-up capability. Valid only when GPIO0_CFG = 10. */
/* No wake-up capability. */
#define BCC_GPIO0_NO_WAKE_UP        MC33771C_GPIO_CFG2_GPIO0_WU(MC33771C_GPIO_CFG2_GPIO0_WU_NO_WAKEUP_ENUM_VAL)

/* GPIO0 Activate Fault Output pin. Valid only when GPIO0_CFG = 10. */
/* Does not activate Fault pin when GPIO0 is configured as an input and is logic 1. */
#define BCC_GPIO0_INP_HIGH_FP_NACT MC33771C_GPIO_CFG2_GPIO0_FLT_ACT(MC33771C_GPIO_CFG2_GPIO0_FLT_ACT_DISABLED_ENUM_VAL)

/* Overvoltage threshold setting for all cell terminals. Returned value is
 * prepared to be placed in register. Enabled through register OV_UV_EN.
 *
 * @param threshold Threshold value in mV.
 */
#define BCC_SET_ALL_CT_OV_TH(threshold) \
    MC33771C_TH_ALL_CT_ALL_CT_OV_TH(((threshold) * 10U) / 195U)

/* Undervoltage threshold setting for all cell terminals. Returned value is
 * prepared to be placed in register. Enabled through register OV_UV_EN.
 *
 * @param threshold Threshold value in mV.
 */
#define BCC_SET_ALL_CT_UV_TH(threshold) \
    MC33771C_TH_ALL_CT_ALL_CT_UV_TH(((threshold) * 10U) / 195U)

/* Undervoltage threshold setting for all cell terminals. Returned value is
 * prepared to be placed in register. Register [COMMON_UV_TH] must be logic 0
 * and [CTx_OVUV_EN] must be logic 1 to use TH_CTx register as threshold.
 *
 * @param threshold Threshold value in mV.
 */
#define BCC_SET_CTX_UV_TH(threshold)                                  \
    ((uint16_t)(((((threshold) * 10U) / 196U) > 0x00FF) ?             \
    0x0FF : (((threshold) * 10U) / 196U)) << MC33771C_TH_ALL_CT_ALL_CT_UV_TH_SHIFT)

/* Overvoltage threshold setting for all cell terminals. Returned value is
 * prepared to be placed in register. Register [COMMON_OV_TH] must be logic 0
 * and [CTx_OVUV_EN] must be logic 1 to use TH_CTx register as threshold.
 *
 * @param threshold Threshold value in mV.
 */
#define BCC_SET_CTX_OV_TH(threshold)                                  \
    ((uint16_t)(((((threshold) * 10U) / 196U) > 0x00FF) ?             \
    0x0FF : (((threshold) * 10U) / 196U)) << MC33771C_TH_ALL_CT_ALL_CT_OV_TH_SHIFT)

/* Configuration of the free running Coulomb counters. */
#define BCC_FREE_CC_CLAMP                (MC33771C_ADC2_OFFSET_COMP_FREE_CNT_CLAMP_ENUM_VAL     << MC33771C_ADC2_OFFSET_COMP_FREE_CNT_SHIFT)
#define BCC_FREE_CC_ROLL_OVER            (MC33771C_ADC2_OFFSET_COMP_FREE_CNT_ROLL_OVER_ENUM_VAL << MC33771C_ADC2_OFFSET_COMP_FREE_CNT_SHIFT)

/* Configuration of the action linked to the read of Coulomb counters results. */
#define BCC_READ_CC_NO_ACTION            (MC33771C_ADC2_OFFSET_COMP_CC_RST_CFG_NO_ACTION_ENUM_VAL   << MC33771C_ADC2_OFFSET_COMP_CC_RST_CFG_SHIFT)
/* Reading any CC register (from @ $2D to @ $2F) also resets the Coulomb counters. */
#define BCC_READ_CC_RESET                (MC33771C_ADC2_OFFSET_COMP_CC_RST_CFG_CC_RESET_ENUM_VAL    << MC33771C_ADC2_OFFSET_COMP_CC_RST_CFG_SHIFT)

/******************************************************************************/
/* CBx_CFG - CB configuration for cell 1~14. */
/******************************************************************************/

#define BCC_REG_CB1_CFG_ADDR    MC33771C_CB1_CFG_OFFSET
#define BCC_REG_CB2_CFG_ADDR    MC33771C_CB2_CFG_OFFSET
#define BCC_REG_CB3_CFG_ADDR    MC33771C_CB3_CFG_OFFSET
#define BCC_REG_CB4_CFG_ADDR    MC33771C_CB4_CFG_OFFSET
#define BCC_REG_CB5_CFG_ADDR    MC33771C_CB5_CFG_OFFSET
#define BCC_REG_CB6_CFG_ADDR    MC33771C_CB6_CFG_OFFSET

/* Following 8 registers are reserved in MC33772. */
#define BCC_REG_CB7_CFG_ADDR    MC33771C_CB7_CFG_OFFSET
#define BCC_REG_CB8_CFG_ADDR    MC33771C_CB8_CFG_OFFSET
#define BCC_REG_CB9_CFG_ADDR    MC33771C_CB9_CFG_OFFSET
#define BCC_REG_CB10_CFG_ADDR   MC33771C_CB10_CFG_OFFSET
#define BCC_REG_CB11_CFG_ADDR   MC33771C_CB11_CFG_OFFSET
#define BCC_REG_CB12_CFG_ADDR   MC33771C_CB12_CFG_OFFSET
#define BCC_REG_CB13_CFG_ADDR   MC33771C_CB13_CFG_OFFSET
#define BCC_REG_CB14_CFG_ADDR   MC33771C_CB14_CFG_OFFSET

#define BCC_REG_CBX_CFG_DEFAULT MC33771C_CB1_CFG_POR_VAL

#define BCC_RW_CB_TIMER_MASK    MC33771C_CB1_CFG_CB_TIMER_MASK
#define BCC_W_CB_EN_MASK        MC33771C_CB1_CFG_CB_EN_MASK
#define BCC_R_CB_STS_MASK       MC33771C_CB1_CFG_CB_STS_MASK

#define BCC_RW_CB_TIMER_SHIFT   MC33771C_CB1_CFG_CB_TIMER_SHIFT
#define BCC_W_CB_EN_SHIFT       MC33771C_CB1_CFG_CB_EN_SHIFT
#define BCC_R_CB_STS_SHIFT      MC33771C_CB1_CFG_CB_STS_SHIFT

/* Cell Balance Timer in minutes. */
#define BCC_SET_CB_TIMER(reg, minutes) \
    BCC_REG_SET_BITS_VALUE(reg, BCC_RW_CB_TIMER_MASK, BCC_RW_CB_TIMER_SHIFT, minutes, 0x01FFU)

/* Cell Balance enable. */
#define BCC_CB_DISABLED         (MC33771C_CB1_CFG_CB_EN_DISABLED_ENUM_VAL << BCC_W_CB_EN_SHIFT)
/* Enabled or re-launched if overwritten (restarts the timer count from zero
 * and enables the driver). */
#define BCC_CB_ENABLED          (MC33771C_CB1_CFG_CB_EN_ENABLED_ENUM_VAL << BCC_W_CB_EN_SHIFT)

/******************************************************************************/
/* TH_ANx_OT - ANx overtemp threshold. */
/******************************************************************************/

#define BCC_REG_TH_AN6_OT_ADDR    MC33771C_TH_AN6_OT_OFFSET
#define BCC_REG_TH_AN5_OT_ADDR    MC33771C_TH_AN5_OT_OFFSET
#define BCC_REG_TH_AN4_OT_ADDR    MC33771C_TH_AN4_OT_OFFSET
#define BCC_REG_TH_AN3_OT_ADDR    MC33771C_TH_AN3_OT_OFFSET
#define BCC_REG_TH_AN2_OT_ADDR    MC33771C_TH_AN2_OT_OFFSET
#define BCC_REG_TH_AN1_OT_ADDR    MC33771C_TH_AN1_OT_OFFSET
#define BCC_REG_TH_AN0_OT_ADDR    MC33771C_TH_AN0_OT_OFFSET

#define BCC_REG_TH_ANX_OT_DEFAULT MC33771C_TH_AN0_OT_AN_OT_TH_DEFAULT_ENUM_VAL

#define BCC_RW_ANX_OT_TH_MASK     MC33771C_TH_AN0_OT_AN_OT_TH_MASK
#define BCC_RW_ANX_OT_TH_SHIFT    MC33771C_TH_AN0_OT_AN_OT_TH_SHIFT

/* Overtemperature threshold setting for analog input x. Returned value is
 * prepared to be placed in register.
 *
 * @param threshold Threshold value in mV.
 */
#define BCC_SET_ANX_OT_TH(threshold)                                                   \
    ((uint16_t)((((((uint32_t)(threshold)) * 100U) / 488U) > BCC_RW_ANX_OT_TH_MASK) ?  \
    BCC_RW_ANX_OT_TH_MASK : ((((uint32_t)(threshold)) * 100U) / 488U)) << BCC_RW_ANX_OT_TH_SHIFT)

/* Default value set to 1.16V. */
#define BCC_ANX_OT_TH_DEFAULT     MC33771C_TH_AN0_OT_AN_OT_TH_DEFAULT_ENUM_VAL

/******************************************************************************/
/* TH_ANx_UT - ANx undertemp threshold. */
/******************************************************************************/

#define BCC_REG_TH_AN6_UT_ADDR    MC33771C_TH_AN6_UT_OFFSET
#define BCC_REG_TH_AN5_UT_ADDR    MC33771C_TH_AN5_UT_OFFSET
#define BCC_REG_TH_AN4_UT_ADDR    MC33771C_TH_AN4_UT_OFFSET
#define BCC_REG_TH_AN3_UT_ADDR    MC33771C_TH_AN3_UT_OFFSET
#define BCC_REG_TH_AN2_UT_ADDR    MC33771C_TH_AN2_UT_OFFSET
#define BCC_REG_TH_AN1_UT_ADDR    MC33771C_TH_AN1_UT_OFFSET
#define BCC_REG_TH_AN0_UT_ADDR    MC33771C_TH_AN0_UT_OFFSET

#define BCC_REG_TH_ANX_UT_DEFAULT MC33771C_TH_AN0_UT_AN_UT_TH_DEFAULT_ENUM_VAL

#define BCC_RW_ANX_UT_TH_MASK     MC33771C_TH_AN0_UT_AN_UT_TH_MASK
#define BCC_RW_ANX_UT_TH_SHIFT    MC33771C_TH_AN0_UT_AN_UT_TH_SHIFT

/* Undertemperature threshold setting for analog input x. Returned value is
 * prepared to be placed in register.
 * @param threshold Threshold value in mV.
 */
#define BCC_SET_ANX_UT_TH(threshold)                                                   \
    ((uint16_t)((((((uint32_t)(threshold)) * 100U) / 488U) > BCC_RW_ANX_UT_TH_MASK) ?  \
    BCC_RW_ANX_UT_TH_MASK : ((((uint32_t)(threshold)) * 100U) / 488U)) << BCC_RW_ANX_UT_TH_SHIFT)

/* Default value set to 3.82V. */
#define BCC_ANX_UT_TH_DEFAULT     MC33771C_TH_AN0_UT_AN_UT_TH_DEFAULT_ENUM_VAL

/******************************************************************************/
/* TH_ISENSE_OC - ISENSE over current threshold. */
/******************************************************************************/

#define BCC_REG_TH_ISENSE_OC_ADDR    MC33771C_TH_ISENSE_OC_OFFSET

#define BCC_REG_TH_ISENSE_OC_DEFAULT MC33771C_TH_ISENSE_OC_POR_VAL

#define BCC_RW_TH_ISENSE_OC_MASK     MC33771C_TH_ISENSE_OC_TH_ISENSE_OC_MASK

#define BCC_RW_TH_ISENSE_OC_SHIFT    MC33771C_TH_ISENSE_OC_TH_ISENSE_OC_SHIFT

/* Overvoltage threshold setting for individual cell terminals. Register
 * [COMMON_OV_TH] must be logic 0 and [CTx_OVUV_EN] must be logic 1 to use
 * TH_CTx register as threshold. Returned value is prepared to be placed
 * in register.
 *
 * @param threshold Threshold value in uV.
 */
#define BCC_SET_TH_ISENSE_OC(threshold)                                  \
    (((uint16_t)(((threshold) * 5) / 6) << BCC_RW_TH_ISENSE_OC_SHIFT) &  \
            BCC_RW_TH_ISENSE_OC_MASK)

/******************************************************************************/
/* TH_COULOMB_CNT_MSB - Coulomb counter threshold. */
/******************************************************************************/

#define BCC_REG_TH_COULOMB_CNT_MSB_ADDR MC33771C_TH_COULOMB_CNT_MSB_OFFSET

#define BCC_RW_TH_COULOMB_CNT_MSB_MASK  MC33771C_TH_COULOMB_CNT_MSB_TH_COULOMB_CNT_MSB_MASK

#define BCC_RW_TH_COULOMB_CNT_MSB_SHIFT MC33771C_TH_COULOMB_CNT_MSB_TH_COULOMB_CNT_MSB_SHIFT

/* Over Coulomb counting threshold  setting. Returned value is prepared to be
 * placed in a register.
 *
 * @param Threshold threshold value in uV.
 */
#define BCC_SET_TH_COULOMB_CNT_MSB(threshold) \
    ((uint16_t)((((threshold) >> 16U) * 10U) / 6U) << BCC_RW_TH_COULOMB_CNT_MSB_SHIFT)

/******************************************************************************/
/* TH_COULOMB_CNT_LSB - Coulomb counter threshold. */
/******************************************************************************/

#define BCC_REG_TH_COULOMB_CNT_LSB_ADDR MC33771C_TH_COULOMB_CNT_LSB_OFFSET

#define BCC_RW_TH_COULOMB_CNT_LSB_MASK  MC33771C_TH_COULOMB_CNT_LSB_TH_COULOMB_CNT_LSB_MASK

#define BCC_RW_TH_COULOMB_CNT_LSB_SHIFT MC33771C_TH_COULOMB_CNT_LSB_TH_COULOMB_CNT_LSB_SHIFT

/* Over Coulomb counting threshold  setting. Returned value is prepared to be
 * placed in a register.
 *
 * @param threshold Threshold value in uV.
 */
#define BCC_SET_TH_COULOMB_CNT_LSB(threshold) \
  ((uint16_t)((((threshold) && 0xFFFF) * 10) / 6) << BCC_RW_TH_COULOMB_CNT_LSB_SHIFT)

/******************************************************************************/
/* OV_UV_EN - CT measurement selection. */
/******************************************************************************/
#define BCC_REG_OV_UV_EN_ADDR              MC33771C_OV_UV_EN_OFFSET
#define BCC_REG_OV_UV_EN_DEFAULT           MC33771C_OV_UV_EN_POR_VAL

/* Note MC33772 does not have CT7_OVUV_EN, ..., CT14_OVUV_EN registers. */
/* Represents CT[1-14]_OVUV_EN mask. */
#define BCC_RW_CTX_OVUV_EN_MASK(ctNumber) \
    (0x0001U << ((ctNumber) - 1U))

#define BCC_RW_COMMON_UV_TH_MASK           MC33771C_OV_UV_EN_COMMON_UV_TH_MASK
#define BCC_RW_COMMON_OV_TH_MASK           MC33771C_OV_UV_EN_COMMON_OV_TH_MASK

/* Represents CT[1-14]_OVUV_EN shift. */
#define BCC_RW_CTX_OVUV_EN_SHIFT(ctNumber) ((ctNumber) - 1U)

#define BCC_RW_COMMON_UV_TH_SHIFT          MC33771C_OV_UV_EN_COMMON_UV_TH_SHIFT
#define BCC_RW_COMMON_OV_TH_SHIFT          MC33771C_OV_UV_EN_COMMON_OV_TH_SHIFT

/* Control bit used to Enable or disable ADC data to be compared with thresholds
 * for OV/UV. If Disabled no OVUV fault is set. */
#define BCC_CTX_OVUV_DISABLED(ctNumber)    (MC33771C_OV_UV_EN_CT14_OVUV_EN_DISABLED_ENUM_VAL    << BCC_RW_CTX_OVUV_EN_SHIFT(ctNumber))
#define BCC_CTX_OVUV_ENABLED(ctNumber)     (MC33771C_OV_UV_EN_CT14_OVUV_EN_ENABLED_ENUM_VAL     << BCC_RW_CTX_OVUV_EN_SHIFT(ctNumber))

/* All CTx measurement use the common or individual under-voltage threshold
 * register for comparison. */
#define BCC_CTX_UV_TH_INDIVIDUAL           (MC33771C_OV_UV_EN_COMMON_UV_TH_INDIVIDUAL_ENUM_VAL  << BCC_RW_COMMON_UV_TH_SHIFT)
#define BCC_CTX_UV_TH_COMMON               (MC33771C_OV_UV_EN_COMMON_UV_TH_COMMON_ENUM_VAL      << BCC_RW_COMMON_UV_TH_SHIFT)

/* All CTx measurement use the common or individual over-voltage threshold
 * register for comparison. */
#define BCC_CTX_OV_TH_INDIVIDUAL           (MC33771C_OV_UV_EN_COMMON_OV_TH_INDIVIDUAL_ENUM_VAL  << BCC_RW_COMMON_OV_TH_SHIFT)
#define BCC_CTX_OV_TH_COMMON               (MC33771C_OV_UV_EN_COMMON_OV_TH_COMMON_ENUM_VAL      << BCC_RW_COMMON_OV_TH_SHIFT)

/******************************************************************************/
/* FAULT1_STATUS - fault status register 1. */
/******************************************************************************/

#define BCC_REG_FAULT1_STATUS_ADDR  MC33771C_FAULT1_STATUS_OFFSET

#define BCC_R_CT_UV_FLT_MASK        MC33771C_FAULT_MASK1_CT_UV_FLT_MASK_0_F_MASK
#define BCC_R_CT_OV_FLT_MASK        MC33771C_FAULT_MASK1_CT_OV_FLT_MASK_1_F_MASK
#define BCC_R_AN_UT_FLT_MASK        MC33771C_FAULT_MASK1_AN_UT_FLT_MASK_2_F_MASK
#define BCC_R_AN_OT_FLT_MASK        MC33771C_FAULT_MASK1_AN_OT_FLT_MASK_3_F_MASK
#define BCC_RW_IS_OC_FLT_MASK       MC33771C_FAULT1_STATUS_IS_OC_FLT_MASK
#define BCC_RW_IS_OL_FLT_MASK       MC33771C_FAULT_MASK1_IS_OL_FLT_MASK_5_F_MASK
#define BCC_RW_I2C_ERR_FLT_MASK     MC33771C_FAULT1_STATUS_I2C_ERR_FLT_MASK
#define BCC_RW_GPIO0_WUP_FLT_MASK   MC33771C_FAULT_MASK1_GPIO0_WUP_FLT_MASK_7_F_MASK
#define BCC_RW_CSB_WUP_FLT_MASK     MC33771C_FAULT_MASK1_CSB_WUP_FLT_MASK_8_F_MASK
#define BCC_RW_COM_ERR_FLT_MASK     MC33771C_FAULT_MASK1_COM_ERR_FLT_MASK_9_F_MASK
#define BCC_RW_COM_LOSS_FLT_MASK    MC33771C_FAULT_MASK1_COM_LOSS_FLT_MASK_10_F_MASK
#define BCC_RW_VPWR_LV_FLT_MASK     MC33771C_FAULT_MASK1_VPWR_LV_FLT_MASK_11_F_MASK
#define BCC_RW_VPWR_OV_FLT_MASK     MC33771C_FAULT_MASK1_VPWR_OV_FLT_MASK_12_F_MASK
#define BCC_RW_COM_ERR_OVR_FLT_MASK MC33771C_FAULT1_STATUS_COM_ERR_OVR_FLT_MASK
#define BCC_RW_RESET_FLT_MASK       MC33771C_FAULT1_STATUS_RESET_FLT_MASK
#define BCC_RW_POR_MASK             MC33771C_FAULT1_STATUS_POR_MASK

/******************************************************************************/
/* FAULT3_STATUS - fault status register 3. */
/******************************************************************************/

#define BCC_R_CC_OVR_FLT_MASK       MC33771C_FAULT3_STATUS_CC_OVR_FLT_MASK

/******************************************************************************/
/* $27 FAULT_MASK1 - FAULT pin mask. */
/******************************************************************************/
/* Mask a fault from activating the FAULT pin output. */
#define BCC_REG_FAULT_MASK1_ADDR    MC33771C_FAULT_MASK1_OFFSET
#define BCC_REG_FAULT_MASK1_DEFAULT MC33771C_FAULT_MASK1_POR_VAL

/* Note: you can use bit masks defined for FAULT1_STATUS register (the same
 * order of bits). */
#define BCC_CT_UV_FLT_EN        MC33771C_FAULT_MASK1_CT_UV_FLT_MASK_0_F_NOT_MASKED_ENUM_VAL     << MC33771C_FAULT_MASK1_CT_UV_FLT_MASK_0_F_SHIFT
#define BCC_CT_UV_FLT_DIS       MC33771C_FAULT_MASK1_CT_UV_FLT_MASK_0_F_MASKED_ENUM_VAL         << MC33771C_FAULT_MASK1_CT_UV_FLT_MASK_0_F_SHIFT
#define BCC_CT_OV_FLT_EN        MC33771C_FAULT_MASK1_CT_OV_FLT_MASK_1_F_NOT_MASKED_ENUM_VAL     << MC33771C_FAULT_MASK1_CT_OV_FLT_MASK_1_F_SHIFT
#define BCC_CT_OV_FLT_DIS       MC33771C_FAULT_MASK1_CT_OV_FLT_MASK_1_F_MASKED_ENUM_VAL         << MC33771C_FAULT_MASK1_CT_OV_FLT_MASK_1_F_SHIFT
#define BCC_AN_UT_FLT_EN        MC33771C_FAULT_MASK1_AN_UT_FLT_MASK_2_F_NOT_MASKED_ENUM_VAL     << MC33771C_FAULT_MASK1_AN_UT_FLT_MASK_2_F_SHIFT
#define BCC_AN_UT_FLT_DIS       MC33771C_FAULT_MASK1_AN_UT_FLT_MASK_2_F_MASKED_ENUM_VAL         << MC33771C_FAULT_MASK1_AN_UT_FLT_MASK_2_F_SHIFT
#define BCC_AN_OT_FLT_EN        MC33771C_FAULT_MASK1_AN_OT_FLT_MASK_3_F_NOT_MASKED_ENUM_VAL     << MC33771C_FAULT_MASK1_AN_OT_FLT_MASK_3_F_SHIFT
#define BCC_AN_OT_FLT_DIS       MC33771C_FAULT_MASK1_AN_OT_FLT_MASK_3_F_MASKED_ENUM_VAL         << MC33771C_FAULT_MASK1_AN_OT_FLT_MASK_3_F_SHIFT
#define BCC_IS_OC_FLT_EN        MC33771C_FAULT_MASK1_IS_OC_FLT_MASK_4_F_NOT_MASKED_ENUM_VAL     << MC33771C_FAULT_MASK1_IS_OC_FLT_MASK_4_F_SHIFT
#define BCC_IS_OC_FLT_DIS       MC33771C_FAULT_MASK1_IS_OC_FLT_MASK_4_F_MASKED_ENUM_VAL         << MC33771C_FAULT_MASK1_IS_OC_FLT_MASK_4_F_SHIFT
#define BCC_IS_OL_FLT_EN        MC33771C_FAULT_MASK1_IS_OL_FLT_MASK_5_F_NOT_MASKED_ENUM_VAL     << MC33771C_FAULT_MASK1_IS_OL_FLT_MASK_5_F_SHIFT
#define BCC_IS_OL_FLT_DIS       MC33771C_FAULT_MASK1_IS_OL_FLT_MASK_5_F_MASKED_ENUM_VAL         << MC33771C_FAULT_MASK1_IS_OL_FLT_MASK_5_F_SHIFT
#define BCC_I2C_ERR_FLT_EN      MC33771C_FAULT_MASK1_I2C_ERR_FLT_MASK_6_F_NOT_MASKED_ENUM_VAL   << MC33771C_FAULT_MASK1_I2C_ERR_FLT_MASK_6_F_SHIFT
#define BCC_I2C_ERR_FLT_DIS     MC33771C_FAULT_MASK1_I2C_ERR_FLT_MASK_6_F_MASKED_ENUM_VAL       << MC33771C_FAULT_MASK1_I2C_ERR_FLT_MASK_6_F_SHIFT
#define BCC_GPIO0_WUP_FLT_EN    MC33771C_FAULT_MASK1_GPIO0_WUP_FLT_MASK_7_F_NOT_MASKED_ENUM_VAL << MC33771C_FAULT_MASK1_GPIO0_WUP_FLT_MASK_7_F_SHIFT
#define BCC_GPIO0_WUP_FLT_DIS   MC33771C_FAULT_MASK1_GPIO0_WUP_FLT_MASK_7_F_MASKED_ENUM_VAL     << MC33771C_FAULT_MASK1_GPIO0_WUP_FLT_MASK_7_F_SHIFT
#define BCC_CSB_WUP_FLT_EN      MC33771C_FAULT_MASK1_CSB_WUP_FLT_MASK_8_F_NOT_MASKED_ENUM_VAL   << MC33771C_FAULT_MASK1_CSB_WUP_FLT_MASK_8_F_SHIFT
#define BCC_CSB_WUP_FLT_DIS     MC33771C_FAULT_MASK1_CSB_WUP_FLT_MASK_8_F_MASKED_ENUM_VAL       << MC33771C_FAULT_MASK1_CSB_WUP_FLT_MASK_8_F_SHIFT
#define BCC_COM_ERR_FLT_EN      MC33771C_FAULT_MASK1_COM_ERR_FLT_MASK_9_F_NOT_MASKED_ENUM_VAL   << MC33771C_FAULT_MASK1_COM_ERR_FLT_MASK_9_F_SHIFT
#define BCC_COM_ERR_FLT_DIS     MC33771C_FAULT_MASK1_COM_ERR_FLT_MASK_9_F_MASKED_ENUM_VAL       << MC33771C_FAULT_MASK1_COM_ERR_FLT_MASK_9_F_SHIFT
#define BCC_COM_LOSS_FLT_EN     MC33771C_FAULT_MASK1_COM_LOSS_FLT_MASK_10_F_NOT_MASKED_ENUM_VAL << MC33771C_FAULT_MASK1_COM_LOSS_FLT_MASK_10_F_SHIFT
#define BCC_COM_LOSS_FLT_DIS    MC33771C_FAULT_MASK1_COM_LOSS_FLT_MASK_10_F_MASKED_ENUM_VAL     << MC33771C_FAULT_MASK1_COM_LOSS_FLT_MASK_10_F_SHIFT
#define BCC_VPWR_LV_FLT_EN      MC33771C_FAULT_MASK1_VPWR_LV_FLT_MASK_11_F_NOT_MASKED_ENUM_VAL  << MC33771C_FAULT_MASK1_VPWR_LV_FLT_MASK_11_F_SHIFT
#define BCC_VPWR_LV_FLT_DIS     MC33771C_FAULT_MASK1_VPWR_LV_FLT_MASK_11_F_MASKED_ENUM_VAL      << MC33771C_FAULT_MASK1_VPWR_LV_FLT_MASK_11_F_SHIFT
#define BCC_VPWR_OV_FLT_EN      MC33771C_FAULT_MASK1_VPWR_OV_FLT_MASK_12_F_NOT_MASKED_ENUM_VAL  << MC33771C_FAULT_MASK1_VPWR_OV_FLT_MASK_12_F_SHIFT
#define BCC_VPWR_OV_FLT_DIS     MC33771C_FAULT_MASK1_VPWR_OV_FLT_MASK_12_F_MASKED_ENUM_VAL      << MC33771C_FAULT_MASK1_VPWR_OV_FLT_MASK_12_F_SHIFT

/******************************************************************************/
/* FAULT_MASK2 - FAULT pin mask. */
/******************************************************************************/
/* Mask a fault from activating the FAULT pin output. */
#define BCC_REG_FAULT_MASK2_ADDR    MC33771C_FAULT_MASK2_OFFSET
#define BCC_REG_FAULT_MASK2_DEFAULT MC33771C_FAULT_MASK2_POR_VAL

/* Note: you can use bit masks defined for FAULT2_STATUS register (the same
 * order of bits). */
#define BCC_FUSE_ERR_FLT_EN     MC33771C_FAULT_MASK2_FUSE_ERR_FLT_MASK_0_F_NOT_MASKED_ENUM_VAL << MC33771C_FAULT_MASK2_FUSE_ERR_FLT_MASK_0_F_SHIFT
#define BCC_FUSE_ERR_FLT_DIS    MC33771C_FAULT_MASK2_FUSE_ERR_FLT_MASK_0_F_MASKED_ENUM_VAL << MC33771C_FAULT_MASK2_FUSE_ERR_FLT_MASK_0_F_SHIFT
#define BCC_DED_ERR_FLT_EN      MC33771C_FAULT_MASK2_DED_ERR_FLT_MASK_1_F_NOT_MASKED_ENUM_VAL << MC33771C_FAULT_MASK2_DED_ERR_FLT_MASK_1_F_SHIFT
#define BCC_DED_ERR_FLT_DIS     MC33771C_FAULT_MASK2_DED_ERR_FLT_MASK_1_F_MASKED_ENUM_VAL << MC33771C_FAULT_MASK2_DED_ERR_FLT_MASK_1_F_SHIFT
#define BCC_OSC_ERR_FLT_EN      MC33771C_FAULT_MASK2_OSC_ERR_FLT_MASK_2_F_NOT_MASKED_ENUM_VAL << MC33771C_FAULT_MASK2_OSC_ERR_FLT_MASK_2_F_SHIFT
#define BCC_OSC_ERR_FLT_DIS     MC33771C_FAULT_MASK2_OSC_ERR_FLT_MASK_2_F_MASKED_ENUM_VAL << MC33771C_FAULT_MASK2_OSC_ERR_FLT_MASK_2_F_SHIFT
#define BCC_CB_OPEN_FLT_EN      MC33771C_FAULT_MASK2_CB_OPEN_FLT_MASK_3_F_NOT_MASKED_ENUM_VAL << MC33771C_FAULT_MASK2_CB_OPEN_FLT_MASK_3_F_SHIFT
#define BCC_CB_OPEN_FLT_DIS     MC33771C_FAULT_MASK2_CB_OPEN_FLT_MASK_3_F_MASKED_ENUM_VAL << MC33771C_FAULT_MASK2_CB_OPEN_FLT_MASK_3_F_SHIFT
#define BCC_CB_SHORT_FLT_EN     MC33771C_FAULT_MASK2_CB_SHORT_FLT_MASK_4_F_NOT_MASKED_ENUM_VAL << MC33771C_FAULT_MASK2_CB_SHORT_FLT_MASK_4_F_SHIFT
#define BCC_CB_SHORT_FLT_DIS    MC33771C_FAULT_MASK2_CB_SHORT_FLT_MASK_4_F_MASKED_ENUM_VAL << MC33771C_FAULT_MASK2_CB_SHORT_FLT_MASK_4_F_SHIFT
#define BCC_GPIO_SHORT_FLT_EN   MC33771C_FAULT_MASK2_GPIO_SHORT_FLT_MASK_5_F_NOT_MASKED_ENUM_VAL << MC33771C_FAULT_MASK2_GPIO_SHORT_FLT_MASK_5_F_SHIFT
#define BCC_GPIO_SHORT_FLT_DIS  MC33771C_FAULT_MASK2_GPIO_SHORT_FLT_MASK_5_F_MASKED_ENUM_VAL << MC33771C_FAULT_MASK2_GPIO_SHORT_FLT_MASK_5_F_SHIFT
#define BCC_AN_OPEN_FLT_EN      MC33771C_FAULT_MASK2_AN_OPEN_FLT_MASK_6_F_NOT_MASKED_ENUM_VAL << MC33771C_FAULT_MASK2_AN_OPEN_FLT_MASK_6_F_SHIFT
#define BCC_AN_OPEN_FLT_DIS     MC33771C_FAULT_MASK2_AN_OPEN_FLT_MASK_6_F_MASKED_ENUM_VAL << MC33771C_FAULT_MASK2_AN_OPEN_FLT_MASK_6_F_SHIFT
#define BCC_GND_LOSS_FLT_EN     MC33771C_FAULT_MASK2_GND_LOSS_FLT_MASK_9_F_NOT_MASKED_ENUM_VAL << MC33771C_FAULT_MASK2_GND_LOSS_FLT_MASK_9_F_SHIFT
#define BCC_GND_LOSS_FLT_DIS    MC33771C_FAULT_MASK2_GND_LOSS_FLT_MASK_9_F_MASKED_ENUM_VAL << MC33771C_FAULT_MASK2_GND_LOSS_FLT_MASK_9_F_SHIFT
#define BCC_ADC1_A_FLT_EN       MC33771C_FAULT_MASK2_ADC1_A_FLT_MASK_10_F_NOT_MASKED_ENUM_VAL << MC33771C_FAULT_MASK2_ADC1_A_FLT_MASK_10_F_SHIFT
#define BCC_ADC1_A_FLT_DIS      MC33771C_FAULT_MASK2_ADC1_A_FLT_MASK_10_F_MASKED_ENUM_VAL << MC33771C_FAULT_MASK2_ADC1_A_FLT_MASK_10_F_SHIFT
#define BCC_ADC1_B_FLT_EN       MC33771C_FAULT_MASK2_ADC1_B_FLT_MASK_11_F_NOT_MASKED_ENUM_VAL << MC33771C_FAULT_MASK2_ADC1_B_FLT_MASK_11_F_SHIFT
#define BCC_ADC1_B_FLT_DIS      MC33771C_FAULT_MASK2_ADC1_B_FLT_MASK_11_F_MASKED_ENUM_VAL << MC33771C_FAULT_MASK2_ADC1_B_FLT_MASK_11_F_SHIFT
#define BCC_VANA_UV_FLT_EN      MC33771C_FAULT_MASK2_VANA_UV_FLT_MASK_12_F_NOT_MASKED_ENUM_VAL << MC33771C_FAULT_MASK2_VANA_UV_FLT_MASK_12_F_SHIFT
#define BCC_VANA_UV_FLT_DIS     MC33771C_FAULT_MASK2_VANA_UV_FLT_MASK_12_F_MASKED_ENUM_VAL << MC33771C_FAULT_MASK2_VANA_UV_FLT_MASK_12_F_SHIFT
#define BCC_VANA_OV_FLT_EN      MC33771C_FAULT_MASK2_VANA_OV_FLT_MASK_13_F_NOT_MASKED_ENUM_VAL << MC33771C_FAULT_MASK2_VANA_OV_FLT_MASK_13_F_SHIFT
#define BCC_VANA_OV_FLT_DIS     MC33771C_FAULT_MASK2_VANA_OV_FLT_MASK_13_F_MASKED_ENUM_VAL << MC33771C_FAULT_MASK2_VANA_OV_FLT_MASK_13_F_SHIFT
#define BCC_VCOM_UV_FLT_EN      MC33771C_FAULT_MASK2_VCOM_UV_FLT_MASK_14_F_NOT_MASKED_ENUM_VAL << MC33771C_FAULT_MASK2_VCOM_UV_FLT_MASK_14_F_SHIFT
#define BCC_VCOM_UV_FLT_DIS     MC33771C_FAULT_MASK2_VCOM_UV_FLT_MASK_14_F_MASKED_ENUM_VAL << MC33771C_FAULT_MASK2_VCOM_UV_FLT_MASK_14_F_SHIFT
#define BCC_VCOM_OV_FLT_EN      MC33771C_FAULT_MASK2_VCOM_OV_FLT_MASK_15_F_NOT_MASKED_ENUM_VAL << MC33771C_FAULT_MASK2_VCOM_OV_FLT_MASK_15_F_SHIFT
#define BCC_VCOM_OV_FLT_DIS     MC33771C_FAULT_MASK2_VCOM_OV_FLT_MASK_15_F_MASKED_ENUM_VAL << MC33771C_FAULT_MASK2_VCOM_OV_FLT_MASK_15_F_SHIFT

/******************************************************************************/
/* FAULT_MASK3 - FAULT pin mask. */
/******************************************************************************/
/* Mask out the cell balance timer from activating the FAULT pin output. */
#define BCC_REG_FAULT_MASK3_ADDR    MC33771C_FAULT_MASK3_OFFSET
#define BCC_REG_FAULT_MASK3_DEFAULT MC33771C_FAULT_MASK3_POR_VAL

/* Note: you can use bit masks defined for FAULT3_STATUS register (the same
 * order of bits). */

/* Disable fault detection for desired cell (end of time cell balancing notification).
 *
 * @param CBNumber cb number ranging from 1 to 6 (MC33772) or 14 (MC33771)
 *                 No check performed.
 * @return Bit mask for desired cell.
 */
#define BCC_EOT_CBX_FLT_DIS(cbNumber) \
    (0x0001U << ((cbNumber) - 1U))

/* Enable fault detection for desired cell (end of time cell balancing notification). */
#define BCC_EOT_CBX_FLT_EN      MC33771C_FAULT_MASK3_EOT_CB1_MASK_0_F_NOT_MASKED_ENUM_VAL << MC33771C_FAULT_MASK3_EOT_CB1_MASK_0_F_SHIFT

//#define BCC_VCP_UV_EN           0x0000U /* MC33772 only. */
//#define BCC_VCP_UV_DIS          0x2000U /* MC33772 only. */
#define BCC_DIAG_TO_FLT_EN      MC33771C_FAULT_MASK3_DIAG_TO_FLT_MASK_14_F_NOT_MASKED_ENUM_VAL << MC33771C_FAULT_MASK3_DIAG_TO_FLT_MASK_14_F_SHIFT
#define BCC_DIAG_TO_FLT_DIS     MC33771C_FAULT_MASK3_DIAG_TO_FLT_MASK_14_F_MASKED_ENUM_VAL << MC33771C_FAULT_MASK3_DIAG_TO_FLT_MASK_14_F_SHIFT
#define BCC_CC_OVR_FLT_EN       MC33771C_FAULT_MASK3_CC_OVR_FLT_MASK_15_F_NOT_MASKED_ENUM_VAL << MC33771C_FAULT_MASK3_CC_OVR_FLT_MASK_15_F_SHIFT
#define BCC_CC_OVR_FLT_DIS      MC33771C_FAULT_MASK3_CC_OVR_FLT_MASK_15_F_MASKED_ENUM_VAL << MC33771C_FAULT_MASK3_CC_OVR_FLT_MASK_15_F_SHIFT

/******************************************************************************/
/* WAKEUP_MASK1 - Wake-up events mask. */
/******************************************************************************/

/* Mask events from waking up device and transition to Normal mode. */
#define BCC_REG_WAKEUP_MASK1_ADDR    MC33771C_WAKEUP_MASK1_OFFSET
#define BCC_REG_WAKEUP_MASK1_DEFAULT MC33771C_WAKEUP_MASK1_POR_VAL

#define BCC_CT_UV_WAKEUP_EN       MC33771C_WAKEUP_MASK1_CT_UV_FLT_MASK_0_F_NOT_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK1_CT_UV_FLT_MASK_0_F_SHIFT
#define BCC_CT_UV_WAKEUP_DIS      MC33771C_WAKEUP_MASK1_CT_UV_FLT_MASK_0_F_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK1_CT_UV_FLT_MASK_0_F_SHIFT
#define BCC_CT_OV_WAKEUP_EN       MC33771C_WAKEUP_MASK1_CT_OV_FLT_MASK_1_F_NOT_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK1_CT_OV_FLT_MASK_1_F_SHIFT
#define BCC_CT_OV_WAKEUP_DIS      MC33771C_WAKEUP_MASK1_CT_OV_FLT_MASK_1_F_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK1_CT_OV_FLT_MASK_1_F_SHIFT
#define BCC_AN_UT_WAKEUP_EN       MC33771C_WAKEUP_MASK1_AN_UT_FLT_MASK_2_F_NOT_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK1_AN_UT_FLT_MASK_2_F_SHIFT
#define BCC_AN_UT_WAKEUP_DIS      MC33771C_WAKEUP_MASK1_AN_UT_FLT_MASK_2_F_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK1_AN_UT_FLT_MASK_2_F_SHIFT
#define BCC_AN_OT_WAKEUP_EN       MC33771C_WAKEUP_MASK1_AN_OT_FLT_MASK_3_F_NOT_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK1_AN_OT_FLT_MASK_3_F_SHIFT
#define BCC_AN_OT_WAKEUP_DIS      MC33771C_WAKEUP_MASK1_AN_OT_FLT_MASK_3_F_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK1_AN_OT_FLT_MASK_3_F_SHIFT
#define BCC_IS_OC_WAKEUP_EN       MC33771C_WAKEUP_MASK1_IS_OC_FLT_MASK_4_F_NOT_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK1_IS_OC_FLT_MASK_4_F_SHIFT
#define BCC_IS_OC_WAKEUP_DIS      MC33771C_WAKEUP_MASK1_IS_OC_FLT_MASK_4_F_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK1_IS_OC_FLT_MASK_4_F_SHIFT
#define BCC_GPIO0_WUP_WAKEUP_EN   MC33771C_WAKEUP_MASK1_GPIO0_WUP_FLT_MASK_7_F_NOT_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK1_GPIO0_WUP_FLT_MASK_7_F_SHIFT
#define BCC_GPIO0_WUP_WAKEUP_DIS  MC33771C_WAKEUP_MASK1_GPIO0_WUP_FLT_MASK_7_F_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK1_GPIO0_WUP_FLT_MASK_7_F_SHIFT
//#define BCC_CSB_WUP_WAKEUP_EN     MC33771C_FAULT_MASK1_CSB_WUP_FLT_MASK_8_F_NOT_MASKED_ENUM_VAL << MC33771C_FAULT_MASK1_CSB_WUP_FLT_MASK_8_F_SHIFT
//#define BCC_CSB_WUP_WAKEUP_DIS    MC33771C_FAULT_MASK1_CSB_WUP_FLT_MASK_8_F_MASKED_ENUM_VAL << MC33771C_FAULT_MASK1_CSB_WUP_FLT_MASK_8_F_SHIFT
#define BCC_VPWR_LV_WAKEUP_EN     MC33771C_WAKEUP_MASK1_VPWR_LV_FLT_MASK_11_F_NOT_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK1_VPWR_LV_FLT_MASK_11_F_SHIFT
#define BCC_VPWR_LV_WAKEUP_DIS    MC33771C_WAKEUP_MASK1_VPWR_LV_FLT_MASK_11_F_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK1_VPWR_LV_FLT_MASK_11_F_SHIFT
#define BCC_VPWR_OV_WAKEUP_EN     MC33771C_WAKEUP_MASK1_VPWR_OV_FLT_MASK_12_F_NOT_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK1_VPWR_OV_FLT_MASK_12_F_SHIFT
#define BCC_VPWR_OV_WAKEUP_DIS    MC33771C_WAKEUP_MASK1_VPWR_OV_FLT_MASK_12_F_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK1_VPWR_OV_FLT_MASK_12_F_SHIFT

/******************************************************************************/
/* WAKEUP_MASK2 - Wake-up events mask. */
/******************************************************************************/

/* Mask events from waking up device and transition to Normal mode. */
#define BCC_REG_WAKEUP_MASK2_ADDR    MC33771C_WAKEUP_MASK2_OFFSET
#define BCC_REG_WAKEUP_MASK2_DEFAULT MC33771C_WAKEUP_MASK2_POR_VAL

#define BCC_DED_ERR_WAKEUP_EN     MC33771C_WAKEUP_MASK2_DED_ERR_FLT_MASK_1_F_NOT_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK2_DED_ERR_FLT_MASK_1_F_SHIFT
#define BCC_DED_ERR_WAKEUP_DIS    MC33771C_WAKEUP_MASK2_DED_ERR_FLT_MASK_1_F_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK2_DED_ERR_FLT_MASK_1_F_SHIFT
#define BCC_OSC_ERR_WAKEUP_EN     MC33771C_WAKEUP_MASK2_OSC_ERR_FLT_MASK_2_F_NOT_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK2_OSC_ERR_FLT_MASK_2_F_SHIFT
#define BCC_OSC_ERR_WAKEUP_DIS    MC33771C_WAKEUP_MASK2_OSC_ERR_FLT_MASK_2_F_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK2_OSC_ERR_FLT_MASK_2_F_SHIFT
#define BCC_CB_SHORT_WAKEUP_EN    MC33771C_WAKEUP_MASK2_CB_SHORT_FLT_MASK_4_F_NOT_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK2_CB_SHORT_FLT_MASK_4_F_SHIFT
#define BCC_CB_SHORT_WAKEUP_DIS   MC33771C_WAKEUP_MASK2_CB_SHORT_FLT_MASK_4_F_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK2_CB_SHORT_FLT_MASK_4_F_SHIFT
#define BCC_GPIO_SHORT_WAKEUP_EN  MC33771C_WAKEUP_MASK2_GPIO_SHORT_FLT_MASK_5_F_NOT_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK2_GPIO_SHORT_FLT_MASK_5_F_SHIFT
#define BCC_GPIO_SHORT_WAKEUP_DIS MC33771C_WAKEUP_MASK2_GPIO_SHORT_FLT_MASK_5_F_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK2_GPIO_SHORT_FLT_MASK_5_F_SHIFT
#define BCC_IC_TSD_WAKEUP_EN      MC33771C_WAKEUP_MASK2_IC_TSD_FLT_MASK_8_F_NOT_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK2_IC_TSD_FLT_MASK_8_F_SHIFT
#define BCC_IC_TSD_WAKEUP_DIS     MC33771C_WAKEUP_MASK2_IC_TSD_FLT_MASK_8_F_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK2_IC_TSD_FLT_MASK_8_F_SHIFT
#define BCC_GND_LOSS_WAKEUP_EN    MC33771C_WAKEUP_MASK2_GND_LOSS_FLT_MASK_9_F_NOT_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK2_GND_LOSS_FLT_MASK_9_F_SHIFT
#define BCC_GND_LOSS_WAKEUP_DIS   MC33771C_WAKEUP_MASK2_GND_LOSS_FLT_MASK_9_F_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK2_GND_LOSS_FLT_MASK_9_F_SHIFT
#define BCC_ADC1_A_WAKEUP_EN      MC33771C_WAKEUP_MASK2_ADC1_A_FLT_MASK_10_F_NOT_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK2_ADC1_A_FLT_MASK_10_F_SHIFT
#define BCC_ADC1_A_WAKEUP_DIS     MC33771C_WAKEUP_MASK2_ADC1_A_FLT_MASK_10_F_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK2_ADC1_A_FLT_MASK_10_F_SHIFT
#define BCC_ADC1_B_WAKEUP_EN      MC33771C_WAKEUP_MASK2_ADC1_B_FLT_MASK_11_F_NOT_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK2_ADC1_B_FLT_MASK_11_F_SHIFT
#define BCC_ADC1_B_WAKEUP_DIS     MC33771C_WAKEUP_MASK2_ADC1_B_FLT_MASK_11_F_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK2_ADC1_B_FLT_MASK_11_F_SHIFT
#define BCC_VANA_UV_WAKEUP_EN     MC33771C_WAKEUP_MASK2_VANA_UV_FLT_MASK_12_F_NOT_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK2_VANA_UV_FLT_MASK_12_F_SHIFT
#define BCC_VANA_UV_WAKEUP_DIS    MC33771C_WAKEUP_MASK2_VANA_UV_FLT_MASK_12_F_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK2_VANA_UV_FLT_MASK_12_F_SHIFT
#define BCC_VANA_OV_WAKEUP_EN     MC33771C_WAKEUP_MASK2_VANA_OV_FLT_MASK_13_F_NOT_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK2_VANA_OV_FLT_MASK_13_F_SHIFT
#define BCC_VANA_OV_WAKEUP_DIS    MC33771C_WAKEUP_MASK2_VANA_OV_FLT_MASK_13_F_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK2_VANA_OV_FLT_MASK_13_F_SHIFT
#define BCC_VCOM_UV_WAKEUP_EN     MC33771C_WAKEUP_MASK2_VCOM_UV_FLT_MASK_14_F_NOT_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK2_VCOM_UV_FLT_MASK_14_F_SHIFT
#define BCC_VCOM_UV_WAKEUP_DIS    MC33771C_WAKEUP_MASK2_VCOM_UV_FLT_MASK_14_F_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK2_VCOM_UV_FLT_MASK_14_F_SHIFT
#define BCC_VCOM_OV_WAKEUP_EN     MC33771C_WAKEUP_MASK2_VCOM_OV_FLT_MASK_15_F_NOT_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK2_VCOM_OV_FLT_MASK_15_F_SHIFT
#define BCC_VCOM_OV_WAKEUP_DIS    MC33771C_WAKEUP_MASK2_VCOM_OV_FLT_MASK_15_F_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK2_VCOM_OV_FLT_MASK_15_F_SHIFT

/******************************************************************************/
/* WAKEUP_MASK3 - Wake-up events mask. */
/******************************************************************************/

/* Mask out the cell balance timeout in Sleep mode from activating Normal mode. */
#define BCC_REG_WAKEUP_MASK3_ADDR    MC33771C_WAKEUP_MASK3_OFFSET
#define BCC_REG_WAKEUP_MASK3_DEFAULT MC33771C_WAKEUP_MASK3_POR_VAL

/* Disable wake-up for desired cell (end of time cell balancing notification).
 *
 * @param cbNumber CB number ranging from 1 to 6 (MC33772) or 14 (MC33771)
 *                 No check performed.
 * @return Bit mask for desired cell.
 */
#define BCC_EOT_CBX_WAKEUP_DIS(cbNumber) \
  (0x0001U << ((cbNumber) - 1U))

/* Enable fault detection for desired cell (end of time cell balancing notification). */
#define BCC_EOT_CBX_WAKEUP_EN    MC33771C_WAKEUP_MASK3_EOT_CB1_MASK_0_F_NOT_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK3_EOT_CB1_MASK_0_F_SHIFT

//#define BCC_VCP_UV_WAKEUP_EN     0x0000U /* MC33772 only. */
//#define BCC_VCP_UV_WAKEUP_DIS    0x2000U /* MC33772 only. */

//#define BCC_DIAG_TO_WAKEUP_EN    0x0000U <<
//#define BCC_DIAG_TO_WAKEUP_DIS   0x4000U <<

#define BCC_CC_OVR_WAKEUP_EN     MC33771C_WAKEUP_MASK3_CC_OVR_FLT_MASK_15_F_NOT_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK3_CC_OVR_FLT_MASK_15_F_SHIFT
#define BCC_CC_OVR_WAKEUP_DIS    MC33771C_WAKEUP_MASK3_CC_OVR_FLT_MASK_15_F_MASKED_ENUM_VAL << MC33771C_WAKEUP_MASK3_CC_OVR_FLT_MASK_15_F_SHIFT

/******************************************************************************/
/* CC_NB_NB_SAMPLES - number of samples taken for the Coulomb count. */
/******************************************************************************/
#define BCC_REG_CC_NB_SAMPLES_ADDR MC33771C_CC_NB_SAMPLES_OFFSET

#define BCC_R_CC_NB_SAMPLES_MASK   MC33771C_CC_NB_SAMPLES_CC_NB_SAMPLES_MASK

/******************************************************************************/
/* COULOMB_CNT1 - Coulomb counting accumulator. */
/******************************************************************************/
#define BCC_REG_COULOMB_CNT1_ADDR  MC33771C_COULOMB_CNT1_OFFSET

#define BCC_R_COULOMB_CNT_MSB_MASK MC33771C_COULOMB_CNT1_COULOMB_CNT_MSB_MASK

/******************************************************************************/
/* COULOMB_CNT2 - Coulomb counting accumulator. */
/******************************************************************************/
#define BCC_REG_COULOMB_CNT2_ADDR  MC33771C_COULOMB_CNT2_OFFSET

#define BCC_R_COULOMB_CNT_LSB_MASK MC33771C_COULOMB_CNT2_COULOMB_CNT_LSB_MASK

/******************************************************************************/
/* MEAS_ISENSE1 - On Demand value of measured current. */
/******************************************************************************/
#define BCC_REG_MEAS_ISENSE1_ADDR   MC33771C_MEAS_ISENSE1_OFFSET

#define BCC_R_MEAS1_I_MASK          MC33771C_MEAS_ISENSE1_MEAS_I_MSB_MASK

/******************************************************************************/
/* MEAS_ISENSE2 - On Demand value of measured current. */
/******************************************************************************/
#define BCC_REG_MEAS_ISENSE2_ADDR   MC33771C_MEAS_ISENSE2_OFFSET

#define BCC_R_MEAS2_I_MASK          MC33771C_MEAS_ISENSE2_MEAS_I_LSB_MASK
#define BCC_RW_PGA_GCHANGE_MASK     MC33771C_MEAS_ISENSE2_PGA_GCHANGE_MASK
#define BCC_RW_ADC2_SAT_MASK        MC33771C_MEAS_ISENSE2_ADC2_SAT_MASK
#define BCC_R_PGA_GAIN_MASK         MC33771C_MEAS_ISENSE2_PGA_GAIN_MASK

/******************************************************************************/
/* MEAS_STACK - Stack voltage measurement. */
/******************************************************************************/
/* Register addresses. */
#define BCC_REG_MEAS_STACK_ADDR MC33771C_MEAS_STACK_OFFSET

/* Bit masks common for MEAS_STACK, MEAS_CELL14~1, MEAS_AN6~0, MEAS_IC_TEMP,
 * MEAS_VBG_DIAG_ADC1A and MEAS_VBG_DIAG_ADC1A registers.
 */
#define BCC_R_MEAS_MASK         MC33771C_MEAS_STACK_MEAS_STACK_MASK
#define BCC_R_DATA_RDY_MASK     MC33771C_MEAS_STACK_DATA_RDY_MASK

/******************************************************************************/
/* MEAS_CELL14~1 - Cell 1~14 voltage measurement. */
/******************************************************************************/
#define BCC_REG_MEAS_CELLX_ADDR_MC33771_START   MC33771C_MEAS_CELL14_OFFSET
//#define BCC_REG_MEAS_CELLX_ADDR_MC33772_START  MC33771C_MEAS_CELL6_OFFSET
#define BCC_REG_MEAS_CELLX_ADDR_END             MC33771C_MEAS_CELL1_OFFSET

#define BCC_REG_MEAS_CELLX_ADDR_MC3377X_START   BCC_REG_MEAS_CELLX_ADDR_MC33771_START

#define BCC_MSR_CELL_VOLTX                      BCC_MSR_CELL_VOLT14

/******************************************************************************/
/* MEAS_AN6~0 - AN 0~6 voltage measurement. */
/******************************************************************************/
#define BCC_REG_MEAS_ANX_ADDR_START       MC33771C_MEAS_AN6_OFFSET
#define BCC_REG_MEAS_ANX_ADDR_END         MC33771C_MEAS_AN0_OFFSET

/******************************************************************************/
/* MEAS_IC_TEMP - IC temperature measurement. */
/******************************************************************************/
#define BCC_REG_MEAS_IC_TEMP_ADDR         MC33771C_MEAS_IC_TEMP_OFFSET

/******************************************************************************/
/* MEAS_VBG_DIAG_ADC1A - ADCIA band gap reference measurement. */
/******************************************************************************/
#define BCC_REG_MEAS_VBG_DIAG_ADC1A_ADDR  MC33771C_MEAS_VBG_DIAG_ADC1A_OFFSET

/******************************************************************************/
/* MEAS_VBG_DIAG_ADC1B - ADCIB band gap reference measurement. */
/******************************************************************************/
#define BCC_REG_MEAS_VBG_DIAG_ADC1B_ADDR  MC33771C_MEAS_VBG_DIAG_ADC1B_OFFSET

/******************************************************************************/
/* TH_ALL_CT - CTx over and undervoltage threshold. */
/******************************************************************************/
#define BCC_REG_TH_ALL_CT_ADDR    MC33771C_TH_ALL_CT_OFFSET
#define BCC_REG_TH_ALL_CT_DEFAULT MC33771C_TH_ALL_CT_POR_VAL

#define BCC_RW_ALL_CT_UV_TH_MASK  MC33771C_TH_ALL_CT_ALL_CT_UV_TH_MASK
#define BCC_RW_ALL_CT_OV_TH_MASK  MC33771C_TH_ALL_CT_ALL_CT_OV_TH_MASK

#define BCC_RW_ALL_CT_UV_TH_SHIFT MC33771C_TH_ALL_CT_ALL_CT_UV_TH_SHIFT
#define BCC_RW_ALL_CT_OV_TH_SHIFT MC33771C_TH_ALL_CT_ALL_CT_OV_TH_SHIFT

/******************************************************************************/
/* COM_STATUS - number of CRC error counted. */
/******************************************************************************/
#define BCC_REG_COM_STATUS_ADDR     MC33771C_COM_STATUS_OFFSET

/* Map the missing verifyCom function to the send nop function */
#define BCC_VerifyCom               BCC_SendNop

#ifdef __cplusplus
}
#endif

#endif
