############################################################################
# apps/nxp_bms/bms/Makefile
# 
# Copyright 2020-2025 NXP
# 
# SPDX-License-Identifier: BSD-3-Clause
#
############################################################################

-include $(APPDIR)/Make.defs

# BMS! built-in application info

PROGNAME  = $(CONFIG_NXP_BMS_PROGNAME)
PRIORITY  = $(CONFIG_NXP_BMS_PRIORITY)
STACKSIZE = $(CONFIG_NXP_BMS_STACKSIZE)
MODULE    = $(CONFIG_NXP_BMS)

# BMS application

CSRCS   = src/data.c
CSRCS   += src/ledState.c
CSRCS   += src/gpio.c
CSRCS   += src/batManagement.c
CSRCS   += src/spi.c
CSRCS   += src/cli.c
CSRCS   += src/cyphalcan.c
CSRCS   += src/dronecan.c

CSRCS   += src/BCC/bcc_wait.c
CSRCS   += src/BCC/bcc_peripheries.c
CSRCS   += src/BCC/bcc_monitoring.c
CSRCS   += src/BCC/bcc_configuration.c


ifdef CONFIG_ARCH_BOARD_RDDRONE_BMS772
CSRCS   += src/BCC/mc3377xB/bcc_spiwrapper.c
CSRCS   += src/BCC/Derivatives/mc3377xB/bcc.c
CSRCS   += src/BCC/Derivatives/mc3377xB/bcc_communication.c
CSRCS   += src/BCC/Derivatives/mc3377xB/bcc_spi.c
CSRCS   += src/BCC/Derivatives/mc3377xB/bcc_tpl.c
else
	ifdef CONFIG_ARCH_BOARD_MR_BMS771
CSRCS   += src/BCC/mc3377xC/bcc_spiwrapper.c
CSRCS   += src/BCC/Derivatives/mc3377xC/bcc.c
CSRCS   += src/BCC/Derivatives/mc3377xC/bcc_communication.c
	endif
endif

CSRCS   += src/CAN/o1heap.c
CSRCS   += src/CAN/socketcan.c
CSRCS   += src/CAN/pnp.c
CSRCS   += src/CAN/portid.c
CSRCS   += src/CAN/timestamp.c
CSRCS   += src/sbc.c
CSRCS   += src/nfc.c
CSRCS   += src/SMBus.c
CSRCS   += src/i2c.c
CSRCS   += src/display.c

ifdef CONFIG_ARCH_BOARD_RDDRONE_BMS772
CSRCS   += src/a1007.c
else 
	ifdef CONFIG_ARCH_BOARD_MR_BMS771
CSRCS   += src/se05x.c
CSRCS 	+= src/tja1463.c 
	endif
endif

CSRCS   += src/balancing.c
CSRCS   += src/power.c

MAINSRC = src/main.c

CFLAGS  += -I inc
CFLAGS  += -I inc/BCC
CFLAGS  += -I inc/BCC/Derivatives
CFLAGS  += -I inc/CAN
CFLAGS  += -I inc/dronecan

ifdef CONFIG_ARCH_BOARD_RDDRONE_BMS772
CFLAGS  += -I inc/BCC/mc3377xB
CFLAGS  += -I inc/BCC/Derivatives/mc3377xB
else
	ifdef CONFIG_ARCH_BOARD_MR_BMS771
CFLAGS  += -I inc/BCC/mc3377xC
CFLAGS  += -I inc/BCC/Derivatives/mc3377xC
	endif
endif

CFLAGS  += -std=c11 -I$(APPDIR)/include/canutils
CFLAGS  += -I $(TOPDIR)/arch/arm/src/common
ifdef CONFIG_ARCH_BOARD_RDDRONE_BMS772
CFLAGS  += -I $(TOPDIR)/boards/arm/s32k1xx/rddrone-bms772/src
endif
ifdef CONFIG_ARCH_BOARD_MR_BMS771
CFLAGS  += -I $(TOPDIR)/boards/arm/s32k1xx/mr-bms771/src
endif
CFLAGS  += -I $(TOPDIR)/include/arch/chip
CFLAGS  += -I $(TOPDIR)/include/arch/board
CFLAGS  += -I $(TOPDIR)/arch/arm/src/s32k1xx
CFLAGS  += -I $(TOPDIR)/boards/arm/s32k1xx/drivers


include $(APPDIR)/Application.mk
