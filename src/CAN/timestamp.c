/****************************************************************************
 * nxp_bms/BMS_v1/src/CAN/timestamp.c
 *
 * Copyright 2022-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#include "timestamp.h"

#include <stdio.h>
#include <stdlib.h>

/****************************************************************************
 * Name: getMonotonicTimestampUSec
 *
 * Description:
 *
 ****************************************************************************/
uint64_t getMonotonicTimestampUSec(void)
{
    struct timespec ts;

    memset(&ts, 0, sizeof(ts));

    if(clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
    {
        abort();
    }

    return ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;
}
