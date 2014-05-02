/*
 * linux/drivers/power/fg/fg_ocv.c
 *
 * TI Fuel Gauge driver for Linux
 *
 * Copyright (C) 2008-2009 Texas Instruments, Inc.
 * Author: Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef __FG_OCV_H
#define __FG_OCV_H

#include <linux/types.h>

#include "fg.h"
#ifdef CONFIG_JET_V2
struct ocv_temperature_config{
	unsigned short table[OCV_TABLE_SIZE];
};
#endif
void fg_ocv(struct cell_state *cell);
bool fg_check_relaxed(struct cell_state *cell);
bool fg_current_debounce_check(struct cell_state *cell);
short voltage_cap_table(unsigned int temp_index, short voltage);
bool fg_ocv_check(struct cell_state *cell);
#endif
