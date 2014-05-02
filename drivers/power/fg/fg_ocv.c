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

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/device.h>
#include <linux/power/ti-fg.h>

#include "fg_ocv.h"
#include "fg_math.h"


/* OCV Lookup table */
#define INTERPOLATE_MAX		1000
unsigned short interpolate(unsigned short value,
				unsigned short *table,
				unsigned char size)
{
	unsigned char i;
	unsigned short d;
#ifdef CONFIG_JET_V2
	int delta= INTERPOLATE_MAX/(size-1);
#endif
	for (i = 0; i < size; i++)
		if (value < table[i])
			break;

#ifdef CONFIG_JET_V2
	if (i==0) {
		d=0;
	} else if(i<size){
		d = (value - table[i-1]) * delta;
		d /=  table[i] - table[i-1];
		d = d + (i-1) * delta;
	} else {
		d=1000;
	}
#else
	if ((i > 0)  && (i < size)) {
		d = (value - table[i-1]) * (INTERPOLATE_MAX/(size-1));
		d /=  table[i] - table[i-1];
		d = d + (i-1) * (INTERPOLATE_MAX/(size-1));
	} else {
		d = i * DIV_ROUND_CLOSEST(INTERPOLATE_MAX, size);
	}
#endif
	if (d > 1000)
		d = 1000;

	return d;
}


/*
 * Open Circuit Voltage (OCV) correction routine. This function estimates SOC,
 * based on the voltage.
 */
#ifdef CONFIG_JET_V2
#if defined(CONFIG_JET_SUN)
//ocv_table: "Analysis_Jet 04 - 200mA Discharge Temperature Test Sept 27-30.xlsx"
static struct ocv_temperature_config ocv_table[]={
	{	//use 40C discharge curve “35+ degC“
		.table={

				3524,3561,3593,3619,3641,
				3659,3673,3686,3698,3712,
				3730,3750,3773,3798,3825,
				3856,3890,3927,3968,4013,
				4080,
		},
	},
	{	//use 25C discharge curve "20 to 34 degC"
		.table={
				3478,3510,3557,3585,3606,
				3624,3640,3654,3667,3681,
				3698,3718,3741,3766,3794,
				3825,3859,3896,3938,3984,
				4066,
		},
	},
	{	//use 15C discharge curve "15 to 19 degC"
		.table={
				3418,3479,3519,3546,3568,
				3586,3601,3615,3630,3645,
				3662,3683,3705,3730,3757,
				3787,3820,3856,3897,3942,
				4035,
		},
	},
	{	//use 10C discharge curve "10 to 14 degC"
		.table={
				3380,3438,3478,3507,3530,
				3549,3566,3582,3597,3613,
				3631,3651,3673,3699,3726,
				3755,3788,3824,3863,3909,
				4008,
		},
	},
	{	//use 5C discharge curve "5 to 9 degC"
		.table={
				3330,3376,3419,3451,3476,
				3497,3516,3534,3550,3567,
				3585,3606,3629,3654,3680,
				3709,3742,3778,3817,3863,
				3969,
		},
	},
	{	//use 0C discharge curve "0 to 4 degC"
		.table={
				3284,3342,3376,3403,3426,
				3446,3465,3483,3500,3517,
				3536,3557,3579,3604,3630,
				3658,3688,3723,3761,3806,
				3918,
		},
	},
	{	//use -5C discharge curve "-4 to -1 degC"
		.table={
				3200,3247,3285,3316,3342,
				3365,3385,3404,3423,3441,
				3460,3481,3504,3528,3554,
				3582,3613,3648,3687,3735,
				3856,
		},
	},
	{	//use -10C discharge curve "-8 to -5 degC"
		.table={
				3224,3251,3270,3288,3305,
				3321,3338,3353,3368,3384,
				3401,3419,3438,3459,3482,
				3507,3535,3568,3606,3656,
				3778,
		},
	},
	{	//use -15C discharge curve "-14 to -9 degC"
		.table={
				3213,3225,3238,3249,3260,
				3271,3282,3292,3303,3314,
				3326,3338,3351,3365,3380,
				3397,3419,3449,3493,3561,
				3678,
		},
	},
	{	//use -20C discharge curve "-40 to -15 degC"
		.table={
				3210,3214,3217,3222,3225,
				3228,3232,3234,3236,3236,
				3237,3237,3238,3238,3238,
				3245,3276,3337,3405,3464,
				3508,
		},
	},
};
#else //CONFIG_JET_SNOW
static struct ocv_temperature_config ocv_table[]={
	{	//use ENV25,BAT25 curve
		.table={
			3578,3615,3645,3669,3688,
			3703,3716,3727,3739,3754,
			3772,3793,3821,3852,3883,
			3915,3948,3984,4023,4065,
			4126
		},
	},
	{	//use ENV5,BAT6 curve
		.table={
			3498,3562,3593,3615,3633,
			3648,3660,3673,3688,3705,
			3724,3747,3772,3799,3830,
			3863,3898,3936,3976,4019,
			4091
		},
	},
	{	//use ENV0,BAT1 curve
		.table={
			3459,3522,3558,3581,3598,
			3613,3626,3640,3656,3674,
			3695,3718,3743,3771,3801,
			3834,3869,3908,3949,3992,
			4068
		},
	},
	{	//use ENV-5,BAT-3 curve
		.table={
			3404,3466,3502,3528,3547,
			3563,3578,3594,3611,3630,
			3652,3675,3700,3729,3758,
			3791,3826,3865,3907,3955,
			4039
		},
	},
	{	//use ENV-10,BAT-7 curve
		.table={
			3338,3390,3426,3453,3475,
			3494,3512,3529,3548,3569,
			3592,3617,3642,3670,3700,
			3733,3768,3805,3847,3896,
			3997
		},
	},
	{	//use ENV-15,BAT-10
		.table={
			3269,3302,3330,3354,3376,
			3398,3418,3439,3460,3482,
			3506,3531,3558,3587,3617,
			3650,3684,3722,3765,3818,
			3940
		},
	},
		{	//use ENV-20,BAT-16
		.table={
			3238,3254,3270,3284,3300,
			3314,3330,3344,3360,3374,
			3390,3404,3420,3433,3448,
			3461,3473,3487,3524,3643,
			3845
		},
	},
};
#endif
short voltage_cap_table(unsigned int temp_index, short voltage)
{
	int tmp;
	unsigned short *table= ocv_table[temp_index].table;
	tmp = interpolate(voltage, table,OCV_TABLE_SIZE);
	return (DIV_ROUND_CLOSEST(tmp * MAX_PERCENTAGE, INTERPOLATE_MAX));
}

bool fg_current_debounce_check(struct cell_state *cell)
{
	if(cell->cur <= FG_CURRENT_CHECK){
		if(cell->current_bounce_counter<FG_CURRENT_MAX_COUNTER){
			cell->current_bounce_counter++;
			dev_dbg(cell->dev, "%s\n", __func__);
			return false;//current too high, may cause voltage read unstable
		}
	}
	cell->current_bounce_counter=0;
	return true;
}
bool fg_ocv_check(struct cell_state *cell)
{
	short soc,delta;
	/*Must in discharge*/
	if(cell->cur > 0)
		return false;
	if(cell->prev_voltage - cell->av_voltage < FG_DELTAV_CHECK)
		return false;

	dev_dbg(cell->dev, "%s:%d\n", __func__,cell->prev_voltage);
	if(fg_current_debounce_check(cell)==false)
		return false;

	cell->prev_voltage = cell->av_voltage;
	soc=voltage_cap_table(cell->temp_index, cell->av_voltage);
	delta=(cell->soc)-soc;
	if(delta>0){
		dev_dbg(cell->dev, "FG: OCV soc=%d\n", soc);
		if(delta>5){
			cell->soc= soc+(delta/2);
		}else{
			cell->soc=soc;
		}
		cell->nac= DIV_ROUND_CLOSEST(cell->fcc * cell->soc, MAX_PERCENTAGE);
		return true;
	}
	return false;
}
#endif


void fg_ocv(struct cell_state *cell)
{
#ifdef CONFIG_JET_V2
	cell->soc=voltage_cap_table(cell->temp_index, cell->av_voltage);
	cell->nac= DIV_ROUND_CLOSEST(cell->fcc * cell->soc, MAX_PERCENTAGE);
#else
	int tmp;
	tmp = interpolate(cell->av_voltage, cell->config->ocv->table,
		OCV_TABLE_SIZE);
	cell->soc = DIV_ROUND_CLOSEST(tmp * MAX_PERCENTAGE, INTERPOLATE_MAX);
	cell->nac = DIV_ROUND_CLOSEST(tmp * cell->fcc, INTERPOLATE_MAX);
#endif
	dev_dbg(cell->dev, "FG: OCV Correction (%dv, %dmAh, %d%%)\n",
			cell->av_voltage, cell->nac, cell->soc);
	/* Reset EL counter */
	cell->electronics_load = 0;
	cell->cumulative_sleep = 0;
	cell->prev_voltage = cell->av_voltage;
#ifdef SOC_CHANGE_ON_TEMPERATURE
	//cell->fcc_start=cell->fcc;
	cell->nac_start=cell->nac;
#endif
	if (!cell->ocv && cell->init) {
		cell->ocv = true;
		cell->ocv_enter_q = cell->nac;
		dev_dbg(cell->dev, "LRN: Entering OCV, OCVEnterQ = %dmAh\n",
			cell->ocv_enter_q);
	}

	do_gettimeofday(&cell->last_ocv);
}


/* Check if the cell is in Sleep */
bool fg_check_relaxed(struct cell_state *cell)
{
	struct timeval now;
	do_gettimeofday(&now);

	if (!cell->sleep) {
		if (abs(cell->cur) <=
			cell->config->ocv->sleep_enter_current) {

			if (cell->sleep_samples < MAX_UINT8)
				cell->sleep_samples++;

			if (cell->sleep_samples >=
				cell->config->ocv->sleep_enter_samples) {
				/* Entering sleep mode */
				cell->sleep_timer.tv_sec = now.tv_sec;
				cell->el_timer.tv_sec = now.tv_sec;
				cell->sleep = true;
				dev_dbg(cell->dev, "FG CHECK: Sleeping\n");
				cell->calibrate = true;
			}
		} else {
			cell->sleep_samples = 0;
		}
	} else {
		/* The battery cell is Sleeping, checking if need to exit
		   sleep mode count number of seconds that cell spent in
		   sleep */
		cell->cumulative_sleep += now.tv_sec - cell->el_timer.tv_sec;
		cell->el_timer.tv_sec = now.tv_sec;

		/* Check if we need to reset Sleep */
		if (abs(cell->av_current) >
			cell->config->ocv->sleep_exit_current) {

			if (abs(cell->cur) >
				cell->config->ocv->sleep_exit_current) {

				if (cell->sleep_samples < MAX_UINT8)
					cell->sleep_samples++;

			} else {
				cell->sleep_samples = 0;
			}

			/* Check if we need to reset a Sleep timer */
			if (cell->sleep_samples >
				cell->config->ocv->sleep_exit_samples) {
				/* Exit sleep mode */
				cell->sleep_timer.tv_sec = 0;
				cell->sleep = false;
				cell->relax = false;
				dev_dbg(cell->dev,
					"FG CHECK: Not relaxed and not sleeping\n");
			}
		} else {
			cell->sleep_samples = 0;

			if (!cell->relax) {

				if (now.tv_sec-cell->sleep_timer.tv_sec >
					cell->config->ocv->relax_period) {

					cell->relax = true;
					dev_dbg(cell->dev, "FG CHECK: Relaxed\n");
					cell->calibrate = true;
				}
			}
		}
	}

	return cell->relax;
}
