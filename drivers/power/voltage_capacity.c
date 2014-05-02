#include <linux/types.h>
#include <linux/kernel.h>
#include "voltage_capacity.h"

/* Voltage and Current buffers */
#define AV_SIZE	5

static unsigned short av_v_index;
static short av_v[AV_SIZE];
#define INTERPOLATE_MAX		1000
#define MAX_PERCENTAGE		100
static struct volt_cap_table temperatue_config_table[]=
{
	{
		.temperature_value=250,//use ENV25,BAT25 curve
		.temp_max_capacity=1183,
		.table={
			3578,3615,3645,3669,3688,
			3703,3716,3727,3739,3754,
			3772,3793,3821,3852,3883,
			3915,3948,3984,4023,4065,
			4126
		},
	},
	{
		.temperature_value=60,//use ENV5,BAT6 curve
		.temp_max_capacity=1148,
		.table={
			3498,3562,3593,3615,3633,
			3648,3660,3673,3688,3705,
			3724,3747,3772,3799,3830,
			3863,3898,3936,3976,4019,
			4091
		},
	},
	{
		.temperature_value=10,//use ENV0,BAT1 curve
		.temp_max_capacity=1119,
		.table={
			3459,3522,3558,3581,3598,
			3613,3626,3640,3656,3674,
			3695,3718,3743,3771,3801,
			3834,3869,3908,3949,3992,
			4068
		},
	},
	{
		.temperature_value=-30,//use ENV-5,BAT-3 curve
		.temp_max_capacity=1074,
		.table={
			3404,3466,3502,3528,3547,
			3563,3578,3594,3611,3630,
			3652,3675,3700,3729,3758,
			3791,3826,3865,3907,3955,
			4039
		},
	},
	{
		.temperature_value=-70,//use ENV-10,BAT-7 curve
		.temp_max_capacity=996,
		.table={
			3338,3390,3426,3453,3475,
			3494,3512,3529,3548,3569,
			3592,3617,3642,3670,3700,
			3733,3768,3805,3847,3896,
			3997
		},
	},
	{
		.temperature_value=-100,//use ENV-15,BAT-10
		.temp_max_capacity=854,
		.table={
			3269,3302,3330,3354,3376,
			3398,3418,3439,3460,3482,
			3506,3531,3558,3587,3617,
			3650,3684,3722,3765,3818,
			3940
		},
	},
	{
		.temperature_value=-150,//use ENV-20,BAT-15
		.temp_max_capacity=651,
		.table={
			3238,3254,3270,3284,3300,
			3314,3330,3344,3360,3374,
			3390,3404,3420,3433,3448,
			3461,3473,3487,3524,3643,
			3845
		},
	},
};

unsigned short interpolate(unsigned short value,
				unsigned short *table,
				unsigned char size)
{
	unsigned char i;
	unsigned short d;

	int delta= INTERPOLATE_MAX/(size-1);
	for (i = 0; i < size; i++)
		if (value < table[i])
			break;

	if (i==0) {
		d=0;
	} else if(i<size){
		d = (value - table[i-1]) * delta;
		d /=  table[i] - table[i-1];
		d = d + (i-1) * delta;
	} else {
		d=1000;
	}

	if (d > 1000)
		d = 1000;

	return d;
}

unsigned int voltage_cap_convert(short voltage,short temperature)
{
	unsigned int cap,i,tmp,low_tmp,high_tmp;
	unsigned int low_index,high_index;
	int table_size = ARRAY_SIZE(temperatue_config_table);
	unsigned short *table;
	short delta_temperature;
	/*find proper temperature range*/
	for(i=0; i<table_size; i++)
	{
		if(temperature>=temperatue_config_table[i].temperature_value)
			break;
	}

	if(i==0 )
		high_index=i;
	else
		high_index=i-1;

	if(i==table_size)
		low_index=i-1;
	else
		low_index=i;

	low_tmp=0;
	table=temperatue_config_table[high_index].table;
	high_tmp = interpolate(voltage, table,
			VOLT_CAPACITY_TABLE_SIZE);
	
	if(low_index!=high_index){
		table=temperatue_config_table[low_index].table;
		low_tmp = interpolate(voltage, table,
			VOLT_CAPACITY_TABLE_SIZE);
		/*Add weight*/
		delta_temperature=temperatue_config_table[high_index].temperature_value-
							temperatue_config_table[low_index].temperature_value;
		high_tmp= high_tmp * (temperature-temperatue_config_table[low_index].temperature_value);
		low_tmp= low_tmp * (temperatue_config_table[high_index].temperature_value-temperature);
		tmp=DIV_ROUND_CLOSEST((high_tmp+low_tmp), delta_temperature);
	}else
		tmp=high_tmp;

	cap=DIV_ROUND_CLOSEST(tmp * MAX_PERCENTAGE, INTERPOLATE_MAX);

#ifdef DEBUG
	printk(KERN_DEBUG "Battery cap:%d:%d,%d,%d\n"
			"Volt:%d,Temp:%d\n", 
			cap,i,high_tmp,low_tmp,
			voltage,temperature);
#endif
	return cap;
}

short get_average_voltage(short voltage)
{
	int i;
	int tmp=0;
	av_v_index++;
	av_v_index %= AV_SIZE;
	av_v[av_v_index] = voltage;
	for (i = 0; i < AV_SIZE; i++)
		tmp += av_v[i];
	return (short)(tmp/AV_SIZE);
}
unsigned int voltage_cap_init(short voltage,short temperature)
{
	unsigned short i;

	for (i = 0; i < AV_SIZE; i++) {
		av_v[i] = voltage;
	}
	av_v_index = 0;
	return voltage_cap_convert(voltage, temperature);
}