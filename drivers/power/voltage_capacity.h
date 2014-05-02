#ifndef __VOLTAGE_CAPACITY_H
#define __VOLTAGE_CAPACITY_H
/* Size of the OCV Lookup table */
#define VOLT_CAPACITY_TABLE_SIZE	21
struct volt_cap_table{
	short temperature_value; //temperature value, 0.1 degree Celsius
	short temp_max_capacity;
	unsigned short table[VOLT_CAPACITY_TABLE_SIZE];
};

short get_average_voltage(short voltage);
unsigned int voltage_cap_convert(short voltage,short temperature);
unsigned int voltage_cap_init(short voltage,short temperature);
#endif