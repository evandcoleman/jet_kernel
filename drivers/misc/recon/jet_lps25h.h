#ifndef __JET_LPS25_H__
#define __JET_LPS25_H__
/* PRESSURE_INT_ALIGNMENT is used to pack each pressure data into int data type (4 bytes)
 * Disable PRESSURE_INT_ALIGNMENT for fast data transfer in future
 * */
#define PRESSURE_INT_ALIGNMENT
#define LPS_FIFO_DEEP 				32 
#define LPS_ONEEVENT_RAW_SIZE	3 //pressure data/4096 to get mbar:= right shit 12 bits
#define LPS_FIFO_LENGTH_MAX			96 //LPS_FIFO_DEEP*PRESSURE_ONEEVENT_RAW_SIZE

#ifdef PRESSURE_INT_ALIGNMENT
#define LPS_ONEEVENT_SIZE 4
#define LPS_DATA_LENGTH_MAX				128//(32 FIFO)*(4 bytes each pressure)
#else
#define LPS_ONEEVENT_SIZE LPS_ONEEVENT_RAW_SIZE
#define LPS_DATA_LENGTH_MAX		LPS_FIFO_LENGTH_MAX
#endif

int lp225h_pressure_hw_init(struct jet_sensors *jet_sensors);

int lp225h_pressure_set_mode(struct jet_sensors *jet_sensors,  unsigned char flag, 
							unsigned char mode, unsigned int ms_delay);

int lp225h_pressure_get_data(struct jet_sensors *jet_sensors, u8 *sensor_data_ptr, unsigned int length);

int lp225h_temperature_get_data(struct jet_sensors *jet_sensors, short *temp_ptr);
#endif