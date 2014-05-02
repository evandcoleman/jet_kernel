#ifndef __JET_tmp103_FLEX_H__
#define __JET_tmp103_FLEX_H__
#include "jet_sensors.h"

#define TMP_ONEEVENT_SIZE 2
struct tmp103_data
{
	struct i2c_client *client;
};

#if defined SENSOR_DEBUG_VERBOSE
extern struct tmp103_data *jet_temp_ptr;
int tmp103_hw_init(struct tmp103_data *jet_temp);
int tmp103_set_mode(struct tmp103_data *jet_temp,  unsigned char flag,
							unsigned char mode, unsigned int ms_delay);
int tmp103_get_data(struct tmp103_data *jet_temp, u8 *sensor_data_ptr, unsigned int length);
#if defined(CONFIG_JET_PROXMUX) || defined(CONFIG_JET_PROXMUX_MODULE)
int jet_tmp103_get_data(struct tmp103_data *jet_temp, u8* sensor_data_ptr,
						unsigned int length, RI_SENSOR_HANDLE handle);
#endif

#endif

#endif