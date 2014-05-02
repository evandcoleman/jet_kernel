#ifndef __JET_LSM9DS0_H__
#define __JET_LSM9DS0_H__

#ifdef CONFIG_JET_SENSORS_TAP_TAP
#include <linux/input.h>
#endif

#define LSM_DATA_ONEEVENT_SIZE 6
#define LSM_FIFO_LENGTH_MAX 192//(32 FIFO)*(3 axes)*(2 bytes each axis)
int lsm9ds0_gyro_hw_init(struct jet_sensors *jet_sensors);
int lsm9ds0_acc_hw_init(struct jet_sensors *jet_sensors);
int lsm9ds0_mag_hw_init(struct jet_sensors *jet_sensors);

int lsm9ds0_gyro_set_mode(struct jet_sensors *jet_sensors,  unsigned char flag, 
							unsigned char mode, unsigned int ms_delay);
int lsm9ds0_acc_set_mode(struct jet_sensors *jet_sensors,  unsigned char flag, 
							unsigned char mode, unsigned int ms_delay);
int lsm9ds0_mag_set_mode(struct jet_sensors *jet_sensors,  unsigned char flag, 
							unsigned char mode, unsigned int ms_delay);

int lsm9ds0_gyro_get_data(struct jet_sensors *jet_sensors, u8 *sensor_data_ptr, unsigned int length);
int lsm9ds0_acc_get_data(struct jet_sensors *jet_sensors, u8 *sensor_data_ptr, unsigned int length);
int lsm9ds0_mag_get_data(struct jet_sensors *jet_sensors, u8 *sensor_data_ptr, unsigned int length);

#ifdef CONFIG_JET_SENSORS_FREE_FALL
int lsm9ds0_freefall_hw_init(struct jet_sensors *jet_sensors);
void freefall_irq_work_func(struct work_struct *work);
int lsm9ds0_freefall_set_mode(struct jet_sensors *jet_sensors,  unsigned char flag);
#endif

#ifdef CONFIG_JET_SENSORS_TAP_TAP
int lsm9ds0_tap_tap_hw_init(struct jet_sensors *jet_sensors);
void tap_tap_irq_work_func(struct work_struct *work);
#endif
#endif
