#ifndef __JET_SENSORS_H__
#define __JET_SENSORS_H__
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include "i2c_transfer.h"

#if defined (CONFIG_JET_SENSORS_FREE_FALL)|| defined(CONFIG_JET_SENSORS_TAP_TAP)
#include <linux/i2c/jet_sensor_platform.h>
#endif
#ifdef CONFIG_JET_SENSORS_FREE_FALL
#define IRQ_ENABLE_FROM_CPU (0)
#define IRQ_DISABLE_FROM_CPU (1)
#endif

#ifdef CONFIG_JET_SENSORS_DEBUG
#define SENSOR_DEBUG
#define SENSOR_DEBUG_VERBOSE
#endif

#define FREEFALL_API

#define GPIO_INT1_A_IRQ 	59
#define GPIO_INT2_A_IRQ 	60

#if defined SENSOR_DEBUG
	#define SENSOR_INFO(fmt, args...) printk("jet_sensors: " fmt, ##args)//printk(KERN_DEBUG fmt, ##args)
#else
	#define SENSOR_INFO(fmt, args...)// nothing
#endif
struct jet_sensors
{
	struct i2c_client *client;
	struct mutex lock;
	struct miscdevice dev;
#if defined (CONFIG_JET_SENSORS_FREE_FALL)|| defined(CONFIG_JET_SENSORS_TAP_TAP)
	struct jet_sensor_platform_data *pdata;
#endif
#ifdef CONFIG_JET_SENSORS_FREE_FALL
	atomic_t irq_ff_status;
	struct work_struct freefall_irq_work;
#endif
#ifdef CONFIG_JET_SENSORS_TAP_TAP
	struct work_struct tap_tap_irq_work;
	struct input_dev *tap_tap_input_dev;
#endif
};

#if defined(CONFIG_JET_PROXMUX) || defined(CONFIG_JET_PROXMUX_MODULE)
#include "risensors_def.h"
#define SENSOR_DATA_SIZE_INDEX		sizeof(RI_SENSOR_HANDLE)		//4
#define SENSOR_DATA_INDEX			(SENSOR_DATA_SIZE_INDEX+sizeof(RI_DATA_SIZE))		//6
#define SENSOR_DATA_HEADER_SIZE		SENSOR_DATA_INDEX

#define SENSOR_TYPE_PTR(x)			(RI_SENSOR_HANDLE*)(x)
#define SENSOR_DATA_SIZE_PTR(x)		(RI_DATA_SIZE*)(x+SENSOR_DATA_SIZE_INDEX)
#define SENSOR_PAYLOAD_PTR(x)		x+SENSOR_DATA_INDEX

#define SENSOR_TYPE(x)				*SENSOR_TYPE_PTR(x)
#define SENSOR_DATA_SIZE(x)			*SENSOR_DATA_SIZE_PTR(x)
#define SENSOR_PAYLOAD(x)			*SENSOR_PAYLOAD_PTR(x)

#else

#define RI_SENSOR_MODE_CR			0x01      // Sensor is in CR   mode
#define RI_SENSOR_MODE_FIFO			0x02     // Sensor is in FIFO mode
#endif

int jet_i2c_read(struct jet_sensors *jet_sensors, u8 chip_address,
					u8 reg_address, u8 *buf, int len);
int jet_i2c_write(struct jet_sensors *jet_sensors, u8 chip_address,
					u8 *buf, int len);
#endif

