#ifndef _JET_SENSOR_PLATFORM_H
#define _JET_SENSOR_PLATFORM_H

struct jet_sensor_platform_data {
	/* set gpio_int[1,2] either to the choosen gpio pin number or to -EINVAL
	 * if leaved unconnected
	 */
	
#ifdef CONFIG_JET_SENSORS_FREE_FALL
	int irq_ff;
#endif
#ifdef CONFIG_JET_SENSORS_TAP_TAP
	int irq_tt;
#endif
};

#endif
