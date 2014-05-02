/*
 * arch/arm/mach-omap2/board-jet-sensors.c
 *
 * Copyright (C) 2011 Texas Instruments
 *
 * Author: Dan Murphy <DMurphy@TI.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <plat/i2c.h>

#include "board-jet.h"
#include "mux.h"

#if defined (CONFIG_JET_SENSORS_FREE_FALL)|| defined(CONFIG_JET_SENSORS_TAP_TAP)
#include <linux/i2c/jet_sensor_platform.h>
#define GPIO_INT1_A_IRQ     59
#define GPIO_INT2_A_IRQ     60
static struct jet_sensor_platform_data jet_sensor_data = {
#ifdef CONFIG_JET_SENSORS_FREE_FALL
	.irq_ff=OMAP_GPIO_IRQ(GPIO_INT2_A_IRQ),
#endif
#ifdef CONFIG_JET_SENSORS_TAP_TAP
	.irq_tt=OMAP_GPIO_IRQ(GPIO_INT1_A_IRQ),
#endif
};
#endif

#define EYE_TRACING_INT 56

static struct i2c_board_info __initdata jet_bus2_sensor_boardinfo[] = {
	{
#if defined(CONFIG_JET_V2)
		I2C_BOARD_INFO("jet_sensors", 0x1D),
#else
		I2C_BOARD_INFO("jet_sensors", 0x1E),
#endif
#if defined (CONFIG_JET_SENSORS_FREE_FALL)|| defined(CONFIG_JET_SENSORS_TAP_TAP)
		.platform_data = &jet_sensor_data,//use platform data instead of i2c client irq for future use
#endif
	},
	
#if defined(CONFIG_JET_SUN) && defined(CONFIG_JET_V2)
	{
		I2C_BOARD_INFO("apds9900", 0x39),
		.irq=OMAP_GPIO_IRQ(EYE_TRACING_INT),
	},
#endif
};

static struct i2c_board_info __initdata jet_bus3_sensor_boardinfo[] = {
	{
		I2C_BOARD_INFO("mfi", 0x11),
	},
#if defined(CONFIG_JET_SUN) && defined(CONFIG_JET_V2)
	{
		I2C_BOARD_INFO("tfa98xx", 0x34),
	},
#endif
};

static struct i2c_board_info __initdata jet_bus4_sensor_boardinfo[] = {
	{
		I2C_BOARD_INFO("tmp102_temp_sensor", 0x49),
	},
#if defined(CONFIG_JET_TMP103_FLEX)
	{
		I2C_BOARD_INFO("tmpflex_103", 0x70),
	},
#endif
#ifdef CONFIG_MOUSE_OFM_PARTRON
	{
		I2C_BOARD_INFO("ofm_driver", 0x53),
	},
#endif
#ifdef CONFIG_SOC_CAMERA_MT9M114
    {
        I2C_BOARD_INFO("mt9m114", 0x48),
    },
#endif
};

int __init jet_sensor_init(void)
{
#if defined (CONFIG_JET_SENSORS_FREE_FALL)|| defined(CONFIG_JET_SENSORS_TAP_TAP)
	omap_mux_init_signal("gpmc_nbe0_cle.gpio_59", OMAP_PIN_INPUT);
	omap_mux_init_gpio(GPIO_INT1_A_IRQ, OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE);

	omap_mux_init_signal("gpmc_nbe1.gpio_60", OMAP_PIN_INPUT);
	omap_mux_init_gpio(GPIO_INT2_A_IRQ, OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE);
#endif

	omap_register_i2c_bus(2, 400, jet_bus2_sensor_boardinfo,
		ARRAY_SIZE(jet_bus2_sensor_boardinfo));

	omap_register_i2c_bus(3, 400, jet_bus3_sensor_boardinfo,
		ARRAY_SIZE(jet_bus3_sensor_boardinfo));

	omap_register_i2c_bus(4, 400, jet_bus4_sensor_boardinfo,
		ARRAY_SIZE(jet_bus4_sensor_boardinfo));

	//platform_device_register(&jet_proximity_device);

	return 0;
}
