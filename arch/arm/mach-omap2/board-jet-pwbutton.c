/*
 * arch/arm/mach-omap2/board-jet-pwbutton.c
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

#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/leds.h>
#include <linux/platform_device.h>

//#include <plat/omap4-pwbutton.h>

#include "board-jet.h"
#include "mux.h"

static struct platform_device jet_pwbutton_led = {
    .name   =   "pwbutton_led",
    .id =   -1,
    .dev    = {
        .platform_data = NULL,
    },
};

static struct platform_device *jet_led_devices[] __initdata = {
    &jet_pwbutton_led,
};

int __init jet_pwbutton_init(void)
{
    platform_add_devices(jet_led_devices, ARRAY_SIZE(jet_led_devices));

    return 0;
}
