/*
 * OMAP4430 JET display LED Driver
 *
 * Copyright (C) 2010 Texas Instruments
 * Copyright (C) 2013 Recon Instruments
 *
 * Author: Dan Murphy <DMurphy@ti.com>
 *         Gil Zhaiek <gil@reconinstruments.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/leds-omap4430jet-display.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/i2c/twl.h>

struct display_led_data {
	struct led_classdev pri_display_class_dev;
	struct omap4430_jet_disp_led_platform_data *led_pdata;
	struct mutex pri_disp_lock;
};

static void omap4430_jet_primary_disp_store(struct led_classdev *led_cdev, enum led_brightness value)
{
	struct display_led_data *led_data = container_of(led_cdev, struct display_led_data, pri_display_class_dev);
	mutex_lock(&led_data->pri_disp_lock);

	if (led_data->led_pdata->primary_display_set)
		led_data->led_pdata->primary_display_set(value);

	mutex_unlock(&led_data->pri_disp_lock);
}

static int omap4430_jet_display_probe(struct platform_device *pdev)
{
	int ret;
	struct display_led_data *info;

	printk(KERN_INFO "[%s:%u] Enter\n", __FUNCTION__,__LINE__);

	if (pdev->dev.platform_data == NULL) {
		printk(KERN_ERR "[%s:%u] platform data required\n", __FUNCTION__,__LINE__);
		return -ENODEV;
	}

	info = kzalloc(sizeof(struct display_led_data), GFP_KERNEL);
	if (info == NULL) {
		printk(KERN_ERR "[%s:%u] kzalloc failed\n", __FUNCTION__,__LINE__);
		return -ENOMEM;
	}

	info->led_pdata = pdev->dev.platform_data;
	platform_set_drvdata(pdev, info);

	info->pri_display_class_dev.name = "lcd-backlight";
	info->pri_display_class_dev.brightness_set = omap4430_jet_primary_disp_store;
	info->pri_display_class_dev.max_brightness = LED_FULL;
	mutex_init(&info->pri_disp_lock);

	ret = led_classdev_register(&pdev->dev, &info->pri_display_class_dev);
	if (ret < 0) {
		printk(KERN_ERR "[%s:%u] Register LED class failed\n", __FUNCTION__,__LINE__);
		kfree(info);
		return ret;
	}

	if (info->led_pdata->display_led_init) {
		printk(KERN_INFO "[%s:%u]\n", __FUNCTION__,__LINE__);
		info->led_pdata->display_led_init();
	}

	printk(KERN_INFO "[%s:%u] Exit\n", __FUNCTION__,__LINE__);

	return ret;
}

static int omap4430_jet_display_remove(struct platform_device *pdev)
{
	struct display_led_data *info = platform_get_drvdata(pdev);
	led_classdev_unregister(&info->pri_display_class_dev);

	return 0;
}

static struct platform_driver omap4430_jet_display_driver = {
	.probe = omap4430_jet_display_probe,
	.remove = omap4430_jet_display_remove,
	.driver = {
		.name = "display_led",
		.owner = THIS_MODULE,
	},
};

static int __init omap4430_jet_display_init(void)
{
	return platform_driver_register(&omap4430_jet_display_driver);
}

static void __exit omap4430_jet_display_exit(void)
{
	platform_driver_unregister(&omap4430_jet_display_driver);
}

module_init(omap4430_jet_display_init);
module_exit(omap4430_jet_display_exit);

MODULE_DESCRIPTION("OMAP4430 JET Display Backlight Driver");
MODULE_AUTHOR("Dan Murphy <DMurphy@ti.com> / Gil Zhaiek <gil@reconinstruments.com");
MODULE_LICENSE("GPL");
