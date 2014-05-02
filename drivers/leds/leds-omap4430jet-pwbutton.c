/*
 * Phoenix pwbutton LED Driver for the OMAP4430 SDP
 *
 * Copyright (C) 2010 Texas Instruments
 *
 * Author: Dan Murphy <DMurphy@ti.com>
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
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/i2c/twl.h>

struct pwbutton_led_data {
	struct	led_classdev pwbutton_led_class_dev;
	struct	work_struct set_brightness_work;
	unsigned long	blink_delay_on, blink_delay_off;
};

#define PW_LED_PWM2ON		0xBD
#define PW_LED_PWM2OFF		0xBE
#define PW_LED_TOGGLE3		0x92

#define PWM2EN_CLOCK_INPUT		(1 << 5)
#define PWM2EN_SIGNAL		(1 << 4)
#define PWM2_RESET		(1 << 3)
#define PWM1_MASK			0x7

#define LED_DEFAULT_VALUE 62
#define PW_LED_PWM2ON_DEFAULT_VALUE 0x60
//#define USR_CHANGE_BRIGHTNESS
static void pwled_set_brightness(enum led_brightness value)
{
	u8 brightness = 0;
	if (value) {
#ifdef USR_CHANGE_BRIGHTNESS
		if (value == LED_FULL)
			brightness = 0x7f;
		else
			brightness = (~(value/2)) & 0x7f;
#else
		brightness=PW_LED_PWM2ON_DEFAULT_VALUE;
#endif
		twl_i2c_write_u8(TWL6030_MODULE_ID1, brightness, PW_LED_PWM2ON);
		twl_i2c_write_u8(TWL6030_MODULE_ID1, (PWM2EN_CLOCK_INPUT|PWM2EN_SIGNAL), PW_LED_TOGGLE3);
	} else {
		twl_i2c_write_u8(TWL6030_MODULE_ID1, (PWM2_RESET), PW_LED_TOGGLE3);
		twl_i2c_write_u8(TWL6030_MODULE_ID1, (PWM2EN_CLOCK_INPUT|PWM2EN_SIGNAL|PWM2_RESET), PW_LED_TOGGLE3);
	}
}

static void set_brightness_work_func(struct work_struct *work)
{
	struct pwbutton_led_data *info=container_of(work, struct pwbutton_led_data, set_brightness_work);
	pwled_set_brightness(info->pwbutton_led_class_dev.brightness);
}
//The led_brightness value here is the same to the led_cdev->brightness due to inheritance from the led_classdev
static void omap4430_pwbutton_led_store(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	struct pwbutton_led_data *info=container_of(led_cdev, struct pwbutton_led_data, pwbutton_led_class_dev);
	//printk("%s:%d,%lu,%lu\n",__func__,led_cdev->brightness,led_cdev->blink_delay_on, led_cdev->blink_delay_off);
	schedule_work(&info->set_brightness_work);
}


static ssize_t set_delay_on(struct device *dev, struct device_attribute *attr, 
			const char *buf, size_t count)
{
	unsigned long val;
	struct led_classdev *led =dev_get_drvdata(dev);
	struct pwbutton_led_data *info=container_of(led, struct pwbutton_led_data, pwbutton_led_class_dev);
	if (strict_strtoul(buf, 10, &val) < 0)
		return -EINVAL;
	del_timer_sync(&led->blink_timer);//Make sure to stop timer for update
	info->blink_delay_on=val;
	return count;
}


static ssize_t set_delay_off(struct device *dev, struct device_attribute *attr, 
			const char *buf, size_t count)
{
	unsigned long val;
	struct led_classdev *led =dev_get_drvdata(dev);
	struct pwbutton_led_data *info=container_of(led, struct pwbutton_led_data, pwbutton_led_class_dev);
	if (strict_strtoul(buf, 10, &val) < 0)
		return -EINVAL;

	info->blink_delay_off=val;
	if(info->blink_delay_on  && info->blink_delay_off)
		led_blink_set(led, &info->blink_delay_on, &info->blink_delay_off);
	else{
		del_timer_sync(&led->blink_timer);
		led->blink_delay_on=0;
		led->blink_delay_off=0;
	}
	return count;
}

static DEVICE_ATTR(delay_on, S_IWUSR|S_IRUGO, NULL, set_delay_on);
static DEVICE_ATTR(delay_off, S_IWUSR|S_IRUGO, NULL, set_delay_off);
static struct attribute* pwbutton_led_attributes[] = 
{
	&dev_attr_delay_on.attr,
	&dev_attr_delay_off.attr,
	NULL
};

static const struct attribute_group pwbutton_led_attr_group = 
{
	.attrs = pwbutton_led_attributes,
}; 

static int omap4430_pwbutton_led_probe(struct platform_device *pdev)
{
	int ret;
	struct pwbutton_led_data *info;

	pr_info("%s:Enter\n", __func__);

	info = kzalloc(sizeof(struct pwbutton_led_data), GFP_KERNEL);
	if (info == NULL) {
		ret = -ENOMEM;
		return ret;
	}

	platform_set_drvdata(pdev, info);
	memset(info, 0, sizeof(struct pwbutton_led_data));
	info->pwbutton_led_class_dev.name = "pwbutton_light";
	info->pwbutton_led_class_dev.brightness_set =
			omap4430_pwbutton_led_store;
	info->pwbutton_led_class_dev.max_brightness = LED_FULL;
	info->pwbutton_led_class_dev.brightness = LED_DEFAULT_VALUE;
	info->pwbutton_led_class_dev.blink_brightness = LED_DEFAULT_VALUE;

	ret = led_classdev_register(&pdev->dev,
					&info->pwbutton_led_class_dev);
	if (ret < 0) {
		pr_err("%s: Register led class failed\n", __func__);
		kfree(info);
		return ret;
	}

	INIT_WORK(&info->set_brightness_work, set_brightness_work_func);

	ret=sysfs_create_group(&(info->pwbutton_led_class_dev.dev->kobj), &pwbutton_led_attr_group);
	if (ret) 
	{
		dev_err(info->pwbutton_led_class_dev.dev, "unable to create attribute\n");
		goto sysfs_create_err;
	}
	twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x7F, PW_LED_PWM2OFF);

	pr_info("%s:Exit\n", __func__);
	return ret;
sysfs_create_err:
	led_classdev_unregister(&info->pwbutton_led_class_dev);
	kfree(info);
	return ret;
}

static int omap4430_pwbutton_led_remove(struct platform_device *pdev)
{
	struct pwbutton_led_data *info = platform_get_drvdata(pdev);
	sysfs_remove_group(&(info->pwbutton_led_class_dev.dev->kobj), &pwbutton_led_attr_group);
	led_classdev_unregister(&info->pwbutton_led_class_dev);
	kfree(info);
	return 0;
}

static struct platform_driver omap4430_pwbutton_led_driver = {
	.probe = omap4430_pwbutton_led_probe,
	.remove = omap4430_pwbutton_led_remove,
	.driver = {
		   .name = "pwbutton_led",
		   .owner = THIS_MODULE,
		   },
};

static int __init omap4430_pwbutton_led_init(void)
{
	return platform_driver_register(&omap4430_pwbutton_led_driver);
}

static void __exit omap4430_pwbutton_led_exit(void)
{
	platform_driver_unregister(&omap4430_pwbutton_led_driver);
}

module_init(omap4430_pwbutton_led_init);
module_exit(omap4430_pwbutton_led_exit);

MODULE_DESCRIPTION("OMAP4430 JET pwbutton Lighting driver");
MODULE_AUTHOR("Li Chen <li@reconinstruments.com");
MODULE_LICENSE("GPL");
