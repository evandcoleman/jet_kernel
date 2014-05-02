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
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/jet_camera.h>
struct jet_mt9m114_data {
	int enable;
};

static ssize_t store_camera_power(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct jet_mt9m114_data *jet_mt9m114= dev_get_drvdata(dev);
	struct cam_plat_data *cam_pdata= dev->platform_data;

	if(cam_pdata->ext_clk==NULL)
		return count;

	sscanf(buf, "%d", &jet_mt9m114->enable);
	printk("%s:%d\n",__func__,jet_mt9m114->enable);
	if(jet_mt9m114->enable){//camera on
		clk_enable(cam_pdata->ext_clk);
		mdelay(1);
		gpio_set_value(cam_pdata->reset_gpio, 1);
		mdelay(45);
	}
	else {//camera off
		gpio_set_value(cam_pdata->reset_gpio, 0);
		mdelay(1);
		clk_disable(cam_pdata->ext_clk);
	}

	return count;
}

static ssize_t show_camera_power(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct jet_mt9m114_data *jet_mt9m114= dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", jet_mt9m114->enable);
}

static struct kobj_attribute camera_power =
__ATTR(camera_power, 0666, (void *)show_camera_power, (void *)store_camera_power);

static struct attribute *jet_mt9m114_attrs[] = {
	&camera_power.attr,
	NULL,
};

static struct attribute_group jet_mt9m114_group = {
	.attrs = jet_mt9m114_attrs,
};
static int jet_mt9m114_probe(struct platform_device *pdev)
{
	struct jet_mt9m114_data *jet_mt9m114;
	int ret;

	jet_mt9m114=kzalloc(sizeof(struct jet_mt9m114_data), GFP_KERNEL);
	if(!jet_mt9m114) {
		pr_err("no mem to allocate");
		return -ENOMEM;
	}

	dev_set_drvdata(&pdev->dev, jet_mt9m114);
	jet_mt9m114->enable=0;

	ret = sysfs_create_group(&pdev->dev.kobj, &jet_mt9m114_group);
	if (ret) {
		pr_err("failed to create sysfs entries");
		goto sysfs_create_err;
	}

	pr_info("%s:succeed\n", __func__);
	return ret;
sysfs_create_err:
	kfree(jet_mt9m114);
	return ret;
}

static int jet_mt9m114_remove(struct platform_device *pdev)
{
	struct jet_mt9m114_data *jet_mt9m114= dev_get_drvdata(&pdev->dev);
	struct cam_plat_data *cam_pdata= pdev->dev.platform_data;
	pr_info("%s",__func__);
	gpio_free(cam_pdata->reset_gpio);
	sysfs_remove_group(&pdev->dev.kobj, &jet_mt9m114_group);
	kfree(jet_mt9m114);
	return 0;
}

static struct platform_driver jet_mt9m114_driver = {
	.probe = jet_mt9m114_probe,
	.remove = jet_mt9m114_remove,
	.driver = {
		   .name = "cam_mt9m114",
		   .owner = THIS_MODULE,
		   },
};

static int __init jet_mt9m114_init(void)
{
	return platform_driver_register(&jet_mt9m114_driver);
}

static void __exit jet_mt9m114_exit(void)
{
	platform_driver_unregister(&jet_mt9m114_driver);
}

module_init(jet_mt9m114_init);
module_exit(jet_mt9m114_exit);

MODULE_DESCRIPTION("JET Camera driver");
MODULE_AUTHOR("Li Chen <li@reconinstruments.com");
MODULE_LICENSE("GPL");
