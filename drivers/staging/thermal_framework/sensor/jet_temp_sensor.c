/*
 * TMP102 Temperature sensor driver file
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Steven King <sfking@fdwdc.com>
 * Author: Sabatier, Sebastien" <s-sabatier1@ti.com>
 * Author: Mandrenko, Ievgen" <ievgen.mandrenko@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/stddef.h>
#include <linux/sysfs.h>
#include <linux/err.h>
#include <linux/reboot.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <plat/common.h>
#include <plat/tmp102_temp_sensor.h>

#include <plat/omap_device.h>
#include <plat/omap-pm.h>

#include <linux/thermal_framework.h>
#include <linux/omap4_duty_cycle_governor.h>

#define USER_SPACE_ALERT
#define DEFAULT_TEMPERATURE		45
#define	TMP102_TEMP_REG			0x00
#define	TMP102_CONF_REG			0x01
/* note: these bit definitions are byte swapped */
#define		TMP102_CONF_SD		0x0100  //shut down
#define		TMP102_CONF_TM		0x0200
#define		TMP102_CONF_POL		0x0400
#define		TMP102_CONF_F0		0x0800
#define		TMP102_CONF_F1		0x1000
#define		TMP102_CONF_R0		0x2000
#define		TMP102_CONF_R1		0x4000
#define		TMP102_CONF_OS		0x8000
#define		TMP102_CONF_EM		0x0010
#define		TMP102_CONF_AL		0x0020
#define		TMP102_CONF_CR0		0x0040
#define		TMP102_CONF_CR1		0x0080
#define		TMP102_TLOW_REG		0x02
#define		TMP102_THIGH_REG	0x03

#define HIGH_TEMP_THRESHOLD 60
enum temp_state {
	LOW_TEMP_STATE = 0,
	HIGH_TEMP_STATE,
};

struct tmp102_temp_sensor {
	struct i2c_client *iclient;
	struct mutex sensor_mutex;
	struct pcb_sens *tpcb;

	unsigned long last_update;
	int temp;
	enum temp_state state;
};
static struct tmp102_temp_sensor *tmp102_data;
/* SMBus specifies low byte first, but the TMP102 returns high byte first,
 * so we have to swab16 the values */
static inline int tmp102_read_reg(struct i2c_client *client, u8 reg)
{
	int result = i2c_smbus_read_word_data(client, reg);

	return result < 0 ? result : swab16(result);
}

static inline int tmp102_write_reg(struct i2c_client *client,
				u8 reg, u16 val)
{
	return i2c_smbus_write_word_data(client, reg, swab16(val));
}

/* convert left adjusted 13-bit TMP102 register value to milliCelsius */
static inline int tmp102_reg_to_mC(s16 val)
{
	return ((val & ~0x01) * 1000) / 128;
}

#ifdef USER_SPACE_ALERT
static void tmp102_uevent_send(struct tmp102_temp_sensor *tmp102)
{
	char buf[32];
	char *envp[3];

	envp[0] = "NAME=PCB_TEMPERATURE";
	snprintf(buf, 32, "TEMPERATURE=%d", tmp102->temp);
	envp[1] = buf;
	envp[2] =NULL;
	kobject_uevent_env(&(tmp102->iclient->dev.kobj),KOBJ_CHANGE,envp);
}
#endif

static int tmp102_read_current_temp(void)
{
	int temp_mC=0;
	struct tmp102_temp_sensor *tmp102=tmp102_data;
	mutex_lock(&tmp102->sensor_mutex);
	if (time_after(jiffies, tmp102->last_update + HZ / 3)) {
		int status = tmp102_read_reg(tmp102->iclient, TMP102_TEMP_REG);
		if (status > -1)
			temp_mC = tmp102_reg_to_mC(status);
			tmp102->temp=temp_mC/1000;
		tmp102->last_update = jiffies;
	}
	mutex_unlock(&tmp102->sensor_mutex);
#ifdef CONFIG_THERMAL_DEBUG
	printk(KERN_DEBUG "%s:%d\n", __func__, tmp102->temp);
#endif
#ifdef USER_SPACE_ALERT
	if(tmp102->temp>=HIGH_TEMP_THRESHOLD){
			tmp102->state=HIGH_TEMP_STATE;
			tmp102_uevent_send(tmp102);
	}else {
		if(tmp102->state==HIGH_TEMP_STATE){//The userspace don't care too much about low temp state
			tmp102->state=LOW_TEMP_STATE;
			tmp102_uevent_send(tmp102);
		}
	}
#endif
	return tmp102->temp;
}

static int omap_read_temp(struct device *dev,
				      struct device_attribute *devattr,
				      char *buf)
{
	int temp;
	struct i2c_client *client = to_i2c_client(dev);
	struct tmp102_temp_sensor *tmp102 = i2c_get_clientdata(client);
	mutex_lock(&tmp102->sensor_mutex);
	temp=tmp102->temp;
	mutex_unlock(&tmp102->sensor_mutex);

	return sprintf(buf, "%d\n", temp);
}

static DEVICE_ATTR(temperature, S_IRUGO, omap_read_temp,
			  NULL);
static struct attribute* temp102_attributes[] =
{
	&dev_attr_temperature.attr,
	NULL
};
static const struct attribute_group temp102_attr_group =
{
	.attrs = temp102_attributes,
};

#define TMP102_RESET  (TMP102_CONF_R0|TMP102_CONF_R1|TMP102_CONF_CR1)
#define TMP102_CONFIG  (TMP102_CONF_TM | TMP102_CONF_EM | TMP102_CONF_CR1)//Interrupt mode, Extended mode, 4Hz
#define TMP102_CONFIG_RD_ONLY (TMP102_CONF_R0 | TMP102_CONF_R1 | TMP102_CONF_AL)

static int __devinit tmp102_temp_sensor_probe(
		struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct tmp102_temp_sensor *tmp102;
#ifdef CONFIG_THERMAL_DEBUG
	printk(KERN_DEBUG "%s\n", __func__);
#endif
	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(&client->dev, "adapter doesn't support SMBus word "
			"transactions\n");

		return -ENODEV;
	}

	tmp102 = kzalloc(sizeof(struct tmp102_temp_sensor), GFP_KERNEL);
	if (!tmp102)
		return -ENOMEM;

	tmp102_data=tmp102;
	mutex_init(&tmp102->sensor_mutex);
	tmp102->iclient = client;

	i2c_set_clientdata(client, tmp102);
	/*Reset TMP102*/
	tmp102_write_reg(client, TMP102_CONF_REG,TMP102_RESET);
	if (ret < 0) {
		dev_err(&client->dev, "error writing config register\n");
		goto free_err;;
	}
	/*Config TMP102*/
	ret = tmp102_write_reg(client, TMP102_CONF_REG, TMP102_CONFIG);
	if (ret < 0) {
		dev_err(&client->dev, "error writing config register\n");
		goto restore_config_err;
	}
	/*Verify config*/
	ret = tmp102_read_reg(client, TMP102_CONF_REG);
	if (ret < 0) {
		dev_err(&client->dev, "error reading config register\n");
		goto restore_config_err;
	}
#ifdef CONFIG_THERMAL_DEBUG
	printk(KERN_DEBUG "config=0x%X\n", ret);
#endif
	ret &= ~TMP102_CONFIG_RD_ONLY;
	if (ret != TMP102_CONFIG) {
		dev_err(&client->dev, "config settings did not stick\n");
		ret = -ENODEV;
		goto restore_config_err;
	}

	tmp102->last_update = jiffies - HZ;
	tmp102->temp=DEFAULT_TEMPERATURE;
	tmp102->state=LOW_TEMP_STATE;

	ret = sysfs_create_group(&client->dev.kobj, &temp102_attr_group);
	if (ret){
		dev_err(&client->dev, "unable to create attribute\n");
		goto sysfs_create_err;
	}

	tmp102->tpcb=kzalloc(sizeof(struct pcb_sens), GFP_KERNEL);
	if(tmp102->tpcb){
		tmp102->tpcb->update_temp=tmp102_read_current_temp;
		omap4_duty_pcb_register(tmp102->tpcb);
	}else {
		ret = -ENOMEM;
		goto tpcb_alloc_err;
	}

	dev_info(&client->dev, "initialized\n");

	return 0;

tpcb_alloc_err:
	sysfs_remove_group(&client->dev.kobj, &temp102_attr_group);
sysfs_create_err:
restore_config_err:
	tmp102_write_reg(client, TMP102_CONF_REG,(TMP102_RESET|TMP102_CONF_SD));
free_err:
	mutex_destroy(&tmp102->sensor_mutex);
	kfree(tmp102);

	return ret;
}

static int __devexit tmp102_temp_sensor_remove(struct i2c_client *client)
{
	struct tmp102_temp_sensor *tmp102 = i2c_get_clientdata(client);
	sysfs_remove_group(&client->dev.kobj, &temp102_attr_group);
	/* Reset TMP102 and Stop monitoring*/
	tmp102_write_reg(client, TMP102_CONF_REG,(TMP102_RESET|TMP102_CONF_SD));

	kfree(tmp102->tpcb);
	kfree(tmp102);

	return 0;
}

#ifdef CONFIG_PM
static int tmp102_temp_sensor_suspend(struct i2c_client *client,
			pm_message_t mesg)
{
	int conf = tmp102_read_reg(client, TMP102_CONF_REG);

	if (conf < 0)
		return conf;
	conf |= TMP102_CONF_SD;

	return tmp102_write_reg(client, TMP102_CONF_REG, conf);
}

static int tmp102_temp_sensor_resume(struct i2c_client *client)
{
	int conf = tmp102_read_reg(client, TMP102_CONF_REG);

	if (conf < 0)
		return conf;
	conf &= ~TMP102_CONF_SD;

	return tmp102_write_reg(client, TMP102_CONF_REG, conf);
}

#else

tmp102_temp_sensor_suspend NULL
tmp102_temp_sensor_resume NULL

#endif /* CONFIG_PM */


static const struct i2c_device_id tmp102_id[] = {
	{ "tmp102_temp_sensor", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tmp102_id);

static struct i2c_driver tmp102_driver = {
	.class		= I2C_CLASS_HWMON,
	.probe = tmp102_temp_sensor_probe,
	.remove = tmp102_temp_sensor_remove,
	.suspend = tmp102_temp_sensor_suspend,
	.resume = tmp102_temp_sensor_resume,
	.driver = {
		//.owner = THIS_MODULE,
		.name = "tmp102_temp_sensor",
	},
	.id_table	= tmp102_id,
};

static int __init tmp102_init(void)
{
	return i2c_add_driver(&tmp102_driver);
}
module_init(tmp102_init);

static void __exit tmp102_exit(void)
{
	i2c_del_driver(&tmp102_driver);
}
module_exit(tmp102_exit);

MODULE_DESCRIPTION("Reconinstruemnts temperature sensor");
MODULE_AUTHOR("Li Chen");
MODULE_LICENSE("GPL");
