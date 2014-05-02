/******************** (C) COPYRIGHT 2011 Recon Instruments ********************
*
* File Name          : tfa98xx.c
* Authors            : Li Chen
* Version            : V 0.1
* Date               : April 2013
* Description        : Recon tfa98xx
*
********************************************************************************
*
******************************************************************************/
#include <linux/err.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>

#include "i2c_transfer.h"

//#define tfa98xx_DEBUG

#if defined tfa98xx_DEBUG
	#define tfa98xx_INFO(fmt, args...) printk("tfa98xx: " fmt, ##args)
#else
	#define tfa98xx_INFO(fmt, args...)
#endif
#define DRIVER_NAME             "tfa98xx"

#define tfa98xx_ADDRESS 0x34

struct tfa98xx_data
{
	struct i2c_client *client;
	struct mutex lock;
	struct miscdevice dev;
};

static int tfa98xx_open(struct inode* inode, struct file* file)
{
	//misc? driver require this for file_operations for some reasons
	return 0;
}

static ssize_t tfa98xx_write(struct file *filp, const char __user *buff, size_t count,loff_t *f_pos)
{
	struct tfa98xx_data *tfa98xx = container_of(filp->private_data, struct tfa98xx_data, dev);
	int ret;
	char *tmp;

	mutex_lock(&tfa98xx->lock);
	tmp = kmalloc(count, GFP_KERNEL);
	if(tmp == NULL){
		ret=-ENOMEM;
		goto write_done;
	}

	ret=copy_from_user(tmp, buff, count);

	if(ret==0)
		ret= mpu_i2c_write(tfa98xx->client->adapter,tfa98xx_ADDRESS, tmp,count) ? -EIO : count;
	else
		ret= -EFAULT;

	kfree(tmp);
write_done:
	mutex_unlock(&tfa98xx->lock);
	tfa98xx_INFO("%s:%d\n", __func__,ret);
	return ret;
}
static ssize_t tfa98xx_read(struct file *filp, char __user *buff, size_t count,loff_t *f_pos)
{
	struct tfa98xx_data *tfa98xx = container_of(filp->private_data, struct tfa98xx_data, dev);
	int ret;
	char *tmp;
	struct i2c_msg msg = {
			.addr = tfa98xx_ADDRESS,
			.flags = I2C_M_RD,
			.len = count,
	};
	mutex_lock(&tfa98xx->lock);
	tmp = kmalloc(count, GFP_KERNEL);
	if(tmp == NULL){
		ret=-ENOMEM;
		goto read_done;
	}

	msg.buf= tmp;
	ret=transfer_i2c_msg(tfa98xx->client->adapter, &msg, 1, I2C_CHIP_RETRIES);
	if(ret==0)
		ret= copy_to_user(buff, tmp, count) ? -EFAULT : count;
	else
		ret= -EIO;

	kfree(tmp);
read_done:
	mutex_unlock(&tfa98xx->lock);
	tfa98xx_INFO("%s:%d\n", __func__,ret);
	return ret;
}
/* File operations on proxmux device */
static const struct file_operations tfa98xx_fops = 
{
	.owner                  = THIS_MODULE,
	.read                   = tfa98xx_read,
	.write                  = tfa98xx_write,
	.open                   = tfa98xx_open,
};


static int tfa98xx_probe(struct i2c_client *client,
						   const struct i2c_device_id *id)
{
	int err = 0;
	struct tfa98xx_data *tfa98xx;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		dev_err(&client->dev, "client not i2c capable\n");
		return -ENODEV;
	}

	tfa98xx = kzalloc(sizeof(*tfa98xx), GFP_KERNEL);
	if (!tfa98xx) 
	{
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, tfa98xx);
	tfa98xx->client = client;
	mutex_init(&tfa98xx->lock);

	// Register misc device
	tfa98xx->dev.minor = MISC_DYNAMIC_MINOR;
	tfa98xx->dev.name = DRIVER_NAME;
	tfa98xx->dev.fops = &tfa98xx_fops;

	err = misc_register(&tfa98xx->dev);
	if (err < 0)
	{
		printk (KERN_ERR "Failed to register %s\n", tfa98xx->dev.name);
		goto misc_register_err;
	}
	return 0;

misc_register_err:
	kfree(tfa98xx);
	return err;
}

static int __devexit tfa98xx_remove(struct i2c_client *client)
{
	struct tfa98xx_data *tfa98xx = i2c_get_clientdata(client);
	printk("%s\n", __func__);
	misc_deregister(&(tfa98xx->dev));
	kfree(tfa98xx);
	return 0;
}

static const struct i2c_device_id tfa98xx_id[] = 
{
		{DRIVER_NAME, 0},
		{},
};

MODULE_DEVICE_TABLE(i2c, tfa98xx_id);

static struct i2c_driver tfa98xx_driver = 
{
	.driver =
	{
		.owner = THIS_MODULE,
		.name = DRIVER_NAME,
	},
	.probe     = tfa98xx_probe,
	.remove    = __devexit_p(tfa98xx_remove),
	.id_table  = tfa98xx_id,
};

static int __init tfa98xx_init(void)
{
	int res=i2c_add_driver(&tfa98xx_driver);
	pr_info("%s: Probe name %s\n", __func__, DRIVER_NAME);
	if(res)
		printk(KERN_ERR "%s failed\n", __func__);
	return res;
}

static void __exit tfa98xx_exit(void)
{
	pr_info("%s\n", __func__);
	i2c_del_driver(&tfa98xx_driver);
}
/* Module Entry Points */
module_init(tfa98xx_init);
module_exit(tfa98xx_exit);

MODULE_DESCRIPTION("Reconinstruemnts tfa98xx");
MODULE_AUTHOR("Li Chen");
MODULE_LICENSE("GPL");

