/******************** (C) COPYRIGHT 2011 Recon Instruments ********************
*
* File Name          : mfi.c
* Authors            : Li Chen
* Version            : V 0.1
* Date               : April 2013
* Description        : Recon Mfi
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

//#define MFI_DEBUG

#if defined MFI_DEBUG
	#define MFI_INFO(fmt, args...) printk("mfi: " fmt, ##args)
#else
	#define MFI_INFO(fmt, args...)
#endif
#define DRIVER_NAME             "mfi"

#define MFI_IOCTL_BASE 'g'
#define MFI_SET_READREG_ADDRESS  _IOW(MFI_IOCTL_BASE, 0,int)
#define MFI_SET_WRITEREG_ADDRESS  _IOW(MFI_IOCTL_BASE, 1,int)
#define MFI_ADDRESS 0x11

struct mfi_data
{
	struct i2c_client *client;
	struct mutex lock;
	struct miscdevice dev;
	u8 reg_address;
};

static int mfi_open(struct inode* inode, struct file* file)
{
	//misc? driver require this for file_operations for some reasons
	return 0;
}

static long mfi_ioctl(struct file *filp,unsigned int cmd, unsigned long arg)
{
	struct mfi_data *mfi = container_of(filp->private_data, struct mfi_data, dev);
	int ret=0;
	u8 reg_address;
	ret=mutex_lock_interruptible(&mfi->lock);
	if(ret)
	{
		printk("%s: mutex_lock_interruptible returned %d\n",__func__, ret);
		return ret;
	}
	switch (cmd)
	{
		case MFI_SET_READREG_ADDRESS:
			get_user(reg_address, (u8 __user *)arg);
			ret=mpu_i2c_write(mfi->client->adapter, MFI_ADDRESS, &reg_address, 1) ? -EFAULT : 0 ;
		break;
		case MFI_SET_WRITEREG_ADDRESS:
			ret=get_user(mfi->reg_address, (u8 __user *)arg);
			MFI_INFO("reg=0x%.2x\n", mfi->reg_address);
		break;
		default:
			ret=-EINVAL;
			break;
	}

	MFI_INFO("%s:%d\n", __func__,ret);
	mutex_unlock(&mfi->lock);
	return ret;
}
static ssize_t mfi_write(struct file *filp, const char __user *buff, size_t count,loff_t *f_pos)
{
	struct mfi_data *mfi = container_of(filp->private_data, struct mfi_data, dev);
	int ret;
	char *tmp;

	mutex_lock(&mfi->lock);
	tmp = kmalloc(count+1, GFP_KERNEL);
	if(tmp == NULL){
		ret=-ENOMEM;
		goto write_done;
	}

	tmp[0]=mfi->reg_address;
	ret=copy_from_user(tmp+1, buff, count);

	if(ret==0)
		ret= mpu_i2c_write(mfi->client->adapter,MFI_ADDRESS, tmp,count+1) ? -EIO : count;
	else
		ret= -EFAULT;

	kfree(tmp);
write_done:
	mutex_unlock(&mfi->lock);
	MFI_INFO("%s:%d\n", __func__,ret);
	return ret;
}
static ssize_t mfi_read(struct file *filp, char __user *buff, size_t count,loff_t *f_pos)
{
	struct mfi_data *mfi = container_of(filp->private_data, struct mfi_data, dev);
	int ret;
	char *tmp;
	struct i2c_msg msg = {
			.addr = MFI_ADDRESS,
			.flags = I2C_M_RD,
			.len = count,
	};
	mutex_lock(&mfi->lock);
	tmp = kmalloc(count, GFP_KERNEL);
	if(tmp == NULL){
		ret=-ENOMEM;
		goto read_done;
	}

	msg.buf= tmp;
	ret=transfer_i2c_msg(mfi->client->adapter, &msg, 1, I2C_CHIP_RETRIES);
	if(ret==0)
		ret= copy_to_user(buff, tmp, count) ? -EFAULT : count;
	else
		ret= -EIO;

	kfree(tmp);
read_done:
	mutex_unlock(&mfi->lock);
	MFI_INFO("%s:%d\n", __func__,ret);
	return ret;
}
/* File operations on proxmux device */
static const struct file_operations mfi_fops = 
{
	.owner                  = THIS_MODULE,
	.read                   = mfi_read,
	.write                  = mfi_write,
	.unlocked_ioctl         = mfi_ioctl,
	.open                   = mfi_open,
};


static int mfi_probe(struct i2c_client *client,
						   const struct i2c_device_id *id)
{
	int err = 0;
	struct mfi_data *mfi;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		dev_err(&client->dev, "client not i2c capable\n");
		return -ENODEV;
	}

	mfi = kzalloc(sizeof(*mfi), GFP_KERNEL);
	if (!mfi) 
	{
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, mfi);
	mfi->client = client;
	mutex_init(&mfi->lock);

	// Register misc device
	mfi->dev.minor = MISC_DYNAMIC_MINOR;
	mfi->dev.name = DRIVER_NAME;
	mfi->dev.fops = &mfi_fops;

	err = misc_register(&mfi->dev);
	if (err < 0)
	{
		printk (KERN_ERR "Failed to register %s\n", mfi->dev.name);
		goto misc_register_err;
	}
	return 0;

misc_register_err:
	kfree(mfi);
	return err;
}

static int __devexit mfi_remove(struct i2c_client *client)
{
	struct mfi_data *mfi = i2c_get_clientdata(client);
	printk("%s\n", __func__);
	misc_deregister(&(mfi->dev));
	kfree(mfi);
	return 0;
}

static const struct i2c_device_id mfi_id[] = 
{
		{DRIVER_NAME, 0},
		{},
};

MODULE_DEVICE_TABLE(i2c, mfi_id);

static struct i2c_driver mfi_driver = 
{
	.driver =
	{
		.owner = THIS_MODULE,
		.name = DRIVER_NAME,
	},
	.probe     = mfi_probe,
	.remove    = __devexit_p(mfi_remove),
	.id_table  = mfi_id,
};

static int __init mfi_init(void)
{
	int res=i2c_add_driver(&mfi_driver);
	pr_info("%s: Probe name %s\n", __func__, DRIVER_NAME);
	if(res)
		printk(KERN_ERR "%s failed\n", __func__);
	return res;
}

static void __exit mfi_exit(void)
{
	pr_info("%s\n", __func__);
	i2c_del_driver(&mfi_driver);
}
/* Module Entry Points */
module_init(mfi_init);
module_exit(mfi_exit);

MODULE_DESCRIPTION("Reconinstruemnts mfi");
MODULE_AUTHOR("Li Chen");
MODULE_LICENSE("GPL");

