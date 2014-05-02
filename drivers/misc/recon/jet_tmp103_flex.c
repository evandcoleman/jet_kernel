#include <linux/err.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include "jet_tmp103_flex.h"

#define DRIVER_NAME				"tmpflex_103"
#define I2C_CHIP_ADDRESS		0x70
#define	TMP103_TEMP_REG			0x00
#define	TMP103_CONF_REG			0x01

#define	TMP103_CONF_SD		0x00  //shut down
#define	TMP103_CONF_OS		0x01  //one shot
#define	TMP103_CONF_MODE_MASK		0x03

#if defined SENSOR_DEBUG_VERBOSE
struct tmp103_data *jet_temp_ptr;
#endif
int tmp103_hw_init(struct tmp103_data *jet_temp)
{
	u8 buffer[2];

	buffer[0]=TMP103_CONF_REG;
	buffer[1]=TMP103_CONF_SD;
	return(mpu_i2c_write(jet_temp->client->adapter,I2C_CHIP_ADDRESS, buffer, 2));
}

int tmp103_set_mode(struct tmp103_data *jet_temp,  unsigned char flag,
							unsigned char mode, unsigned int ms_delay)
{
	u8 buffer[2];

	buffer[0]=TMP103_CONF_REG;
	if(flag)
		buffer[1]=TMP103_CONF_OS;
	else
		buffer[1]=TMP103_CONF_SD;
	return(mpu_i2c_write(jet_temp->client->adapter,I2C_CHIP_ADDRESS, buffer, 2));
}

int tmp103_get_data(struct tmp103_data *jet_temp, u8 *sensor_data_ptr, unsigned int length)
{
	u8 buffer[2];
	int err;
	err=mpu_i2c_read(jet_temp->client->adapter,I2C_CHIP_ADDRESS, TMP103_CONF_REG, buffer,1);
	if(err)
		return err;
#ifdef SENSOR_DEBUG_VERBOSE
		SENSOR_INFO("TMP103_CONF_REG=%.2x\n",buffer[0]);
#endif
	if(buffer[0]&TMP103_CONF_MODE_MASK){
		printk(KERN_ERR "Conversion not ready\n");
		return 0;
	}
	err=mpu_i2c_read(jet_temp->client->adapter,I2C_CHIP_ADDRESS, TMP103_TEMP_REG, buffer,1);
	if(err)
		return err;
	//Swap byte
	*sensor_data_ptr=0;//Pad one byte
	*(sensor_data_ptr+1)=buffer[0];
	//Conversion: (*(short*)sensor_data_ptr)/256 to get degree celsius

	//Start single conversion. Need 30ms to get sensor conversion done
	buffer[0]=TMP103_CONF_REG;
	buffer[1]=TMP103_CONF_OS;
	err=mpu_i2c_write(jet_temp->client->adapter,I2C_CHIP_ADDRESS, buffer, 2);
	if(err)
		return err;
	return TMP_ONEEVENT_SIZE;
}
#if defined(CONFIG_JET_PROXMUX) || defined(CONFIG_JET_PROXMUX_MODULE)
/* @return
		>=0  Number of bytes filled in sliding buffer shared between sensor and MUX device
		<0   Error.
*/
int jet_tmp103_get_data
(
    struct tmp103_data *jet_temp,       // context, registered and passed back from MUX device
    u8*                 sensor_data_ptr,   // sliding buffer shared between us and MUX. Mux maintains offsets
    unsigned int        length,            // number of bytes left in sliding buffer; we fill what we can
    RI_SENSOR_HANDLE    handle             // Sensor ID
)
{
	unsigned int size;
	int ret;
	if (length < SENSOR_DATA_HEADER_SIZE+TMP_ONEEVENT_SIZE)
	{
		return 0;
	}
	size= length-SENSOR_DATA_HEADER_SIZE;
	ret=tmp103_get_data(jet_temp, SENSOR_PAYLOAD_PTR(sensor_data_ptr), size);
	if(ret<=0)
		return ret;

	size=ret;
	SENSOR_DATA_SIZE(sensor_data_ptr)=size;
	size+= SENSOR_DATA_HEADER_SIZE;//total size
	return size;
}
#endif

static int tmp103_probe(struct i2c_client *client,
						   const struct i2c_device_id *id)
{
	struct tmp103_data *tmp103;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		dev_err(&client->dev, "client not i2c capable\n");
		return -ENODEV;
	}

	tmp103 = kzalloc(sizeof(*tmp103), GFP_KERNEL);
	if (!tmp103) 
	{
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, tmp103);
	tmp103->client = client;
#if defined SENSOR_DEBUG_VERBOSE
	jet_temp_ptr=tmp103;
#endif
#if defined(CONFIG_JET_PROXMUX) || defined(CONFIG_JET_PROXMUX_MODULE)
	if(tmp103_hw_init(tmp103)==0)
	{
	//register temperature to proxmux
		risensor_register(RI_SENSOR_HANDLE_TEMPERATURE, DRIVER_NAME, tmp103,
				RI_SENSOR_MODE_CR, RI_SENSOR_MODE_CR,
				TMP_ONEEVENT_SIZE+ SENSOR_DATA_HEADER_SIZE,
		(PFNSENS_ACTIVATE)tmp103_set_mode,
		(PFNSENS_READ)jet_tmp103_get_data);
	}
	else {
		printk(KERN_ERR "couldn't init temperature\n");
		return -ENOSYS;
	}
#endif
	return 0;
}

static int __devexit tmp103_remove(struct i2c_client *client)
{
	struct tmp103_data *tmp103 = i2c_get_clientdata(client);
	printk("%s\n", __func__);
	kfree(tmp103);
	return 0;
}

static const struct i2c_device_id tmp103_id[] = 
{
		{DRIVER_NAME, 0},
		{},
};

MODULE_DEVICE_TABLE(i2c, tmp103_id);

static struct i2c_driver tmp103_driver = 
{
	.driver =
	{
		.owner = THIS_MODULE,
		.name = DRIVER_NAME,
	},
	.probe     = tmp103_probe,
	.remove    = __devexit_p(tmp103_remove),
	.id_table  = tmp103_id,
};


static int __init tmp103_init(void)
{
	int res=i2c_add_driver(&tmp103_driver);
	if(res)
		printk(KERN_ERR "%s failed\n", __func__);
	return res;
}

static void __exit tmp103_exit(void)
{
	pr_info("%s\n", __func__);
	i2c_del_driver(&tmp103_driver);
}

/* Module Entry Points */
module_init(tmp103_init);
module_exit(tmp103_exit);

MODULE_DESCRIPTION("Reconinstruemnts tmp103");
MODULE_AUTHOR("Li Chen");
MODULE_LICENSE("GPL");


