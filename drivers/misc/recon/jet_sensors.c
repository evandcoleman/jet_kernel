/******************** (C) COPYRIGHT 2011 Recon Instruments ********************
*
* File Name          : jet_sensors.c
* Authors            : Li Chen
* Version            : V 0.1
* Date               : January 2013
* Description        : Recon sensors
*
********************************************************************************
*
******************************************************************************/

#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include "jet_sensors.h"
#include "jet_lsm9ds0.h"
#include "jet_lps25h.h"

#define DRIVER_NAME             "jet_sensors"
// sensor names
const char* pszaccname  = "acc";
const char* pszgyroname = "gyro";
const char* pszmagname  = "mag";
const char* pszpressname  = "press";
#ifdef FREEFALL_API
const char* pszfreefallname  = "freefall";
#endif
/*!
 * \brief read data from the i2c sensors in the same bus, lock controled
 */
int jet_i2c_read(struct jet_sensors *jet_sensors, u8 chip_address,
					u8 reg_address, u8 *buf, int len)
{
	int err;
	mutex_lock(&jet_sensors->lock);

	err=mpu_i2c_read(jet_sensors->client->adapter, chip_address,
					reg_address, buf, len);

	mutex_unlock(&jet_sensors->lock);
	return err;
}
/*!
 * \brief write data to the i2c sensors in the same bus, lock controled
 */
int jet_i2c_write(struct jet_sensors *jet_sensors, u8 chip_address,
					u8 *buf, int len)
{
	int err;
	mutex_lock(&jet_sensors->lock);

	err=mpu_i2c_write(jet_sensors->client->adapter, chip_address,
					buf, len);

	mutex_unlock(&jet_sensors->lock);
	return err;
}

#if defined(CONFIG_JET_PROXMUX) || defined(CONFIG_JET_PROXMUX_MODULE)
/* Main sensors data read function. Return values:
      >=0  Number of bytes filled in sliding buffer shared between us and MUX device
      <0   Error. MUX must cope with negative value
*/
int jet_sensors_get_data
(
    struct jet_sensors* jet_sensors,       // context, registered and passed back from MUX device
    u8*                 sensor_data_ptr,   // sliding buffer shared between us and MUX. Mux maintains offsets
    unsigned int        length,            // number of bytes left in sliding buffer; we fill what we can
    RI_SENSOR_HANDLE    handle             // Sensor ID
)
{
	unsigned int size;
	int err;

   // Each packet must start with header: Sensor Handle + size. 
	if (length <= SENSOR_DATA_HEADER_SIZE)
	{
//		printk(KERN_DEBUG "+++ jet_sensors: sensor buffer size too short. Handle [0x%x[. Passed [%d], Minimum: [%d] +++\n", handle, length, SENSOR_DATA_HEADER_SIZE);
		return 0;   // we can't copy anything at this time, but not an error
	}

   // extract payload size; each sensor will determine if there is enough left, because data sizes vary
	size= length-SENSOR_DATA_HEADER_SIZE;
	
	switch (handle)
	{
		case RI_SENSOR_HANDLE_ACCELEROMETER:
			err=lsm9ds0_acc_get_data(jet_sensors, SENSOR_PAYLOAD_PTR(sensor_data_ptr), size);
		break;

		case RI_SENSOR_HANDLE_MAGNETOMETER:
			err=lsm9ds0_mag_get_data(jet_sensors, SENSOR_PAYLOAD_PTR(sensor_data_ptr), size);
		break;

		case RI_SENSOR_HANDLE_GYROSCOPE:
			err=lsm9ds0_gyro_get_data(jet_sensors, SENSOR_PAYLOAD_PTR(sensor_data_ptr), size);
		break;

		case RI_SENSOR_HANDLE_PRESSURE:
			err=lp225h_pressure_get_data(jet_sensors, SENSOR_PAYLOAD_PTR(sensor_data_ptr), size);
		break;

		default:
			printk(KERN_ERR "sensor wrong handle\n");
			return -EINVAL;   // this is an error. MUX must cope
	}

	SENSOR_TYPE(sensor_data_ptr)= handle;
	if (err<=0)
	{
		if (err < 0)  // 0 value is not an error; simply means we couldn't fit single payload
           printk(KERN_ERR "%s:err=%d\n", __func__,err);

		SENSOR_DATA_SIZE(sensor_data_ptr)=0;

		return err;
	}

   // here we have at least 1 meaningfull payload
	length= err;   // payload size
	SENSOR_DATA_SIZE(sensor_data_ptr)=length;
	length+= SENSOR_DATA_HEADER_SIZE;//total size

	return length;   // total number of bytes filled for MUX: header + payload
}

int jet_sensors_get_acc(struct jet_sensors *jet_sensors, u8 *sensor_data_ptr, unsigned int length)
{
	return jet_sensors_get_data(jet_sensors, sensor_data_ptr, length, RI_SENSOR_HANDLE_ACCELEROMETER);
}

int jet_sensors_get_mag(struct jet_sensors *jet_sensors, u8 *sensor_data_ptr, unsigned int length)
{
	return jet_sensors_get_data(jet_sensors, sensor_data_ptr, length, RI_SENSOR_HANDLE_MAGNETOMETER);
}

int jet_sensors_get_gyro(struct jet_sensors *jet_sensors, u8 *sensor_data_ptr, unsigned int length)
{
	return jet_sensors_get_data(jet_sensors, sensor_data_ptr, length, RI_SENSOR_HANDLE_GYROSCOPE);
}

int jet_sensors_get_pressure(struct jet_sensors *jet_sensors, u8 *sensor_data_ptr, unsigned int length)
{
	return jet_sensors_get_data(jet_sensors, sensor_data_ptr, length, RI_SENSOR_HANDLE_PRESSURE);
}
#endif

static struct regulator *sensor_switch_reg;

static ssize_t show_sensor_power(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if(regulator_is_enabled(sensor_switch_reg))
		return sprintf(buf, "on\n");
	else
		return sprintf(buf, "off\n");
}

static ssize_t set_sensor_power(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	int status = count;
	//struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	/*
	 * Revisit: add limit range checking
	 */
	if (strict_strtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if(val)
	{
		SENSOR_INFO("Power On\n");
		regulator_enable(sensor_switch_reg);
	}
	else
	{
		SENSOR_INFO("Power Off\n");
		regulator_disable(sensor_switch_reg);
	}

	return status;
}

static DEVICE_ATTR(sensor_power, S_IWUSR|S_IRUGO, show_sensor_power, set_sensor_power);
static struct attribute* jet_sensors_attributes[] = 
{
	&dev_attr_sensor_power.attr,
	NULL
};
static const struct attribute_group jet_sensors_attr_group = 
{
	.attrs = jet_sensors_attributes,
}; 

static int jet_sensors_open(struct inode* inode, struct file* file)
{
	//misc? driver require this for file_operations for some reasons
	return 0;
}
#if defined SENSOR_DEBUG_VERBOSE
//The debug_write function is only used for developer to play with the i2c chips to get general feelings
#define BUFFER_LENGTH		20
#define BUS_NUM_INDEX		2
#define CHIP_ADDRESS_INDEX	4
#define REG_ADDRESS_INDEX	7
#define NUMBER_INDEX		10
#define VALUE_INDEX			10
#define FLAG_INDEX			2
#define MODE_INDEX			4
#define DELAY_INDEX			6
#define HANDLE_INDEX		2
#define DATA_SIZE			192+6

#ifdef CONFIG_JET_TMP103_FLEX
#include "jet_tmp103_flex.h"
#endif
static ssize_t jet_sensors_debug_write(struct file *filp, const char __user *buff, size_t count, loff_t *f_pos)
{
	ssize_t status = count;
	struct i2c_adapter *adapter;
	u8 cmd[BUFFER_LENGTH],i,j;
	u8 i2c_bus_num, chip_address, buf[2];
	int err, handle;
	int len=0;
	u8 flag,mode;
	unsigned int delay;
	u8 sensorevent[DATA_SIZE];
	short *sensor_ptr;
	unsigned short *pressure_ptr;
	struct jet_sensors *jet_sensors = container_of(filp->private_data, struct jet_sensors, dev);

	if (count>BUFFER_LENGTH)
		return -ENOSPC;
		

	if (copy_from_user(cmd, buff, count)) {
		status = -EFAULT;
		goto write_done;
	}
	//remove spaces in valid data
	for(i=0;i<count;i++){
		if(cmd[i]==' ')
			cmd[i]=0;
	}

	switch (cmd[0])
	{
		case 'w':
		{
			//w x xx xx xx
			//w i2c_bus_num chip_address reg_address value
			i2c_bus_num=(u8)simple_strtol(&cmd[BUS_NUM_INDEX], NULL, 16) ;
			chip_address=(u8)simple_strtol(&cmd[CHIP_ADDRESS_INDEX], NULL, 16) ;
			buf[0]=(u8)simple_strtol(&cmd[REG_ADDRESS_INDEX], NULL, 16) ;
			buf[1]=(u8)simple_strtol(&cmd[VALUE_INDEX], NULL, 16) ;
			
			SENSOR_INFO("i2c Bus %d, Chip 0x%.2x, Reg 0x%.2x, Write 0x%.2x\n", 
						i2c_bus_num, chip_address, buf[0], buf[1]);
			
			adapter=i2c_get_adapter(i2c_bus_num);
			if(mpu_i2c_write(adapter,chip_address, buf, 2))
				printk("i2c write wrong\n");
			break;
		}
		case 'r':
		case 'd':
		{
			//r x xx xx xx
			//r i2c_bus_num chip_address reg_address 
			i2c_bus_num=(u8)simple_strtol(&cmd[BUS_NUM_INDEX], NULL, 16) ;
			chip_address=(u8)simple_strtol(&cmd[CHIP_ADDRESS_INDEX], NULL, 16) ;
			buf[0]=(u8)simple_strtol(&cmd[REG_ADDRESS_INDEX], NULL, 16) ;
			if(count>10)
				buf[1]=(u8)simple_strtol(&cmd[NUMBER_INDEX], NULL, 16) ;
			else
				buf[1]=1;
			if(buf[1]>DATA_SIZE)
				return -ENOSPC;

			adapter=i2c_get_adapter(i2c_bus_num);

			if(cmd[0]=='d'){//for Mfi chip-------
				err=mpu_i2c_read_slow(adapter,chip_address, buf[0], sensorevent,buf[1]);
			}
			else
				err=mpu_i2c_read(adapter,chip_address, buf[0], sensorevent,buf[1]);
			if(err==0){
				for(i=0;i<buf[1];i++){
					SENSOR_INFO("0x%.2x,", sensorevent[i]);
				}
				
				SENSOR_INFO("i2c Bus %d, Chip 0x%.2x, Reg 0x%.2x, Read number 0x%.2x\n", 
						i2c_bus_num, chip_address, buf[0], buf[1]);

			}
			break;
		}
		case 'g':
		case 'a':
		case 'm':
		case 'p':
		case 'f':
		case 't':
		{
			//g x x xxxx
			//g flag mode delay
			if(count>10)
			{
				err=0;
				flag=(u8)simple_strtol(&cmd[FLAG_INDEX], NULL, 10);
				mode=(u8)simple_strtol(&cmd[MODE_INDEX], NULL, 10);
				delay=simple_strtol(&cmd[DELAY_INDEX], NULL, 10);
				SENSOR_INFO("flag=%d, mode=%d, delay=%dms\n", 
						flag,mode,delay);
				if(cmd[0]=='g')
					err=lsm9ds0_gyro_set_mode(jet_sensors, flag, mode, delay);
				else if(cmd[0]=='a')
					err=lsm9ds0_acc_set_mode(jet_sensors, flag, mode, delay);
				else if(cmd[0]=='m')
					err=lsm9ds0_mag_set_mode(jet_sensors, flag, mode, delay);
				else if(cmd[0]=='p')
					err=lp225h_pressure_set_mode(jet_sensors, flag, mode, delay);
#ifdef CONFIG_JET_TMP103_FLEX
				else if(cmd[0]=='t'){
					if(jet_temp_ptr!=NULL)
						err=tmp103_set_mode(jet_temp_ptr, flag, mode, delay);
					else
						err=-EIO;
				}
#endif
#ifdef CONFIG_JET_SENSORS_FREE_FALL
				else if(cmd[0]=='f')
					err=lsm9ds0_freefall_set_mode(jet_sensors, flag);
#endif
				if(err)
					printk(KERN_ERR "cannot change\n, err=%d", err);
			}
			else if(cmd[1]=='g')
			{
				if(cmd[0]=='g')
					len=lsm9ds0_gyro_get_data(jet_sensors,sensorevent, 192);
				else if(cmd[0]=='a')
					len=lsm9ds0_acc_get_data(jet_sensors,sensorevent, 192);
				else if(cmd[0]=='m')
					len=lsm9ds0_mag_get_data(jet_sensors,sensorevent, 192);
				else if(cmd[0]=='p')
					len=lp225h_pressure_get_data(jet_sensors,sensorevent, LPS_DATA_LENGTH_MAX);
#ifdef CONFIG_JET_TMP103_FLEX
				else if(cmd[0]=='t'){
					if(jet_temp_ptr!=NULL)
						len=tmp103_get_data(jet_temp_ptr, sensorevent, TMP_ONEEVENT_SIZE);
					else
						len=0;
				}
#endif
				SENSOR_INFO("len=%d\n",len);
				if(len>0)
				{
					if(cmd[0]=='p')
					{
						len=len/LPS_ONEEVENT_SIZE;
						sensor_ptr=(short*)sensorevent;
						for(i=0; i< len; i++){
							for(j=0;j<LPS_ONEEVENT_SIZE;j++)
								printk("0x%.2x,", *(sensorevent+i*LPS_ONEEVENT_SIZE+j));
							printk("\n");
							pressure_ptr=(unsigned short*)(sensorevent+i*LPS_ONEEVENT_SIZE+1);
							SENSOR_INFO("P=%hdmbar\n", ((*pressure_ptr)>>4));//just get integer part here
						}
					}
#ifdef CONFIG_JET_TMP103_FLEX
					else if(cmd[0]=='t'){
						printk("0x%.2x,0x%.2x\n", *sensorevent, *(sensorevent+1));
					}
#endif
					else
					{
						len=len/LSM_DATA_ONEEVENT_SIZE;
						sensor_ptr=(short*)sensorevent;
						for(i=0; i< len; i++)
							SENSOR_INFO("x=%hd,y=%hd,z=%hd\n",*(sensor_ptr+i*3),*(sensor_ptr+i*3+1),*(sensor_ptr+i*3+2));
					}
				}
			}
			break;
		}
		case 's':  //sensor test
		{
			//s xxxx
			handle=(unsigned int)simple_strtol(&cmd[HANDLE_INDEX], NULL, 16);
			SENSOR_INFO("hanlde=0x%X,\n", handle);

			len=jet_sensors_get_data(jet_sensors, sensorevent, sizeof(sensorevent), handle);
			SENSOR_INFO("len=%d, data size=%d, handle=%X\n",len, SENSOR_DATA_SIZE(sensorevent), SENSOR_TYPE(sensorevent));
			len= len-SENSOR_DATA_HEADER_SIZE;
			if(len>0)
			{
				len=len/6;
				sensor_ptr=(short*)(sensorevent+SENSOR_DATA_INDEX);
				for(i=0; i< len; i++)
					SENSOR_INFO("x=%hd,y=%hd,z=%hd\n",*(sensor_ptr+i*3),*(sensor_ptr+i*3+1),*(sensor_ptr+i*3+2));
			}
			break;
		}
		default:
			break;
	}
	
write_done:
	*f_pos += count;
	return status;
}
#endif
/* File operations on proxmux device */
static const struct file_operations jet_sensors_fops = 
{
	.owner                  = THIS_MODULE,
	.open                   = jet_sensors_open,
#if defined SENSOR_DEBUG_VERBOSE
	.write                  = jet_sensors_debug_write,
#endif
};

#ifdef CONFIG_JET_SENSORS_FREE_FALL
static irqreturn_t freefall_irq_handler(int irq, void *dev_id)
{
	struct jet_sensors *jet_sensors=dev_id;
	schedule_work(&jet_sensors->freefall_irq_work);
	return IRQ_HANDLED;
}
#endif
#ifdef CONFIG_JET_SENSORS_TAP_TAP
static irqreturn_t tap_tap_irq_handler(int irq, void *dev_id)
{
	struct jet_sensors *jet_sensors = dev_id;
	printk(KERN_INFO "[%s:%u]\n",__FUNCTION__,__LINE__);
	schedule_work(&jet_sensors->tap_tap_irq_work);
	return IRQ_HANDLED;
}
#endif

static int jet_sensors_probe(struct i2c_client *client,
						   const struct i2c_device_id *id)
{
	int err = 0;
	struct jet_sensors *jet_sensors;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		dev_err(&client->dev, "client not i2c capable\n");
		return -ENODEV;
	}

	jet_sensors = kzalloc(sizeof(*jet_sensors), GFP_KERNEL);
	if (!jet_sensors)
	{
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
		return -ENOMEM;
	}
	//memset(jet_sensors, 0, sizeof(*jet_sensors));
	i2c_set_clientdata(client, jet_sensors);
	jet_sensors->client = client;
	mutex_init(&jet_sensors->lock);

	// Register misc device
	jet_sensors->dev.minor = MISC_DYNAMIC_MINOR;
	jet_sensors->dev.name = DRIVER_NAME;
	jet_sensors->dev.fops = &jet_sensors_fops;

	err = misc_register(&jet_sensors->dev);
	if (err < 0)
	{
		printk (KERN_ERR "Failed to register %s\n", jet_sensors->dev.name);
		goto misc_register_err;
	}
	
	err = sysfs_create_group(&(jet_sensors->dev.this_device->kobj), &jet_sensors_attr_group);
	if (err) 
	{
		dev_err(jet_sensors->dev.this_device, "unable to create attribute\n");
		goto sysfs_create_err;
	}
	
	sensor_switch_reg=regulator_get(jet_sensors->dev.this_device,"switch");
	
	if (IS_ERR(sensor_switch_reg)) {
		err = PTR_ERR(sensor_switch_reg);
		printk(KERN_ERR "couldn't get Sensor Switch regulator %d\n", err);
		goto regulator_err;
	}
	//enable power supply
	regulator_enable(sensor_switch_reg);
	
	if (lsm9ds0_gyro_hw_init(jet_sensors)==0)
	{
	   // register gyro to proxmux
		risensor_register(RI_SENSOR_HANDLE_GYROSCOPE, pszgyroname, jet_sensors,
						(RI_SENSOR_MODE_CR|RI_SENSOR_MODE_FIFO), RI_SENSOR_MODE_FIFO,
						LSM_FIFO_LENGTH_MAX + SENSOR_DATA_HEADER_SIZE,
                  (PFNSENS_ACTIVATE)lsm9ds0_gyro_set_mode,
                  (PFNSENS_READ)jet_sensors_get_gyro);
	}
	else
		printk(KERN_ERR "couldn't init gyro\n");

	if (lsm9ds0_acc_hw_init(jet_sensors)==0)
	{
	   // register acc to proxmux
		risensor_register(RI_SENSOR_HANDLE_ACCELEROMETER, pszaccname, jet_sensors,
				(RI_SENSOR_MODE_CR|RI_SENSOR_MODE_FIFO), RI_SENSOR_MODE_FIFO,
				LSM_FIFO_LENGTH_MAX + SENSOR_DATA_HEADER_SIZE,
            (PFNSENS_ACTIVATE)lsm9ds0_acc_set_mode,
            (PFNSENS_READ)jet_sensors_get_acc);
	}
	else
		printk(KERN_ERR "couldn't init acc\n");

	if (lsm9ds0_mag_hw_init(jet_sensors)==0)
	{
	   // register amag to proxmux
		risensor_register(RI_SENSOR_HANDLE_MAGNETOMETER, pszmagname, jet_sensors,
					RI_SENSOR_MODE_CR, RI_SENSOR_MODE_CR,
					LSM_DATA_ONEEVENT_SIZE + SENSOR_DATA_HEADER_SIZE,
              (PFNSENS_ACTIVATE)lsm9ds0_mag_set_mode,
              (PFNSENS_READ)jet_sensors_get_mag);
	}
	else
		printk(KERN_ERR "couldn't init mag\n");
#ifdef CONFIG_JET_V2
	if(lp225h_pressure_hw_init(jet_sensors)==0)
	{
	//register pressure to proxmux
		risensor_register(RI_SENSOR_HANDLE_PRESSURE, pszpressname, jet_sensors,
				(RI_SENSOR_MODE_CR|RI_SENSOR_MODE_FIFO), RI_SENSOR_MODE_FIFO,
				 LPS_DATA_LENGTH_MAX+ SENSOR_DATA_HEADER_SIZE,
		(PFNSENS_ACTIVATE)lp225h_pressure_set_mode,
		(PFNSENS_READ)jet_sensors_get_pressure);
	}
	else
		printk(KERN_ERR "couldn't init pressure\n");
#endif

#if defined (CONFIG_JET_SENSORS_FREE_FALL)|| defined(CONFIG_JET_SENSORS_TAP_TAP)
	if (client->dev.platform_data == NULL)
	{
		dev_err(&client->dev, "no platform data\n");
		return 0;
	}
	jet_sensors->pdata = kmalloc(sizeof(*jet_sensors->pdata), GFP_KERNEL);
	if (jet_sensors->pdata == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate memory for pdata: %d\n", err);
		return 0;
	}
	memcpy(jet_sensors->pdata, client->dev.platform_data, sizeof(*jet_sensors->pdata));
#endif

#ifdef CONFIG_JET_SENSORS_FREE_FALL
	if(lsm9ds0_freefall_hw_init(jet_sensors)==0)
	{
		INIT_WORK(&jet_sensors->freefall_irq_work, freefall_irq_work_func);
		err = request_irq(jet_sensors->pdata->irq_ff, freefall_irq_handler, IRQ_TYPE_EDGE_RISING, "freefall_irq", jet_sensors);
		if (err<0)
		{
			printk(KERN_ERR "[%s:%u] freefall irq fail to register: %d\n",__FUNCTION__,__LINE__,err);
		}
#ifdef FREEFALL_API
		else
      {
			// register FreeFall sensor with MUX device
         RI_DATA_SIZE payload = sizeof(RI_SENSOR_HANDLE) + sizeof(RI_DATA_SIZE) + sizeof(struct timespec);
			RI_SENSOR_STATUS stat = risensor_register_irq(
              RI_SENSOR_HANDLE_FREEFALL,                    // sensor handle
              pszfreefallname,                              // humand readable name, owned by us
              jet_sensors,                                  // context, passed back during activate calls
				  payload,                                      // event data: Header + Timestamp
				  (PFNSENS_ACTIVATE)lsm9ds0_freefall_set_mode   // activation callback
           );

         if (stat != 0) printk(KERN_ERR "Mux Device failed to register FreeFall Interrupt. Jump feature will not be available");

		}
#endif
	}
	else
		printk(KERN_ERR "[%s:%u] couldn't init irq\n",__FUNCTION__,__LINE__);
#endif
#ifdef CONFIG_JET_SENSORS_TAP_TAP
	if(lsm9ds0_tap_tap_hw_init(jet_sensors)==0)
	{
		printk(KERN_INFO "[%s:%u]\n",__FUNCTION__,__LINE__);
		INIT_WORK(&jet_sensors->tap_tap_irq_work, tap_tap_irq_work_func);
		err = request_irq(jet_sensors->pdata->irq_tt, tap_tap_irq_handler, IRQ_TYPE_EDGE_RISING, "tap_tap_irq", jet_sensors);
		if (err<0)
		{
			printk("[%s:%u] tap tap irq fail to register: %d\n",__FUNCTION__,__LINE__, err);
		}
	}
	else
		printk(KERN_ERR "[%s:%u] couldn't init irq\n",__FUNCTION__,__LINE__);

#endif
	return 0;

regulator_err:
	sysfs_remove_group(&(jet_sensors->dev.this_device->kobj), &jet_sensors_attr_group);
sysfs_create_err:
	misc_deregister(&(jet_sensors->dev));
misc_register_err:
	kfree(jet_sensors);
	return err;
}

static int __devexit jet_sensors_remove(struct i2c_client *client)
{
	struct jet_sensors *jet_sensors = i2c_get_clientdata(client);
	printk("%s\n", __func__);

#ifdef CONFIG_JET_SENSORS_FREE_FALL
	free_irq(jet_sensors->pdata->irq_ff, jet_sensors);
#endif

#ifdef CONFIG_JET_SENSORS_TAP_TAP
	input_unregister_device(jet_sensors->tap_tap_input_dev);
	input_free_device(jet_sensors->tap_tap_input_dev);
	free_irq(jet_sensors->pdata->irq_tt, jet_sensors);
#endif

#if defined (CONFIG_JET_SENSORS_FREE_FALL)|| defined(CONFIG_JET_SENSORS_TAP_TAP)
	kfree(jet_sensors->pdata);
#endif

	regulator_put(sensor_switch_reg);
	sysfs_remove_group(&(jet_sensors->dev.this_device->kobj), &jet_sensors_attr_group);
	misc_deregister(&(jet_sensors->dev));
	kfree(jet_sensors);
	return 0;
}

static const struct i2c_device_id jet_sensors_id[] = 
{
		{DRIVER_NAME, 0},
		{},
};

MODULE_DEVICE_TABLE(i2c, jet_sensors_id);

static struct i2c_driver jet_sensors_driver = 
{
	.driver =
	{
		.owner = THIS_MODULE,
		.name = DRIVER_NAME,
	},
	.probe     = jet_sensors_probe,
	.remove    = __devexit_p(jet_sensors_remove),
	.id_table  = jet_sensors_id,
};

static int __init jet_sensors_init(void)
{
	int res=i2c_add_driver(&jet_sensors_driver);
	pr_info("%s: Probe name %s\n", __func__, DRIVER_NAME);
	if(res)
		printk(KERN_ERR "%s failed\n", __func__);
	return res;
}

static void __exit jet_sensors_exit(void)
{
	pr_info("%s\n", __func__);
	i2c_del_driver(&jet_sensors_driver);
}
/* Module Entry Points */
module_init(jet_sensors_init);
module_exit(jet_sensors_exit);

MODULE_DESCRIPTION("Reconinstruemnts sensor");
MODULE_AUTHOR("Li Chen");
MODULE_LICENSE("GPL");

