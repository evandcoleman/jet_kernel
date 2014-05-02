/******************** (C) COPYRIGHT 2011 Recon Instruments ********************
*
* File Name          : apds9900.c
* Authors            : Li Chen
* Version            : V 0.1
* Date               : April 2013
* Description        : Recon apds
*
********************************************************************************
*
******************************************************************************/
#include <linux/err.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/poll.h> 
#include <linux/interrupt.h>

#include "jet_sensors.h"
#include "i2c_transfer.h"
#include "risensors_def.h"
#include "apds9900.h"

#define APDS_IOCTL_BASE 'a'
#define APDS_READ_BYTE  _IOWR(APDS_IOCTL_BASE, 0,int)
#define APDS_READ_2BYTES  _IOWR(APDS_IOCTL_BASE, 1,int)

#define APDS_CLEAR_IT  _IOW(APDS_IOCTL_BASE, 0,int)

#define APDS_FAKE_IT  _IO(APDS_IOCTL_BASE, 0)

#define COMMAND_MODE 0X80
#define AUTO_INCREMENT 0x20
#define SPECIAL_FUNCTION 0x60

//#define CONTINUATION_READ
#define DRIVER_NAME             "apds9900"
#define APDS_ADDRESS 0x39

#define APDS_ONEEVENT_SIZE 4

// sensor names
const char* snsrnamelight  = "light";
const char* snsrnameproximity  = "proximity";

#define NO_IT_EVENT 0
#define IT_EVENT 1
struct apds_data
{
	struct i2c_client *client;
	struct mutex lock;
	struct miscdevice dev;
	wait_queue_head_t signal;
	atomic_t interrupt_state;
#ifdef CONTINUATION_READ
	u8 reg_address;
#endif
};

struct apds_coefs
{
	int coef_df;	//Device Factor
	int coef_ga;	//Glass (or Lens) Attenuation Factor
	int coef_b;		//Coeficient-B
	int coef_c;		//Coeficient-C
	int coef_d;		//Coeficient-D
};

/***************
* Initial Configuration of the APDS9900
* Sets all relevant registers to the default desired operation
****************/
static int apds9900_base_config(struct apds_data *apds_data)
{
	u8 buf[13];
	int err;

	printk(KERN_INFO "[%s:%u]\n",__FUNCTION__,__LINE__);

	/********************************************************************************/
	/* APDS ENABLE REGISTER 			0x00										*/
	/* [7-6] = 00b		Reserved, must be 00										*/
	/* [5-0] = 000000b	Clear all - DISABLE											*/
	/********************************************************************************/
	buf[0]=(COMMAND_MODE|AUTO_INCREMENT|APDS_ENABLE_REG);
	buf[1] = APDS_DISABLE; //Disable and power down

  	/********************************************************************************/
	/* APDS ATIME REGISTER 				0x01										*/
	/* [7:0] = 0xFF		ALS Integration Time	2.72ms	(minimum)					*/
	/********************************************************************************/
	buf[2] = APDS_ATIME_MIN;

	/********************************************************************************/
	/* APDS PTIME REGISTER 				0x02										*/
	/* [7:0] = 0xFF		Proximity Integration Time	2.72ms	(minimum)				*/
	/********************************************************************************/
	buf[3] = APDS_PTIME;

	/********************************************************************************/
	/* APDS WTIME REGISTER 				0x03										*/
	/* [7:0] = 0xFF		Wait Time	2.72ms	(minimum)								*/
	/********************************************************************************/
	buf[4] = APDS_WTIME_200MS;

	err = mpu_i2c_write(apds_data->client->adapter, APDS_ADDRESS, buf, 5);
	if (err < 0)
		return err;

	/*Interupt Config Initialization*/

	/********************************************************************************/
	/*  APDS ALS Interrupt Registers												*/
	/*	APDS_AILTL_REG					0x04		   								*/
	/*	APDS_AILTH_REG					0x05										*/
	/* 	APDS_AIHTL_REG					0x06		 								*/
	/*	APDS_AIHTH_REG					0x07		  								*/
	/********************************************************************************/
	buf[0]=(COMMAND_MODE|AUTO_INCREMENT|APDS_AILTL_REG);

	buf[1] = APDS_AILTL_INIT;
	buf[2] = APDS_AILTH_INIT;
	buf[3] = APDS_AIHTL_INIT;
	buf[4] = APDS_AIHTH_INIT;

	/********************************************************************************/
	/*  APDS Proximity Interrupt Registers											*/
	/*	APDS_PILTL_REG					0x08		   								*/
	/*	APDS_PILTH_REG					0x09										*/
	/* 	APDS_PIHTL_REG					0x0A		 								*/
	/*	APDS_PIHTH_REG					0x0B		  								*/
	/*	APDS_PERS_REG					0x0C		  								*/
	/********************************************************************************/
	//buf[0]=(COMMAND_MODE|AUTO_INCREMENT|APDS_PILTL_REG);
	buf[5] = APDS_PILTL_INIT;
	buf[6] = APDS_PILTH_INIT;
	buf[7] = APDS_PIHTL_INIT;
	buf[8] = APDS_PIHTH_INIT;
	buf[9] = APDS_PERS_INIT;

	/********************************************************************************/
	/* APDS CONFIG REGISTER 			0x0D										*/
	/* [1] = 0	Wait Config WLONG = 0												*/
	/* [1] = 1	Wait Config WLONG = 1 12x Wait cycle								*/
	/********************************************************************************/
	buf[10] = APDS_WSHORT;

	/********************************************************************************/
	/* APDS PPCOUNT REGISTER 			0x0E										*/
	/* [7:0] = 0x01		Proximity Pulse count		(minimum)						*/
	/********************************************************************************/
	buf[11] = APDS_PPCOUNT_MIN;

	/********************************************************************************/
	/* APDS CONTROL REGISTER 			0x0F										*/
	/* [7:6] = 0x01		LED drive		50 mA										*/
	/* [5:4] = 0x20		Proximity Diode Select		RESERVED						*/
	/* [3:2] = 0x0		Proximity Gain Control		RESERVED						*/
	/* [1:0] = 0x0		ALS Gain Control			RESERVED						*/
	/********************************************************************************/
	buf[12] = (APDS_PDRIVE_50 | APDS_PDIODE | APDS_PGAIN | APDS_AGAIN_1X);

	err = mpu_i2c_write(apds_data->client->adapter, APDS_ADDRESS, buf, 13);
	if (err < 0)
		return err;

	/********************************************************************************/
	/* APDS ENABLE REGISTER 		0x00											*/
	/* [7-6] = 00b		Reserved, must be 00										*/
	/* [5-0] = 101111b	Enable all but ALS interupts								*/
	/********************************************************************************/
	buf[0]=COMMAND_MODE | APDS_ENABLE_REG;
	buf[1]=APDS_PIEN_OFF | APDS_AIEN_OFF | APDS_WEN_ON | APDS_PEN_OFF | APDS_AEN_ON | APDS_PON; //both intrrupts OFF, only ALS ON
	err = mpu_i2c_write(apds_data->client->adapter, APDS_ADDRESS, buf, 2);
	if (err < 0)
		return err;

	mdelay(12); //Wait for 12 ms

	return 0;
}

/************
 * Configuration of the ALS portion of the APDS9900
************/
int apds9900_als_config(struct apds_data *apds_data)
{
	int err;
	u8 buf[2];

	printk(KERN_INFO "[%s:%u]\n",__FUNCTION__,__LINE__);

	/********************************************************************************/
	/* APDS ATIME REGISTER 		0x01												*/
	/* [7:0] = 0xFF		Wait Time	2.72ms	(minimum)								*/
	/********************************************************************************/

	buf[0]=COMMAND_MODE | APDS_ATIME_REG;
	buf[1]=APDS_ATIME_MAX_COUNT;
	err = mpu_i2c_write(apds_data->client->adapter, APDS_ADDRESS, buf, 2);
	if (err < 0)
		return err;

	err = mpu_i2c_read(apds_data->client->adapter, APDS_ADDRESS, (COMMAND_MODE|APDS_ATIME_REG), &buf[1], 1);
	printk(KERN_INFO "[%s:%u] ATIME=0x%X err=%d\n",__FUNCTION__,__LINE__,buf[1], err);


	/********************************************************************************/
	/* APDS CONTROL REGISTER 	0x0F												*/
	/* [7:6] = 0x00		LED drive													*/
	/* [5:4] = 0x20		Proximity Diode Select		RESERVED						*/
	/* [3:2] = 0x0		Proximity Gain Control		RESERVED						*/
	/* [1:0] = 0x0		ALS Gain Control											*/
	/********************************************************************************/

	//Read register first so other configurations are not overwritten
	err = mpu_i2c_read(apds_data->client->adapter, APDS_ADDRESS, (COMMAND_MODE|APDS_CONTROL_REG), &buf[1], 1);
	if (err < 0){
		printk(KERN_ERR "[%s:%u] CONTROL Reg Read Error=%d \n",__FUNCTION__,__LINE__,err);
		return err;
	}

	buf[0]=COMMAND_MODE | APDS_CONTROL_REG;
	buf[1]=((buf[1] & 0xFC) | APDS_AGAIN_1X);
	err = mpu_i2c_write(apds_data->client->adapter, APDS_ADDRESS, buf, 2);
	if (err < 0){
		printk(KERN_ERR "[%s:%u] CONTROL Reg Write Error=%d \n",__FUNCTION__,__LINE__,err);
		return err;
	}

	return 0;
}

/***********************
 * Enable ALS operation
***********************/
int apds9900_als_enable(struct apds_data *apds_data)
{
	int err;
	u8 buf[2];

	printk(KERN_INFO "[%s:%u]\n",__FUNCTION__,__LINE__);

	/********************************************************************************/
	/* APDS ENABLE REGISTER 	0x00												*/
	/* [7-6] = 00b		Reserved, must be 00										*/
	/* [5-0] = 																		*/
	/********************************************************************************/

	//read register first so we don't overwrite other configurations
	err = mpu_i2c_read(apds_data->client->adapter, APDS_ADDRESS, (COMMAND_MODE|APDS_ENABLE_REG), &buf[1], 1);
	if (err < 0){
		printk(KERN_ERR "[%s:%u] ENABLE Reg Read Error=%d \n",__FUNCTION__,__LINE__,err);
		return err;
	}

	buf[0]=COMMAND_MODE | APDS_ENABLE_REG;
	buf[1]=((buf[1] & 0x2A) | APDS_PIEN_OFF | APDS_AIEN_OFF | APDS_WEN_ON | APDS_PEN_OFF | APDS_AEN_ON | APDS_PON);
	err = mpu_i2c_write(apds_data->client->adapter, APDS_ADDRESS, buf, 2);
	if (err < 0){
		printk(KERN_ERR "[%s:%u] ENABLE Reg Write Error=%d \n",__FUNCTION__,__LINE__,err);
		return err;
	}

	return 0;
}

/**********************
 * Disable ALS operation
**********************/
int apds9900_als_disable(struct apds_data *apds_data)
{
	int err;
	u8 buf[2];

	printk(KERN_INFO "[%s:%u]\n",__FUNCTION__,__LINE__);

	/********************************************************************************/
	/* APDS ENABLE REGISTER 0x00													*/
	/* [7-6] = 00b		Reserved, must be 00										*/
	/* [5-0] =																		*/
	/********************************************************************************/

	//read register first so we don't overwrite other configurations
	err = mpu_i2c_read(apds_data->client->adapter, APDS_ADDRESS, (COMMAND_MODE|APDS_ENABLE_REG), &buf[1], 1);
	if (err < 0){
		printk(KERN_ERR "[%s:%u] ENABLE Reg Read Error=%d \n",__FUNCTION__,__LINE__,err);
		return err;
	}

	buf[0]=COMMAND_MODE | APDS_ENABLE_REG;
	buf[1]=((buf[1] & 0x2A) | APDS_AIEN_OFF | APDS_AEN_OFF | APDS_PON); // ALS Disable, ALS Interrupt Disable
	err = mpu_i2c_write(apds_data->client->adapter, APDS_ADDRESS, buf, 2);
	if (err < 0){
		printk(KERN_ERR "[%s:%u] ENABLE Reg Write Error=%d \n",__FUNCTION__,__LINE__,err);
		return err;
	}

	return 0;
}


/**********************
 * Get Coeficients
**********************/
void apds9900_get_coeficients(struct apds_coefs *cc)
{
	// open air coeficients (default)
	cc->coef_df = APDS_OPENAIR_COEF_DF;
	cc->coef_ga = APDS_OPENAIR_COEF_GA;
	cc->coef_b = APDS_OPENAIR_COEF_B;
	cc->coef_c = APDS_OPENAIR_COEF_C;
	cc->coef_d = APDS_OPENAIR_COEF_D;
#if 0
	// calibrated coeficients
	cc->coef_df = APDS_CALIB_COEF_DF;
	cc->coef_ga = APDS_CALIB_COEF_GA;
	cc->coef_b = APDS_CALIB_COEF_B;
	cc->coef_c = APDS_CALIB_COEF_C;
	cc->coef_c = APDS_CALIB_COEF_D;
#endif
}

/***********************
 * Read and return lux value from ALS
***********************/
int apds9900_als_read(struct apds_data *apds_data, u8 *sensor_data_ptr, unsigned int length)
{
	int err;
	int len, size=0;
	u8 *data;

	u8 buf[4];
	u16 ch0_data=0;		//Visible Infrared Wavelength λp=640 nm
	u16 ch1_data=0;		//InVisible Infrared Wavelength λp=850 nm

	int iac_max=0;	//IR Adjusted ALS ADC Counts
	int iac_1=0;
	int iac_2=0;
	int alsit=0; 	//ALS integration time
	int again=0;	//ALS gain
	int lpc=0;		//lux per count
	int lux=0;		//lux value

	struct apds_coefs cc; //calibration coeficients

	apds9900_get_coeficients(&cc);

	// printk(KERN_ERR "[%s:%u] Calibration Coeficients _DF=%d _GA=%d _B=%d _C=%d _D=%d \n",
	//		__FUNCTION__,__LINE__,cc.coef_df,cc.coef_ga,cc.coef_b,cc.coef_c,cc.coef_d);


	data=sensor_data_ptr;
	len=length;

	if(len<APDS_ONEEVENT_SIZE)
	{
		printk(KERN_ERR "[%s:%u] buffer length too small len=%d length=%d\n", __FUNCTION__,__LINE__,len,length);
		return 0;
	}

	//Get ATIME for calculation of ALSIT
	err = mpu_i2c_read(apds_data->client->adapter, APDS_ADDRESS, (COMMAND_MODE|APDS_ATIME_REG), &buf[0], 1);
	if (err < 0){
		printk(KERN_ERR "[%s:%u] ATIME Read Error=%d \n",__FUNCTION__,__LINE__,err);
		return err;
	}

	// alsit = 2.72 * (256 - ATIME);
	alsit = ((256 - buf[0]) * 87)>>5; // Note: 2.72 ms can be estimated as 87 / 32. Multiply by 87 then shift >> by 5 bits.

	//Get AGAIN from register for calculation of lux per count
	err = mpu_i2c_read(apds_data->client->adapter, APDS_ADDRESS, (COMMAND_MODE|APDS_CONTROL_REG), &buf[0], 1);
	if (err < 0){
		printk(KERN_ERR "[%s:%u] AGAIN Read Error=%d \n",__FUNCTION__,__LINE__,err);
		return err;
	}

	buf[0] = buf[0] & 0x03;
	switch (buf[0])
	{
		case APDS_AGAIN_1X:
			again = ALS_GAIN_1;
		break;
		case APDS_AGAIN_8X:
			again = ALS_GAIN_8;
		break;
		case APDS_AGAIN_16X:
			again = ALS_GAIN_16;
		break;
		case APDS_AGAIN_120X:
			again = ALS_GAIN_120;
		break;
	}

	//Calculate lux per count
	lpc = cc.coef_ga * cc.coef_df / ( alsit * again);

	// printk(KERN_INFO "[%s:%u] AGAIN=0x%X again=%d alsit=%d lpc=%d\n",__FUNCTION__,__LINE__,buf[0], again, alsit, lpc);

	//Read status register to see if ALS valid..
	err = mpu_i2c_read(apds_data->client->adapter, APDS_ADDRESS, (COMMAND_MODE|APDS_STATUS_REG), &buf[0], 1);
	if (err < 0){
		printk(KERN_ERR "[%s:%u] STATUS Read Error=%d \n",__FUNCTION__,__LINE__,err);
		return err;
	}

	// LUX calculation
	if(buf[0] & APDS_AVALID)
	{
		//Read CH0 (visible light data) and CH1 (invisible IR data) ADC counts together
		err = mpu_i2c_read(apds_data->client->adapter, APDS_ADDRESS, (COMMAND_MODE|AUTO_INCREMENT|APDS_CDATAL_REG), buf, 4);
		if (err < 0) {
			printk(KERN_ERR "[%s:%u] CH0 & CH1 ADC Counts Read Error=%d \n",__FUNCTION__,__LINE__,err);
			return err;
		}

		//printk(KERN_INFO "[%s:%u] CDATAL_buf[0]=%d CDATAH_buf[1]=%d \n",__FUNCTION__,__LINE__,buf[0],buf[1]);
		ch0_data =(buf[1]<<8) + buf[0];

		//printk(KERN_INFO "[%s:%u] IRDATAL_buf[2]=%d IRDATAH_buf[3]=%d \n",__FUNCTION__,__LINE__,buf[2],buf[3]);
		ch1_data =(buf[3]<<8) + buf[2];

		//Calculate the IR Adjusted count
		iac_1 = ch0_data - ((cc.coef_b * ch1_data)/1000);
		iac_2 = ((cc.coef_c * ch0_data)/1000) - ((cc.coef_d * ch1_data)/1000);

		if (iac_1 >= iac_2)
			iac_max = iac_1;
		else iac_max = iac_2;


		//Calculate Lux
		lux = (iac_max * lpc)/1000;

		//printk(KERN_INFO "[%s:%u] CDATA=%d IRDATA=%d lux=%d\n",__FUNCTION__,__LINE__,ch0_data,ch1_data,lux);

		//TODO ALS Coeficients calibrtion/tuning needs to be done.
		//Calculated lux value becomes negative if..
		//ch1_data (IR count) is greater then approx half of ch0_data
		//sending only +ve lux value to user space
		if(lux >=0)
		{
			//copy lux value to sensor data buffer
			size = APDS_ONEEVENT_SIZE;
			memset(data, 0, size);
			memcpy(data, (u8*)&lux, size);
		}
		else
		{
			printk(KERN_ERR "[%s:%u] ERROR: ch0_data=%d ch1_data=%d iac_1=%d iac_2=%d iac_max=%d lpc=%d lux=%d\n",__FUNCTION__,__LINE__,ch0_data,ch1_data,iac_1,iac_2,iac_max,lpc,lux);
		}

		//printk(KERN_INFO "[%s:%u] ch0_data=0x%X ch1_data=0x%X data[0]=%d  data[1]=%d data[2]=%d data[3]=%d lux=%d\n",__FUNCTION__,__LINE__,ch0_data, ch1_data, data[0],data[1],data[2],data[3],lux);
	}

	return size;
}

int apds9900_light_hw_init(struct apds_data *apds_data)
{
	int err;

	//Initialize the APDS chip with base configuration that all runtimes need
	err = apds9900_base_config(apds_data);
	if(err<0)
		return err;

	//Initialize ALS configuration of the APDS9900
	err = apds9900_als_config(apds_data);
	if(err<0)
		return err;

	return 0;
}

int apds9900_light_set_mode(struct apds_data *apds_data,  unsigned char flag,
							unsigned char mode, unsigned int ms_delay)
{
	int err;

	if(flag)//turn on ALS
	{
		err = apds9900_als_enable(apds_data);
	}
	else//turn off ALS
	{
		err = apds9900_als_disable(apds_data);
	}

	return err;
}


int apds9900_light_get_data(struct apds_data *apds_data, u8 *sensor_data_ptr, unsigned int length)
{

	unsigned int size;
	int err;

	//printk(KERN_INFO "[%s:%u] length=%d\n", __FUNCTION__,__LINE__,length);

	if (length <= SENSOR_DATA_HEADER_SIZE)
	{
		return 0;
	}

	// extract payload size; each sensor will determine if there is enough left, because data sizes vary
	size= length-SENSOR_DATA_HEADER_SIZE;

	err=apds9900_als_read(apds_data, SENSOR_PAYLOAD_PTR(sensor_data_ptr), size);

	SENSOR_TYPE(sensor_data_ptr)= RI_SENSOR_HANDLE_LIGHT;
	if (err<=0)
	{
		if (err < 0)
           printk(KERN_ERR "%s:err=%d\n", __func__,err);

		SENSOR_DATA_SIZE(sensor_data_ptr)=0;

		return err;
	}

	length= err;   // payload size
	SENSOR_DATA_SIZE(sensor_data_ptr)=length;
	length+= SENSOR_DATA_HEADER_SIZE;//total size

	return length;   // total number of bytes filled for MUX: header + payload

}


static irqreturn_t apds9900_irq(int irq, void *data)
{
	struct apds_data *apds = data;
	mutex_lock(&apds->lock);
	atomic_set(&apds->interrupt_state, IT_EVENT);
	wake_up(&apds->signal);
	mutex_unlock(&apds->lock);
	return IRQ_HANDLED;
}

static int apds_open(struct inode* inode, struct file* file)
{
	//misc? driver require this for file_operations for some reasons
	return 0;
}

static ssize_t apds_write(struct file *filp, const char __user *buff, size_t count,loff_t *f_pos)
{
	struct apds_data *apds = container_of(filp->private_data, struct apds_data, dev);
	int ret;
	char *tmp;
	unsigned long mode=0;
	u8 buffer[4];

	mutex_lock(&apds->lock);
#ifdef CONTINUATION_READ
	if(count==1){
		ret=copy_from_user(apds->reg_address, buff, count);
		if(ret==0)
			ret=count;
		else
			ret= -EFAULT;
	} else {
#endif
		tmp = kmalloc(count, GFP_KERNEL);
		if(tmp == NULL){
			ret=-ENOMEM;
			goto write_done;
		}

		ret=copy_from_user(tmp, buff, count);
		if(ret==0){

			mode = simple_strtoul(tmp, NULL, 0);

			printk(KERN_INFO "[%s:%u] mode=%d  ret=%d  tmp=%s count=%d\n",__FUNCTION__,__LINE__,(int)mode, ret, tmp,count);

			switch (mode)
			{
				case 6:	// # echo 6 > /dev/apds9900
					ret = apds9900_light_hw_init(apds);
					printk(KERN_INFO "[%s:%u] INIT APDS9900 ret=%d\n",__FUNCTION__,__LINE__, ret);
					ret = 2;
				break;
				case 7:	// # echo 7 > /dev/apds9900
					ret = apds9900_als_enable(apds);
					printk(KERN_INFO "[%s:%u] Enable ALS ret=%d\n",__FUNCTION__,__LINE__, ret);
					ret = 2;
				break;
				case 8:	// # echo 8 > /dev/apds9900
					ret = apds9900_als_disable(apds);
					printk(KERN_INFO "[%s:%u] Disable ALS ret=%d\n",__FUNCTION__,__LINE__,ret);
					ret = 2;
				break;
				case 9:	// # echo 9 > /dev/apds9900
					ret = apds9900_als_read(apds, buffer, 4);
					printk(KERN_INFO "[%s:%u] Read ALS ret=%d \n",__FUNCTION__,__LINE__,ret);
					printk(KERN_INFO "[%s:%u] buffer[0]=0x%X  buffer[1]=0x%X buffer[2]=0x%X buffer[3]=0x%X\n",__FUNCTION__,__LINE__,buffer[0],buffer[1],buffer[2],buffer[3]);
					ret = 2;
				break;
				default:
					ret= mpu_i2c_write(apds->client->adapter,APDS_ADDRESS, tmp,count) ? -EIO : count;
				break;
			}

		}
		else
			ret= -EFAULT;

		kfree(tmp);
#ifdef CONTINUATION_READ
	}
#endif
write_done:
	mutex_unlock(&apds->lock);
#ifdef SENSOR_DEBUG
	SENSOR_INFO("%s:%d\n", __func__,ret);
#endif
	return ret;
}
#ifdef CONTINUATION_READ
static ssize_t apds_read(struct file *filp, char __user *buff, size_t count,loff_t *f_pos)
{
	struct apds_data *apds = container_of(filp->private_data, struct apds_data, dev);
	int ret;
	char *tmp;

	mutex_lock(&apds->lock);
	tmp = kmalloc(count, GFP_KERNEL);
	if(tmp == NULL){
		ret=-ENOMEM;
		goto read_done;
	}

	ret=mpu_i2c_read(apds->client->adapter,APDS_ADDRESS, apds->reg_address, tmp,count);
	if(ret==0)
		ret= copy_to_user(buff, tmp, count) ? -EFAULT : count;
	else
		ret= -EIO;

	kfree(tmp);
read_done:
	mutex_unlock(&apds->lock);
#ifdef SENSOR_DEBUG
	SENSOR_INFO("%s:%d\n", __func__,ret);
#endif
	return ret;
}
#endif

static long apds_ioctl(struct file *filp,unsigned int cmd, unsigned long arg)
{
	struct apds_data *apds = container_of(filp->private_data, struct apds_data, dev);
	int ret=0;
	unsigned long count=0;
	u8 reg_address;
	u8 buf[2];
	ret=mutex_lock_interruptible(&apds->lock);
	if(ret)
	{
		printk("%s: mutex_lock_interruptible returned %d\n",__func__, ret);
		return ret;
	}
	
	switch(cmd)
	{
		case APDS_READ_BYTE:
			count=1;
		break;
		case APDS_READ_2BYTES:
			count=2;
		break;
		case APDS_CLEAR_IT:
			get_user(reg_address, (u8 __user *)arg);
			reg_address|=(COMMAND_MODE|SPECIAL_FUNCTION);
			ret=mpu_i2c_write(apds->client->adapter,APDS_ADDRESS, &reg_address,1);
		break;
		case APDS_FAKE_IT:
			atomic_set(&apds->interrupt_state, IT_EVENT);
			wake_up(&apds->signal);
		break;
		default:
			ret=-EINVAL;
		break;
	}

	if(count){
		get_user(reg_address, (u8 __user *)arg);
		reg_address|=COMMAND_MODE;
		if(count==2)
			reg_address|=AUTO_INCREMENT;
		ret=mpu_i2c_read(apds->client->adapter,APDS_ADDRESS, reg_address, buf,count);
		if(ret==0)
			ret=copy_to_user((u8 __user *)arg, buf, count);
		else
			printk(KERN_ERR "%s i2c error:%d",__func__, ret);
	}

#ifdef SENSOR_DEBUG
	SENSOR_INFO("%s:%d\n", __func__,ret);
#endif
	mutex_unlock(&apds->lock);
	return ret;
}

static unsigned int apds_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct apds_data *apds = container_of(filp->private_data, struct apds_data, dev);
	poll_wait(filp, &apds->signal, wait);
	if(atomic_read(&apds->interrupt_state)==IT_EVENT){
		atomic_set(&apds->interrupt_state, NO_IT_EVENT);
		return (POLLIN | POLLRDNORM); /* readable */
	}
	return 0;
}

/* File operations on proxmux device */
static const struct file_operations apds_fops = 
{
	.owner                  = THIS_MODULE,
	.open                   = apds_open,
#ifdef CONTINUATION_READ
	.read                   = apds_read,
#endif
	.write                  = apds_write,
	.unlocked_ioctl         = apds_ioctl,
	.poll                   = apds_poll,
};


static int apds_probe(struct i2c_client *client,
						   const struct i2c_device_id *id)
{
	int err = 0;
	struct apds_data *apds;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		dev_err(&client->dev, "client not i2c capable\n");
		return -ENODEV;
	}

	apds = kzalloc(sizeof(*apds), GFP_KERNEL);
	if (!apds) 
	{
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, apds);
	apds->client = client;
	mutex_init(&apds->lock);

	// Register misc device
	apds->dev.minor = MISC_DYNAMIC_MINOR;
	apds->dev.name = DRIVER_NAME;
	apds->dev.fops = &apds_fops;

	err = misc_register(&apds->dev);
	if (err < 0)
	{
		printk (KERN_ERR "Failed to register %s\n", apds->dev.name);
		goto misc_register_err;
	}


	atomic_set(&apds->interrupt_state, NO_IT_EVENT);
	init_waitqueue_head(&(apds->signal));
	err = request_threaded_irq(client->irq, NULL,
				apds9900_irq,
				IRQF_TRIGGER_FALLING |IRQF_ONESHOT,
				DRIVER_NAME, apds);
	if (err) {
		dev_err(&client->dev, "could not get IRQ %d\n",
			client->irq);
		goto misc_register_err;
	}
#ifdef SENSOR_DEBUG
	SENSOR_INFO("%s:irq=%d\n", __func__,client->irq);
#endif

	err = apds9900_light_hw_init(apds);
	if (err < 0)
	{
		printk (KERN_ERR "Failed apds9900_light_hw_init() %s\n", apds->dev.name);
		goto misc_register_err;
	}

	// register apds9900 light sensor with proxmux driver
	err = risensor_register(RI_SENSOR_HANDLE_LIGHT, snsrnamelight, apds,
						RI_SENSOR_MODE_CR, RI_SENSOR_MODE_CR,
						APDS_ONEEVENT_SIZE + SENSOR_DATA_HEADER_SIZE,
                  (PFNSENS_ACTIVATE)apds9900_light_set_mode,
                  (PFNSENS_READ)apds9900_light_get_data);

	printk(KERN_INFO "[%s:%u] risensor_register() err=%d SENSOR_DATA_HEADER_SIZE=%d\n", __FUNCTION__,__LINE__,err,SENSOR_DATA_HEADER_SIZE );
	if (err < 0)
	{
		printk (KERN_ERR "couldn't register light sensor with proxmux %s\n", apds->dev.name);
		goto misc_register_err;
	}

	return 0;

misc_register_err:
	kfree(apds);
	return err;
}

static int __devexit apds_remove(struct i2c_client *client)
{
	struct apds_data *apds = i2c_get_clientdata(client);
	printk("%s\n", __func__);
	free_irq(client->irq, apds);
	misc_deregister(&(apds->dev));
	kfree(apds);
	return 0;
}

static const struct i2c_device_id apds_id[] = 
{
		{DRIVER_NAME, 0},
		{},
};

MODULE_DEVICE_TABLE(i2c, apds_id);

static struct i2c_driver apds_driver = 
{
	.driver =
	{
		.owner = THIS_MODULE,
		.name = DRIVER_NAME,
	},
	.probe     = apds_probe,
	.remove    = __devexit_p(apds_remove),
	.id_table  = apds_id,
};

static int __init apds_init(void)
{
	int res=i2c_add_driver(&apds_driver);
	pr_info("%s: Probe name %s\n", __func__, DRIVER_NAME);
	if(res)
		printk(KERN_ERR "%s failed\n", __func__);
	return res;
}

static void __exit apds_exit(void)
{
	pr_info("%s\n", __func__);
	i2c_del_driver(&apds_driver);
}
/* Module Entry Points */
module_init(apds_init);
module_exit(apds_exit);

MODULE_DESCRIPTION("Reconinstruemnts apds");
MODULE_AUTHOR("Li Chen");
MODULE_LICENSE("GPL");

