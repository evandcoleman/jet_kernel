#include "jet_sensors.h"
#include "jet_lsm9ds0.h"
#include <linux/device.h>
#include <linux/miscdevice.h>

#define I2C_AUTO_INCREMENT	(0x80)

#define ALL_ZEROES		(0x00)
#define PM_OFF			(0x00)
#define FIFO_ENABLE		(1<<6)
#define HP_CLICK_FILTER (1<<2)
#define ENABLE_ALL_AXES	(0x07)
#define FIFO_STREAM		(0x40)
#define FIFO_BYPASS		(0x00)
#define FIFO_OVRN		(0x40)
#define FIFO_DATA_LEVEL_MASK	(0x1F)

#define CTRL_REG1		(0x20)	/* CTRL REG1 */
#define OUT_X_L			(0x28)	/* 1st AXIS OUT REG of 6 */
#define FIFO_CTRL_REG	(0x2E)	/* FIFO CONTROL REGISTER */
#define FIFO_SRC_REG	(0x2F)	/* FIFO SOURCE REGISTER */

//Gyro-----------------------------------------
#ifdef CONFIG_JET_V2
#define GYRO_ADDRESS 0x6B
#else
#define GYRO_ADDRESS 0x6A
#endif
#define G_WHO_AM_I	(0x0F)
#define G_CTRL_REG1	(0x20)	/* CTRL REG1 */
#define G_CTRL_REG2	(0x21)	/* CTRL REG2 */
#define G_CTRL_REG3	(0x22)	/* CTRL_REG3 */
#define G_CTRL_REG4	(0x23)	/* CTRL_REG4 */
#define G_CTRL_REG5	(0x24)	/* CTRL_REG5 */
#define REFERENCE	(0x25)	/* REFERENCE REG */
#define OUT_TEMP_G	(0x26)	/* 8 bit temperature */

#define G_ID		(0xD4)
#define PM_NORMAL	(0x08)
#define BW00		(0x00)
#define BW01		(0x10)
#define BW10		(0x20)
#define BW11		(0x30)
#define ODR_MASK	(0xC0)
#define G_ODR095		(0x00)	/* ODR =  95Hz */
#define G_ODR190		(0x40)	/* ODR = 190Hz */
#define G_ODR380		(0x80)	/* ODR = 380Hz */
#define G_ODR760		(0xC0)	/* ODR = 760Hz */
#define GYRO_FS_2000DPS		(0x20)  //0.07 dps/LSB
#define G_BDU			(0x80)	/* output registers not updated until MSB and LSB have been read */

//Acc+Mag--------------------------------------------------
#ifdef CONFIG_JET_V2
#define ACC_MAG_ADDRESS 0x1D
#else
#define ACC_MAG_ADDRESS 0x1E
#endif
#define AM_WHO_AM_I			(0x0F)
#define AM_INT_CTRL_REG		(0x12)
#define AM_CNTRL0_ADDR		(0x1F)	/** CNTRL0 address register */
#define AM_CNTRL1_ADDR		(0x20)	/** CNTRL1 address register */
#define AM_CNTRL2_ADDR		(0x21)	/** CNTRL2 address register */
#define AM_CNTRL3_ADDR		(0x22)	/** CNTRL3 address register */
#define AM_CNTRL4_ADDR		(0x23)	/** CNTRL4 address register */
#define AM_CNTRL5_ADDR		(0x24)	/** CNTRL5 address register */
#define AM_CNTRL6_ADDR		(0x25)	/** CNTRL6 address register */
#define AM_CNTRL7_ADDR		(0x26)	/** CNTRL7 address register */
#define AM_FIFO_SRC_REG		(0x2F)	/* FIFO SOURCE REGISTER */
#define OUT_TEMP_L_XM		(0x05)	/* 16 bit temperature lower byte */
#define OUT_TEMP_H_XM		(0x06)	/* 16 bit temperature higher byte */
#define M_STATUS_REG		(0x07)
#define M_OUT_X_L			(0x08)
#define MAG_TEMP_EN         (1<<7)

#define AM_ID			(0x49)
#define A_ODR025		(0x40)	/* ODR =  25Hz */
#define A_ODR050		(0x50)	/* ODR = 50Hz */
#define A_ODR100		(0x60)	/* ODR = 100Hz */
#define A_ODR200		(0x70)	/* ODR = 200Hz */
#define AM_BDU			(0x8)	/* output registers not updated until MSB and LSB have been read; This bit is for both Acc and Mag , so keep it on no matter turn on/off Acc */
#define A_FS_4G			(0x08)	//0.12 mg/LSB
#define M_ODRRESERVE	(0x18)
#define M_ODR025		(0x0C)	/* ODR =  25Hz */
#define M_ODR100		(0x14)	/* ODR = 100Hz */
#define M_FS_4GAUSS		(0x20)	/* 0.16 mgauss/LSB: 0.016uT/LSB */
#define M_CONTINUOUS	(0x00)
#define M_SINGLE		(0x01)	/* Megnetometer single conversion*/
#define M_PM_DOWN		(0x03)
#define M_DATE_READY	(0x7)

// Interrupts
/*        Interrupt mode
-------------------------------------
AOI | 6D | Interrupt mode
 0  | 0  | OR combination of interrupt events
 0  | 1  | 6 direction movement recognition
 1  | 0  | AND combination of interrupt events
 1  | 1  | 6 direction position recognition   */
#define INT_GEN_1_REG			(0x30)
#define INT_GEN_1_SRC			(0x31)
#define INT_GEN_1_THS			(0x32)
#define INT_GEN_1_DURATION		(0x33)
#define INT_GEN_2_REG			(0x34)
#define INT_GEN_2_SRC			(0x35)
#define INT_GEN_2_THS			(0x36)
#define INT_GEN_2_THS_VAL		(0x08)
#define INT_GEN_2_DURATION		(0x37)
#define INT_GEN_2_DURATION_VAL	(0x01)
#define INT_ACTIVE_HIGH			(0x8)
#define INT_AND_CONDITION		(0x80)
#define AXIS_LOW_INT_EVENT		(0x15) /* AOI 6D ZHIE/ZUPE ZLIE/ZDOWNE YHIE/YUPE YLIE/YDOWNE XHIE/XUPE XLIE/XDOWNE = 0001 0101 => 0x15 */
#define AXIS_HIGH_INT_EVENT		(0x2A) /* AOI 6D ZHIE/ZUPE ZLIE/ZDOWNE YHIE/YUPE YLIE/YDOWNE XHIE/XUPE XLIE/XDOWNE = 0010 1010 => 0x2A */
#define IA_EVENT				(0x40) /* Interrupt active */
#define P1_TAP_EN				(1<<6)
#define P1_INT1_EN				(1<<5)
#define P1_INT2_EN				(1<<4)
#define P2_TAP_EN				(1<<7)
#define P2_INT1_EN				(1<<6)
#define P2_INT2_EN				(1<<5)

// TAP TAP
#define P2_TAP_EN				(1<<7)

#define CLICK_CFG 			(0x38)
#define CLICK_CFG_VAL		(0x20) /* ZD ZS YD YS XD XS = 0010 0000 => 0x20 Z Axis [D]ouble */

#define CLICK_SRC			(0x39)

#define CLICK_THS			(0x3A)
#define CLICK_THS_VAL		(0x10) /* 0.5g @ 4g FS */

#define TIME_LIMIT			(0x3B)
#define TIME_LIMIT_VAL		(0x02) /* 20ms */

#define TIME_LATENCY		(0x3C)
#define TIME_LATENCY_VAL	(0x09) /* 90ms */

#define TIME_WINDOW			(0x3D)
#define TIME_WINDOW_VAL		(0x20) /* 320 ms */

#define ACT_THS				(0x3E)
#define ACT_DUR				(0x3F)
// CLICK_SRC
#define CLICK_SRC_IA_BIT		(1<<6)
#define CLICK_SRC_DCLICK_BIT	(1<<5)
#define CLICK_SRC_STAP_BIT		(1<<4)
#define CLICK_SRC_SIGN_BIT		(1<<3)
#define CLICK_SRC_Z_BIT			(1<<2)
#define CLICK_SRC_Y_BIT			(1<<1)
#define CLICK_SRC_X_BIT			(1<<0)

#ifdef CONFIG_JET_SENSORS_TAP_TAP

#define DEFAULT_IDLE_ACC_DELAY_MS	10

#define KEY_PRESSED 		1
#define KEY_RELEASED 		0
#define TT_KEY_ENTER   		0
#define TT_KEY_BACK    		1
#define MAX_KEYPAD_CNT		2
unsigned int tap_tap_keypad_keycode_map[MAX_KEYPAD_CNT] = {KEY_ENTER, KEY_BACK};
unsigned char tap_tap_keypad_keycode_str[MAX_KEYPAD_CNT][20] = { "KEY_ENTER", "KEY_BACK"};

#else

#define DEFAULT_IDLE_ACC_DELAY_MS	1000

#endif

#define GYRO_INDEX 0
#define ACC_INDEX 1
static struct lsm_status {
	u8 low_odr;
	u8 high_odr;
	u8 threshold;
	u8 power_on;
	u8 power_off;
	u8 fifo_mode;
} lsm_status_array[] = {
	{
		.low_odr= G_ODR095,
		.high_odr= G_ODR190,
		.threshold= 15, //ms
		.power_on=  BW00 | ENABLE_ALL_AXES | PM_NORMAL,
		.power_off= PM_OFF,
		.fifo_mode=RI_SENSOR_MODE_CR,
	},
	{
#ifdef CONFIG_JET_SENSORS_TAP_TAP
		.low_odr= A_ODR100,
#else
		.low_odr= A_ODR025,
#endif
		.high_odr= A_ODR100,
		.threshold= 40,
		.power_on= AM_BDU|ENABLE_ALL_AXES,//The AM_BDU is shared for both Mag and Acc, so keep it on
		.power_off= AM_BDU|PM_OFF,
		.fifo_mode=RI_SENSOR_MODE_CR,
	},
};

static unsigned int	mag_ms_delay;
#define MAG_TIME_THRESHOLD	20 //ms


static u8 get_chip_index(unsigned char chip_address)
{
	if(chip_address==GYRO_ADDRESS)
	{
		return GYRO_INDEX;
	}
	else
	{
		return ACC_INDEX;
	}
}
static int lsm9ds0_set_fifo(struct jet_sensors *jet_sensors, unsigned char fifo_mode, unsigned char chip_address)
{
	u8 buffer[2];
	int err=0;
	u8 index;
	u8 fifo_mode_old;

	index= get_chip_index(chip_address);
	fifo_mode_old= lsm_status_array[index].fifo_mode;
	SENSOR_INFO("fifo_mode_old=%d,fifo_mode_new=%d\n", 
				fifo_mode_old, fifo_mode);

	if(fifo_mode!=fifo_mode_old)
	{
		buffer[0]= FIFO_CTRL_REG;
		if(fifo_mode==RI_SENSOR_MODE_FIFO)
			buffer[1]= FIFO_STREAM;
		else
			buffer[1]= FIFO_BYPASS;
		err=jet_i2c_write(jet_sensors, chip_address , buffer, 2);
		if(err==0)
			lsm_status_array[index].fifo_mode=fifo_mode;
	}

	return err;
}

static int lsm9ds0_gyro_set_fifo(struct jet_sensors *jet_sensors, unsigned char fifo_mode)
{
	return lsm9ds0_set_fifo(jet_sensors, fifo_mode, GYRO_ADDRESS);
}

static int lsm9ds0_acc_set_fifo(struct jet_sensors *jet_sensors, unsigned char fifo_mode)
{
	return lsm9ds0_set_fifo(jet_sensors, fifo_mode, ACC_MAG_ADDRESS);
}
/*!
 * \brief config CTRL_REG1 register
 */
static int lsm9ds0_set_mode(struct jet_sensors *jet_sensors,  unsigned char flag,
							unsigned int ms_delay, unsigned char chip_address)
{
	u8 buffer[2];
	u8 chip_index;
	u8 odr;
	//TODO: add more ODR support? but for our cpu and android, around 100Hz seems good enough
	chip_index= get_chip_index(chip_address);
#ifdef SENSOR_DEBUG_VERBOSE
		SENSOR_INFO("threshold=%d, powercmd=0x%.2X\n",lsm_status_array[chip_index].threshold, lsm_status_array[chip_index].power_on);
#endif
	if(ms_delay < lsm_status_array[chip_index].threshold)
		odr=lsm_status_array[chip_index].high_odr;
	else
		odr=lsm_status_array[chip_index].low_odr;

	buffer[0]=CTRL_REG1;
	if(flag)//turn on
	{
		buffer[1]= ((odr) | (lsm_status_array[chip_index].power_on));
	}
	else//turn off
	{
		buffer[1]= lsm_status_array[chip_index].power_off;
	}

	return jet_i2c_write(jet_sensors,chip_address, buffer, 2);
}

static int lsm9ds0_get_data(struct jet_sensors *jet_sensors, u8 *sensor_data_ptr, unsigned int length, unsigned char chip_address)
{
	int err;
	u8 buffer, index;
	int len, size;
	u8 *data;
	u8 fifo_mode;
	
	data=sensor_data_ptr;
	len=length;

	if(len<LSM_DATA_ONEEVENT_SIZE)
	{
		printk(KERN_ERR "%s: buffer length too small %d\n", __func__,len);
		return 0;
	}

	index= get_chip_index(chip_address);
	fifo_mode= lsm_status_array[index].fifo_mode;

	if(fifo_mode==RI_SENSOR_MODE_FIFO)
	{
		err=jet_i2c_read(jet_sensors, chip_address,
			FIFO_SRC_REG,&buffer, 1);
#ifdef SENSOR_DEBUG_VERBOSE
		SENSOR_INFO("FIFO_SRC_REG=%.2x\n",buffer);
#endif
		if(err)
			return err;

		if(buffer&FIFO_OVRN)
			len=LSM_FIFO_LENGTH_MAX;
		else
		{
			len=(buffer&FIFO_DATA_LEVEL_MASK);

			if(len==0)
			{
				printk(KERN_ERR "%s: FIFO no data\n", __func__);
				return 0;//FIFO empty
			}
			else
			{
				len= len*LSM_DATA_ONEEVENT_SIZE;//each event needs 6 bytes(x,y,z short)
			}
		}
		size= (len <= length) ? len : length;

		len= size%LSM_DATA_ONEEVENT_SIZE;
		if(len)//must be multiples of LSM_DATA_ONE
		{
			printk(KERN_INFO "%s: truncate to multiplers of LSM_DATA_ONEEVENT_SIZE\n", __func__);
			size= size-len;
		}

		err=jet_i2c_read(jet_sensors, chip_address,
			(OUT_X_L|I2C_AUTO_INCREMENT), data, size);
	}
	else
	{
		size=LSM_DATA_ONEEVENT_SIZE;
		err=jet_i2c_read(jet_sensors, chip_address,
			(OUT_X_L|I2C_AUTO_INCREMENT), data, LSM_DATA_ONEEVENT_SIZE);
	}

	if(err==0)
		return size;
	return err;
}


int lsm9ds0_gyro_hw_init(struct jet_sensors *jet_sensors)
{
	int err;
	u8 buffer[6];

	err=jet_i2c_read(jet_sensors, GYRO_ADDRESS,
		G_WHO_AM_I,&buffer[0], 1);
	if(err<0)
		return err;

	if(buffer[0]!=G_ID)
		return (-ENXIO); // No such device or address

	buffer[0]= G_CTRL_REG1|I2C_AUTO_INCREMENT;
	buffer[1]= PM_OFF|G_ODR095;//Gyro default ODR
	buffer[2]= ALL_ZEROES;
	buffer[3]= ALL_ZEROES;
	buffer[4]= G_BDU|GYRO_FS_2000DPS;
	buffer[5]= FIFO_ENABLE;
	err=jet_i2c_write(jet_sensors,GYRO_ADDRESS, buffer, 6);
	if(err<0)
		return err;

	//force to enable fifo by default
	return lsm9ds0_gyro_set_fifo(jet_sensors, RI_SENSOR_MODE_FIFO);
}

int lsm9ds0_gyro_set_mode(struct jet_sensors *jet_sensors,  unsigned char flag, 
							unsigned char mode, unsigned int ms_delay)
{
	int err;

	err=lsm9ds0_gyro_set_fifo(jet_sensors, mode);
	if(err)
		return err;

	return lsm9ds0_set_mode(jet_sensors,flag, ms_delay, GYRO_ADDRESS);
}

int lsm9ds0_gyro_get_data(struct jet_sensors *jet_sensors, u8 *sensor_data_ptr, unsigned int length)
{
	return lsm9ds0_get_data(jet_sensors, sensor_data_ptr, length, GYRO_ADDRESS);
}


int lsm9ds0_acc_hw_init(struct jet_sensors *jet_sensors)
{
	//add new device in the same bus?
	//i2c_new_dummy(jet_sensors->client->adapter, ACC_MAG_ADDRESS);
	int err;
	u8 buffer[6];

	err=jet_i2c_read(jet_sensors, ACC_MAG_ADDRESS,
		AM_WHO_AM_I,&buffer[0], 1);
	if(err<0)
		return err;

	if(buffer[0]!=AM_ID)
		return (-ENXIO); // No such device or address

	buffer[0]= AM_CNTRL0_ADDR | I2C_AUTO_INCREMENT;
	buffer[1]= FIFO_ENABLE | HP_CLICK_FILTER;
	buffer[2]= AM_BDU|PM_OFF;
	buffer[3]= A_FS_4G;
	buffer[4]= ALL_ZEROES;
	buffer[5]= ALL_ZEROES;

	err=jet_i2c_write(jet_sensors,ACC_MAG_ADDRESS, buffer, 6);
	if(err<0)
		return err;

	return lsm9ds0_acc_set_fifo(jet_sensors, RI_SENSOR_MODE_FIFO);
}

int lsm9ds0_acc_set_mode(struct jet_sensors *jet_sensors,  unsigned char flag,
							unsigned char mode, unsigned int ms_delay)
{
	int err;

	err=lsm9ds0_acc_set_fifo(jet_sensors, mode);
	if(err)
		return err;
	if(flag==0) // should turn on acc all the time for freefall?
	{
		flag = 1; // Force on for tap-tap for now
		ms_delay = DEFAULT_IDLE_ACC_DELAY_MS;
	}
	return lsm9ds0_set_mode(jet_sensors,flag, ms_delay, ACC_MAG_ADDRESS);
}

int lsm9ds0_acc_get_data(struct jet_sensors *jet_sensors, u8 *sensor_data_ptr, unsigned int length)
{
	return lsm9ds0_get_data(jet_sensors, sensor_data_ptr, length, ACC_MAG_ADDRESS);
}

int lsm9ds0_mag_hw_init(struct jet_sensors *jet_sensors)
{
	u8 buffer[4];
	mag_ms_delay=1000;//default setting
	buffer[0] = AM_CNTRL5_ADDR|I2C_AUTO_INCREMENT;
	buffer[1] = M_ODR100;//ODR no effect in single conversion mode
#ifdef JET_SENSORS_MAG_TEMPERATURE
	buffer[1] |= MAG_TEMP_EN;
#endif
	buffer[2] = M_FS_4GAUSS;
	buffer[3] = M_PM_DOWN;

	return jet_i2c_write(jet_sensors,ACC_MAG_ADDRESS, buffer, 4);

}

int lsm9ds0_mag_set_mode(struct jet_sensors *jet_sensors,  unsigned char flag,
							unsigned char mode, unsigned int ms_delay)
{
	u8 buffer[2];
	buffer[0]=AM_CNTRL7_ADDR;

	if(flag){
		mag_ms_delay=ms_delay;
		if(mag_ms_delay>=MAG_TIME_THRESHOLD)
			buffer[1]=M_SINGLE;
		else
			buffer[1]=M_CONTINUOUS;
		/*TODO: For fast polling(max 100Hz for current compass)
		* the accelerometer must in power-down mode or ODR>=100.
		* See data sheet Page 58 note
		*/
	}
	else{
		buffer[1]=M_PM_DOWN;
		mag_ms_delay=1000;//default setting
	}
	return jet_i2c_write(jet_sensors,ACC_MAG_ADDRESS, buffer, 2);
}

int lsm9ds0_mag_get_data(struct jet_sensors *jet_sensors, u8 *sensor_data_ptr, unsigned int length)
{
	int err;
	int len, size;
	u8 *data;
	u8 buffer[2];

	data=sensor_data_ptr;
	len=length;
	if(len<LSM_DATA_ONEEVENT_SIZE)
	{
		printk(KERN_ERR "%s: buffer length too small %d\n", __func__,len);
		return 0;
	}

#if 0//test code
	err=jet_i2c_read(jet_sensors, ACC_MAG_ADDRESS,
					M_STATUS_REG,buffer, 1);
	printk("M_STATUS_REG=%.2x\n",buffer[0]);
#endif

	if(mag_ms_delay>=MAG_TIME_THRESHOLD){//single conversion

		err=jet_i2c_read(jet_sensors, ACC_MAG_ADDRESS,
					AM_CNTRL7_ADDR,buffer, 1);

		//SENSOR_INFO("AM_CNTRL7_ADDR=%.2x\n",buffer[0]);
		if((buffer[0]&M_PM_DOWN) != M_PM_DOWN)
		{
			printk("mag data not ready\n");
			return 0; //return 0 data length
		}
	}

	size=LSM_DATA_ONEEVENT_SIZE;
	err=jet_i2c_read(jet_sensors, ACC_MAG_ADDRESS,
		(M_OUT_X_L|I2C_AUTO_INCREMENT), data, size);
	if(err)
		return err;

	if(mag_ms_delay>=MAG_TIME_THRESHOLD){//single conversion
		buffer[0]=AM_CNTRL7_ADDR;
		buffer[1]=M_SINGLE;
		err=jet_i2c_write(jet_sensors,ACC_MAG_ADDRESS, buffer, 2);
		if(err)
			return err;
	}
	return size;
}

#ifdef CONFIG_JET_SENSORS_TAP_TAP
int lsm9ds0_tap_tap_set_mode(struct jet_sensors *jet_sensors,  unsigned char flag)
{
	int err;
	u8 buffer[2];

	buffer[0] = AM_CNTRL3_ADDR;

	if(flag)
		buffer[1] = P1_TAP_EN;//we don't want to enable other type of interrupt for int1 pin
	else
		buffer[1] =0;
	err=jet_i2c_write(jet_sensors, ACC_MAG_ADDRESS, buffer, 2);
	if(err){
		printk(KERN_ERR "cannot set %s\n", __func__);
		return err;
	}
	return 0;
}
int lsm9ds0_tap_tap_hw_init(struct jet_sensors *jet_sensors) {
	u8 buffer[5];
	int err;
	int key, code;

	printk(KERN_INFO "[%s:%u]\n",__FUNCTION__,__LINE__);

	jet_sensors->tap_tap_input_dev = input_allocate_device();
	if (jet_sensors->tap_tap_input_dev == NULL)
	{
		printk(KERN_ERR "[%s:%u] Couldn't allocate input device\n",__FUNCTION__,__LINE__);
		return -ENOMEM;
	}

	jet_sensors->tap_tap_input_dev->name = "tap_tap";
	jet_sensors->tap_tap_input_dev->dev.init_name = "tap_tap";
	jet_sensors->tap_tap_input_dev->phys = "9_axis";
	jet_sensors->tap_tap_input_dev->id.vendor = 0x0001;//0xDEAD;
	jet_sensors->tap_tap_input_dev->id.product = 0x0001;//0xBEEF;
	jet_sensors->tap_tap_input_dev->id.version = 0x0100;//0x01;

	set_bit(EV_KEY, jet_sensors->tap_tap_input_dev->evbit);
	/* Allocation key event bits */
	for(key = 0; key < MAX_KEYPAD_CNT; key++){
		code = tap_tap_keypad_keycode_map[key];
		if(code <= 0)
			continue;
		set_bit(code&KEY_MAX, jet_sensors->tap_tap_input_dev->keybit);
	}

	if(input_register_device(jet_sensors->tap_tap_input_dev)) {
		printk(KERN_ERR "[%s:%u] Couldn't register input device\n",__FUNCTION__,__LINE__);
		input_free_device(jet_sensors->tap_tap_input_dev);
		return -ENOMEM;
	}

	buffer[0] = CLICK_CFG;
	buffer[1] = CLICK_CFG_VAL;
	err = jet_i2c_write(jet_sensors, ACC_MAG_ADDRESS, buffer, 2);
	if(err)	goto err1;

	buffer[0] = CLICK_THS|I2C_AUTO_INCREMENT;
	buffer[1] = CLICK_THS_VAL;
	buffer[2] = TIME_LIMIT_VAL;
	buffer[3] = TIME_LATENCY_VAL;
	buffer[4] = TIME_WINDOW_VAL;
	err=jet_i2c_write(jet_sensors, ACC_MAG_ADDRESS, buffer, 5);
	if(err) goto err1;


	//buffer[0] = AM_CNTRL3_ADDR;
	//err=jet_i2c_read(jet_sensors, ACC_MAG_ADDRESS, buffer[0], &buffer[1], 1);
	//if(err) goto err1;

	//buffer[1] |= P1_TAP_EN;
	//err=jet_i2c_write(jet_sensors, ACC_MAG_ADDRESS, buffer, 2);
	//if(err) goto err1;
	if(lsm9ds0_tap_tap_set_mode(jet_sensors, 1))
		goto err1;
	return 0;

err1:
	input_unregister_device(jet_sensors->tap_tap_input_dev);
	return err;
}

int lsm9ds0_tap_tap_disable(struct jet_sensors *jet_sensors)
{
	int err;
	u8 buffer[2];

	printk(KERN_INFO "[%s:%u]\n",__FUNCTION__,__LINE__);

	buffer[0]= AM_CNTRL3_ADDR;
	err=jet_i2c_read(jet_sensors, ACC_MAG_ADDRESS, buffer[0], &buffer[1], 1);
	if(err)
		return err;

	buffer[1] &= (0xff ^ P1_TAP_EN);
	err=jet_i2c_write(jet_sensors, ACC_MAG_ADDRESS, buffer, 2);
	if(err)
		return err;

	return 0;
}

void tap_tap_click_run(struct input_dev *tap_tap_input_dev,int code)
{
	input_report_key(tap_tap_input_dev, tap_tap_keypad_keycode_map[code], KEY_PRESSED);
	input_sync(tap_tap_input_dev);
	input_report_key(tap_tap_input_dev, tap_tap_keypad_keycode_map[code], KEY_RELEASED);
	input_sync(tap_tap_input_dev);
}

void tap_tap_irq_work_func(struct work_struct *work)
{
	int err;
	u8 buffer;

	struct jet_sensors *jet_sensors = container_of(work, struct jet_sensors, tap_tap_irq_work);

	printk(KERN_INFO "[%s:%u]\n",__FUNCTION__,__LINE__);

	err = jet_i2c_read(jet_sensors, ACC_MAG_ADDRESS, CLICK_SRC, &buffer, 1);
	if(err)
		printk(KERN_ERR "[%s:%u] CLICK_SRC read wrong\n",__FUNCTION__,__LINE__);
	else {
		printk(KERN_INFO "[%s:%u] CLICK_SRC=0x%x\n",__FUNCTION__,__LINE__, buffer);
//		if(buffer & CLICK_SRC_IA_BIT) { printk(KERN_INFO "CLICK_SRC_IA_BIT\n");}
//		if(buffer & CLICK_SRC_DCLICK_BIT) { printk(KERN_INFO "CLICK_SRC_DCLICK_BIT\n");}
//		if(buffer & CLICK_SRC_STAP_BIT) { printk(KERN_INFO "CLICK_SRC_STAP_BIT\n");}
//		if(buffer & CLICK_SRC_SIGN_BIT) { printk(KERN_INFO "CLICK_SRC_SIGN_BIT\n");}
//		if(buffer & CLICK_SRC_Z_BIT) { printk(KERN_INFO "CLICK_SRC_Z_BIT\n");}
//		if(buffer & CLICK_SRC_Y_BIT) { printk(KERN_INFO "CLICK_SRC_Y_BIT\n");}
//		if(buffer & CLICK_SRC_X_BIT) { printk(KERN_INFO "CLICK_SRC_X_BIT\n");}
		if((buffer & CLICK_SRC_DCLICK_BIT) && (buffer & CLICK_SRC_Z_BIT)) {
			printk(KERN_INFO "[%s:%u] Double Click Detected!\n",__FUNCTION__,__LINE__);
			tap_tap_click_run(jet_sensors->tap_tap_input_dev, TT_KEY_BACK);
		}
	}
}
#endif

#ifdef CONFIG_JET_SENSORS_FREE_FALL
int lsm9ds0_freefall_set_mode(struct jet_sensors *jet_sensors,  unsigned char flag)
{
	int err;
	u8 buffer[2];
#ifdef FREEFALL_API
	int irq_status;
#endif
	buffer[0] = AM_CNTRL4_ADDR;//AM_CNTRL3_ADDR;

	if(flag){
		buffer[1] = P2_INT1_EN;//P1_INT1_EN;//we don't want to enable other type of interrupt for int1 pin
#ifdef FREEFALL_API
		irq_status=atomic_read(&(jet_sensors->irq_ff_status));
		if(irq_status==IRQ_DISABLE_FROM_CPU){
			enable_irq(jet_sensors->pdata->irq_ff); 
			atomic_set(&jet_sensors->irq_ff_status, IRQ_ENABLE_FROM_CPU);
		}
#endif
	}
	else
		buffer[1] =0;
	err=jet_i2c_write(jet_sensors, ACC_MAG_ADDRESS, buffer, 2);
	if(err){
		printk(KERN_ERR "cannot set %s\n", __func__);
		return err;
	}
	return 0;
}

int lsm9ds0_freefall_hw_init(struct jet_sensors *jet_sensors)
{
	u8 buffer[4];
	int err;
	buffer[0]= AM_INT_CTRL_REG;
	buffer[1]= INT_ACTIVE_HIGH;
	err=jet_i2c_write(jet_sensors,ACC_MAG_ADDRESS, buffer, 2);
	if(err)
		return err;

	buffer[0] = INT_GEN_1_REG;
	buffer[1] = (INT_AND_CONDITION | AXIS_LOW_INT_EVENT);
	err=jet_i2c_write(jet_sensors,ACC_MAG_ADDRESS, buffer, 2);
	if(err)
		return err;

	buffer[0] = INT_GEN_1_THS|I2C_AUTO_INCREMENT;
	buffer[1] = 0x08;//8*31 mg?
	buffer[2] = 2; //1/ODR
	err=jet_i2c_write(jet_sensors,ACC_MAG_ADDRESS, buffer, 3);
	if(err)
		return err;
//Enable INT1
	atomic_set(&jet_sensors->irq_ff_status, IRQ_ENABLE_FROM_CPU);
	lsm9ds0_freefall_set_mode(jet_sensors, 1);

	return 0;
}

void freefall_irq_work_func(struct work_struct *work)
{
	int err;
	u8 buffer;
	struct jet_sensors *jet_sensors = container_of(work, struct jet_sensors, freefall_irq_work);

	err=jet_i2c_read(jet_sensors, ACC_MAG_ADDRESS,
			INT_GEN_1_SRC,&buffer, 1);

	if (err)
		printk(KERN_WARNING "[%s:%u] read wrong\n", __FUNCTION__,__LINE__);
	else
   {
		SENSOR_INFO("INT_GEN_1_SRC=%.2x\n",buffer);
#ifdef FREEFALL_API
		if (buffer == (IA_EVENT | AXIS_LOW_INT_EVENT) )
		{
         unsigned char data[16];   // buffer we send over: (4+2+8 = 14 required)

         RI_SENSOR_HANDLE handle = RI_SENSOR_HANDLE_FREEFALL;   // who we are
         RI_DATA_SIZE     payload = sizeof(struct timespec);    // payload size: Single Timespec structure

         struct timespec tspec = current_kernel_time();         // event data is freefall interrupt kernel time

         /* disabling free-fall interrupt once event is received should NOT be done
            by driver. Drivers are slaves, this is app responsibility */
         // ACC_INFO("int time %ld,%ld\n", tspec.tv_sec, tspec.tv_nsec);
		   //	disable_irq_nosync(jet_sensors->pdata->irq_ff);
		   //	atomic_set(&jet_sensors->irq_ff_status, IRQ_DISABLE_FROM_CPU);

         memcpy(data, &handle, sizeof(RI_SENSOR_HANDLE) );
         memcpy(data + sizeof(RI_SENSOR_HANDLE), &payload, sizeof(RI_DATA_SIZE) );
         memcpy(data + sizeof(RI_SENSOR_HANDLE) + sizeof(RI_DATA_SIZE), &tspec, sizeof(struct timespec) );

         SENSOR_INFO("jet_lsm9ds0 *** FreeFall interrupt: [%ld] sec, [%ld] nsec ***\n", tspec.tv_sec, tspec.tv_nsec);

         risensor_irq_data (RI_SENSOR_HANDLE_FREEFALL, data, 
               sizeof(RI_SENSOR_HANDLE) + sizeof(RI_DATA_SIZE) + sizeof(struct timespec) );

		}
#endif
	}
}
#endif
