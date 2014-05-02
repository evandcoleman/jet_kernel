#include "jet_sensors.h"
#include "jet_lps25h.h"
#define PRESSURE_ADDRESS 0x5D

#define P_WHO_AM_I		(0x0F)
#define RES_CONF		(0x10)	/* resolution */
#define	CTRL_REG1		(0x20)	/* power / ODR control reg */
#define	CTRL_REG2		(0x21)	/* FIFO / boot / single trigger reg */
#define	CTRL_REG3		(0x22)	/* interrupt control reg */
#define	CTRL_REG4		(0x23)	/* interrupt source configuration reg */
#define	PRESS_STATUS	(0x27)	/* press status */
#define	PRESS_OUT_XL	(0x28)	/* press output (3 regs) */
#define	TEMP_OUT_L		(0x2B)	/* temp output (2 regs) */
#define FIFO_CTRL_REG	(0x2E)	/* FIFO CONTROL REGISTER */
#define FIFO_SRC_REG	(0x2F)	/* FIFO SOURCE REGISTER */


#define I2C_AUTO_INCREMENT	(0x80)
#define ALL_ZEROES		(0x00)
#define PM_OFF			(0x00)
#define FIFO_ENABLE		(0x40)
#define ENABLE_ALL_AXES	(0x07)
#define FIFO_STREAM		(0x40)
#define FIFO_BYPASS		(0x00)
#define FIFO_OVRN		(0x40)
#define FIFO_DATA_LEVEL_MASK	(0x1F)

#define P_ID		(0xBD)
#define PM_ON		(0x80)
#define ODR_25		(0x40)
#define ONESHOT		(0x01) /* single conversion */
#define P_AVG8		(0x00)	/* Number of internal pressure average: 8 */
#define P_AVG32		(0x01)
#define P_AVG128	(0x02)
#define P_AVG512	(0x03)
#define T_AVG8		(0x00)	/* Number of internal temperature: 8 */
#define T_AVG16		(0x04)
#define T_AVG32		(0x08)
#define T_AVG64		(0x0C)
#define P_DA		(0x02) /* new pressure ready */
#define T_DA		(0x01) /* new temperature ready */



#define FIFO_MINIMUM_DELAY_TIME		40 //ms
#define DELAY_TIME_THRESHOLD		500 //ms

static u8 pressure_fifo_mode;
static unsigned int pressure_ms_delay;

static int lp225h_pressure_single_trigger(struct jet_sensors *jet_sensors)
{
	u8 buffer[2];
	buffer[0]=CTRL_REG2;
	buffer[1]=ONESHOT;
	return jet_i2c_write(jet_sensors, PRESSURE_ADDRESS , buffer, 2);
}

static int lp225h_pressure_set_fifo(struct jet_sensors *jet_sensors, unsigned char fifo_mode)
{
	u8 buffer[2];
	int err=0;

	SENSOR_INFO("fifo_mode_old=%d,fifo_mode_new=%d\n", 
				pressure_fifo_mode, fifo_mode);

	if(pressure_fifo_mode!=fifo_mode)
	{
		buffer[0]= FIFO_CTRL_REG;
		if(fifo_mode==RI_SENSOR_MODE_FIFO)
			buffer[1]= FIFO_STREAM;
		else
			buffer[1]= FIFO_BYPASS;
		err=jet_i2c_write(jet_sensors, PRESSURE_ADDRESS , buffer, 2);
		if(err==0)
			pressure_fifo_mode=fifo_mode;
	}

	return err;
}

int lp225h_pressure_hw_init(struct jet_sensors *jet_sensors)
{
	int err;
	u8 buffer[6];

	err=jet_i2c_read(jet_sensors, PRESSURE_ADDRESS,
					P_WHO_AM_I,&buffer[0], 1);
	if(err<0)
		return err;
	
	if(buffer[0]!=P_ID)
		return (-ENXIO); // No such device or address

	buffer[0]= CTRL_REG1|I2C_AUTO_INCREMENT;
	buffer[1]= PM_OFF;
	buffer[2]= ALL_ZEROES;
	buffer[3]= ALL_ZEROES;
	buffer[4]= ALL_ZEROES;
	err=jet_i2c_write(jet_sensors,PRESSURE_ADDRESS, buffer, 5);
	if(err)
		return err;

	buffer[0]= RES_CONF;
	buffer[1]= P_AVG512|T_AVG64; //default resolution value
	err=jet_i2c_write(jet_sensors,PRESSURE_ADDRESS, buffer, 2);
	if(err)
		return err;
	//force to enable fifo by default
	pressure_fifo_mode=RI_SENSOR_MODE_CR;
	pressure_ms_delay=1000;
	return lp225h_pressure_set_fifo(jet_sensors,RI_SENSOR_MODE_FIFO);
}

int lp225h_pressure_set_mode(struct jet_sensors *jet_sensors,  unsigned char flag, 
							unsigned char mode, unsigned int ms_delay)
{
	u8 new_mode;
	u8 buffer[3];
	int err=0;
	if(ms_delay<FIFO_MINIMUM_DELAY_TIME)
		new_mode=RI_SENSOR_MODE_CR; //FIFO mode only works for more than 40ms time interval
	else
		new_mode=mode;
	lp225h_pressure_set_fifo(jet_sensors, new_mode);

	if(flag)
	{
		buffer[0]=RES_CONF;
		if(ms_delay<FIFO_MINIMUM_DELAY_TIME)
			buffer[1]=P_AVG32|T_AVG32; //lower resolution in order to fit in high sampling rate
		else
			buffer[1]=P_AVG512|T_AVG64;
		err=jet_i2c_write(jet_sensors,PRESSURE_ADDRESS, buffer, 2);
		if(err)
			return err;

		buffer[0]=CTRL_REG1|I2C_AUTO_INCREMENT;
		if(pressure_fifo_mode==RI_SENSOR_MODE_FIFO){
			buffer[1]= PM_ON|ODR_25;
			buffer[2]= FIFO_ENABLE;
		}
		else{
			buffer[1]= PM_ON;
			buffer[2]= ONESHOT;
		}
		err=jet_i2c_write(jet_sensors,PRESSURE_ADDRESS, buffer, 3);
		if(err)
			return err;
			
		pressure_ms_delay=ms_delay;//update delay time
	}
	else
	{
		buffer[0]=CTRL_REG1;
		buffer[1]= PM_OFF;
		err=jet_i2c_write(jet_sensors,PRESSURE_ADDRESS, buffer, 2);
	}
	return err;
}

int lp225h_pressure_get_data(struct jet_sensors *jet_sensors, u8 *sensor_data_ptr, unsigned int length)
{
	int err;
	u8 buffer;
	int len, size,fifo_event_count;
	u8 *data;
#ifdef PRESSURE_INT_ALIGNMENT
	int i,j;
	u8 pressure_data[LPS_FIFO_LENGTH_MAX];
#endif
	u8 fifo_mode= pressure_fifo_mode;
	unsigned int ms_delay= pressure_ms_delay;
	data=sensor_data_ptr;
	len=length;

   // validate buffer; Must have room for at least 1 event 
	if(len<LPS_ONEEVENT_SIZE)   
	{
		printk(KERN_DEBUG "%s: buffer length too small %d\n", __func__,len);
		return 0;
	}

	if(fifo_mode==RI_SENSOR_MODE_FIFO)
	{
		err=jet_i2c_read(jet_sensors, PRESSURE_ADDRESS,
			FIFO_SRC_REG,&buffer, 1);
#ifdef SENSOR_DEBUG_VERBOSE
		SENSOR_INFO("FIFO_SRC_REG=%.2x\n",buffer);
#endif
		if(err)
			return err;

		if(buffer&FIFO_OVRN){
#ifdef PRESSURE_INT_ALIGNMENT
			fifo_event_count=LPS_FIFO_DEEP;
#endif
			len=LPS_DATA_LENGTH_MAX;
		}
		else
		{
			fifo_event_count=(buffer&FIFO_DATA_LEVEL_MASK);

			if(fifo_event_count==0)
			{
				return 0;//FIFO empty
			}
			else
			{
				len= fifo_event_count*LPS_ONEEVENT_SIZE;
			}
		}
		size= (len <= length) ? len : length;
#ifdef PRESSURE_INT_ALIGNMENT
//readjust fifo event counter to report to upper level; 
//length>>2 equal to length/LPS_ONEEVENT_SIZE(4)
		fifo_event_count= (len <= length) ? fifo_event_count : (length>>2);
		err=jet_i2c_read(jet_sensors, PRESSURE_ADDRESS,
			(PRESS_OUT_XL|I2C_AUTO_INCREMENT), pressure_data, fifo_event_count*LPS_ONEEVENT_RAW_SIZE);
		if(err)
			return err;
//copy data to sensor data buffer
		memset(data, 0, size);
		for(i=0;i<fifo_event_count;i++)
		{
			j=3*i;
			//4*i==3*i+i==j+i;
			memcpy(data+j+i, pressure_data+j, LPS_ONEEVENT_RAW_SIZE);
		}
#else
		err=jet_i2c_read(jet_sensors, PRESSURE_ADDRESS,
			(PRESS_OUT_XL|I2C_AUTO_INCREMENT), data, size);
#endif
	}
	else// none fifo mode
	{
		size=LPS_ONEEVENT_SIZE;
		
		if(ms_delay>=DELAY_TIME_THRESHOLD)//pressure data needs newest data if sampling rate low
		{
			err=lp225h_pressure_single_trigger(jet_sensors);
			if(err)
				return err;
			msleep(40);//sleep at least 40ms
		}

		err=jet_i2c_read(jet_sensors, PRESSURE_ADDRESS,
			PRESS_STATUS,&buffer, 1);
#if 0
		SENSOR_INFO("PRESS_STATUS=%.2x\n",buffer);
#endif
		if(err)
			return err;
		if( (buffer&(P_DA|T_DA)) != (P_DA|T_DA))
		{
			printk("pressure data not ready\n");
			return 0;
		}

		err=jet_i2c_read(jet_sensors, PRESSURE_ADDRESS,
			(PRESS_OUT_XL|I2C_AUTO_INCREMENT), data, LPS_ONEEVENT_RAW_SIZE);
#ifdef PRESSURE_INT_ALIGNMENT
//pad the fourth byte to zero
		*(data+3)=0;
#endif
		if(ms_delay<DELAY_TIME_THRESHOLD)
			err=lp225h_pressure_single_trigger(jet_sensors);//trigger next single conversion
	}
	//TODO: Pressure= P_raw/4096. (P_raw>>12 to get integer)
	if(err==0) {
		if((data[0] | data[1] | data[2]) == 0) {
			printk(KERN_ERR "Pressure data is 0 (zero)\n");
			return 0;
		}
		return size;
	}
	return err;
}

//manufacture test prupose
int lp225h_temperature_get_data(struct jet_sensors *jet_sensors, short *temp_ptr)
{
	int err;
	u8 buffer;
	u8 data[2];
	short *ptr;
	//TODO: lp225h_pressure_set_mode(et_sensors, 1, 0, 1000);
	err=lp225h_pressure_single_trigger(jet_sensors);
	if(err)
		return err;
	msleep(40);//sleep at least 40ms
	err=jet_i2c_read(jet_sensors, PRESSURE_ADDRESS,
		PRESS_STATUS,&buffer, 1);
#if 0
		SENSOR_INFO("PRESS_STATUS=%.2x\n",buffer);
#endif
	if(err)
		return err;
	if( (buffer&(P_DA|T_DA)) != (P_DA|T_DA))
	{
		printk("temperature data not ready\n");
		return 0;
	}

	err=jet_i2c_read(jet_sensors, PRESSURE_ADDRESS,
	(TEMP_OUT_L|I2C_AUTO_INCREMENT), data, 2);
	if(err)
		return err;

	ptr=(short*)data;
	//T(degC) =42.5 + (TEMP_OUT/480)
	*temp_ptr= 425+ (*ptr)/48; //ignore decimal points
	*temp_ptr /= 10;
	return 1; //1 data ready
}
