/******************** (C) COPYRIGHT 2011 Recon Instruments ********************
*
* File Name          : i2c_trnasfer.c
* Authors            : Li Chen
* Version            : V 0.1
* Date               : January 2013
* Description        : Recon i2c transfer 
*
********************************************************************************
*
******************************************************************************/
#include "i2c_transfer.h"
inline int transfer_i2c_msg(struct i2c_adapter *adapter, struct i2c_msg *msgs, int msgs_len, int retires_num)
{
	int err;
	int retires_cnt = 0;

	while(1)
	{
		err = i2c_transfer(adapter, msgs, msgs_len);
		retires_cnt++;
		if(err==msgs_len || retires_cnt>retires_num)
			break;
		
		msleep_interruptible(I2C_CHIP_RETRY_DELAY);
	}

	if (err != msgs_len) {
		printk(KERN_ERR "[%s:%u] i2c transfer error\n",__FUNCTION__,__LINE__);
		err = -EIO;
	} else {
		err = 0;
	}

	return err;  //return 0 for normal
}


/*!
 * \brief read data from the i2c chips
 * \param adapter i2c adapter pointer
 * \param chip_address chip address
 * \param reg_address register address
 * \param buf pointer to the reading buffers 
 * \param len length of buffers
 * \return 0 for success others for failure
 * example: recon_i2c_read(adapter, 0x76,0x55, buf, 1); 
 */
int mpu_i2c_read(struct i2c_adapter *adapter, u8 chip_address,
					u8 reg_address, u8 *buf, int len)
{
	struct i2c_msg msgs[] = {
		{
		 .addr = chip_address,
		 .flags = 0,
		 .len = 1,
		 .buf = &reg_address,
		 },
		{
		 .addr = chip_address,
		 .flags = I2C_M_RD,
		 .len = len,
		 .buf = buf,
		 },
	};

	return transfer_i2c_msg(adapter,msgs, 2, I2C_CHIP_RETRIES);
}

/*!
 * \brief Write data to the i2c chips
 * \param adapter i2c adapter pointer
 * \param chip_address chip address
 * \param buf pointer to the writing buffers 
 * \param len length of buffers
 * \return 0 for success others for failure
//example: recon_i2c_write(adapter, 0x76, buf, 2); 
//buf[0] : register address; buf[1] register value
 */
int mpu_i2c_write(struct i2c_adapter *adapter, u8 chip_address,
					u8 *buf, int len)
{
	struct i2c_msg msgs[] = {
		{
		 .addr = chip_address,
		 .flags = 0,
		 .len = len,
		 .buf = buf,
		},
	};

	return transfer_i2c_msg(adapter,msgs, 1, I2C_CHIP_RETRIES);
}

/*!
 * \brief read data from the i2c chips,
 * which will send NACK all the time if the chip not ready
 * \param adapter i2c adapter pointer
 * \param chip_address chip address
 * \param reg_address register address
 * \param buf pointer to the reading buffers 
 * \param len length of buffers
 * example: recon_i2c_read(adapter, 0x11,0x00, buf, 1); 
 */
int mpu_i2c_read_slow(struct i2c_adapter *adapter, u8 chip_address,
					u8 reg_address, u8 *buf, int len)
{
	int err;
	struct i2c_msg msgs[] = {
		{
			.addr = chip_address,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	err=mpu_i2c_write(adapter, chip_address, &reg_address, 1); 
	if(err)
	{
		return err;  //return 0 for normal
	}
	
	return transfer_i2c_msg(adapter, msgs, 1, I2C_CHIP_RETRIES);
}

/*!
 * \brief Write data to the i2c chips
 * \param adapter i2c adapter pointer
 * \param chip_address chip address
 * \param buf pointer to the writing buffers 
 * \param len length of buffers
//example: recon_i2c_write_fast(adapter, 0x76, buf, 2); 
//buf[0] : register address; buf[1] register value
 */
int mpu_i2c_write_fast(struct i2c_adapter *adapter, u8 chip_address,
						u8 *buf, int len)
{
	struct i2c_msg msgs[] = {
		{
		 .addr = chip_address,
		 .flags = 0,
		 .len = len,
		 .buf = buf,
		},
	};
	return transfer_i2c_msg(adapter, msgs, 1, 0);
}