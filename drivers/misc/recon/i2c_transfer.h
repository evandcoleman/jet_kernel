#ifndef __I2C_TRANSFER_H__
#define __I2C_TRANSFER_H__
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/delay.h>
#define I2C_CHIP_RETRY_DELAY         1
#define I2C_CHIP_RETRIES             5
inline int transfer_i2c_msg(struct i2c_adapter *adapter, struct i2c_msg *msgs, int msgs_len, int retires_num);
int mpu_i2c_read(struct i2c_adapter *adapter, u8 chip_address,
					u8 reg_address, u8 *buf, int len);

int mpu_i2c_write(struct i2c_adapter *adapter, u8 chip_address,
					u8 *buf, int len);

int mpu_i2c_read_slow(struct i2c_adapter *adapter, u8 chip_address,
					u8 reg_address, u8 *buf, int len);

int mpu_i2c_write_fast(struct i2c_adapter *adapter, u8 chip_address,
						u8 *buf, int len);

#endif