/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Lattice Semiconductor Corporation
 *
 */

#ifndef I2C_MASTER_H
#define I2C_MASTER_H

#define I2C_CONTROLLER_DRV_VER "v2.0.0"

#define INT_MODE	0	/* 0, polling mode; 1, INT mode(not support) */

enum {
	I2CM_STATE_IDLE = 0,
	I2CM_STATE_READ,
	I2CM_STATE_WRITE,
	I2CM_STATE_TIMEOUT,
	I2CM_STATE_ERROR = 0xFF
} i2cm_state;

enum {
	I2CM_ADDR_7BIT_MODE = 7,
	I2CM_ADDR_10BIT_MODE = 10,
} I2CM_ADDR_MODE;

struct i2cm_instance {
	const char *instance_name;
	u32 base_address;	/* i2c master base address assigned */
	u8 state;		/* IDLE, READ, WRITE */
	u8 addr_mode;	/* 7-bit or 10-bit mode */
	u16 interrupts_en;
	u8 *rx_buff;
	u8 rcv_length;
};

/*
 *****************************************************************************
 *
 * uint_8 i2c_master_init(struct i2cm_instance* this_i2cm,
 *                        u32 base_addr)
 *
 * performs i2c master block initialization
 *
 * Note: This function initializes the i2c master block
 *
 *
 * Arguments:
 *    struct i2cm_instance* this_i2cm: i2c master instance
 *    u32 base_addr             : base address of the i2c master
 *
 * Return Value:
 *    int:
 *
 *
 *****************************************************************************
 */
u8 i2c_master_init(struct i2cm_instance *this_i2cm, u32 base_addr);

/*
 *****************************************************************************
 *
 * u8 i2c_master_config(struct i2cm_instance* this_i2cm,
 *                           u8 i2c_mode,
 *                           u8 interrupts_en);
 *
 * configure the i2c master controller
 *
 * Note: This function configurs the i2c master block
 *
 *
 * Arguments:
 *    struct i2cm_instance* this_i2cm: i2c master instance
 *    u32 base_addr             : base address of the i2c master
 *    u8  i2c_mode              : i2c mode: 7 - 7 bit address mode
 *                                               10 - 10 bit address mode
 *    u8  interrupts_en         : enabled interrupt bits
 *    u16 pre_scaler            : calculate and set the prescaler to tune the clock
 *
 * Return Value:
 *    int:
 *
 *
 *****************************************************************************
 */
u8 i2c_master_config(struct i2cm_instance *this_i2cm,
		     u8 i2c_mode, u16 interrupts_en, u16 pre_scaler);

/*
 *****************************************************************************
 *
 * u8 i2c_master_read(struct i2cm_instance* this_i2cm,
 *                    u16 address,
 *                    u8  *data_buffer,
 *                    u8  buffer_size)
 *
 * performs i2c master read operation
 *
 * Note: This function perform the i2c master read operation
 *
 *
 * Arguments:
 *    struct i2cm_instance* this_i2cm: i2c master instance
 *    u16 address               : address of the slave device
 *    u8  read_length           : number of bytes to read
 *    u8  *data_buffer          : pointer to data buffer storing the dare read back
 *
 * Return Value:
 *    int:
 *
 *
 *****************************************************************************
 */
u8 i2c_master_read(struct i2cm_instance *this_i2cm,
		   u16 address,
		   u8 *data_buffer, u8 read_length);

/*
 *****************************************************************************
 *
 * u8 i2c_master_write(struct i2cm_instance* this_i2cm,
 *                          u16 address,
 *                          u8  *data_buffer,
 *                          u8  buffer_size)
 *
 * performs i2c master write operation
 *
 * Note: This function perform the i2c master write operation
 *
 *
 * Arguments:
 *    struct i2cm_instance* this_i2cm: i2c master instance
 *    u16 address               : address of the slave device
 *    u8  buffer_size           : number of bytes to write
 *    u8  *data_buffer          : pointer to data buffer
 *
 * Return Value:
 *    int:
 *
 *
 *****************************************************************************
 */
u8 i2c_master_write(struct i2cm_instance *this_i2cm,
		    u16 address,
		    u8 *data_buffer, u8 buffer_size);

/*
 *****************************************************************************
 *
 * u8 i2c_master_repeated_start(struct i2cm_instance *this_i2cm,
 *			u16 address,
 *			u8 wr_data_size, u8 *wr_data_buffer,
 *			u8 rd_data_size, u8 *rd_data_buffer);
 *
 * performs i2c master repeated start operation
 *
 * Note: This function perform the i2c master repeated start operation
 *
 *
 * Arguments:
 *    struct i2cm_instance* this_i2cm: i2c master instance
 *    u16 address               : address of the slave device
 *    u8  wr_data_size          : number of bytes to write
 *    u8  *wr_data_buffer       : pointer to write data buffer
 *    u8  rd_data_size          : number of bytes to read
 *    u8  *rd_data_buffer       : pointer to read data buffer
 *
 * Return Value:
 *    int:
 *
 *
 *****************************************************************************
 */
u8 i2c_master_repeated_start(struct i2cm_instance *this_i2cm,
			     u16 address,
			     u8 *wr_data_buffer, u8 wr_data_size, 
			     u8 *rd_data_buffer, u8 rd_data_size);

//void i2c_master_isr(void *ctx);

#endif	/*I2C Master Header File */
