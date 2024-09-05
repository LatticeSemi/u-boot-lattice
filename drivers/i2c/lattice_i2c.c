// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2019-2024 by Lattice Semiconductor Corporation
 */

#include <dm.h>
#include <log.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <dm/device_compat.h>
#include <dm/root.h>
#include <i2c.h>
#include <fdtdec.h>
#include <wait_bit.h>
#include "lattice_i2c_regs.h"
#include <lattice/lattice_reg_access.h>

#define I2C_CONTROLLER_DRV_VER "v2.0.0"

#define INT_MODE    0   /* 0, polling mode; 1, INT mode(not support) */

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

/* i2c register set */
struct lattice_i2c_regs {
	u32 control;
	u32 status;
	u32 address;
	u32 data;
	u32 interrupt_status;
	u32 transfer_size;
	u32 slave_mon_pause;
	u32 time_out;
	u32 interrupt_mask;
	u32 interrupt_enable;
	u32 interrupt_disable;
};

struct lattice_i2c_bus {
	const char *instance_name;
	//u32 __iomem *base_address; /* register base */
	u32 base_address;   /* i2c master base address assigned */
	u8 state;       /* IDLE, READ, WRITE */
	u8 addr_mode;   /* 7-bit or 10-bit mode */
	u16 interrupts_en;
	u8 *rx_buff;
	u8 rcv_length;
};

u8 lattice_i2c_init(struct lattice_i2c_bus *this_i2cm, u32 base_addr)
{
	if (!this_i2cm)
		return 1;

	this_i2cm->base_address = base_addr;
	this_i2cm->addr_mode = I2CM_ADDR_7BIT_MODE;
	this_i2cm->state = I2CM_STATE_IDLE;
	this_i2cm->interrupts_en = 0;
	this_i2cm->rx_buff = NULL;
	this_i2cm->rcv_length = 0;

	return 0;
}

u8 lattice_i2c_config(struct lattice_i2c_bus *this_i2cm,
		      u8 i2c_mode, u16 interrupts_en, u16 pre_scaler)
{
	if (!this_i2cm)
		return 1;

	if (interrupts_en != this_i2cm->interrupts_en) {
		this_i2cm->interrupts_en = interrupts_en;
		reg_8b_write(this_i2cm->base_address | REG_INT_ENABLE1,
			     interrupts_en);
		reg_8b_write(this_i2cm->base_address | REG_INT_ENABLE2,
			     interrupts_en >> 8);
	}

    // address mode set, 7-bit/10-bit mode
	if (this_i2cm->addr_mode != i2c_mode) {
		this_i2cm->addr_mode = i2c_mode;
		if (i2c_mode == I2CM_ADDR_10BIT_MODE) {
			reg_8b_modify(this_i2cm->base_address | REG_MODE,
				      I2C_ADDR_MODE, I2C_ADDR_MODE);
		}
	}

	//set the pre_scaler to tune the i2c clock
	reg_8b_modify(this_i2cm->base_address | REG_MODE,
		      I2C_CLK_DIV_HIGH, pre_scaler >> 8);

	return 0;
}

u8 lattice_i2c_read_data(struct lattice_i2c_bus *this_i2cm,
			 u16 address,
			 u8 *data_buffer, u8 read_length)
{
	u8 fifo_status = 0;
	u8 i2c_int2 = 0;
	u8 data_count = 0;
	u8 i2c_status = 1;

	// assert this_i2cm and data_buffer
	if (!this_i2cm)
		return 1;

	// config the register before issue the transaction
	reg_8b_write(this_i2cm->base_address | REG_BYTE_CNT, read_length);

	reg_8b_write(this_i2cm->base_address | REG_SLAVE_ADDR_LOW,
		     address & 0x7F);

	// 10-bit mode
	if (this_i2cm->addr_mode == I2CM_ADDR_10BIT_MODE)
		reg_8b_write(this_i2cm->base_address | REG_SLAVE_ADDR_HIGH,
			     (address >> 7) & 0x03);

	// set to read mode
	reg_8b_modify(this_i2cm->base_address | REG_MODE, I2C_TXRX_MODE,
		      I2C_TXRX_MODE);

	// check i2c status
	if (this_i2cm->state == I2CM_STATE_IDLE) {
		this_i2cm->state = I2CM_STATE_READ;
		reg_8b_write(this_i2cm->base_address | REG_CONFIG, I2C_START);
	} else {
		printf("%s: I2C_STATE_READ Error!\n", __func__);
		return 1;
	}
#if !INT_MODE
	// clear I2C_ERR bits if set
	reg_8b_read(this_i2cm->base_address | REG_INT_STATUS2, &i2c_int2);
	reg_8b_write(this_i2cm->base_address | REG_INT_STATUS2, i2c_int2);

	while (1) {
		reg_8b_read(this_i2cm->base_address | FIFO_STATUS_REG,
			    &fifo_status);

		// if rx fifo not empty, read a byte
		if ((fifo_status & RX_FIFO_EMPTY_MASK) == 0) {
			if (data_count <= read_length)
				reg_8b_read(this_i2cm->base_address |
					    REG_DATA_BUFFER, data_buffer);

			// update the counter and data buffer pointer
			data_buffer++;
			data_count++;
		}

		if (read_length == data_count) {
			i2c_status = 0;
			this_i2cm->state = I2CM_STATE_IDLE;
			break;
		}

		// check for I2C errors
		reg_8b_read(this_i2cm->base_address | REG_INT_STATUS2, &i2c_int2);
		if (i2c_int2 & I2C_ERR) {
			printf("%s: I2C READ: I2C_ERROR\n", __func__);
			// reset the i2c master
			reg_8b_modify(this_i2cm->base_address | REG_CONFIG,
				      I2C_MASTER_RESET, I2C_MASTER_RESET);
			reg_8b_modify(this_i2cm->base_address | REG_CONFIG,
				      I2C_MASTER_RESET, ~I2C_MASTER_RESET);

			i2c_status = 1;
			this_i2cm->state = I2CM_STATE_IDLE;
			break;
		}
	}
#else
	this_i2cm->rx_buff = data_buffer;
	this_i2cm->rcv_length = 0;
#endif
	return i2c_status;
}

u8 i2c_master_repeated_start(struct lattice_i2c_bus *this_i2cm,
			     u16 address,
			     u8 *wr_data_buffer, u8 wr_data_size,
			     u8 *rd_data_buffer, u8 rd_data_size)
{
	u8 data_count = 0;
	u8 i2c_status = 0;
	u8 status = 0;
	u8 i2c_int2 = 0;
	u8 fifo_status = 0;

	if (!this_i2cm)
		return 1;

	// config the register before issue the transaction
	reg_8b_write(this_i2cm->base_address | REG_BYTE_CNT, wr_data_size);

	reg_8b_write(this_i2cm->base_address | REG_SLAVE_ADDR_LOW,
		     address & 0x7F);

	// 10-bit mode
	if (this_i2cm->addr_mode == I2CM_ADDR_10BIT_MODE)
		reg_8b_write(this_i2cm->base_address | REG_SLAVE_ADDR_HIGH,
			     (address >> 8) & 0x03);

	// set to write mode
	reg_8b_modify(this_i2cm->base_address | REG_MODE, I2C_TXRX_MODE, 0);

	// clear status bits
	reg_8b_read(this_i2cm->base_address | REG_INT_STATUS1, &status);
	reg_8b_write(this_i2cm->base_address | REG_INT_STATUS1, status);
	reg_8b_read(this_i2cm->base_address | REG_INT_STATUS2, &i2c_int2);
	reg_8b_write(this_i2cm->base_address | REG_INT_STATUS2, i2c_int2);

	while (data_count < wr_data_size) {
		// check tx fifo level,
		reg_8b_read(this_i2cm->base_address | REG_INT_STATUS1, &status);

		/*
		 * if tx fifo is full, stop loading fifo for now,
		 * resume in interrupt or polling loop
		 */
		if ((status & TX_FIFO_FULL_MASK) != 0)
			break;

		// push the data into tx buffer
		reg_8b_write(this_i2cm->base_address | REG_DATA_BUFFER,
			     *wr_data_buffer);

		// update the counter and data buffer pointer
		wr_data_buffer++;
		data_count++;
	}

	if (this_i2cm->state == I2CM_STATE_IDLE) {
		// start the transaction
		this_i2cm->state = I2CM_STATE_WRITE;
		reg_8b_write(this_i2cm->base_address | REG_CONFIG,
			     I2C_START | I2C_MASTER_REPEATED_START);
	} else {
		return 1;
	}

#if !INT_MODE
	// check the status until transfer done
	while (1) {
		/*
		 * cycle completes when all bytes are transmitted
		 * or a NACK is received or an ERROR is detected
		 */
		reg_8b_read(this_i2cm->base_address | REG_INT_STATUS1, &status);
		if (status & I2C_TRANSFER_COMP_MASK) {
			this_i2cm->state = I2CM_STATE_IDLE;
			break;
		}

		// still have bytes to send
		if (data_count < wr_data_size) {
			// load any additional bytes into tx fifo when it becomes almost empty
			reg_8b_read(this_i2cm->base_address | FIFO_STATUS_REG, &fifo_status);
			// if tx fifo is almost empty
			if (fifo_status & TX_FIFO_AEMPTY_MASK) {
				// push the data into tx buffer
				reg_8b_write(this_i2cm->base_address |
					     REG_DATA_BUFFER, *wr_data_buffer);

				// update the counter and data buffer pointer
				wr_data_buffer++;
				data_count++;
			}
		}

		// check for I2C errors including NACK
		reg_8b_read(this_i2cm->base_address | REG_INT_STATUS2, &i2c_int2);
		if (i2c_int2 & I2C_ERR) {
			// reset the i2c master
			reg_8b_modify(this_i2cm->base_address | REG_CONFIG,
				      I2C_MASTER_RESET, I2C_MASTER_RESET);
			reg_8b_modify(this_i2cm->base_address | REG_CONFIG,
				      I2C_MASTER_RESET, ~I2C_MASTER_RESET);

			i2c_status = 1;
			this_i2cm->state = I2CM_STATE_IDLE;
			break;
		}
	}
#endif
	/* To perform read transaction of a combined I2C transaction */
	fifo_status = 0;
	i2c_int2 = 0;
	data_count = 0;
	i2c_status = 1;

	/* assert this_i2cm and data_buffer */
	if (!this_i2cm || !rd_data_buffer)
		return 1;

	/* configure the register before issue the transaction */
	reg_8b_write(this_i2cm->base_address | REG_BYTE_CNT, rd_data_size);

	reg_8b_write(this_i2cm->base_address | REG_SLAVE_ADDR_LOW,
		     address & 0x7F);

	// 10-bit mode
	if (this_i2cm->addr_mode == I2CM_ADDR_10BIT_MODE)
		reg_8b_write(this_i2cm->base_address | REG_SLAVE_ADDR_HIGH,
			     (address >> 8) & 0x03);

	// set to read mode
	reg_8b_modify(this_i2cm->base_address | REG_MODE,
		      I2C_TXRX_MODE, I2C_TXRX_MODE);

	if (this_i2cm->state == I2CM_STATE_IDLE) {
		this_i2cm->state = I2CM_STATE_READ;
		reg_8b_write(this_i2cm->base_address | REG_CONFIG, I2C_START);
		//reg_8b_write(this_i2cm->base_address | REG_CONFIG,
		//	       I2C_MASTER_REPEATED_START);//0
	} else {
		return 1;
	}
#if !INT_MODE
	// clear I2C_ERR bits if set
	reg_8b_read(this_i2cm->base_address | REG_INT_STATUS2, &i2c_int2);
	reg_8b_write(this_i2cm->base_address | REG_INT_STATUS2, i2c_int2);

	while (1) {
		reg_8b_read(this_i2cm->base_address | FIFO_STATUS_REG, &fifo_status);

		// if rx fifo not empty, read a byte
		if ((fifo_status & RX_FIFO_EMPTY_MASK) == 0) {
			if (data_count <= rd_data_size)
				reg_8b_read(this_i2cm->base_address |
						REG_DATA_BUFFER, rd_data_buffer);

			// update the counter and data buffer pointer
			rd_data_buffer++;
			data_count++;
		}

		if (rd_data_size == data_count) {
			i2c_status = 0;
			this_i2cm->state = I2CM_STATE_IDLE;
			break;
		}

		// check for I2C errors
		reg_8b_read(this_i2cm->base_address | REG_INT_STATUS2, &i2c_int2);
		if (i2c_int2 & I2C_ERR) {
			// reset the i2c master
			reg_8b_modify(this_i2cm->base_address | REG_CONFIG,
				      I2C_MASTER_RESET, I2C_MASTER_RESET);
			reg_8b_modify(this_i2cm->base_address | REG_CONFIG,
				      I2C_MASTER_RESET, ~I2C_MASTER_RESET);

			i2c_status = 1;
			this_i2cm->state = I2CM_STATE_IDLE;
			break;
		}
	}
#else
	this_i2cm->rx_buff = rd_data_buffer;
	this_i2cm->rcv_length = 0;
#endif
	return i2c_status;
}

u8 lattice_i2c_write_data(struct lattice_i2c_bus *this_i2cm,
			  u16 address,
			  u8 *data_buffer, u8 data_size)
{
	u8 data_count = 0;
	u8 i2c_status = 0;
	u8 status = 0;
	u8 i2c_int2 = 0;
	u8 fifo_status = 0;

	if (!this_i2cm)
		return 1;

	/* configure the register before issue the transaction */
	reg_8b_write(this_i2cm->base_address | REG_BYTE_CNT, data_size);

	reg_8b_write(this_i2cm->base_address | REG_SLAVE_ADDR_LOW,
		     address & 0x7F);

	// 10-bit mode
	if (this_i2cm->addr_mode == I2CM_ADDR_10BIT_MODE)
		reg_8b_write(this_i2cm->base_address | REG_SLAVE_ADDR_HIGH,
			     (address >> 8) & 0x03);

	// set to write mode
	reg_8b_modify(this_i2cm->base_address | REG_MODE, I2C_TXRX_MODE, 0);

	// clear status bits
	reg_8b_read(this_i2cm->base_address | REG_INT_STATUS1, &status);
	reg_8b_write(this_i2cm->base_address | REG_INT_STATUS1, status);
	reg_8b_read(this_i2cm->base_address | REG_INT_STATUS2, &i2c_int2);
	reg_8b_write(this_i2cm->base_address | REG_INT_STATUS2, i2c_int2);

	while (data_count < data_size) {
		// check tx fifo level,
		reg_8b_read(this_i2cm->base_address | REG_INT_STATUS1, &status);

		/*
		 * if tx fifo is full, stop loading fifo for now,
		 * resume in interrupt or polling loop
		 */
		if ((status & TX_FIFO_FULL_MASK) != 0)
			break;

		/* push the data into tx buffer */
		reg_8b_write(this_i2cm->base_address | REG_DATA_BUFFER, *data_buffer);

		/* update the counter and data buffer pointer */
		data_buffer++;
		data_count++;
	}

	if (this_i2cm->state == I2CM_STATE_IDLE) {
		// start the transaction
		this_i2cm->state = I2CM_STATE_WRITE;
		reg_8b_write(this_i2cm->base_address | REG_CONFIG, I2C_START);
	} else {
		printf("%s: I2C_STATE_WRITE Error!\n", __func__);
		return 1;
	}
#if !INT_MODE
	// check the status until transfer done
	while (1) {
		/*
		 * cycle completes when all bytes are transmitted
		 * or a NACK is received or an ERROR is detected
		 */
		reg_8b_read(this_i2cm->base_address | REG_INT_STATUS1, &status);
		if (status & I2C_TRANSFER_COMP_MASK) {
			this_i2cm->state = I2CM_STATE_IDLE;
			break;
		}

		// still have bytes to send
		if (data_count < data_size) {
			/*
			 * load any additional bytes into tx fifo
			 * when it becomes almost empty
			 */
			reg_8b_read(this_i2cm->base_address | FIFO_STATUS_REG,
				    &fifo_status);
			/* if tx fifo is almost empty */
			if (fifo_status & TX_FIFO_AEMPTY_MASK) {
				/* push the data into tx buffer */
				reg_8b_write(this_i2cm->base_address |
					     REG_DATA_BUFFER, *data_buffer);

				// update the counter and data buffer pointer
				data_buffer++;
				data_count++;
			}
		}

		/* check for I2C errors including NACK */
		reg_8b_read(this_i2cm->base_address | REG_INT_STATUS2, &i2c_int2);
		if (i2c_int2 & I2C_ERR) {
			printf("%s: I2C WRITE: I2C_ERROR\n", __func__);
			// reset the i2c master
			reg_8b_modify(this_i2cm->base_address | REG_CONFIG,
				      I2C_MASTER_RESET, I2C_MASTER_RESET);
			reg_8b_modify(this_i2cm->base_address | REG_CONFIG,
				      I2C_MASTER_RESET, ~I2C_MASTER_RESET);

			i2c_status = 1;
			this_i2cm->state = I2CM_STATE_IDLE;
			break;
		}
	}
#endif
	return i2c_status;
}

void i2c_master_isr(struct udevice *dev, void *ctx)
{
	u8 i2c_int1 = 0;
	u8 i2c_int2 = 0;

	struct lattice_i2c_bus *this_i2cm = dev_get_priv(dev);

	reg_8b_read(this_i2cm->base_address | REG_INT_STATUS1, &i2c_int1);
	reg_8b_read(this_i2cm->base_address | REG_INT_STATUS2, &i2c_int2);

	reg_8b_write(this_i2cm->base_address | REG_INT_STATUS1, i2c_int1);
	reg_8b_write(this_i2cm->base_address | REG_INT_STATUS2, i2c_int2);

	if (i2c_int1 & I2C_TRANSFER_COMP_MASK)
		this_i2cm->state = I2CM_STATE_IDLE;

	if (i2c_int1 & RX_FIFO_RDY_MASK) {
		if (this_i2cm->state == I2CM_STATE_READ &&
		    this_i2cm->rx_buff) {
			reg_8b_read(this_i2cm->base_address |
				    REG_DATA_BUFFER, this_i2cm->rx_buff);

			this_i2cm->rx_buff++;
			this_i2cm->rcv_length++;
		}
	}

	if (i2c_int2 & I2C_ERR) {
		printf("%s: I2C ISR: I2C_ERROR\n", __func__);
		// reset the i2c master
		reg_8b_modify(this_i2cm->base_address | REG_CONFIG,
			      I2C_MASTER_RESET, I2C_MASTER_RESET);

		this_i2cm->state = I2CM_STATE_IDLE;
	}
}

static int lattice_i2c_xfer(struct udevice *dev, struct i2c_msg *msg,
			    int nmsgs)
{
	struct lattice_i2c_bus *i2c_bus = dev_get_priv(dev);
	int ret = 0;
	int retry = 0;
	struct i2c_msg *message = msg;
	int num_msgs = nmsgs;
	int idx;

	debug("i2c_xfer: %d messages\n", nmsgs);
	for (idx = 0; idx < num_msgs; idx++) {
		debug("i2c_xfer: chip=0x%x, len=0x%x\n", msg->addr, msg->len);
		if (msg->flags & I2C_M_RD) {
			debug("%s: I2C_M_READ\n", __func__);
			ret = lattice_i2c_read_data(i2c_bus, msg->addr, msg->buf,
						    msg->len);
		} else {
			debug("%s: I2C_M_WRITE\n", __func__);
			ret = lattice_i2c_write_data(i2c_bus, msg->addr, msg->buf,
						     msg->len);
		}
		if (ret == -EAGAIN) {
			msg = message;
			nmsgs = num_msgs;
			retry++;
			printf("%s,arbitration lost, retrying:%d\n", __func__,
				   retry);
			continue;
		}
		nmsgs--;
		msg++;
		if (ret) {
			printf("i2c_write: error sending\n");
			return -EREMOTEIO;
		}
	}

	return ret;
}

static int lattice_i2c_of_to_plat(struct udevice *dev)
{
	struct lattice_i2c_bus *i2c_bus = dev_get_priv(dev);

	i2c_bus->base_address = dev_read_addr_ptr(dev);
	if (!i2c_bus->base_address)
		return -EINVAL;

	return 0;
}

int lattice_i2c_probe(struct udevice *bus)
{
	struct lattice_i2c_bus *i2c_bus = dev_get_priv(bus);

	return lattice_i2c_init(i2c_bus, i2c_bus->base_address);
}

static int lattice_i2c_probe_chip(struct udevice *bus, uint chip_addr,
				  uint chip_flags)
{
	struct lattice_i2c_bus *i2c_bus = dev_get_priv(bus);
	u32 tmp;
	int ret;

	ret = lattice_i2c_init(i2c_bus, i2c_bus->base_address);
	if (ret)
		return ret;

	/* Try to read the first location of the chip */
	ret = lattice_i2c_read_data(i2c_bus, chip_addr, (uchar *)&tmp, 0);

	return ret;
}

static const struct dm_i2c_ops lattice_i2c_ops = {
	.xfer = lattice_i2c_xfer,
	.probe_chip = lattice_i2c_probe_chip,
};

static const struct udevice_id lattice_i2c_of_match[] = {
	{ .compatible = "lattice,lattice-i2c-2.0.0"},
	{ /* end of table */ }
};

U_BOOT_DRIVER(lattice_i2c) = {
	.name = "i2c_lattice",
	.id = UCLASS_I2C,
	.of_match = lattice_i2c_of_match,
	.of_to_plat = lattice_i2c_of_to_plat,
	.probe = lattice_i2c_probe,
	.priv_auto  = sizeof(struct lattice_i2c_bus),
	.ops = &lattice_i2c_ops,
};
