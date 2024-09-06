/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2006-2024 by
 * Lattice Semiconductor Corporation
 *
 */

#ifndef I2C_MASTER_REGS_H
#define I2C_MASTER_REGS_H

/*
 *  i2c master register definition
 */

#define REG_DATA_BUFFER				(0x00)

#define REG_SLAVE_ADDR_LOW			(0x04)
#define REG_SLAVE_ADDR_HIGH			(0x08)

#define REG_CONFIG				(0x0c)
#define I2C_MASTER_RX_FIFO_RESET		(0x40)
#define I2C_MASTER_TX_FIFO_RESET		(0x20)
#define I2C_MASTER_REPEATED_START		(0x08)
#define I2C_MASTER_RESET			(0x04)
#define I2C_ABORT				(0x02)
#define I2C_START				(0x01)

#define REG_BYTE_CNT				(0x10)

#define REG_MODE				(0x14)
#define I2C_SPEED_MODE				(0xc0)
#define I2C_ADDR_MODE				(0x20)
#define I2C_ACK_MODE				(0x10)
#define I2C_TXRX_MODE				(0x08)
#define I2C_CLK_DIV_HIGH			(0x03)

#define REG_CLK_DIV_LSB				(0x18)

#define REG_INT_STATUS1				(0x1c)
#define REG_INT_ENABLE1				(0x20)
#define REG_INT_SET1				(0x24)
#define I2C_TRANSFER_COMP_MASK			(0x80)
#define TX_FIFO_FULL_MASK			(0x20)
#define TX_FIFO_AEMPTY_MASK			(0x10)
#define TX_FIFO_EMPTY_MASK			(0x08)
#define RX_FIFO_FULL_MASK			(0x04)
#define RX_FIFO_AFULL_MASK			(0x02)
#define RX_FIFO_RDY_MASK			(0x01)

#define REG_INT_STATUS2				(0x28)
#define REG_INT_ENABLE2				(0x2c)
#define REG_INT_SET2				(0x30)
#define NACK_ERR_MASK				(0x08)
#define ABORT_ACK_MASK				(0x04)
#define ARB_LOST_MASK				(0x02)
#define TIMEOUT_MASK				(0x01)

#define FIFO_STATUS_REG				(0x34)
#define RX_FIFO_EMPTY_MASK			(0x01)

#define SCL_TIMEOUT_REG				(0x38)

#define I2C_ERR					(0x0F)

#endif	/*I2C Master Registers Header File */
