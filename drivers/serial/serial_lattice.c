// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Lattice Semiconductor Corporation
 */

#include <dm.h>
#include <errno.h>
#include <serial.h>
#include <asm/io.h>
#include <linux/bitops.h>

#define _UART_NO_INTERRUPTS_
#define UART_BUFFER_SIZE			16

#define UART_ERR_WOULD_BLOCK			1
#define UART_ERR_DEVICE_ERROR			2
#define UART_ERR_INVALID_ARGUMENT		3

/* LINE STATUS REGISTER BITS USED IN THE CODE */
#define UART_LSR_RX_RDY_MASK			0x01
#define UART_LSR_PE_MASK			0x04
#define UART_LSR_FE_MASK			0x08
#define UART_LSR_TX_RDY_MASK			0x20

/* INTERRUPT ENABLE REGISTER BITS USED IN THE CODE */
#define UART_IER_RX_INT_MASK			0x01
#define UART_IER_TX_INT_MASK			0x02

/* INTERRUPT IDENTIFICATION REGISTER BITS USED IN THE CODE */
#define UART_IIR_RXRDY				0x04
#define UART_IIR_TXRDY				0x02

#define S1_APB_UART_INST_UART_MEM_MAP_BASE_ADDR	0x40090000
#define CPU_CLOCK_HZ					100000000UL
#define S1_APB_UART_INST_BAUD_RATE		115200

/* UART Device Register-map */
struct lattice_uart_regs {
	u32 rxtx;
	u32 ier;
	u32 iir;
	u32 lcr;
	u32 reserved1;
	u32 lsr;
	u32 reserved2;
	u32 reserved3;
	u32 dlr_lsb;
	u32 dlr_msb;
};

struct lattice_uart_plat {
	const char *name;
	struct lattice_uart_regs *regs;
	u32 sys_clk;
	uchar intrlevel;
	uchar intravail;
	u32 baudrate;
	u32 databits;
	u32 stopbits;
	uchar rxbuffersize;
	uchar txbuffersize;
	uchar blockingtx;
	uchar blockingrx;
	u32 fifoenable;
	uchar rxbuffer[UART_BUFFER_SIZE];
	uchar txbuffer[UART_BUFFER_SIZE];
	uchar rxwriteloc;
	uchar rxreadloc;
	uchar txwriteloc;
	uchar txreadloc;
	u32 timeoutmsecs;
	uchar txdatabytes;
	uchar rxdatabytes;
	u32 errors;
	uchar ier;
};

/*
 * Retrieves a character from the UART
 * Returns 0 if no error.
 * If interrupts are enabled, go through the buffer.
 * Else, interact directly with the uart.
 */
static int lattice_uart_getc(struct udevice *dev)
{
	struct lattice_uart_plat *plat = dev_get_plat(dev);
	struct lattice_uart_regs *regs = plat->regs;

	if (!(readl(&regs->lsr) & UART_LSR_RX_RDY_MASK))
		return -EAGAIN;

	return readl(&regs->rxtx);
}

/*
 * Sends a character over the UART
 * Returns 0 if no error
 */
static int lattice_uart_putc(struct udevice *dev, const char uchar)
{
	struct lattice_uart_plat *plat = dev_get_plat(dev);
	struct lattice_uart_regs *regs = plat->regs;

	if (!(readl(&regs->lsr) & UART_LSR_TX_RDY_MASK))
		return -EAGAIN;

	regs->rxtx = uchar;
	return 0;
}

static int lattice_uart_pending(struct udevice *dev, bool input)
{
	struct lattice_uart_plat *plat = dev_get_plat(dev);
	struct lattice_uart_regs *regs = plat->regs;
	u32 st = readl(&regs->lsr);

	if (input)
		return (st & UART_LSR_RX_RDY_MASK) ? 1 : 0;
	else
		return !(st & UART_LSR_TX_RDY_MASK);
}

static int lattice_uart_probe(struct udevice *dev)
{
	struct lattice_uart_plat *plat = dev_get_plat(dev);

	/*initialize the instance data */
	//plat->regs = S1_APB_UART_INST_UART_MEM_MAP_BASE_ADDR;
	plat->sys_clk = CPU_CLOCK_HZ;
	plat->baudrate = S1_APB_UART_INST_BAUD_RATE;
	plat->databits = 8;
	plat->stopbits = 1;
	plat->blockingtx = 1;

	return 0;
}

static int lattice_uart_of_to_plat(struct udevice *dev)
{
	struct lattice_uart_plat *plat = dev_get_plat(dev);

	plat->regs = (struct lattice_uart_regs *)dev_read_addr(dev);
	if (!plat->regs)
		return -EINVAL;

	return 0;
}

static const struct dm_serial_ops lattice_uart_ops = {
	.putc = lattice_uart_putc,
	.getc = lattice_uart_getc,
	.pending = lattice_uart_pending,
};

static const struct udevice_id lattice_uart_ids[] = {
	{ .compatible = "lattice,uart" },
	{}
};

U_BOOT_DRIVER(lattice_uart) = {
	.name	= "lattice_uart",
	.id	= UCLASS_SERIAL,
	.of_match = lattice_uart_ids,
	.of_to_plat = lattice_uart_of_to_plat,
	.plat_auto	= sizeof(struct lattice_uart_plat),
	.probe = lattice_uart_probe,
	.ops	= &lattice_uart_ops,
};

