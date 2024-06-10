// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2019-2024 by Lattice Semiconductor Corporation
 */

#include "lattice_tse.h"
#include <lattice/lattice_sgdma.h>
#include <lattice/lattice_reg_access.h>
#include <dm.h>
#include <errno.h>
#include <fdt_support.h>
#include <log.h>
#include <memalign.h>
#include <miiphy.h>
#include <net.h>
#include <asm/cache.h>
#include <asm/global_data.h>
#include <linux/dma-mapping.h>
#include <asm/io.h>

struct lattice_tse_plat {
	const char *instance_name;
	void __iomem *tse_base_address;
	void __iomem *sgdma_base_address;
	u32 mm2s_address;
	u32 s2mm_address;
	u32 mm2s_bd_address;
	u32 s2mm_bd_address;
};

static struct sgdma_instance_t sgdma_core;

/**
 * @brief	This function is used to configure to receive all address
 *		frame and enabling the tsemac at speed of 10/100/1000 Mbps.
 * @param	handle	handler for the tsemac.
 * @return	status for initialization
 */
u8 ethernet_init(struct tsemac_handle_t *handle)
{
	if (handle->addr) {
		struct tsemac_reg_type_t *tsemac =
			(struct tsemac_reg_type_t *)(handle->addr);

		/* configure maximum packet size register */
		tsemac->max_packet_size = MAX_PACKET_SIZE;
		tsemac->ipg = IPG_TIME;

		/*
		 * configure Rx MAC to receive multicast and broadcast
		 * frame received
		 */
		tsemac->tx_rx_ctrl |= handle->tx_rx_ctrl_var;
		ethernet_set_speed(handle);
	} else {
		return 1;
	}

	return 0;
}

/**
 * @brief	This function is used to transmit the frame by copying
 *		src_packet buffer into dest_packet and same packet is
 *		transmitted to tsemac.
 * @param	handle handler for the tsemac.
 *		src_packet buffer containing the packet to transmit to tsemac.
 *		dest_packet src_packet to copy in dest_packet for user.
 * @return	void
 */
void ethernet_packet_handle(struct tsemac_handle_t *handle, u32 *src_packet,
			    u32 *dest_packet)
{
	int count;

	for (count = 0; count < handle->frame_length; count++)
		dest_packet[count] = src_packet[count];
}

/**
 * @brief	This function is used to set the mac address for tsemac.
 * @param	handle handler for the tsemac.
 * @return	status of setting MAC address.
 */
u8 ethernet_set_mac_address(struct tsemac_handle_t *handle)
{
	if (handle->addr && handle->mac_upper && handle->mac_lower) {
		struct tsemac_reg_type_t *tsemac =
			(struct tsemac_reg_type_t *)(handle->addr);
		/*
		 * storing the MAC address for first 4 byte in register
		 * MAC address0
		 */
		tsemac->mac_addr0 = handle->mac_upper;
		/*
		 * storing the MAC address for last 2 byte in register
		 * MAC address1
		 */
		tsemac->mac_addr1 = handle->mac_lower;
	} else {
		return 1;
	}

	return 0;
}

/**
 * @brief	This function is used to read the mac address for tsemac.
 * @param	handle handler for the tsemac.
 * @return	status of MAC address read.
 */
u8 ethernet_get_mac_address(struct tsemac_handle_t *handle)
{
	if (handle->addr) {
		struct tsemac_reg_type_t *tsemac =
			(struct tsemac_reg_type_t *)(handle->addr);
		/*
		 * read first 4 byte of MAC address from address word 0
		 * register
		 */
		handle->mac_upper = tsemac->mac_addr0;
		/*
		 * read last 2 byte of MAC address from address word 1
		 * register
		 */
		handle->mac_lower = tsemac->mac_addr1;
	} else {
		return 1;
	}

	return 0;
}

/**
 * @brief	This function is used to set for multicast address for tsemac.
 * @param	handle handler for the tsemac.
 * @return	status of setting the multicast address.
 */
u8 ethernet_set_multicast_address(struct tsemac_handle_t *handle)
{
	if (handle->addr && handle->multicast_upper &&
	    handle->multicast_lower) {
		struct tsemac_reg_type_t *tsemac =
			(struct tsemac_reg_type_t *)(handle->addr);
		/*
		 * storing the first 4 byte of the 64 bit hash in mutlicast
		 * table word 0 register
		 */
		tsemac->multi_cast0 = handle->multicast_upper;
		/*
		 * storing the last 4 byte of the 64 bit hash in mutlicast
		 * table word 1 register
		 */
		tsemac->multi_cast1 = handle->multicast_lower;
	} else {
		return 1;
	}

	return 0;
}

/**
 * @brief	This function is used to get for multicast address.
 * @param	handle handler for the tsemac.
 * @return	status of multicast address read.
 */
u8 ethernet_get_multicast_address(struct tsemac_handle_t *handle)
{
	if (handle->addr) {
		struct tsemac_reg_type_t *tsemac =
			(struct tsemac_reg_type_t *)(handle->addr);
		/*
		 * read first 4 byte of the 64 bit hash from multicast table
		 * word 0 register
		 */
		handle->multicast_upper = tsemac->multi_cast0;
		/*
		 * read last 4 byte of the 64 bit hash from multicast table
		 * word 1 register
		 */
		handle->multicast_lower = tsemac->multi_cast1;
	} else {
		return 1;
	}

	return 0;
}

/**
 * @brief	This function is used to set the speed for tsemac at 10/100/1000 Mbps.
 * @param	handle handler for the tsemac.
 * @return	success or failure status based on condition.
 */
u8 ethernet_set_speed(struct tsemac_handle_t *handle)
{
	if (handle->addr) {
		struct tsemac_reg_type_t *tsemac =
			(struct tsemac_reg_type_t *)(handle->addr);

		if (handle->speed_mode == one_g_mode) {
			/* set full duplex mode */
			tsemac->tx_rx_ctrl &= (~(0x1 << SET_FULL_DUPLEX_MODE));
			/* set into 1G mode */
			tsemac->mode_reg |= (0x1 << SPEED_1G);
		} else if (handle->speed_mode == fast_half_duplex_mode) {
			/* set half duplex mode */
			tsemac->tx_rx_ctrl |= (0x1 << SET_HALF_DUPLEX_MODE);
			/* set into 10 or 100 mbps */
			tsemac->mode_reg &= (~(0x1 << SPEED_10_OR_100_MBPS));
		} else {
			/* set full duplex mode */
			tsemac->tx_rx_ctrl &= (~(0x1 << SET_FULL_DUPLEX_MODE));
			/* set into 10 or 100 mbps */
			tsemac->mode_reg &= (~(0x1 << SPEED_10_OR_100_MBPS));
		}
		/* enable transmit and receive mode */
		tsemac->mode_reg |= handle->enable_tx_mac |
				    handle->enable_rx_mac;
	} else {
		return 1;
	}

	return 0;
}

/**
 * @brief	This function is used to read the statistics counter register.
 * @param	handle handler for the tsemac.
 *		offset enum for statistics counter register.
 * @return	val after reading the statistics counter register.
 * Note		This API could not be tested due to IP limitation for
 *		Release 2023.2
 */
u64 ethernet_statistics_counter_register_read(struct tsemac_handle_t *handle,
					      u8 offset)
{
	u64 val;
	void __iomem *reg0 = handle->addr + (offset * 4);
	void __iomem *reg1 = handle->addr + (offset + 1) * 4;

	val = ((u64)readl(reg1) << 32) | readl(reg0);

	return val;
}

/**
 * @brief	This function is used to read the Transmit and receive status
 *		register.
 * @param	handle handler for the tsemac.
 * @return	status register value after reading.
 */
u32 ethernet_tx_rx_status_reg_read(struct tsemac_handle_t *handle)
{
	struct tsemac_reg_type_t *tsemac =
		(struct tsemac_reg_type_t *)(handle->addr);

	return tsemac->tx_rx_status;
}

/**
 * @brief	This function is used to read the mode register.
 * @param	handle handler for the tsemac.
 * @return	mode register value after reading.
 */
u32 ethernet_mode_reg_read(struct tsemac_handle_t *handle)
{
	struct tsemac_reg_type_t *tsemac =
		(struct tsemac_reg_type_t *)(handle->addr);

	return tsemac->mode_reg;
}

/**
 * @brief	This function is used to set the particular bit in Transmit
 *		and Receive control register.
 * @param	handle handler for the tsemac.
 * @return	success or failure status based on condition.
 */
u8 ethernet_tx_rx_control_reg_set(struct tsemac_handle_t *handle, u8 bit_pos)
{
	struct tsemac_reg_type_t *tsemac =
		(struct tsemac_reg_type_t *)(handle->addr);

	if (!tsemac)
		return 1;

	tsemac->tx_rx_ctrl |= 0x1 << bit_pos;

	return 0;
}

static int lattice_tse_send(struct udevice *dev, void *packet, int length)
{
	int i, j = 0;
	u32 ret;
	u32 status = 0;
	u32 *write_packet = (u32 *)packet;
	struct lattice_tse_plat *plat = dev_get_plat(dev);

	for (i = 0; i <= length / 4; i++) {
		reg_32b_write((void *)(plat->mm2s_address + j),
			      write_packet[i]);
		j += 4;
	}

	sgdma_core.base_addr = plat->sgdma_base_address;
	sgdma_core.buffer = (u32 *)plat->mm2s_address;
	sgdma_core.num_of_desc = 1;
	sgdma_core.blocking_mm2s = TRUE;
	sgdma_core.per_desc_length = length;

	ret = mm2s_buf_desc_dma(&sgdma_core);
	if (ret) {
		sgdma_reset(&sgdma_core, S2MM_RESET);
		debug("SGDMA write fail\r\n");
		return -EIO;
	}

	status = get_mm2s_bd_status(0);
	if (!(status & 0x80000000)) {
		sgdma_reset(&sgdma_core, MM2S_RESET);
		debug("Failed: SGDMA write status = %08X\n", status);
		return -EAGAIN;
	}
	debug("Success: SGDMA write status = %08X\n", status);

	return status;
}

static int lattice_tse_recv(struct udevice *dev, int flags, uchar **packetp)
{
	u32 ret;
	u16 readbytes = 0;
	u32 status = 0;
	struct lattice_tse_plat *plat = dev_get_plat(dev);

	sgdma_core.buffer = (u32 *)plat->s2mm_address;
	sgdma_core.per_desc_length = 8192;

	ret = s2mm_buf_desc_dma(&sgdma_core);
	if (ret) {
		sgdma_reset(&sgdma_core, S2MM_RESET);
		debug("SGDMA read fail\r\n");
		return -EAGAIN;
	}

	status = get_s2mm_bd_status(0);
	if (((status & 0x20000000) != 0) || ((status & 0x2000FFFF) == 0)) {
		debug("Failed: S2MM read status  = %08X\n", status);
		sgdma_reset(&sgdma_core, S2MM_RESET);
		return -EAGAIN;
	}

	readbytes = status & 0xFFFF;
	debug("Success: S2MM read status  = %08X, readbyte = %d\n",
	      status, readbytes);

	debug("Dump received bytes:\n");
	for (int i = 0; i < readbytes / 4; i++)
		debug("0x%x ", sgdma_core.buffer[i]);
	debug("\n");

	*packetp = memalign(ARCH_DMA_MINALIGN, MAX_PACKET_SIZE);
	if (!*packetp) {
		debug("%s: error allocating packetp\n", __func__);
		return -ENOMEM;
	}

	memcpy(*packetp, (char *)sgdma_core.buffer, readbytes);

	return readbytes;
}

static int lattice_tse_start(struct udevice *dev)
{
	static struct tsemac_handle_t tse_core;
	struct lattice_tse_plat *plat = dev_get_plat(dev);

	tse_core.addr = plat->tse_base_address;
	tse_core.speed_mode = one_g_mode;
	tse_core.tx_rx_ctrl_var = RCV_SHORT_FRAME | RCV_PAUSE_FRAME |
				  RCV_MULTICAST | RCV_BROADCAST |
				  RCV_ALL_ADDR_FRAME;
	tse_core.enable_rx_mac = ENABLE_RX_MODE;
	tse_core.enable_tx_mac = ENABLE_TX_MODE;
	tse_core.mac_upper = 0x12EFCDAA;
	tse_core.mac_lower = 0x5634;

	if (ethernet_set_mac_address(&tse_core))
		return -EIO;

	if (ethernet_init(&tse_core))
		return -EAGAIN;

	sgdma_core.mm2s_bd_addr = plat->mm2s_bd_address;
	sgdma_core.s2mm_bd_addr = plat->s2mm_bd_address;
	sgdma_core.base_addr = plat->sgdma_base_address;
	sgdma_core.buffer = (u32 *)plat->s2mm_address;
	sgdma_core.num_of_desc = 1;
	sgdma_core.per_desc_length = 64;
	sgdma_core.blocking_s2mm = TRUE;
	sgdma_init(&sgdma_core, plat->sgdma_base_address, 1);

	return 0;
}

static int lattice_tse_free_pkt(struct udevice *dev, uchar *packet,
				int length)
{
	/*
	 * TODO: Suppose to free packet buffer after received package
	 * processed.
	 */
	return 0;
}

static void lattice_tse_stop(struct udevice *dev)
{
	/* Disable the hardware or reset */
	sgdma_reset(&sgdma_core, MM2S_RESET);
	sgdma_reset(&sgdma_core, S2MM_RESET);
}

static int lattice_tse_write_hwaddr(struct udevice *dev)
{
	/*
	 * Need to mask out TX and RX operation flag to prevent
	 * MAC programming conflict
	 */
	static struct tsemac_handle_t tse_core;

	tse_core.mac_upper = 0x12EFCDAA;
	tse_core.mac_lower = 0x5634;
	ethernet_set_mac_address(&tse_core);

	return 0;
}

static const struct eth_ops lattice_tse_ops = {
	.start		= lattice_tse_start,
	.send		= lattice_tse_send,
	.recv		= lattice_tse_recv,
	.free_pkt	= lattice_tse_free_pkt,
	.stop		= lattice_tse_stop,
	.write_hwaddr	= lattice_tse_write_hwaddr,
};

static const struct udevice_id lattice_tse_ids[] = {
	{ .compatible = "lattice,lattice-tse-1.5.0"},
	{}
};

static int lattice_tse_probe(struct udevice *dev)
{
	struct lattice_tse_plat *plat = dev_get_plat(dev);
	int ret;

	plat->tse_base_address = dev_read_addr_ptr(dev);
	if (!plat->tse_base_address)
		return -EINVAL;

	plat->sgdma_base_address = dev_read_addr_name_ptr(dev, "sgdma_base");
	if (!plat->sgdma_base_address)
		plat->sgdma_base_address = (void *)SGDMA_BASE;

	ret = dev_read_u32(dev, "mm2s_address", &plat->mm2s_address);
	if (ret)
		plat->mm2s_address = MM2S_ADDR;

	ret = dev_read_u32(dev, "s2mm_address", &plat->s2mm_address);
	if (ret)
		plat->s2mm_address = S2MM_ADDR;

	return 0;
}

U_BOOT_DRIVER(lattice_tse) = {
	.name	= "lattice_tse",
	.id	= UCLASS_ETH,
	.of_match = lattice_tse_ids,
	.ops	= &lattice_tse_ops,
	.plat_auto	= sizeof(struct lattice_tse_plat),
	.probe	= lattice_tse_probe,
};
