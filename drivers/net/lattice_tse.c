// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 Lattice Semiconductor Corporation
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

#define NUM_OF_MM2S_DESC 1
#define NUM_OF_S2MM_DESC 1

struct lattice_tse_plat {
	const char *instance_name;
	unsigned int tse_base_address;
	unsigned int sgdma_base_address;
	unsigned int mm2s_bd_address;
	unsigned int mm2s_address;
	unsigned int s2mm_bd_address;
	unsigned int s2mm_address;
};

struct sgdma_instance_t sgdma_core;

u8 mm2s_tx_data[8192] = {0};
u8 s2mm_rx_data[8192] = {0};
union mm2s_desc_t mm2s_bd_address[NUM_OF_MM2S_DESC] = {0};
union s2mm_desc_t s2mm_bd_address[NUM_OF_S2MM_DESC] = {0};
unsigned int mm2s_buffer_address[NUM_OF_MM2S_DESC] = {0};
unsigned int s2mm_buffer_address[NUM_OF_S2MM_DESC] = {0};
unsigned int mm2s_desc_length[NUM_OF_MM2S_DESC] = {sizeof(mm2s_tx_data)};
unsigned int s2mm_desc_length[NUM_OF_S2MM_DESC] = {sizeof(s2mm_rx_data)};

unsigned char ethernet_init(struct tsemac_handle_t *handle)
{
	if (!handle)
		return FAILURE;

	struct tsemac_reg_type_t *tsemac = (struct tsemac_reg_type_t *)(handle->adr);

	ethernet_disable_tx_rx_mac(handle);
	tsemac->max_packet_size = MAX_PACKET_SIZE;
	tsemac->ipg = IPG_TIME;
	tsemac->tx_rx_ctrl |= handle->tx_rx_ctrl_var;
	ethernet_set_speed(handle);
	ethernet_set_mac_address(handle);
	ethernet_enable_tx_rx_mac(handle);

	return SUCCESS;
}

unsigned char ethernet_set_mac_address(struct tsemac_handle_t *handle)
{
	if (!handle)
		return FAILURE;

	struct tsemac_reg_type_t *tsemac = (struct tsemac_reg_type_t *)(handle->adr);

	tsemac->mac_addr0 = handle->mac_upper;
	tsemac->mac_addr1 = handle->mac_lower;

	return SUCCESS;
}

unsigned char ethernet_get_mac_address(struct tsemac_handle_t *handle)
{
	if (!handle)
		return FAILURE;

	struct tsemac_reg_type_t *tsemac = (struct tsemac_reg_type_t *)(handle->adr);

	handle->mac_upper = tsemac->mac_addr0;
	handle->mac_lower = tsemac->mac_addr1;

	return SUCCESS;
}

unsigned char ethernet_set_speed(struct tsemac_handle_t *handle)
{
	if (!handle)
		return FAILURE;

	struct tsemac_reg_type_t *tsemac = (struct tsemac_reg_type_t *)(handle->adr);

	if (handle->speed_mode == one_g_mode) {
		tsemac->tx_rx_ctrl &= (~(1 << SET_FULL_DUPLEX_MODE));
		tsemac->mode_reg |= (1 << SPEED_1G);
	} else if (handle->speed_mode == fast_half_duplex_mode) {
		tsemac->tx_rx_ctrl |= (1 << SET_HALF_DUPLEX_MODE);
		tsemac->mode_reg &= (~(1 << SPEED_10_OR_100_MBPS));
	} else {
		tsemac->tx_rx_ctrl &= (~(1 << SET_FULL_DUPLEX_MODE));
		tsemac->mode_reg &= (~(1 << SPEED_10_OR_100_MBPS));
	}

	return SUCCESS;
}

unsigned char ethernet_enable_tx_rx_mac(struct tsemac_handle_t *handle)
{
	if (!handle)
		return FAILURE;

	struct tsemac_reg_type_t *tsemac = (struct tsemac_reg_type_t *)(handle->adr);

	tsemac->mode_reg |= (TX_EN | RX_EN);

	return SUCCESS;
}

unsigned char ethernet_disable_tx_rx_mac(struct tsemac_handle_t *handle)
{
	if (!handle)
		return FAILURE;

	struct tsemac_reg_type_t *tsemac = (struct tsemac_reg_type_t *)(handle->adr);

	tsemac->mode_reg &= ~(TX_EN | RX_EN);

	return SUCCESS;
}

unsigned int ethernet_tx_rx_status_reg_read(struct tsemac_handle_t *handle)
{
	if (!handle)
		return FAILURE;

	struct tsemac_reg_type_t *tsemac = (struct tsemac_reg_type_t *)(handle->adr);

	tsemac->mode_reg |= (TX_EN | RX_EN);

	return SUCCESS;
}

unsigned int ethernet_mode_reg_read(struct tsemac_handle_t *handle)
{
	struct tsemac_reg_type_t *tsemac = (struct tsemac_reg_type_t *)(handle->adr);

	return tsemac->mode_reg;
}

unsigned char ethernet_tx_rx_control_reg_set(struct tsemac_handle_t *handle, unsigned char bit_pos)
{
	if (!handle)
		return FAILURE;

	struct tsemac_reg_type_t *tsemac = (struct tsemac_reg_type_t *)(handle->adr);

	tsemac->tx_rx_ctrl |= 1 << bit_pos;

	return SUCCESS;
}

static int lattice_tse_send(struct udevice *dev, void *packet, int length)
{
	int i = 0;
	unsigned int ret;
	unsigned int status = 0;
	u8 *write_packet = (u8 *)packet;

	for (i = 0; i <= length; i++)
		mm2s_tx_data[i] = write_packet[i];

	ret = mm2s_buf_desc_dma(&sgdma_core);
	if (ret == FAILURE) {
		sgdma_reset_mm2s(&sgdma_core);
		printf("SGDMA write fail\r\n");
		return -EIO;
	}

	status = get_mm2s_bd_status(&sgdma_core, 0);
	if ((status & 0x80000000) == 0) {
		sgdma_reset_mm2s(&sgdma_core);
		printf("Failed: SGDMA write status = %08X\r\n", status);
		return -EAGAIN;
	}

	printf("Success: SGDMA write status = %08X\r\n", status);

	return 0;
}

static int lattice_tse_recv(struct udevice *dev, int flags, uchar **packetp)
{
	u32 ret;
	unsigned short readbytes = 0;
	unsigned int status = 0;

	ret = s2mm_buf_desc_dma(&sgdma_core);
	if (ret == FAILURE) {
		sgdma_reset_s2mm(&sgdma_core);
		printf("SGDMA read fail\r\n");
		readbytes = 0;
		return -EAGAIN;
	}

	status = get_s2mm_bd_status(&sgdma_core, 0);
	if (((status & 0x20000000) != 0) || ((status & 0x2000FFFF) == 0)) {
		printf("Failed: S2MM read status  = %08X\r\n", status);
		sgdma_reset_s2mm(&sgdma_core);
		readbytes = 0;
		return -EAGAIN;
	}

	readbytes = status & 0xFFFF;
	printf("Success: S2MM read status  = %08X, readbyte = %d\r\n",
	       status, readbytes);

	*packetp = memalign(ARCH_DMA_MINALIGN, MAX_PACKET_SIZE);
	if (*packetp == 0) {
		printf("%s: error allocating packetp\n", __func__);
		return -ENOMEM;
	}

	memcpy(*packetp, s2mm_rx_data, readbytes);

	printf("Dump received bytes:\n");
	for (int i = 0; i < readbytes; i++)
		printf("0x%2x ", s2mm_rx_data[i]);

	printf("\n");

	return readbytes;
}

static int lattice_tse_start(struct udevice *dev)
{
	static struct tsemac_handle_t tse_core;
	struct lattice_tse_plat *plat = dev_get_plat(dev);

	tse_core.adr = plat->tse_base_address;
	tse_core.speed_mode = one_g_mode;
	tse_core.tx_rx_ctrl_var = (RCV_SHORT_FRAME    |
				   RCV_PAUSE_FRAME    |
				   RCV_MULTICAST      |
				   RCV_BROADCAST      |
				   RCV_DISCARD_FCS    |
				   RCV_ALL_ADDR_FRAME);
	tse_core.mac_upper = 0x12EFCDAA;
	tse_core.mac_lower = 0x5634;
	if (ethernet_init(&tse_core) != SUCCESS)
		return -EAGAIN;

	mm2s_buffer_address[0] = (unsigned int)mm2s_tx_data;
	s2mm_buffer_address[0] = (unsigned int)s2mm_rx_data;

	sgdma_core.base_addr = plat->sgdma_base_address;
	sgdma_core.axi4_mm_data_width = 32;
	sgdma_core.num_of_mm2s_desc = NUM_OF_MM2S_DESC;
	sgdma_core.num_of_s2mm_desc = NUM_OF_S2MM_DESC;
	sgdma_core.mm2s_bd_addr = mm2s_bd_address;
	sgdma_core.s2mm_bd_addr = s2mm_bd_address;
	sgdma_core.mm2s_buffer_address = mm2s_buffer_address;
	sgdma_core.s2mm_buffer_address = s2mm_buffer_address;
	sgdma_core.per_mm2s_desc_length = mm2s_desc_length;
	sgdma_core.per_s2mm_desc_length = s2mm_desc_length;
	sgdma_core.fp = 0;
	sgdma_core.blocking_s2mm = true;
	sgdma_core.blocking_mm2s = true;
	sgdma_init(&sgdma_core, plat->sgdma_base_address);

	return 0;
}

static int lattice_tse_free_pkt(struct udevice *dev, uchar *packet,
				int length)
{
	// Suppose to free packet buffer after received packaget processed. TBD
	return 0;
}

static void lattice_tse_stop(struct udevice *dev)
{
	// Disable the hardware or reset.
	sgdma_reset(&sgdma_core);
}

static int lattice_tse_write_hwaddr(struct udevice *dev)
{
	// Need to mask out TX and RX operation flag to prevent MAC programming conflict
	static struct tsemac_handle_t tse_core;

	tse_core.mac_upper = 0x12EFCDAA;
	tse_core.mac_lower = 0x5634;
	ethernet_disable_tx_rx_mac(&tse_core);
	ethernet_set_mac_address(&tse_core);
	ethernet_enable_tx_rx_mac(&tse_core);
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
	fdt_addr_t base_addr;
	u32 sgdma_base_addr;

	base_addr = dev_read_addr(dev);
	if (base_addr == FDT_ADDR_T_NONE)
		return -EINVAL;
	plat->tse_base_address = base_addr;
	dev_read_u32(dev, "sgdma_base", &sgdma_base_addr);
	plat->sgdma_base_address = sgdma_base_addr;

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
