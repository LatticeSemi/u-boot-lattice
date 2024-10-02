/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (C) 2024 Lattice Semiconductor Corporation
 */

#ifndef LATTICE_TSE_H_
#define LATTICE_TSE_H_

#include <linux/bitops.h>

#define IPG_TIME					12
#define MAX_PACKET_SIZE					1500
#define SPEED_10_OR_100_MBPS				0
#define SPEED_1G					0
#define SET_FULL_DUPLEX_MODE				5
#define SET_HALF_DUPLEX_MODE				5

// Mode Register
#define TX_EN						BIT(3)
#define RX_EN						BIT(2)
#define FC_EN						BIT(1)
#define GBIT_EN						BIT(0)

#define RCV_SHORT_FRAME					0x00000100
#define RCV_PAUSE_FRAME					0x00000008
#define RCV_MULTICAST					0x00000010
#define RCV_BROADCAST					0x00000080
#define RCV_ALL_ADDR_FRAME				0x00000001
#define RCV_DISCARD_FCS					0x00000002

struct tsemac_handle_t {
	unsigned char speed_mode;
	unsigned int adr;
	unsigned int frame_length;
	unsigned int mac_upper;
	unsigned int mac_lower;
	unsigned int multicast_upper;
	unsigned int multicast_lower;
	unsigned int tx_rx_ctrl_var;
};

/**
 * @brief Ethernet register.
 */
struct tsemac_reg_type_t {
	unsigned int mode_reg;					//0x0000
	unsigned int tx_rx_ctrl;
	unsigned int max_packet_size;
	unsigned int ipg;
	unsigned int mac_addr0;					//0x0010
	unsigned int mac_addr1;
	unsigned int tx_rx_status;
	unsigned int vlan_tag;
	unsigned int gmii_mgmt_ctrl;				//0x0020
	unsigned int gmii_mgmt_data;
	unsigned int multi_cast0;
	unsigned int multi_cast1;
	unsigned int pause_opcode;				//0x0030
	unsigned int tx_fifo_afull;
	unsigned int tx_fifo_aempty;
	unsigned int rx_fifo_afull;
	unsigned int rx_fifo_aempty;				//0x0040
	unsigned int interrupt_status;
	unsigned int interrupt_enable;				//0x0048
};

/**
 * @brief enum to set speed for tsemac
 */
enum speed_mode_set {
	fast_half_duplex_mode = 1,
	fast_full_duplex_mode,
	one_g_mode,
};

unsigned char ethernet_init(struct tsemac_handle_t *handle);
unsigned char ethernet_set_mac_address(struct tsemac_handle_t *handle);
unsigned char ethernet_get_mac_address(struct tsemac_handle_t *handle);
unsigned char ethernet_set_speed(struct tsemac_handle_t *handle);
unsigned char ethernet_enable_tx_rx_mac(struct tsemac_handle_t *handle);
unsigned char ethernet_disable_tx_rx_mac(struct tsemac_handle_t *handle);
unsigned char ethernet_tx_rx_control_reg_set(struct tsemac_handle_t *handle, unsigned char bit_pos);
unsigned int ethernet_mode_reg_read(struct tsemac_handle_t *handle);
unsigned int ethernet_tx_rx_status_reg_read(struct tsemac_handle_t *handle);

#endif /* ETHERNET_H_ */
