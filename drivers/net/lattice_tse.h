/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2019-2024 by Lattice Semiconductor Corporation
 *
 * This file is used to intizalize the ethernet module and configure for
 * sending and receiving of ethernet packet.
 */

#ifndef LATTICE_TSE_H_
#define LATTICE_TSE_H_

#include <linux/types.h>

/* Driver Details */
#define TSE_DRV_VER "v1.0.0"

#define	IPG_TIME			12
#define MAX_PACKET_SIZE			1500
#define	SPEED_10_OR_100_MBPS		0
#define	SPEED_1G			0
#define	SET_FULL_DUPLEX_MODE		5
#define	SET_HALF_DUPLEX_MODE		5

#define RCV_SHORT_FRAME			0x00000100
#define RCV_PAUSE_FRAME			0x00000008
#define RCV_MULTICAST			0x00000010
#define RCV_BROADCAST			0x00000080
#define RCV_ALL_ADDR_FRAME		0x00000001

#define ENABLE_RX_MODE			0x00000004
#define ENABLE_TX_MODE			0x00000008
#define SGDMA_BASE			0x10093000
#define MM2S_ADDR			0x90000000
#define S2MM_ADDR			0x90006000

struct tsemac_handle_t {
	u_char speed_mode;
	void __iomem *addr;
	u32 frame_length;
	u32 mac_upper;
	u32 mac_lower;
	u32 multicast_upper;
	u32 multicast_lower;
	u32 tx_rx_ctrl_var;
	u32 enable_tx_mac;
	u32 enable_rx_mac;
};

/**
 * @brief Ethernet register.
 */
struct tsemac_reg_type_t {
	u32 mode_reg;
	u32 tx_rx_ctrl;
	u32 max_packet_size;
	u32 ipg;
	u32 mac_addr0;
	u32 mac_addr1;
	u32 tx_rx_status;
	u32 vlan_tag;
	u32 gmii_mgmt_ctrl;
	u32 gmii_mgmt_data;
	u32 multi_cast0;
	u32 multi_cast1;
	u32 pause_opcode;
	u32 tx_fifo_afull;
	u32 tx_fifo_aempty;
	u32 rx_fifo_afull;
	u32 rx_fifo_aempty;
	u32 interrupt_status;
	u32 interrupt_enable;
	u32 tx_stat_unicst_0;
	u32 tx_stat_unicst_1;
	u32 tx_stat_multcst_0;
	u32 tx_stat_multcst_1;
	u32 tx_stat_brdcst_0;
	u32 tx_stat_brdcst_1;
	u32 tx_stat_badfcs_0;
	u32 tx_stat_badfcs_1;
	u32 tx_stat_jumbo_0;
	u32 tx_stat_jumbo_1;
	u32 tx_stat_under_run_0;
	u32 tx_stat_under_run_1;
	u32 tx_stat_pause_0;
	u32 tx_stat_pause_1;
	u32 tx_stat_vlan_tg_0;
	u32 tx_stat_vlan_tg_1;
	u32 tx_stat_frm_length_0;
	u32 tx_stat_frm_length_1;
	u32 tx_stat_deferred_trans_0;
	u32 tx_stat_deferred_trans_1;
	u32 tx_stat_excess_deferred_trans_0;
	u32 tx_stat_excess_deferred_trans_1;
	u32 tx_stat_late_col_0;
	u32 tx_stat_late_col_1;
	u32 tx_stat_excess_col_0;
	u32 tx_stat_excess_col_1;
	u32 tx_stat_num_early_col_0;
	u32 tx_stat_num_early_col_1;
	u32 tx_stat_shrt_frm_dis_fcs_0;
	u32 tx_stat_shrt_frm_dis_fcs_1;
	u32 tx_stat_ptp_1588_frm_0;
	u32 tx_stat_ptp_1588_frm_1;
	u32 tx_stat_frm_64_0;
	u32 tx_stat_frm_64_1;
	u32 tx_stat_frm_65_127_0;
	u32 tx_stat_frm_65_127_1;
	u32 tx_stat_frm_128_255_0;
	u32 tx_stat_frm_128_255_1;
	u32 tx_stat_frm_256_511_0;
	u32 tx_stat_frm_256_511_1;
	u32 tx_stat_frm_512_1023_0;
	u32 tx_stat_frm_512_1023_1;
	u32 tx_stat_frm_1024_1518_0;
	u32 tx_stat_frm_1024_1518_1;
	u32 tx_stat_frm_1519_2047_0;
	u32 tx_stat_frm_1519_2047_1;
	u32 tx_stat_frm_2048_4095_0;
	u32 tx_stat_frm_2048_4095_1;
	u32 tx_stat_frm_4096_9216_0;
	u32 tx_stat_frm_4096_9216_1;
	u32 tx_stat_frm_9217_16383_0;
	u32 tx_stat_frm_9217_16383_1;
	u32 rx_stat_frm_length_0;
	u32 rx_stat_frm_length_1;
	u32 rx_stat_vlan_tg_0;
	u32 rx_stat_vlan_tg_1;
	u32 rx_stat_pause_0;
	u32 rx_stat_pause_1;
	u32 rx_stat_ctrl_0;
	u32 rx_stat_ctrl_1;
	u32 rx_stat_unsp_opcode_0;
	u32 rx_stat_unsp_opcode_1;
	u32 rx_stat_dribb_nibb_0;
	u32 rx_stat_dribb_nibb_1;
	u32 rx_stat_brdcst_0;
	u32 rx_stat_brdcst_1;
	u32 rx_stat_multcst_0;
	u32 rx_stat_multcst_1;
	u32 rx_stat_unicst_0;
	u32 rx_stat_unicst_1;
	u32 rx_stat_rcvd_ok_0;
	u32 rx_stat_rcvd_ok_1;
	u32 rx_stat_length_error_0;
	u32 rx_stat_length_error_1;
	u32 rx_stat_crc_error_0;
	u32 rx_stat_crc_error_1;
	u32 rx_stat_pkt_ignore_0;
	u32 rx_stat_pkt_ignore_1;
	u32 rx_stat_previous_carrier_event_0;
	u32 rx_stat_previous_carrier_event_1;
	u32 rx_stat_ptp1588_frm_0;
	u32 rx_stat_ptp1588_frm_1;
	u32 rx_stat_ipg_viol_0;
	u32 rx_stat_ipg_viol_1;
	u32 rx_stat_shrt_frm_0;
	u32 rx_stat_shrt_frm_1;
	u32 rx_stat_lng_frm_0;
	u32 rx_stat_lng_frm_1;
	u32 rx_stat_frm_undersize_0;
	u32 rx_stat_frm_undersize_1;
	u32 rx_stat_frm_fragement_0;
	u32 rx_stat_frm_fragement_1;
	u32 rx_stat_frm_jabber_0;
	u32 rx_stat_frm_jabber_1;
	u32 rx_stat_frm_64_good_crc_0;
	u32 rx_stat_frm_64_good_crc_1;
	u32 rx_stat_frm_1518_good_crc_0;
	u32 rx_stat_frm_1518_good_crc_1;
	u32 rx_stat_frm_64_0;
	u32 rx_stat_frm_64_1;
	u32 rx_stat_frm_65_127_0;
	u32 rx_stat_frm_65_127_1;
	u32 rx_stat_frm_128_255_0;
	u32 rx_stat_frm_128_255_1;
	u32 rx_stat_frm_256_511_0;
	u32 rx_stat_frm_256_511_1;
	u32 rx_stat_frm_512_1023_0;
	u32 rx_stat_frm_512_1023_1;
	u32 rx_stat_frm_1024_1518_0;
	u32 rx_stat_frm_1024_1518_1;
	u32 rx_stat_frm_1519_2047_0;
	u32 rx_stat_frm_1519_2047_1;
	u32 rx_stat_frm_2048_4095_0;
	u32 rx_stat_frm_2048_4095_1;
	u32 rx_stat_frm_4096_9216_0;
	u32 rx_stat_frm_4096_9216_1;
	u32 rx_stat_frm_9217_16383_0;
	u32 rx_stat_frm_9217_16383_1;
};

/**
 * @brief enum to set speed for tsemac
 */
enum speed_mode_set {
	fast_half_duplex_mode = 1,
	fast_full_duplex_mode,
	one_g_mode,
};

/**
 * @brief enum for statistics counter register
 */
enum statistics_counter_reg {
	tx_unicast_frame_counter_reg = 19, /* index value for offset 0x04C */
	tx_multicast_frame_counter_reg = 21,
	tx_broadcast_frame_counter_reg = 23,
	tx_badfcs_frame_counter_reg = 25,
	tx_jumbo_frame_counter_reg = 27,
	tx_under_run_frame_counter_reg = 29,
	tx_pause_frame_counter_reg = 31,
	tx_vlan_tag_frame_counter_reg = 33,
	tx_frame_length_counter_reg = 35,
	tx_deferred_transmission_counter_reg = 37,
	tx_excessive_deferred_transmission_counter_reg = 39,
	tx_late_collision_counter_reg = 41,
	tx_excessive_collision_counter_reg = 43,
	tx_num_early_collision_counter_reg = 45,
	tx_short_frame_dis_fcs_counter_reg = 47,
	tx_ptp_1588_frame_counter_reg = 49,
	tx_frame_length_64_counter_reg = 51,
	tx_frame_length_65_127_counter_reg = 53,
	tx_frame_length_128_255_counter_reg = 55,
	tx_frame_length_256_511_counter_reg = 57,
	tx_frame_length_512_1023_counter_reg = 59,
	tx_frame_length_1024_1518_counter_reg = 61,
	tx_frame_length_1519_2047_counter_reg = 63,
	tx_frame_length_2048_4095_counter_reg = 65,
	tx_frame_length_4096_9216_counter_reg = 67,
	tx_frame_length_9217_16383_counter_reg = 69,
	rx_frame_length_counter_reg = 71,
	rx_vlan_tag_frame_counter_reg = 73,
	rx_pause_frame_counter_reg = 75,
	rx_control_frame_counter_reg = 77,
	rx_unsupported_opcode_reg = 79,
	rx_dribble_nibble_counter_reg = 81,
	rx_broadcast_frame_counter_reg = 83,
	rx_multicast_frame_counter_reg = 85,
	rx_unicast_frame_counter_reg = 87,
	rx_frame_received_ok_counter_reg = 89,
	rx_frame_received_length_error_counter_reg = 91,
	rx_frame_received_crc_error_counter_reg = 93,
	rx_frame_received_packet_ignored_counter_reg = 95,
	rx_frame_received_carrier_event_counter_reg = 97,
	rx_ptp_1588_frame_counter_reg = 99,
	rx_frame_received_ipg_violation_counter_reg = 101,
	rx_received_short_frame_counter_reg = 103,
	rx_received_long_frame_counter_reg = 105,
	rx_received_undersize_frame_counter_reg = 107,
	rx_received_frame_fragments_counter_reg = 109,
	rx_received_frame_jabber_counter_reg = 111,
	rx_received_frame_length64_good_crc_counter_reg = 113,
	rx_received_frame_length1518_good_crc_counter_reg = 115,
	rx_frame_length_64_counter_reg = 117,
	rx_frame_length_65_127_counter_reg = 119,
	rx_frame_length_128_255_counter_reg = 121,
	rx_frame_length_256_511_counter_reg = 123,
	rx_frame_length_512_1023_counter_reg = 125,
	rx_frame_length_1024_1518_counter_reg = 127,
	rx_frame_length_1519_2047_counter_reg = 129,
	rx_frame_length_2048_4095_counter_reg = 131,
	rx_frame_length_4096_9216_counter_reg = 133,
	rx_frame_length_9217_16383_counter_reg = 135,
};

u8 ethernet_init(struct tsemac_handle_t *handle);
void ethernet_packet_handle(struct tsemac_handle_t *handle, u32 *src_packet,
			    u32 *dest_packet);
u8 ethernet_set_mac_address(struct tsemac_handle_t *handle);
u8 ethernet_get_mac_address(struct tsemac_handle_t *handle);
u8 ethernet_set_multicast_address(struct tsemac_handle_t *handle);
u8 ethernet_get_multicast_address(struct tsemac_handle_t *handle);
u64 ethernet_statistics_counter_register_read(struct tsemac_handle_t *handle,
					      u8 offset);
u8 ethernet_set_speed(struct tsemac_handle_t *handle);
u8 ethernet_tx_rx_control_reg_set(struct tsemac_handle_t *handle, u8 bit_pos);
u32 ethernet_mode_reg_read(struct tsemac_handle_t *handle);
u32 ethernet_tx_rx_status_reg_read(struct tsemac_handle_t *handle);

#endif /* ETHERNET_H_ */
