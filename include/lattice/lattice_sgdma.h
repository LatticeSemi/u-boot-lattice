/* SPDX-License-Identifier: GPL-2.0+
 *
 * (C) Copyright 2019-2024 by Lattice Semiconductor Corporation
 */

#ifndef SGDMA_H_
#define SGDMA_H_

#include <linux/types.h>

/* Register name, address and offset */
#define MM2S_CTRL					(0x00U)
#define MM2S_STS					(0x04U)
#define MM2S_CURDESC					(0x08U)
#define S2MM_CTRL					(0x20U)
#define S2MM_STS					(0x24U)
#define S2MM_CURDESC					(0x28U)
#define MAX_TRANSFER_SIZE				(0xffffU)
#define FP_BIT_POSITION					(30U)
#define NXT_BIT_POSITION				(31U)
#define TID_BIT_POSITION				(20U)
#define TDEST_BIT_POSITION				(16U)
#define XFER_CMPL_POSITION				(16U)
#define BUFFER_SIZE_MASK				(0xffffU)
#define SGDMA_RESET					(1U)
#define DMA_TRIGGER					(1U)
#define DESC_NEXT					(1U)
#define DESC_LAST					(0U)
#define FP_1						(1U)
#define FP_0						(0U)
#define XFER_CMPL_1					(0x01U)
#define SUCCESS						(1U)
#define FAILURE						(0U)

/*Enum*/
enum control_type_t {
	MM2S_RESET = 0,
	S2MM_RESET = 0x20,
};

enum status_type_t {
	MM2S_STATUS = 0x04,
	S2MM_STATUS = 0x24,
};

enum irq_mask_t {
	UNMASK = 0,
	MASK,
};

enum sgdma_reg_type_t {
	MM2S_CTRL_REG = 0,
	MM2S_STATUS_REG = 0x04,
	MM2S_CURDESC_REG = 0x08,
	S2MM_CTRL_REG = 0x20,
	S2MM_STATUS_REG = 0x24,
	S2MM_CURDESC_REG = 0x28,
};

struct mm2s_ctrl_reg_t {
	unsigned int mm_request : 1;
	unsigned int mm_reset : 1;
	unsigned int mm_rsvd_1 : 14;
	unsigned int mm_cmpl_irq_mask : 1;
	unsigned int mm_rsvd_2 : 15;
};

struct mm2s_sts_reg_t {
	unsigned int mm_status : 1;
	unsigned int mm_rsvd_1 : 7;
	unsigned int mm_reg_bd_len_err : 1;
	unsigned int mm_reg_axi_slave_err : 1;
	unsigned int mm_reg_axi_desc_err : 1;
	unsigned int mm_rsvd_2 : 5;
	unsigned int mm_xfer_cmpl : 1;
	unsigned int mm_xfer_err : 1;
	unsigned int mm_rsvd_3 : 14;
};

/* mm2s single and multi buffer Descriptor Structure */
struct mm2s_desc_tx_t {
	unsigned int mm_buffer_addr;			//Buffer addr
	unsigned int mm_buffer_msb_addr;		//Buffer msb addr
	unsigned int mm_control_buffer_size : 16;	//Control
	unsigned int mm_control_tdest : 4;
	unsigned int mm_control_tid : 4;
	unsigned int mm_control_arprot : 3;
	unsigned int mm_control_rsvd : 3;
	unsigned int mm_control_fp : 1;
	unsigned int mm_control_nxt : 1;
	unsigned int mm_status_transffered_size : 16;	//Status
	unsigned int mm_status_rsvd : 11;
	unsigned int mm_status_axi_desc_err : 1;
	unsigned int mm_status_axi_slave_err : 1;
	unsigned int mm_status_transffered_len_err : 1;
	unsigned int mm_status_bd_len_err : 1;
	unsigned int mm_status_cmpl : 1;
};

struct mm2s_desc_tx_ext_t {
	unsigned int mm_buffer_addr;
	unsigned int mm_buffer_msb_addr;
	unsigned int mm_control;
	unsigned int mm_status;
};

/*mm2s  union*/
union mm2s_desc_t {
	struct mm2s_desc_tx_ext_t mm_bd_ext;
	struct mm2s_desc_tx_t mm_bd;
};

/* s2mm control and status register structure */
struct s2mm_ctrl_reg_t {
	unsigned int s_request : 1;
	unsigned int s_reset : 1;
	unsigned int s_rsvd_1 : 14;
	unsigned int s_cmpl_irq_mask : 1;
	unsigned int s_rsvd_2 : 15;
};

struct s2mm_sts_reg_t {
	unsigned int s_status : 1;
	unsigned int s_rsvd_1 : 7;
	unsigned int s_reg_bd_len_err : 1;
	unsigned int s_reg_axi_slave_err : 1;
	unsigned int s_reg_axi_desc_err : 1;
	unsigned int s_rsvd_2 : 5;
	unsigned int s_xfer_cmpl : 1;
	unsigned int s_xfer_err : 1;
	unsigned int s_rsvd_3 : 14;
};

/*s2mm single and multi buffer Descriptor Structure */
struct s2mm_desc_tx_t {
	unsigned int s_buffer_addr;			//Buffer addr
	unsigned int s_buffer_msb_addr;			//Buffer msb addr
	unsigned int s_control_buffer_size : 16;	//Control
	unsigned int s_control_rsvd_1 : 8;
	unsigned int s_control_awprot : 3;
	unsigned int s_control_rsvd_2 : 4;
	unsigned int s_control_nxt : 1;
	unsigned int s_status_transffered_size : 16;	//Status
	unsigned int s_status_rsvd : 11;
	unsigned int s_status_axi_desc_err : 1;
	unsigned int s_status_axi_slave_err : 1;
	unsigned int s_status_transffered_len_err : 1;
	unsigned int s_status_bd_len_err : 1;
	unsigned int s_status_cmpl : 1;
};

struct s2mm_desc_tx_ext_t {
	unsigned int s_buffer_addr;
	unsigned int s_buffer_msb_addr;
	unsigned int s_control;
	unsigned int s_status;
};

/*s2mm union */
union s2mm_desc_t {
	struct s2mm_desc_tx_ext_t s_bd_ext;
	struct s2mm_desc_tx_t s_bd;
};

struct sgdma_ctrl_reg_t {
	unsigned int request : 1;
	unsigned int reset : 1;
	unsigned int rsvd_1 : 14;
	unsigned int cmpl_irq_mask : 1;
	unsigned int rsvd_2 : 15;
};

struct sgdma_sts_reg_t {
	unsigned int status : 1;
	unsigned int rsvd_1 : 7;
	unsigned int reg_bd_len_err : 1;
	unsigned int reg_axi_slave_err : 1;
	unsigned int reg_axi_desc_err : 1;
	unsigned int rsvd_2 : 5;
	unsigned int xfer_cmpl : 1;
	unsigned int xfer_err : 1;
	unsigned int rsvd_3 : 14;
};

struct sgdma_instance_t {
	unsigned int base_addr;
	unsigned int *mm2s_buffer_address;
	unsigned int *s2mm_buffer_address;
	unsigned int axi4_mm_data_width;
	unsigned int num_of_mm2s_desc;
	unsigned int num_of_s2mm_desc;
	// If the user sets blocking_mm2s or blocking_s2mm, the system will continuously check the
	// descriptor status until the operation is complete or an error occurs.
	// However, if the user does not intend to monitor the system closely, they should set both
	// blocking_s2mm and blocking_mm2s to false.
	unsigned int blocking_s2mm;
	unsigned int blocking_mm2s;
	union mm2s_desc_t *mm2s_bd_addr;
	union s2mm_desc_t *s2mm_bd_addr;
	unsigned int *per_mm2s_desc_length;
	unsigned int *per_s2mm_desc_length;
	unsigned int fp;
};

void sgdma_init(struct sgdma_instance_t *this_sgdma, unsigned int base_addr);
void sgdma_reset_s2mm(struct sgdma_instance_t *this_sgdma);
void sgdma_reset_mm2s(struct sgdma_instance_t *this_sgdma);
void sgdma_reset(struct sgdma_instance_t *this_sgdma);
void sgdma_irq_mask(struct sgdma_instance_t *this_sgdma, enum control_type_t sgdma_cntl_type,
		    enum irq_mask_t mask_value);
struct sgdma_sts_reg_t *get_sgdma_reg_status(struct sgdma_instance_t *this_sgdma,
					     enum status_type_t sgdma_sts_type);
unsigned int get_mm2s_bd_status(struct sgdma_instance_t *this_sgdma, int idx);
unsigned int get_s2mm_bd_status(struct sgdma_instance_t *this_sgdma, int idx);
unsigned int mm2s_buf_desc_dma(struct sgdma_instance_t *this_sgdma);
unsigned int s2mm_buf_desc_dma(struct sgdma_instance_t *this_sgdma);
unsigned int sgdma_register_read(struct sgdma_instance_t *this_sgdma, enum sgdma_reg_type_t index,
				 unsigned int *reg_data);
unsigned int sgdma_register_write(struct sgdma_instance_t *this_sgdma, enum sgdma_reg_type_t index,
				  unsigned int reg_data);

/*
 * Explicit synchronization between writes to instruction memory and instruction fetches on the
 * same hart by flushing full instruction cache line.
 */
void cache_flush_ins(void);

/*
 * Invalidate 8-words(32-bytes) in data cache.
 * The starting address to be invalidate is aligned to 32-bytes boundary based on <addr>.
 * .e.g: 0x0 - 0x1c will be invalidate if addr=0x10, instead of invalidating 0x10-0x2c
 */
void cache_invalidate(uint32_t addr);

/*
 * Invalidate the whole data cache.
 */
void cache_invalidate_all(void);

#endif
