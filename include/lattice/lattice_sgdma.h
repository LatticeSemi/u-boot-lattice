/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2019-2024 by Lattice Semiconductor Corporation
 */

#ifndef SGDMA_H_
#define SGDMA_H_

#include <linux/types.h>

/* Sgdma driver source code versions */
#define DRIVER_MAJOR_VERSION			1
#define DRIVER_MINOR_VERSION			0
#define DRIVER_TEST_VERSION			0

/* Register name, address and offset */
#define MM2S_CTRL					(0x00U)
#define MM2S_STS					(0x04U)
#define MM2S_CURDESC				(0x08U)
#define S2MM_CTRL					(0x20U)
#define S2MM_STS					(0x24U)
#define S2MM_CURDESC				(0x28U)

#define MAX_TRANSFER_SIZE			(0xffffU)
#define MULTI_BUF_SIZE				(100U)
#define FP_BIT_POSITION				(30U)
#define NXT_BIT_POSITION			(31U)
#define TID_BIT_POSITION			(20U)
#define TDEST_BIT_POSITION			(16U)
#define XFER_CMPL_POSITION			(16U)
#define DESC_SIZE					(0x10U)
#define BUFFER_SIZE_MASK			(0xffffU)
#define SGDMA_RESET_FLAG			(1U)
#define DMA_TRIGGER					(1U)
#define MSB_ADDR					(0x00U)
#define NXT_1						(1U)
#define	NXT_0						(0U)
#define FP_1						(1U)
#define FP_0						(0U)
#define XFER_CMPL_1					(0x01U)
#define IDX_0						(0U)
#define TRUE						(1U)
#define FALSE						(0U)

/* Enum */
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

/* structure and union */
struct sgdma_instance_t {
	void __iomem *base_addr;
	u32 *buffer;
	u32 num_of_desc;
	u32 blocking_s2mm;
	u32 blocking_mm2s;
	u32 mm2s_buffer_addr;
	u32 s2mm_buffer_addr;
	u32 mm2s_bd_addr;
	u32 s2mm_bd_addr;
	u32 per_desc_length;
};

/* mm2s control and status register structure */
struct mm2s_ctrl_reg_t {
	u32 mm_request : 1;
	u32 mm_reset : 1;
	u32 mm_rsvd_1 : 14;
	u32 mm_cmpl_irq_mask : 1;
	u32 mm_rsvd_2 : 15;
};

struct mm2s_sts_reg_t {
	u32 mm_status : 1;
	u32 mm_rsvd_1 : 7;
	u32 mm_reg_bd_len_err : 1;
	u32 mm_reg_axi_slave_err : 1;
	u32 mm_reg_axi_desc_err : 1;
	u32 mm_rsvd_2 : 5;
	u32 mm_xfer_cmpl : 1;
	u32 mm_xfer_err : 1;
	u32 mm_rsvd_3 : 14;
};

/* mm2s single and multi buffer Descriptor Structure */
struct mm2s_desc_tx_t {
	u32 mm_buffer_addr;	/* Buffer addr */
	u32 mm_buffer_msb_addr;	/* Buffer msb addr */
	u32 mm_control_buffer_size : 16;	/* Control */
	u32 mm_control_tdest : 4;
	u32 mm_control_tid : 4;
	u32 mm_control_arprot : 3;
	u32 mm_control_rsvd : 3;
	u32 mm_control_fp : 1;
	u32 mm_control_nxt : 1;
	u32 mm_status_transffered_size : 16;	/* Status */
	u32 mm_status_rsvd : 11;
	u32 mm_status_axi_desc_err : 1;
	u32 mm_status_axi_slave_err : 1;
	u32 mm_status_transffered_len_err : 1;
	u32 mm_status_bd_len_err : 1;
	u32 mm_status_cmpl : 1;
};

struct mm2s_desc_tx_ext_t {
	u32 mm_buffer_addr;
	u32 mm_buffer_msb_addr;
	u32 mm_control;
	u32 mm_status;
};

/* mm2s union */
struct mm2s_desc_t {
	struct mm2s_desc_tx_ext_t mm_bd_ext;
	struct mm2s_desc_tx_t mm_bd;
};

/* s2mm control and status register structure */
struct s2mm_ctrl_reg_t {
	u32 s_request : 1;
	u32 s_reset : 1;
	u32 s_rsvd_1 : 14;
	u32 s_cmpl_irq_mask : 1;
	u32 s_rsvd_2 : 15;
};

struct s2mm_sts_reg_t {
	u32 s_status : 1;
	u32 s_rsvd_1 : 7;
	u32 s_reg_bd_len_err : 1;
	u32 s_reg_axi_slave_err : 1;
	u32 s_reg_axi_desc_err : 1;
	u32 s_rsvd_2 : 5;
	u32 s_xfer_cmpl : 1;
	u32 s_xfer_err : 1;
	u32 s_rsvd_3 : 14;
};

/* s2mm single and multi buffer Descriptor Structure */
struct s2mm_desc_tx_t {
	u32 s_buffer_addr;	/* Buffer addr */
	u32 s_buffer_msb_addr;	/* Buffer msb addr */
	u32 s_control_buffer_size : 16;	/* Control */
	u32 s_control_rsvd_1 : 8;
	u32 s_control_awprot : 3;
	u32 s_control_rsvd_2 : 4;
	u32 s_control_nxt : 1;
	u32 s_status_transffered_size : 16;	/* Status */
	u32 s_status_rsvd : 11;
	u32 s_status_axi_desc_err : 1;
	u32 s_status_axi_slave_err : 1;
	u32 s_status_transffered_len_err : 1;
	u32 s_status_bd_len_err : 1;
	u32 s_status_cmpl : 1;
};

struct s2mm_desc_tx_ext_t {
	u32 s_buffer_addr;
	u32 s_buffer_msb_addr;
	u32 s_control;
	u32 s_status;
};

/* s2mm union */
struct s2mm_desc_t {
	struct s2mm_desc_tx_ext_t s_bd_ext;
	struct s2mm_desc_tx_t s_bd;
};

struct sgdma_ctrl_reg_t {
	u32 request : 1;
	u32 reset : 1;
	u32 rsvd_1 : 14;
	u32 cmpl_irq_mask : 1;
	u32 rsvd_2 : 15;
};

struct sgdma_sts_reg_t {
	u32 status : 1;
	u32 rsvd_1 : 7;
	u32 reg_bd_len_err : 1;
	u32 reg_axi_slave_err : 1;
	u32 reg_axi_desc_err : 1;
	u32 rsvd_2 : 5;
	u32 xfer_cmpl : 1;
	u32 xfer_err : 1;
	u32 rsvd_3 : 14;
};

/* Declaration */
void sgdma_init(struct sgdma_instance_t *sgdma, void __iomem *base_addr,
		u32 num_of_desc);
void sgdma_reset(struct sgdma_instance_t *sgdma,
		 enum control_type_t sgdma_cntl_type);
void sgdma_irq_mask(struct sgdma_instance_t *sgdma,
		    enum control_type_t sgdma_cntl_type,
		    enum irq_mask_t mask_value);
struct sgdma_sts_reg_t *get_sgdma_reg_status(struct sgdma_instance_t *sgdma,
					     enum status_type_t sgdma_sts_type);
u32 get_mm2s_bd_status(int idx);
u32 get_s2mm_bd_status(int idx);
u32 mm2s_buf_desc_dma(struct sgdma_instance_t *sgdma);
u32 s2mm_buf_desc_dma(struct sgdma_instance_t *sgdma);
u32 sgdma_register_read(struct sgdma_instance_t *sgdma,
			enum sgdma_reg_type_t index, u32 *reg_data);
u32 sgdma_register_write(struct sgdma_instance_t *sgdma,
			 enum sgdma_reg_type_t index, u32 reg_data);

/*
 * Explicit synchronization between writes to instruction memory and
 * instruction fetches on the same hart by flushing full instruction cache line.
 */
void cache_flush_ins(void);

/*
 * Invalidate 8-words(32-bytes) in data cache.
 * The starting address to be invalidate is aligned to 32-bytes boundary
 * based on <addr>.
 * e.g: 0x0 - 0x1c will be invalidate if addr=0x10, instead of
 * invalidating 0x10-0x2c
 */
void cache_invalidate(u32 addr);

/*
 * Invalidate the whole data cache.
 */
void cache_invalidate_all(void);

#endif
