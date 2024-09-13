// SPDX-License-Identifier: GPL-2.0
/*
 * (C) Copyright 2019-2024 by Lattice Semiconductor Corporation
 */

#include <lattice/lattice_sgdma.h>
#include <asm/io.h>
#include <dm.h>
#include <time.h>
#include <stdbool.h>

#define TSE_SGDMA_BUSY_TIMEOUT		(CONFIG_SYS_HZ)

/*
 * Explicit synchronization between writes to instruction memory and instruction fetches on the
 * same hart by flushing full instruction cache line.
 */
void cache_flush_ins(void)
{
	__asm__ __volatile__("fence.i");
}

/*
 * Invalidate 8-words(32-bytes) in data cache.
 * The starting address to be invalidate is aligned to 32-bytes boundary based on <addr>.
 * .e.g: 0x0 - 0x1c will be invalidate if addr=0x10, instead of invalidating 0x10-0x2c
 */
void cache_invalidate(uint32_t addr)
{
	__asm__ __volatile__(
		"mv a0, %0\n"
		".word 0x5500f\n"
			:
			: "r"(addr)
	);
}

/*
 * Invalidate the whole data cache.
 */
void cache_invalidate_all(void)
{
	__asm__ __volatile__(".word 0x500f");
}

void sgdma_init(struct sgdma_instance_t *this_sgdma, unsigned int base_addr)
{
	this_sgdma->base_addr = base_addr;
}

void sgdma_reset_s2mm(struct sgdma_instance_t *this_sgdma)
{
	struct sgdma_ctrl_reg_t *s2mm_cntl_reg =
		(struct sgdma_ctrl_reg_t *)(this_sgdma->base_addr + S2MM_CTRL);

	s2mm_cntl_reg->reset = SGDMA_RESET;
}

void sgdma_reset_mm2s(struct sgdma_instance_t *this_sgdma)
{
	struct sgdma_ctrl_reg_t *mm2s_cntl_reg =
		(struct sgdma_ctrl_reg_t *)(this_sgdma->base_addr + MM2S_CTRL);

	mm2s_cntl_reg->reset = SGDMA_RESET;
}

void sgdma_reset(struct sgdma_instance_t *this_sgdma)
{
	struct sgdma_ctrl_reg_t *mm2s_cntl_reg =
		(struct sgdma_ctrl_reg_t *)(this_sgdma->base_addr + MM2S_CTRL);
	struct sgdma_ctrl_reg_t *s2mm_cntl_reg =
		(struct sgdma_ctrl_reg_t *)(this_sgdma->base_addr + S2MM_CTRL);

	mm2s_cntl_reg->reset = SGDMA_RESET;
	s2mm_cntl_reg->reset = SGDMA_RESET;
}

void sgdma_irq_mask(struct sgdma_instance_t *this_sgdma, enum control_type_t sgdma_cntl_type,
		    enum irq_mask_t mask_value)
{
	struct sgdma_ctrl_reg_t *cntl_reg =
		(struct sgdma_ctrl_reg_t *)(this_sgdma->base_addr + sgdma_cntl_type);

	cntl_reg->cmpl_irq_mask = mask_value;
}

struct sgdma_sts_reg_t *get_sgdma_reg_status(struct sgdma_instance_t *this_sgdma,
					     enum status_type_t sgdma_sts_type)
{
	struct sgdma_sts_reg_t *sts_reg =
		(struct sgdma_sts_reg_t *)(this_sgdma->base_addr + sgdma_sts_type);

	return sts_reg;
}

unsigned int get_mm2s_bd_status(struct sgdma_instance_t *this_sgdma, int idx)
{
	return this_sgdma->mm2s_bd_addr[idx].mm_bd_ext.mm_status;
}

unsigned int get_s2mm_bd_status(struct sgdma_instance_t *this_sgdma, int idx)
{
	return this_sgdma->s2mm_bd_addr[idx].s_bd_ext.s_status;
}

unsigned int mm2s_buf_desc_dma(struct sgdma_instance_t *this_sgdma)
{
	unsigned int idx = 0;
	unsigned int fp = 0;
	unsigned int *mm_multi_wr_reg_value =
			(unsigned int *)(this_sgdma->base_addr + MM2S_CURDESC);
	unsigned int *mm_multi_rd_sts_reg_value =
			(unsigned int *)(this_sgdma->base_addr + MM2S_STS);
	struct mm2s_ctrl_reg_t *cntl_reg =
		(struct mm2s_ctrl_reg_t *)(this_sgdma->base_addr + MM2S_CTRL);

	for (idx = 0; idx < this_sgdma->num_of_mm2s_desc; idx++) {
		// The `mm2s_buffer_address` should point to an array that stores
		// all the scatter memory locations.
		this_sgdma->mm2s_bd_addr[idx].mm_bd_ext.mm_buffer_addr =
			this_sgdma->mm2s_buffer_address[idx];
		this_sgdma->mm2s_bd_addr[idx].mm_bd_ext.mm_buffer_msb_addr = 0;

		switch (this_sgdma->axi4_mm_data_width) {
		case 64:
			this_sgdma->mm2s_bd_addr[idx].mm_bd_ext.mm_buffer_addr &= 0xFFFFFFF8;
		break;
		case 128:
			this_sgdma->mm2s_bd_addr[idx].mm_bd_ext.mm_buffer_addr &= 0xFFFFFFF0;
		break;
		case 32:
		default:
			this_sgdma->mm2s_bd_addr[idx].mm_bd_ext.mm_buffer_addr &= 0xFFFFFFFC;
		break;
		}

		//Control
		fp = this_sgdma->fp;
		if (this_sgdma->per_mm2s_desc_length[idx] < MAX_TRANSFER_SIZE) {
			if (idx == (this_sgdma->num_of_mm2s_desc - 1)) {
				this_sgdma->mm2s_bd_addr[idx].mm_bd_ext.mm_control =
					(DESC_LAST << NXT_BIT_POSITION) |
					(fp << FP_BIT_POSITION) |
					(BUFFER_SIZE_MASK &
					this_sgdma->per_mm2s_desc_length[idx]);
			} else {
				this_sgdma->mm2s_bd_addr[idx].mm_bd_ext.mm_control =
					(DESC_NEXT << NXT_BIT_POSITION) |
					(fp << FP_BIT_POSITION) |
					(BUFFER_SIZE_MASK &
					this_sgdma->per_mm2s_desc_length[idx]);
			}
		} else {
			return FAILURE;
		}
	}

	//MM2S_CURDESC
	*mm_multi_wr_reg_value = (unsigned int)this_sgdma->mm2s_bd_addr;

	//DMA trigger
	cntl_reg->mm_request = DMA_TRIGGER;

	ulong ctime;
	/* Wait for the descriptor (chain) to complete */
	ctime = get_timer(0);
	if (this_sgdma->blocking_mm2s) {
		while (!(*mm_multi_rd_sts_reg_value >> XFER_CMPL_POSITION) & XFER_CMPL_1) {
			if (get_timer(ctime) > TSE_SGDMA_BUSY_TIMEOUT) {
				printf("SGDMA MM2S timeout\n");
				break;
			}
		};
	}
	printf("MM2S done\n");
	return SUCCESS;
}

unsigned int s2mm_buf_desc_dma(struct sgdma_instance_t *this_sgdma)
{
	unsigned int idx = 0;
	unsigned int *s_multi_wr_reg_value =
				(unsigned int *)(this_sgdma->base_addr + S2MM_CURDESC);
	unsigned int *s_multi_rd_sts_reg_value =
				(unsigned int *)(this_sgdma->base_addr + S2MM_STS);
	struct s2mm_ctrl_reg_t *cntl_reg =
		(struct s2mm_ctrl_reg_t *)(this_sgdma->base_addr + S2MM_CTRL);

	for (idx = 0; idx < this_sgdma->num_of_s2mm_desc; idx++) {
		this_sgdma->s2mm_bd_addr[idx].s_bd_ext.s_buffer_addr =
							this_sgdma->s2mm_buffer_address[idx];
		this_sgdma->s2mm_bd_addr[idx].s_bd_ext.s_buffer_msb_addr = 0;

		switch (this_sgdma->axi4_mm_data_width) {
		case 64:
			this_sgdma->s2mm_bd_addr[idx].s_bd_ext.s_buffer_addr &= 0xFFFFFFF8;
		break;
		case 128:
			this_sgdma->s2mm_bd_addr[idx].s_bd_ext.s_buffer_addr &= 0xFFFFFFF0;
		break;
		case 32:
		default:
			this_sgdma->s2mm_bd_addr[idx].s_bd_ext.s_buffer_addr &= 0xFFFFFFFC;
		break;
		}
		//Control
		if (this_sgdma->per_s2mm_desc_length[idx] < MAX_TRANSFER_SIZE) {
			if (idx == (this_sgdma->num_of_s2mm_desc - 1)) {
				this_sgdma->s2mm_bd_addr[idx].s_bd_ext.s_control =
					(DESC_LAST << NXT_BIT_POSITION) |
					(BUFFER_SIZE_MASK &
					this_sgdma->per_s2mm_desc_length[idx]);
			} else {
				this_sgdma->s2mm_bd_addr[idx].s_bd_ext.s_control =
					(DESC_NEXT << NXT_BIT_POSITION) |
					(BUFFER_SIZE_MASK &
					this_sgdma->per_s2mm_desc_length[idx]);
			}
		} else {
			return FAILURE;
		}
	}

	cache_flush_ins();
	cache_invalidate_all();

	//S2MM_CURDESC
	*s_multi_wr_reg_value = (unsigned int)this_sgdma->s2mm_bd_addr;
	//DMA trigger
	cntl_reg->s_request = DMA_TRIGGER;

	ulong ctime;
	/* Wait for the descriptor (chain) to complete */
	ctime = get_timer(0);
	if (this_sgdma->blocking_s2mm) {
		while (!(*s_multi_rd_sts_reg_value >> XFER_CMPL_POSITION) & XFER_CMPL_1) {
			if (get_timer(ctime) > TSE_SGDMA_BUSY_TIMEOUT) {
				printf("SGDMA S2MM timeout\n");
				break;
			}
		};
	}
	printf("S2MM done\n");
	return SUCCESS;
}

unsigned int sgdma_register_read(struct sgdma_instance_t *this_sgdma,
				 enum sgdma_reg_type_t index, unsigned int *reg_data)
{
	if (!this_sgdma)
		return FAILURE;

	unsigned int base_addr = (unsigned int)(this_sgdma->base_addr);
	unsigned int *read_reg = (unsigned int *)(base_addr + index);
	*reg_data = *read_reg;

	return SUCCESS;
}

unsigned int sgdma_register_write(struct sgdma_instance_t *this_sgdma,
				  enum sgdma_reg_type_t index, unsigned int reg_data)
{
	if (!this_sgdma)
		return FAILURE;

	unsigned int base_addr = (unsigned int)(this_sgdma->base_addr);
	unsigned int *write_reg = (unsigned int *)(base_addr + index);
	*write_reg = reg_data;

	return SUCCESS;
}
