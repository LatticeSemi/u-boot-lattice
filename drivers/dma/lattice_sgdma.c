// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2019-2024 by Lattice Semiconductor Corporation
 */

#include <lattice/lattice_sgdma.h>
#include <linux/iopoll.h>
#include <asm/io.h>
#include <dm.h>

#define TSE_SGDMA_BUSY_TIMEOUT		(CONFIG_SYS_HZ)

/* Global variable of the single and multi buffer descriptor */
static struct mm2s_desc_t *mm_multi_bd_confg[MULTI_BUF_SIZE];
static struct s2mm_desc_t *s_multi_bd_confg[MULTI_BUF_SIZE];

/*
 * Explicit synchronization between writes to instruction memory and
 * instruction fetches on the same hart by flushing full instruction cache line.
 */
void cache_flush_ins(void)
{
	__asm__ __volatile__("fence.i");
}

/*
 * Invalidate 8-words(32-bytes) in data cache.
 * The starting address to be invalidate is aligned to 32-bytes boundary
 * based on <addr>.
 * e.g: 0x0 - 0x1c will be invalidate if addr=0x10, instead of
 * invalidating 0x10-0x2c
 */
void cache_invalidate(u32 addr)
{
	__asm__ __volatile__("mv a0, %0\n"
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

/*
 * sgdma_init
 * @brief	Initialize the single and multi descriptor instances.
 *
 * @param *sgdma	This structure pointer initilaize with the sgdma IP base
 *			address and number of descriptor value, have the m2s and
 *						s2mm BD address.
 * @param base_addr	This parameter specify the sgdma IP base address.
 * @param num_of_desc	This parameter specify the single or Multi Buffer
 *			descriptor value.
 *
 * @return	None.
 */
void sgdma_init(struct sgdma_instance_t *sgdma, void __iomem *base_addr,
		u32 num_of_desc)
{
	u32 i = 0;

	sgdma->base_addr = base_addr;
	sgdma->num_of_desc = num_of_desc;

	for (i = 0; i  < sgdma->num_of_desc; i++) {
		mm_multi_bd_confg[i] =
			(struct mm2s_desc_t *)(sgdma->mm2s_bd_addr +
						(i * DESC_SIZE));
		s_multi_bd_confg[i] =
			(struct s2mm_desc_t *)(sgdma->s2mm_bd_addr +
						(i * DESC_SIZE));
	}
}

/*
 * sgdma_reset
 * @brief	In this API reset the MM2S ctrl and S2MM cntrl register.
 *
 * @param *sgdma	This structure pointer has the info of sgdma IP base
 *			address.
 * @param sgdma_cntl_type	This parameter get the mm2s control offset
 *				and s2mm control offset.
 *
 * @return	none.
 */
void sgdma_reset(struct sgdma_instance_t *sgdma,
		 enum control_type_t sgdma_cntl_type)
{
	struct sgdma_ctrl_reg_t *cntl_reg =
		(struct sgdma_ctrl_reg_t *)(sgdma->base_addr +
						sgdma_cntl_type);
	cntl_reg->reset = SGDMA_RESET_FLAG;
}

/*
 * sgdma_irq_mask
 * @brief	Mask and unmask the mm2s and s2mm interrupt request.
 *
 * @param *sgdma	This structure pointer has the info of sgdma IP base
 *			address.
 * @param sgdma_cntl_type	This parameter get the mm2s control offset
 *				and s2mm control offset.
 * @param mask_value	This parameter describe mask and unmask value.
 *
 * @return	None.
 */
void sgdma_irq_mask(struct sgdma_instance_t *sgdma,
		    enum control_type_t sgdma_cntl_type,
		    enum irq_mask_t mask_value)
{
	struct sgdma_ctrl_reg_t *cntl_reg =
		(struct sgdma_ctrl_reg_t *)(sgdma->base_addr +
						sgdma_cntl_type);
	cntl_reg->cmpl_irq_mask = mask_value;
}

/*
 * get_sgdma_reg_status
 * @brief	Get the mm2s and s2mm register status , bd len err, xfer_cmpl,
 *		xfer_err information.
 *
 * @param *sgdma		This structure pointer has the info of sgdma IP
 *				base address.
 * @param sgdma_sts_type	This parameter get the mm2s status offset and s2mm
 *				status offset as a parameter.
 *
 * @return	This API return the mm2s status register address.
 */
struct sgdma_sts_reg_t *get_sgdma_reg_status(struct sgdma_instance_t *sgdma,
					     enum status_type_t sgdma_sts_type)
{
	struct sgdma_sts_reg_t *sts_reg =
		(struct sgdma_sts_reg_t *)(sgdma->base_addr +
						sgdma_sts_type);
	return sts_reg;
}

/*
 * get_mm2s_bd_status
 * @brief	Get the mm2s BD status transffered len err, cmpl, slave_err,
 *		descriptor error inforamtion.
 *
 * @param idx	This parameter defines the number of multiple buffer descriptor
 *		sequences.
 *
 * @return	This API return the mm2s status register value.
 */
u32 get_mm2s_bd_status(int idx)
{
	return mm_multi_bd_confg[idx]->mm_bd_ext.mm_status;
}

/*
 * get_s2mm_bd_status
 * @brief	Get the s2mm BD status transffered len err, cmpl, slave_err,
 *		descriptor error information.
 *
 * @param idx	This parameter defines the number of multiple buffer descriptor
 *		sequences.
 *
 * @return	This API return the s2mm status register value.
 */
u32 get_s2mm_bd_status(int idx)
{
	return s_multi_bd_confg[idx]->s_bd_ext.s_status;
}

/*
 * mm2s_buf_desc_dma
 * @brief	Performs a mm2s_buf_desc_dma API configure single and multiple
 *		buffer descriptor to update the registers value and trigger
 *		the DMA.
 *
 * @param *sgdma	This structure pointer parameter has the info of base
 *			address of mm2s buffer, length and number of descriptor.
 *			length is specify the buffer length and num_of_desc is
 *			specify the single or multiple buffer descriptor value.
 *
 * @return	This API returns 0 on success 1 on failure.
 */
u32 mm2s_buf_desc_dma(struct sgdma_instance_t *sgdma)
{
	int ret;
	u32 i, tmp;
	void __iomem *mm_multi_wr_reg_value = sgdma->base_addr + MM2S_CURDESC;
	void __iomem *mm_multi_rd_sts_reg_value = sgdma->base_addr + MM2S_STS;
	struct mm2s_ctrl_reg_t *cntl_reg =
		(struct mm2s_ctrl_reg_t *)(sgdma->base_addr + MM2S_CTRL);

	for (i = 0; i < sgdma->num_of_desc; i++) {
		mm_multi_bd_confg[i]->mm_bd_ext.mm_buffer_addr =
			(u32)(sgdma->buffer +
			      ((i * sgdma->per_desc_length) >> 2));
		mm_multi_bd_confg[i]->mm_bd_ext.mm_buffer_msb_addr = MSB_ADDR;

		/* Control */
		if (sgdma->per_desc_length < MAX_TRANSFER_SIZE) {
			if ((sgdma->num_of_desc - 1) == i)
				mm_multi_bd_confg[i]->mm_bd_ext.mm_control =
					(NXT_0 << NXT_BIT_POSITION) |
					(FP_0 << FP_BIT_POSITION) |
					(BUFFER_SIZE_MASK & sgdma->per_desc_length);
			else
				mm_multi_bd_confg[i]->mm_bd_ext.mm_control =
					(NXT_1 << NXT_BIT_POSITION) |
					(FP_0 << FP_BIT_POSITION) |
					(BUFFER_SIZE_MASK & sgdma->per_desc_length);
		} else {
			debug("sgdma failed here, size issue? 0x%x\n",
			      sgdma->per_desc_length);
			return 1;
		}
	}

	/* MM2S_CURDESC */
	writel((u32)mm_multi_bd_confg[IDX_0], mm_multi_wr_reg_value);

	/* DMA trigger */
	cntl_reg->mm_request = DMA_TRIGGER;

	/* Wait for the descriptor (chain) to complete */
	if (sgdma->blocking_mm2s) {
		ret = readl_poll_timeout(mm_multi_rd_sts_reg_value, tmp,
					 ((tmp >> XFER_CMPL_POSITION) &
					  XFER_CMPL_1),
					 TSE_SGDMA_BUSY_TIMEOUT);
		if (ret) {
			debug("SGDMA MM2S timeout\n");
			return 1;
		}
	}

	debug("MM2S done\n");

	return 0;
}

/*
 * s2mm_buf_desc_dma
 * @brief	Performs a s2mm_buf_desc_dma API configure single and multiple
 *		buffer descriptor to update the registers value and trigger the DMA.
 *
 * @param *sgdma	This structure pointer parameter has the info of base
 *			address of s2mm buffer, length and number of
 *			descriptor. length is specify the buffer length and
 *			num_of_desc is specify the single or multiple buffer
 *			descriptor value.
 *
 * @return	This API returns 0 on success and 1 for failure.
 */
u32 s2mm_buf_desc_dma(struct sgdma_instance_t *sgdma)
{
	int ret;
	u32 i, tmp;
	void __iomem *s_multi_wr_reg_value = sgdma->base_addr + S2MM_CURDESC;
	void __iomem *s_multi_rd_sts_reg_value = sgdma->base_addr + S2MM_STS;
	struct s2mm_ctrl_reg_t *cntl_reg =
		(struct s2mm_ctrl_reg_t *)(sgdma->base_addr + S2MM_CTRL);

	for (i = 0; i < sgdma->num_of_desc; i++) {
		s_multi_bd_confg[i]->s_bd_ext.s_buffer_addr =
			(u32)(sgdma->buffer +
				((i * sgdma->per_desc_length) >> 2));
		s_multi_bd_confg[i]->s_bd_ext.s_buffer_msb_addr = MSB_ADDR;

		/* Control */
		if (sgdma->per_desc_length < MAX_TRANSFER_SIZE) {
			if ((sgdma->num_of_desc - 1) == i)
				s_multi_bd_confg[i]->s_bd_ext.s_control =
					(NXT_0 << NXT_BIT_POSITION) |
					(FP_0 << FP_BIT_POSITION) |
					(BUFFER_SIZE_MASK & sgdma->per_desc_length);
			else
				s_multi_bd_confg[i]->s_bd_ext.s_control =
					(NXT_1 << NXT_BIT_POSITION) |
					(BUFFER_SIZE_MASK & sgdma->per_desc_length);
		} else {
			debug("%s: per_desc_length = %d\n", __func__,
			      sgdma->per_desc_length);
			return 1;
		}
	}

	cache_flush_ins();
	cache_invalidate_all();

	/* S2MM_CURDESC */
	writel((u32)s_multi_bd_confg[IDX_0], s_multi_wr_reg_value);

	/* DMA trigger */
	cntl_reg->s_request = DMA_TRIGGER;

	/* Wait for the descriptor (chain) to complete */
	if (sgdma->blocking_s2mm) {
		ret = readl_poll_timeout(s_multi_rd_sts_reg_value, tmp,
					 ((tmp >> XFER_CMPL_POSITION) &
					  XFER_CMPL_1),
					 TSE_SGDMA_BUSY_TIMEOUT);
		if (ret) {
			debug("SGDMA S2MM timeout\n");
			return 1;
		}
	}

	debug("S2MM done\n");

	return 0;
}

/*
 * sgdma_register_read
 * @brief	Performs a s2mm_buf_desc_dma API configure single and multiple
 *		buffer descriptor to update the registers value and trigger
 *		the DMA.
 *
 * @param *sgdma	This structure pointer has the info of sgdma IP base
 *			address.
 * @param index		It contains the offset value of particular register.
 * @param *reg_data	Pointer to the variable where data to be stored.
 *
 * @return	This API returns 0 on success and 1 for failure.
 */
u32 sgdma_register_read(struct sgdma_instance_t *sgdma,
			enum sgdma_reg_type_t index,
			u32 *reg_data)
{
	void __iomem *read_reg = sgdma->base_addr + index;

	*reg_data = readl(read_reg);

	return 0;
}

/**
 * sgdma_register_write
 * @brief	This API writes data to the mm2s or s2mm register address.
 *
 * @param *sgdma	This structure pointer has the info of sgdma IP base
 *			address.
 * @param index		It contains the offset value of particular register.
 * @param reg_data	It contains data that is write on the register address.
 *
 * @return	This API returns 0 on success and 1 for failure.
 */
u32 sgdma_register_write(struct sgdma_instance_t *sgdma,
			 enum sgdma_reg_type_t index,
			 u32 reg_data)
{
	void __iomem *write_reg = sgdma->base_addr + index;

	writel(reg_data, write_reg);

	return 0;
}
