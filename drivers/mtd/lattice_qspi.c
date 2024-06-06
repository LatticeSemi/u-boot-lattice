// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2019-2024 by Lattice Semiconductor Corporation
 */

#include <common.h>
#include <console.h>
#include <dm.h>
#include <errno.h>
#include <fdt_support.h>
#include <flash.h>
#include <log.h>
#include <mtd.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/iopoll.h>
#include <linux/sizes.h>

DECLARE_GLOBAL_DATA_PTR;

#define CLR_INTERRUPT			        (0x00000FFFU)
#define FLASH_ADDRESS_FIFO_DIS			(0x00060000U)
#define FLASH_ADDRESS_FIFO_EN			(0x00080000U)
#define START_TRANS_VAL			        (0x00000001U)
#define CLR_START_TRANS_VAL			    (0x00000000U)
#define FLASH_CMD_CODE_ERASE		    (0x10U)
#define FLASH_CMD_CODE_READ		        (0x0FU)
#define FLASH_CMD_CODE_WRITE_STD		(0x17U)
#define FLASH_CMD_CODE_WRITE		    (0x1AU)
#define NUM_WAIT_CYCLE					(0x6U)
#define MULT_FLASH_TGT					(0U)
#define	WITH_PAYLOAD_WR					(1U)
#define WITH_PAYLOAD_RD					(0U)		// 0 for read & 1 for write
#define	SUPP_FLASH_CMD					(1U)
#define FLASH_ADDR_WIDTH_FIFO_DIS		(0x2U)
#define TGT_CS						(0x0U)
#define ADDR_LANE_WIDTH					(0x0U)
#define CMD_LANE_WIDTH					(0x0U)
#define FLASH_ADDR_WIDTH_FIFO_EN		(0x2U)
#define RET_FAILURE						(1U)
#define RET_SUCCESS						(0U)
#define IDLE							(0U)
#define IS_NULL							(0U)
#define TRANSFR_LEN_SHIFT				(16U)
#define FLASH_ADDR_WID_SHIFT_FIFO_EN	(12U)
#define FLASH_ADDR_WID_SHIFT_FIFO_DIS	(13U)
#define FLASH_CMD_CODE_SHIFT			(2U)
#define NUM_WAIT_CYCLE_SHIFT			(8U)
#define MULT_FLASH_TGT_SHIFT			(7U)
#define WITH_PAYLOAD_SHIFT				(1U)
#define TGT_CS_SHIFT					(8U)
#define DATA_LANE_WIDTH_SHIFT			(4U)
#define ADDR_LANE_WIDTH_SHIFT			(2U)
#define WR_INT_EN_VAL                   (0x00000003U)
#define INT_EN_VAL                      (0x00000FFFU)
#define INT_DIS_VAL                     (0x00000000U)
#define SET_PKT_HDR3                    (0x0U)
#define CLR_START_TRANS                 (0x0U)
#define REG_GAP                         (2U)
#define IRQ_WR_FIFO_EN		            (0U)
#define IRQ_RD_FIFO_EN		            (1U)
#define IRQ_WR_FIFO_DIS					(0U)
#define IRQ_RD_FIFO_DIS	                (0U)
#define TRANS_STS					(1U)
#define CLR_IRQ_STS_VAR                 (0U)
#define IDX_ZERO			            (0U)
#define IDX_ONE						    (1U)
#define IDX_TWO						    (2U)
#define INCREMENT_ONE				    (1U)
#define INCREMENT_TWO				    (2U)

#define TGT_RD_TRANS_CNT				(0x00100000U)
#define TGT_WR_TRANS_CNT				(0x00000000U)

// Flash Command Code 3 Register
#define QUAD_IO_FAST_RD_CMD_CODE					(0x000000ECU)

// Flash Command Code 2 Register
#define  STD_IO_RD_CMD_CODE							(0x00000013U)
#define  STD_IO_FAST_RD_CMD_CODE		            (0x0000000C)

// Flash Command Code 7 Register
#define ENTER_4_BYTE_ADDR_MODE_CMD_CODE				(0xB7000000U)
#define EXIT_4_BYTE_ADDR_MODE_CMD_CODE				(0x008C0000U)
#define ENTER_QUAD_IO_MODE_CMD_CODE					(0x1EU)
#define RESET_QUAD_IO_CMD_CODE						(0x1FU)
#define FLASH_CMD_CODE_7_REG_CMD_QUAD_EN			(0x00003500U)
#define FLASH_CMD_CODE_7_REG_CMD_QUAD_RST			(0x000000F5U)

#define PKT_HDR_0_CMD_QUAD_EN						(0x00000079U)
#define PKT_HDR_1_CMD_QUAD_EN						(0x00000000U)
#define PKT_HDR_0_CMD_QUAD_RST						(0x0000007DU)
#define PKT_HDR_1_CMD_QUAD_RST						(0x0000002AU)

#define FLASH_ADDR_SHIFT				(8U)
#define FLASH_ADDR_MSB				    (0)

#define EN_FRAME_END_DONE_CNTR		    (0x00000U)
#define EN_FLASH_ADDR_SPACE_MAP		    (0x00000U)

#define LITTLE_ENDIAN					(0x00000U)
#define BIG_ENDIAN					    (0x02000U)

#define DIV_BY_2				        (0x0100U)
#define DIV_BY_4				        (0x0200U)
#define DIV_BY_8				        (0x0300U)

#define CHIP_SEL_BEH				    (0x000U)

#define MIN_IDLE_TIME				    (0x000U)

#define EN_BACK_TO_BACK_TRANS			(0x00U)

#define SAMPLE_ODD_EDGES				(0x00U)
#define SAMPLE_EVEN_EDGES				(0x04U)

#define ACTIVE_LOW						(0x00U)
#define ACTIVE_HIGH						(0x02U)

#define MSB_FIRST						(0x00U)
#define LSB_FIRST						(0x01U)

// Packet header 0
#define XFER_LEN_BYTES                  (0x00000000U)
#define NUM_WAIT_STATE                  (0x00000U)
#define MULT_FLASH_TARGET               (0x000U)
#define FLASH_CMD_CODE                  (0x070U)
#define WITH_PAYLOAD                    (0x00U)
#define SUPPORTED_FLASH_CMD             (0x01U)

// Packet header 1
#define FLASH_ADDRESS_WIDTH             (0x00000U)
#define TARGET_CS		                (0x00000U)
#define DATA_LANE_WIDTH		            (0x000U)
#define ADDRESS_LANE_WIDTH		        (0x00U)
#define COMMAND_LANE_WIDTH		        (0x00U)

#define QSPI_FETCH_SEQ						7
#define GOLDEN_COPY_MEMORY_START_ADDR		0x02000000
#define PRIMARY_COPY_MEMORY_START_ADDR		0x020A0000
#define MSB_MEMORY_START_ADDR			0x0
#define FLASH_ADDR_WIDTH				0x3
#define LANE_WIDTH_X4					0x2

#define FLASH_CMD_CODE_STD_RD					0x0B
#define NUM_WAIT_CYCLE_STD					0x8
#define LANE_WIDTH_X1						0x0

#define QSPI_FW_FETCH_SIZE					0xf000
#define QSPI_FW_WR_SIZE						0x100

/* lattice qspi register set */
struct lattice_qspi_regs {
	/* Configuration Registers */
	u32 reserved;				/* 0x00 */
	u32 config_reg_0;			/* 0x04 */
	u32 config_reg_1;			/* 0x08 */
	u32 flash_cmd_code_0;		/* 0x0C */
	u32 flash_cmd_code_1;		/* 0x10 */
	u32 flash_cmd_code_2;		/* 0x14 */
	u32 flash_cmd_code_3;		/* 0x18 */
	u32 flash_cmd_code_4;		/* 0x1C */
	u32 flash_cmd_code_5;		/* 0x20 */
	u32 flash_cmd_code_6;		/* 0x24 */
	u32 flash_cmd_code_7;		/* 0x28 */
	u32 min_flash_addr_align;	/* 0x2C */
	u32 start_flash_addr;		/* 0x30 */
	u32 flash_mem_map_size;		/* 0x34 */
	u32 axi_mem_map;			/* 0x38 */
	u32 reserved1[49];

	/* Status and Interrupt Registers */
	u32 trans_status;			/* 0x100 */
	u32 int_status;				/* 0x104 */
	u32 int_en;					/* 0x108 */
	u32 int_set;				/* 0x10c */
	u32 supp_flash_cmd_cnt ;	/* 0x110 */
	u32 generic_flash_cmd_cnt;	/* 0x114 */
	u32 reserved2[58];

	/* Control Registers */
	u32 tx_fifo_mapping;		/* 0x200 */
	u32 rx_fifo_mapping;		/* 0x204 */
	u32 packet_header_0;		/* 0x208 */
	u32 packet_header_1;		/* 0x20c */
	u32 packet_header_2;		/* 0x210 */
	u32 packet_header_3;		/* 0x214 */
	u32 packet_data_0;			/* 0x218 */
	u32 packet_data_1;			/* 0x21c */
	u32 start_trans;			/* 0x220 */
	u32 en_looping_test;		/* 0x224 */
	u32 reserved3[54];
};

/* lattice qspi platform data */
struct lattice_qspi_plat {
	struct lattice_qspi_regs *regs;
	u32 frequency;          /* input frequency */
	u32 speed_hz;
};

/* lattice qspi priv */
struct lattice_qspi_priv {
	struct lattice_qspi_regs *regs;
	u8 cs;
	u8 mode;
	u32 freq;		/* required frequency */
	u32 max_hz;
	const void *tx_buf;
	void *rx_buf;
	u32 len;
	int bytes_to_transfer;
	int bytes_to_receive;
	u32 is_inst;
	unsigned cs_change:1;
};

struct qspi_params_ext_t {
	u32 supp_flash_cmd : 1; // h0
	u32 with_payload : 1;
	u32 flash_cmd_code : 5;
	u32 mult_flash_tgt : 1;
	u32 num_wait_cycle : 8;
	u32 buff_length : 16;
	u32 cmd_lane_width : 2;
	u32 addr_lane_width : 2;
	u32 data_lane_width : 2;
	u32 reservedbits_2 : 2;
	u32 tgt_cs : 5;
	u32 flash_addr_width : 3;
	u32 reservedbits_1 : 16;
	u32 flash_addr_LSB : 32;
	u32 flash_addr_MSB : 32;
};

struct qspi_params_reg_t {
	u32 packet_header0;
	u32 packet_header1;
	u32 packet_header2;
	u32 packet_header3;
};

struct qspi_params_t {
	struct qspi_params_ext_t params_ext_t;
	struct qspi_params_reg_t params_reg_t;
};

static struct qspi_params_t pkt_param;
flash_info_t flash_info[CONFIG_SYS_MAX_FLASH_BANKS];	/* FLASH chips info */

static int lattice_qspi_of_to_plat(struct udevice *dev)
{
	fdt_addr_t addr;
	struct lattice_qspi_plat *plat = dev_get_plat(dev);

	addr = dev_read_addr(dev);
	if (addr == FDT_ADDR_T_NONE)
		return -EINVAL;

	plat->regs = (void __iomem *)addr;

	return 0;
}

void flash_print_info(flash_info_t *info)
{
}

int flash_erase(flash_info_t *info, int s_first, int s_last)
{
	return 0;
}

unsigned long flash_init(void)
{
	return SZ_128M;
}

int write_buff(flash_info_t *info, uchar *src, ulong addr, ulong cnt)
{
	return 0;
}

static void lattice_qspi_reset_quad_mode(struct lattice_qspi_priv *priv)
{
	struct lattice_qspi_regs *regs = priv->regs;

	writel(FLASH_CMD_CODE_7_REG_CMD_QUAD_RST, &regs->flash_cmd_code_7);
	writel(PKT_HDR_0_CMD_QUAD_RST, &regs->packet_header_0);
	writel(PKT_HDR_1_CMD_QUAD_RST, &regs->packet_header_1);
	writel(START_TRANS_VAL, &regs->start_trans);
	writel(CLR_INTERRUPT, &regs->int_status);
	writel(CLR_START_TRANS, &regs->start_trans);
}

static void lattice_qspi_enable_quad_mode(struct lattice_qspi_priv *priv)
{
	struct lattice_qspi_regs *regs = priv->regs;

	writel(FLASH_CMD_CODE_7_REG_CMD_QUAD_EN, &regs->flash_cmd_code_7);
	writel(PKT_HDR_0_CMD_QUAD_EN, &regs->packet_header_0);
	writel(PKT_HDR_1_CMD_QUAD_EN, &regs->packet_header_1);
	writel(START_TRANS_VAL, &regs->start_trans);
	writel(CLR_INTERRUPT, &regs->int_status);
	writel(CLR_START_TRANS, &regs->start_trans);
}

static void lattice_qspi_set_pkt_hdr(struct lattice_qspi_priv *priv,
				     struct qspi_params_t *this_pkt_hdr)
{
	struct lattice_qspi_regs *regs = priv->regs;

	writel(this_pkt_hdr->params_reg_t.packet_header0, &regs->packet_header_0);
	writel(this_pkt_hdr->params_reg_t.packet_header1, &regs->packet_header_1);
	writel(this_pkt_hdr->params_reg_t.packet_header2, &regs->packet_header_2);
	writel(this_pkt_hdr->params_reg_t.packet_header3, &regs->packet_header_3);
}

static void lattice_qspi_init(struct lattice_qspi_priv *priv)
{
	struct lattice_qspi_regs *regs = priv->regs;

	pkt_param.params_ext_t.buff_length = QSPI_FW_FETCH_SIZE;
	pkt_param.params_ext_t.mult_flash_tgt = MULT_FLASH_TGT;
	pkt_param.params_ext_t.with_payload = WITH_PAYLOAD_RD;
	pkt_param.params_ext_t.supp_flash_cmd = SUPP_FLASH_CMD;
	pkt_param.params_ext_t.flash_addr_LSB = GOLDEN_COPY_MEMORY_START_ADDR;
	pkt_param.params_ext_t.flash_addr_MSB = MSB_MEMORY_START_ADDR;
	pkt_param.params_ext_t.flash_addr_width = FLASH_ADDR_WIDTH;
	pkt_param.params_ext_t.tgt_cs = TGT_CS;

	if (IS_ENABLED(CONFIG_SPI_FLASH_MACRONIX)) {
		/* QUAD_SPI_MODE - support Macronix flash only */
		pkt_param.params_ext_t.flash_cmd_code = FLASH_CMD_CODE_READ;
		pkt_param.params_ext_t.num_wait_cycle = NUM_WAIT_CYCLE;
		pkt_param.params_ext_t.data_lane_width = LANE_WIDTH_X4;
		pkt_param.params_ext_t.addr_lane_width = LANE_WIDTH_X4;
		pkt_param.params_ext_t.cmd_lane_width = LANE_WIDTH_X4;
	} else {
		/* STANDARD_SPI_MODE - support Winbond flash only */
		pkt_param.params_ext_t.flash_cmd_code = FLASH_CMD_CODE_STD_RD;
		pkt_param.params_ext_t.num_wait_cycle = NUM_WAIT_CYCLE_STD;
		pkt_param.params_ext_t.data_lane_width = LANE_WIDTH_X1;
		pkt_param.params_ext_t.addr_lane_width = LANE_WIDTH_X1;
		pkt_param.params_ext_t.cmd_lane_width = LANE_WIDTH_X1;
	}

	writel(CLR_INTERRUPT, &regs->int_status);
	writel(EN_FRAME_END_DONE_CNTR | EN_FLASH_ADDR_SPACE_MAP | BIG_ENDIAN |
	       DIV_BY_4 | CHIP_SEL_BEH | MIN_IDLE_TIME |
	       EN_BACK_TO_BACK_TRANS | SAMPLE_EVEN_EDGES | ACTIVE_HIGH |
	       MSB_FIRST, &regs->config_reg_0);
	writel(TGT_RD_TRANS_CNT, &regs->config_reg_1);

	if (IS_ENABLED(CONFIG_SPL_BUILD)) {
		/* FLASH_ADDR_32BIT (4 BYTE ADDRESSING) */
		writel(ENTER_4_BYTE_ADDR_MODE_CMD_CODE, &regs->flash_cmd_code_7);
		writel(XFER_LEN_BYTES | NUM_WAIT_STATE | MULT_FLASH_TARGET |
			   FLASH_CMD_CODE | WITH_PAYLOAD | SUPPORTED_FLASH_CMD,
			   &regs->packet_header_0);
		writel(FLASH_ADDRESS_WIDTH | TARGET_CS | DATA_LANE_WIDTH |
			   ADDRESS_LANE_WIDTH | COMMAND_LANE_WIDTH,
			   &regs->packet_header_1);
		writel(START_TRANS_VAL, &regs->start_trans);
		writel(CLR_INTERRUPT, &regs->int_status);
		writel(CLR_START_TRANS_VAL, &regs->start_trans);

		if (IS_ENABLED(CONFIG_SPI_FLASH_MACRONIX)) {
			/* QUAD_SPI_MODE - support Macronix flash only */
			lattice_qspi_enable_quad_mode(priv);
			writel(QUAD_IO_FAST_RD_CMD_CODE, &regs->flash_cmd_code_3);
		} else {
			/* STANDARD_SPI_MODE - support Winbond flash only */
			lattice_qspi_reset_quad_mode(priv);
			writel(STD_IO_FAST_RD_CMD_CODE, &regs->flash_cmd_code_2);
		}
	}

	/* disable interrupt */
	writel(INT_DIS_VAL, &regs->int_en);
	/* enable interrupt */
	writel(INT_EN_VAL, &regs->int_en);
}

static void lattice_qspi_read_xfer(struct mtd_info *mtd, loff_t from,
				   size_t len, u32 *buffer)
{
	int i;
	u32 tmp;
	struct udevice *dev = mtd->dev;
	struct lattice_qspi_priv *priv = dev_get_priv(dev);
	struct lattice_qspi_regs *regs = priv->regs;
	struct qspi_params_t *this_qspi_pkt = &pkt_param;
	u32 *read_buffer = buffer;

	tmp = 0;
	pkt_param.params_ext_t.flash_addr_LSB = from;
	pkt_param.params_ext_t.buff_length = len;

	/* write packet headers */
	lattice_qspi_set_pkt_hdr(priv, this_qspi_pkt);

	/* interrupt enable */
	writel(INT_EN_VAL, &regs->int_en);

	/* start transaction */
	writel(CLR_START_TRANS, &regs->start_trans);
	writel(START_TRANS_VAL, &regs->start_trans);

	for (i = 0; i < len / 4; i++) {
		/* check the interrupt */
		//while (((readl(&regs->int_status) >> 4) & 0x1) == IRQ_RD_FIFO_DIS);
		readl_poll_timeout(&regs->int_status, tmp, ~((tmp  >> 4) & 0x1), 10);
		/* clear the interrupt */
		writel(CLR_INTERRUPT, &regs->int_status);
		/* read the data packet */
		*(read_buffer + i) = readl(&regs->packet_data_0);
	}

	writel(CLR_START_TRANS, &regs->start_trans);
}

static int lattice_qspi_read_fifo_dis(struct mtd_info *mtd, loff_t from, size_t len,
				      size_t *retlen, u_char *buf)
{
	struct udevice *dev = mtd->dev;
	struct lattice_qspi_priv *priv = dev_get_priv(dev);
	struct lattice_qspi_regs *regs = priv->regs;
	u32 *buffer = (u32 *)buf;
	int cycle, rem;

	cycle = 0;
	rem = 0;

	if (IS_ENABLED(CONFIG_SPI_FLASH_MACRONIX)) {
		/* Macronix flash */
		pkt_param.params_ext_t.cmd_lane_width = LANE_WIDTH_X4;
		pkt_param.params_ext_t.flash_cmd_code = FLASH_CMD_CODE_READ;
	} else {
		/* Winbond flash */
		pkt_param.params_ext_t.cmd_lane_width = LANE_WIDTH_X1;
		pkt_param.params_ext_t.flash_cmd_code = FLASH_CMD_CODE_STD_RD;
	}

	pkt_param.params_ext_t.with_payload = WITH_PAYLOAD_RD;

	if (len >= QSPI_FW_FETCH_SIZE) {
		cycle = len / QSPI_FW_FETCH_SIZE;
		rem = len % QSPI_FW_FETCH_SIZE;
	} else {
		rem = len;
	}

	/* Performing read transaction */
	while (cycle > 0) {
		lattice_qspi_read_xfer(mtd, from, QSPI_FW_FETCH_SIZE, buffer);
		from += QSPI_FW_FETCH_SIZE;
		buffer += QSPI_FW_FETCH_SIZE / 4;
		cycle--;
	}

	if (rem != 0)
		lattice_qspi_read_xfer(mtd, from, rem, buffer);

	/* disable interrupt */
	writel(INT_DIS_VAL, &regs->int_en);
	*retlen = len;

	return 0;
}

static void lattice_qspi_write_xfer(struct mtd_info *mtd, loff_t to,
				    size_t len, u32 *buffer)
{
	struct udevice *dev = mtd->dev;
	struct lattice_qspi_priv *priv = dev_get_priv(dev);
	struct lattice_qspi_regs *regs = priv->regs;
	struct qspi_params_t *this_qspi_pkt = &pkt_param;
	u32 *write_buffer = buffer;
	int i, count;
	u32 tmp;

	tmp = 0;
	count = len / 4;

	/* clear transfer */
	writel(CLR_START_TRANS, &regs->start_trans);

	pkt_param.params_ext_t.flash_addr_LSB = to;
	pkt_param.params_ext_t.buff_length = len;

	/* write packet headers */
	lattice_qspi_set_pkt_hdr(priv, this_qspi_pkt);

	/* interrupt enable */
	writel(WR_INT_EN_VAL, &regs->int_en);

	for (i = 0; i < len / 4; i += 2) {
		count -= 2;

		/* write the data */
		writel(*(write_buffer + i), &regs->packet_data_0);
		debug("packet_data_0 = %x\n", regs->packet_data_0);
		writel(*(write_buffer + i + 1), &regs->packet_data_1);
		debug("packet_data_1 = %x\n", regs->packet_data_1);

		/* start transfer */
		if ((readl(&regs->start_trans) & 0x1) != START_TRANS_VAL)
			writel(START_TRANS_VAL, &regs->start_trans);
		/* check the transaction status */
		//while ((readl(&regs->trans_status) & 0x1) != TRANS_STS);

		/* check the interrupt
		 * 0x0 - new data can be written
		 * 0x1 - data can be transmitted
		 */
		if (count) {
			readl_poll_timeout(&regs->int_status, tmp,
					   (tmp & ~IRQ_WR_FIFO_DIS), 10);
			readl_poll_timeout(&regs->int_status, tmp,
					   ((tmp >> 1) & ~IRQ_WR_FIFO_DIS), 10);
			/* clear the interrupt */
			writel(CLR_INTERRUPT, &regs->int_status);
		}
	}

	writel(CLR_START_TRANS, &regs->start_trans);
	/* clear the interrupt */
	writel(CLR_INTERRUPT, &regs->int_status);

	debug("transfer is done\n");
}

static int lattice_qspi_write_fifo_dis(struct mtd_info *mtd, loff_t to, size_t len,
				       size_t *retlen, const u_char *buf)
{
	struct udevice *dev = mtd->dev;
	struct lattice_qspi_priv *priv = dev_get_priv(dev);
	struct lattice_qspi_regs *regs = priv->regs;
	u32 *buffer = (u32 *)buf;
	int cycle, rem;

	cycle = 0;
	rem = 0;

	pkt_param.params_ext_t.cmd_lane_width = LANE_WIDTH_X1;
	pkt_param.params_ext_t.with_payload = WITH_PAYLOAD_WR;
	pkt_param.params_ext_t.flash_cmd_code = FLASH_CMD_CODE_WRITE_STD;

	if (len >= QSPI_FW_WR_SIZE) {
		cycle = len / QSPI_FW_WR_SIZE;
		rem = len % QSPI_FW_WR_SIZE;
	} else {
		rem = len;
	}

	/* Performing read transaction */
	while (cycle > 0) {
		lattice_qspi_write_xfer(mtd, to, QSPI_FW_WR_SIZE, buffer);
		to += QSPI_FW_WR_SIZE;
		buffer += QSPI_FW_WR_SIZE / 4;
		cycle--;
	}

	if (rem != 0)
		lattice_qspi_write_xfer(mtd, to, rem, buffer);

	/* disable interrupt */
	writel(INT_DIS_VAL, &regs->int_en);
	*retlen = len;

	return 0;
}

static int lattice_qspi_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	u32 tmp;
	struct udevice *dev = mtd->dev;
	struct lattice_qspi_priv *priv = dev_get_priv(dev);
	struct lattice_qspi_regs *regs = priv->regs;
	struct qspi_params_t *this_qspi_pkt = &pkt_param;
	size_t addr = instr->addr;
	size_t len = instr->len;
	size_t end = addr + len;

	pkt_param.params_ext_t.data_lane_width = LANE_WIDTH_X1;
	pkt_param.params_ext_t.addr_lane_width = LANE_WIDTH_X1;
	pkt_param.params_ext_t.cmd_lane_width = LANE_WIDTH_X1;
	pkt_param.params_ext_t.with_payload = WITH_PAYLOAD_RD;
	pkt_param.params_ext_t.flash_cmd_code = FLASH_CMD_CODE_ERASE;
	pkt_param.params_ext_t.num_wait_cycle = 0;

	pkt_param.params_ext_t.flash_addr_LSB = addr;
	pkt_param.params_ext_t.buff_length = 0;

	instr->state = MTD_ERASING;

	while (addr < end) {
		pkt_param.params_ext_t.flash_addr_LSB = addr;
		/* write packet headers */
		lattice_qspi_set_pkt_hdr(priv, this_qspi_pkt);

		/* interrupt enable */
		writel(INT_EN_VAL, &regs->int_en);

		/* start transfer */
		writel(START_TRANS_VAL, &regs->start_trans);

		/* check the transaction status */
		//while ((readl(&regs->trans_status) & 0x1) == TRANS_STS);
		readl_poll_timeout(&regs->trans_status, tmp, ~(tmp & 0x1), 10);

		/* clear transaction */
		writel(CLR_START_TRANS, &regs->start_trans);
		addr += mtd->erasesize;
	}

	/* disable interrupt */
	writel(INT_DIS_VAL, &regs->int_en);
	instr->state = MTD_ERASE_DONE;

	return 0;
}

static int lattice_qspi_probe(struct udevice *dev)
{
	struct lattice_qspi_plat *plat = dev_get_plat(dev);
	struct lattice_qspi_priv *priv = dev_get_priv(dev);
	struct mtd_info *mtd;
	flash_info_t *flash = &flash_info[0];
	int i;

	priv->regs = plat->regs;

	mtd = dev_get_uclass_priv(dev);
	mtd->dev = dev;
	mtd->name		= "lattice_qspi";
	mtd->type		= MTD_NORFLASH;
	mtd->flags		= MTD_CAP_NORFLASH;
	/*128MB QSOI flash */
	mtd->size		= SZ_128M;
	mtd->writesize		= 1;
	mtd->writebufsize	= mtd->writesize;
	mtd->_erase		= lattice_qspi_erase;
	mtd->_read		= lattice_qspi_read_fifo_dis;
	mtd->_write		= lattice_qspi_write_fifo_dis;
	mtd->numeraseregions = 0;
	mtd->erasesize = 0x10000;
	if (add_mtd_device(mtd))
		return -ENOMEM;

	flash->mtd = mtd;
	flash->size = mtd->size;
	flash->sector_count = mtd->size / mtd->erasesize;
	/* Macronix flash ID */
	//flash->flash_id = 0xc2201a;
	/* Winbond dlash ID */
	//flash->flash_id = 0xef4020;
	flash->start[0] = 0;
	for (i = 1; i < flash->sector_count; i++)
		flash->start[i] = flash->start[i - 1] + mtd->erasesize;

	gd->bd->bi_flashstart = 0;

	lattice_qspi_init(priv);
	return 0;
}

static const struct udevice_id lattice_qspi_ids[] = {
	{ .compatible = "lattice,lattice-qspi" },
	{ }
};

U_BOOT_DRIVER(lattice_qspi) = {
	.name   = "lattice_qspi",
	.id     = UCLASS_MTD,
	.of_match = lattice_qspi_ids,
	.of_to_plat = lattice_qspi_of_to_plat,
	.plat_auto	= sizeof(struct lattice_qspi_plat),
	.priv_auto	= sizeof(struct lattice_qspi_priv),
	.probe  = lattice_qspi_probe,
};
