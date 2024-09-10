/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2023-2024 Lattice Semiconductor Corporation
 * <www.latticesemi.com>
 */

#ifndef _LPDDR4_NEXUS_H_
#define _LPDDR4_NEXUS_H_

/* Driver version */
#define LPDDR4_DRV_VER "v24.01.00."
/**
 * @brief : Control Register name, address and offset
 */

#define FEATURE_CTRL_REG			(0x00U)
#define RESET_REG					(0x04U)
#define SETTINGS_REG				(0x08U)
#define RSVD_1                      (0x0CU)
#define INT_STATUS_REG				(0x10U)
#define INT_ENABLE_REG				(0x14U)
#define INT_SET_REG					(0x18U) /* debug purpose only */
#define CLK_CHANGE_REG              (0x01C)
#define TRN_OP_REG					(0x20U)
#define STATUS_REG					(0x24U)
#define SUCCESS						1
#define	FAILURE						0
#define IS_NULL						0U
#define OUT_OF_RESET				(0x03U)
#define LPDDR_DONE_BITS				(0x0000001F)
#define LPDDR_ERR_DONE_BITS			(0x00000F1F)
#define PLL_LOCK_BIT             	(0x08U)
#define TRN_EN		             	(0x5FU)

typedef struct{
	unsigned int lpddr4_base_address;
}lpddr4;

/**
 * @brief : Feature Control Register (FEATURE_CTRL_REG)
 */
typedef union{
	struct{
		unsigned int fea_rsvd_1 : 1;
		unsigned int dbi_en : 1;
		unsigned int power_down_en : 1;
		unsigned int gear_ratio : 1;
		unsigned int addr_translation : 4;
		unsigned int ddr_type : 4;
		unsigned int ddr_width : 4;
		unsigned int num_ranks : 1;
		unsigned int fea_rsvd_2 : 15;
	}fields;
	unsigned int reg;
}feature_ctrl_reg_t;

/**
 * @brief : Reset Register (RESET_REG)
 */
typedef union {
	struct {
		unsigned int trn_eng_rst_n : 1;
		unsigned int cpu_reset_n : 1;
		unsigned int res_rsvd : 30;
	} fields;
	unsigned int reg;
}reset_reg_t;

/**
 * @brief : Settings Register (SETTINGS_REG)
 */
typedef union {
	struct {
		unsigned int write_latency : 4;
		unsigned int set_rsvd_1 : 4;
		unsigned int read_latency : 4;
		unsigned int set_rsvd_2 : 4;
		unsigned int clk_freq : 12;
		unsigned int set_rsvd_3 : 4;
	} fields;
	unsigned int reg;
}settings_reg_t;

/**
 * @brief : Interrupt Status Register (INT_STATUS_REG)
 */
typedef union {
	struct {
		 unsigned int trn_done_int : 1;
		 unsigned int trn_err_int : 1;
		 unsigned int int_sts_rsvd_1 : 2;
		 unsigned int temp_change_int : 1;
		 unsigned int int_sts_rsvd_2 : 27;
	} fields;
	unsigned int reg;
}int_status_reg_t;

/**
 * @brief : Interrupt Enable Register (INT_ENABLE_REG)
 */
typedef union {
	struct {
		unsigned int trn_done_en : 1;
		unsigned int trn_err_en : 1;
		unsigned int int_en_rsvd_1 : 1;
		unsigned int temp_change_int : 1;
		unsigned int int_en_rsvd_2 : 27;
	}fields;
	unsigned int reg;
}int_enable_reg_t;

/**
 * @brief : Clock Register (CLK_CHANGE_REG)
 */
typedef union {
	struct {
		 unsigned int ddr_clk_sel : 1;
		 unsigned int dll_update_en : 1;
		 unsigned int clk_update : 1;
		 unsigned int pll_lock : 1;
		 unsigned int clk_chng_rsvd : 28;
	} fields;
	unsigned int reg;
}clk_chng_t;

/**
 * @brief : Training Operation Register (TRN_OP_REG)
 */
typedef union {
	struct {
		unsigned int init_en : 1;
		unsigned int cbt_en : 1;
		unsigned int write_lvl_en : 1;
		unsigned int read_trn_en : 1;
		unsigned int write_trn_en : 1;
		unsigned int ca_vref_training_en : 1;
		unsigned int mc_vref_training_en : 1;
		unsigned int mem_vref_training_en : 1;
		unsigned int trn_op_rsvd1 : 24;
	}fields;
	unsigned int reg;
}trn_op_reg_t;

/**
 * @brief : Status Register (STATUS_REG)
 */
typedef union {
	struct {
		 unsigned int phy_ready : 1;
		 unsigned int cbt_done : 1;
		 unsigned int write_lvl_done : 1;
		 unsigned int read_trn_done : 1;
		 unsigned int write_trn_done : 1;
		 unsigned int in_self_refresh : 2;
		 unsigned int sts_rsvd1 : 1;
		 unsigned int cbt_err : 1;
		 unsigned int write_lvl_err : 1;
		 unsigned int read_trn_err : 1;
		 unsigned int write_trn_err : 1;
		 unsigned int err_on_rank : 2;
		 unsigned int sts_rsvd2 : 2;
		 unsigned int refresh_rate : 3;
		 unsigned int rank0_done : 1;
		 unsigned int rank1_done : 1;
		 unsigned int sts_rsvd3 : 11;
	} fields;
	unsigned int reg;
}status_reg_t;

/**
 * @brief : Enum LPDDR Training Error codes
 */
typedef enum {
	NO_FAIL = 0,
	CBT_FAIL,
	WR_LVL_FAIL,
	RD_TRN_FAIL,
	WR_TRN_FAIL,
	OTHER_FAIL
}LPDDR_RET;

/* APIs */
LPDDR_RET lpddr4_init(lpddr4 *instancePtr, unsigned int base_addr);
unsigned int lpddr4_GetFeatureControlReg(lpddr4 *instancePtr, unsigned int *reg_data);
unsigned int lpddr4_GetSettingReg(lpddr4 *instancePtr, unsigned int *reg_data);
unsigned int lpddr4_TemperatureChangeInterruptEnable(lpddr4 *instancePtr);
unsigned int lpddr4_TemperatureChangeInterruptDisable(lpddr4 *instancePtr);
unsigned int lpddr4_TrainingErrorInterruptEnable(lpddr4 *instancePtr);
unsigned int lpddr4_TrainingErrorInterruptDisable(lpddr4 *instancePtr);
unsigned int lpddr4_TrainingDoneInterruptEnable(lpddr4 *instancePtr);
unsigned int lpddr4_TrainingDoneInterruptDisable(lpddr4 *instancePtr);
unsigned int lpddr4_GetTrainingOperationReg(lpddr4 *instancePtr, unsigned int *reg_data);
unsigned int lpddr4_GetStatusReg(lpddr4 *instancePtr, unsigned int *reg_data);

#endif
