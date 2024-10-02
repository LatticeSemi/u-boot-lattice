// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 Lattice Semiconductor Corporation
 * <www.latticesemi.com>
 *
 */
#include <lattice/lpddr4_nexus.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>

static struct lpddr4 *lpddr4_inst;

/**
 * @brief :	lpddr4_write32 - This API is used to write data to a
 *		specific register.
 * @param :	base_address - base address of a LPDDR IP.
 *		offset - address offset of a specific register.
 *		val - value to be written into specific register.
 * @return :	Value written to register.
 */
unsigned int lpddr4_write32(unsigned int base_address, unsigned int offset,
			    unsigned int val)
{
	unsigned int *const reg = (unsigned int *)(base_address + offset);
	*reg = val;
	return *reg;
}

/**
 * @brief :	lpddr4_read32 - This API is used to read data to a
 *		specific register.
 * @param :	base_address - base address of a LPDDR IP.
 *		offset - address offset of a specific register.
 * @return :	Value read from register.
 */
unsigned int lpddr4_read32(unsigned int base_address, unsigned int offset)
{
	unsigned int *const reg = (unsigned int *)(base_address + offset);
	return *reg;
}

/**
 * @brief :	lpddr4_init - This API is used to initialize LPDDR IP.
 *		This will start the initialization and training sequence of the
 *		LPDDR4 SDRAM device.
 *		After all sequence steps are complete, the control of the PHY
 *		is transferred to the Memory Controller and the user can now
 *		start accessing the LPDDR4 memory through the data interface.
 *		Note: This API must be called at bootup time/BSP initialization
 *		phase if user wants to use external LPDDR4 SDRAM.
 *
 * @param :	instance_ptr [In] - Handle of the lpddr4 structure.
 *		base_address [In] - base address of a LPDDR IP (from sys_platform.h)
 * @return :	Success or Training error codes.
 */
enum LPDDR_RET lpddr4_init(struct lpddr4 *instance_ptr, unsigned int base_addr)
{
	enum LPDDR_RET ret = NO_FAIL;
	int delay;
	union status_reg_t sts_reg;
	union clk_chng_t clk_chng_reg;

	if (!instance_ptr)
		return FAILURE;

	instance_ptr->lpddr4_base_address = base_addr;
	lpddr4_inst = instance_ptr;
	lpddr4_inst->lpddr4_base_address = instance_ptr->lpddr4_base_address;

	/* Waiting until PLL is locked */
	do {
		clk_chng_reg = (union clk_chng_t)lpddr4_read32
					(lpddr4_inst->lpddr4_base_address,
					 CLK_CHANGE_REG);
	} while ((clk_chng_reg.reg & PLL_LOCK_BIT) != PLL_LOCK_BIT);
	printf("\r\nINFO: DDR PLL Locked");
	/* Set LPDDR4 training sequence parameters */
	lpddr4_write32(lpddr4_inst->lpddr4_base_address, TRN_OP_REG, TRN_EN);
	/* Bring LPDDR out of reset to start training sequence */
	lpddr4_write32(lpddr4_inst->lpddr4_base_address, RESET_REG, OUT_OF_RESET);
	/* Reading LPDDR4 Status register */
	sts_reg = (union status_reg_t)lpddr4_read32
			(lpddr4_inst->lpddr4_base_address, STATUS_REG);

	while ((sts_reg.reg & LPDDR_ERR_DONE_BITS) != LPDDR_DONE_BITS) {
		sts_reg = (union status_reg_t)lpddr4_read32
				(lpddr4_inst->lpddr4_base_address, STATUS_REG);
		//printf("\r\nINFO:DDR status = %x\n", sts_reg);
		if (sts_reg.status_fields.phy_ready == 1) {
			if (sts_reg.status_fields.cbt_err == 1) {
				printf("\r\nErr:LPDDR CBT_FAIL ocurred\n");
				return CBT_FAIL;
			}
			if (sts_reg.status_fields.write_lvl_err == 1) {
				printf("\r\nErr:LPDDR WR_LVL_FAIL ocurred\n");
				return WR_LVL_FAIL;
			}
			if (sts_reg.status_fields.read_trn_err == 1) {
				printf("\r\nErr:LPDDR RD_TRN_FAIL ocurred\n");
				return RD_TRN_FAIL;
			}
			if (sts_reg.status_fields.write_trn_err == 1) {
				printf("\r\nErr:LPDDR WR_TRN_FAIL ocurred\n");
				return WR_TRN_FAIL;
			}
		} else {
			/* Mandatory delay to sync with hardware training engine */
			delay = TWO_US_DELAY;
			while (--delay)
				__asm__ __volatile__("nop");
		}
	}
	printf("\r\nINFO: LPDDR4 training complete\r\n");
	printf("LPDDR4 training status = %x \r\nINFO: LPDDR4 initialized successfully\n",
	       sts_reg.reg);
	return ret;
}

/**
 * @brief :	lpddr4_GetFeatureControlReg - This API is used to read
 *		Feature Control Register (FEATURE_CTRL_REG).
 *		Refer IP user guide for a register description.
 * @param :	instance_ptr [In] - Handle of the lpddr4 structure.
 *		reg_data [Out] - Value read from register.
 * @return :	Success or Failure.
 */
unsigned int lpddr4_GetFeatureControlReg(struct lpddr4 *inst_ptr,
					 unsigned int *reg_data)
{
	if (!inst_ptr)
		return FAILURE;

	*reg_data = lpddr4_read32(inst_ptr->lpddr4_base_address,
				  FEATURE_CTRL_REG);
	return SUCCESS;
}

/**
 * @brief :	lpddr4_GetSettingReg - This API is used to read
 *		Settings Register (SETTINGS_REG).
 *		User set this reg in GUI, only can read here.
 *		Refer IP user guide for a register description.
 * @param :	instance_ptr [In] - Handle of the lpddr4 structure.
 *		reg_data [Out] - Value read from register.
 * @return :	Success or Failure.
 */
unsigned int lpddr4_GetSettingReg(struct lpddr4 *inst_ptr,
				  unsigned int *reg_data)
{
	if (!inst_ptr)
		return FAILURE;

	*reg_data = lpddr4_read32(inst_ptr->lpddr4_base_address,
				  SETTINGS_REG);
	return SUCCESS;
}

/**
 * @brief :	lpddr4_TemperatureChangeInterruptEnable - This API is used to
 *		enable temperature change interrupt.
 *		The Memory Controller periodically reads MR4(Mode Register 4)
 *		and according to the 'Temperature Check Period' attribute,
 *		This interrupt bit asserts when MR4 indicates a temperature
 *		change, and the refresh rate is not equal to 1x refresh.
 *		Refer IP user guide for a register description.
 * @param :	instance_ptr [In] - Handle of the lpddr4 structure.
 * @return :	Success or Failure.
 */
unsigned int lpddr4_TemperatureChangeInterruptEnable(struct lpddr4 *inst_ptr)
{
	union int_enable_reg_t en_reg;

	if (!inst_ptr)
		return FAILURE;

	en_reg = (union int_enable_reg_t)lpddr4_read32
			(inst_ptr->lpddr4_base_address, INT_ENABLE_REG);
	en_reg.int_enable_fields.temp_change_int = 1;
	lpddr4_write32(inst_ptr->lpddr4_base_address,
		       INT_ENABLE_REG, en_reg.reg);
	return SUCCESS;
}

/**
 * @brief :	lpddr4_TemperatureChangeInterruptDisable - This API is used to
 *		disable temperature change interrupt.
 *		Refer IP user guide for a register description.
 * @param :	instance_ptr [In] - Handle of the lpddr4 structure.
 * @return :	Success or Failure.
 */
unsigned int lpddr4_TemperatureChangeInterruptDisable(struct lpddr4 *inst_ptr)
{
	union int_enable_reg_t en_reg;

	if (!inst_ptr)
		return FAILURE;

	en_reg = (union int_enable_reg_t)lpddr4_read32
			(inst_ptr->lpddr4_base_address, INT_ENABLE_REG);
	en_reg.int_enable_fields.temp_change_int = 0;
	lpddr4_write32(inst_ptr->lpddr4_base_address,
		       INT_ENABLE_REG, en_reg.reg);
	return SUCCESS;
}

/**
 * @brief :	lpddr4_TrainingErrorInterruptEnable - This API is used to
 *		enable training error interrupt.
 *		This Interrupt bit asserts when the Training Engine encounters
 *		an error during training. The user should read STATUS_REG to
 *		determine the specific error.
 *		Refer IP user guide for a register description.
 * @param :	instance_ptr [In] - Handle of the lpddr4 structure.
 * @return :	Success or Failure.
 */
unsigned int lpddr4_TrainingErrorInterruptEnable(struct lpddr4 *inst_ptr)
{
	union int_enable_reg_t en_reg;

	if (!inst_ptr)
		return FAILURE;

	en_reg = (union int_enable_reg_t)lpddr4_read32
			(inst_ptr->lpddr4_base_address, INT_ENABLE_REG);
	en_reg.int_enable_fields.trn_err_en = 1;
	lpddr4_write32(inst_ptr->lpddr4_base_address,
		       INT_ENABLE_REG, en_reg.reg);
	return SUCCESS;
}

/**
 * @brief :	lpddr4_TrainingErrorInterruptDisable - This API is used to
 *		disable training error interrupt.
 *		In this case, The user can poll STATUS_REG to determine the
 *		specific error.
 *		Refer IP user guide for a register description.
 * @param :	instance_ptr [In] - Handle of the lpddr4 structure.
 * @return :	Success or Failure.
 */
unsigned int lpddr4_TrainingErrorInterruptDisable(struct lpddr4 *inst_ptr)
{
	union int_enable_reg_t en_reg;

	if (!inst_ptr)
		return FAILURE;

	en_reg = (union int_enable_reg_t)lpddr4_read32
			(inst_ptr->lpddr4_base_address, INT_ENABLE_REG);
	en_reg.int_enable_fields.trn_err_en = 0;
	lpddr4_write32(inst_ptr->lpddr4_base_address,
		       INT_ENABLE_REG, en_reg.reg);
	return SUCCESS;
}

/**
 * @brief :	lpddr4_TrainingDoneInterruptEnable - This API is used to
 *		enable training done interrupt.
 *		This Interrupt bit asserts when initialization and training
 *		is completed successfully.
 *		Refer IP user guide for a register description.
 * @param :	instance_ptr [In] - Handle of the lpddr4 structure.
 * @return :	Success or Failure.
 */
unsigned int lpddr4_TrainingDoneInterruptEnable(struct lpddr4 *inst_ptr)
{
	union int_enable_reg_t en_reg;

	if (!inst_ptr)
		return FAILURE;

	en_reg = (union int_enable_reg_t)lpddr4_read32
			(inst_ptr->lpddr4_base_address, INT_ENABLE_REG);
	en_reg.int_enable_fields.trn_done_en = 1;
	lpddr4_write32(inst_ptr->lpddr4_base_address,
		       INT_ENABLE_REG, en_reg.reg);
	return SUCCESS;
}

/**
 * @brief :	lpddr4_TrainingDoneInterruptDisable - This API is used to
 *		disable training done interrupt.
 *		In this case, The user can poll STATUS_REG.
 *		Refer IP user guide for a register description.
 * @param :	instance_ptr [In] - Handle of the lpddr4 structure.
 * @return :	Success or Failure.
 */
unsigned int lpddr4_TrainingDoneInterruptDisable(struct lpddr4 *inst_ptr)
{
	union int_enable_reg_t en_reg;

	if (!inst_ptr)
		return FAILURE;

	en_reg = (union int_enable_reg_t)lpddr4_read32
			(inst_ptr->lpddr4_base_address, INT_ENABLE_REG);
	en_reg.int_enable_fields.trn_done_en = 0;
	lpddr4_write32(inst_ptr->lpddr4_base_address,
		       INT_ENABLE_REG, en_reg.reg);
	return SUCCESS;
}

/**
 * @brief :	lpddr4_GetTrainingOperationReg - This API is used to read
 *		Training Operation Register (TRN_OP_REG).
 *		Refer IP user guide for a register description.
 * @param :	instance_ptr [In] - Handle of the lpddr4 structure.
 *		reg_data [Out] - Value read from register.
 * @return :	Success or Failure.
 */
unsigned int lpddr4_GetTrainingOperationReg(struct lpddr4 *inst_ptr,
					    unsigned int *reg_data)
{
	if (!inst_ptr)
		return FAILURE;

	*reg_data = lpddr4_read32(inst_ptr->lpddr4_base_address, TRN_OP_REG);
	return SUCCESS;
}

/**
 * @brief :	lpddr4_GetStatusReg - This API is used to read Status Register
 *		(STATUS_REG).
 *		Refer IP user guide for a register description.
 * @param :	instance_ptr [In] - Handle of the lpddr4 structure.
 *		reg_data [Out] - Value read from register.
 * @return :	Success or Failure.
 */
unsigned int lpddr4_GetStatusReg(struct lpddr4 *inst_ptr,
				 unsigned int *reg_data)
{
	if (!inst_ptr)
		return FAILURE;

	*reg_data = lpddr4_read32(inst_ptr->lpddr4_base_address, STATUS_REG);
	return SUCCESS;
}
