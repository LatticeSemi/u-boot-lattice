// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Lattice Semiconductor Corporation
 */

#include <linux/io.h>
#include <lattice/lattice_reg_access.h>

u8 reg_32b_write(void *reg_addr, u32 value)
{
	u32 *reg = reg_addr;

	*reg = value;

	return 0;
}

u8 reg_32b_read(void *reg_addr, u32 *value)
{
	u32 *reg = reg_addr;

	*value = *reg;

	return 0;
}

u8 reg_32b_modify(void *reg_addr, u32 bits_mask, u32 value)
{
	u32 *reg = reg_addr;

	*reg = (*reg & ~bits_mask) | (value & bits_mask);

	return 0;
}

u8 reg_16b_write(void *reg_addr, u16 value)
{
	u16 *reg = reg_addr;

	*reg = value;

	return 0;
}

u8 reg_16b_read(void *reg_addr, u16 *value)
{
	u16 *reg = reg_addr;

	*value = *reg;

	return 0;
}

u8 reg_16b_modify(void *reg_addr, u16 bits_mask, u16 value)
{
	u16 *reg = reg_addr;

	*reg = (*reg & ~bits_mask) | (value & bits_mask);

	return 0;
}

u8 reg_8b_write(void *reg_addr, u8 value)
{
	u8 *reg = reg_addr;

	*reg = value;

	return 0;
}

u8 reg_8b_read(void *reg_addr, u8 *value)
{
	u8 *reg = reg_addr;

	*value = *reg;

	return 0;
}

u8 reg_8b_modify(void *reg_addr, u8 bits_mask, u8 value)
{
	u8 *reg = reg_addr;

	*reg = (*reg & ~bits_mask) | (value & bits_mask);

	return 0;
}
