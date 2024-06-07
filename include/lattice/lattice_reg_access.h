/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2019-2024 by Lattice Semiconductor Corporation
 */

#ifndef LATTICE_REG_ACCESS_H
#define LATTICE_REG_ACCESS_H

#include <linux/types.h>

/*
 * reg_32b_write is used to write the content of a 32 bits wide peripheral
 * register.
 *
 * @param reg_addr  Address in the processor's memory map of the register to
 *                  write.
 * @param value     Value to be written into the peripheral register.
 */
u8 reg_32b_write(void *reg_addr, u32 value);

/*
 * reg_32b_read is used to read the content of a 32 bits wide peripheral
 * register.
 *
 * @param reg_addr  Address in the processor's memory map of the register to
 *                  read.
 * @return          32 bits value read from the peripheral register.
 */
u8 reg_32b_read(void *reg_addr, u32 *reg_32b_value);

u8 reg_32b_modify(void *reg_addr, u32 bits_mask, u32 value);

u8 reg_16b_write(void *reg_addr, u16 value);

u8 reg_16b_read(void *reg_addr, u16 *reg_16b_value);

u8 reg_16b_modify(void *reg_addr, u16 bits_mask, u16 value);

u8 reg_8b_write(void *reg_addr, u8 value);

/*
 * reg_8b_read is used to read the content of a 8 bits wide peripheral
 * register.
 *
 * @param reg_addr  Address in the processor's memory map of the register to
 *                  read.
 * @return          8 bits value read from the peripheral register.
 */
u8 reg_8b_read(void *reg_addr, u8 *reg_8b_value);

/*
 * reg_8b_modify is used to modify the masked bits of a 8 bits wide peripheral
 * register.
 *
 * @param reg_addr  Address in the processor's memory map of the register to
 *                  read.
 * @bits_mask       Bits that will be modified within the register
 * @value           The value will be modified to the masked bits of the register
 * @return
 */
u8 reg_8b_modify(void *reg_addr, u8 bits_mask, u8 value);

#endif	/* LATTICE_REG_ACCESS_H */

