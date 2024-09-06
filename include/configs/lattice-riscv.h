/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Lattice Semiconductor Corp
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#define CFG_SYS_SDRAM_BASE		0x80000000

#define RISCV_MMODE_TIMERBASE		0xf2000000
#define RISCV_MMODE_TIMEROFF		0xbff8
#define RISCV_MMODE_TIMER_FREQ		1000000
#define RISCV_SMODE_TIMER_FREQ		1000000

#endif /* __CONFIG_H */
