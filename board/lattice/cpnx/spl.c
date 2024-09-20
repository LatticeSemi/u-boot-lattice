// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Lattice Semiconductor Corp
 * Author: WeiLiang Lim<weiliang.lim@latticesemi.com>
 */

#include <asm/io.h>
#include <asm/system.h>
#include <lattice/lpddr4_nexus.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <spl.h>

#define LPDDR4_CONTR_BASE_ADDR  0x40007000
#define LPDDR4_OUT_OF_RESET     0x00000003

static void lpddr_init(void)
{
	struct lpddr4 lpddr4_instance;

	lpddr4_init(&lpddr4_instance, LPDDR4_CONTR_BASE_ADDR);
}

#if IS_ENABLED(CONFIG_SPL_LOAD_FIT)
u32 spl_boot_device(void)
{
	return BOOT_DEVICE_MTD;
}
#endif

void board_init_f(ulong dummy)
{
	int ret;

	ret = spl_early_init();
	if (ret)
		panic("spl_early_init() failed: %d\n", ret);

	riscv_cpu_setup();
	preloader_console_init();
	lpddr_init();
}
