// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Lattice Semiconductor Corp
 * Author: WeiLiang Lim<weiliang.lim@latticesemi.com>
 */

#include <asm/io.h>
#include <asm/system.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <spl.h>

#define LPDDR4_CONTR_BASE_ADDR  0x40007000
#define LPDDR4_OUT_OF_RESET     0x00000003

static void lpddr_init(void)
{
	void __iomem *reg = (void __iomem *)LPDDR4_CONTR_BASE_ADDR + 0x04;

	/* Bringing LPDDR4 out of reset */
	writel(LPDDR4_OUT_OF_RESET, reg);
	reg = (void __iomem *)LPDDR4_CONTR_BASE_ADDR + 0x24;
	while((readl(reg) & 0x000000FF) != 0x0000001F) {
	}
	printf("%s: reg_value: 0x%08x\n", __func__, readl(reg));
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
