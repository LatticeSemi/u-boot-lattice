// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Lattice Semiconductor Corp
 * Author: WeiLiang Lim<weiliang.lim@latticesemi.com>
 */

#include <asm/io.h>
#include <asm/system.h>
#include <asm/gpio.h>
#include <common.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <spl.h>

#define LPDDR4_CONTR_BASE_ADDR  0x10092000
#define LPDDR4_OUT_OF_RESET     0x00000003
#define GPIO_CONTR_BASE_ADDR    0x10000000
#define LED_OFFSET              0x4

static void lpddr_init(void)
{
	int ret;
	u32 tmp = 0;
	void __iomem *reg = (void __iomem *)LPDDR4_CONTR_BASE_ADDR + 0x04;

	/* Bringing LPDDR4 out of reset */
	writel(LPDDR4_OUT_OF_RESET, reg);
	reg = (void __iomem *)LPDDR4_CONTR_BASE_ADDR + 0x24;
	ret = readl_poll_timeout(reg, tmp, (tmp & 0x000000FF) == 0x0000001F, 1000);
}

#if IS_ENABLED(CONFIG_SPL_LOAD_FIT)
u32 spl_boot_device(void)
{
	return BOOT_DEVICE_MTD;
}
#endif

void set_led(int value)
{
	void __iomem *reg = (void __iomem *)GPIO_CONTR_BASE_ADDR + LED_OFFSET;

	writel(value, reg);
}

void led_init(void)
{
	/* 100ms blink to test LED */
	set_led(0);
	mdelay(100);
	set_led(0x55);
	mdelay(100);
	set_led(0);
	mdelay(100);
	set_led(0xAA);
	mdelay(100);
}

void board_init_f(ulong dummy)
{
	int ret;

	ret = spl_early_init();
	if (ret)
		panic("spl_early_init() failed: %d\n", ret);

	riscv_cpu_setup();
	preloader_console_init();
	led_init();
	lpddr_init();
}
