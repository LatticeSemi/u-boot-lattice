// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Lattice Semiconductor Corp
 * Author: WeiLiang Lim<weiliang.lim@latticesemi.com>
 */

#include <common.h>
#include <dm.h>
#include <dm/ofnode.h>
#include <env.h>
#include <fdtdec.h>
#include <image.h>
#include <init.h>
#include <log.h>
#include <spl.h>

int board_init(void)
{
	return 0;
}

#if IS_ENABLED(CONFIG_SPL_LOAD_FIT)
int board_fit_config_name_match(const char *name)
{
	/* boot using first FIT config */
	return 0;
}
#endif

void *board_fdt_blob_setup(int *err)
{
	*err = 0;
	/* Stored the DTB address there during our init */
	return (void *)(ulong)gd->arch.firmware_fdt_addr;
}
