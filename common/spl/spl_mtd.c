// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2000-2004
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * Copyright (C) 2024 by Lattice Semiconductor Corporation
 * <www.latticesemi.com>
 */
#include <common.h>
#include <image.h>
#include <imx_container.h>
#include <log.h>
#include <mtd.h>
#include <errno.h>
#include <spl.h>
#include <spl_load.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <dm/ofnode.h>

static struct mtd_info *get_mtd_dev_by_name(const char *name)
{
	struct mtd_info *mtd;

	mtd_probe_devices();

	mtd = get_mtd_device_nm(name);
	if (!mtd)
		debug("MTD device %s not found\n", name);

	return mtd;
}

static ulong spl_mtd_fit_read(struct spl_load_info *load, ulong sector,
			      ulong count, void *buf)
{
	ulong ret;
	size_t retlen;
	struct mtd_info *mtd;

	mtd = get_mtd_dev_by_name("lattice_qspi");
	if (!mtd)
		return -ENODEV;

	ret = mtd_read(mtd, (loff_t)sector, (size_t)count, &retlen, buf);
	if (!ret)
		return count;
	else
		return 0;
}

static int spl_mtd_load_image(struct spl_image_info *spl_image,
			      struct spl_boot_device *bootdev)
{
	int err = 0;
	unsigned int payload_offs, ram_offs;
	struct mtd_info *mtd;
	unsigned int *buf;
	struct spl_load_info load;

	mtd = get_mtd_dev_by_name("lattice_qspi");
	if (!mtd) {
		debug("%s: Failed to get mtd device\n", __func__);
		return -ENODEV;
	}

	/*
	 * Load U-Boot image from Lattice QSPI flash into RAM
	 */
	if (CONFIG_IS_ENABLED(OF_REAL)) {
		payload_offs = (loff_t)ofnode_conf_read_int("u-boot,spl-payload-flash-offset",
							    payload_offs);
		ram_offs = ofnode_conf_read_int("u-boot,spl-payload-ram-offset",
						ram_offs);
		buf = (unsigned int *)ram_offs;
	}

	load.priv = mtd;
	spl_set_bl_len(&load, 1);
	load.read = spl_mtd_fit_read;
	err = spl_load(spl_image, bootdev, &load, 0, payload_offs);
	if (err)
		debug("%s: Failed to read from SPI flash (err=%d)\n",
		      __func__, err);

	return err;
}

SPL_LOAD_IMAGE_METHOD("MTD", 0, BOOT_DEVICE_MTD, spl_mtd_load_image);
