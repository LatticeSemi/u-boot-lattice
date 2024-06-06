// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2000-2004
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * (C) Copyright 2011
 * Texas Instruments, <www.ti.com>
 *
 * Matt Porter <mporter@ti.com>
 */
#include <common.h>
#include <gzip.h>
#include <image.h>
#include <imx_container.h>
#include <log.h>
#include <mapmem.h>
#include <mtd.h>
#include <errno.h>
#include <spl.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <dm/ofnode.h>
#include <asm/u-boot.h>
#include <linux/libfdt.h>

static struct mtd_info *get_mtd_dev_by_name(const char *name)
{
	struct mtd_info *mtd;

	mtd_probe_devices();
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
	size_t *retlen;
	struct mtd_info *mtd;

	mtd = get_mtd_dev_by_name("lattice_qspi");
	if (!mtd)
		return -ENODEV;

	ret = mtd_read(mtd, (loff_t)sector, (size_t)count, retlen, buf);
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
	struct legacy_img_hdr *header;
	struct mtd_info *mtd;
	unsigned int *buf;
	size_t *retlen;

	mtd = get_mtd_dev_by_name("lattice_qspi");
	if (!mtd)
		return -ENODEV;

	/*
	 * Load U-Boot image from Lattice QSPI flash into RAM
	 */
	header = spl_get_load_buffer(-sizeof(*header), sizeof(*header));

	if (CONFIG_IS_ENABLED(OF_REAL)) {
		payload_offs = (loff_t)ofnode_conf_read_int("u-boot,spl-payload-flash-offset",
							    payload_offs);
		ram_offs = ofnode_conf_read_int("u-boot,spl-payload-ram-offset",
						ram_offs);
		buf = (unsigned int *)ram_offs;
	}

	/* Load u-boot, mkimage header is 64 bytes. */
	err = mtd_read(mtd, (loff_t)payload_offs, (size_t)sizeof(*header),
		       retlen, (void *)buf);
	if (err) {
		debug("%s: Failed to read from SPI flash (err=%d)\n",
		      __func__, err);
		return err;
	}

	if (IS_ENABLED(CONFIG_SPL_LOAD_FIT_FULL) &&
	    image_get_magic(header) == FDT_MAGIC) {
		u32 size = roundup(fdt_totalsize(header), 4);

		err = mtd_read(mtd, (loff_t)payload_offs,
			       (size_t)size, retlen,
			       map_sysmem(CONFIG_SYS_LOAD_ADDR, size));
		if (err)
			return err;
		err = spl_parse_image_header(spl_image, bootdev,
					     phys_to_virt(CONFIG_SYS_LOAD_ADDR));
		} else if (IS_ENABLED(CONFIG_SPL_LOAD_FIT) &&
			   image_get_magic(header) == FDT_MAGIC) {
			struct spl_load_info load;

			puts("Found FIT\n");
			load.priv = NULL;
			spl_set_bl_len(&load, 1);
			load.read = spl_mtd_fit_read;
			err = spl_load_simple_fit(spl_image, &load,
						  payload_offs,
						  header);
		} else if (IS_ENABLED(CONFIG_SPL_LOAD_IMX_CONTAINER) &&
			   valid_container_hdr((void *)header)) {
			struct spl_load_info load;

			load.priv = NULL;
			spl_set_bl_len(&load, 1);
			load.read = spl_mtd_fit_read;

			err = spl_load_imx_container(spl_image, &load,
						     payload_offs);
		} else {
			err = spl_parse_image_header(spl_image, bootdev, header);
			if (err)
				return err;
			err = mtd_read(mtd, (loff_t)payload_offs + spl_image->offset,
				       (size_t)spl_image->size, retlen,
				       map_sysmem(spl_image->load_addr,
						  spl_image->size));
		}

	return err;
}

SPL_LOAD_IMAGE_METHOD("MTD", 0, BOOT_DEVICE_MTD, spl_mtd_load_image);
