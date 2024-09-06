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
#include <i2c.h>
#include <linux/delay.h>

#define CHIP_ADDR	0x56

static int i2c_read_operation(struct udevice *dev,
			  struct i2c_msg *qmsg, struct i2c_msg *rmsg)
{
	int err;

	err = dm_i2c_xfer(dev, qmsg, 1);
	if (err)
		return err;

	/*
	 * device sends IRQ when its ok to read. To keep the code
	 * simple we just wait an arbitrary, long enough time period.
	 */
	mdelay(10);

	rmsg->len = 1;
	err = dm_i2c_xfer(dev, rmsg, 1);
	if (err)
		return err;

	rmsg->buf = rmsg->buf+1;
	rmsg->len = 1;
	err = dm_i2c_xfer(dev, rmsg, 1);
	if (err)
		return err;

	return 0;
}

static int i2c_phy_init(void)
{
	int err;
	u32 model;
	u8 query[1] = {0x2};
	u8 query2[1] = {0x3};
	u8 query3[1] = {0x1B};
	u8 write_phy[3] = {0x1B, 0x80, 0x84};
	u8 reset_phy[3] = {0x00, 0x81, 0x40};
	u8 phy_reg2[2] = {0};
	u8 phy_reg3[2] = {0};
	u8 phy_reg_buf[2] = {0};
	struct udevice *bus;
	struct udevice *dev;

	struct i2c_msg qmsg = {
		.addr = CHIP_ADDR,
		.flags = 0,
		.len = sizeof(query),
		.buf = query,
	};

	struct i2c_msg qmsg2 = {
		.addr = CHIP_ADDR,
		.flags = 0,
		.len = sizeof(query2),
		.buf = query2,
	};

	struct i2c_msg qmsg3 = {
		.addr = CHIP_ADDR,
		.flags = 0,
		.len = sizeof(query3),
		.buf = query3,
	};

	struct i2c_msg w_msg = {
		.addr = CHIP_ADDR,
		.flags = 0,
		.len = sizeof(write_phy),
		.buf = write_phy,
	};

	struct i2c_msg w_msg2 = {
		.addr = CHIP_ADDR,
		.flags = 0,
		.len = sizeof(reset_phy),
		.buf = reset_phy,
	};

	struct i2c_msg phy_rmsg = {
		.addr = CHIP_ADDR,
		.flags = I2C_M_RD,
		.len = sizeof(phy_reg2),
		.buf = phy_reg2,
	};

	struct i2c_msg phy_rmsg2 = {
		.addr = CHIP_ADDR,
		.flags = I2C_M_RD,
		.len = sizeof(phy_reg3),
		.buf = phy_reg3,
	};

	struct i2c_msg phy_rmsg3 = {
		.addr = CHIP_ADDR,
		.flags = I2C_M_RD,
		.len = sizeof(phy_reg_buf),
		.buf = phy_reg_buf,
	};

	err = uclass_get_device_by_seq(UCLASS_I2C, 0, &bus);
	if (err) {
		printf("Could not get I2C bus: %d\n", err);
		return err;
	}

	err = i2c_get_chip(bus, CHIP_ADDR, 1, &dev);
	if (err) { /* Not Marvell PHY */
		return err;
	}

	err = i2c_read_operation(dev, &qmsg, &phy_rmsg);
	if (err)
		return err;

	for (int i = 0; i < 2; i++)
		debug("%02x ", phy_reg2[i]);
	debug("\n\n");

	err = i2c_read_operation(dev, &qmsg2, &phy_rmsg2);
	if (err)
		return err;

	for (int i = 0; i < 2; i++)
		debug("%02x ", phy_reg3[i]);
	debug("\n\n");

	model = phy_reg2[0] << 24 | phy_reg2[1] << 16 |
		    phy_reg3[0] << 8 | phy_reg3[1];
	printf("Model: 0x%08x\n", model);
	if (model == 0x01410CC2) {
		printf("Configure Marvell Alaska 88E1111 SFP to SGMII mode\n");
		err = dm_i2c_xfer(dev, &w_msg, 1);
		if (err)
			return err;

		printf("PHY reset\n");
		err = dm_i2c_xfer(dev, &w_msg2, 1);
		if (err)
			return err;

		err = i2c_read_operation(dev, &qmsg3, &phy_rmsg3);
		if (err)
			return err;

		printf("After configuration: I2C Extended PHY Specific Status Register: ");
		for (int i = 0; i < 2; i++)
			printf("0x%02x ", phy_reg_buf[i]);
		printf("\n");
	}

	return 0;
}

int board_init(void)
{
	int ret;

	ret = i2c_phy_init();

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
