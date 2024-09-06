// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2019-2024 by Lattice Semiconductor Corporation
 */

#include <asm/io.h>
#include <asm/gpio.h>
#include <dm.h>
#include <dt-bindings/gpio/gpio.h>
#include <lattice/lattice_reg_access.h>
#include <linux/io.h>
#include <linux/sizes.h>

/*
 * gpio register definition
 */
#define DATA_WIDTH						(0x8)

#define GPIO_RD_DATA                    (0x00)
#define GPIO_WR_DATA                    (0x01 * 4)
#define GPIO_SET_DATA                   (0x02 * 4)
#define GPIO_CLEAR_DATA                 (0x03 * 4)
#define GPIO_DIRECTION                  (0x04 * 4)
#define GPIO_INT_TYPE                   (0x05 * 4)
#define GPIO_INT_METHOD                 (0x06 * 4)
#define GPIO_INT_STATUS                 (0x07 * 4)
#define GPIO_INT_ENABLE                 (0x08 * 4)
#define GPIO_INT_SET                    (0x09 * 4)

enum gpio_direction {
	GPIO_INPUT,
	GPIO_OUTPUT
};

struct gpio_cfg {
	u32 pin;
	enum gpio_direction direction;
};

struct lattice_gpio_plat {
	const char *instance_name;
	void __iomem *base_address;
	struct gpio_cfg gpio_config[32];
};

static int lattice_gpio_probe(struct udevice *dev)
{
	struct lattice_gpio_plat *plat = dev_get_plat(dev);
	fdt_addr_t base_addr;

	base_addr = dev_read_addr(dev);
	if (base_addr == FDT_ADDR_T_NONE)
		return -EINVAL;

	plat->base_address = devm_ioremap(dev, base_addr, SZ_4K);
	if (!plat->base_address)
		return -ENOMEM;

	return 0;
}

static int lattice_gpio_output_write(struct udevice *dev,
				     u32 index, int value)
{
	if (!dev)
		return 1;

	struct lattice_gpio_plat *plat = dev_get_plat(dev);

	if (plat->gpio_config[index].direction == GPIO_OUTPUT) {
		reg_32b_modify(plat->base_address + GPIO_WR_DATA,
			       plat->gpio_config[index].pin, value);
	}

	return 0;
}

static int lattice_gpio_input_get(struct udevice *dev,
				  u32 index)
{
	u32 data;

	if (!dev)
		return 1;

	struct lattice_gpio_plat *plat = dev_get_plat(dev);

	reg_32b_read(plat->base_address + GPIO_RD_DATA, &data);
	if (index < DATA_WIDTH)
		data = data >> index && 1;
	else
		return -EAGAIN;

	return data;
}

static int lattice_gpio_set_direction_input(struct udevice *dev,
					    u32 index)
{
	if (!dev)
		return 1;

	struct lattice_gpio_plat *plat = dev_get_plat(dev);

	reg_32b_modify(plat->base_address + GPIO_DIRECTION,
		       plat->gpio_config[index].pin, GPIO_INPUT);

	return 0;
}

static int lattice_gpio_set_direction_output(struct udevice *dev,
					     u32 index, int value)
{
	if (!dev)
		return 1;

	struct lattice_gpio_plat *plat = dev_get_plat(dev);

	reg_32b_modify(plat->base_address + GPIO_DIRECTION,
		       plat->gpio_config[index].pin, GPIO_OUTPUT);
	lattice_gpio_output_write(dev, index, value);

	return 0;
}

static const struct dm_gpio_ops lattice_gpio_ops = {
	.direction_output = lattice_gpio_set_direction_output,
	.direction_input = lattice_gpio_set_direction_input,
	.get_value = lattice_gpio_input_get,
	.set_value = lattice_gpio_output_write,
};

static const struct udevice_id lattice_gpio_ids[] = {
	{ .compatible = "lattice,lattice-gpio-1.0",},
	{ }
};

U_BOOT_DRIVER(lattice_gpio) = {
	.name = "lattice_gpio",
	.id = UCLASS_GPIO,
	.probe	= lattice_gpio_probe,
	.ops = &lattice_gpio_ops,
	.of_match = lattice_gpio_ids,
	.plat_auto = sizeof(struct lattice_gpio_plat),
};
