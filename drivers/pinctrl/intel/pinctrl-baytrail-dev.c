/*
 * pinctrl-baytrail-dev.c: BayTrail pinctrl GPIO Platform Device
 *
 * (C) Copyright 2013 Intel Corporation
 * Author: Kean Ho, Chew (kean.ho.chew@intel.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/pci.h>
#include <linux/pinctrl/pinctrl-byt.h>

/* PCI Memory Base Access */
#define PCI_DEVICE_ID_INTEL_BYT_PCU	0x0f1c
#define NO_REGISTER_SETTINGS	(BIT(0) | BIT(1) | BIT(2))

/* Offsets */
#define SCORE_OFFSET		0x0
#define NCORE_OFFSET		0x1000
#define SUS_OFFSET		0x2000
#define SCORE_END		0x72C
#define NCORE_END		0x970
#define SUS_END			0x98C

static struct byt_pinctrl_port byt_gpio_score_platform_data = {
	.unique_id = "1",
};

static struct resource byt_gpio_score_resources[] = {
	{
		.start	= 0x0,
		.end	= 0x0,
		.flags	= IORESOURCE_MEM,
		.name	= "io-memory",
	},
	{
		.start	= 49,
		.end	= 49,
		.flags	= IORESOURCE_IRQ,
		.name	= "irq",
	}
};

static struct byt_pinctrl_port byt_gpio_ncore_platform_data = {
	.unique_id = "2",
};

static struct resource byt_gpio_ncore_resources[] = {
	{
		.start	= 0x0,
		.end	= 0x0,
		.flags	= IORESOURCE_MEM,
		.name	= "io-memory",
	},
	{
		.start	= 48,
		.end	= 48,
		.flags	= IORESOURCE_IRQ,
		.name	= "irq",
	}
};

static struct byt_pinctrl_port byt_gpio_sus_platform_data = {
	.unique_id = "3",
};

static struct resource byt_gpio_sus_resources[] = {
	{
		.start	= 0x0,
		.end	= 0x0,
		.flags	= IORESOURCE_MEM,
		.name	= "io-memory",
	},
	{
		.start	= 50,
		.end	= 50,
		.flags	= IORESOURCE_IRQ,
		.name	= "irq",
	}
};

static struct platform_device byt_gpio_score_device = {
	.name			= "byt_gpio",
	.id			= 0,
	.num_resources		= ARRAY_SIZE(byt_gpio_score_resources),
	.resource		= byt_gpio_score_resources,
	.dev			= {
		.platform_data	= &byt_gpio_score_platform_data,
	}
};

static struct platform_device byt_gpio_ncore_device = {
	.name			= "byt_gpio",
	.id			= 1,
	.num_resources		= ARRAY_SIZE(byt_gpio_ncore_resources),
	.resource		= byt_gpio_ncore_resources,
	.dev			= {
		.platform_data	= &byt_gpio_ncore_platform_data,
	}
};

static struct platform_device byt_gpio_sus_device = {
	.name			= "byt_gpio",
	.id			= 2,
	.num_resources		= ARRAY_SIZE(byt_gpio_sus_resources),
	.resource		= byt_gpio_sus_resources,
	.dev			= {
		.platform_data	= &byt_gpio_sus_platform_data,
	}
};

static struct platform_device *devices[] __initdata = {
	&byt_gpio_score_device,
	&byt_gpio_ncore_device,
	&byt_gpio_sus_device,
};

static int __init get_pci_memory_init(void)
{
	u32 io_base_add;
	struct pci_dev *pci_dev;
	pci_dev = pci_get_device(PCI_VENDOR_ID_INTEL,
				PCI_DEVICE_ID_INTEL_BYT_PCU,
				NULL);

	if (pci_dev == NULL) {
		return -EFAULT;
	};
	pci_read_config_dword(pci_dev, 0x4c, &io_base_add);
	io_base_add &= ~NO_REGISTER_SETTINGS;
	byt_gpio_score_resources[0].start = io_base_add + SCORE_OFFSET;
	byt_gpio_score_resources[0].end =
				io_base_add + SCORE_OFFSET + SCORE_END;
	byt_gpio_ncore_resources[0].start = io_base_add + NCORE_OFFSET;
	byt_gpio_ncore_resources[0].end =
				io_base_add + NCORE_OFFSET + NCORE_END;
	byt_gpio_sus_resources[0].start = io_base_add + SUS_OFFSET;
	byt_gpio_sus_resources[0].end = io_base_add + SUS_OFFSET + SUS_END;
	return 0;
};
rootfs_initcall(get_pci_memory_init);


static int __init byt_gpio_device_init(void)
{
	return platform_add_devices(devices, ARRAY_SIZE(devices));
};
device_initcall(byt_gpio_device_init);
