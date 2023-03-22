// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/* Copyright (c) 2015, The Linux Foundation. All rights reserved. */
/* Copyright (c) 2020 Sartura Ltd. */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#define MDIO_MODE_REG				0x40
#define MDIO_ADDR_REG				0x44
#define MDIO_DATA_WRITE_REG			0x48
#define MDIO_DATA_READ_REG			0x4c
#define MDIO_CMD_REG				0x50
#define MDIO_CMD_ACCESS_BUSY		BIT(16)
#define MDIO_CMD_ACCESS_START		BIT(8)
#define MDIO_CMD_ACCESS_CODE_READ	0
#define MDIO_CMD_ACCESS_CODE_WRITE	1
#define MDIO_CMD_ACCESS_CODE_C45_ADDR	0
#define MDIO_CMD_ACCESS_CODE_C45_WRITE	1
#define MDIO_CMD_ACCESS_CODE_C45_READ	2

/* 0 = Clause 22, 1 = Clause 45 */
#define MDIO_MODE_C45				BIT(8)

#define IPQ4019_MDIO_TIMEOUT	10000
#define IPQ4019_MDIO_SLEEP		10

/* MDIO clock source frequency is fixed to 100M */
#define IPQ_MDIO_CLK_RATE	100000000

#define IPQ_PHY_SET_DELAY_US	100000

#define IPQ_HIGH_ADDR_PREFIX	0x18
#define IPQ_LOW_ADDR_PREFIX	0x10

#define PHY_ADDR_LENGTH		5
#define PHY_ADDR_NUM		4
#define UNIPHY_ADDR_NUM		3

struct ipq4019_mdio_data {
	void __iomem	*membase;
	void __iomem *eth_ldo_rdy;
	struct clk *mdio_clk;
};

static int ipq4019_mdio_wait_busy(struct mii_bus *bus)
{
	struct ipq4019_mdio_data *priv = bus->priv;
	unsigned int busy;

	return readl_poll_timeout(priv->membase + MDIO_CMD_REG, busy,
				  (busy & MDIO_CMD_ACCESS_BUSY) == 0,
				  IPQ4019_MDIO_SLEEP, IPQ4019_MDIO_TIMEOUT);
}

static int ipq4019_mdio_read_c45(struct mii_bus *bus, int mii_id, int mmd,
				 int reg)
{
	struct ipq4019_mdio_data *priv = bus->priv;
	unsigned int data;
	unsigned int cmd;

	if (ipq4019_mdio_wait_busy(bus))
		return -ETIMEDOUT;

	data = readl(priv->membase + MDIO_MODE_REG);

	data |= MDIO_MODE_C45;

	writel(data, priv->membase + MDIO_MODE_REG);

	/* issue the phy address and mmd */
	writel((mii_id << 8) | mmd, priv->membase + MDIO_ADDR_REG);

	/* issue reg */
	writel(reg, priv->membase + MDIO_DATA_WRITE_REG);

	cmd = MDIO_CMD_ACCESS_START | MDIO_CMD_ACCESS_CODE_C45_ADDR;

	/* issue read command */
	writel(cmd, priv->membase + MDIO_CMD_REG);

	/* Wait read complete */
	if (ipq4019_mdio_wait_busy(bus))
		return -ETIMEDOUT;

	cmd = MDIO_CMD_ACCESS_START | MDIO_CMD_ACCESS_CODE_C45_READ;

	writel(cmd, priv->membase + MDIO_CMD_REG);

	if (ipq4019_mdio_wait_busy(bus))
		return -ETIMEDOUT;

	/* Read and return data */
	return readl(priv->membase + MDIO_DATA_READ_REG);
}

static int ipq4019_mdio_read_c22(struct mii_bus *bus, int mii_id, int regnum)
{
	struct ipq4019_mdio_data *priv = bus->priv;
	unsigned int data;
	unsigned int cmd;

	if (ipq4019_mdio_wait_busy(bus))
		return -ETIMEDOUT;

	data = readl(priv->membase + MDIO_MODE_REG);

	data &= ~MDIO_MODE_C45;

	writel(data, priv->membase + MDIO_MODE_REG);

	/* issue the phy address and reg */
	writel((mii_id << 8) | regnum, priv->membase + MDIO_ADDR_REG);

	cmd = MDIO_CMD_ACCESS_START | MDIO_CMD_ACCESS_CODE_READ;

	/* issue read command */
	writel(cmd, priv->membase + MDIO_CMD_REG);

	/* Wait read complete */
	if (ipq4019_mdio_wait_busy(bus))
		return -ETIMEDOUT;

	/* Read and return data */
	return readl(priv->membase + MDIO_DATA_READ_REG);
}

static int ipq4019_mdio_write_c45(struct mii_bus *bus, int mii_id, int mmd,
				  int reg, u16 value)
{
	struct ipq4019_mdio_data *priv = bus->priv;
	unsigned int data;
	unsigned int cmd;

	if (ipq4019_mdio_wait_busy(bus))
		return -ETIMEDOUT;

	data = readl(priv->membase + MDIO_MODE_REG);

	data |= MDIO_MODE_C45;

	writel(data, priv->membase + MDIO_MODE_REG);

	/* issue the phy address and mmd */
	writel((mii_id << 8) | mmd, priv->membase + MDIO_ADDR_REG);

	/* issue reg */
	writel(reg, priv->membase + MDIO_DATA_WRITE_REG);

	cmd = MDIO_CMD_ACCESS_START | MDIO_CMD_ACCESS_CODE_C45_ADDR;

	writel(cmd, priv->membase + MDIO_CMD_REG);

	if (ipq4019_mdio_wait_busy(bus))
		return -ETIMEDOUT;

	/* issue write data */
	writel(value, priv->membase + MDIO_DATA_WRITE_REG);

	cmd = MDIO_CMD_ACCESS_START | MDIO_CMD_ACCESS_CODE_C45_WRITE;
	writel(cmd, priv->membase + MDIO_CMD_REG);

	/* Wait write complete */
	if (ipq4019_mdio_wait_busy(bus))
		return -ETIMEDOUT;

	return 0;
}

static int ipq4019_mdio_write_c22(struct mii_bus *bus, int mii_id, int regnum,
				  u16 value)
{
	struct ipq4019_mdio_data *priv = bus->priv;
	unsigned int data;
	unsigned int cmd;

	if (ipq4019_mdio_wait_busy(bus))
		return -ETIMEDOUT;

	/* Enter Clause 22 mode */
	data = readl(priv->membase + MDIO_MODE_REG);

	data &= ~MDIO_MODE_C45;

	writel(data, priv->membase + MDIO_MODE_REG);

	/* issue the phy address and reg */
	writel((mii_id << 8) | regnum, priv->membase + MDIO_ADDR_REG);

	/* issue write data */
	writel(value, priv->membase + MDIO_DATA_WRITE_REG);

	/* issue write command */
	cmd = MDIO_CMD_ACCESS_START | MDIO_CMD_ACCESS_CODE_WRITE;

	writel(cmd, priv->membase + MDIO_CMD_REG);

	/* Wait write complete */
	if (ipq4019_mdio_wait_busy(bus))
		return -ETIMEDOUT;

	return 0;
}

static inline void split_addr(u32 regaddr, u16 *r1, u16 *r2, u16 *page, u16 *sw_addr)
{
	*r1 = regaddr & 0x1c;

	regaddr >>= 5;
	*r2 = regaddr & 0x7;

	regaddr >>= 3;
	*page = regaddr & 0xffff;

	regaddr >>= 16;
	*sw_addr = regaddr & 0xff;
}

u32 ipq_mii_read(struct mii_bus *bus, unsigned int reg)
{
	u16 r1, r2, page, sw_addr;
	u16 lo, hi;

	split_addr(reg, &r1, &r2, &page, &sw_addr);

	/* There is no competition, so the lock is not needed.
	 * since this function is only called before mii_bus registered.
	 */
	bus->write(bus, IPQ_HIGH_ADDR_PREFIX | (sw_addr >> 5), sw_addr & 0x1f, page);

	lo = bus->read(bus, IPQ_LOW_ADDR_PREFIX | r2, r1);
	hi = bus->read(bus, IPQ_LOW_ADDR_PREFIX | r2, r1 | BIT(1));

	return hi << 16 | lo;
};

int ipq_mii_write(struct mii_bus *bus, unsigned int reg, unsigned int val)
{
	u16 r1, r2, page, sw_addr;
	u16 lo, hi;

	lo = val & 0xffff;
	hi = (u16)(val >> 16);

	split_addr(reg, &r1, &r2, &page, &sw_addr);

	/* There is no competition, so the lock is not needed.
	 * since this function is only called before mii_bus registered.
	 */
	bus->write(bus, IPQ_HIGH_ADDR_PREFIX | (sw_addr >> 5), sw_addr & 0x1f, page);

	bus->write(bus, IPQ_LOW_ADDR_PREFIX | r2, r1, lo);
	bus->write(bus, IPQ_LOW_ADDR_PREFIX | r2, r1 | BIT(1), hi);

	return 0;
};

static void ipq_phy_addr_fixup(struct mii_bus *bus, struct device_node *np)
{
	void __iomem *ephy_cfg_base;
	struct device_node *child;
	int phy_index, addr, len;
	const __be32 *phy_cfg, *uniphy_cfg;
	u32 val;
	bool mdio_access = false;
	unsigned long phyaddr_mask = 0;

	phy_cfg = of_get_property(np, "phyaddr_fixup", &len);
	uniphy_cfg = of_get_property(np, "uniphyaddr_fixup", NULL);

	/*
	 * For MDIO access, phyaddr_fixup only provides the register address,
	 * as for local bus, the register length also needs to be provided
	 */
	if(!phy_cfg || (len != (2 * sizeof(__be32)) && len != sizeof(__be32)))
		return;

	if (len == sizeof(__be32))
		mdio_access = true;

	if (!mdio_access) {
		ephy_cfg_base = ioremap(be32_to_cpup(phy_cfg), be32_to_cpup(phy_cfg + 1));
		if (!ephy_cfg_base)
			return;
		val = readl(ephy_cfg_base);
	} else
		val = ipq_mii_read(bus, be32_to_cpup(phy_cfg));

	phy_index = 0;
	addr = 0;
	for_each_available_child_of_node(np, child) {
		if (phy_index >= PHY_ADDR_NUM)
			break;

		addr = of_mdio_parse_addr(&bus->dev, child);
		if (addr < 0) {
			continue;
		}
		phyaddr_mask |= BIT(addr);

		if (!of_find_property(child, "fixup", NULL))
			continue;

		addr &= GENMASK(4, 0);
		val &= ~(GENMASK(4, 0) << (phy_index * PHY_ADDR_LENGTH));
		val |= addr << (phy_index * PHY_ADDR_LENGTH);
		phy_index++;
	}

	/* Programe the PHY address */
	dev_info(bus->parent, "Program EPHY reg 0x%x with 0x%x\n",
			be32_to_cpup(phy_cfg), val);

	if (!mdio_access) {
		writel(val, ephy_cfg_base);
		iounmap(ephy_cfg_base);
	} else {
		ipq_mii_write(bus, be32_to_cpup(phy_cfg), val);

		/* Programe the UNIPHY address if uniphyaddr_fixup specified.
		 * the UNIPHY address will select three MDIO address from
		 * unoccupied MDIO address space. */
		if (uniphy_cfg) {
			val = ipq_mii_read(bus, be32_to_cpup(uniphy_cfg));

			/* For qca8386, the switch occupies the other 16 MDIO address,
			 * for example, if the phy address is in the range of 0 to 15,
			 * the switch will occupy the MDIO address from 16 to 31. */
			if (addr > 15)
				phyaddr_mask |= GENMASK(15, 0);
			else
				phyaddr_mask |= GENMASK(31, 16);

			phy_index = 0;
			for_each_clear_bit_from(addr, &phyaddr_mask, PHY_MAX_ADDR) {
				if (phy_index >= UNIPHY_ADDR_NUM)
					break;

				val &= ~(GENMASK(4, 0) << (phy_index * PHY_ADDR_LENGTH));
				val |= addr << (phy_index * PHY_ADDR_LENGTH);
				phy_index++;
			}

			if (phy_index < UNIPHY_ADDR_NUM) {
				for_each_clear_bit(addr, &phyaddr_mask, PHY_MAX_ADDR) {
					if (phy_index >= UNIPHY_ADDR_NUM)
						break;

					val &= ~(GENMASK(4, 0) << (phy_index * PHY_ADDR_LENGTH));
					val |= addr << (phy_index * PHY_ADDR_LENGTH);
					phy_index++;
				}
			}

			dev_info(bus->parent, "Program UNIPHY reg 0x%x with 0x%x\n",
					be32_to_cpup(uniphy_cfg), val);

			ipq_mii_write(bus, be32_to_cpup(uniphy_cfg), val);
		}
	}
}

void ipq_mii_preinit(struct mii_bus *bus)
{
	struct device_node *np = bus->parent->of_node;
	if (!np)
		return;

	ipq_phy_addr_fixup(bus, np);
	return;
}
EXPORT_SYMBOL_GPL(ipq_mii_preinit);

static int ipq_mdio_reset(struct mii_bus *bus)
{
	struct ipq4019_mdio_data *priv = bus->priv;
	u32 val;
	int ret;

	/* To indicate CMN_PLL that ethernet_ldo has been ready if platform resource 1
	 * is specified in the device tree.
	 */
	if (priv->eth_ldo_rdy) {
		val = readl(priv->eth_ldo_rdy);
		val |= BIT(0);
		writel(val, priv->eth_ldo_rdy);
		fsleep(IPQ_PHY_SET_DELAY_US);
	}

	/* Configure MDIO clock source frequency if clock is specified in the device tree */
	ret = clk_set_rate(priv->mdio_clk, IPQ_MDIO_CLK_RATE);
	if (ret)
		return ret;

	ret = clk_prepare_enable(priv->mdio_clk);
	if (ret == 0) {
		mdelay(10);

		/* Configure the fixup PHY address and clocks for qca8386 chip if specified */
		ipq_mii_preinit(bus);
	}

	return ret;
}

static int ipq4019_mdio_probe(struct platform_device *pdev)
{
	struct ipq4019_mdio_data *priv;
	struct mii_bus *bus;
	struct resource *res;
	int ret;

	bus = devm_mdiobus_alloc_size(&pdev->dev, sizeof(*priv));
	if (!bus)
		return -ENOMEM;

	priv = bus->priv;

	priv->membase = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->membase))
		return PTR_ERR(priv->membase);

	priv->mdio_clk = devm_clk_get_optional(&pdev->dev, "gcc_mdio_ahb_clk");
	if (IS_ERR(priv->mdio_clk))
		return PTR_ERR(priv->mdio_clk);

	/* The platform resource is provided on the chipset IPQ5018 */
	/* This resource is optional */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res)
		priv->eth_ldo_rdy = devm_ioremap_resource(&pdev->dev, res);

	bus->name = "ipq4019_mdio";
	bus->read = ipq4019_mdio_read_c22;
	bus->write = ipq4019_mdio_write_c22;
	bus->read_c45 = ipq4019_mdio_read_c45;
	bus->write_c45 = ipq4019_mdio_write_c45;
	bus->reset = ipq_mdio_reset;
	bus->parent = &pdev->dev;
	snprintf(bus->id, MII_BUS_ID_SIZE, "%s%d", pdev->name, pdev->id);

	ret = of_mdiobus_register(bus, pdev->dev.of_node);
	if (ret) {
		dev_err(&pdev->dev, "Cannot register MDIO bus!\n");
		return ret;
	}

	platform_set_drvdata(pdev, bus);

	return 0;
}

static int ipq4019_mdio_remove(struct platform_device *pdev)
{
	struct mii_bus *bus = platform_get_drvdata(pdev);

	mdiobus_unregister(bus);

	return 0;
}

static const struct of_device_id ipq4019_mdio_dt_ids[] = {
	{ .compatible = "qcom,ipq4019-mdio" },
	{ .compatible = "qcom,ipq5018-mdio" },
	{ }
};
MODULE_DEVICE_TABLE(of, ipq4019_mdio_dt_ids);

static struct platform_driver ipq4019_mdio_driver = {
	.probe = ipq4019_mdio_probe,
	.remove = ipq4019_mdio_remove,
	.driver = {
		.name = "ipq4019-mdio",
		.of_match_table = ipq4019_mdio_dt_ids,
	},
};

module_platform_driver(ipq4019_mdio_driver);

MODULE_DESCRIPTION("ipq4019 MDIO interface driver");
MODULE_AUTHOR("Qualcomm Atheros");
MODULE_LICENSE("Dual BSD/GPL");
