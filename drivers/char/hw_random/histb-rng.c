// SPDX-License-Identifier: GPL-2.0-or-later OR MIT
/*
 * Copyright (c) 2023 David Yang
 */

#include <linux/err.h>
#include <linux/hw_random.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#define RNG_CTRL		0x0
#define  RNG_SOURCE			GENMASK(1, 0)
#define  DRBG_ENABLE			BIT(3)		/* hi3798cv200 布局 */
#define  DROP_ENABLE			BIT(5)
#define  POST_PROCESS_ENABLE		BIT(7)
#define  POST_PROCESS_DEPTH		GENMASK(15, 8)
#define RNG_NUMBER		0x4
#define RNG_STAT		0x8
#define  DATA_COUNT			GENMASK(2, 0)	/* max 4 */
#define  FIFO_DATA_COUNT		GENMASK(15, 8)	/* hi3798cv200 布局，最大 32 */

/*
 * hi3798cv200 这一代（厂商称 "TRNG v200"：cv200、mv200、mv300、mv310）保留三个寄存器，但把就绪计数移到 STAT[15:8]（其低字节为振荡器状态标志），并把 CTRL 的 bit 3、4、6 改作 DRBG、pre-process 与 mix 的使能位。
 */
struct histb_rng_soc {
	u32 count_mask;
	bool drbg;	/* 经 DRBG 输出；没有 post-process depth 可调 */
};

struct histb_rng_priv {
	struct hwrng rng;
	void __iomem *base;
	const struct histb_rng_soc *soc;
};

/*
 * Observed:
 * depth = 1 -> ~1ms
 * depth = 255 -> ~16ms
 */
static int histb_rng_wait(struct histb_rng_priv *priv)
{
	u32 val;

	return readl_relaxed_poll_timeout(priv->base + RNG_STAT, val,
					  val & priv->soc->count_mask,
					  1000, 30 * 1000);
}

static void histb_rng_init(void __iomem *base, unsigned int depth)
{
	u32 val;

	val = readl_relaxed(base + RNG_CTRL);

	val &= ~RNG_SOURCE;
	val |= 2;

	val &= ~POST_PROCESS_DEPTH;
	val |= min(depth, 0xffu) << 8;

	val |= POST_PROCESS_ENABLE;
	val |= DROP_ENABLE;

	writel_relaxed(val, base + RNG_CTRL);
}

/*
 * 振荡源 2 经 DRBG 输出，pre/post processing、drop 与 mix 全部关闭：这是厂商内核的 drv_trng_randnum 在这一代上设置的配置。
 */
static void histb_rng_drbg_init(void __iomem *base)
{
	u32 val = readl_relaxed(base + RNG_CTRL);

	val &= ~(GENMASK(15, 4) | RNG_SOURCE);
	val |= DRBG_ENABLE | 2;
	writel_relaxed(val, base + RNG_CTRL);
}

static int histb_rng_read(struct hwrng *rng, void *data, size_t max, bool wait)
{
	struct histb_rng_priv *priv = container_of(rng, typeof(*priv), rng);
	void __iomem *base = priv->base;

	for (int i = 0; i < max; i += sizeof(u32)) {
		if (!(readl_relaxed(base + RNG_STAT) & priv->soc->count_mask)) {
			if (!wait)
				return i;
			if (histb_rng_wait(priv)) {
				pr_err("failed to generate random number, generated %d\n",
				       i);
				return i ? i : -ETIMEDOUT;
			}
		}
		*(u32 *) (data + i) = readl_relaxed(base + RNG_NUMBER);
	}

	return max;
}

static unsigned int histb_rng_get_depth(void __iomem *base)
{
	return (readl_relaxed(base + RNG_CTRL) & POST_PROCESS_DEPTH) >> 8;
}

static ssize_t
depth_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct histb_rng_priv *priv = dev_get_drvdata(dev);
	void __iomem *base = priv->base;

	return sprintf(buf, "%u\n", histb_rng_get_depth(base));
}

static ssize_t
depth_store(struct device *dev, struct device_attribute *attr,
	    const char *buf, size_t count)
{
	struct histb_rng_priv *priv = dev_get_drvdata(dev);
	void __iomem *base = priv->base;
	unsigned int depth;

	if (kstrtouint(buf, 0, &depth))
		return -ERANGE;

	histb_rng_init(base, depth);
	return count;
}

static DEVICE_ATTR_RW(depth);

static struct attribute *histb_rng_attrs[] = {
	&dev_attr_depth.attr,
	NULL,
};

static umode_t histb_rng_attr_is_visible(struct kobject *kobj,
					 struct attribute *attr, int n)
{
	struct histb_rng_priv *priv = dev_get_drvdata(kobj_to_dev(kobj));

	return priv->soc->drbg ? 0 : attr->mode;
}

static const struct attribute_group histb_rng_group = {
	.attrs = histb_rng_attrs,
	.is_visible = histb_rng_attr_is_visible,
};
__ATTRIBUTE_GROUPS(histb_rng);

static int histb_rng_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct histb_rng_priv *priv;
	void __iomem *base;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	priv->base = base;
	priv->soc = device_get_match_data(dev);

	if (priv->soc->drbg)
		histb_rng_drbg_init(base);
	else
		histb_rng_init(base, 144);
	if (histb_rng_wait(priv)) {
		dev_err(dev, "cannot bring up device\n");
		return -ENODEV;
	}

	priv->rng.name = pdev->name;
	priv->rng.read = histb_rng_read;
	ret = devm_hwrng_register(dev, &priv->rng);
	if (ret) {
		dev_err(dev, "failed to register hwrng: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, priv);
	dev_set_drvdata(dev, priv);
	return 0;
}

static const struct histb_rng_soc histb_rng_soc = {
	.count_mask = DATA_COUNT,
};

static const struct histb_rng_soc hi3798cv200_rng_soc = {
	.count_mask = FIFO_DATA_COUNT,
	.drbg = true,
};

static const struct of_device_id histb_rng_of_match[] = {
	{ .compatible = "hisilicon,histb-rng", .data = &histb_rng_soc },
	{ .compatible = "hisilicon,hi3798cv200-rng", .data = &hi3798cv200_rng_soc },
	{ }
};
MODULE_DEVICE_TABLE(of, histb_rng_of_match);

static struct platform_driver histb_rng_driver = {
	.probe = histb_rng_probe,
	.driver = {
		.name = "histb-rng",
		.of_match_table = histb_rng_of_match,
		.dev_groups = histb_rng_groups,
	},
};

module_platform_driver(histb_rng_driver);

MODULE_DESCRIPTION("Hisilicon STB random number generator driver");
MODULE_LICENSE("Dual MIT/GPL");
MODULE_AUTHOR("David Yang <mmyangfl@gmail.com>");
