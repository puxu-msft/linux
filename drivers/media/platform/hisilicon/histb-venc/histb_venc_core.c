// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon HiSTB video encoder driver.
 *
 * The family has two blocks with genuinely different programming models: the
 * v2 VEDU of hi3716cv200 and hi3798mv100 (the vendor's V2R8 part) is driven by
 * register writes and a packet ring, the v5 V500 of hi3798mv200h by a command
 * image in DMA memory. Each generation keeps its own V4L2 M2M node in its
 * backend; this file owns the platform driver so that the device-tree match,
 * the module and the runtime-PM entry points exist once.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include "histb-venc.h"

static int histb_venc_probe(struct platform_device *pdev)
{
	const struct histb_venc_variant *variant;

	variant = of_device_get_match_data(&pdev->dev);
	if (!variant || !variant->probe)
		return -ENODEV;

	return variant->probe(pdev);
}

static void histb_venc_remove(struct platform_device *pdev)
{
	const struct histb_venc_variant *variant;

	variant = of_device_get_match_data(&pdev->dev);
	if (variant && variant->remove)
		variant->remove(pdev);
}

static int histb_venc_runtime_suspend(struct device *dev)
{
	const struct histb_venc_variant *variant;

	variant = of_device_get_match_data(dev);
	if (variant && variant->runtime_suspend)
		return variant->runtime_suspend(dev);
	return 0;
}

static int histb_venc_runtime_resume(struct device *dev)
{
	const struct histb_venc_variant *variant;

	variant = of_device_get_match_data(dev);
	if (variant && variant->runtime_resume)
		return variant->runtime_resume(dev);
	return 0;
}

static int histb_venc_suspend(struct device *dev)
{
	const struct histb_venc_variant *variant;

	variant = of_device_get_match_data(dev);
	if (variant && variant->system_suspend)
		return variant->system_suspend(dev);
	return 0;
}

static int histb_venc_resume(struct device *dev)
{
	const struct histb_venc_variant *variant;

	variant = of_device_get_match_data(dev);
	if (variant && variant->system_resume)
		return variant->system_resume(dev);
	return 0;
}

static const struct dev_pm_ops histb_venc_pm_ops = {
	RUNTIME_PM_OPS(histb_venc_runtime_suspend, histb_venc_runtime_resume,
		       NULL)
	SYSTEM_SLEEP_PM_OPS(histb_venc_suspend, histb_venc_resume)
};

static const struct of_device_id histb_venc_of_match[] = {
	{ .compatible = "hisilicon,hi3798mv100-venc",
	  .data = &histb_venc_v2_variant },
	{ .compatible = "hisilicon,hi3716cv200-venc",
	  .data = &histb_venc_v2_variant },
	{ }
};
MODULE_DEVICE_TABLE(of, histb_venc_of_match);

static struct platform_driver histb_venc_driver = {
	.probe		= histb_venc_probe,
	.remove		= histb_venc_remove,
	.driver		= {
		.name			= "histb-venc",
		.of_match_table		= histb_venc_of_match,
		.pm			= pm_ptr(&histb_venc_pm_ops),
	},
};
module_platform_driver(histb_venc_driver);

MODULE_AUTHOR("David Yang <mmyangfl@gmail.com>");
MODULE_DESCRIPTION("HiSilicon HiSTB H.264 encoder driver");
MODULE_LICENSE("GPL");
