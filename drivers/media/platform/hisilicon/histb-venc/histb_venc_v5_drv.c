// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon HiSTB V500 encoder: device binding and lifetime.
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <media/media-device.h>
#include <media/v4l2-device.h>

#include "histb-venc.h"
#include "histb_venc_v5.h"

static int histb_venc_v5_claim(struct histb_venc_v5 *enc)
{
	/*
	 * The block is reset by a request in the clock and reset generator, not
	 * by a register of its own, and the vendor pulses it before any
	 * configuration. Configure only after the request is released: the
	 * registers do not retain anything written during the pulse.
	 */
	/*
	 * The reset request is held un-pulsed for now: on the board, claiming it
	 * and pulsing the bit the vendor's CRG35 layout names for this block cost
	 * the neighbouring decoder its launches (a launch then reports no
	 * interrupt and a busy state), so the bit number is not settled -- see the
	 * ABI note. The block is already out of reset after the bootloader.
	 */
	if (enc->reset)
		reset_control_deassert(enc->reset);

	return histb_venc_v5_buffers_alloc(enc);
}

static int histb_venc_v5_runtime_resume(struct device *dev)
{
	struct histb_venc_v5 *enc = dev_get_drvdata(dev);
	int ret;

	ret = clk_bulk_prepare_enable(enc->nclks, enc->clks);
	if (ret)
		return ret;
	enc->clocks_on = true;
	/* Nothing latched during the powered-down window may survive. */
	writel(V500_INT_CLEAR_ALL, enc->base + V500_INT_CLEAR);
	writel(V500_INT_ENABLE_DEFAULT, enc->base + V500_INT_ENABLE);

	return 0;
}

static int histb_venc_v5_runtime_suspend(struct device *dev)
{
	struct histb_venc_v5 *enc = dev_get_drvdata(dev);

	writel(0, enc->base + V500_INT_ENABLE);
	writel(V500_INT_CLEAR_ALL, enc->base + V500_INT_CLEAR);
	clk_bulk_disable_unprepare(enc->nclks, enc->clks);
	enc->clocks_on = false;

	return 0;
}

static void histb_venc_v5_release(struct histb_venc_v5 *enc)
{
	histb_venc_v5_buffers_free(enc);
	if (enc->clocks_on) {
		clk_bulk_disable_unprepare(enc->nclks, enc->clks);
		enc->clocks_on = false;
	};
}

int histb_venc_v5_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct histb_venc_v5 *enc;
	int ret;

	enc = devm_kzalloc(dev, sizeof(*enc), GFP_KERNEL);
	if (!enc)
		return -ENOMEM;

	enc->dev = dev;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "no 32-bit DMA aperture\n");
	platform_set_drvdata(pdev, enc);
	mutex_init(&enc->lock);

	enc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(enc->base))
		return PTR_ERR(enc->base);

	enc->irq = platform_get_irq(pdev, 0);
	if (enc->irq < 0)
		return enc->irq;

	enc->nclks = devm_clk_bulk_get_all(dev, &enc->clks);
	if (enc->nclks < 0)
		return enc->nclks;

	if (enc->nclks == 0)
		return dev_err_probe(dev, -ENOENT, "no clocks given\n");

	enc->reset = devm_reset_control_get_optional_exclusive(dev, NULL);
	if (IS_ERR(enc->reset))
		return dev_err_probe(dev, PTR_ERR(enc->reset),
				     "failed to get reset\n");

	ret = histb_venc_v5_claim(enc);
	if (ret)
		return ret;

	pm_runtime_set_suspended(dev);
	pm_runtime_enable(dev);

	ret = histb_venc_v5_v4l2_init(enc);
	if (ret)
		goto release;

	dev_info(dev, "V500 core claimed\n");

	return 0;

release:
	pm_runtime_disable(dev);
	histb_venc_v5_release(enc);
	return ret;
}

void histb_venc_v5_remove(struct platform_device *pdev)
{
	struct histb_venc_v5 *enc = platform_get_drvdata(pdev);

	histb_venc_v5_v4l2_exit(enc);
	pm_runtime_disable(&pdev->dev);
	histb_venc_v5_release(enc);
}

const struct histb_venc_variant histb_venc_v5_variant = {
	.probe			= histb_venc_v5_probe,
	.remove			= histb_venc_v5_remove,
	.runtime_suspend	= histb_venc_v5_runtime_suspend,
	.runtime_resume		= histb_venc_v5_runtime_resume,
	.system_suspend		= pm_runtime_force_suspend,
	.system_resume		= pm_runtime_force_resume,
};
