// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon HiSTB video decoder driver, V5R6 generation.
 *
 * Claims the block, checks its clock-enable, reset and interrupt contract
 * on the board and hands frames to the protocol sources.
 */
#include <linux/bitfield.h>
#include <linux/mfd/syscon.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include "histb_vdec_v5.h"

/*
 * A reset request only takes effect while the unit is clocked, and its
 * acknowledgement is a level in the status word rather than a latch, so the
 * request is held until the bit arrives and then dropped.
 *
 * The decode core is reset through its own request bit (the vendor
 * names that unit MFDE). Pulling the whole
 * block down instead would take the stream distributor and the bitplane engine
 * with it, and the vendor's bring-up for this part does not do that.
 */
static int histb_vdec_v5_reset(struct histb_vdec_v5 *dec)
{
	u32 state;
	int ret;

	histb_vdec_v5_clocks(dec, true);
	histb_vdec_v5_write(dec, V5R6_RESET, V5R6_RESET_CORE);
	ret = readl_relaxed_poll_timeout(dec->base + V5R6_RST_STATE, state,
					 state & V5R6_RST_STATE_CORE, 10, 30000);
	histb_vdec_v5_write(dec, V5R6_RESET, 0);
	if (ret)
		return dev_err_probe(dec->dev, ret,
				     "core reset never acknowledged (status %#x)\n",
				     state);

	ret = readl_relaxed_poll_timeout(dec->base + V5R6_RST_STATE, state,
					 !(state & V5R6_RST_STATE_CORE), 10, 30000);
	if (ret)
		return dev_err_probe(dec->dev, ret,
				     "core stayed in reset (status %#x)\n", state);

	/*
	 * The mask word disables by setting a bit, so asking for the completion
	 * line alone means masking everything else. Both latches are cleared
	 * first, because the block was left running by whoever held it last.
	 */
	histb_vdec_v5_write(dec, V5R6_INT_STATE,
			  V5R6_INT_DEC_OVER | V5R6_INT_DEC_ERR);
	histb_vdec_v5_write(dec, V5R6_INT_MASK,
			  U32_MAX ^ (V5R6_INT_DEC_OVER | V5R6_INT_DEC_ERR));

	dev_dbg(dec->dev, "ready: clksel %#x status %#x\n",
		histb_vdec_v5_read(dec, V5R6_CLKSEL),
		histb_vdec_v5_read(dec, V5R6_RST_STATE));

	return 0;
}

int histb_vdec_v5_runtime_resume(struct device *dev)
{
	struct histb_vdec_v5 *dec = dev_get_drvdata(dev);

	return histb_vdec_v5_reset(dec);
}

int histb_vdec_v5_runtime_suspend(struct device *dev)
{
	struct histb_vdec_v5 *dec = dev_get_drvdata(dev);

	/* A level line must not stay armed on an unclocked block. */
	histb_vdec_v5_write(dec, V5R6_INT_MASK, ~0U);
	histb_vdec_v5_write(dec, V5R6_INT_STATE, ~0U);
	histb_vdec_v5_clocks(dec, false);

	return 0;
}

static irqreturn_t histb_vdec_v5_irq(int irq, void *data)
{
	struct histb_vdec_v5 *dec = data;
	u32 state;

	spin_lock(&dec->irq_lock);
	state = histb_vdec_v5_read(dec, V5R6_INT_STATE);
	histb_vdec_v5_write(dec, V5R6_INT_STATE, state);
	dec->int_state = state;
	spin_unlock(&dec->irq_lock);

	/*
	 * A picture the core flagged has stopped just as one it finished
	 * has: the wait is for the core to let go of the surface, and the
	 * state word says what came of it.
	 */
	if (state & (V5R6_INT_DEC_OVER | V5R6_INT_DEC_ERR))
		complete(&dec->done);

	return IRQ_HANDLED;
}

int histb_vdec_v5_probe(struct platform_device *pdev,
			const struct histb_vdec_gen *gen)
{
	struct device *dev = &pdev->dev;
	struct histb_vdec_v5 *dec;
	int ret;

	dec = devm_kzalloc(dev, sizeof(*dec), GFP_KERNEL);
	if (!dec)
		return -ENOMEM;

	dec->dev = dev;
	dec->chip = gen->chip;
	dec->common.priv = dec;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "no 32-bit DMA aperture\n");
	dec->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dec->base))
		return PTR_ERR(dec->base);

	dec->irq = platform_get_irq_byname(pdev, "vdm");
	if (dec->irq < 0)
		return dec->irq;

	ret = devm_request_threaded_irq(dev, dec->irq, NULL, histb_vdec_v5_irq,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					dev_name(dev), dec);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request the decode irq\n");

	/*
	 * The request above starts masked: nothing may be latched until the
	 * handler's own completion object and locks exist, which is the case
	 * from here on.
	 */

	mutex_init(&dec->hw_lock);
	mutex_init(&dec->common.vdev_lock);
	spin_lock_init(&dec->common.job_lock);
	spin_lock_init(&dec->irq_lock);
	init_completion(&dec->done);
	init_waitqueue_head(&dec->open_wq);
	INIT_DELAYED_WORK(&dec->common.watchdog, histb_vdec_v5_watchdog);

	ret = histb_vdec_v5_pool_alloc(dec);
	if (ret)
		return dev_err_probe(dev, ret, "cannot allocate the message pool\n");

	dec->crg = syscon_regmap_lookup_by_compatible("hisilicon,hi3798mv200-crg");
	if (IS_ERR(dec->crg))
		return dev_err_probe(dev, PTR_ERR(dec->crg),
				     "cannot reach the clock and reset generator\n");

	platform_set_drvdata(pdev, dec);

	pm_runtime_set_suspended(dev);
	pm_runtime_enable(dev);

	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		goto disable_pm;
	enable_irq(dec->irq);
	ret = histb_vdec_core_init(&dec->common, pdev, gen);
	if (ret)
		goto put_pm;

	dev_info(dev, "V5R6 core claimed\n");
	pm_runtime_put(dev);

	return 0;

put_pm:
	pm_runtime_put(dev);
	pm_runtime_set_suspended(dev);
disable_pm:
	pm_runtime_disable(dev);

	return ret;
}

void histb_vdec_v5_remove(struct platform_device *pdev)
{
	struct histb_vdec_v5 *dec = platform_get_drvdata(pdev);

	histb_vdec_core_stop(&dec->common);
	/*
	 * A file handle that was open when the node went away still owns driver
	 * state and its release runs after this point. Let those drain before
	 * the mem2mem device and the v4l2 device go, or the close path
	 * dereferences memory the platform device gave back.
	 */
	if (!wait_event_timeout(dec->open_wq,
				atomic_read(&dec->open_count) == 0,
				msecs_to_jiffies(10000)))
		dev_warn(dec->dev, "%d file handles still open at remove\n",
			 atomic_read(&dec->open_count));
	disable_irq(dec->irq);
	cancel_delayed_work_sync(&dec->common.watchdog);
	histb_vdec_core_exit(&dec->common);
	pm_runtime_disable(&pdev->dev);
}
