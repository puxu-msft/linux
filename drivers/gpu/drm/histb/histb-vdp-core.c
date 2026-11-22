// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DRM driver for the HiSTB VDP display engine family.
 *
 * The VDP block provides two display channels; the driver exposes DHD0 (the
 * HD channel) as one CRTC with a primary plane fed by the G0 graphics layer
 * and sends its output to the transmitter selected through the device tree
 * of_graph port@0 endpoint, which is a drm_bridge component of the same
 * aggregate.
 *
 * The generations differ in how that pipeline is built: the v2 engine routes
 * G0 -> GP0 -> MIXG0 -> CBM(MIX0) -> DHD0 and has write-back taps, the v4
 * engine admits G0 to the mixer directly (G0 -> CBM MIX0 -> DHD0) and has
 * none.  Their clocks, resets, enable orders and write-back blocks all live
 * in the backend; this file owns everything the aggregate has to do the same
 * way on both, so that the platform driver, the DRM device and the component
 * lifetime exist once.
 */

#include <linux/component.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_mode_config.h>
#include <drm/drm_of.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "histb-vdp.h"

static const struct component_master_ops histb_vdp_master_ops;

static const struct drm_encoder_funcs histb_vdp_encoder_funcs = {
	.destroy	= drm_encoder_cleanup,
};

DEFINE_DRM_GEM_DMA_FOPS(histb_vdp_fops);

static const struct drm_driver histb_vdp_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops			= &histb_vdp_fops,
	DRM_GEM_DMA_DRIVER_OPS,
	DRM_FBDEV_DMA_DRIVER_OPS,
	.name			= "histb-vdp",
	.desc			= "HiSTB VDP display engine",
	.major			= 1,
	.minor			= 0,
};

static int histb_vdp_bind(struct device *dev)
{
	struct histb_vdp *vdp = dev_get_drvdata(dev);
	const struct histb_vdp_variant *variant = vdp->variant;
	struct histb_vdp_display *disp;
	struct drm_device *drm;
	struct drm_bridge *bridge;
	struct drm_connector *connector;
	int ret;

	ret = component_bind_all(dev, NULL);
	if (ret)
		return dev_err_probe(dev, ret, "component_bind_all failed\n");

	/*
	 * Only the devm variant of this lookup exists, so each rebind adds a
	 * bridge reference tied to this device rather than to the aggregate.
	 * They are all dropped when the device goes away; the transmitter
	 * outlives the aggregate by construction (it is a component of it).
	 */
	bridge = devm_drm_of_get_bridge(dev, dev->of_node, 0, 0);
	if (IS_ERR(bridge)) {
		ret = PTR_ERR(bridge);
		dev_err_probe(dev, ret, "failed to find downstream HDMI bridge\n");
		goto err_unbind;
	}

	drm = drm_dev_alloc(&histb_vdp_driver, dev);
	if (IS_ERR(drm)) {
		ret = PTR_ERR(drm);
		dev_err_probe(dev, ret, "drm_dev_alloc failed\n");
		goto err_unbind;
	}

	disp = drmm_kzalloc(drm, variant->display_size, GFP_KERNEL);
	if (!disp) {
		ret = -ENOMEM;
		goto err_put;
	}
	disp->vdp = vdp;
	disp->drm = drm;
	drm->dev_private = disp;
	WRITE_ONCE(vdp->disp, disp);

	ret = drmm_mode_config_init(drm);
	if (ret) {
		dev_err_probe(dev, ret, "mode_config_init failed\n");
		goto err_put;
	}

	drm->mode_config.min_width	= variant->min_width;
	drm->mode_config.max_width	= variant->max_width;
	drm->mode_config.min_height	= variant->min_height;
	drm->mode_config.max_height	= variant->max_height;
	drm->mode_config.funcs		= variant->mode_config_funcs;

	/* one CRTC, so the vblank core must exist before the IRQ can be served
	 * and before any atomic commit waits on it
	 */
	ret = drm_vblank_init(drm, 1);
	if (ret) {
		dev_err_probe(dev, ret, "vblank_init failed\n");
		goto err_put;
	}

	ret = drm_universal_plane_init(drm, &disp->primary,
				       variant->plane_possible_crtcs,
				       variant->plane_funcs,
				       variant->formats,
				       variant->num_formats,
				       NULL, DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret) {
		dev_err_probe(dev, ret, "plane_init failed\n");
		goto err_put;
	}
	drm_plane_helper_add(&disp->primary, variant->plane_helper);

	/*
	 * The v2 mixer can take the alpha byte as a coverage weight and can
	 * treat the surface as premultiplied, so it offers both upstream modes
	 * next to "None".  The v4 pipeline has no such stage and only offers
	 * the layer's global opacity.
	 */
	if (variant->blend_mode_property) {
		ret = drm_plane_create_blend_mode_property(&disp->primary,
						BIT(DRM_MODE_BLEND_PIXEL_NONE) |
						BIT(DRM_MODE_BLEND_PREMULTI) |
						BIT(DRM_MODE_BLEND_COVERAGE));
		if (ret) {
			dev_err_probe(dev, ret, "blend_mode failed\n");
			goto err_put;
		}
	}

	ret = drm_plane_create_alpha_property(&disp->primary);
	if (ret) {
		dev_err_probe(dev, ret, "alpha failed\n");
		goto err_put;
	}

	ret = drm_crtc_init_with_planes(drm, &disp->crtc, &disp->primary, NULL,
					variant->crtc_funcs, variant->crtc_name);
	if (ret) {
		dev_err_probe(dev, ret, "crtc_init failed\n");
		goto err_put;
	}
	drm_crtc_helper_add(&disp->crtc, variant->crtc_helper);
	drm_crtc_vblank_reset(&disp->crtc);

	disp->encoder.possible_crtcs = drm_crtc_mask(&disp->crtc);
	ret = drm_encoder_init(drm, &disp->encoder, &histb_vdp_encoder_funcs,
			       DRM_MODE_ENCODER_TMDS, variant->encoder_name);
	if (ret) {
		dev_err_probe(dev, ret, "encoder_init failed\n");
		goto err_put;
	}

	ret = drm_bridge_attach(&disp->encoder, bridge, NULL,
				DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret) {
		dev_err_probe(dev, ret, "bridge_attach failed\n");
		goto err_put;
	}

	connector = drm_bridge_connector_init(drm, &disp->encoder);
	if (IS_ERR(connector)) {
		dev_err_probe(dev, PTR_ERR(connector),
			      "failed to init bridge connector\n");
		goto err_put;
	}
	drm_connector_attach_encoder(connector, &disp->encoder);

	if (variant->ops->display_init) {
		ret = variant->ops->display_init(disp);
		if (ret) {
			dev_err_probe(dev, ret, "display init failed\n");
			goto err_put;
		}
	}

	/*
	 * The transmitter has no usable HPD output (it is a level signal that
	 * storms while a sink is attached), so the connector the bridge helper
	 * created asks for DRM_CONNECTOR_POLL_CONNECT|DISCONNECT.  Without
	 * starting the poll work here nothing would ever re-detect a plug
	 * event.  The drmm variant stops with the DRM device.
	 */
	drmm_kms_helper_poll_init(drm);

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret) {
		dev_err_probe(dev, ret, "drm_dev_register failed\n");
		goto err_put;
	}

	drm_client_setup(drm, NULL);

	return 0;

err_put:
	WRITE_ONCE(vdp->disp, NULL);
	drm_dev_put(drm);
err_unbind:
	component_unbind_all(dev, NULL);
	return ret;
}

static void histb_vdp_unbind(struct device *dev)
{
	struct histb_vdp *vdp = dev_get_drvdata(dev);
	struct histb_vdp_display *disp = vdp->disp;
	struct drm_device *drm = disp->drm;

	/* drm_dev_unplug() also unregisters; tear down while clocks are on */
	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);

	WRITE_ONCE(vdp->disp, NULL);
	component_unbind_all(dev, NULL);
	drm_dev_put(drm);
}

static const struct component_master_ops histb_vdp_master_ops = {
	.bind		= histb_vdp_bind,
	.unbind		= histb_vdp_unbind,
};

/*
 * Who belongs to this pipeline is read out of the device tree, not out of the
 * order someone happens to echo into unbind: the transmitter on the far side
 * of our own graph endpoint is added as a component of this aggregate, so
 * removing its driver takes the DRM device down before that driver's bridge,
 * regmaps and MMIO are released.  Without it, unbinding the transmitter under
 * a live card lets the atomic paths dereference the freed bridge.
 */
static int histb_vdp_add_master(struct device *dev)
{
	struct component_match *match = NULL;
	struct device_node *endpoint;

	for_each_endpoint_of_node(dev->of_node, endpoint) {
		struct device_node *remote =
			of_graph_get_remote_port_parent(endpoint);

		if (remote && of_device_is_available(remote))
			drm_of_component_match_add(dev, &match,
						   component_compare_of, remote);
		of_node_put(remote);
	}

	/*
	 * component_master_add_with_match() reads match->num before anything
	 * else, so an aggregate with no available component must fail instead
	 * of handing it NULL.  Disabling only the transmitter node is a valid
	 * DT configuration.
	 */
	if (!match)
		return -ENODEV;

	return component_master_add_with_match(dev, &histb_vdp_master_ops,
					       match);
}

static int histb_vdp_probe(struct platform_device *pdev)
{
	const struct histb_vdp_variant *variant;
	int ret;

	variant = of_device_get_match_data(&pdev->dev);
	if (!variant || !variant->probe)
		return -ENODEV;

	ret = variant->probe(pdev, variant);
	if (ret)
		return ret;

	ret = histb_vdp_add_master(&pdev->dev);
	if (ret) {
		variant->remove(pdev);
		return ret;
	}

	return 0;
}

static void histb_vdp_remove(struct platform_device *pdev)
{
	const struct histb_vdp_variant *variant =
		((struct histb_vdp *)platform_get_drvdata(pdev))->variant;

	component_master_del(&pdev->dev, &histb_vdp_master_ops);
	variant->remove(pdev);
}

/*
 * drm_dev_register() documents that a KMS driver stops the pipeline at
 * shutdown and reboot: the channel would otherwise keep scanning a
 * framebuffer whose pages are about to be torn down underneath it.
 */
static void histb_vdp_shutdown(struct platform_device *pdev)
{
	struct histb_vdp *vdp = platform_get_drvdata(pdev);
	struct histb_vdp_display *disp = READ_ONCE(vdp->disp);

	if (disp)
		drm_atomic_helper_shutdown(disp->drm);
}

static int histb_vdp_runtime_suspend(struct device *dev)
{
	struct histb_vdp *vdp = dev_get_drvdata(dev);

	if (vdp->variant->ops->runtime_suspend)
		return vdp->variant->ops->runtime_suspend(dev);
	return 0;
}

static int histb_vdp_runtime_resume(struct device *dev)
{
	struct histb_vdp *vdp = dev_get_drvdata(dev);

	if (vdp->variant->ops->runtime_resume)
		return vdp->variant->ops->runtime_resume(dev);
	return 0;
}

static int histb_vdp_suspend(struct device *dev)
{
	struct histb_vdp *vdp = dev_get_drvdata(dev);

	if (vdp->variant->ops->system_suspend)
		return vdp->variant->ops->system_suspend(dev);
	return 0;
}

static int histb_vdp_resume(struct device *dev)
{
	struct histb_vdp *vdp = dev_get_drvdata(dev);

	if (vdp->variant->ops->system_resume)
		return vdp->variant->ops->system_resume(dev);
	return 0;
}

static const struct dev_pm_ops histb_vdp_pm_ops = {
	RUNTIME_PM_OPS(histb_vdp_runtime_suspend, histb_vdp_runtime_resume, NULL)
	SYSTEM_SLEEP_PM_OPS(histb_vdp_suspend, histb_vdp_resume)
};

static const struct of_device_id histb_vdp_of_match[] = {
	{ .compatible = "hisilicon,hi3798mv100-vdp",
	  .data = &histb_vdp2_mv100_variant },
	/*
	 * Hi3716CV200 drives its output channel through the same register file:
	 * every offset this driver touches is identical on this part except the
	 * GP0 zoom block, whose cv200 offsets and 4.12 precisions are recorded in
	 * histb-vdp2-regs.h beside the board reading that fixed them.
	 */
	{ .compatible = "hisilicon,hi3716cv200-vdp",
	  .data = &histb_vdp2_cv200_variant },
	{ .compatible = "hisilicon,hi3798mv200-vdp",
	  .data = &histb_vdp4_mv200_variant },
	/*
	 * Hi3798MV300 的 VDP 与上面同属 v4 引擎，但寄存器布局、上电顺序与像素钟来源不同，见 histb-vdp4.c 的 Hi3798MV300 一节。
	 */
	{ .compatible = "hisilicon,hi3798mv300-vdp",
	  .data = &histb_vdp4_mv300_variant },
	{ }
};
MODULE_DEVICE_TABLE(of, histb_vdp_of_match);

static struct platform_driver histb_vdp_platform_driver = {
	.probe		= histb_vdp_probe,
	.remove		= histb_vdp_remove,
	.shutdown	= histb_vdp_shutdown,
	.driver		= {
		.name			= "histb-vdp",
		.of_match_table		= histb_vdp_of_match,
		.pm			= pm_ptr(&histb_vdp_pm_ops),
	},
};
module_platform_driver(histb_vdp_platform_driver);

MODULE_AUTHOR("David Yang <mmyangfl@gmail.com>");
MODULE_DESCRIPTION("DRM driver for the HiSTB VDP display engine");
MODULE_LICENSE("GPL");
