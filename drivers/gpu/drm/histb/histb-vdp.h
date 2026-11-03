/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Shared contract between the HiSTB VDP display driver core and its
 * per-generation backends.
 *
 * The two generations keep their own register programming: the v2 pipeline
 * has a GP0 stage the v4 one does not, their clocks and resets are requested
 * differently and only v2 has write-back taps.  What they share is the DRM
 * device, the plane/crtc/encoder scaffolding and the component lifetime, and
 * that is what the core in histb-vdp-core.c owns.
 */
#ifndef __HISTB_VDP_H__
#define __HISTB_VDP_H__

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/types.h>

#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_encoder.h>
#include <drm/drm_mode_config.h>
#include <drm/drm_plane.h>
#include <drm/drm_plane_helper.h>

struct histb_vdp;
struct histb_vdp_display;
struct histb_vdp_variant;

/*
 * The DRM objects every generation has.  A backend's own display structure
 * embeds this as its first member and keeps whatever else it needs (the
 * write-back connectors on v2) behind it.
 */
struct histb_vdp_display {
	struct histb_vdp	*vdp;
	struct drm_device	*drm;
	struct drm_plane	primary;
	struct drm_crtc		crtc;
	struct drm_encoder	encoder;
};

/*
 * Generation hooks the core calls.  Every one is optional: an absent
 * system-sleep hook means the generation has nothing to do there.
 */
struct histb_vdp_ops {
	int (*runtime_suspend)(struct device *dev);
	int (*runtime_resume)(struct device *dev);
	int (*system_suspend)(struct device *dev);
	int (*system_resume)(struct device *dev);
	/*
	 * Called after the core has created the plane, CRTC, encoder and
	 * connector, and with the DRM device fully initialised but not yet
	 * registered.  A backend uses it to publish itself to its interrupt
	 * path and to create the objects the other generation lacks.
	 */
	int (*display_init)(struct histb_vdp_display *disp);
};

struct histb_vdp_variant {
	const struct histb_vdp_ops	*ops;
	/* Generation data handed to @probe, NULL for a single-generation part. */
	const void			*data;
	int (*probe)(struct platform_device *pdev,
		     const struct histb_vdp_variant *variant);
	void (*remove)(struct platform_device *pdev);

	size_t				display_size;
	unsigned int			min_width;
	unsigned int			min_height;
	unsigned int			max_width;
	unsigned int			max_height;

	const u32			*formats;
	unsigned int			num_formats;
	const struct drm_plane_funcs	*plane_funcs;
	const struct drm_plane_helper_funcs *plane_helper;
	const struct drm_crtc_funcs	*crtc_funcs;
	const struct drm_crtc_helper_funcs *crtc_helper;
	const struct drm_mode_config_funcs *mode_config_funcs;

	/* What the plane advertises to generic atomic clients. */
	unsigned int			plane_possible_crtcs;
	bool				blend_mode_property;

	const char			*crtc_name;
	const char			*encoder_name;
};

/*
 * Per-device state the core owns.  A backend's structure has to put this
 * first so that platform_get_drvdata() and the core's bind/unbind agree on
 * the address.
 */
struct histb_vdp {
	struct device			*dev;
	const struct histb_vdp_variant	*variant;
	struct histb_vdp_display	*disp;
};

/* Provided by the generation backends; referenced by the core's match table. */
extern const struct histb_vdp_variant histb_vdp2_mv100_variant;
extern const struct histb_vdp_variant histb_vdp2_cv200_variant;
extern const struct histb_vdp_variant histb_vdp4_mv200_variant;

#endif /* __HISTB_VDP_H__ */
