/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Shared contract between the HiSTB video encoder driver core and its
 * per-generation backends.
 *
 * The v2 block (VEDU V2R8, the vendor's venc_v1.0 layer) takes register writes
 * and a packet ring and only codes IDR pictures; the v5 block (V500, its
 * venc_v2.0 layer) takes a command image and has its own memory-management unit
 * and picture types.  Their V4L2 M2M nodes stay with the backends; the core
 * owns the platform driver identity, the device-tree match and the runtime-PM
 * dispatch both generations need.
 */
#ifndef __HISTB_VENC_H__
#define __HISTB_VENC_H__

#include <linux/device.h>
#include <linux/platform_device.h>

/* The node name is the merged driver's, whichever backend a chip binds. */
#define HISTB_VENC_NAME		"histb-venc"

struct histb_venc_variant {
	int (*probe)(struct platform_device *pdev);
	void (*remove)(struct platform_device *pdev);
	int (*runtime_suspend)(struct device *dev);
	int (*runtime_resume)(struct device *dev);
	int (*system_suspend)(struct device *dev);
	int (*system_resume)(struct device *dev);
};

extern const struct histb_venc_variant histb_venc_v2_variant;
extern const struct histb_venc_variant histb_venc_v5_variant;

int histb_venc_v2_probe(struct platform_device *pdev);
void histb_venc_v2_remove(struct platform_device *pdev);
int histb_venc_v5_probe(struct platform_device *pdev);
void histb_venc_v5_remove(struct platform_device *pdev);

#endif /* __HISTB_VENC_H__ */
