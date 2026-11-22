// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DRM driver for the HiSTB VDP v4 display engine, the variant found on the
 * Hi3798MV200.
 *
 * The VDP block at 0xf8cc0000 provides two display channels. This driver
 * exposes DHD0 (the HD channel) as a single CRTC with a primary plane fed by
 * the G0 graphics (RGB) layer, admitted to the channel by the priority slot of
 * the DHD0 mixer word (G0 -> CBM MIX0 -> DHD0) -> HDMI. Scanout data leaves the
 * VDP towards the HDMI transmitter selected through of_graph port@0; the
 * downstream node is that transmitter's bridge.
 *
 * Clocking: the channel takes its pixel clock through CRG54 -- vo_hd_clk_sel
 * (bit 16) picks the source and vo_hd_clk_div (bits 19:18) divides it; this part
 * has no gate for the HD channel (bits 5:4 are reserved in the user guide, even
 * though the Hi3798MV210 decode of the same register names bit 4 vo_hd_cken).
 * The clock the driver asks for is the display FPLL in PERI_CRG_PLL10/PLL11 over
 * those two fields, which drivers/clk computes, and the transmitter is fed from
 * the same clock, so the two blocks cannot disagree about the pixel clock.
 *
 * Enable order is a hardware constraint, not taste: an access to the register
 * file while the HD domain is clocked and a reset request of the block is held
 * stalls the bus with the transaction outstanding. So the driver first puts
 * both requests (CRG211 bit 29 for the block, CRG54 bit 30 for the VOU) into
 * their running state while nothing clocks the domain yet, then raises the
 * clocks, and only then touches the register file.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/clients/drm_client_setup.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_mode_config.h>
#include <drm/drm_of.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "histb-vdp.h"
#include "histb-vdp4-regs.h"

/* The clocks in struct histb_vdp4.clks, in the order they are raised. */
enum histb_vdp4_clk {
	HISTB_VDP4_CLK_PIX,	/* HD channel pixel clock, shared with HDMI */
	HISTB_VDP4_CLK_BUS,	/* CRG54 bus gate of the channel */
	HISTB_VDP4_CLK_SDATE,	/* CRG54 gate of the channel's timing encoder */
	HISTB_VDP4_CLK_NUM,
};

/*
 * PERI_CRG54's VOU reset request.  The register file is only reachable with
 * this bit set: cleared while the HD domain runs, the next access to the block
 * locks the bus rather than returning a value.  Measured on the board from the
 * vendor bootloader, where CRG54=0x400d0019 reads VDPVER1=0x20170929 and
 * 0x000d0019 (that one bit back to 0, nothing else changed) hangs the read.
 * So the driver only ever raises it, and never pulses it down again: the pulse
 * cannot be made safe once the transmitter shares the clock.
 */
#define CRG54			0x0d8
#define  CRG54_VOU_SRST_REQ	BIT(30)

/*
 * Register layout of this part.  The mv200 header this driver started from
 * describes a different one: its VOAXICTRL at 0x30, its VO_MUX at 0x100 and its
 * G0 fetch fields inside G0_CTRL do not exist here.
 */
#define MV310_VO_MUX			0xc004
#define   MV310_VO_MUX_HDMI_SEL		BIT(12)
/* AXI master limits and read-channel priority. */
#define MV310_MST_OUTSTANDING		0x8000
#define MV310_MST_CTRL			0x8004
#define MV310_MST_RCHN_PRIO		0x8010

/* Graphics fetch unit. */
#define MV310_GFX_DTV_MODE		0x8980
#define MV310_GFX_MAC_CTRL		0x8a04
#define MV310_GFX_OUT_CTRL		0x8a08
#define   MV310_GFX_PALPHA_RANGE	BIT(0)
#define   MV310_GFX_BITEXT		GENMASK(9, 8)
#define MV310_GFX_SMMU_BYPASS		0x8a20
#define   MV310_GFX_SMMU_BYPASS_ALL	GENMASK(3, 0)
#define MV310_GFX_1555_ALPHA		0x8a28
#define MV310_GFX_SRC_CROP		0x8a48
#define MV310_GFX_ADDR_H		0x8a50
#define MV310_GFX_NADDR_H		0x8a58
#define MV310_GFX_NADDR_L		0x8a5c
#define   MV310_GFX_CK_GT_EN		BIT(12)
#define   MV310_GFX_READ_PROGRESSIVE	1	/* U_GFX_READ_CTRL.read_mode */
#define MV310_L0_ALPHA			0x5094

/* Post-scale stage GP0. */
#define MV310_GP0_DFPOS			0x6880
#define MV310_GP0_DLPOS			0x6884
#define MV310_GP0_VFPOS			0x6888
#define MV310_GP0_VLPOS			0x688c
#define MV310_GP0_ZME_HINFO		0x6a00
#define MV310_GP0_ZME_HSP		0x6a04
#define MV310_GP0_ZME_VINFO		0x6a10
#define MV310_GP0_ZME_VSP		0x6a14
#define   MV310_GP0_MUX1		GENMASK(20, 19)
#define   MV310_GP0_MUX2		GENMASK(22, 21)
#define   MV310_GP0_BOUT		GENMASK(18, 17)
#define   MV310_GP0_UPD_PROGRESSIVE	BIT(29)
#define   MV310_GP0_READ_MODE		BIT(30)
#define   MV310_GP0_EN			BIT(31)

/* Display channel DHD0 and its HDMI interface. */
#define MV310_DHD0_CTRL1		0xc204
#define MV310_DHD0_VTTHD3		0xc224
#define MV310_DHD0_VTTHD		0xc228
#define MV310_INTF_HDMI_CTRL		0xc600
#define   MV310_INTF_HDMI_RGB		BIT(6)	/* set: RGB lanes, clear: YCbCr */
#define MV310_INTF_HDMI_SYNC_INV	0xc604
#define   MV310_INTF_HSYNC_INV		BIT(1)
#define   MV310_INTF_VSYNC_INV		BIT(2)
#define MV310_INTF_HDMI_CLIP_L		0xc608
#define MV310_INTF_HDMI_CLIP_H		0xc60c
#define MV310_CLIP_FULL			GENMASK(29, 0)
/* regup + iop + fpga_lmt_width bit 3: the idle channel word. */
#define MV310_DHD0_CTRL_DEFAULT		0x00008011

struct histb_vdp4;

/*
 * 一种寄存器布局的全部硬件编程，由变体的 data 选定。DRM 对象、中断处理与设备生命周期各布局共用；上电顺序、像素钟来源与寄存器位置各布局不同。
 */
struct histb_vdp4_hw {
	/* 打开时钟并让寄存器文件可访问；中断处理在它返回之后才会访问寄存器 */
	int (*power_on)(struct histb_vdp4 *vdp);
	/* power_on 的逆过程；调用前中断处理已停止访问寄存器 */
	void (*power_off)(struct histb_vdp4 *vdp);
	enum drm_mode_status (*mode_valid)(struct histb_vdp4 *vdp,
					   const struct drm_display_mode *mode);
	/* 像素钟、管线与通道时序，接口保持关闭，由 intf_set() 打开 */
	void (*enable)(struct histb_vdp4 *vdp,
		       const struct drm_display_mode *mode);
	void (*g0_set)(struct histb_vdp4 *vdp, const struct drm_framebuffer *fb,
		       struct drm_gem_dma_object *gem, bool enable, u8 alpha);
	void (*intf_set)(struct histb_vdp4 *vdp, bool enable);
	/* G0 取数宽高字段能表示的最大值 */
	unsigned int fetch_max;
};

struct histb_vdp4 {
	struct histb_vdp	base;
	struct device		*dev;
	void __iomem		*regs;
	const struct histb_vdp4_hw	*hw;

	struct clk_bulk_data	clks[HISTB_VDP4_CLK_NUM];

	struct reset_control	*rst_vdp;

	struct regmap		*crg;

	int			irq;
	/* 寄存器文件可访问；中断处理据此避开断电之后的访问 */
	bool			powered;
};

/*
 * The DRM objects belong to the aggregate driver, not to the platform device:
 * they come into existence when the pipeline the device tree describes is up
 * and go away when it comes down, which can happen while this device stays
 * probed (the transmitter is a component of the aggregate). devm cannot cover
 * that lifetime, so this block is allocated by the aggregate's bind and
 * released by its unbind.
 */
struct histb_vdp4_display {
	struct histb_vdp_display	base;
	struct histb_vdp4		*vdp;
};

static inline struct histb_vdp4_display *
plane_to_display(struct drm_plane *plane)
{
	struct histb_vdp_display *base =
		container_of(plane, struct histb_vdp_display, primary);

	return container_of(base, struct histb_vdp4_display, base);
}

static inline struct histb_vdp4_display *
crtc_to_display(struct drm_crtc *crtc)
{
	struct histb_vdp_display *base =
		container_of(crtc, struct histb_vdp_display, crtc);

	return container_of(base, struct histb_vdp4_display, base);
}

static inline struct histb_vdp4 *plane_to_vdp(struct drm_plane *plane)
{
	return plane_to_display(plane)->vdp;
}

static inline struct histb_vdp4 *crtc_to_vdp(struct drm_crtc *crtc)
{
	return crtc_to_display(crtc)->vdp;
}

/*
 * The mode's pixel clock is the HD channel clock.  The CRG driver builds it
 * from the display FPLL in PERI_CRG_PLL10/PLL11 and CRG54's select and divider
 * for the channel, and it prefers to pass the PLL's output straight through, so
 * requesting the channel rate here is what programs the PLL: 297MHz for
 * 3840x2160@30, which is what the board's vendor system runs.  The transmitter
 * takes that clock 1:1 -- the CRG driver pins the ratio bit at 1:1 in the
 * clock's .prepare, because with 1:2 the VDP register file cannot be reached
 * at all on this part.
 */
static int histb_vdp4_mv310_set_pixclk(struct histb_vdp4 *vdp, unsigned int mode_khz)
{
	unsigned long rate = (unsigned long)mode_khz * 1000;
	struct clk *clk = vdp->clks[HISTB_VDP4_CLK_PIX].clk;
	int ret;

	ret = clk_set_rate(clk, rate);
	if (ret) {
		dev_err(vdp->dev, "failed to set pixel clock to %lu Hz: %d\n",
			rate, ret);
		return ret;
	}

	dev_dbg(vdp->dev, "pixel clock %lu Hz for mode clock %u kHz\n",
		clk_get_rate(clk), mode_khz);
	return 0;
}

static inline u32 vo_read(struct histb_vdp4 *vdp, u32 off)
{
	return readl_relaxed(vdp->regs + off);
}

static inline void vo_write(struct histb_vdp4 *vdp, u32 off, u32 val)
{
	writel_relaxed(val, vdp->regs + off);
}

/* ------------------------------------------------------------------ */
/* Plane (G0 graphics layer)                                           */
/* ------------------------------------------------------------------ */

/*
 * No per-pixel alpha is programmed in the atomic update path, so the ARGB
 * format is deliberately not exposed (drm core would warn about alpha formats
 * without blend mode support).
 */
static const u32 histb_vdp4_formats[] = {
	DRM_FORMAT_XRGB8888,
};

/* `value - 1`, clamped so that a zero porch/sync width never underflows */
static u32 histb_vdp4_m1(u32 v)
{
	return max(v, 1u) - 1;
}

/*
 * On this part the fetch parameters no longer live in G0_CTRL: the fetch unit
 * carries format, geometry, address and stride, and G0_CTRL keeps only alpha,
 * update mode and enable.
 */
static void histb_vdp4_mv310_g0_set(struct histb_vdp4 *vdp, const struct drm_framebuffer *fb,
			      struct drm_gem_dma_object *gem, bool enable, u8 alpha)
{
	u32 fmt, ctrl, last;

	if (!enable) {
		ctrl = vo_read(vdp, L0_CTRL);
		ctrl &= ~L0_EN;
		vo_write(vdp, L0_CTRL, ctrl);
		vo_write(vdp, L0_REGUP, 1);
		return;
	}

	/* KRGB888 consumes the padding byte, ARGB8888 its alpha byte. */
	fmt = fb->format->format == DRM_FORMAT_ARGB8888 ?
	      GFX0_IFMT_ARGB8888 : GFX0_IFMT_KRGB888;

	/*
	 * Fetch unit: 32-bit address, progressive raster read, SMMU bypassed,
	 * no compression, no colour key, no test pattern.  The request shape is
	 * VDP_GFX_InitFdrCfg()'s default (16-beat requests, 16-word length,
	 * not the offline master) and the fetch clock gate is on.
	 */
	vo_write(vdp, MV310_GFX_ADDR_H, 0);
	vo_write(vdp, GFX0_ADDR, (u32)(gem->dma_addr + fb->offsets[0]));
	vo_write(vdp, MV310_GFX_NADDR_H, 0);
	vo_write(vdp, MV310_GFX_NADDR_L, 0);
	vo_write(vdp, MV310_GFX_DTV_MODE, 0);
	vo_write(vdp, GFX0_CTRL, MV310_GFX_CK_GT_EN | MV310_GFX_READ_PROGRESSIVE);
	vo_write(vdp, MV310_GFX_MAC_CTRL, 0);
	vo_write(vdp, MV310_GFX_SMMU_BYPASS, MV310_GFX_SMMU_BYPASS_ALL);
	vo_write(vdp, MV310_GFX_OUT_CTRL,
		 FIELD_PREP(MV310_GFX_BITEXT, 3) | MV310_GFX_PALPHA_RANGE);
	vo_write(vdp, MV310_GFX_1555_ALPHA, 0xffff);
	vo_write(vdp, GFX0_FMT, fmt);
	vo_write(vdp, GFX0_IRESO,
		 FIELD_PREP(GFX0_RESO_W, fb->width - 1) |
		 FIELD_PREP(GFX0_RESO_H, fb->height - 1));
	vo_write(vdp, MV310_GFX_SRC_CROP, 0);
	vo_write(vdp, GFX0_ORESO,
		 FIELD_PREP(GFX0_RESO_W, fb->width - 1) |
		 FIELD_PREP(GFX0_RESO_H, fb->height - 1));
	/* Surface stride in 16-byte units; the compression stride stays 0. */
	vo_write(vdp, GFX0_STRIDE, FIELD_PREP(GFX0_LSTRIDE, fb->pitches[0] / 16));

	/*
	 * Layer stage: 1:1 placement at the display origin.  G0_ALPHA is the
	 * layer's background alpha; the layer's own opacity is the galpha field
	 * of G0_CTRL (the vendor's VDP_GFX_SetLayerGalpha writes exactly that
	 * byte), so 0xff would make every commit fully opaque.
	 */
	last = FIELD_PREP(L0_POS_X, fb->width - 1) |
	       FIELD_PREP(L0_POS_Y, fb->height - 1);
	vo_write(vdp, L0_DISP_START, 0);
	vo_write(vdp, L0_DISP_END, last);
	vo_write(vdp, L0_VIR_START, 0);
	vo_write(vdp, L0_VIR_END, last);
	vo_write(vdp, MV310_L0_ALPHA, 0xff);
	vo_write(vdp, L0_CTRL,
		 FIELD_PREP(L0_GALPHA, alpha) | L0_UPD_MODE | L0_NO_SEC | L0_EN);

	/* Latch G0 and GP0, then the graphics parameter-update bit. */
	vo_write(vdp, L0_REGUP, 1);
	readl(vdp->regs + L0_REGUP);
	vo_write(vdp, GP0_REGUP, 1);
	readl(vdp->regs + GP0_REGUP);
	vo_write(vdp, PARA_UPD, vo_read(vdp, PARA_UPD) | PARA_UPD_GFX(0));

	dev_dbg(vdp->dev, "g0 fetch: addr 0x%08x pitch %u %ux%u fourcc %p4cc\n",
		(u32)(gem->dma_addr + fb->offsets[0]), fb->pitches[0], fb->width,
		fb->height, &fb->format->format);
}

static void histb_vdp4_plane_atomic_update(struct drm_plane *plane,
					 struct drm_atomic_commit *state)
{
	struct histb_vdp4 *vdp = plane_to_vdp(plane);
	struct drm_plane_state *new_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_gem_dma_object *gem;
	struct drm_framebuffer *fb = new_state->fb;

	if (!fb)
		return;

	gem = drm_fb_dma_get_gem_obj(fb, 0);
	if (!gem)
		return;

	vdp->hw->g0_set(vdp, fb, gem, new_state->visible,
			new_state->alpha >> 8);
}

static void histb_vdp4_plane_atomic_disable(struct drm_plane *plane,
					  struct drm_atomic_commit *state)
{
	struct histb_vdp4 *vdp = plane_to_vdp(plane);

	vdp->hw->g0_set(vdp, NULL, NULL, false, 0);
}

static int histb_vdp4_plane_atomic_check(struct drm_plane *plane,
				       struct drm_atomic_commit *state)
{
	struct histb_vdp4 *vdp = plane_to_vdp(plane);
	struct drm_plane_state *new_state =
		drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *crtc_state;

	if (!new_state->fb)
		return 0;

	/* stride unit is mv100's (16 bytes) until a working system says otherwise */
	if (new_state->fb->pitches[0] % 16)
		return -EINVAL;

	/* G0 取数宽高字段的位宽随布局而定 */
	if (new_state->fb->width > vdp->hw->fetch_max ||
	    new_state->fb->height > vdp->hw->fetch_max)
		return -EINVAL;

	if (!new_state->crtc)
		return 0;

	crtc_state = drm_atomic_get_new_crtc_state(state, new_state->crtc);
	if (IS_ERR(crtc_state))
		return PTR_ERR(crtc_state);

	if (new_state->crtc_w > 4095 || new_state->crtc_h > 4095)
		return -EINVAL;

	/*
	 * This is what fills in new_state->visible, and atomic_update acts on
	 * it: without the helper visible stays false, so every plane commit
	 * switches the surface off again. can_position is false because the
	 * driver always places the surface at 0,0 with the framebuffer's own
	 * size.
	 */
	return drm_atomic_helper_check_plane_state(new_state, crtc_state,
						   DRM_PLANE_NO_SCALING,
						   DRM_PLANE_NO_SCALING,
						   false, true);
}

static const struct drm_plane_helper_funcs histb_vdp4_plane_helper_funcs = {
	/*
	 * No DRM_GEM_SHADOW_PLANE_HELPER_FUNCS here: they upcast the state with
	 * container_of() to struct drm_shadow_plane_state, which only the
	 * matching DRM_GEM_SHADOW_PLANE_FUNCS duplicate() allocates.
	 */
	.atomic_check		= histb_vdp4_plane_atomic_check,
	.atomic_update		= histb_vdp4_plane_atomic_update,
	.atomic_disable		= histb_vdp4_plane_atomic_disable,
};

static const struct drm_plane_funcs histb_vdp4_plane_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.destroy		= drm_plane_cleanup,
	.atomic_create_state	= drm_atomic_helper_plane_create_state,
	.atomic_duplicate_state	= drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
};

/* ------------------------------------------------------------------ */
/* CRTC (DHD0 channel)                                                 */
/* ------------------------------------------------------------------ */

/*
 * One-time display pipeline setup: route GP0 into the DHD0 mixer, set the AXI
 * outstanding limits and enable the block's clock gate.
 */

/*
 * One-time pipeline setup before the channel starts: the AXI master limits and
 * read-channel priority, and the graphics topology G0 -> GP0 -> DHD0.  GP0's
 * own mixer admits G0 in slot 0 and the channel's mixer admits GP0 in slot 0.
 * The mv200 header's VOAXICTRL words at 0x30/0x34 do not exist on this part;
 * its VO_MUX at 0x100 is this part's VO_MUX at 0xc004.
 */
static void histb_vdp4_mv310_pipeline_init(struct histb_vdp4 *vdp)
{
	u32 ctrl;

	/* Clock gate first: writes into a gated VDP domain are dropped. */
	ctrl = vo_read(vdp, VO_CTRL);
	ctrl |= VO_CTRL_CK_GT_EN;
	vo_write(vdp, VO_CTRL, ctrl);

	vo_write(vdp, MV310_MST_OUTSTANDING,
		 (vo_read(vdp, MV310_MST_OUTSTANDING) & ~0xff) | 0x2f);
	vo_write(vdp, MV310_MST_CTRL,
		 (vo_read(vdp, MV310_MST_CTRL) & ~GENMASK(8, 0)) |
		 BIT(8) | BIT(4) | 1);
	vo_write(vdp, MV310_MST_RCHN_PRIO,
		 vo_read(vdp, MV310_MST_RCHN_PRIO) | BIT(2));

	vo_write(vdp, MIXG0_BKG, 0);
	vo_write(vdp, MIXG0_BKALPHA, 0);
	vo_write(vdp, MIXG0_SLOT, 1);
	vo_write(vdp, CBM0_BKG, 0);
	vo_write(vdp, CBM0_SLOT, 2);
}

/*
 * Vertical thresholds, in the vendor's own arithmetic (the non-stereo branch of
 * PF_SetChnBaseTiming(), hal/common/drv_disp_hal.c:952-993 and
 * DISP_GFX_GetIsrPoint(), :734-802): thd1 is half a frame on the HD channel and
 * 60 % on the other, thd2 sits 1.5 ms worth of lines before the end of the
 * frame, thd3 a tenth of a frame. All three go in field-based mode, which is
 * what the vendor passes for any mode that is not frame-packed stereo
 * (DHD_VTXTHD_FIELD_MODE, hal/98mv200/hal_specific/vdp_software_selfdefine.h:3).
 * For 1280x720 at 60 Hz this is thd1 375, thd2 662, thd3 75.
 */
/*
 * Vertical thresholds, in the vendor's own arithmetic (the non-stereo branch of
 * PF_SetChnBaseTiming(), hal/common/drv_disp_hal.c:952-993 and
 * DISP_GFX_GetIsrPoint(), :734-802): thd1 is half a frame on the HD channel and
 * 60 % on the other, thd2 sits 1.5 ms worth of lines before the end of the
 * frame, thd3 a tenth of a frame. All three go in field-based mode, which is
 * what the vendor passes for any mode that is not frame-packed stereo.
 * The two words are DHD0_VTTHD (thd1/thd2) and DHD0_VTTHD3 (thd3/thd4); on this
 * part they sit at 0xc228 and 0xc224, the reverse of the mv200 header.
 */
static void histb_vdp4_mv310_set_thresholds(struct histb_vdp4 *vdp, u32 total, u32 vfb,
				      u32 vrefresh)
{
	u32 period = vrefresh ? 100000 / (vrefresh * 100) : 1;
	u32 thd1, thd2, thd3, lead;

	/* the vendor divides by this directly; a zero would be its own bug */
	if (period == 0)
		period = 1;

	thd1 = total * (HD_CHN == 0 ? 50 : 60) / 100;
	thd3 = total / 10;

	/* 2 * total / period for the HD channel, 7 * total / (4 * period) there */
	lead = HD_CHN == 0 ? (total * 2) / period : (total * 7) / (period * 4);
	if (lead > vfb)
		lead -= vfb;
	thd2 = total - lead;

	vo_write(vdp, MV310_DHD0_VTTHD,
		 FIELD_PREP(DHD_VTTHD1, thd1) | DHD_VTTHD1_MODE |
		 FIELD_PREP(DHD_VTTHD2, thd2) | DHD_VTTHD2_MODE);
	vo_write(vdp, MV310_DHD0_VTTHD3,
		 FIELD_PREP(DHD_VTTHD3, thd3) | DHD_VTTHD3_MODE);

	dev_dbg(vdp->dev, "thresholds: total %u vfb %u period %u -> thd %u/%u/%u\n",
		total, vfb, period, thd1, thd2, thd3);
}

/*
 * Channel timing, interface and post-scale stage: the channel counts
 * ACT + BB + FB per axis with the sync pulse inside BB, and the interface has
 * its own RGB-mode word at 0xc600.  The mv200 header's single-word VSYNC and
 * its DATA_SEL/SYNC_INV do not describe this block: here the front porch and
 * the bottom field live in separate words.
 */
static void histb_vdp4_mv310_setup_timing(struct histb_vdp4 *vdp,
				       const struct drm_display_mode *mode)
{
	u32 hbb = mode->htotal - mode->hsync_start;	/* sync + back porch */
	u32 hfb = mode->hsync_start - mode->hdisplay;
	u32 hpw = mode->hsync_end - mode->hsync_start;
	u32 vbb = mode->vtotal - mode->vsync_start;	/* sync + back porch */
	u32 vfb = mode->vsync_start - mode->vdisplay;
	u32 vpw = mode->vsync_end - mode->vsync_start;
	u32 last = FIELD_PREP(L0_POS_X, mode->hdisplay - 1) |
		   FIELD_PREP(L0_POS_Y, mode->vdisplay - 1);
	u32 ctrl, mux, inv = 0;

	/* VDP_CHIP_CFG_DHD0_CTRL_DEFAULT, interface still disabled. */
	ctrl = MV310_DHD0_CTRL_DEFAULT;
	vo_write(vdp, HD_CTRL, ctrl);
	vo_write(vdp, MV310_DHD0_CTRL1, 0);

	vo_write(vdp, HD_HSYNC1,
		 FIELD_PREP(DHD_HSYNC1_HACT, histb_vdp4_m1(mode->hdisplay)) |
		 FIELD_PREP(DHD_HSYNC1_HBB, histb_vdp4_m1(hbb)));
	vo_write(vdp, HD_HSYNC2, FIELD_PREP(DHD_HSYNC2_HFB, histb_vdp4_m1(hfb)));
	vo_write(vdp, HD_VSYNC,
		 FIELD_PREP(DHD_VSYNC_VACT, histb_vdp4_m1(mode->vdisplay)) |
		 FIELD_PREP(DHD_VSYNC_VBB, histb_vdp4_m1(vbb)));
	vo_write(vdp, HD_VSYNC_VFB, FIELD_PREP(DHD_VSYNC_VFB, histb_vdp4_m1(vfb)));
	/* Progressive: the vendor's table leaves the bottom field zeroed. */
	vo_write(vdp, HD_VPLUS, 0);
	vo_write(vdp, HD_VPLUS_VFB, 0);
	vo_write(vdp, HD_PWR,
		 FIELD_PREP(DHD_PWR_HPW, histb_vdp4_m1(hpw)) |
		 FIELD_PREP(DHD_PWR_VPW, histb_vdp4_m1(vpw)));

	histb_vdp4_mv310_set_thresholds(vdp, mode->vtotal, vfb, drm_mode_vrefresh(mode));

	/* Latch the timing with the interface still off. */
	vo_write(vdp, HD_CTRL, ctrl);

	/*
	 * VO_MUX.hdmi_sel = 0: DHD0 feeds the HDMI interface.  On this part the
	 * selector lives at 0xc004 bit 12 (U_VO_MUX in hi3798mv310/hi_reg_vdp.h);
	 * the mv200 header's 0x100/bit 3 is not this register.
	 */
	mux = vo_read(vdp, MV310_VO_MUX);
	vo_write(vdp, MV310_VO_MUX, mux & ~MV310_VO_MUX_HDMI_SEL);

	/* RGB 4:4:4 to the transmitter (VDP_DISP_SetHdmiMode(0)). */
	vo_write(vdp, MV310_INTF_HDMI_CTRL, MV310_INTF_HDMI_RGB);
	if (mode->flags & DRM_MODE_FLAG_NHSYNC)
		inv |= MV310_INTF_HSYNC_INV;
	if (mode->flags & DRM_MODE_FLAG_NVSYNC)
		inv |= MV310_INTF_VSYNC_INV;
	vo_write(vdp, MV310_INTF_HDMI_SYNC_INV, inv);
	vo_write(vdp, MV310_INTF_HDMI_CLIP_L, 0);
	vo_write(vdp, MV310_INTF_HDMI_CLIP_H, MV310_CLIP_FULL);

	/*
	 * Post-scale stage GP0: G0 in, 1:1, VDP_GP_ORDER_NULL (mux1=3, mux2=2,
	 * aout=0, bout=1 in VDP_GP_SetIpOrder), no ZME, no CSC/HDR.
	 */
	vo_write(vdp, GP0_CTRL, 0xff |
		 FIELD_PREP(MV310_GP0_MUX1, 3) |
		 FIELD_PREP(MV310_GP0_MUX2, 2) |
		 FIELD_PREP(MV310_GP0_BOUT, 1) |
		 MV310_GP0_UPD_PROGRESSIVE | MV310_GP0_READ_MODE | MV310_GP0_EN);
	vo_write(vdp, GP0_IRESO, last);
	vo_write(vdp, MV310_GP0_ZME_HINFO, mode->hdisplay - 1);
	vo_write(vdp, MV310_GP0_ZME_VINFO, (mode->vdisplay - 1) | BIT(18));
	vo_write(vdp, MV310_GP0_ZME_HSP, 0);
	vo_write(vdp, MV310_GP0_ZME_VSP, 0);
	vo_write(vdp, MV310_GP0_DFPOS, 0);
	vo_write(vdp, MV310_GP0_DLPOS, last);
	vo_write(vdp, MV310_GP0_VFPOS, 0);
	vo_write(vdp, MV310_GP0_VLPOS, last);
	vo_write(vdp, GP0_REGUP, 1);
	readl(vdp->regs + GP0_REGUP);

	/* Latch the timing again; the interface is enabled once G0 is up. */
	vo_write(vdp, HD_CTRL, ctrl);

}

static enum drm_mode_status
histb_vdp4_mv310_mode_valid(struct histb_vdp4 *vdp,
			    const struct drm_display_mode *mode)
{
	long rate;

	/* G0/GP0 position and resolution fields are 12 bits wide */
	if (mode->hdisplay > 4095 || mode->vdisplay > 4095)
		return MODE_BAD;

	/* 3840x2160 is the largest mode the pipeline reaches */
	if (mode->hdisplay > 3840 || mode->vdisplay > 2160)
		return MODE_BAD;

	if (mode->hdisplay % 2 || mode->vdisplay % 2)
		return MODE_BAD;

	/*
	 * DHD0_PWR holds VPW in 8 bits; the vertical/horizontal porch fields are
	 * 16 bits. An out-of-range timing would be truncated by FIELD_PREP into
	 * a different picture, so reject it before it reaches the registers.
	 */
	{
		unsigned int vsync = mode->vsync_end - mode->vsync_start;
		unsigned int vbb = vsync + mode->vtotal - mode->vsync_end;
		unsigned int vfb = mode->vsync_start - mode->vdisplay;
		unsigned int hsync = mode->hsync_end - mode->hsync_start;
		unsigned int hbb = hsync + mode->htotal - mode->hsync_end;
		unsigned int hfb = mode->hsync_start - mode->hdisplay;

		if (vsync > 255 || vbb > 65535 || vfb > 65535 ||
		    hsync > 65535 || hbb > 65535 || hfb > 65535)
			return MODE_BAD;
	}

	/* the HD channel clock can only produce rates that divide out exactly */
	rate = clk_round_rate(vdp->clks[HISTB_VDP4_CLK_PIX].clk,
			      mode->clock * 1000);
	if (rate < 0)
		return MODE_ERROR;

	/*
	 * The clock's search only accepts a rate it can solve without remainder,
	 * while mode->clock is the nominal rate rounded to whole kHz -- a few
	 * hundred Hz away from what the timing adds up to. Allow half a percent;
	 * keep the arithmetic in Hz so a large mismatch cannot overflow and look
	 * like a pass.
	 */
	if (abs(rate - (long)mode->clock * 1000) > (long)mode->clock * 5)
		return MODE_NOCLOCK;

	return MODE_OK;
}

static void histb_vdp4_mv310_enable(struct histb_vdp4 *vdp,
				    const struct drm_display_mode *mode)
{
	/* the channel clock is what the HDMI transmitter receives, too */
	histb_vdp4_mv310_set_pixclk(vdp, mode->clock);

	histb_vdp4_mv310_pipeline_init(vdp);

	histb_vdp4_mv310_setup_timing(vdp, mode);
}

/*
 * The channel word is written whole, never read-modify-written: these
 * registers do not read back, so a read-modify-write would write a word made
 * up of zeros plus the enable bit and clear iop, hdmi_mode and twochn_debug
 * again.  Enabling follows VDP_DISP_SetIntfEnable; disabling keeps the
 * vendor's idle channel word.
 */
static void histb_vdp4_mv310_intf_set(struct histb_vdp4 *vdp, bool enable)
{
	vo_write(vdp, HD_CTRL,
		 MV310_DHD0_CTRL_DEFAULT | (enable ? DHD_CTRL_INTF_EN : 0));
}

static enum drm_mode_status histb_vdp4_crtc_mode_valid(struct drm_crtc *crtc,
						     const struct drm_display_mode *mode)
{
	struct histb_vdp4 *vdp = crtc_to_vdp(crtc);

	return vdp->hw->mode_valid(vdp, mode);
}

static void histb_vdp4_crtc_atomic_enable(struct drm_crtc *crtc,
					struct drm_atomic_commit *state)
{
	struct histb_vdp4 *vdp = crtc_to_vdp(crtc);
	struct drm_crtc_state *new_state = drm_atomic_get_new_crtc_state(state, crtc);
	const struct drm_display_mode *mode = &new_state->adjusted_mode;
	struct drm_plane_state *pstate;
	struct drm_gem_dma_object *gem;
	struct drm_framebuffer *fb;

	/* 像素钟、管线与通道时序；接口在 G0 编程之后才打开 */
	vdp->hw->enable(vdp, mode);

	/*
	 * commit_planes runs before modeset_enables, and G0 drops register
	 * writes while the VDP pipeline is not up (the block then reads back
	 * its reset values), so (re)program the layer from the fb the commit
	 * just installed.
	 */
	pstate = drm_atomic_get_new_plane_state(state, crtc->primary);
	if (!pstate)
		pstate = crtc->primary->state;
	fb = pstate->fb;
	if (fb) {
		gem = drm_fb_dma_get_gem_obj(fb, 0);
		if (gem)
			vdp->hw->g0_set(vdp, fb, gem, pstate->visible,
					pstate->alpha >> 8);
	}

	/* G0 is latched; now start the interface. */
	vdp->hw->intf_set(vdp, true);

	drm_crtc_vblank_on(crtc);
}

static void histb_vdp4_crtc_atomic_disable(struct drm_crtc *crtc,
					 struct drm_atomic_commit *state)
{
	struct histb_vdp4 *vdp = crtc_to_vdp(crtc);

	drm_crtc_vblank_off(crtc);

	vdp->hw->intf_set(vdp, false);
}

static int histb_vdp4_crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct histb_vdp4 *vdp = crtc_to_vdp(crtc);

	if (vdp->irq < 0)
		return -EOPNOTSUPP;

	/*
	 * A set bit of VOINTMSK lets its source through, so enabling the vblank
	 * source means setting it -- see the note on the name in
	 * histb-vdp4-regs.h.
	 *
	 * Written whole, not read-modify-written: the board was measured with this
	 * word at 0xffffffff while the channel was running, and every source then
	 * passes into VOMSKINTSTA, which is what made INTID 122 fire about 2100
	 * times a second (the two channel thresholds nobody consumes, VTTHD1 and
	 * VTTHD3, were among them).  This driver consumes one source, so it owns
	 * the word and leaves the other bits down.
	 */
	vo_write(vdp, VO_INTMSK, HD_INT_VBLANK);
	return 0;
}

static void histb_vdp4_crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct histb_vdp4 *vdp = crtc_to_vdp(crtc);

	if (vdp->irq < 0)
		return;

	/*
	 * Ack before masking.  The latch is cleared by writing the status back to
	 * VOMSKINTSTA (the vendor's own ISR does that, hal/98mv200/vdp_hal_chn.c:1952),
	 * and a vblank that arrives while the mask bit is being dropped would
	 * otherwise stay latched with its source masked -- the handler then reads
	 * nothing, returns IRQ_NONE, and the level line never falls.  That is the
	 * second half of the storm measured above: VOINTMSK read 0xfffffffd with
	 * VOINTSTA still holding the vblank bit.
	 */
	vo_write(vdp, VO_MSKINTSTA, HD_INT_VBLANK);
	vo_write(vdp, VO_INTMSK, 0);
}

static void histb_vdp4_crtc_atomic_flush(struct drm_crtc *crtc,
				       struct drm_atomic_commit *state)
{
	struct drm_crtc_state *new_state = drm_atomic_get_new_crtc_state(state, crtc);

	if (new_state->event) {
		int ret = drm_crtc_vblank_get(crtc);

		spin_lock_irq(&crtc->dev->event_lock);
		if (ret) {
			/* No vblank reference: deliver now, do not strand the client. */
			drm_crtc_send_vblank_event(crtc, new_state->event);
		} else {
			/* The client's "on screen" is the frame boundary. */
			drm_crtc_arm_vblank_event(crtc, new_state->event);
		}
		new_state->event = NULL;	/* consumed: keep hw_done quiet */
		spin_unlock_irq(&crtc->dev->event_lock);
	}
}

static const struct drm_crtc_helper_funcs histb_vdp4_crtc_helper_funcs = {
	.mode_valid		= histb_vdp4_crtc_mode_valid,
	.atomic_enable		= histb_vdp4_crtc_atomic_enable,
	.atomic_disable		= histb_vdp4_crtc_atomic_disable,
	.atomic_flush		= histb_vdp4_crtc_atomic_flush,
};

static const struct drm_crtc_funcs histb_vdp4_crtc_funcs = {
	.set_config		= drm_atomic_helper_set_config,
	.page_flip		= drm_atomic_helper_page_flip,
	.destroy		= drm_crtc_cleanup,
	.atomic_create_state	= drm_atomic_helper_crtc_create_state,
	.atomic_duplicate_state	= drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_crtc_destroy_state,
	.enable_vblank		= histb_vdp4_crtc_enable_vblank,
	.disable_vblank		= histb_vdp4_crtc_disable_vblank,
};

static irqreturn_t histb_vdp4_irq_handler(int irq, void *data)
{
	struct histb_vdp4 *vdp = data;
	/*
	 * 显示对象用 core 维护的 base.disp：bind 出错与 unbind 时 core 会把它清成 NULL，本驱动原先自存的指针从不清空，解绑后被调到会碰已释放的 CRTC。
	 */
	struct histb_vdp_display *disp = READ_ONCE(vdp->base.disp);
	u32 sta;

	/*
	 * 中断线是共享申请的：断电（复位按住、配置时钟关闭）之后被调到时不能碰寄存器文件，那样的访问会挂住总线。
	 */
	if (!READ_ONCE(vdp->powered))
		return IRQ_NONE;

	sta = vo_read(vdp, VO_MSKINTSTA);
	if (!sta)
		return IRQ_NONE;

	/*
	 * The vendor ISR takes its status from VOMSKINTSTA (the *masked* status,
	 * hal/98mv200/hal_specific/vdp_software_selfdefine.c:877) and acknowledges
	 * by writing that same value back to it (hal/98mv200/vdp_hal_chn.c:1952),
	 * so dispatch and ack are the same word here.  VOINTMSK is misnamed: a bit
	 * of 1 lets the source through, because the vendor's "enable" path ORs it
	 * in (VDP_DISP_SetIntMask) and its "disable" path clears it -- reading the
	 * raw VOINTSTA and filtering with ~VO_INTMSK therefore discarded exactly
	 * the sources that were enabled.
	 *
	 * Ack every latched source, not just the vblank one: returning IRQ_NONE
	 * while a VDP status bit stays set makes the core disable the line
	 * ("nobody cared") and vblank dies with it.
	 */
	vo_write(vdp, VO_MSKINTSTA, sta);

	if (sta & ~HD_INT_VBLANK)
		dev_warn_ratelimited(vdp->dev, "vdp irq: unexpected status %08x\n",
				     sta);

	/* the line stays requested across an aggregate unbind, no CRTC then */
	if (disp && (sta & HD_INT_VBLANK))
		drm_crtc_handle_vblank(&disp->crtc);
	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/* Mode config / binding                                               */
/* ------------------------------------------------------------------ */

static const struct drm_mode_config_funcs histb_vdp4_mode_config_funcs = {
	.fb_create		= drm_gem_fb_create,
	.atomic_check		= drm_atomic_helper_check,
	.atomic_commit		= drm_atomic_helper_commit,
};

/* ------------------------------------------------------------------ */
/* Generation hooks                                                    */
/* ------------------------------------------------------------------ */

/*
 * The VOU reset request is CRG54[30]. A reset only reaches the block while it
 * is clocked, so the gates come up first: CRG54's own gate group is the clock
 * "vo" and the HD channel clock is "pix", both enabled by
 * histb_vdp4_runtime_resume(). The reset has to be held before it is released
 * and the block needs settling time afterwards, so the pulse comes with delays.
 */

/*
 * Put the block into the reset state a clocked access survives and then hand it
 * back: the VOU request raised (see CRG54_VOU_SRST_REQ for the measurement) and
 * the block's own request pulsed.  The caller enables the clocks first, so this
 * runs with the HD domain clocked; the channel control readback at the end is the
 * first live access of the register file and it has to answer.
 */
static int histb_vdp4_mv310_reset_release(struct histb_vdp4 *vdp)
{
	int ret;

	ret = regmap_update_bits(vdp->crg, CRG54, CRG54_VOU_SRST_REQ,
				 CRG54_VOU_SRST_REQ);
	if (ret)
		return ret;

	/*
	 * Pulse the block's own reset before the register file is touched at all.
	 * HiSilicon's own mainline-facing port for this IP (linux-6.12.y,
	 * drivers/gpu/drm/hisilicon/histb/histb-vdp.c, Hi3798CV200) asserts, waits
	 * and then deasserts for one stated reason: to drop every display state
	 * inherited from the boot-logo pipeline.  This port only ever deasserted, so
	 * whatever the vendor fastboot left inside the VDP state machines has been
	 * carried into every boot since.
	 *
	 * The pulse is deliberately not interrupted by a read.  That driver polls
	 * VDPVERSION1/2 for reset completion, but on this part both words read the
	 * constant date stamp 0x20170929, so
	 * such a poll could only time out -- and reading the register file while a
	 * VDP reset is held is the access shape that already locked the bus once
	 * here, with the wider VOU request.  mb() plus 10 us is that
	 * driver's own bracket around the reset edge.
	 */
	ret = reset_control_assert(vdp->rst_vdp);
	if (ret)
		return ret;
	mb();
	udelay(10);

	ret = reset_control_deassert(vdp->rst_vdp);
	if (ret)
		return ret;
	mb();
	udelay(10);

	return 0;
}

static void histb_vdp4_mv310_power_off(struct histb_vdp4 *vdp)
{
	/*
	 * Both reset requests stay as histb_vdp4_mv310_reset_release() left them:
	 * putting either back would leave a block held in reset while the
	 * transmitter still keeps the pixel clock running, and that is the
	 * combination that locks the bus.
	 */
	clk_bulk_disable_unprepare(HISTB_VDP4_CLK_NUM, vdp->clks);
}

static int histb_vdp4_mv310_power_on(struct histb_vdp4 *vdp)
{
	int ret;

	/*
	 * Clocks first, then the reset edge.  HiSilicon's own histb port enables the
	 * clocks before it asserts and deasserts the VDP reset, and that order is a
	 * hardware requirement rather than taste: a synchronous reset only propagates
	 * while the block is clocked, so pulsing it with the clocks down -- which is
	 * what this driver used to do -- can leave the state machines exactly as the
	 * boot loader handed them over.  The VOU request bit stays set through the
	 * pulse, as a displaying OEM system holds it (CRG54 reads 0x400d0019 there).
	 */
	ret = clk_bulk_prepare_enable(HISTB_VDP4_CLK_NUM, vdp->clks);
	if (ret)
		return ret;

	ret = histb_vdp4_mv310_reset_release(vdp);
	if (ret)
		return ret;

	return 0;
}

static int histb_vdp4_runtime_suspend(struct device *dev)
{
	struct histb_vdp4 *vdp = dev_get_drvdata(dev);

	WRITE_ONCE(vdp->powered, false);
	if (vdp->irq > 0)
		synchronize_irq(vdp->irq);

	vdp->hw->power_off(vdp);

	return 0;
}

static int histb_vdp4_runtime_resume(struct device *dev)
{
	struct histb_vdp4 *vdp = dev_get_drvdata(dev);
	int ret;

	ret = vdp->hw->power_on(vdp);
	if (ret)
		return ret;

	WRITE_ONCE(vdp->powered, true);

	return 0;
}

static int histb_vdp4_probe(struct platform_device *pdev,
			    const struct histb_vdp_variant *variant)
{
	struct device *dev = &pdev->dev;
	struct histb_vdp4 *vdp;
	int ret;

	vdp = devm_kzalloc(dev, sizeof(*vdp), GFP_KERNEL);
	if (!vdp)
		return -ENOMEM;

	vdp->dev = dev;
	vdp->hw = variant->data;
	vdp->base.dev = dev;
	vdp->base.variant = variant;
	platform_set_drvdata(pdev, &vdp->base);

	vdp->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(vdp->regs))
		return dev_err_probe(dev, PTR_ERR(vdp->regs), "ioremap failed\n");

	/*
	 * "pix" is the HD channel pixel clock that histb_vdp4_mv310_set_pixclk()
	 * programs through the display PLL and CRG54's select and divider; "bus"
	 * and "sdate" are the two CRG54 gates the channel runs through.  The VOU
	 * reset request only shifts while the block is clocked, so all three are
	 * raised before it is pulsed, and the SD channel gate (CRG54 bit 2) is
	 * left alone: an HD picture does not run through it, and the vendor's own
	 * system holds it clear.
	 */
	vdp->clks[HISTB_VDP4_CLK_PIX].id = "pix";
	vdp->clks[HISTB_VDP4_CLK_BUS].id = "bus";
	vdp->clks[HISTB_VDP4_CLK_SDATE].id = "sdate";
	ret = devm_clk_bulk_get(dev, HISTB_VDP4_CLK_NUM, vdp->clks);
	if (ret)
		return dev_err_probe(dev, ret, "clk\n");

	/*
	 * Two requests and one clock-generator field have to be in their running
	 * state before the register file is touched, and the order is a hardware
	 * constraint, not taste: histb_vdp4_mv310_reset_release() records what the
	 * board shows.  The VOU request is not a reset_control because the CRG
	 * offers no way to say that its bit means the opposite of what the reset
	 * framework's assert does, and getting that wrong on a clocked block
	 * locks the bus.
	 */
	vdp->rst_vdp = devm_reset_control_get_exclusive(dev, "vdp");
	if (IS_ERR(vdp->rst_vdp))
		return dev_err_probe(dev, PTR_ERR(vdp->rst_vdp), "vdp rst\n");

	vdp->crg = syscon_regmap_lookup_by_phandle(dev->of_node,
						   "hisilicon,crg");
	if (IS_ERR(vdp->crg))
		return dev_err_probe(dev, PTR_ERR(vdp->crg), "no crg regmap\n");

	/*
	 * Power the block up before anything reads or writes a VDP register:
	 * histb_vdp4_runtime_resume() enables the channel clock and takes the
	 * VOU through a reset of our own, so the register file starts from its
	 * documented defaults instead of from the state it was handed over in.
	 * The device then stays permanently powered for the whole probe/remove
	 * lifetime: the atomic paths and the vblank IRQ touch registers with no
	 * pm_runtime references of their own, and the shared pixel clock makes
	 * idle power-down of little use anyway. histb_vdp4_remove() drops the
	 * reference again; an aggregate unbind leaves the clocks up because the
	 * transmitter may bind again.
	 */
	pm_runtime_enable(dev);
	ret = pm_runtime_resume_and_get(dev);
	if (ret) {
		dev_err_probe(dev, ret, "pm_runtime_resume failed\n");
		goto err_pm_disable;
	}

	ret = dma_set_coherent_mask(dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err_probe(dev, ret, "dma_mask failed\n");
		goto err_pm_put;
	}

	/*
	 * Frame buffers have to be physically contiguous: this channel takes
	 * one base address (VO_G0_ADDR), and the display path is addressed
	 * physically -- the VDP does have its own MMU (the vendor's VDP_SMMU_*
	 * family, at VDP+0xe000 here rather than the header's 0xf000) but this
	 * board's boot leaves its global bypass raised, read as SCR = 0x1, and
	 * the port keeps that state.  So an imported
	 * buffer is refused unless its scatter/gather
	 * table covers the whole object. Bind the pool named by the
	 * "memory-region" property, which keeps them out of the default CMA
	 * area the decoder queues allocate from. A board whose DTB has no such
	 * property is not an error (-ENODEV): the device then keeps using the
	 * default pool.
	 */
	ret = devm_of_reserved_mem_device_init(dev);
	if (ret && ret != -ENODEV) {
		dev_err_probe(dev, ret, "memory-region init failed\n");
		goto err_pm_put;
	}

	/*
	 * 申请中断线之前清掉已锁存的状态并屏蔽全部中断源：未屏蔽的线上先到一个挂起位，内核会因“nobody cared”关掉这条中断。VOINTMSK 的位为 1 是放行（见 histb-vdp4-regs.h 里对这个名字的说明；原厂 VDP_DISP_SetIntMask 是把位或进去），所以屏蔽全部是写 0；vblank 源由 histb_vdp4_crtc_enable_vblank() 在需要时单独放行。
	 */
	vdp->irq = platform_get_irq_optional(pdev, 0);
	if (vdp->irq >= 0) {
		vo_write(vdp, VO_MSKINTSTA, ~0u);
		vo_write(vdp, VO_INTMSK, 0);

		ret = devm_request_irq(dev, vdp->irq, histb_vdp4_irq_handler,
				       IRQF_SHARED, dev_name(dev), vdp);
		if (ret) {
			dev_err_probe(dev, ret, "request_irq failed\n");
			goto err_pm_put;
		}
	}

	return 0;

err_pm_put:
	pm_runtime_put_sync(dev);
err_pm_disable:
	pm_runtime_disable(dev);
	return ret;
}

static void histb_vdp4_remove(struct platform_device *pdev)
{
	pm_runtime_put_sync_suspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
}

static int histb_vdp4_display_init(struct histb_vdp_display *base)
{
	struct histb_vdp4_display *disp =
		container_of(base, struct histb_vdp4_display, base);
	struct histb_vdp4 *vdp = (struct histb_vdp4 *)base->vdp;

	disp->vdp = vdp;

	return 0;
}

static const struct histb_vdp_ops histb_vdp4_ops = {
	.runtime_suspend	= histb_vdp4_runtime_suspend,
	.runtime_resume		= histb_vdp4_runtime_resume,
	.display_init		= histb_vdp4_display_init,
};

/* 照 MV310 寄存器布局写的硬件编程（VDPVERSION 0x20170929 那颗芯片） */
static const struct histb_vdp4_hw histb_vdp4_mv310_hw = {
	.power_on	= histb_vdp4_mv310_power_on,
	.power_off	= histb_vdp4_mv310_power_off,
	.mode_valid	= histb_vdp4_mv310_mode_valid,
	.enable		= histb_vdp4_mv310_enable,
	.g0_set		= histb_vdp4_mv310_g0_set,
	.intf_set	= histb_vdp4_mv310_intf_set,
	.fetch_max	= 0xffff,	/* the fetch geometry words are 16-bit halves */
};

const struct histb_vdp_variant histb_vdp4_mv200_variant = {
	.ops			= &histb_vdp4_ops,
	.data			= &histb_vdp4_mv310_hw,
	.probe			= histb_vdp4_probe,
	.remove			= histb_vdp4_remove,
	.display_size		= sizeof(struct histb_vdp4_display),
	.min_width		= 64,
	.min_height		= 64,
	.max_width		= 3840,
	.max_height		= 2160,
	.formats		= histb_vdp4_formats,
	.num_formats		= ARRAY_SIZE(histb_vdp4_formats),
	.plane_funcs		= &histb_vdp4_plane_funcs,
	.plane_helper		= &histb_vdp4_plane_helper_funcs,
	.crtc_funcs		= &histb_vdp4_crtc_funcs,
	.crtc_helper		= &histb_vdp4_crtc_helper_funcs,
	.mode_config_funcs	= &histb_vdp4_mode_config_funcs,
	.plane_possible_crtcs	= 0,
	.blend_mode_property	= false,
	.crtc_name		= "histb-vdp-crtc",
	.encoder_name		= "histb-vdp-encoder",
};
