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
 *
 * 上面关于时钟与使能顺序的说明，以及下面以 MV310_ 开头的定义，都属于驱动作者那颗芯片的 MV310 布局（VDPVERSION 0x20170929，hisilicon,hi3798mv200-vdp）。Hi3798MV300（hisilicon,hi3798mv300-vdp）是另一版布局，CRG54 bit30 在它上面是 VOU 复位（1 = 按住）、bit31 是配置时钟，与这里的说法相反；它的编程见文件后部“Hi3798MV300 布局”一节。两种布局经 struct histb_vdp4_hw 分派。
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
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
#include <drm/drm_print.h>
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
#define MV310_CRG54			0x0d8
#define  MV310_CRG54_VOU_SRST_REQ	BIT(30)

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
	/* state 为 NULL 或不可见时关 G0，否则按它的帧缓冲、源矩形与 alpha 编程；都请求在下一个帧边界锁存 */
	void (*g0_set)(struct histb_vdp4 *vdp, const struct drm_plane_state *state,
		       struct drm_gem_dma_object *gem);
	/* G0 还有没锁存的更新；NULL 表示读不出锁存状态 */
	bool (*g0_pending)(struct histb_vdp4 *vdp);
	void (*intf_set)(struct histb_vdp4 *vdp, bool enable);
	/* G0 取数宽高字段能表示的最大值 */
	unsigned int fetch_max;
	/* G0 能从帧缓冲里的任意 16 字节对齐处起取；false 时源矩形必须从帧缓冲原点起 */
	bool src_crop;
	/* 当作 vblank 的 VOINTMSK/VOMSKINTSTA 位：每帧一次、帧边界的寄存器锁存之后 */
	u32 vblank_int;
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
	/*
	 * 已提交、等 G0 锁存后才送出的翻页完成事件，占着一个 vblank 引用；event_waited 表示它已经让过一个 vblank。都由 drm_device.event_lock 保护。
	 */
	struct drm_pending_vblank_event	*event;
	bool				event_waited;
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
static void histb_vdp4_mv310_g0_set(struct histb_vdp4 *vdp,
				    const struct drm_plane_state *state,
				    struct drm_gem_dma_object *gem)
{
	const struct drm_framebuffer *fb;
	u32 fmt, ctrl, last, w, h;
	u8 alpha;

	if (!state || !state->visible) {
		ctrl = vo_read(vdp, L0_CTRL);
		ctrl &= ~L0_EN;
		vo_write(vdp, L0_CTRL, ctrl);
		vo_write(vdp, L0_REGUP, 1);
		return;
	}

	/* 这个布局从帧缓冲原点起取（src_crop 为 false，atomic_check 保证源矩形起点是原点），宽高取源矩形的 */
	fb = state->fb;
	alpha = state->alpha >> 8;
	w = drm_rect_width(&state->src) >> 16;
	h = drm_rect_height(&state->src) >> 16;

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
		 FIELD_PREP(GFX0_RESO_W, w - 1) |
		 FIELD_PREP(GFX0_RESO_H, h - 1));
	vo_write(vdp, MV310_GFX_SRC_CROP, 0);
	vo_write(vdp, GFX0_ORESO,
		 FIELD_PREP(GFX0_RESO_W, w - 1) |
		 FIELD_PREP(GFX0_RESO_H, h - 1));
	/* Surface stride in 16-byte units; the compression stride stays 0. */
	vo_write(vdp, GFX0_STRIDE, FIELD_PREP(GFX0_LSTRIDE, fb->pitches[0] / 16));

	/*
	 * Layer stage: 1:1 placement at the display origin.  G0_ALPHA is the
	 * layer's background alpha; the layer's own opacity is the galpha field
	 * of G0_CTRL (the vendor's VDP_GFX_SetLayerGalpha writes exactly that
	 * byte), so 0xff would make every commit fully opaque.
	 */
	last = FIELD_PREP(L0_POS_X, w - 1) |
	       FIELD_PREP(L0_POS_Y, h - 1);
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
		(u32)(gem->dma_addr + fb->offsets[0]), fb->pitches[0], w, h,
		&fb->format->format);
}

/* 源矩形左上角在帧缓冲里的总线地址 */
static dma_addr_t histb_vdp4_fetch_addr(const struct drm_plane_state *state,
					const struct drm_gem_dma_object *gem)
{
	const struct drm_framebuffer *fb = state->fb;

	return gem->dma_addr + fb->offsets[0] +
	       (dma_addr_t)(state->src.y1 >> 16) * fb->pitches[0] +
	       (dma_addr_t)(state->src.x1 >> 16) * fb->format->cpp[0];
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

	vdp->hw->g0_set(vdp, new_state, gem);
}

static void histb_vdp4_plane_atomic_disable(struct drm_plane *plane,
					  struct drm_atomic_commit *state)
{
	struct histb_vdp4 *vdp = plane_to_vdp(plane);

	vdp->hw->g0_set(vdp, NULL, NULL);
}

static int histb_vdp4_plane_atomic_check(struct drm_plane *plane,
				       struct drm_atomic_commit *state)
{
	struct histb_vdp4 *vdp = plane_to_vdp(plane);
	struct drm_plane_state *new_state =
		drm_atomic_get_new_plane_state(state, plane);
	struct drm_framebuffer *fb = new_state->fb;
	struct drm_crtc_state *crtc_state;
	struct drm_gem_dma_object *gem;
	dma_addr_t start;
	u32 src_w, src_h;
	int ret;

	if (!fb)
		return 0;

	/* stride unit is mv100's (16 bytes) until a working system says otherwise */
	if (fb->pitches[0] % 16)
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
	 * driver always places the surface at 0,0; with no scaling the source
	 * rectangle then has the CRTC's size.
	 */
	ret = drm_atomic_helper_check_plane_state(new_state, crtc_state,
						  DRM_PLANE_NO_SCALING,
						  DRM_PLANE_NO_SCALING,
						  false, true);
	if (ret || !new_state->visible)
		return ret;

	/* 源矩形：起点取整像素，宽高受 G0 取数字段的位宽限制（随布局而定） */
	if ((new_state->src.x1 | new_state->src.y1) & 0xffff) {
		drm_dbg_kms(plane->dev, "source origin " DRM_RECT_FP_FMT " is not whole pixels\n",
			    DRM_RECT_FP_ARG(&new_state->src));
		return -EINVAL;
	}
	src_w = drm_rect_width(&new_state->src) >> 16;
	src_h = drm_rect_height(&new_state->src) >> 16;
	if (src_w > vdp->hw->fetch_max || src_h > vdp->hw->fetch_max) {
		drm_dbg_kms(plane->dev, "source %ux%u exceeds the fetch limit %u\n",
			    src_w, src_h, vdp->hw->fetch_max);
		return -EINVAL;
	}

	if (!vdp->hw->src_crop) {
		if (new_state->src.x1 || new_state->src.y1) {
			drm_dbg_kms(plane->dev, "this layout fetches from the framebuffer origin only\n");
			return -EINVAL;
		}
		return 0;
	}

	/* 起点经取数地址表达；地址不足 16 字节的部分要靠 SFPOS，本布局没有实测过它，这样的起点不接受 */
	gem = drm_fb_dma_get_gem_obj(fb, 0);
	if (!gem)
		return -EINVAL;
	start = histb_vdp4_fetch_addr(new_state, gem);
	if (start % 16) {
		drm_dbg_kms(plane->dev, "fetch start %pad (source origin %d,%d) is not 16-byte aligned\n",
			    &start, new_state->src.x1 >> 16, new_state->src.y1 >> 16);
		return -EINVAL;
	}
	return 0;
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
 *
 * 两种布局的阈值字形状相同，这里只算出两个整字：thd1/thd2 进 DHD0_VTTHD，thd3/thd4 进 DHD0_VTTHD3。MV310 布局把它们放在 0xc228 与 0xc224（与 mv200 头文件相反），MV300 布局放在 0xc01c 与 0xc018（原厂 VDP_DISP_SetVtThd）。
 */
static void histb_vdp4_vtthd_words(struct histb_vdp4 *vdp, u32 total, u32 vfb,
				   u32 vrefresh, u32 *vtthd, u32 *vtthd3)
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

	*vtthd = FIELD_PREP(DHD_VTTHD1, thd1) | DHD_VTTHD1_MODE |
		 FIELD_PREP(DHD_VTTHD2, thd2) | DHD_VTTHD2_MODE;
	*vtthd3 = FIELD_PREP(DHD_VTTHD3, thd3) | DHD_VTTHD3_MODE;

	dev_dbg(vdp->dev, "thresholds: total %u vfb %u period %u -> thd %u/%u/%u\n",
		total, vfb, period, thd1, thd2, thd3);
}

static void histb_vdp4_mv310_set_thresholds(struct histb_vdp4 *vdp, u32 total, u32 vfb,
				      u32 vrefresh)
{
	u32 vtthd, vtthd3;

	histb_vdp4_vtthd_words(vdp, total, vfb, vrefresh, &vtthd, &vtthd3);
	vo_write(vdp, MV310_DHD0_VTTHD, vtthd);
	vo_write(vdp, MV310_DHD0_VTTHD3, vtthd3);
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

/*
 * 翻页完成事件。图层寄存器写下后要到下一个帧边界才锁存，而 vblank 源在锁存点之后（MV300 是阈值 3，约十分之一帧之后）：提交若落在锁存点与 vblank 之间，紧接着的那个 vblank 到来时新图层还没生效，事件若在这时送出，客户端会提前重用还在扫描的缓冲。所以事件不交给 drm_crtc_arm_vblank_event()，而是挂在显示对象上，由 vblank 中断按锁存状态（histb_vdp4_hw.g0_pending）决定送出还是再等一帧。MV300 的判据覆盖换缓冲与开关图层：地址一半与原厂 HiFB 帧完成回调释放 fence 的判据相同（见 MV300_G0_ADDR_LATCHED），另比较控制字；只改 alpha 或尺寸、地址不变的提交可能早一帧送出事件，它不涉及缓冲的复用。旧帧缓冲的释放也要等到这个事件（histb_vdp4_commit_tail）。
 */
static void histb_vdp4_send_event_locked(struct histb_vdp4_display *disp)
{
	struct drm_crtc *crtc = &disp->base.crtc;

	drm_crtc_send_vblank_event(crtc, disp->event);
	drm_crtc_vblank_put(crtc);
	disp->event = NULL;
}

/* 挂起本次提交的事件；拿不到 vblank 引用（CRTC 已关、没有中断）时没有要等的帧，当场送出 */
static void histb_vdp4_crtc_queue_event(struct drm_crtc *crtc,
					struct drm_crtc_state *crtc_state)
{
	struct histb_vdp4_display *disp = crtc_to_display(crtc);
	struct drm_pending_vblank_event *event = crtc_state->event;
	int ret;

	if (!event)
		return;
	crtc_state->event = NULL;	/* consumed: keep hw_done quiet */

	ret = drm_crtc_vblank_get(crtc);

	spin_lock_irq(&crtc->dev->event_lock);
	if (ret) {
		drm_crtc_send_vblank_event(crtc, event);
	} else {
		/* 提交要等上一次的 flip_done，上一个事件此时已经送出 */
		if (drm_WARN_ON_ONCE(crtc->dev, disp->event))
			histb_vdp4_send_event_locked(disp);
		disp->event = event;
		disp->event_waited = false;
	}
	spin_unlock_irq(&crtc->dev->event_lock);
}

/*
 * vblank 中断里处理挂着的事件。更新位在 flush 挂起事件之前就已写下，所以挂起之后的第二个 vblank 之前必然跨过一个帧边界：G0 已锁存就送出，没锁存就让一个 vblank，第二个 vblank 时仍报未锁存说明锁存判据与硬件不符，告警并送出。
 */
static void histb_vdp4_finish_flip(struct histb_vdp4 *vdp,
				   struct histb_vdp4_display *disp)
{
	struct drm_device *drm = disp->base.crtc.dev;
	unsigned long flags;

	spin_lock_irqsave(&drm->event_lock, flags);
	if (disp->event) {
		if (!vdp->hw->g0_pending || !vdp->hw->g0_pending(vdp)) {
			histb_vdp4_send_event_locked(disp);
		} else if (!disp->event_waited) {
			disp->event_waited = true;
		} else {
			dev_warn_ratelimited(vdp->dev,
					     "G0 update still not latched a frame after the commit\n");
			histb_vdp4_send_event_locked(disp);
		}
	}
	spin_unlock_irqrestore(&drm->event_lock, flags);
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
			vdp->hw->g0_set(vdp, pstate, gem);
	}

	/* G0 is latched; now start the interface. */
	vdp->hw->intf_set(vdp, true);

	drm_crtc_vblank_on(crtc);

	/* 打开 CRTC 的提交里 atomic_flush 把事件留到了这里（见它的说明） */
	histb_vdp4_crtc_queue_event(crtc, new_state);
}

static void histb_vdp4_crtc_atomic_disable(struct drm_crtc *crtc,
					 struct drm_atomic_commit *state)
{
	struct histb_vdp4_display *disp = crtc_to_display(crtc);
	const struct drm_crtc_state *old_state = drm_atomic_get_old_crtc_state(state, crtc);
	int vrefresh = drm_mode_vrefresh(&old_state->adjusted_mode);

	/*
	 * 关接口要到帧边界才生效，原厂关通道后等两帧（Chip_Specific_DispSetMSChnEnable 的 msleep(2 × 帧周期)）。等过之后硬件不再取图层，挂着的事件与本次提交在 flush 里当场送出的事件才不早于扫描停止，旧帧缓冲随后才释放。
	 */
	disp->vdp->hw->intf_set(disp->vdp, false);
	msleep(2 * DIV_ROUND_UP(MSEC_PER_SEC, vrefresh > 0 ? vrefresh : 24));

	spin_lock_irq(&crtc->dev->event_lock);
	if (disp->event)
		histb_vdp4_send_event_locked(disp);
	spin_unlock_irq(&crtc->dev->event_lock);

	drm_crtc_vblank_off(crtc);
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
	/* 屏蔽期间源仍会锁存，放行前先清掉，免得第一个 vblank 提前到来 */
	vo_write(vdp, VO_MSKINTSTA, vdp->hw->vblank_int);
	vo_write(vdp, VO_INTMSK, vdp->hw->vblank_int);
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
	vo_write(vdp, VO_MSKINTSTA, vdp->hw->vblank_int);
	vo_write(vdp, VO_INTMSK, 0);
}

static void histb_vdp4_crtc_atomic_flush(struct drm_crtc *crtc,
				       struct drm_atomic_commit *state)
{
	struct drm_crtc_state *new_state = drm_atomic_get_new_crtc_state(state, crtc);

	/*
	 * 要打开 CRTC 的提交里 commit_planes 先于 modeset_enables，flush 时 vblank 还关着；事件留在状态里，由 atomic_enable 在 drm_crtc_vblank_on() 之后挂起。
	 */
	if (new_state->active && drm_atomic_crtc_needs_modeset(new_state))
		return;

	histb_vdp4_crtc_queue_event(crtc, new_state);
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

	if (sta & ~vdp->hw->vblank_int)
		dev_warn_ratelimited(vdp->dev, "vdp irq: unexpected status %08x\n",
				     sta);

	/* the line stays requested across an aggregate unbind, no CRTC then */
	if (disp && (sta & vdp->hw->vblank_int)) {
		drm_crtc_handle_vblank(&disp->crtc);
		histb_vdp4_finish_flip(vdp, container_of(disp, struct histb_vdp4_display, base));
	}
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

/*
 * drm_atomic_helper_commit_tail() 只等 vblank 计数变一次就清理旧的平面状态、放掉旧帧缓冲；本驱动的翻页完成事件要等 G0 锁存，可能比那个 vblank 晚一帧（见 histb_vdp4_send_event_locked 前的说明），所以这里改等 flip_done，其余与它相同：旧帧缓冲在硬件不再取它之后才放掉。
 */
static void histb_vdp4_commit_tail(struct drm_atomic_commit *state)
{
	struct drm_device *dev = state->dev;

	drm_atomic_helper_commit_modeset_disables(dev, state);
	drm_atomic_helper_commit_planes(dev, state, 0);
	drm_atomic_helper_commit_modeset_enables(dev, state);
	drm_atomic_helper_fake_vblank(state);
	drm_atomic_helper_commit_hw_done(state);
	drm_atomic_helper_wait_for_flip_done(dev, state);
	drm_atomic_helper_cleanup_planes(dev, state);
}

static const struct drm_mode_config_helper_funcs histb_vdp4_mode_config_helpers = {
	.atomic_commit_tail	= histb_vdp4_commit_tail,
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
 * back: the VOU request raised (see MV310_CRG54_VOU_SRST_REQ for the measurement) and
 * the block's own request pulsed.  The caller enables the clocks first, so this
 * runs with the HD domain clocked; the channel control readback at the end is the
 * first live access of the register file and it has to answer.
 */
static int histb_vdp4_mv310_reset_release(struct histb_vdp4 *vdp)
{
	int ret;

	ret = regmap_update_bits(vdp->crg, MV310_CRG54, MV310_CRG54_VOU_SRST_REQ,
				 MV310_CRG54_VOU_SRST_REQ);
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
	 * MV300 布局不经这个时钟设频率，由 histb_vdp4_mv300_set_pixclk() 直接编程 CRG，三个时钟只用来开门控。
	 *
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
	/* core 只装 mode_config.funcs；提交尾部的顺序是本驱动的（histb_vdp4_commit_tail） */
	base->drm->mode_config.helper_private = &histb_vdp4_mode_config_helpers;

	return 0;
}

/* ------------------------------------------------------------------ */
/* Hi3798MV300 布局                                                    */
/* ------------------------------------------------------------------ */

/*
 * Hi3798MV300 的 VDP（VDPVERSION1/2 读 0x09349963/0x20150807）是厂商 SDK hal/98mv200 那一版布局，不是上面的 MV310 布局：DHD0 在 0xc000、通道步长 0x400，图形层 G0 在 0x7000、层间步长 0x200，GP0 在 0x8000，图形混合器 MIXG0 在 0xb208、DHD0 的混合器 MIX0 在 0xb408，HDMI 的源通道选择是 VO_MUX（0x100）bit3。偏移与位域读自本机原厂 3.18 内核（boot.img）的 VDP 函数反汇编：VDP_DISP_OpenIntf、VDP_DISP_SetVtThd、VDP_DISP_SetIntfEnable、VDP_DISP_SetRegUp、VDP_DISP_SetHdmiMode、VDP_DISP_SetSyncInv、VDP_DISP_SetIntfMuxSel、VDP_DHD_DEFAULT、VDP_GFX_*、VDP_GP_*、VDP_CBM_SetMixerPrio（原厂函数对照见设备文档 docs/14-VDP-HDMI.md，反汇编摘录在设备目录 captures/20261122-hdmi/re/）。图层、GP0 与混合器内部的相对布局与 histb-vdp2 驱动的 Hi3798MV100 相同、只是基址不同，所以编程顺序照 histb-vdp2 在板上验证过的写法。
 *
 * 管线：G0 → MIXG0 槽 0 → GP0（ORDER_NULL：不经缩放与 CSC）→ MIX0 槽 1 → DHD0 → HDMI，全程 RGB。
 */
/*
 * VOCTRL 的双通道位（VDP_SetTwoChnMode 写 bit25、VDP_SetTwoChnEnable 写 bit24）。原厂 3.18 的 VDP_DRIVER_Initial（@0xc07ba944，由 PF_ResetVdpHardware 调用）对任何格式都先置 bit25 再置 bit24（@0xc07baa9c、@0xc07baaa4；4.4 内核的 VDP_DRIVER_Initial 同样）。两位都清零时图层这条路的每个像素在 DHD0 输出上横向重复一次，屏上是帧缓冲左半被放大一倍铺满全屏，DHD0 自带彩条不受影响（07 #129）；在线把 VOCTRL 从 0x8080000b 改成 0x8380000b 后测试图恢复正常（07 #130）。字段的硬件含义没有数据手册。
 */
#define   MV300_VO_CTRL_TWO_CHN_EN	BIT(24)
#define   MV300_VO_CTRL_TWO_CHN_MODE	BIT(25)
#define MV300_VDPVERSION1		0x0020
#define MV300_VDPVERSION2		0x0024
#define MV300_VOLOWPOWER		0x002c
/* VDP_SetVDPMemLowPower 写的整字，原厂在每次设置通道时钟时写 */
#define   MV300_VOLOWPOWER_STOCK	0x5b2b

#define MV300_GFX_STEP			0x200
/* VDP_GFX_SetLayerEnable 接受的层号 0..5 */
#define MV300_GFX_NUM			6
#define MV300_G0_CTRL			0x7000
#define   MV300_G0_IFMT			GENMASK(7, 0)
#define   MV300_G0_BITEXT		GENMASK(9, 8)
#define   MV300_G0_READ_MODE		BIT(26)	/* 1 = 逐行取数 */
#define   MV300_G0_UPD_MODE		BIT(27)	/* 1 = 逐行更新 */
#define   MV300_G0_NO_SEC		BIT(30)	/* 非安全访问；复位默认值即为 1 */
#define   MV300_G0_EN			BIT(31)
/*
 * 图层寄存器更新：bit0 写 1 请求在下一帧边界锁存。字里其余位由硬件维护（读到 0x1020，置更新位一帧后读到 0x102c：07 #128、#129，captures/20261122-hdmi/F-g0-latch.txt），原厂 HiFB 的 OPTM_VDP_GFX_SetRegUp（@0xc07bd28c）读改写置 bit0，这里照做。
 */
#define MV300_G0_UPD			0x7004
#define   MV300_REGUP			BIT(0)
#define MV300_G0_ADDR			0x7010
#define MV300_G0_STRIDE			0x701c	/* [15:0]，16 字节为单位 */
#define MV300_G0_IRESO			0x7020
#define MV300_G0_SFPOS			0x7024
#define MV300_G0_SMMU_BYPASS		0x7028	/* bit0 2D、bit1 3D 右眼、bit2/3 压缩头 */
#define   MV300_G0_SMMU_BYPASS_ALL	GENMASK(3, 0)
#define MV300_G0_CBMPARA		0x7030
#define   MV300_G0_GALPHA		GENMASK(7, 0)
#define MV300_G0_DFPOS			0x7080
#define MV300_G0_DLPOS			0x7084
#define MV300_G0_VFPOS			0x7088
#define MV300_G0_VLPOS			0x708c
/*
 * 锁存后的工作副本：+0xe0 是控制字，只在置更新位之后的帧边界跟着 0x7000 变（07 #129，F-g0-latch.txt）；+0xe4 是正在取数的地址，原厂 HiFB 的 HIFB_HAL_GetWorkedLayerAddr（@0xc07bbae0）读它、HIFB_HAL_GetWillWorkLayerAddr（@0xc07bbb6c）读 0x7010，帧完成回调 DRV_HIFB_ADP_FrameEndCallBack（@0xc07bf744）在阈值 3 的中断里比较两者（@0xc07bf810），相等才释放 fence、唤醒 vblank 等待。板上读数里 +0xe4 与 0x7010 相同（captures/20261122-hdmi/G-layer-dump.txt）。
 */
#define MV300_G0_CTRL_LATCHED		0x70e0
#define MV300_G0_ADDR_LATCHED		0x70e4
/* 宽高与位置字段：x/宽 [11:0]，y/高 [23:12]，都填“值 - 1”或坐标 */
#define   MV300_RESO_W			GENMASK(11, 0)
#define   MV300_RESO_H			GENMASK(23, 12)
/* 原厂 HiFB 格式表（OPTM_PixerFmtTransferToHalFmt，字节表 0xc0daa5f4）里 KRGB888 的码；平面只声明 XRGB8888 */
#define MV300_G0_IFMT_KRGB888		0x60
#define MV300_G0_BITEXT_3RD		3

#define MV300_GP0_CTRL			0x8000
/*
 * VDP_GP_SetIpOrder 的缺省分支（VDP_GP_ORDER_NULL）：mux1 [1:0] = 3、mux2 [5:4] = 2、aout [9:8] = 0、bout [13:12] = 1，缩放与 CSC 都不在通路上。
 */
#define   MV300_GP0_ORDER_NULL		0x00001023
#define   MV300_GP0_READ_MODE		BIT(31)
/* GP0 的寄存器更新：写 1，同原厂 VDP_GP_SetRegUp（@0xc075146c）；板上读数为 0（G-layer-dump.txt 里 GP0 +0x004 不在非零字之列） */
#define MV300_GP0_UPD			0x8004
#define MV300_GP0_ORESO			0x8008
#define MV300_GP0_IRESO			0x800c
#define MV300_GP0_GALPHA		0x8020
#define MV300_GP0_DFPOS			0x8100
#define MV300_GP0_DLPOS			0x8104
#define MV300_GP0_VFPOS			0x8108
#define MV300_GP0_VLPOS			0x810c
#define MV300_GP0_CSC_IDC		0x8120	/* csc_en 在 bit22 */
/*
 * GP0 缩放（VDP_GP_SetZmeEnable/HorRatio/VerRatio）：HSP 0x8140 的 [23:0] 是水平步长（1.0 = 1 << 20）、bit31 hsc_en；VSP 0x814c bit31 vsc_en；VSR 0x8150 [15:0] 是垂直步长（1.0 = 1 << 12，复位值即 0x1000）。原厂 PQ 的 PQ_MNG_SetGfxZme（@0xc06e1f64）按 (输入宽 << 20) / 输出宽 算水平步长并写进 0x8140（PQ_HAL_SetGZmeHorRatio，@0xc06e21d0 处调用），1:1 时即 1.0；这里缩放关着，也照样写 1.0，不留 0。
 */
#define MV300_GP0_ZME_HSP		0x8140
#define   MV300_GP0_ZME_HRATIO_1	0x00100000
#define MV300_GP0_ZME_VSP		0x814c

/*
 * 混合器按原厂 VDP_CBM_SetMixerPrio（@0xc074f058）的混合器号命名：MIXG0（2 号）在 0xb208，每个优先级槽写“层号 + 1”；DHD0 的混合器 MIX0（4 号）在 0xb408，槽里直接写 VDP_PRIO_E（VP0 = 1、GP0 = 2）。0xb008 是视频混合器 MIXV0（0 号），不是 DHD0 的。
 */
#define MV300_MIXG0_MIX			0xb208
#define MV300_MIX0_BKG			0xb400	/* MIX0 背景：Cr[9:0] Cb[19:10] Y[29:20] */
#define MV300_MIX0_MIX			0xb408
#define   MV300_MIX_PRIO0		GENMASK(3, 0)
#define   MV300_MIX_PRIO1		GENMASK(7, 4)
#define   MV300_MIXG0_LAYER_G0		1
#define   MV300_MIX0_LAYER_VP0		1
#define   MV300_MIX0_LAYER_GP0		2

#define MV300_DHD0_CTRL			0xc000
#define   MV300_DHD_REGUP		BIT(0)
#define   MV300_DHD_IOP			BIT(4)	/* 1 = 逐行 */
/* VDP_DISP_SetHdmiMode；原厂 PF_DispIntfHdmi0Reset 对 VESA（RGB 输出）格式置 1、对 CEA 格式清 0 */
#define   MV300_DHD_HDMI_RGB		BIT(14)
#define   MV300_DHD_INTFB		BIT(15)
#define   MV300_DHD_INTF_EN		BIT(31)
/* VDP_DHD_DEFAULT 写的空闲通道字：regup + iop + bit15，接口关 */
#define   MV300_DHD_CTRL_DEFAULT	(MV300_DHD_REGUP | MV300_DHD_IOP | MV300_DHD_INTFB)
#define MV300_DHD0_VSYNC		0xc004
#define   MV300_DHD_VACT		GENMASK(11, 0)
#define   MV300_DHD_VBB			GENMASK(21, 12)
#define   MV300_DHD_VFB			GENMASK(31, 22)
#define MV300_DHD0_HSYNC1		0xc008	/* hact [15:0]、hbb [31:16]，同 DHD_HSYNC1_* */
#define MV300_DHD0_HSYNC2		0xc00c	/* hfb [15:0]、hmid [31:16]，同 DHD_HSYNC2_* */
#define MV300_DHD0_VPLUS		0xc010	/* 下场的 vact/vbb/vfb，逐行模式为 0 */
#define MV300_DHD0_PWR			0xc014	/* hpw [15:0]、vpw [23:16]，同 DHD_PWR_* */
#define MV300_DHD0_VTTHD3		0xc018	/* thd3/thd4，字段同 DHD_VTTHD3 */
#define MV300_DHD0_VTTHD		0xc01c	/* thd1/thd2，字段同 DHD_VTTHD1/2 */
#define MV300_DHD0_SYNC_INV		0xc020
/* VDP_DHD_DEFAULT 写的 0x2000；HDMI 的极性位在 [11:8]（VDP_DISP_SetSyncInv 的 HDMI 分支），位次同 histb-vdp2 */
#define   MV300_DHD_SYNC_INV_DEFAULT	0x00002000
#define   MV300_DHD_SYNC_HDMI_HS_INV	BIT(9)
#define   MV300_DHD_SYNC_HDMI_VS_INV	BIT(10)
#define MV300_DHD0_DATA_SEL		0xc02c	/* VDP_DHD_DEFAULT 写 0，原厂不再改 */
#define MV300_DHD0_CSC_IDC		0xc040
#define   MV300_DHD_CSC_EN		BIT(22)
/* HDMI 接口的输出钳位：下限、上限各三个 10 位分量 [29:20][19:10][9:0]（VDP_DISP_SetClipCoef 的 HDMI 分支） */
#define MV300_DHD0_HDMI_CLIP_L		0xc090
#define MV300_DHD0_HDMI_CLIP_H		0xc094
#define   MV300_CLIP_FULL		GENMASK(29, 0)
#define MV300_DHD_STEP			0x400	/* DHD1 = DHD0 + 0x400 */
/*
 * 视频层 V0：VDP_VID_SetLayerEnable（@0xc07688ac）读改写 0x800 的 bit31，随后往 0xf00 写 1（VDP_VID_SetRegionEnable_sdk 也写这个字，含义没有数据手册）；寄存器更新是 VDP_VID_SetRegUp（@0xc076adf0）往 0x804 写 1。
 */
#define MV300_V0_CTRL			0x0800
#define   MV300_V0_EN			BIT(31)
#define MV300_V0_UPD			0x0804
#define MV300_V0_REGION_EN		0x0f00
/* 第一个回写块：VDP_WBC_SetEnable（@0xc0774d58）读改写 0x9000 的 bit31，VDP_WBC_SetRegUp（@0xc07756a8）往 0x9004 写 1 */
#define MV300_WBC0_CTRL			0x9000
#define   MV300_WBC_EN			BIT(31)
#define MV300_WBC0_UPD			0x9004

/* CRG 里属于显示的字段，位定义按原厂 HDMI 驱动的 REG_HDMI_CRG_* 访问器（MV300 布局） */
#define MV300_CRG_PLL10			0x028
#define   MV300_PLL10_CFG		GENMASK(30, 0)
#define MV300_CRG_PLL11			0x02c
#define   MV300_PLL11_CFG		GENMASK(27, 0)
#define MV300_CRG54			0x0d8
#define   MV300_CRG54_VO_BUS_CKEN	BIT(0)
#define   MV300_CRG54_VO_SD_CKEN	BIT(2)
#define   MV300_CRG54_VO_SDATE_CKEN	BIT(3)
#define   MV300_CRG54_VDAC_CH0_CKEN	BIT(6)
#define   MV300_CRG54_VO_SD_CLK_SEL	BIT(12)
#define   MV300_CRG54_VO_SD_CLK_DIV	GENMASK(15, 14)
#define   MV300_CRG54_VO_HD_CLK_SEL	BIT(16)
#define   MV300_CRG54_VO_HD_CLK_DIV	GENMASK(19, 18)
#define   MV300_CRG54_HDMI_CLK_SEL	BIT(25)
#define   MV300_CRG54_VO_SD_HDMI_CLK_SEL	BIT(26)
#define   MV300_CRG54_VDP_CLK_SEL	BIT(27)
#define   MV300_CRG54_VO_HD_HDMI_CLK_SEL	BIT(29)
#define   MV300_CRG54_VOU_SRST_REQ	BIT(30)	/* 1 = 复位按住 */
#define   MV300_CRG54_VDP_CFG_CKEN	BIT(31)	/* VDP 配置时钟门控 */
#define MV300_CRG211			0x34c
#define   MV300_CRG211_GATES		0x00007fcf
#define   MV300_CRG211_SD_PPC_DIV	BIT(17)
#define   MV300_CRG211_HD_PPC_DIV	BIT(20)
#define   MV300_CRG211_SD_PPC_SEL	BIT(21)
#define   MV300_CRG211_HD_PPC_SEL	BIT(22)
#define   MV300_CRG211_WORK_CLK		(MV300_CRG211_SD_PPC_DIV | MV300_CRG211_HD_PPC_DIV | \
					 MV300_CRG211_SD_PPC_SEL | MV300_CRG211_HD_PPC_SEL)
#define   MV300_CRG211_VDP_DIV		GENMASK(28, 24)
#define   MV300_CRG211_VDP_SRST_REQ	BIT(29)	/* 即 DT 的 "vdp" 复位 */

/*
 * VDP_OpenClkAndInitCrgConfig 第二步对 CRG54 的写法：置 bit0、2、3、16、25、31，清 bit6、12、26、27、29，[15:14] = 2，[19:18] = 0；bit30 不动。
 */
#define MV300_CRG54_OPEN_MASK		(MV300_CRG54_VO_BUS_CKEN | MV300_CRG54_VO_SD_CKEN | \
					 MV300_CRG54_VO_SDATE_CKEN | MV300_CRG54_VDAC_CH0_CKEN | \
					 MV300_CRG54_VO_SD_CLK_SEL | MV300_CRG54_VO_SD_CLK_DIV | \
					 MV300_CRG54_VO_HD_CLK_SEL | MV300_CRG54_VO_HD_CLK_DIV | \
					 MV300_CRG54_HDMI_CLK_SEL | MV300_CRG54_VO_SD_HDMI_CLK_SEL | \
					 MV300_CRG54_VDP_CLK_SEL | MV300_CRG54_VO_HD_HDMI_CLK_SEL | \
					 MV300_CRG54_VDP_CFG_CKEN)
#define MV300_CRG54_OPEN_VAL		(MV300_CRG54_VO_BUS_CKEN | MV300_CRG54_VO_SD_CKEN | \
					 MV300_CRG54_VO_SDATE_CKEN | \
					 FIELD_PREP(MV300_CRG54_VO_SD_CLK_DIV, 2) | \
					 MV300_CRG54_VO_HD_CLK_SEL | MV300_CRG54_HDMI_CLK_SEL | \
					 MV300_CRG54_VDP_CFG_CKEN)

/*
 * 像素钟。原厂 Chip_Specific_DispSetChanClk 对 HD 通道按格式表（0xc1376f98 起、步长 0xd4）把显示 PLL 编成 PLL10/PLL11，再按格式号把 CRG211[28:24] 设成 1、3、7 或 0，CRG54[19:18] 设 0（4K 等两像素每拍的格式设 3，这里不支持）。表中每个格式都满足：像素钟 = PLL 输出 ÷ 4 ÷ (CRG211[28:24] + 1)，其中 PLL 输出 = 24 MHz × (PLL11[11:0] + PLL10[23:0] / 2^24) ÷ PLL11[17:12] ÷ PLL10[26:24] ÷ PLL10[30:28]。
 *
 * 这里照搬原厂表里每像素一拍、逐行格式的取值，按像素钟去重（同一像素钟有两种 PLL 写法时取 60 Hz 格式的那种），不另解 PLL：主线 clk 驱动的 clk_vo_hd 把像素钟建模成 PLL ÷ 4 ÷ CRG54[19:18] 的分频，与 MV300 不符，所以本布局不调 clk_set_rate()，由这里经 hisilicon,crg 直接编程；clk_vo_hd 仍由本驱动与 HDMI 驱动 prepare/enable，它的 prepare 只碰 CRG54 bit4、16、29，与原厂一致的部分不冲突。代价是 clk_summary 里 clk_vo_hd 的频率在 MV300 上不准（1080p60 碰巧对，720p 与 1024x768 不对）。
 */
struct histb_vdp4_mv300_pixclk {
	u32	khz;	/* 原厂格式的标称像素钟 */
	u32	pll10;	/* PERI_CRG_PLL10[30:0] */
	u32	pll11;	/* PERI_CRG_PLL11[27:0] */
	u32	div;	/* PERI_CRG211[28:24] */
};

static const struct histb_vdp4_mv300_pixclk histb_vdp4_mv300_pixclks[] = {
	{  25200, 0x1233336c, 0x00001043, 7 },	/* 格式 40，640x480@60 */
	{  27000, 0x12000000, 0x00001048, 7 },	/* 格式 9，576p50 */
	{  27027, 0x12126e97, 0x00001048, 7 },	/* 格式 10，480p60 */
	{  40000, 0x11000000, 0x000030a0, 7 },	/* 格式 41，800x600@60 */
	{  65000, 0x11000000, 0x00003082, 3 },	/* 格式 42，1024x768@60 */
	{  74176, 0x12e6a7f0, 0x00001062, 3 },	/* 格式 73，720p59.94 */
	{  74250, 0x11800000, 0x00001031, 3 },	/* 格式 7，720p60 */
	{  83500, 0x11000000, 0x000030a7, 3 },	/* 格式 44，1280x800@60 */
	{  85500, 0x11000000, 0x000030ab, 3 },	/* 格式 46，1360x768@60 */
	{  88750, 0x11557300, 0x00002076, 3 },	/* 格式 50，1440x900@60 RB */
	{  97750, 0x11557300, 0x00002082, 3 },	/* 格式 51，1600x900@60 RB */
	{ 106500, 0x1181b320, 0x0000306a, 1 },	/* 格式 49，1440x900@60 */
	{ 108000, 0x12000000, 0x00001048, 1 },	/* 格式 45，1280x1024@60 */
	{ 119000, 0x11557300, 0x0000204f, 1 },	/* 格式 54，1680x1050@60 RB */
	{ 121750, 0x11c042c0, 0x00003079, 1 },	/* 格式 48，1400x1050@60 */
	{ 146250, 0x11400e70, 0x00003092, 1 },	/* 格式 53，1680x1050@60 */
	{ 148352, 0x12e6a7f0, 0x00001062, 1 },	/* 格式 74，1080p59.94 */
	{ 148500, 0x11800000, 0x00001031, 1 },	/* 格式 0，1080p60 */
	{ 156750, 0x11800000, 0x00002068, 1 },	/* 格式 58，2048x1152@60 RB */
	{ 162000, 0x11000000, 0x00001036, 1 },	/* 格式 52，1600x1200@60 */
	{ 193250, 0x114002b8, 0x000030c1, 1 },	/* 格式 56，1920x1200@60 */
};

/* 与 MV310 布局的 mode_valid 同一容差：0.5 %，覆盖 EDID 里 25.175 MHz 一类按整 kHz 取整的模式 */
static const struct histb_vdp4_mv300_pixclk *histb_vdp4_mv300_pixclk(int khz)
{
	const struct histb_vdp4_mv300_pixclk *best = NULL;
	unsigned int i, diff, best_diff = UINT_MAX;

	for (i = 0; i < ARRAY_SIZE(histb_vdp4_mv300_pixclks); i++) {
		diff = abs((int)histb_vdp4_mv300_pixclks[i].khz - khz);
		if (diff < best_diff) {
			best = &histb_vdp4_mv300_pixclks[i];
			best_diff = diff;
		}
	}

	if (!best || khz <= 0 || best_diff * 200 > (unsigned int)khz)
		return NULL;

	return best;
}

/* 图形层 G0–G5 的寄存器更新：读改写置 bit0，保留硬件维护的其余位（见 MV300_G0_UPD） */
static void histb_vdp4_mv300_gfx_regup(struct histb_vdp4 *vdp, unsigned int layer)
{
	u32 off = MV300_G0_UPD + layer * MV300_GFX_STEP;

	vo_write(vdp, off, vo_read(vdp, off) | MV300_REGUP);
}

/* G0 还有更新没在帧边界生效：影子寄存器与锁存副本（见 MV300_G0_CTRL_LATCHED）不同 */
static bool histb_vdp4_mv300_g0_pending(struct histb_vdp4 *vdp)
{
	return vo_read(vdp, MV300_G0_CTRL) != vo_read(vdp, MV300_G0_CTRL_LATCHED) ||
	       vo_read(vdp, MV300_G0_ADDR) != vo_read(vdp, MV300_G0_ADDR_LATCHED);
}

/*
 * 原厂每改一次 CRG 的复位或门控都跟一个 dsb sy 和 10 µs（__const_udelay 的参数 0xa3d702 = 10 × UDELAY_MULT，3.18 的 UDELAY_MULT = (2199023 × HZ) >> 11，原厂内核 CONFIG_HZ=1000，即 udelay(10)）：先让这次写到达 CRG，再给复位或时钟沿留出生效时间，之后才动下一步。
 */
static void histb_vdp4_mv300_crg_settle(void)
{
	mb();
	udelay(10);
}

static u32 histb_vdp4_mv300_version(struct histb_vdp4 *vdp)
{
	return vo_read(vdp, MV300_VDPVERSION1) | vo_read(vdp, MV300_VDPVERSION2);
}

/*
 * 寄存器文件此刻可访问：配置时钟开、VOU 复位放开、VDP 块复位放开、CRG211 门控全开。读 CRG 失败按不可访问处理。
 */
static bool histb_vdp4_mv300_reachable(struct histb_vdp4 *vdp)
{
	u32 crg54, crg211;

	if (regmap_read(vdp->crg, MV300_CRG54, &crg54) ||
	    regmap_read(vdp->crg, MV300_CRG211, &crg211))
		return false;

	return (crg54 & MV300_CRG54_VDP_CFG_CKEN) &&
	       !(crg54 & MV300_CRG54_VOU_SRST_REQ) &&
	       !(crg211 & MV300_CRG211_VDP_SRST_REQ) &&
	       (crg211 & MV300_CRG211_GATES) == MV300_CRG211_GATES;
}

/*
 * 复位之前让 VDP 停止读写内存：关两个通道的接口、各图形层、视频层 V0 与第一个回写块，再等在途的帧做完，否则复位会切断正在进行的 AXI 传输（顺序同 histb-vdp2 的 histb_vdp2_hw_reset()）。关接口与图层的写只在帧边界生效，原厂关通道后等两帧（Chip_Specific_DispSetMSChnEnable 的 msleep(2 × 帧周期)）；这里等 100 ms，覆盖 24 Hz 的两帧。上电时只在发现有东西在跑时才等；断电时总要等，断电的调用方（runtime suspend、remove、上电出错）不保证接口已在帧边界停下。调用方保证寄存器文件可访问。
 */
static void histb_vdp4_mv300_quiesce(struct histb_vdp4 *vdp, bool always_wait)
{
	static const u32 dhd[] = { MV300_DHD0_CTRL, MV300_DHD0_CTRL + MV300_DHD_STEP };
	bool live = false;
	unsigned int i;
	u32 ctrl;

	for (i = 0; i < ARRAY_SIZE(dhd); i++) {
		ctrl = vo_read(vdp, dhd[i]);
		if (!(ctrl & MV300_DHD_INTF_EN))
			continue;
		vo_write(vdp, dhd[i], (ctrl & ~MV300_DHD_INTF_EN) | MV300_DHD_REGUP);
		live = true;
	}

	for (i = 0; i < MV300_GFX_NUM; i++) {
		u32 off = MV300_G0_CTRL + i * MV300_GFX_STEP;

		ctrl = vo_read(vdp, off);
		if (!(ctrl & MV300_G0_EN))
			continue;
		vo_write(vdp, off, ctrl & ~MV300_G0_EN);
		histb_vdp4_mv300_gfx_regup(vdp, i);
		live = true;
	}

	ctrl = vo_read(vdp, MV300_V0_CTRL);
	if (ctrl & MV300_V0_EN) {
		vo_write(vdp, MV300_V0_CTRL, ctrl & ~MV300_V0_EN);
		vo_write(vdp, MV300_V0_REGION_EN, 1);
		vo_write(vdp, MV300_V0_UPD, MV300_REGUP);
		live = true;
	}

	ctrl = vo_read(vdp, MV300_WBC0_CTRL);
	if (ctrl & MV300_WBC_EN) {
		vo_write(vdp, MV300_WBC0_CTRL, ctrl & ~MV300_WBC_EN);
		vo_write(vdp, MV300_WBC0_UPD, MV300_REGUP);
		live = true;
	}

	if (live || always_wait)
		msleep(100);
	if (live)
		dev_info(vdp->dev, "stopped VDP channels and layers that were still running\n");
}

/*
 * 原厂 VDP_CloseClkResetModule 的第一步：配置时钟开着的时候置 CRG54 bit30，再以 VDP_CheckResetFinished 的判据（VDPVERSION1/2 都读 0）等复位生效，最多 41 次 2 ms。轮询只在原厂关时钟时的那个状态下做（配置时钟开、VDP 块复位 CRG211 bit29 放开、CRG211 门控全开，07 #125 在板上读过）；其余状态只置位、不碰寄存器文件。超时照原厂只告警、继续往下走。
 */
static void histb_vdp4_mv300_hold_reset(struct histb_vdp4 *vdp)
{
	u32 crg54, crg211 = 0, ver;
	int ret;

	ret = regmap_read(vdp->crg, MV300_CRG54, &crg54);
	if (!ret)
		ret = regmap_read(vdp->crg, MV300_CRG211, &crg211);
	if (ret)
		crg54 = 0;

	regmap_update_bits(vdp->crg, MV300_CRG54, MV300_CRG54_VOU_SRST_REQ,
			   MV300_CRG54_VOU_SRST_REQ);
	histb_vdp4_mv300_crg_settle();

	if (!(crg54 & MV300_CRG54_VDP_CFG_CKEN) ||
	    (crg211 & MV300_CRG211_VDP_SRST_REQ) ||
	    (crg211 & MV300_CRG211_GATES) != MV300_CRG211_GATES)
		return;

	ret = read_poll_timeout(histb_vdp4_mv300_version, ver, !ver, 2000,
				41 * 2000, false, vdp);
	if (ret)
		dev_warn(vdp->dev, "VDP reset did not take: VDPVERSION %08x\n", ver);
}

/*
 * 先停下通道与图层，再走 VDP_CloseClkResetModule：复位按住之后才关配置时钟（bit31），最后按住 VDP 块复位（CRG211 bit29）。CRG54 bit0、3 与 CRG211 的门控由 clk 驱动管（clk_vo_bus、clk_vo_sdate 是关不掉的 critical 时钟），这里只动 clk 驱动不建模或本驱动独占的位。
 */
static void histb_vdp4_mv300_shutdown(struct histb_vdp4 *vdp)
{
	if (histb_vdp4_mv300_reachable(vdp))
		histb_vdp4_mv300_quiesce(vdp, true);

	histb_vdp4_mv300_hold_reset(vdp);

	regmap_update_bits(vdp->crg, MV300_CRG54,
			   MV300_CRG54_VDP_CFG_CKEN | MV300_CRG54_VO_SD_CKEN |
			   MV300_CRG54_VDAC_CH0_CKEN, 0);
	histb_vdp4_mv300_crg_settle();

	reset_control_assert(vdp->rst_vdp);
}

/*
 * 上电。顺序是硬约束：任何时候都不在 CRG54 bit31（配置时钟）为 0 时清 bit30（VOU 复位）或访问寄存器文件。
 *
 * 1. 引导程序留着 VDP 在跑（bit31 = 1、bit30 = 0、CRG211 bit29 = 0、CRG211 门控全开）时先停下它；
 * 2. 置 bit30 并等复位生效（原厂关时钟的第一步）；复位按住期间脉冲 VDP 块复位 CRG211 bit29，对应原厂关时钟的最后一步与开时钟的第一步，让寄存器文件回到默认值；
 * 3. 原厂 VDP_OpenClkAndInitCrgConfig：清 CRG211 bit29、等 10 µs；CRG54 打开门控与配置时钟、各选择位归位（bit30 不动）；CRG211 门控与工作时钟选择全开、等 10 µs；最后清 bit30、等 10 µs。
 *
 * 原厂在 (CRG54 & 0x40000001) == 1（引导程序在出图）时跳过整段；这里总是复位，好让驱动从默认值开始，而不是从 logo 管线留下的状态开始。
 */
static int histb_vdp4_mv300_power_on(struct histb_vdp4 *vdp)
{
	u32 crg54, crg211, ver;
	int ret;

	/* clk_vo_hd 的 prepare 置 CRG54 bit16、4，清 bit29；bus、sdate 是 bit0、3 */
	ret = clk_bulk_prepare_enable(HISTB_VDP4_CLK_NUM, vdp->clks);
	if (ret)
		return ret;

	ret = regmap_read(vdp->crg, MV300_CRG54, &crg54);
	if (!ret)
		ret = regmap_read(vdp->crg, MV300_CRG211, &crg211);
	if (ret)
		goto err_clk;

	dev_dbg(vdp->dev, "power on from CRG54 %08x CRG211 %08x\n", crg54, crg211);

	if (histb_vdp4_mv300_reachable(vdp))
		histb_vdp4_mv300_quiesce(vdp, false);

	histb_vdp4_mv300_hold_reset(vdp);

	ret = reset_control_assert(vdp->rst_vdp);
	if (ret)
		goto err_reset;
	histb_vdp4_mv300_crg_settle();
	ret = reset_control_deassert(vdp->rst_vdp);
	if (ret)
		goto err_reset;
	histb_vdp4_mv300_crg_settle();

	ret = regmap_update_bits(vdp->crg, MV300_CRG54, MV300_CRG54_OPEN_MASK,
				 MV300_CRG54_OPEN_VAL);
	if (ret)
		goto err_reset;
	ret = regmap_update_bits(vdp->crg, MV300_CRG211,
				 MV300_CRG211_GATES | MV300_CRG211_WORK_CLK,
				 MV300_CRG211_GATES | MV300_CRG211_WORK_CLK);
	if (ret)
		goto err_reset;
	histb_vdp4_mv300_crg_settle();

	/*
	 * 清 bit30 之前回读确认配置时钟真的开了：bit31 从 ARM 侧写入在本板上没单独验证过（07 #125 时它是 fastboot 留下的 1），而 CRG54 同时被 clk 驱动无锁地读改写。没开就让复位按着，不去碰寄存器文件。
	 */
	ret = regmap_read(vdp->crg, MV300_CRG54, &crg54);
	if (!ret)
		ret = regmap_read(vdp->crg, MV300_CRG211, &crg211);
	if (ret)
		goto err_reset;
	if (!(crg54 & MV300_CRG54_VDP_CFG_CKEN) ||
	    (crg211 & MV300_CRG211_VDP_SRST_REQ) ||
	    (crg211 & MV300_CRG211_GATES) != MV300_CRG211_GATES) {
		dev_err(vdp->dev, "VDP clocks did not come up: CRG54 %08x CRG211 %08x\n",
			crg54, crg211);
		ret = -EIO;
		goto err_reset;
	}

	ret = regmap_update_bits(vdp->crg, MV300_CRG54, MV300_CRG54_VOU_SRST_REQ, 0);
	if (ret)
		goto err_reset;
	histb_vdp4_mv300_crg_settle();

	/* 原厂与 4.4 内核都不读硬件版本字做分支；这里只要求复位放开之后它在 1 ms 内变成非零 */
	ret = read_poll_timeout(histb_vdp4_mv300_version, ver, ver, 10, 1000,
				false, vdp);
	if (ret) {
		dev_err(vdp->dev, "VDP register file does not answer after reset release\n");
		goto err_reset;
	}

	dev_info(vdp->dev, "VDP version %08x %08x\n",
		 vo_read(vdp, MV300_VDPVERSION1), vo_read(vdp, MV300_VDPVERSION2));

	return 0;

err_reset:
	histb_vdp4_mv300_shutdown(vdp);
err_clk:
	clk_bulk_disable_unprepare(HISTB_VDP4_CLK_NUM, vdp->clks);
	return ret;
}

static void histb_vdp4_mv300_power_off(struct histb_vdp4 *vdp)
{
	histb_vdp4_mv300_shutdown(vdp);
	clk_bulk_disable_unprepare(HISTB_VDP4_CLK_NUM, vdp->clks);
}

static enum drm_mode_status
histb_vdp4_mv300_mode_valid(struct histb_vdp4 *vdp,
			    const struct drm_display_mode *mode)
{
	u32 hbb = mode->htotal - mode->hsync_start;
	u32 hfb = mode->hsync_start - mode->hdisplay;
	u32 hpw = mode->hsync_end - mode->hsync_start;
	u32 vbb = mode->vtotal - mode->vsync_start;
	u32 vfb = mode->vsync_start - mode->vdisplay;
	u32 vpw = mode->vsync_end - mode->vsync_start;

	/* 原厂把隔行与像素重复的格式交给另一套时钟与场计数，这里只做逐行 */
	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		return MODE_NO_INTERLACE;
	if (mode->flags & DRM_MODE_FLAG_DBLSCAN)
		return MODE_NO_DBLESCAN;
	if (mode->flags & DRM_MODE_FLAG_DBLCLK)
		return MODE_BAD;

	/* G0/GP0 的宽高与位置字段、DHD0 的 vact 都是 12 位，填“值 - 1” */
	if (mode->hdisplay > 4096 || mode->vdisplay > 4096)
		return MODE_BAD;

	/* vbb、vfb 各 10 位，vpw 8 位，水平各字段 16 位，都填“值 - 1”；阈值字段 13 位 */
	if (!vbb || vbb > 1024 || !vfb || vfb > 1024 || !vpw || vpw > 256 ||
	    mode->vtotal > 8191)
		return MODE_V_ILLEGAL;
	if (!hbb || hbb > 65536 || !hfb || hfb > 65536 || !hpw || hpw > 65536)
		return MODE_H_ILLEGAL;

	if (!histb_vdp4_mv300_pixclk(mode->clock))
		return MODE_NOCLOCK;

	return MODE_OK;
}

/*
 * Chip_Specific_DispSetChanClk 的 HD 通道分支（每像素一拍的逐行格式）：SelectChannelPllSource 选 PLL10/PLL11（CRG54 bit16 = 1），Chip_Specific_DispSetPll 写 PLL10[30:0] 与 PLL11[27:0]，CRG211[28:24] 按格式写分频，VDP_DISP_SelectChanClkDiv 写 CRG54[19:18] = 0，非标清格式清 CRG54 bit29，VDP_SelectClk(0) 清 bit27；随后 Chip_Specific_DispSetClkLowPower 对所有格式最终都落在 HD 像素处理全速（bit22 = 0、bit20 = 1）、SD 半速（bit21 = 1、bit17 = 0）。原厂不等 PLL 锁定。
 */
static void histb_vdp4_mv300_set_pixclk(struct histb_vdp4 *vdp,
					const struct drm_display_mode *mode)
{
	const struct histb_vdp4_mv300_pixclk *pc = histb_vdp4_mv300_pixclk(mode->clock);

	if (!pc) {
		dev_err(vdp->dev, "no pixel clock setting for %d kHz\n", mode->clock);
		return;
	}

	regmap_update_bits(vdp->crg, MV300_CRG_PLL10, MV300_PLL10_CFG, pc->pll10);
	regmap_update_bits(vdp->crg, MV300_CRG_PLL11, MV300_PLL11_CFG, pc->pll11);
	regmap_update_bits(vdp->crg, MV300_CRG211,
			   MV300_CRG211_VDP_DIV | MV300_CRG211_WORK_CLK,
			   FIELD_PREP(MV300_CRG211_VDP_DIV, pc->div) |
			   MV300_CRG211_HD_PPC_DIV | MV300_CRG211_SD_PPC_SEL);
	regmap_update_bits(vdp->crg, MV300_CRG54,
			   MV300_CRG54_VO_HD_CLK_SEL | MV300_CRG54_VO_HD_CLK_DIV |
			   MV300_CRG54_VDP_CLK_SEL | MV300_CRG54_VO_HD_HDMI_CLK_SEL,
			   MV300_CRG54_VO_HD_CLK_SEL);

	dev_dbg(vdp->dev, "pixel clock %u kHz for mode clock %d kHz: PLL10 %08x PLL11 %08x CRG211 div %u\n",
		pc->khz, mode->clock, pc->pll10, pc->pll11, pc->div);
}

/*
 * 一次性的管线设置：VDP 时钟门控、原厂 VDP_DRIVER_Initial 的双通道（先 bit25 再 bit24，见 MV300_VO_CTRL_TWO_CHN_*）与原厂的存储低功耗字（Chip_Specific_DispSetClkLowPower 里的 VDP_SetClkGateEn(1)、VDP_SetVDPMemLowPower），MIXG0 槽 0 收 G0，MIX0 槽 0 留给 VP0、槽 1 收 GP0（与 histb-vdp2 相同的 0x21），GP0 走 ORDER_NULL、整体不透明、CSC 与缩放关闭。混合器背景在 RGB 链路上写 0 即黑。
 */
static void histb_vdp4_mv300_pipeline_init(struct histb_vdp4 *vdp)
{
	vo_write(vdp, VO_CTRL, vo_read(vdp, VO_CTRL) | VO_CTRL_CK_GT_EN);
	vo_write(vdp, VO_CTRL, vo_read(vdp, VO_CTRL) | MV300_VO_CTRL_TWO_CHN_MODE);
	vo_write(vdp, VO_CTRL, vo_read(vdp, VO_CTRL) | MV300_VO_CTRL_TWO_CHN_EN);
	vo_write(vdp, MV300_VOLOWPOWER, MV300_VOLOWPOWER_STOCK);

	vo_write(vdp, MV300_MIXG0_MIX,
		 FIELD_PREP(MV300_MIX_PRIO0, MV300_MIXG0_LAYER_G0));
	vo_write(vdp, MV300_MIX0_MIX,
		 FIELD_PREP(MV300_MIX_PRIO0, MV300_MIX0_LAYER_VP0) |
		 FIELD_PREP(MV300_MIX_PRIO1, MV300_MIX0_LAYER_GP0));
	vo_write(vdp, MV300_MIX0_BKG, 0);

	vo_write(vdp, MV300_GP0_CTRL, MV300_GP0_ORDER_NULL | MV300_GP0_READ_MODE);
	vo_write(vdp, MV300_GP0_GALPHA, 0xff);
	vo_write(vdp, MV300_GP0_CSC_IDC, 0);
	vo_write(vdp, MV300_GP0_ZME_HSP, MV300_GP0_ZME_HRATIO_1);
	vo_write(vdp, MV300_GP0_ZME_VSP, 0);
	vo_write(vdp, MV300_GP0_UPD, MV300_REGUP);
}

/*
 * 通道时序，按原厂 VDP_DHD_DEFAULT + VDP_DISP_OpenIntf + PF_SetChnBaseTiming 的阈值：三个方向都是 ACT + BB + FB，同步脉冲在 BB 里，各字段填“值 - 1”（原厂格式表的 1080p60 是 vbb 41、hbb 192，即同步加后沿）；逐行模式下场的 VPLUS 与 hmid 为 0。
 *
 * 三个阈值照原厂 PF_SetChnBaseTiming（@0xc0735d64）：阈值 1 半帧、阈值 2 是 DISP_GFX_GetIsrPoint 的图形更新点（帧尾前 2 × 总行 ÷ 周期 − vfb 行，周期 = 100000 ÷ 刷新率（0.01 Hz 单位）取整，60 Hz 为 16，约 2 ms）、阈值 3 十分之一帧；行计数从有效区结束（前沿开始）算起，模式位都是场模式（PF_ResetChn 对非帧封装 3D 传 1）。
 *
 * vblank 源用阈值 3（VOINTMSK bit2，struct histb_vdp4_hw 的 vblank_int）。原厂 DISP_ISR_Main（@0xc0790e70）按表把 HD 通道的中断类型 1/2/3 分派到 0x1/0x2/0x4，3.18 的 HiFB 把图层寄存器更新（DRV_HIFB_ADP_UpDataCallBack）挂在类型 2、把帧完成（DRV_HIFB_ADP_FrameEndCallBack，fence 在这里释放）挂在类型 3（OPTM_GPClose 注销的正是这两项：OPTM_SetCallbackToDisp 的类型参数 2 与 3）：翻页完成的语义是类型 3。三个阈值在板上每帧各触发一次（07 #128：1080p60 下 2 s 各 121 次），以阈值 3 为源翻页稳定在 60 Hz（07 #129）。阈值 3 在有效区结束后约十分之一帧（1080p60 下是新帧有效区第 67 行前后），已过帧边界的锁存点，与原厂 fence 释放的时刻相同；提交落在锁存点与阈值 3 之间时，翻页完成事件顺延到下一帧（histb_vdp4_finish_flip）。
 */
static void histb_vdp4_mv300_setup_timing(struct histb_vdp4 *vdp,
					  const struct drm_display_mode *mode)
{
	u32 hbb = mode->htotal - mode->hsync_start;
	u32 hfb = mode->hsync_start - mode->hdisplay;
	u32 hpw = mode->hsync_end - mode->hsync_start;
	u32 vbb = mode->vtotal - mode->vsync_start;
	u32 vfb = mode->vsync_start - mode->vdisplay;
	u32 vpw = mode->vsync_end - mode->vsync_start;
	u32 inv = MV300_DHD_SYNC_INV_DEFAULT;
	u32 vtthd, vtthd3;

	if (mode->flags & DRM_MODE_FLAG_NHSYNC)
		inv |= MV300_DHD_SYNC_HDMI_HS_INV;
	if (mode->flags & DRM_MODE_FLAG_NVSYNC)
		inv |= MV300_DHD_SYNC_HDMI_VS_INV;
	vo_write(vdp, MV300_DHD0_SYNC_INV, inv);
	vo_write(vdp, MV300_DHD0_DATA_SEL, 0);

	/* 原厂 PF_ResetChn：通道 CSC 关（VDP_DISP_SetCscEnable(chn, 0)），HDMI 钳位设成 0..1023 即不钳 */
	vo_write(vdp, MV300_DHD0_CSC_IDC,
		 vo_read(vdp, MV300_DHD0_CSC_IDC) & ~MV300_DHD_CSC_EN);
	vo_write(vdp, MV300_DHD0_HDMI_CLIP_L, 0);
	vo_write(vdp, MV300_DHD0_HDMI_CLIP_H, MV300_CLIP_FULL);
	vo_write(vdp, MV300_DHD0_CTRL, MV300_DHD_CTRL_DEFAULT | MV300_DHD_HDMI_RGB);

	vo_write(vdp, MV300_DHD0_VSYNC,
		 FIELD_PREP(MV300_DHD_VACT, histb_vdp4_m1(mode->vdisplay)) |
		 FIELD_PREP(MV300_DHD_VBB, histb_vdp4_m1(vbb)) |
		 FIELD_PREP(MV300_DHD_VFB, histb_vdp4_m1(vfb)));
	vo_write(vdp, MV300_DHD0_HSYNC1,
		 FIELD_PREP(DHD_HSYNC1_HACT, histb_vdp4_m1(mode->hdisplay)) |
		 FIELD_PREP(DHD_HSYNC1_HBB, histb_vdp4_m1(hbb)));
	vo_write(vdp, MV300_DHD0_HSYNC2,
		 FIELD_PREP(DHD_HSYNC2_HFB, histb_vdp4_m1(hfb)));
	vo_write(vdp, MV300_DHD0_VPLUS, 0);
	vo_write(vdp, MV300_DHD0_PWR,
		 FIELD_PREP(DHD_PWR_HPW, histb_vdp4_m1(hpw)) |
		 FIELD_PREP(DHD_PWR_VPW, histb_vdp4_m1(vpw)));

	histb_vdp4_vtthd_words(vdp, mode->vtotal, vfb, drm_mode_vrefresh(mode),
			       &vtthd, &vtthd3);
	vo_write(vdp, MV300_DHD0_VTTHD, vtthd);
	vo_write(vdp, MV300_DHD0_VTTHD3, vtthd3);

	/* VDP_DISP_SetIntfMuxSel(0, HDMI)：hdmi_sel 写通道号 0 */
	vo_write(vdp, VO_MUX, vo_read(vdp, VO_MUX) & ~VO_MUX_HDMI_SEL);

	/* 带 regup 再写一次，锁存时序；接口仍关着 */
	vo_write(vdp, MV300_DHD0_CTRL, MV300_DHD_CTRL_DEFAULT | MV300_DHD_HDMI_RGB);
}

static void histb_vdp4_mv300_enable(struct histb_vdp4 *vdp,
				    const struct drm_display_mode *mode)
{
	histb_vdp4_mv300_set_pixclk(vdp, mode);
	histb_vdp4_mv300_pipeline_init(vdp);
	histb_vdp4_mv300_setup_timing(vdp, mode);
}

/* VDP_DISP_SetIntfEnable + VDP_DISP_SetRegUp；整字写，regup 在默认字里 */
static void histb_vdp4_mv300_intf_set(struct histb_vdp4 *vdp, bool enable)
{
	vo_write(vdp, MV300_DHD0_CTRL,
		 MV300_DHD_CTRL_DEFAULT | MV300_DHD_HDMI_RGB |
		 (enable ? MV300_DHD_INTF_EN : 0));
}

/*
 * G0 取数与 GP0 的 1:1 几何，顺序照 histb_vdp2_g0_set()：先写图层格式与取数参数、GP0 的输入输出尺寸与位置，GP0 锁存之后再打开图层并锁存。取数从源矩形的左上角起（地址 32 位，DMA 掩码 32 位；atomic_check 保证它 16 字节对齐），宽高取源矩形的，平面铺满 CRTC、不缩放，所以显示窗口与 GP0 的输入输出都是同一尺寸；行距以 16 字节为单位。图层自己的 SMMU 全部旁路（VDP_GFX_SetSmmu*Bypass，0x7028 的 bit0–3），不动 0xf000 的全局 SMMU。
 */
static void histb_vdp4_mv300_g0_set(struct histb_vdp4 *vdp,
				    const struct drm_plane_state *state,
				    struct drm_gem_dma_object *gem)
{
	u32 ctrl, reso, addr;

	if (!state || !state->visible) {
		ctrl = vo_read(vdp, MV300_G0_CTRL);
		vo_write(vdp, MV300_G0_CTRL, ctrl & ~MV300_G0_EN);
		histb_vdp4_mv300_gfx_regup(vdp, 0);
		return;
	}

	addr = histb_vdp4_fetch_addr(state, gem);
	reso = FIELD_PREP(MV300_RESO_W, (drm_rect_width(&state->src) >> 16) - 1) |
	       FIELD_PREP(MV300_RESO_H, (drm_rect_height(&state->src) >> 16) - 1);

	ctrl = FIELD_PREP(MV300_G0_IFMT, MV300_G0_IFMT_KRGB888) |
	       FIELD_PREP(MV300_G0_BITEXT, MV300_G0_BITEXT_3RD) |
	       MV300_G0_READ_MODE | MV300_G0_UPD_MODE | MV300_G0_NO_SEC;
	vo_write(vdp, MV300_G0_CTRL, ctrl);
	vo_write(vdp, MV300_G0_SMMU_BYPASS, MV300_G0_SMMU_BYPASS_ALL);
	vo_write(vdp, MV300_G0_CBMPARA,
		 FIELD_PREP(MV300_G0_GALPHA, state->alpha >> 8));

	vo_write(vdp, MV300_G0_SFPOS, 0);
	vo_write(vdp, MV300_G0_ADDR, addr);
	vo_write(vdp, MV300_G0_STRIDE, state->fb->pitches[0] / 16);
	vo_write(vdp, MV300_G0_IRESO, reso);
	vo_write(vdp, MV300_G0_VFPOS, 0);
	vo_write(vdp, MV300_G0_VLPOS, reso);
	vo_write(vdp, MV300_G0_DFPOS, 0);
	vo_write(vdp, MV300_G0_DLPOS, reso);

	vo_write(vdp, MV300_GP0_IRESO, reso);
	vo_write(vdp, MV300_GP0_ORESO, reso);
	vo_write(vdp, MV300_GP0_DFPOS, 0);
	vo_write(vdp, MV300_GP0_DLPOS, reso);
	vo_write(vdp, MV300_GP0_VFPOS, 0);
	vo_write(vdp, MV300_GP0_VLPOS, reso);
	vo_write(vdp, MV300_GP0_UPD, MV300_REGUP);

	vo_write(vdp, MV300_G0_CTRL, ctrl | MV300_G0_EN);
	histb_vdp4_mv300_gfx_regup(vdp, 0);

	dev_dbg(vdp->dev, "g0 fetch: addr 0x%08x pitch %u src " DRM_RECT_FP_FMT "\n",
		addr, state->fb->pitches[0], DRM_RECT_FP_ARG(&state->src));
}

static const struct histb_vdp4_hw histb_vdp4_mv300_hw = {
	.power_on	= histb_vdp4_mv300_power_on,
	.power_off	= histb_vdp4_mv300_power_off,
	.mode_valid	= histb_vdp4_mv300_mode_valid,
	.enable		= histb_vdp4_mv300_enable,
	.g0_set		= histb_vdp4_mv300_g0_set,
	.g0_pending	= histb_vdp4_mv300_g0_pending,
	.intf_set	= histb_vdp4_mv300_intf_set,
	.fetch_max	= 4096,
	.src_crop	= true,
	.vblank_int	= HD_CHN ? VO_INT_DHD1_VTTHD3 : VO_INT_DHD0_VTTHD3,
};

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
	.vblank_int	= HD_INT_VBLANK,
};

/* 两种布局只差硬件编程表，DRM 侧的描述相同 */
#define HISTB_VDP4_VARIANT(_hw) {					\
	.ops			= &histb_vdp4_ops,			\
	.data			= (_hw),				\
	.probe			= histb_vdp4_probe,			\
	.remove			= histb_vdp4_remove,			\
	.display_size		= sizeof(struct histb_vdp4_display),	\
	.min_width		= 64,					\
	.min_height		= 64,					\
	.max_width		= 3840,					\
	.max_height		= 2160,					\
	.formats		= histb_vdp4_formats,			\
	.num_formats		= ARRAY_SIZE(histb_vdp4_formats),	\
	.plane_funcs		= &histb_vdp4_plane_funcs,		\
	.plane_helper		= &histb_vdp4_plane_helper_funcs,	\
	.crtc_funcs		= &histb_vdp4_crtc_funcs,		\
	.crtc_helper		= &histb_vdp4_crtc_helper_funcs,	\
	.mode_config_funcs	= &histb_vdp4_mode_config_funcs,	\
	.plane_possible_crtcs	= 0,					\
	.blend_mode_property	= false,				\
	.crtc_name		= "histb-vdp-crtc",			\
	.encoder_name		= "histb-vdp-encoder",			\
}

const struct histb_vdp_variant histb_vdp4_mv200_variant =
	HISTB_VDP4_VARIANT(&histb_vdp4_mv310_hw);

const struct histb_vdp_variant histb_vdp4_mv300_variant =
	HISTB_VDP4_VARIANT(&histb_vdp4_mv300_hw);
