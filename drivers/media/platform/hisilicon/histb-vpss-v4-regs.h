/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Offsets and fields of the vpss_v4_0 generation's control-plane block, the one
 * the Hi3798MV200H carries.
 *
 * Three shapes differ from the Hi3798MV100 file this driver was written
 * against. The constant id word moved from 0x330 to 0x340, and 0x330 there is
 * the end-of-frame counter. The interrupt mask moved from 0x1fc to 0x324. And
 * the geometry of a job is no longer a flat set of cells at the head of the
 * image: the memory accessor takes it through one descriptor per stream
 * channel, the picture being read at VPSS4_CF_CTRL and the picture this port
 * writes at VPSS4_OUT0_CTRL. The image is still handed over through the same
 * PNEXT/START pair.
 *
 * The word deltas below are the ones the vendor's own channel accessors use
 * (drv/vpss_hal_ip_rchn.c:185-391 for the read channel, drv/vpss_hal_ip_wchn.c
 * :145-296 for the write channel), and the cell addresses are those accessors'
 * anchors in drv/inc/vpss_regcp.h. The reset values that place this part
 * in the mv310 lineage, rather than in the hi3798mv200 header, are the
 * ones below.
 */
#ifndef __HISTB_VPSS_V4_REGS_H
#define __HISTB_VPSS_V4_REGS_H
#define VPSS4_CTRL		0x000
#define VPSS4_CTRL_VHD0_EN	BIT(3)		/* the one output port this driver uses */
#define VPSS4_CTRL3		0x008
#define VPSS4_CTRL3_ZME_VHD0_EN	BIT(3)
#define VPSS4_CTRL3_VHD0_CROP_EN BIT(17)
/* Over-read sizes and the four filter-table pointers of the scaler. The vendor
 * fills this word from the size the port produces (drv/vpss_tc_wrapper.c:232-233,
 * hal/vpss_cbb_hal.c:8296-8297), and the window answers zero for it even when the
 * job image carries a size, so its readback is not evidence of what the block
 * used. A job whose image leaves this cell at zero completes having written
 * only the first sample of each plane, so the image carries the produced size\.
 */
#define VPSS4_ZMEORESO		0x18
#define VPSS4_ZME_OH		GENMASK(28, 16)
#define VPSS4_ZME_OW		GENMASK(12, 0)
#define VPSS4_ZME_LHADDR	0x1c
#define VPSS4_ZME_LVADDR	0x20
#define VPSS4_ZME_CHADDR	0x24
#define VPSS4_ZME_CVADDR	0x28
/*
 * Per-stream bypass bits, one for every channel the accessor could fetch or
 * dump. The window answers 0 here after reset, the vendor's own layer has no
 * caller that writes either word, and which stream is served is decided by each
 * channel descriptor's enable bit -- so the job image leaves both at 0.
 */
#define VPSS4_RCH_BYPASS	0x010
#define VPSS4_WCH_BYPASS	0x014
/*
 * The parameter regions of the sub-blocks are not inside the words the accessor
 * fetches with the node itself: each of these cells holds the address of the
 * same-named region of this very image, the way the vendor fills them for every
 * node (hal/vpss_cbb_hal.c:8705-8708). A pass-through does not fault
 * because of these cells, but a node with a null parameter pointer is not
 * a shape the block is used to.
 */
#define VPSS4_TNR_ADDR		0x2c
#define VPSS4_SNR_ADDR		0x34
#define VPSS4_ZME_ADDR		0x3c
#define VPSS4_DEI_ADDR		0x40
#define VPSS4_HDR_ADDR		0x44
#define VPSS4_TNR_PARA		0x3800		/* VPSS_TNR_CONTRL */
#define VPSS4_SNR_PARA		0x2100		/* VPSS_DB_CTRL */
#define VPSS4_ZME_PARA		0x2000		/* VPSS_VHD0_HSP */
#define VPSS4_DEI_PARA		0x1000		/* VPSS_DIECTRL */

/*
 * The scaler's own cells, all inside the ZME parameter region. The ratio fields
 * carry the vendor's precisions: 20 fractional bits horizontally, 12
 * vertically, so 1:1 is 0x100000 and 0x1000.
 */
#define VPSS4_ZME_HSP		(VPSS4_ZME_PARA + 0x00)
#define VPSS4_ZME_HCOFFSET	(VPSS4_ZME_PARA + 0x08)
#define VPSS4_ZME_VSP		(VPSS4_ZME_PARA + 0x0c)
#define VPSS4_ZME_VSR		(VPSS4_ZME_PARA + 0x10)
#define VPSS4_ZME_H_RATIO	GENMASK(23, 0)
#define VPSS4_ZME_HFIR_ORDER	BIT(24)
#define VPSS4_ZME_H_CHFIR	BIT(25)
#define VPSS4_ZME_H_LFIR	BIT(26)
#define VPSS4_ZME_H_CHMID	BIT(28)
#define VPSS4_ZME_H_LMID	BIT(29)
#define VPSS4_ZME_H_CHMSC	BIT(30)
#define VPSS4_ZME_H_LMSC	BIT(31)
#define VPSS4_ZME_V_OUT_FMT	GENMASK(22, 21)	/* 1 = 4:2:0 semi, 0 = 4:2:2 */
#define VPSS4_ZME_V_CHFIR	BIT(23)
#define VPSS4_ZME_V_LFIR	BIT(24)
#define VPSS4_ZME_V_CHMID	BIT(28)
#define VPSS4_ZME_V_LMID	BIT(29)
#define VPSS4_ZME_V_CHMSC	BIT(30)
#define VPSS4_ZME_V_LMSC	BIT(31)
#define VPSS4_ZME_V_RATIO	GENMASK(15, 0)
#define VPSS4_ZME_HR_PREC	20
#define VPSS4_ZME_VR_PREC	12
/* The memory accessor's own request shaping, and the statistics sink. */
#define VPSS4_CHN_CFG_CTRL	0x5000
#define VPSS4_CHN_IMG_PRO_MODE	GENMASK(3, 2)
/* The read channel of the picture being input. */
#define VPSS4_CF_CTRL		0x5080
#define VPSS4_CF_FORMAT		GENMASK(3, 0)
#define VPSS4_CF_TYPE		GENMASK(7, 4)
#define VPSS4_CF_ORDER		GENMASK(11, 8)
#define VPSS4_CF_BITW		GENMASK(15, 12)
#define VPSS4_CF_DCMP_MODE	GENMASK(19, 16)
#define VPSS4_CF_UV_INVERT	BIT(20)
#define VPSS4_CF_MIRROR		BIT(21)
#define VPSS4_CF_FLIP		BIT(22)
/*
 * Which read the channel performs. The vendor's default for a whole frame is
 * progressive, whose value is 1: 0 is the interlaced read, whose field and
 * head tables this driver has no reason to fill (drv/inc/xdp_define.h:55-64,
 * drv/vpss_func_mac_rchn.c:33).
 */
#define VPSS4_CF_LM_RMODE	GENMASK(25, 23)
#define VPSS4_LM_PROGRESSIVE	1
#define VPSS4_LM_INTERLACE	0
#define VPSS4_CF_EN		BIT(31)
/*
 * b_bypass, h_bypass and d_bypass of the accessor's read descriptor
 * (drv/inc/vpss_mac_define.h:84-86). The vendor drives all three from a single
 * bMmuBypass flag of its own, which is what a picture in plain coherent memory
 * needs here.
 */
#define VPSS4_CF_TILE_BYPASS	(BIT(26) | BIT(27) | BIT(28))
#define VPSS4_CF_SIZE		0x5088
#define VPSS4_CF_SIZE_WIDTH	GENMASK(11, 0)
#define VPSS4_CF_SIZE_HEIGHT	GENMASK(27, 16)
#define VPSS4_CF_OFFSET		0x508c
#define VPSS4_CF_OFFSET_HOR	GENMASK(11, 0)
#define VPSS4_CF_OFFSET_VER	GENMASK(27, 16)
#define VPSS4_CF_YADDR_H	0x5090
#define VPSS4_CF_YADDR_L	0x5094
#define VPSS4_CF_CADDR_H	0x5098
#define VPSS4_CF_CADDR_L	0x509c
#define VPSS4_CF_STRIDE		0x50c8
#define VPSS4_CF_STRIDE_Y	GENMASK(15, 0)
#define VPSS4_CF_STRIDE_C	GENMASK(31, 16)
/* The write channel of port zero. */
#define VPSS4_OUT0_CTRL		0x5800
#define VPSS4_OUT0_FORMAT	GENMASK(3, 0)
#define VPSS4_OUT0_TYPE		GENMASK(7, 4)
#define VPSS4_OUT0_BITW		GENMASK(11, 8)
#define VPSS4_OUT0_UV_INVERT	BIT(16)
#define VPSS4_OUT0_MIRROR	BIT(17)
#define VPSS4_OUT0_FLIP		BIT(18)
#define VPSS4_OUT0_DITHER_EN	BIT(19)
#define VPSS4_OUT0_LM_RMODE	GENMASK(25, 23)
/* The vendor's frame layer sets both bypass bits of a channel from one
 * flag
 * (frame/vpss_cbb_frame.c:440-441), so a channel that talks to plain addresses
 * carries the pair; leaving one out is not a "no change" value.
 */
#define VPSS4_OUT0_H_BYPASS	BIT(28)
#define VPSS4_OUT0_D_BYPASS	BIT(29)
#define VPSS4_OUT0_EN		BIT(31)
#define VPSS4_OUT0_OFFSET	0x5804
#define VPSS4_OUT0_YADDR_H	0x5808
#define VPSS4_OUT0_YADDR_L	0x580c
#define VPSS4_OUT0_CADDR_H	0x5810
#define VPSS4_OUT0_CADDR_L	0x5814
#define VPSS4_OUT0_STRIDE	0x5818
#define VPSS4_OUT0_STRIDE_Y	GENMASK(15, 0)
#define VPSS4_OUT0_STRIDE_C	GENMASK(31, 16)
#define VPSS4_OUT0_SIZE		0x5830
#define VPSS4_OUT0_SIZE_WIDTH	GENMASK(15, 0)
#define VPSS4_OUT0_SIZE_HEIGHT	GENMASK(31, 16)
/* Formats, as the accessor's own code lists them (drv/inc/vpss_hal_ip_rchn.h:79-95). */
#define VPSS4_FMT_YUV420SP	0
#define VPSS4_FMT_YUV422SP	1
#define VPSS4_FMT_YUV420P	2
#define VPSS4_FMT_YUV422P	3
#define VPSS4_FMT_UYVY		7
#define VPSS4_TYPE_LINEAR	0
#define VPSS4_BITW_8		0
#define VPSS4_NODEID		0x2f8
#define VPSS4_PNEXT		0x2fc
#define VPSS4_START_RUN		BIT(0)
#define VPSS4_INTSTATE_EOF	BIT(0)		/* one node finished */
#define VPSS4_INTSTATE_TIMEOUT	BIT(1)
#define VPSS4_INTSTATE_BUS_W	BIT(2)
#define VPSS4_INTSTATE_EOF_END	BIT(3)		/* the whole chain finished */
#define VPSS4_INTSTATE_TUNL	BIT(4)
#define VPSS4_INTSTATE_DCMP	BIT(5)
#define VPSS4_INTSTATE_BUS_R	BIT(6)
#define VPSS4_INTSTATE_ERRORS	(VPSS4_INTSTATE_TIMEOUT | VPSS4_INTSTATE_BUS_W | \
				 VPSS4_INTSTATE_TUNL | VPSS4_INTSTATE_DCMP | \
				 VPSS4_INTSTATE_BUS_R)
#define VPSS4_INTCLR		0x308
#define VPSS4_RAWINT		0x30c
#define VPSS4_PFCNT		0x310
/*
 * The memory accessor's own request shaping. The window answers these
 * at reset with 0x314 6466, 0x318 0, 0x344 0, 0x400 777f and 0x404 501 -- and
 * 0x404's 501 is split_mode 1 with mid_enable set, which the vendor's own
 * pre-run check demands (debug/vpss_cbb_assert.c:127-129).
 */
#define VPSS4_MACCFG		0x318
#define VPSS4_BUS_CTRL		0x344
#define VPSS4_MST_OUTSTANDING	0x400
#define VPSS4_MST_CTRL		0x404
#define VPSS4_MST_RCHN_PRIO0	0x410
#define VPSS4_MST_WCHN_PRIO0	0x418
/* Latches the block leaves when one of its three masters faults. */
#define VPSS4_MST_ERR_CLR	0x440
#define VPSS4_MST_DEBUG_CTRL	0x470
#define VPSS4_MST_DEBUG_CLR	0x474
#define VPSS4_MST_AXI_DET_EN	BIT(0)
#define VPSS4_INTMASK		0x324		/* set bit = mask that source */
#define VPSS4_VERSION		0x340	/* 0 in reset, 0x4000 once released */
#define VPSS4_VERSION_RELEASED	0x4000
#define VPSS4_MISCELLANEOUS	0x314
#define VPSS4_TIMEOUT		0x31c
/*
 * Clock gating the block does for itself. The vendor asks for both as it
 * enables the clock; the read-only half of the answer is the reset
 * default at 0x400 (r 15, w 7, timer 119).
 */
#define VPSS4_LOWPOWER_CTRL	0x348
#define VPSS4_LOWPOWER_CK_GT_EN	BIT(31)
#define VPSS4_HARDGATE_CTRL	0x354
#define VPSS4_HARDGATE_PPC_EN	BIT(1)
#define VPSS4_HARDGATE_AXI_EN	BIT(0)
#define VPSS4_SMMU_SCR		0xf000		/* bit 0: the block's own translation unit */
/*
 * One job image is the whole register file, sized the way the vendor's hardware
 * layer sizes it: (sizeof(VPSS_REG_S) + 15) & ~15, whose last decoded member is
 * the translation unit's eighth master debug word at 0xf530.
 */
#define VPSS4_REG_IMAGE_SIZE	0xf540
#endif /* __HISTB_VPSS_V4_REGS_H */
