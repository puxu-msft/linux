/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Register offsets and fields of the Hi3798MV100 VPSS register file.
 *
 * The block does not take field writes: a job is a register image in memory
 * whose physical address is handed to the block through PNEXT and released by
 * START, so every offset below is used on that image (and on the MMIO window,
 * which the image mirrors).
 */
#ifndef __HISTB_VPSS_REGS_H
#define __HISTB_VPSS_REGS_H

#define VPSS_CTRL		0x000
#define VPSS_CTRL_VHD_EN	BIT(1)
#define VPSS_CTRL_UV_INVERT	BIT(5)
#define VPSS_CTRL_IMG_PRO_MODE	GENMASK(7, 6)
#define VPSS_CTRL_DEI_EN	BIT(9)
#define VPSS_CTRL_PROT		GENMASK(13, 12)
#define VPSS_CTRL_ROTATE_ANGLE	BIT(21)
#define VPSS_CTRL_ROTATE_EN	BIT(22)
#define VPSS_CTRL_PRE_VFIR_MODE	GENMASK(24, 23)
#define VPSS_CTRL_PRE_VFIR_EN	BIT(25)
#define VPSS_CTRL_VHD_LBA_EN	BIT(15)
#define VPSS_CTRL_BFIELD_FIRST	BIT(29)
#define VPSS_CTRL_BFIELD_MODE	BIT(30)
#define VPSS_CTRL_BFIELD	BIT(31)

#define VPSS_CTRL2		0x004
#define VPSS_CTRL2_VHD_FORMAT	GENMASK(7, 4)
#define VPSS_CTRL2_IN_FORMAT	GENMASK(20, 16)

#define VPSS_IMGSIZE		0x008
#define VPSS_IMGSIZE_WIDTH	GENMASK(11, 0)
#define VPSS_IMGSIZE_HEIGHT	GENMASK(27, 16)

#define VPSS_CUR_CTRL		0x00c
#define VPSS_CUR_CTRL_HOR_OFFSET	GENMASK(17, 5)
#define VPSS_CUR_CTRL_VER_OFFSET	GENMASK(30, 18)

#define VPSS_CURYADDR		0x018
#define VPSS_CURCADDR		0x01c
#define VPSS_CURCRADDR		0x020

#define VPSS_CURSTRIDE		0x024
#define VPSS_CURSTRIDE_Y	GENMASK(15, 0)
#define VPSS_CURSTRIDE_C	GENMASK(31, 16)

/* The three field nodes DEI reads beside the current one. */
#define VPSS_REF_CTRL		0x028
#define VPSS_REFYADDR		0x034
#define VPSS_REFCADDR		0x038
#define VPSS_REFCRADDR		0x03c
#define VPSS_REFSTRIDE		0x040
#define VPSS_NXT1_CTRL		0x044
#define VPSS_NXT1YADDR		0x050
#define VPSS_NXT1CADDR		0x054
#define VPSS_NXT1CRADDR		0x058
#define VPSS_NXT1STRIDE		0x05c
#define VPSS_NXT2_CTRL		0x060
#define VPSS_NXT2YADDR		0x06c
#define VPSS_NXT2CADDR		0x070
#define VPSS_NXT2CRADDR		0x074
#define VPSS_NXT2STRIDE		0x078
#define VPSS_FIELDSTRIDE_Y	GENMASK(15, 0)
#define VPSS_FIELDSTRIDE_C	GENMASK(31, 16)

/* Motion history: the block reads one plane and writes the next. */
#define VPSS_STRADDR		0x098
#define VPSS_STWADDR		0x09c
#define VPSS_STSTRIDE		0x0a0
#define VPSS_STSTRIDE_VAL	GENMASK(19, 0)
#define VPSS_DEI_ADDR		0x240

#define VPSS_VHDCTRL		0x0a4
#define VPSS_VHDCTRL_MIRROR	BIT(4)
#define VPSS_VHDCTRL_FLIP	BIT(5)
#define VPSS_VHDCTRL_UV_INVERT	BIT(8)

#define VPSS_VHDSIZE		0x0a8
#define VPSS_VHDSIZE_WIDTH	GENMASK(12, 0)
#define VPSS_VHDSIZE_HEIGHT	GENMASK(28, 16)

#define VPSS_VHDYADDR		0x0b4
#define VPSS_VHDCADDR		0x0b8

#define VPSS_VHDSTRIDE		0x0bc
#define VPSS_VHDSTRIDE_Y	GENMASK(15, 0)
#define VPSS_VHDSTRIDE_C	GENMASK(31, 16)

#define VPSS_INCROP_POS		0x114
#define VPSS_INCROP_SIZE	0x118
#define VPSS_CROP_EN		BIT(31)

#define VPSS_VHDLBA_VFPOS	0x288
#define VPSS_VHDLBA_BK		0x290
#define VPSS_VHDCROP_POS	0x124
#define VPSS_VHDCROP_SIZE	0x128
#define VPSS_VHDCROP_X		GENMASK(11, 0)
#define VPSS_VHDCROP_Y		GENMASK(27, 16)
#define VPSS_VHDCROP_WIDTH	GENMASK(11, 0)
#define VPSS_VHDCROP_HEIGHT	GENMASK(27, 16)

#define VPSS_VHD_HLOFFSET	0x164
#define VPSS_VHD_HCOFFSET	0x168
#define VPSS_VHD_HSP		0x160
#define VPSS_VHD_VSP		0x16c
#define VPSS_VHD_VSR		0x170
#define VPSS_VHD_VOFFSET	0x174
#define VPSS_VHD_ZMEORESO	0x178
#define VPSS_VHD_ZMEIRESO	0x17c
#define VPSS_VHD_ZME_LHADDR	0x210
#define VPSS_VHD_ZME_LVADDR	0x214
#define VPSS_VHD_ZME_CHADDR	0x218
#define VPSS_VHD_ZME_CVADDR	0x21c

/* The sharpen filters read their phase table 256 bytes into the scaler's. */
#define VPSS_VHD_LTI_CTRL	0x180
#define VPSS_LTI_CTRL_LHPASS_COEF4	GENMASK(7, 0)
#define VPSS_LTI_CTRL_LMIXING_RATIO	GENMASK(15, 8)
#define VPSS_LTI_CTRL_LGAIN_RATIO	GENMASK(27, 16)
#define VPSS_LTI_CTRL_LTI_EN		BIT(31)
#define VPSS_VHD_LHPASS_COEF	0x184
#define VPSS_VHD_LTI_THD	0x188
#define VPSS_LTI_THD_LOVER_SWING	GENMASK(9, 0)
#define VPSS_LTI_THD_LUNDER_SWING	GENMASK(19, 10)
#define VPSS_LTI_THD_LCORING_THD	GENMASK(31, 20)
#define VPSS_VHD_LHFREQ_THD	0x18c
#define VPSS_VHD_LGAIN_COEF	0x190
#define VPSS_VHD_CTI_CTRL	0x194
#define VPSS_CTI_CTRL_CMIXING_RATIO	GENMASK(15, 8)
#define VPSS_CTI_CTRL_CGAIN_RATIO	GENMASK(27, 16)
#define VPSS_CTI_CTRL_CTI_EN		BIT(31)
#define VPSS_VHD_CHPASS_COEF	0x198
#define VPSS_VHD_CTI_THD	0x19c
#define VPSS_CTI_THD_COVER_SWING	GENMASK(9, 0)
#define VPSS_CTI_THD_CUNDER_SWING	GENMASK(19, 10)
#define VPSS_CTI_THD_CCORING_THD	GENMASK(31, 20)
#define VPSS_MSC_H_MSSC		BIT(31)
#define VPSS_MSC_H_CSSC		BIT(30)
#define VPSS_MSC_H_LFIR		BIT(26)
#define VPSS_MSC_H_CFIR		BIT(25)
#define VPSS_MSC_H_RATIO		GENMASK(23, 0)
#define VPSS_MSC_V_MSSC		BIT(31)
#define VPSS_MSC_V_CSSC		BIT(30)
#define VPSS_MSC_V_LFIR		BIT(24)
#define VPSS_MSC_V_CFIR		BIT(23)
#define VPSS_MSC_V_RATIO		GENMASK(15, 0)
#define VPSS_ZME_IN_FMT		GENMASK(20, 19)	/* 0: 4:2:2, 1: 4:2:0 */
#define VPSS_ZME_OUT_FMT	GENMASK(22, 21)
#define VPSS_ZME_ORESO_W	GENMASK(11, 0)
#define VPSS_ZME_ORESO_H	GENMASK(23, 12)
#define VPSS_ZME_IRESO_W	GENMASK(11, 0)
#define VPSS_ZME_IRESO_H	GENMASK(23, 12)

#define VPSS_DIECTRL		0x1000
#define VPSS_DIECTRL_LMODE	GENMASK(27, 26)
#define VPSS_DIECTRL_CMODE	GENMASK(25, 24)

#define VPSS_INTMASK		0x1fc
#define VPSS_PNEXT		0x2fc
#define VPSS_START		0x300
#define VPSS_INTSTATE		0x304
#define VPSS_INTSTATE_EOF	BIT(0)		/* raw_eof */
#define VPSS_INTSTATE_TIMEOUT	BIT(1)		/* raw_timeout: the block's own limit */
#define VPSS_INTSTATE_BUS_W	BIT(2)		/* write path rejected by the bus */
#define VPSS_INTSTATE_DONE	BIT(3)		/* raw_eof_end */
#define VPSS_INTSTATE_VHD_TUNL	BIT(4)		/* output tunnel overrun */
#define VPSS_INTSTATE_STR_TUNL	BIT(6)		/* stride tunnel overrun */
#define VPSS_INTSTATE_DCMP	BIT(7)		/* tile decompress error */
#define VPSS_INTSTATE_BUS_R	BIT(10)		/* read path rejected by the bus */
/* hi_reg_vpss.h U_VPSS_INTSTATE: everything but the two end-of-frame bits */
#define VPSS_INTSTATE_ERRORS	(VPSS_INTSTATE_TIMEOUT | VPSS_INTSTATE_BUS_W | \
				 VPSS_INTSTATE_VHD_TUNL | VPSS_INTSTATE_STR_TUNL | \
				 VPSS_INTSTATE_DCMP | VPSS_INTSTATE_BUS_R)
#define VPSS_INTCLR		0x308
#define VPSS_MISCELLANEOUS	0x314
#define VPSS_TIMEOUT		0x31c
#define VPSS_VERSION		0x330		/* constant id word; 0 while the block
						 * has no clock or does not decode
						 */

/* Register images are one contiguous block; the deepest field is at 0x5058. */
#define VPSS_REG_IMAGE_SIZE	0x6000

/* One phase table set per plane and axis: 256 bytes of scaler taps, then
 * 256 bytes of sharpen taps.
 */
#define VPSS_COEF_BLOCK		512
#define VPSS_COEF_BYTES		(4 * VPSS_COEF_BLOCK)

/* Pre-scaler modes for turning one field into a frame. */
#define VPSS_PREZME_DISABLE	0
#define VPSS_PREZME_2X		1
#define VPSS_PREZME_VFIELD	4

#endif /* __HISTB_VPSS_REGS_H */
