/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Registers of the HiSilicon HiSTB video decoder, V5R6 generation
 * (the core the vendor calls HiVDHV5R6C02 on Hi3798MV200H / Hi3798MV310).
 * Derived from the vendor firmware's register description header for this
 * family (vdm_hal_v5r6c1_local.h), cross-checked against that firmware's own
 * ARM32 assembly and against the live board.
 */
#ifndef HISTB_VDEC_V5_REGS_H
#define HISTB_VDEC_V5_REGS_H

#include <linux/bits.h>

/*
 * One 64 KiB window holds the decode core, the stream distributor 0xc000
 * bytes in and the bitplane unpacker 0xd000 bytes in. The clock enable, reset
 * request and status of every sub-block live inside this window rather than in
 * the SoC's clock and reset generator, which offers only a clock selector for
 * the core.
 */
#define V5R6_START			0x000
#define V5R6_REPAIR			0x004
#define V5R6_CFG0			0x008
#define V5R6_CFG1			0x00c
/*
 * The three addresses the core reads first are the picture message, the report
 * area it fills in, and the bitstream, each as a 16-byte granule index.
 */
#define V5R6_AVM_ADDR			0x010
#define V5R6_VAM_ADDR			0x014
#define V5R6_STREAM_BASE		0x018
#define V5R6_YSTADDR_1D			0x060
#define V5R6_YSTRIDE_1D			0x064
#define V5R6_UVOFFSET_1D		0x068
#define V5R6_HEAD_INF_OFFSET	0x06c
#define V5R6_PPFD_ADDR			0x080
#define V5R6_PPFD_LEN			0x084
#define V5R6_STATE			0x01c
#define V5R6_INT_STATE			0x020
#define V5R6_INT_MASK			0x024
#define V5R6_VCTRL_STATE		0x028
/* Which reference-memory arrangement the core was built with; read-only. */
#define V5R6_EMAR_ID			0x030
#define V5R6_REF_PIC_TYPE		0x094
#define V5R6_DOWN_CLK_CFG		0x09c

/*
 * What the syntax-error stage recorded for the last launch: the first and the
 * last macroblock it accepted, its own state word, and the number of report
 * entries it queued. These sit past the 64 KiB mark, so the window has to reach
 * 0x1100 to read them.
 */
#define V5R6_RPT_CNT			0x034
/* End-of-picture mark of the report word the core fills in. */
#define V5R6_REPORT_ENDOFPIC		BIT(17)
#define V5R6_SED_STA			0x1000
#define V5R6_SED_FSTMB			0x1008
#define V5R6_SED_LSTMB			0x100c
#define V5R6_SED_STATE1			0x1014

/*
 * Per-picture performance counters: the cycles the core spent on the picture,
 * the bus bandwidth and request counts for reads and writes, and the qp it used
 * for macroblock 0. The request counts are what say whether the core fetched the
 * bitstream and wrote a surface at all.
 */
#define V5R6_DEC_CYCLES			0x0b0
#define V5R6_RD_BW_PER_PIC		0x0b4
#define V5R6_WR_BW_PER_PIC		0x0b8
#define V5R6_RD_REQ_PER_PIC		0x0bc
#define V5R6_WR_REQ_PER_PIC		0x0c0
#define V5R6_MB0_QP			0x0d0

/* The reconstruction post-process block, which starts on its own. */
#define V5R6_RPR_START			0x140

/* The doorbell is a pulse: 0, then 1, then back to 0. */
#define V5R6_START_GO			BIT(0)

/* CFG0: what the engine checks and how it works. */
#define V5R6_CFG0_MBAMT			GENMASK(19, 0)
#define V5R6_CFG0_MEM_CG		BIT(20)	/* memory clock gating */
#define V5R6_CFG0_MOD_CG		BIT(21)	/* module clock gating */
#define V5R6_CFG0_MARKER_DETECT		BIT(22)
#define V5R6_CFG0_ACLAST_DETECT		BIT(23)
#define V5R6_CFG0_COEFIDX_DETECT	BIT(24)
#define V5R6_CFG0_VOPTYPE_DETECT	BIT(25)
#define V5R6_CFG0_WORK_MODE		GENMASK(27, 26)
#define V5R6_CFG0_LUMA_SUM		BIT(28)
#define V5R6_CFG0_LUMA_HISTOGRAM	BIT(29)
#define V5R6_CFG0_LD_QMATRIX		BIT(30)
#define V5R6_CFG0_SEC_MODE		BIT(31)	/* HEVC: safe flag for the core */

/*
 * CFG1: the protocol plus the output geometry knobs. There is no stride field
 * here, unlike the older generation: the pitch is a 32-bit register of its own
 * at V5R6_YSTRIDE_1D.
 */
#define V5R6_CFG1_VID_STD		GENMASK(3, 0)
#define V5R6_CFG1_VPSS_INFO		BIT(11)
#define V5R6_CFG1_MFD_MMU		BIT(12)
#define V5R6_CFG1_UV_ORDER		BIT(13)
#define V5R6_CFG1_FST_SLC_GRP		BIT(14)
#define V5R6_CFG1_MV_OUT		BIT(15)
#define V5R6_CFG1_MAX_SLCGRP_NUM	GENMASK(27, 16)
#define V5R6_CFG1_LINE_NUM_OUT		BIT(28)
#define V5R6_CFG1_2D			BIT(29)	/* HEVC: vdh_2d_en */
#define V5R6_CFG1_COMPRESS		BIT(30)	/* HEVC: frame compression */
#define V5R6_CFG1_PPFD			BIT(31)

/*
 * State of the core after a launch. The header of the sibling revision calls
 * bit 19 a version field, while the firmware's assembly polls the very same
 * bit as the partial-decode completion; a board that has decoded pictures
 * reports 0 there, so which one it is stays open until a launch with a known
 * macroblock count says otherwise. Bits 16:0 count slices or macroblocks and
 * were seen at 94621 on an idle block, so they are not a status.
 */
#define V5R6_STATE_SLICES		GENMASK(16, 0)
#define V5R6_STATE_DEC_OVER		BIT(17)
#define V5R6_STATE_DEC_ERR		BIT(18)
#define V5R6_STATE_PART_DEC_OVER	BIT(19)

/*
 * Latched events. A set bit in the mask word disables the interrupt, so the
 * driver masks everything it does not ask for. Bit 2 is the partial-decode
 * completion, named by the firmware's assembly rather than by the header.
 */
#define V5R6_INT_DEC_OVER		BIT(0)
#define V5R6_INT_DEC_ERR		BIT(1)
#define V5R6_INT_PART_DEC_OVER		BIT(2)

/*
 * Watchdogs of the pipeline stages, all seven of which the older firmware of
 * this family programs to the same value at launch. The word ahead of the run
 * of seven is not one of them.
 */
#define V5R6_TIMEOUT_STAGES	7
#define V5R6_VCTRL_TO		0x038
#define V5R6_SED_TO			0x03c
#define V5R6_ITRANS_TO			0x040
#define V5R6_PMV_TO			0x044
#define V5R6_PRC_TO			0x048
#define V5R6_RCN_TO			0x04c
#define V5R6_DBLK_TO			0x050
#define V5R6_PPFD_TO			0x054
#define V5R6_PART_DEC_LEVEL		0x05c
#define V5R6_TIMEOUT_DEFAULT		0x00300c03

/* Written at every launch by the firmware this revision descends from. */
#define V5R6_DOWN_CLK_VALUE		0xaaaaaaaa

/* The sub-block clock enables, two bits each. */
#define V5R6_CLKSEL			0xf804
#define V5R6_CLKSEL_FIELD(which)	GENMASK((which) * 2 + 1, (which) * 2)
#define V5R6_CLKSEL_VDEC		0
#define V5R6_CLKSEL_DSP			1
#define V5R6_CLKSEL_MMU			2
#define V5R6_CLKSEL_BPD			3
#define V5R6_CLKSEL_SCD			4
#define V5R6_CLKSEL_MFDE		5
#define V5R6_CLKSEL_RUN			2

/*
 * Reset requests and their acknowledgement, plus the idle flags. Reading the
 * request word as zero with the status word's top four bits set is the quiet
 * state: out of reset, every sub-block idle. The decode core is the unit the
 * vendor calls MFDE; asking for the whole block would drop the stream
 * distributor and the bitplane engine along with it.
 */
#define V5R6_RESET			0xf80c
#define V5R6_RESET_ALL			BIT(0)
#define V5R6_RESET_CORE			BIT(1)
#define V5R6_RESET_SCD			BIT(2)
#define V5R6_RESET_BPD			BIT(3)

#define V5R6_RST_STATE			0xf810
#define V5R6_RST_STATE_ALL		BIT(0)
#define V5R6_RST_STATE_CORE		BIT(1)
#define V5R6_RST_STATE_SCD		BIT(2)
#define V5R6_RST_STATE_BPD		BIT(3)
#define V5R6_RST_STATE_IDLE		BIT(4)
#define V5R6_RST_STATE_BPD_IDLE		BIT(5)
#define V5R6_RST_STATE_SCD_IDLE		BIT(6)
#define V5R6_RST_STATE_MFDE_IDLE	BIT(7)

/*
 * The stream distributor's own registers, inside the same window. The vendor
 * build writes all four at every launch; the clock gate value 2 is this part's
 * and 3 is the older one's.
 */
#define V5R6_SCD_AVS_FLAG		0xc000
#define V5R6_SCD_EMAR_CFG		0xc004
#define V5R6_SCD_EMAR_ENABLE		BIT(0)
#define V5R6_SCD_VDH_SELRST		0xc008
#define V5R6_SCD_CLOCK_GATE		0xc00c
#define V5R6_SCD_GATE_VALUE		2

/*
 * The core's own SMMU. Zero in the control register keeps translation off and
 * lets the message and reference addresses be programmed as physical ones; the
 * firmware turns translation on by writing 0x00030008 there. The fault latches
 * share the core's interrupt line, so they are cleared with it.
 */
#define V5R6_SMMU_SCR			0xf000
#define V5R6_SMMU_SCR_ENABLE		0x00030008
#define V5R6_SMMU_INTSTAT_S		0xf018
#define V5R6_SMMU_INTCLR_S		0xf01c
#define V5R6_SMMU_INTMASK		0xf020
#define V5R6_SMMU_INTSTAT_NS		0xf028
#define V5R6_SMMU_INTCLR_NS		0xf02c
#define V5R6_SMMU_FAULT_BITS		GENMASK(2, 0)
#define V5R6_SMMU_CB_TTBR		0xf20c
#define V5R6_SMMU_ERR_RDADDR		0xf304
#define V5R6_SMMU_ERR_WRADDR		0xf308

#endif /* HISTB_VDEC_V5_REGS_H */
