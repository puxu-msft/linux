/* SPDX-License-Identifier: GPL-2.0 */
/*
 * HiSilicon HiSTB V500 encoder: register image.
 *
 * The offsets and bit meanings follow the vendor's V500R001 register access
 * layer, which is the build this SoC's clock part uses; the ABI note lists the
 * sources and the evidence level of every entry here.
 */
#ifndef HISTB_VENC_V5_REGS_H
#define HISTB_VENC_V5_REGS_H

/*
 * One window holds the encoder core and, 0xf000 bytes into it, the SMMU/TBU
 * block the core sits behind. The vendor maps 0xf700 bytes, which is the last
 * register it touches plus four.
 */
#define V500_WINDOW_SIZE			0x10000

#define V500_INT_ENABLE				0x0000
#define V500_INT_CLEAR				0x0004
#define V500_START				0x0008
#define V500_CMD_ADDR				0x0118

/* Interrupt sources, in both the enable and the clear word. */
#define V500_INT_PIC				BIT(0)
#define V500_INT_SLICE				BIT(1)
#define V500_INT_BUF_FULL			BIT(2)
#define V500_INT_PBIT_OVERFLOW			BIT(3)
#define V500_INT_TIMEOUT			BIT(6)
/*
 * The vendor unmasks 0xcd, which is PIC, BUF_FULL, PBIT_OVERFLOW, TIMEOUT and
 * bit 7 together, and adds SLICE again when low-latency mode is on. Bit 7 is
 * unmasked there but never decoded from the status word, so the interrupt
 * handler clears any report it cannot name.
 */
#define V500_INT_ENABLE_DEFAULT			0xcd
#define V500_INT_CLEAR_ALL			GENMASK(31, 0)

/*
 * Completion and rate-control read-backs. The end-of-picture and error bits are
 * the same values as the interrupt sources.
 */
#define V500_STATUS				0x3004
#define V500_STATUS_PIC				BIT(0)
#define V500_STATUS_SLICE			BIT(1)
#define V500_STATUS_BUF_FULL			BIT(2)
#define V500_STATUS_PBIT_OVERFLOW		BIT(3)
#define V500_STATUS_TIMEOUT			BIT(6)

/*
 * Code length of the finished picture, in bytes: the vendor's read-back takes
 * bits as the register value shifted by three. Two offsets exist because the
 * core moves it with the slice-report set; the vendor reads one of them
 * according to the protocol in use.
 */
#define V500_PIC_BITS_A				0x37c0
#define V500_PIC_BITS_B				0x3840
#define V500_PIC_BITS_H264			V500_PIC_BITS_B
#define V500_PIC_BITS_HEVC			V500_PIC_BITS_A

/*
 * Per-slice report, eight bytes apart for the sixteen slice slots: the running
 * bit count and, next to it, an amount the vendor subtracts masked to seven
 * bits. Bit 31 of the first word marks the slice as the last one.
 */
#define V500_SLICE_BITS(i)			(0x3844 + (i) * 8)
#define V500_SLICE_ADJUST(i)			(0x3848 + (i) * 8)
#define V500_SLICE_BITS_MASK			GENMASK(28, 0)
#define V500_SLICE_ADJUST_MASK			GENMASK(6, 0)
#define V500_SLICE_LAST				BIT(31)

/* Rate-control statistics of the finished picture. */
#define V500_RC_STAT_A				0x31c0
#define V500_RC_STAT_B				0x31c4
#define V500_RC_MADI				0x37c8
#define V500_RC_MADP				0x37cc
#define V500_RC_MB_BITS				0x3ac4
#define V500_RC_QP_SUM				0x3ae4

/*
 * The core also takes register-side configuration while it reads the picture
 * from the image. Only the words whose every field is derivable from the
 * picture are written here; the rest keep the value the block comes up with.
 * The ABI note lists which field each of these carries.
 */
#define V500_CORE_CTRL			0x001c
#define  V500_CORE_ALWAYS		(BIT(1) | BIT(6) | BIT(7) | BIT(17))
#define  V500_CORE_DDR_MODE		BIT(4)
#define  V500_CORE_NOT_HEVC		BIT(2)
#define  V500_CORE_HEVC			BIT(26)
#define  V500_CORE_NOT_HIGH		GENMASK(19, 18)
#define  V500_CORE_CAVLC		BIT(12)
#define  V500_CORE_INTRA		BIT(16)
#define  V500_CORE_INTER		BIT(10)
#define  V500_CORE_LOW_DELAY		BIT(5)
#define V500_TUNE_WORD(n)			((n) * 4)
#define V500_SEARCH_CFG			0x0044
#define  V500_SEARCH_CFG_DEFAULT	0x91b1
#define V500_MB_TARGET_BITS		0x024c

/* Sub-window at the top of the mapping, the core's own SMMU/TBU. */
#define V500_SMMU_OFFSET			0xf000
#define V500_SMMU_INT_STATUS			(V500_SMMU_OFFSET + 0x018)
#define V500_SMMU_INT_CLEAR			(V500_SMMU_OFFSET + 0x01c)
#define V500_SMMU_PAGE_BASE_NS			(V500_SMMU_OFFSET + 0x20c)
#define V500_SMMU_ERR_WRITE_NS			(V500_SMMU_OFFSET + 0x308)
#define V500_SMMU_ERR_READ_NS			(V500_SMMU_OFFSET + 0x304)
#define V500_SMMU_FAULT_WRITE_S			(V500_SMMU_OFFSET + 0x330)
#define V500_SMMU_FAULT_WRITE_NS		(V500_SMMU_OFFSET + 0x340)
#define V500_SMMU_FAULT_READ_S			(V500_SMMU_OFFSET + 0x350)
#define V500_SMMU_FAULT_READ_NS			(V500_SMMU_OFFSET + 0x360)

/*
 * The frame descriptor the core reads. The vendor packs it from a C structure
 * of the same field set; the offsets here are that structure's words, as the
 * command image holds them.
 */
#define V500_IMAGE_BYTES			4096
#define V500_IMAGE_ALIGN			64

#endif /* HISTB_VENC_V5_REGS_H */
