/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * HiSilicon HiSTB V5R6 video decoder, driver contract.
 *
 * The hardware consumes fully parsed syntax: userspace submits per-frame
 * parsing results through the V4L2 stateless controls, the driver packs them
 * into the downstream message slots, programs the core and rings the doorbell.
 * Slot geometry, the launch sequence and the message fields follow
 * the V5R6 ABI note of this project; the register names come from the
 * vendor header for the sibling revision of this core.
 */
#ifndef HISTB_VDEC_V5_H
#define HISTB_VDEC_V5_H

#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/wait.h>
#include <linux/videodev2.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-v4l2.h>

#include "histb_vdec_core.h"
#include "histb_vdec_v5_regs.h"

#define HISTB_VDEC_V5_NAME		"histb-vdec-v5"

/*
 * One downstream message slot holds 256 words and, for this channel, the slots
 * step by one slot: the H.264 assembly indexes a slot as index times 2 to the
 * 10, the picture message of a frame sits at the table's fourth slot, and the
 * first slice message right behind it. The engine takes the slice message at
 * one step past the picture message, so the stride is what puts a descriptor
 * where the core looks for it.
 */
#define V5R6_SLOT_WORDS			256
#define V5R6_SLOT_BYTES			(V5R6_SLOT_WORDS * 4)
#define V5R6_SLOT_STRIDE		1024
#define V5R6_SLOT_NUM			210

/* Slot indices: upstream reports, repair, picture, then the slice chain. */
#define V5R6_SLOT_UP			0
#define V5R6_SLOT_RP			2
#define V5R6_SLOT_PIC			4
#define V5R6_SLOT_SLICE			5
#define V5R6_MAX_SLICES			136

/* The upstream record is four words per slice, up to 200 slices. */
#define V5R6_UP_WORDS_PER_SLICE		4
#define V5R6_UP_MAX_SLICES		200

/* Reference and motion-history limits this generation offers. */
#define V5R6_MAX_DPB_SLOTS		16
#define V5R6_MAX_FRAMES			32
#define V5R6_MAX_PMV_SLOTS		20
#define V5R6_MAX_WIDTH			4096
#define V5R6_MAX_HEIGHT			2304

/* A launch that does not report in that long is not going to. */
#define V5R6_TIMEOUT			msecs_to_jiffies(500)
#define V5R6_TIMEOUT_MS_POLL		200

/* Nominal OUTPUT (bitstream) plane size. */
#define V5R6_OUTPUT_SIZE_DEFAULT	(3 * 1024 * 1024)

struct histb_vdec_v5;
struct histb_vdec_v5_ctx;

/**
 * struct histb_vdec_v5_codec_ops - message production for one protocol.
 * @run:	pack the picture slot and the slice chain of one access unit,
 *		program the core and wait for it, for the frame the caller
 *		handed as destination surface.
 * @try_ctrl:	reject control values this protocol cannot express to the core.
 * @stop:	drop whatever per-stream state the protocol kept for a context.
 */
struct histb_vdec_v5_codec_ops {
	int (*run)(struct histb_vdec_v5_ctx *ctx, struct vb2_v4l2_buffer *src,
		   struct vb2_v4l2_buffer *dst);
	int (*try_ctrl)(struct v4l2_ctrl *ctrl);
	void (*stop)(struct histb_vdec_v5_ctx *ctx);
	/* @buf_release: the capture buffer at @index is gone; drop whatever
	 * slots this protocol gave it. @index is not a slot id here. */
	void (*buf_release)(struct histb_vdec_v5_ctx *ctx, unsigned int index);
};

/**
 * histb_vdec_v5_fmt_desc - one protocol's description of itself.
 * @fourcc:	OUTPUT format.
 * @vidstd:	value for CFG1's video_standard field.
 * @ctrls:	stateless controls this protocol needs, and how many.
 * @ops:	how to turn the controls of one access unit into message slots.
 * @stride:	band pitch the core writes for a coded width, which is also the
 *		bytesperline the capture queue has to advertise.
 * @grid_align:	height granularity of that pitch, in lines.
 */
typedef struct histb_vdec_fmt histb_vdec_v5_fmt_desc;
typedef struct histb_vdec_chip histb_vdec_v5_chip_t;

/* Per-protocol descriptors, defined by the codec sources. */
extern const histb_vdec_v5_fmt_desc histb_vdec_v5_h264_fmt;
extern const histb_vdec_v5_fmt_desc histb_vdec_v5_hevc_fmt;
extern const histb_vdec_v5_fmt_desc histb_vdec_v5_mpeg2_fmt;
extern const histb_vdec_v5_fmt_desc histb_vdec_v5_mpeg4_fmt;
extern const histb_vdec_v5_fmt_desc histb_vdec_v5_h263_fmt;

/*
 * A capture buffer as the core sees it: the luma start, the geometry the
 * launch is programmed with, and the two slot ids the messages use -- the
 * picture container the reference lists name, and the motion-history area the
 * core keeps for this surface.
 */
struct histb_vdec_v5_frame {
	dma_addr_t	dma;
	void		*va;
	unsigned int	index;
	unsigned int	apc_slot;
	unsigned int	pmv_slot;
	unsigned int	pitch;
	unsigned int	height;
	size_t		size;
	bool		used;
};

/*
 * The motion-history areas the core keeps per reference surface, one block per
 * picture container in one allocation per context. The vendor sizes a block as
 * one macroblock row of 64 bytes per macroblock, with room for the row beyond
 * the coded grid.
 */
struct histb_vdec_v5_pmv {
	struct device	*dev;
	dma_addr_t	dma;
	void		*va;
	size_t		slot_bytes;
	unsigned int	slots;
};

struct histb_vdec_v5_ctx {
	struct v4l2_fh		fh;
	struct v4l2_ctrl_handler ctrl_hdl;
	const struct histb_vdec_fmt *desc;
	struct v4l2_format	out_fmt;
	struct v4l2_format	cap_fmt;

	/* One record per capture buffer of this context's capture queue, so a
	 * reference can be named by the buffer index the engine is given.
	 */
	struct histb_vdec_v5_frame frames[V5R6_MAX_FRAMES];
	u8			apc_next;
	u8			pmv_next;
	struct histb_vdec_v5_pmv	pmv;

	/* What the last submitted picture claims to cover, for the
	 * report-coverage check the completion path runs.
	 */
	unsigned int		mbs;
	unsigned int		slices;

	void			*priv;
};

/*
 * Pool layout, taken from the vendor build's own OpenHAL: the slot array
 * starts at the 1024-aligned pool base and the constant tables and the
 * neighbour-history areas follow at fixed offsets. Several unrelated names
 * point at one area, which is why they are grouped here rather than one per
 * message word.
 */
#define V5R6_POOL_BYTES			0x96000	/* GetHalMemSize(): 614400 */
#define V5R6_POOL_SED_LEFT_PHY		0x41000	/* also pmv_top and pmv_left */
#define V5R6_POOL_CABAC_MN_H264		0x41c00
#define V5R6_POOL_RCNTOP_PHY		0x42c00	/* also the tile/segment image */
#define V5R6_POOL_CABAC_MN_HEVC		0x43000
#define V5R6_POOL_DBLKLEFT_PHY		0x43300
#define V5R6_POOL_DBLKTOP_PHY		0x49600	/* also sao_left */
#define V5R6_POOL_NEIGHBOURS		0x4c000	/* the CPU-side view of all five */
#define V5R6_POOL_SAO_TOP_PHY		0x55000	/* also the upstream slot image */
#define V5R6_POOL_PPFD			0x55c00

/*
 * The per-picture tables each get a span of the length this revision's
 * firmware gives it, so that the history one stage writes cannot land inside
 * the input of another. They sit past the vendor pool's own map, which the
 * constants above keep describing.
 */
#define V5R6_TBL_SED_TOP		0x56000
#define V5R6_TBL_PMV_TOP		(V5R6_TBL_SED_TOP + 352 * 96)
#define V5R6_TBL_RCNT_TOP		(V5R6_TBL_PMV_TOP + 352 * 128)
#define V5R6_TBL_DBLK_TOP		(V5R6_TBL_RCNT_TOP + 352 * 128)
#define V5R6_TBL_PMV_FRAME		(V5R6_TBL_DBLK_TOP + 352 * 192)
#define V5R6_TBL_END			(V5R6_TBL_PMV_FRAME + 25600)

static_assert(V5R6_TBL_END <= V5R6_POOL_BYTES,
	      "the per-picture tables do not fit the pool");

/*
 * Frame geometry the vendor's ArrangeMem programs, for the uncompressed 8-bit
 * write path: the luma band pitch is ALIGN256(ALIGN16(width)) for the
 * macroblock protocols, the chroma plane starts at pitch * ALIGN32(height),
 * and a frame slot is the two planes plus their 1.5x accounting.
 */
#define V5R6_BAND_PITCH(width)	ALIGN(ALIGN(width, 16), 256)


/*
 * Message pool: one allocation, owned by the device, borrowed by the job in
 * flight. The slots hold the downstream messages, the upstream reports and the
 * per-protocol constant tables; the neighbour-history areas the core reads back
 * sit after them. Addresses handed to the core are 16-byte aligned.
 */
struct histb_vdec_v5_pool {
	dma_addr_t	dma;
	void		*va;
	size_t		size;
	unsigned int	slot_off;	/* the vendor's own pool base alignment */
};

struct histb_vdec_v5 {
	struct histb_vdec_common common;
	struct device		*dev;
	void __iomem		*base;
	int			irq;

	const struct histb_vdec_chip *chip;

	struct histb_vdec_v5_pool	pool;
	struct mutex		hw_lock;	/* one frame at a time */
	spinlock_t		irq_lock;
	struct completion	done;
	u32			int_state;
	u32			last_state;
	struct regmap		*crg;
	atomic_t		open_count;
	wait_queue_head_t	open_wq;
};

static inline void *histb_vdec_v5_slot(struct histb_vdec_v5 *dec,
				     unsigned int index)
{
	return dec->pool.va + dec->pool.slot_off + index * V5R6_SLOT_STRIDE;
}

static inline dma_addr_t histb_vdec_v5_slot_dma(struct histb_vdec_v5 *dec,
					      unsigned int index)
{
	return dec->pool.dma + dec->pool.slot_off +
	       index * (dma_addr_t)V5R6_SLOT_STRIDE;
}

/*
 * One frame handed to the core. The message slots are already packed by the
 * protocol layer; the launch only programs the addresses and rings the doorbell.
 */
struct histb_vdec_v5_job {
	u32	cfg0;		/* the engine's first config word, as the protocol builds it */
	u32	cfg1;		/* the engine's second config word */
	u32	to_val;		/* stage watchdog value for all eight stages */
	u32	reg_shift;	/* unit of the address registers, as a right shift */
	unsigned int pitch;		/* band pitch in bytes, as the core counts it */
	dma_addr_t stream;		/* bitstream base, 16-byte aligned */
	dma_addr_t recon;		/* reconstructed frame base */
	unsigned int uv_offset;		/* chroma start from the luma base */
	dma_addr_t pic_dma;		/* picture message slot the core reads */
	dma_addr_t up_dma;		/* report slot the core fills */
	void		*pic;		/* packed picture slot */
	void		*slice;		/* packed first slice slot */
};

/* hw.c */
void histb_vdec_v5_clocks(struct histb_vdec_v5 *dec, bool on);
int histb_vdec_v5_pool_alloc(struct histb_vdec_v5 *dec);
void histb_vdec_v5_pool_seed(struct histb_vdec_v5 *dec);
int histb_vdec_v5_launch(struct histb_vdec_v5 *dec, const struct histb_vdec_v5_job *job);
void histb_vdec_v5_abort(struct histb_vdec_v5 *dec);

/* drv.c, used by the protocol sources */

/* v4l2.c, the media node this driver offers. */
int histb_vdec_v5_probe(struct platform_device *pdev,
			const struct histb_vdec_gen *gen);
void histb_vdec_v5_remove(struct platform_device *pdev);
int histb_vdec_v5_runtime_suspend(struct device *dev);
int histb_vdec_v5_runtime_resume(struct device *dev);
void histb_vdec_v5_watchdog(struct work_struct *work);

/* hw.c, the per-context motion-history areas. */
int histb_vdec_v5_pmv_alloc(struct histb_vdec_v5 *dec, struct histb_vdec_v5_ctx *ctx,
			  unsigned int mbs);
void histb_vdec_v5_pmv_free(struct histb_vdec_v5_ctx *ctx);
static inline void histb_vdec_v5_write(struct histb_vdec_v5 *dec, unsigned int reg,
				     u32 val)
{
	writel(val, dec->base + reg);
}

static inline u32 histb_vdec_v5_read(struct histb_vdec_v5 *dec, unsigned int reg)
{
	return readl(dec->base + reg);
}

/*
 * This generation hands the core 16-byte granules, in the address registers
 * and in the address words of the messages alike. The older engine takes byte
 * addresses, so nothing here may be shared between the two.
 */
#define V5R6_ADDR_SHIFT			4

static inline u32 histb_vdec_v5_addr_at(dma_addr_t dma, unsigned int shift)
{
	return (lower_32_bits(dma) & ~15U) >> shift;
}

static inline u32 histb_vdec_v5_addr(dma_addr_t dma)
{
	return histb_vdec_v5_addr_at(dma, V5R6_ADDR_SHIFT);
}

/* drv.c, for the protocol sources. */
struct histb_vdec_v5 *histb_vdec_v5_of_ctx(const struct histb_vdec_v5_ctx *ctx);
void histb_vdec_v5_buf_geom(const struct histb_vdec_v5_ctx *ctx, unsigned int *pitch,
			  unsigned int *lines, unsigned int *uv_offset);
unsigned int histb_vdec_v5_frame_size(const struct histb_vdec_v5_ctx *ctx,
				    unsigned int width, unsigned int height);
unsigned int histb_vdec_v5_linear_size(unsigned int width, unsigned int height);
struct histb_vdec_v5_frame *
histb_vdec_v5_frame_of(struct histb_vdec_v5_ctx *ctx, unsigned int index);

#endif /* HISTB_VDEC_V5_H */
