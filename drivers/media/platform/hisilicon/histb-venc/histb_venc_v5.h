/* SPDX-License-Identifier: GPL-2.0 */
/*
 * HiSilicon HiSTB V500 encoder driver, internal contract.
 *
 * The core is a stateful encoder: raw pictures in, network abstraction layer
 * data out, so it keeps reconstruction buffers and a coded-stream ring of its
 * own, and the driver generates the parameter sets and the slice headers.
 *
 * A picture is handed over as a command image in DMA memory rather than as
 * register writes; the ABI note lists the layout and its sources. The vendor's
 * hardware layer keeps its state in two globals and cannot be re-entered, so
 * one streaming context holds the core for a whole run.
 */
#ifndef HISTB_VENC_V5_H
#define HISTB_VENC_V5_H

#include <linux/clk.h>
#include <linux/math.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/reset.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-mem2mem.h>

#include "histb_venc_v5_regs.h"

/* ctx->fmt is indexed by the queue's role, as V4L2_TYPE_IS_OUTPUT says. */
#define V500_FMT_CAPTURE	0
#define V500_FMT_OUTPUT		1

#define V500_STREAM_SLOTS		16
/*
 * Every slice writes into its own stream slot. The vendor build only fills
 * four, but the core reports sixteen, so all sixteen get a slot: a slice that
 * lands on an address-zero slot is a write to physical zero.
 */
#define V500_SLICE_BUFFERS		16
#define V500_RECON_BUFFERS		2

/*
 * The ring keeps the two pointers the core advances in its first cell, so the
 * coded bytes start after it rather than at the allocation's face.
 */
#define V500_POINT_READ			0
#define V500_POINT_WRITE		16
#define V500_RING_OFFSET		32
#define V500_STREAM_BYTES		(V500_RING_OFFSET + SZ_4M)
#define V500_RECON_BYTES		SZ_4M
#define V500_PARAM_BYTES		SZ_64K
#define V500_PARAM_WORDS		48
/* Space the coded slice still needs after the joined parameter sets. */
#define V500_PARAM_HEADROOM		SZ_1K
/* Words in each of the image's two lambda tables. */
#define V500_LAMBDA_WORDS		40
/*
 * The core's protocol selector, the value the vendor's own enum gives: the
 * command image and the core control word both name the syntax this way.
 */
#define V500_PROTOCOL_H264		0
#define V500_PROTOCOL_HEVC		2
/* The H.264 profiles the image names, in the vendor's numbering. */
#define V500_H264_PROFILE_BASELINE	0
#define V500_H264_PROFILE_MAIN		1
#define V500_H264_PROFILE_EXTENDED	2
#define V500_H264_PROFILE_HIGH		3
/*
 * How the input planes are stored, in the form the image's format word takes:
 * the vendor's layer turns semi-planar into zero, planar into three and a
 * packed picture into six before it puts the value six bits up.
 */
#define V500_STORE_SEMI_PLANAR		0
#define V500_STORE_PLANAR		3
#define V500_STORE_PACKAGE		6
/* The core works on whole blocks of this edge. */
#define V500_BLOCK_ALIGN		16
/* The interleaved pair's order, which is all the image's byte names there. */
#define V500_SEMI_VU			0
#define V500_SEMI_UV			1
/* The byte a packed picture is named by: Y0 U Y1 V, and U Y0 V Y1. */
#define V500_PACKAGE_YUYV		0xd8
#define V500_PACKAGE_UYVY		0x8d
/* The two HEVC profiles this core can be told to signal. */
#define V500_HEVC_PROFILE_MAIN		0
#define V500_HEVC_PROFILE_MSP		1
/* The picture-type word's two forms. */
#define V500_PICTURE_INTRA		0x26010200
#define V500_PICTURE_INTER		0x02010200
#define V500_SLICE_BYTES		SZ_4K
/*
 * One area covers every buffer the core only touches once the picture is not
 * intra: it is handed addresses for them even for an intra picture, and a zero
 * address there is a write to physical zero.
 */
#define V500_SCRATCH_BYTES		(SZ_128K * 8)

struct histb_venc_v5_ctx;

struct histb_venc_v5_buffer {
	struct vb2_v4l2_buffer	vb;
	struct histb_venc_v5_ctx	*ctx;
	struct list_head	list;
	enum vb2_buffer_state	state;
	unsigned int		flags;
};

struct histb_venc_v5_dma {
	void *va;
	dma_addr_t dma;
	size_t size;
};

struct histb_venc_v5 {
	struct device		*dev;
	void __iomem		*base;
	int			irq;
	struct clk_bulk_data	*clks;
	int			nclks;
	struct reset_control	*reset;
	bool			clocks_on;
	/* the core serves one context at a time */
	struct mutex		lock;
	struct histb_venc_v5_ctx	*ctx;
	/* protocol the running stream codes, for the read-backs that move */
	unsigned int		protocol;
	struct v4l2_device	v4l2;
	struct media_device	mdev;
	struct video_device	vfd;
	struct v4l2_m2m_dev	*m2m;
	struct mutex		vdev_lock;
	struct completion	done;
	bool			removing;
	atomic_t		open_count;
	wait_queue_head_t	open_wq;
	unsigned int		status;
	struct histb_venc_v5_dma	image;
	struct histb_venc_v5_dma	stream;
	struct histb_venc_v5_dma	recon;
	struct histb_venc_v5_dma	param;
	struct histb_venc_v5_dma	slice;
	struct histb_venc_v5_dma	scratch;
};

/*
 * What one picture needs of the core, in the units the command image carries:
 * bus addresses, bytes, and macroblock-relative counts.
 */
struct histb_venc_v5_frame {
	unsigned int width;
	unsigned int height;
	unsigned int src_stride_y;
	unsigned int src_stride_c;
	unsigned int recon_stride_y;
	unsigned int recon_stride_c;
	dma_addr_t src_y;
	dma_addr_t src_c;
	dma_addr_t src_v;
	unsigned int store;
	unsigned int package;
	dma_addr_t recon[V500_RECON_BUFFERS];
	unsigned int recon_index;
	dma_addr_t stream[V500_STREAM_SLOTS];
	unsigned int stream_size[V500_STREAM_SLOTS];
	dma_addr_t stream_read;
	dma_addr_t stream_write;
	dma_addr_t param;
	size_t param_size;
	u32 param_reg[V500_PARAM_WORDS];
	unsigned int frame_num;
	unsigned int idr_pic_id;
	/* HEVC names pictures by a sixteen bit count instead of frame_num. */
	unsigned int poc;
	dma_addr_t slice_header;
	unsigned int slice_header_bits;
	dma_addr_t tmv[2];
	dma_addr_t pme[2];
	dma_addr_t pme_info[3];
	dma_addr_t tunnel_cell;
	unsigned int intra;
	unsigned int target_bits;
	/* The rate this picture is coded at, for the budget and the VUI. */
	unsigned int bitrate;
	unsigned int fps_num;
	unsigned int fps_den;
	unsigned int start_qp;
	unsigned int min_qp;
	unsigned int max_qp;
	int qp_i_delta;
	int qp_p_delta;
	/* The H.264 profile in the image's own numbering. */
	unsigned int profile;
	unsigned int cabac;
	unsigned int slice_split;
	unsigned int slice_split_size;
	unsigned int low_latency;
	unsigned int parameter_set;
	unsigned int protocol;
	unsigned int hevc_profile;
	unsigned int hevc_level;
};

/*
 * The core codes whole blocks, so the picture it codes is the negotiated one
 * rounded up to its own grid: macroblocks for H.264, 32-pixel coding trees for
 * HEVC. The parameter sets crop the difference away.
 */
static inline unsigned int
histb_venc_v5_coded_height(const struct histb_venc_v5_frame *frame)
{
	return ALIGN(frame->height,
		     frame->protocol == V500_PROTOCOL_HEVC ? 32 : 16);
}

static inline unsigned int
histb_venc_v5_block_rows(const struct histb_venc_v5_frame *frame)
{
	return DIV_ROUND_UP(frame->height,
			    frame->protocol == V500_PROTOCOL_HEVC ? 32 : 16);
}

static inline unsigned int
histb_venc_v5_blocks(const struct histb_venc_v5_frame *frame)
{
	return DIV_ROUND_UP(frame->width,
			    frame->protocol == V500_PROTOCOL_HEVC ? 32 : 16) *
	       histb_venc_v5_block_rows(frame);
}

struct histb_venc_v5_ctx {
	struct v4l2_fh		fh;
	struct histb_venc_v5	*enc;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_m2m_ctx	*m2m_ctx;
	/* Which syntax the capture queue asks the core to code. */
	bool			hevc;
	/* Negotiated format of each queue, by the V4L2_TYPE_IS_OUTPUT test. */
	struct v4l2_format	fmt[2];
	unsigned int		sequence;
	unsigned int		gop;
	unsigned int		bitrate;
	unsigned int		bitrate_mode;
	/* Pictures left before the next one opens a group; zero means now. */
	unsigned int		gop_left;
	/* The next picture is an IDR because the client asked for one. */
	bool			force_idr;
	/* H.264 entropy coder and profile, as the menus name them. */
	unsigned int		cabac;
	unsigned int		profile;
	/* The two quantisers the constant-quality mode codes with. */
	unsigned int		qp_i;
	unsigned int		qp_p;
	/* Bits spent in the group of pictures, for the VBR budget. */
	u64			gop_bits;
	/* Slice splitting: units of macroblocks, 0 = one slice a picture. */
	unsigned int		slice_mb;
	unsigned int		hevc_profile;
	unsigned int		hevc_level;
	unsigned int		fps_num;
	unsigned int		fps_den;
	unsigned int		rc_qp;
	unsigned int		user_min_qp;
	unsigned int		user_max_qp;
	s64			rc_credit;
	/* The OUTPUT queue's picture: the geometry a buffer is laid out with. */
	unsigned int		src_w;
	unsigned int		src_h;
	/*
	 * The sub-rectangle of that picture the core is told to code, or an all
	 * zero rectangle for the whole of it.
	 */
	struct v4l2_rect	crop;
	struct histb_venc_v5_frame frame;
};

static inline struct histb_venc_v5 *histb_venc_v5_of_ctx(struct histb_venc_v5_ctx *ctx)
{
	return ctx->enc;
}

static inline struct histb_venc_v5_buffer *vb_to_hbuf(struct vb2_v4l2_buffer *vb)
{
	return container_of(vb, struct histb_venc_v5_buffer, vb);
}

/* The three groups of slice-header words the core splices in front of the
 * macroblock data, plus the bit counts of each. HEVC instead hands the core a
 * short first segment and the rest as CABAC-side header words, in the same two
 * bit counts: [0] for the first segment, [1] for the rest.
 */
struct histb_venc_v5_slice {
	u32 header[4];
	u32 reorder[2];
	u32 marking[2];
	u32 bits[2];
	u32 part1;
	u32 cabac[8];
};

/* histb_venc_v5_hw.c */
int histb_venc_v5_buffers_alloc(struct histb_venc_v5 *enc);
void histb_venc_v5_buffers_free(struct histb_venc_v5 *enc);
void histb_venc_v5_image_pack(struct histb_venc_v5 *enc,
			    const struct histb_venc_v5_frame *frame,
			    const struct histb_venc_v5_slice *slice);
void histb_venc_v5_launch(struct histb_venc_v5 *enc,
		       const struct histb_venc_v5_frame *frame);
unsigned int histb_venc_v5_slot_stride(struct histb_venc_v5 *enc);
ssize_t histb_venc_v5_drain(struct histb_venc_v5 *enc,
			  const struct histb_venc_v5_frame *frame,
			  void *dst, size_t size);
unsigned int histb_venc_v5_pic_bits(struct histb_venc_v5 *enc);
unsigned int histb_venc_v5_status(struct histb_venc_v5 *enc);
void histb_venc_v5_stop_core(struct histb_venc_v5 *enc);

/* histb_venc_v5_bitstream.c */
int histb_venc_v5_parameter_sets(struct histb_venc_v5_frame *frame,
			       void *param_va, size_t room, u32 *words);
void histb_venc_v5_slice_header(struct histb_venc_v5_frame *frame,
			      struct histb_venc_v5_slice *slice);

/* histb_venc_v5_v4l2.c */
int histb_venc_v5_v4l2_init(struct histb_venc_v5 *enc);
void histb_venc_v5_v4l2_exit(struct histb_venc_v5 *enc);

#endif /* HISTB_VENC_V5_H */
