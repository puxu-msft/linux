// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * histb-vpss.c - HiSilicon HiSTB video processing subsystem, mem2mem form.
 *
 * The block sits between a decoder and an encoder or a display: it scales and
 * de-interlaces frames and converts between the semi-planar 4:2:0 and 4:2:2
 * layouts. Everything the block can do that has no V4L2 interface on a mem2mem
 * device (tone mapping, temporal noise reduction, a second output port, a
 * write-back capture channel) is deliberately not implemented: what this driver
 * offers is the format conversion, the two-axis scaling implied by the OUTPUT
 * and CAPTURE sizes, rotation, and the deinterlacing mode control.
 */
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>

#include "histb-vpss-regs.h"
#include "histb-vpss-zme-coef.h"

/* Rotation runs as one image per plane, chained. */
#define HISTB_VPSS_IMAGES	2

#define HISTB_VPSS_NAME		"histb-vpss"

/*
 * The block only needs its clock while a job is in flight, but a client that
 * dequeues and requeues back-to-back would otherwise pay a reset per job.
 */
#define HISTB_VPSS_AUTOSUSPEND_MS	200

#define VPSS_MIN_WIDTH		32
#define VPSS_MIN_HEIGHT		32
#define VPSS_MAX_WIDTH		1920
#define VPSS_MAX_HEIGHT		1088
#define VPSS_ALIGN		16

/*
 * Deinterlacing modes offered through V4L2_CID_DEINTERLACING_MODE.
 * Bob turns one field into a frame by doubling it vertically; Weave states
 * that no filtering happens, so its job is the plain conversion of a frame
 * that already holds both fields. Motion-adaptive interpolation reads the
 * field, the one before it and the two after it from a driver-owned ring, and
 * a motion history the block keeps across jobs.
 */
#define VPSS_DEI_NONE		0
#define VPSS_DEI_BOB		1
#define VPSS_DEI_WEAVE		2
#define VPSS_DEI_MAD		3

/* The ring holds whole pictures: each one carries the two fields of its
 * moment in the half they occupy, so the block can weave a field with its
 * partner and compare whole pictures across time.
 */
#define VPSS_DEI_RING		4
/* Motion history planes are fixed to the block's maximum picture (960 x 544
 * bytes per plane on a 1920-wide frame), as the vendor's allocator does.
 */
#define VPSS_DEI_HIST_STRIDE	960
#define VPSS_DEI_HIST_PLANE	(VPSS_DEI_HIST_STRIDE * 544)
#define VPSS_DEI_HIST_PLANES	3

struct histb_vpss_fmt {
	u32	fourcc;
	bool	nv21;
	bool	fmt422;
	bool	packed;
};

struct histb_vpss {
	struct device		*dev;
	void __iomem		*regs;
	struct clk		*clk;
	struct reset_control	*rst;
	struct completion	done;
	struct v4l2_device	v4l2_dev;
	struct video_device	vdev;
	struct v4l2_m2m_dev	*m2m_dev;
	struct mutex		lock;	/* serialises jobs and format changes */
	void			*shadow;	/* register images the block runs */
	dma_addr_t		shadow_dma;
	void			*coef;		/* scaler phase tables for one bucket */
	dma_addr_t		coef_dma;
	bool			busy;
	u32			version;	/* VPSS_VERSION as first read */
	bool			faulted;	/* reset failed; no job is attempted */
	bool			removing;	/* device_run() refuses work */
};

struct histb_vpss_ctx {
	struct v4l2_fh		fh;
	struct histb_vpss	*vpss;
	struct v4l2_pix_format	src_fmt;
	struct v4l2_pix_format	dst_fmt;
	struct v4l2_rect	crop;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*dei_ctrl;
	struct v4l2_ctrl	*hflip;
	struct v4l2_ctrl	*vflip;
	struct v4l2_ctrl	*sharpness;
	struct v4l2_ctrl	*rotate;

	/* Motion-adaptive deinterlacing keeps the fields the block reads
	 * beside the current one, and the motion history it accumulates
	 * between jobs. All of it is the driver's own memory: a client hands
	 * the block one field per buffer and gets a frame back.
	 */
	void		*dei_fields;
	dma_addr_t	dei_fields_dma;
	size_t		dei_frame_bytes;
	unsigned int	dei_count;
	unsigned int	dei_hist_wr;
	u64		dei_ts[VPSS_DEI_RING][2];
	bool		dei_first_bottom;
	bool		dei_last_bottom;
	void		*dei_hist;
	dma_addr_t	dei_hist_dma;
};

struct histb_vpss_job {
	dma_addr_t	src;
	dma_addr_t	dst;
	unsigned int	width;
	unsigned int	height;
	unsigned int	out_width;
	unsigned int	out_height;
	unsigned int	src_stride;
	unsigned int	dst_stride;
	bool		in_422;
	bool		out_422;
	bool		in_packed;
	unsigned int	in_fmt;
	bool		invert_uv;
	unsigned int	sharpness;
	bool		hflip;
	bool		vflip;
	unsigned int	in_x;
	unsigned int	in_y;
	unsigned int	in_w;
	unsigned int	in_h;
	unsigned int	angle;
};

static const struct histb_vpss_fmt histb_vpss_out_fmts[] = {
	{ V4L2_PIX_FMT_NV12, false, false, false },
	{ V4L2_PIX_FMT_NV21, true, false, false },
	{ V4L2_PIX_FMT_NV16, false, true, false },
	{ V4L2_PIX_FMT_NV61, true, true, false },
	{ V4L2_PIX_FMT_YUYV, false, true, true },
	{ V4L2_PIX_FMT_UYVY, false, true, true },
};

static const struct histb_vpss_fmt histb_vpss_cap_fmts[] = {
	{ V4L2_PIX_FMT_NV12, false, false, false },
	{ V4L2_PIX_FMT_NV21, true, false, false },
	{ V4L2_PIX_FMT_NV16, false, true, false },
	{ V4L2_PIX_FMT_NV61, true, true, false },
};

static int histb_vpss_reset(struct histb_vpss *vpss);
static int histb_vpss_kick(struct histb_vpss *vpss, dma_addr_t first);
static void histb_vpss_reset_crop(struct histb_vpss_ctx *ctx);
static void histb_vpss_dei_free(struct histb_vpss_ctx *ctx);

static struct histb_vpss_ctx *histb_vpss_ctx_of(struct file *file)
{
	return container_of(file->private_data, struct histb_vpss_ctx, fh);
}

static const struct histb_vpss_fmt *histb_vpss_find(const struct histb_vpss_fmt *t,
						    unsigned int n, u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < n; i++)
		if (t[i].fourcc == fourcc)
			return &t[i];

	return NULL;
}

static unsigned int histb_vpss_stride(unsigned int width, unsigned int bpl)
{
	return max(width, round_up(bpl, VPSS_ALIGN));
}

/*
 * The input sampler's code for the source layout (vpss_reg_3798m.c:580-640):
 * 0 is semi-planar 4:2:0, 2 semi-planar 4:2:2, and the packed pairs carry their
 * own codes because the byte order inside a pair is part of the layout.
 */
static unsigned int histb_vpss_in_format(const struct histb_vpss_fmt *f)
{
	if (f->fourcc == V4L2_PIX_FMT_YUYV)
		return 0xb;
	if (f->fourcc == V4L2_PIX_FMT_UYVY)
		return 0xd;
	return f->fmt422 ? 2 : 0;
}

/*
 * A client's bytesperline reaches the stride registers unchanged, and both of
 * them are 16 bits wide (VPSS_CURSTRIDE_Y, VPSS_VHDSTRIDE_Y), so a request
 * above that would be masked into a different geometry: the buffer gets sized
 * by the requested stride while the block walks the truncated one. Rejecting
 * the stride also keeps the sizeimage arithmetic below in range, since
 * U16_MAX * VPSS_MAX_HEIGHT * 2 is nowhere near a 32-bit wrap.
 */
static int histb_vpss_fill_fmt(const struct histb_vpss_fmt *f,
			       struct v4l2_pix_format *pix)
{
	unsigned int width = clamp_t(u32, ALIGN(pix->width, VPSS_ALIGN),
				     VPSS_MIN_WIDTH, VPSS_MAX_WIDTH);
	unsigned int height = clamp_t(u32, ALIGN(pix->height, VPSS_ALIGN),
				      VPSS_MIN_HEIGHT, VPSS_MAX_HEIGHT);
	unsigned int stride;

	if (pix->bytesperline > U16_MAX)
		return -EINVAL;

	pix->width = width;
	pix->height = height;
	pix->pixelformat = f->fourcc;
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = V4L2_COLORSPACE_SMPTE170M;

	/* A packed pair is two bytes per pixel and one plane, so the row is at
	 * least 2 * width long and a client asking for less is asking for a
	 * different geometry.
	 */
	if (f->packed && pix->bytesperline < 2 * width)
		pix->bytesperline = 2 * width;

	stride = histb_vpss_stride(width, pix->bytesperline);
	pix->bytesperline = stride;
	/* 4:2:2 keeps a full-height chroma plane beside the luma. */
	if (f->packed)
		pix->sizeimage = stride * height;
	else if (f->fmt422)
		pix->sizeimage = stride * height * 2;
	else
		pix->sizeimage = stride * height * 3 / 2;
	return 0;
}

static int histb_vpss_querycap(struct file *file, void *priv,
			       struct v4l2_capability *cap)
{
	strscpy(cap->driver, HISTB_VPSS_NAME, sizeof(cap->driver));
	strscpy(cap->card, HISTB_VPSS_NAME, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 HISTB_VPSS_NAME);
	return 0;
}

static int histb_vpss_enum_fmt(struct file *file, void *priv,
			       struct v4l2_fmtdesc *f)
{
	const struct histb_vpss_fmt *tab;
	unsigned int n;

	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		tab = histb_vpss_out_fmts;
		n = ARRAY_SIZE(histb_vpss_out_fmts);
	} else {
		tab = histb_vpss_cap_fmts;
		n = ARRAY_SIZE(histb_vpss_cap_fmts);
	}
	if (f->index >= n)
		return -EINVAL;

	f->pixelformat = tab[f->index].fourcc;
	return 0;
}

static int histb_vpss_g_fmt(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct histb_vpss_ctx *ctx = histb_vpss_ctx_of(file);

	if (V4L2_TYPE_IS_OUTPUT(f->type))
		f->fmt.pix = ctx->src_fmt;
	else
		f->fmt.pix = ctx->dst_fmt;
	return 0;
}

static int histb_vpss_try_fmt(struct file *file, void *priv,
			      struct v4l2_format *f)
{
	const struct histb_vpss_fmt *tab, *fmt;
	unsigned int n;

	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		tab = histb_vpss_out_fmts;
		n = ARRAY_SIZE(histb_vpss_out_fmts);
	} else {
		tab = histb_vpss_cap_fmts;
		n = ARRAY_SIZE(histb_vpss_cap_fmts);
	}

	fmt = histb_vpss_find(tab, n, f->fmt.pix.pixelformat);
	if (!fmt)
		fmt = &tab[0];

	return histb_vpss_fill_fmt(fmt, &f->fmt.pix);
}

static int histb_vpss_s_fmt(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct histb_vpss_ctx *ctx = histb_vpss_ctx_of(file);
	struct vb2_queue *vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	int ret;

	if (vb2_is_busy(vq))
		return -EBUSY;

	ret = histb_vpss_try_fmt(file, priv, f);
	if (ret)
		return ret;

	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		ctx->src_fmt = f->fmt.pix;
		histb_vpss_reset_crop(ctx);
	} else {
		/* The block converts between chroma orders but not between the
		 * 4:2:0 and 4:2:2 families: mixing them leaves the job
		 * unfinished, so a client has to stay in one family.
		 */
		const struct histb_vpss_fmt *src = histb_vpss_find(
			histb_vpss_out_fmts, ARRAY_SIZE(histb_vpss_out_fmts),
			ctx->src_fmt.pixelformat);
		const struct histb_vpss_fmt *dst = histb_vpss_find(
			histb_vpss_cap_fmts, ARRAY_SIZE(histb_vpss_cap_fmts),
			f->fmt.pix.pixelformat);

		if (!src || !dst || src->fmt422 != dst->fmt422)
			return -EINVAL;
		ctx->dst_fmt = f->fmt.pix;
	}
	return 0;
}

static void histb_vpss_reset_crop(struct histb_vpss_ctx *ctx)
{
	ctx->crop.left = 0;
	ctx->crop.top = 0;
	ctx->crop.width = ctx->src_fmt.width;
	ctx->crop.height = ctx->src_fmt.height;
}

static int histb_vpss_g_selection(struct file *file, void *priv,
				  struct v4l2_selection *sel)
{
	struct histb_vpss_ctx *ctx = histb_vpss_ctx_of(file);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
		if (V4L2_TYPE_IS_CAPTURE(sel->type))
			return -EINVAL;
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = ctx->src_fmt.width;
		sel->r.height = ctx->src_fmt.height;
		return 0;
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		if (V4L2_TYPE_IS_CAPTURE(sel->type))
			return -EINVAL;
		sel->r = ctx->crop;
		return 0;
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
		if (!V4L2_TYPE_IS_CAPTURE(sel->type))
			return -EINVAL;
		/* The port only ever writes the whole CAPTURE frame. */
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = ctx->dst_fmt.width;
		sel->r.height = ctx->dst_fmt.height;
		return 0;
	default:
		return -EINVAL;
	}
}

/*
 * The rectangle is taken on the input side, the way the vendor's split nodes
 * read a part of a frame: CUR_CTRL offsets the read inside the picture and
 * IMGSIZE shrinks to the window. A VHDCROP window instead leaves the pipeline
 * waiting for a completion that never arrives, so the destination stays whole.
 *
 * Offsets are on the block's own grid: rows and columns of 16 pixels.
 */
static int histb_vpss_s_selection(struct file *file, void *priv,
				  struct v4l2_selection *sel)
{
	struct histb_vpss_ctx *ctx = histb_vpss_ctx_of(file);
	struct v4l2_rect r = sel->r;

	if (V4L2_TYPE_IS_CAPTURE(sel->type)) {
		/* G_SELECTION reports COMPOSE as the whole CAPTURE frame, so
		 * S_SELECTION accepts exactly that and nothing smaller: the port
		 * has no working destination window.
		 */
		if (sel->target != V4L2_SEL_TGT_COMPOSE || r.left || r.top ||
		    r.width != ctx->dst_fmt.width ||
		    r.height != ctx->dst_fmt.height)
			return -EINVAL;
		sel->r = r;
		return 0;
	}
	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	if (r.left % 16 || r.top % 16 || r.width % 16 || r.height % 16 ||
	    !r.width || !r.height ||
	    r.left + r.width > ctx->src_fmt.width ||
	    r.top + r.height > ctx->src_fmt.height)
		return -EINVAL;
	ctx->crop = r;
	sel->r = r;
	return 0;
}

static int histb_vpss_qbuf_prepare(struct vb2_buffer *vb)
{
	struct histb_vpss_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct v4l2_pix_format *pix;
	size_t need;

	pix = V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type) ? &ctx->src_fmt :
							 &ctx->dst_fmt;
	need = pix->sizeimage;
	if (vb2_plane_size(vb, 0) < need) {
		dev_dbg(ctx->vpss->dev, "plane %lu < %zu\n",
			vb2_plane_size(vb, 0), need);
		return -EINVAL;
	}
	vb2_set_plane_payload(vb, 0, V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type) ?
					  (unsigned long)need : 0);
	return 0;
}

static void histb_vpss_qbuf_queue(struct vb2_buffer *vb)
{
	struct histb_vpss_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
}

static int histb_vpss_queue_setup(struct vb2_queue *vq, unsigned int *count,
				  unsigned int *planes, unsigned int sizes[],
				  struct device *alloc_devs[])
{
	struct histb_vpss_ctx *ctx = vb2_get_drv_priv(vq);
	struct v4l2_pix_format *pix = V4L2_TYPE_IS_OUTPUT(vq->type) ?
		&ctx->src_fmt : &ctx->dst_fmt;
	size_t need = pix->sizeimage;

	if (!need)
		return -EINVAL;
	if (*planes) {
		if (sizes[0] < need)
			return -EINVAL;
		return 0;
	}
	*planes = 1;
	sizes[0] = need;
	if (!*count)
		*count = 2;
	return 0;
}

static void histb_vpss_stop_streaming(struct vb2_queue *vq)
{
	struct histb_vpss_ctx *ctx = vb2_get_drv_priv(vq);
	struct vb2_v4l2_buffer *buf;

	if (V4L2_TYPE_IS_OUTPUT(vq->type)) {
		while ((buf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);
		/* A new stream starts with no fields and no motion history. */
		histb_vpss_dei_free(ctx);
	} else {
		while ((buf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);
	}
}

static const struct vb2_ops histb_vpss_qops = {
	.queue_setup = histb_vpss_queue_setup,
	.buf_prepare = histb_vpss_qbuf_prepare,
	.buf_queue = histb_vpss_qbuf_queue,
	.stop_streaming = histb_vpss_stop_streaming,
};

/*
 * One job: the block reads the OUTPUT plane, writes the CAPTURE plane, and the
 * driver waits for the completion the interrupt releases.
 */
/*
 * The scaler picks one of seven phase-table sets by the inverse of the zoom
 * ratio, with the thresholds the vendor's PQ layer uses.
 */
static unsigned int histb_vpss_zme_bucket(unsigned int in, unsigned int out)
{
	unsigned int ratio = (in << 12) / out;
	unsigned int t = (4096u * 4096u) / ratio;

	if (t > 4096u)
		return 0;
	if (t == 4096u)
		return 1;
	if (t >= 3072u)
		return 2;
	if (t >= 2048u)
		return 3;
	if (t >= 1365u)
		return 4;
	if (t >= 1024u)
		return 5;

	return 6;
}

/* Four signed 8-bit taps in one word, lowest byte first; FIELD_PREP refuses
 * negative constants, and the field keeps the low byte either way.
 */
static u32 histb_vpss_coef4(int c0, int c1, int c2, int c3)
{
	return ((u32)c0 & 0xff) | (((u32)c1 & 0xff) << 8) |
	       (((u32)c2 & 0xff) << 16) | (((u32)c3 & 0xff) << 24);
}

static void histb_vpss_load_coef(struct histb_vpss *vpss, unsigned int b,
				 bool sharpen, dma_addr_t addr[4])
{
	static const u32 *const lh[] = {
		histb_vpss_zme_lh_r0, histb_vpss_zme_lh_r1, histb_vpss_zme_lh_r2,
		histb_vpss_zme_lh_r3, histb_vpss_zme_lh_r4, histb_vpss_zme_lh_r5,
		histb_vpss_zme_lh_r6,
	};
	static const u32 *const lv[] = {
		histb_vpss_zme_lv_r0, histb_vpss_zme_lv_r1, histb_vpss_zme_lv_r2,
		histb_vpss_zme_lv_r3, histb_vpss_zme_lv_r4, histb_vpss_zme_lv_r5,
		histb_vpss_zme_lv_r6,
	};
	static const u32 *const ch[] = {
		histb_vpss_zme_ch_r0, histb_vpss_zme_ch_r1, histb_vpss_zme_ch_r2,
		histb_vpss_zme_ch_r3, histb_vpss_zme_ch_r4, histb_vpss_zme_ch_r5,
		histb_vpss_zme_ch_r6,
	};
	static const u32 *const cv[] = {
		histb_vpss_zme_cv_r0, histb_vpss_zme_cv_r1, histb_vpss_zme_cv_r2,
		histb_vpss_zme_cv_r3, histb_vpss_zme_cv_r4, histb_vpss_zme_cv_r5,
		histb_vpss_zme_cv_r6,
	};
	static const u32 *const dti_lh[] = {
		histb_vpss_dti_lh_r0, histb_vpss_dti_lh_r1, histb_vpss_dti_lh_r2,
		histb_vpss_dti_lh_r3, histb_vpss_dti_lh_r4, histb_vpss_dti_lh_r5,
		histb_vpss_dti_lh_r6,
	};
	static const u32 *const dti_lv[] = {
		histb_vpss_dti_lv_r0, histb_vpss_dti_lv_r1, histb_vpss_dti_lv_r2,
		histb_vpss_dti_lv_r3, histb_vpss_dti_lv_r4, histb_vpss_dti_lv_r5,
		histb_vpss_dti_lv_r6,
	};
	static const u32 *const dti_ch[] = {
		histb_vpss_dti_ch_r0, histb_vpss_dti_ch_r1, histb_vpss_dti_ch_r2,
		histb_vpss_dti_ch_r3, histb_vpss_dti_ch_r4, histb_vpss_dti_ch_r5,
		histb_vpss_dti_ch_r6,
	};
	static const u32 *const dti_cv[] = {
		histb_vpss_dti_cv_r0, histb_vpss_dti_cv_r1, histb_vpss_dti_cv_r2,
		histb_vpss_dti_cv_r3, histb_vpss_dti_cv_r4, histb_vpss_dti_cv_r5,
		histb_vpss_dti_cv_r6,
	};
	static const unsigned int words[4] = { 51, 51, 34, 34 };
	static const unsigned int dti_words[4] = { 51, 17, 34, 17 };
	static const u32 *const *const dti[] = { dti_lh, dti_lv, dti_ch, dti_cv };
	const u32 *tab[4] = { lh[b], lv[b], ch[b], cv[b] };
	unsigned int i;

	for (i = 0; i < 4; i++) {
		memcpy(vpss->coef + i * VPSS_COEF_BLOCK, tab[i], words[i] * 4);
		if (sharpen)
			memcpy(vpss->coef + i * VPSS_COEF_BLOCK + 256,
			       dti[i][b], dti_words[i] * 4);
		addr[i] = vpss->coef_dma + i * VPSS_COEF_BLOCK;
	}
}

/*
 * A frame is one or two passes of the same register image: one for a plain
 * conversion, two for a rotation, whose luma and chroma planes the block
 * transforms separately (img_pro_mode selects which one).
 */
struct histb_vpss_pass {
	dma_addr_t	in;
	dma_addr_t	out;
	unsigned int	in_w;
	unsigned int	in_h;
	unsigned int	off_x;
	unsigned int	off_y;
	unsigned int	frame_h;
	unsigned int	out_w;
	unsigned int	out_h;
	unsigned int	in_stride;
	unsigned int	out_stride;
	bool		in_422;
	bool		out_422;
	bool		in_packed;
	unsigned int	in_fmt;
	bool		invert_uv;
	unsigned int	sharpness;
	bool		hflip;
	bool		vflip;
};

static int histb_vpss_pass_run(struct histb_vpss *vpss,
			       const struct histb_vpss_pass *p)
{
	u32 *s = vpss->shadow;
	unsigned int src_c, dst_c, stride;

	if (!s)
		return -ENODEV;
	if (upper_32_bits(p->in) || upper_32_bits(p->out))
		return -ERANGE;

	memset(s, 0, VPSS_REG_IMAGE_SIZE);
	s[VPSS_MISCELLANEOUS / 4] = 0x03003244;
	s[VPSS_INTMASK / 4] = 0xff;
	s[VPSS_TIMEOUT / 4] = 0xffffffff;

	s[VPSS_CTRL / 4] = VPSS_CTRL_VHD_EN |
			  FIELD_PREP(VPSS_CTRL_PROT, 2);

	/* 0 selects the 4:2:0 semi-planar family, 2 the 4:2:2 one and the packed
	 * pairs their own codes; the port's format field counts 4:2:0 as 0 and
	 * 4:2:2 as 1.
	 */
	s[VPSS_CTRL2 / 4] = FIELD_PREP(VPSS_CTRL2_IN_FORMAT, p->in_fmt) |
			   FIELD_PREP(VPSS_CTRL2_VHD_FORMAT,
				      p->out_422 ? 1 : 0);
	s[VPSS_IMGSIZE / 4] = FIELD_PREP(VPSS_IMGSIZE_WIDTH, p->in_w - 1) |
			     FIELD_PREP(VPSS_IMGSIZE_HEIGHT, p->in_h - 1);
	/* The window is read by moving the picture's start, the way the vendor's
	 * split nodes do it: the address and stride stay those of the whole
	 * frame, IMGSIZE shrinks to the window, and CUR_CTRL says where inside
	 * the frame the block begins reading.
	 */
	s[VPSS_CUR_CTRL / 4] =
		FIELD_PREP(VPSS_CUR_CTRL_HOR_OFFSET, p->off_x) |
		FIELD_PREP(VPSS_CUR_CTRL_VER_OFFSET, p->off_y);

	/* A packed pair has no chroma plane of its own: the sampler takes the
	 * bytes out of the luma rows, so both the chroma origin and its pitch
	 * are left at zero.
	 */
	src_c = p->in_packed ? 0 :
		lower_32_bits(p->in) + p->in_stride * p->frame_h;
	s[VPSS_CURYADDR / 4] = lower_32_bits(p->in);
	s[VPSS_CURCADDR / 4] = src_c;
	stride = p->in_stride;	/* both layouts take the pitch in bytes */
	s[VPSS_CURSTRIDE / 4] = FIELD_PREP(VPSS_CURSTRIDE_Y, stride) |
				FIELD_PREP(VPSS_CURSTRIDE_C,
					   p->in_packed ? 0 : stride);

	s[VPSS_VHDCTRL / 4] = (p->invert_uv ? VPSS_VHDCTRL_UV_INVERT : 0) |
			      (p->hflip ? VPSS_VHDCTRL_MIRROR : 0) |
			      (p->vflip ? VPSS_VHDCTRL_FLIP : 0);
	s[VPSS_VHDSIZE / 4] =
		FIELD_PREP(VPSS_VHDSIZE_WIDTH, p->out_w - 1) |
		FIELD_PREP(VPSS_VHDSIZE_HEIGHT, p->out_h - 1);
	dst_c = lower_32_bits(p->out) + p->out_stride * p->out_h;
	s[VPSS_VHDYADDR / 4] = lower_32_bits(p->out);
	s[VPSS_VHDCADDR / 4] = dst_c;
	s[VPSS_VHDSTRIDE / 4] = FIELD_PREP(VPSS_VHDSTRIDE_Y, p->out_stride) |
				FIELD_PREP(VPSS_VHDSTRIDE_C, p->out_stride);

	/* The sharpen filters sit in the scaler's datapath, so a unity ratio
	 * still has to be programmed for them to see the frame.
	 */
	if (p->out_w != p->in_w || p->out_h != p->in_h || p->sharpness) {
		dma_addr_t caddr[4];
		unsigned int b = histb_vpss_zme_bucket(p->in_w, p->out_w);
		unsigned int zme_in = p->in_422 ? 0 : 1;
		unsigned int zme_out = p->out_422 ? 0 : 1;

		histb_vpss_load_coef(vpss, b, p->sharpness != 0, caddr);
		s[VPSS_VHD_ZME_LHADDR / 4] = lower_32_bits(caddr[0]);
		s[VPSS_VHD_ZME_LVADDR / 4] = lower_32_bits(caddr[1]);
		s[VPSS_VHD_ZME_CHADDR / 4] = lower_32_bits(caddr[2]);
		s[VPSS_VHD_ZME_CVADDR / 4] = lower_32_bits(caddr[3]);
		s[VPSS_VHD_ZMEIRESO / 4] =
			FIELD_PREP(VPSS_ZME_IRESO_W, p->in_w - 1) |
			FIELD_PREP(VPSS_ZME_IRESO_H, p->in_h - 1);
		s[VPSS_VHD_ZMEORESO / 4] =
			FIELD_PREP(VPSS_ZME_ORESO_W, p->out_w - 1) |
			FIELD_PREP(VPSS_ZME_ORESO_H, p->out_h - 1);
		s[VPSS_VHD_HSP / 4] =
			VPSS_MSC_H_MSSC | VPSS_MSC_H_CSSC |
			VPSS_MSC_H_LFIR | VPSS_MSC_H_CFIR |
			FIELD_PREP(VPSS_MSC_H_RATIO,
				   (p->in_w << 20) / p->out_w);
		s[VPSS_VHD_VSR / 4] =
			FIELD_PREP(VPSS_MSC_V_RATIO,
				   (p->in_h << 12) / p->out_h);
		s[VPSS_VHD_VSP / 4] =
			VPSS_MSC_V_MSSC | VPSS_MSC_V_CSSC |
			VPSS_MSC_V_LFIR | VPSS_MSC_V_CFIR |
			FIELD_PREP(VPSS_ZME_IN_FMT, zme_in) |
			FIELD_PREP(VPSS_ZME_OUT_FMT, zme_out);
	}

	/* Sharpen runs off the same phase table set, its taps 256 bytes in.
	 * The gain follows the vendor PQ scale: 128 is nominal, and the
	 * default luma and chroma ratios are 180 and 256.
	 */
	if (p->sharpness) {
		unsigned int lgain = (180 * p->sharpness + 64) / 128;
		unsigned int cgain = (256 * p->sharpness + 64) / 128;

		s[VPSS_VHD_LTI_CTRL / 4] =
			VPSS_LTI_CTRL_LTI_EN |
			FIELD_PREP(VPSS_LTI_CTRL_LHPASS_COEF4, 0xff) |
			FIELD_PREP(VPSS_LTI_CTRL_LMIXING_RATIO, 127) |
			FIELD_PREP(VPSS_LTI_CTRL_LGAIN_RATIO, lgain);
		s[VPSS_VHD_LHPASS_COEF / 4] = histb_vpss_coef4(-19, -8, -2, -1);
		s[VPSS_VHD_LTI_THD / 4] =
			FIELD_PREP(VPSS_LTI_THD_LOVER_SWING, 3) |
			FIELD_PREP(VPSS_LTI_THD_LUNDER_SWING, 3) |
			FIELD_PREP(VPSS_LTI_THD_LCORING_THD, 2);
		s[VPSS_VHD_LHFREQ_THD / 4] =
			FIELD_PREP(GENMASK(15, 0), 50) |
			FIELD_PREP(GENMASK(31, 16), 51);
		s[VPSS_VHD_LGAIN_COEF / 4] = histb_vpss_coef4(128, 128, 128, 0);
		s[VPSS_VHD_CTI_CTRL / 4] =
			VPSS_CTI_CTRL_CTI_EN |
			FIELD_PREP(VPSS_CTI_CTRL_CMIXING_RATIO, 127) |
			FIELD_PREP(VPSS_CTI_CTRL_CGAIN_RATIO, cgain);
		s[VPSS_VHD_CHPASS_COEF / 4] = histb_vpss_coef4(-21, -21, -11, 0);
		s[VPSS_VHD_CTI_THD / 4] =
			FIELD_PREP(VPSS_CTI_THD_COVER_SWING, 4) |
			FIELD_PREP(VPSS_CTI_THD_CUNDER_SWING, 4) |
			FIELD_PREP(VPSS_CTI_THD_CCORING_THD, 4);
	}

	/* The port writes the whole CAPTURE frame: a VHDCROP window smaller than
	 * the port never reports done, so a client that wants fewer pixels
	 * crops the source instead.
	 */
	s[VPSS_VHDLBA_BK / 4] = FIELD_PREP(GENMASK(15, 8), 0x10) |
				FIELD_PREP(GENMASK(23, 16), 0x80) |
				FIELD_PREP(GENMASK(31, 24), 0x80);
	s[VPSS_VHDCROP_SIZE / 4] =
		VPSS_CROP_EN |
		FIELD_PREP(VPSS_VHDCROP_WIDTH, p->out_w - 1) |
		FIELD_PREP(VPSS_VHDCROP_HEIGHT, p->out_h - 1);

	s[VPSS_PNEXT / 4] = 0;
	s[VPSS_START / 4] = 1;
	wmb();

	return histb_vpss_kick(vpss, vpss->shadow_dma);
}

/*
 * Rotation is a chain of two images, one per plane, exactly as the vendor's
 * rotate node builds it: the whole frame goes in (address, stride, size), the
 * rotated geometry comes out, and IMG_PRO_MODE picks luma in the first image
 * and chroma in the second. The node writes nothing else, so no crop, no
 * letterbox background, no scaler and no timeout are programmed here.
 */
static int histb_vpss_rotate_run(struct histb_vpss *vpss,
				 const struct histb_vpss_pass *p,
				 unsigned int angle)
{
	unsigned int i, src_c, dst_c;

	if (!vpss->shadow)
		return -ENODEV;
	if (upper_32_bits(p->in) || upper_32_bits(p->out))
		return -ERANGE;

	for (i = 0; i < 2; i++) {
		u32 *s = (u32 *)vpss->shadow +
			 i * (VPSS_REG_IMAGE_SIZE / sizeof(u32));

		memset(s, 0, VPSS_REG_IMAGE_SIZE);
		s[VPSS_MISCELLANEOUS / 4] = 0x03003244;
		s[VPSS_INTMASK / 4] = 0xff;
		s[VPSS_PNEXT / 4] = i ? 0 :
			lower_32_bits(vpss->shadow_dma + VPSS_REG_IMAGE_SIZE);
		s[VPSS_START / 4] = 1;

		s[VPSS_CTRL / 4] = VPSS_CTRL_VHD_EN |
				   FIELD_PREP(VPSS_CTRL_PROT, 2) |
				   VPSS_CTRL_ROTATE_EN |
				   (angle == 270 ? VPSS_CTRL_ROTATE_ANGLE : 0) |
				   FIELD_PREP(VPSS_CTRL_IMG_PRO_MODE, i + 1);
		s[VPSS_CTRL2 / 4] = FIELD_PREP(VPSS_CTRL2_IN_FORMAT,
					       p->in_422 ? 2 : 0) |
				   FIELD_PREP(VPSS_CTRL2_VHD_FORMAT,
					      p->out_422 ? 1 : 0);
		s[VPSS_IMGSIZE / 4] = FIELD_PREP(VPSS_IMGSIZE_WIDTH, p->in_w - 1) |
				     FIELD_PREP(VPSS_IMGSIZE_HEIGHT, p->in_h - 1);
		src_c = lower_32_bits(p->in) + p->in_stride * p->frame_h;
		s[VPSS_CURYADDR / 4] = lower_32_bits(p->in);
		s[VPSS_CURCADDR / 4] = src_c;
		s[VPSS_CURSTRIDE / 4] = FIELD_PREP(VPSS_CURSTRIDE_Y, p->in_stride) |
					FIELD_PREP(VPSS_CURSTRIDE_C, p->in_stride);

		s[VPSS_VHDCTRL / 4] = p->invert_uv ? VPSS_VHDCTRL_UV_INVERT : 0;
		s[VPSS_VHDSIZE / 4] =
			FIELD_PREP(VPSS_VHDSIZE_WIDTH, p->out_w - 1) |
			FIELD_PREP(VPSS_VHDSIZE_HEIGHT, p->out_h - 1);
		dst_c = lower_32_bits(p->out) + p->out_stride * p->out_h;
		s[VPSS_VHDYADDR / 4] = lower_32_bits(p->out);
		s[VPSS_VHDCADDR / 4] = dst_c;
		s[VPSS_VHDSTRIDE / 4] = FIELD_PREP(VPSS_VHDSTRIDE_Y, p->out_stride) |
					FIELD_PREP(VPSS_VHDSTRIDE_C, p->out_stride);
	}
	/* Both images have to be in memory before the block is given the head of
	 * the chain.
	 */
	wmb();

	return histb_vpss_kick(vpss, vpss->shadow_dma);
}

/* Name the failure the block reported, for the job-failed log line. */
static const char *histb_vpss_err_name(u32 st)
{
	if (st & VPSS_INTSTATE_TIMEOUT)
		return "block timeout";
	if (st & VPSS_INTSTATE_BUS_R)
		return "bus read error";
	if (st & VPSS_INTSTATE_BUS_W)
		return "bus write error";
	if (st & VPSS_INTSTATE_DCMP)
		return "tile decompress error";
	if (st & VPSS_INTSTATE_VHD_TUNL)
		return "output tunnel overrun";
	return "stride tunnel overrun";
}

/* Hand the chain head to the block and wait for the completion it releases. */
static int histb_vpss_kick(struct histb_vpss *vpss, dma_addr_t first)
{
	unsigned long deadline;
	u32 st;

	/*
	 * The state bits are latched: a completion left over from the previous
	 * job would end this one before the block had even started.
	 */
	writel(0xffffffff, vpss->regs + VPSS_INTCLR);
	writel(lower_32_bits(first), vpss->regs + VPSS_PNEXT);
	writel(1, vpss->regs + VPSS_START);

	deadline = jiffies + msecs_to_jiffies(200);
	do {
		st = readl(vpss->regs + VPSS_INTSTATE);
		/* A single-image chain may report either the node or the chain
		 * end; either means the frame is written.
		 */
		if (st & (VPSS_INTSTATE_EOF | VPSS_INTSTATE_DONE)) {
			writel(st, vpss->regs + VPSS_INTCLR);
			return 0;
		}
		/*
		 * A block that has said "no" does not change its mind by waiting,
		 * so report it now instead of spending the timeout on a poll. Only
		 * its own timeout word maps to ETIMEDOUT; the rest are faults.
		 */
		if (st & VPSS_INTSTATE_ERRORS) {
			writel(st, vpss->regs + VPSS_INTCLR);
			dev_err(vpss->dev, "job failed: intstate %#x (%s)\n", st,
				histb_vpss_err_name(st));
			if (st & VPSS_INTSTATE_TIMEOUT)
				return -ETIMEDOUT;
			return -EIO;
		}
		usleep_range(200, 400);
	} while (time_before(jiffies, deadline));

	dev_warn(vpss->dev, "job timed out, intstate %#x\n", st);
	writel(0xffffffff, vpss->regs + VPSS_INTCLR);
	if (histb_vpss_reset(vpss))
		return -EIO;
	return -ETIMEDOUT;
}

static void histb_vpss_pass_fill(struct histb_vpss_pass *p,
				 const struct histb_vpss_job *job)
{
	p->in = job->src;
	p->out = job->dst;
	p->in_w = job->in_w;
	p->in_h = job->in_h;
	p->off_x = job->in_x;
	p->off_y = job->in_y;
	p->frame_h = job->height;
	p->out_w = job->out_width;
	p->out_h = job->out_height;
	p->in_stride = job->src_stride;
	p->out_stride = job->dst_stride;
	p->in_422 = job->in_422;
	p->in_packed = job->in_packed;
	p->in_fmt = job->in_fmt;
	p->sharpness = job->sharpness;
	p->out_422 = job->out_422;
	p->invert_uv = job->invert_uv;
	p->hflip = job->hflip;
	p->vflip = job->vflip;
}

static int histb_vpss_run(struct histb_vpss *vpss,
			  const struct histb_vpss_job *job)
{
	struct histb_vpss_pass p = { 0 };

	histb_vpss_pass_fill(&p, job);
	return histb_vpss_pass_run(vpss, &p);
}

/*
 * The block's interpolation parameters are one register image of their own,
 * taken from the vendor's PQ defaults for this chip (SDK
 * msp/drv/pq/pq_v3_0/hal/hi3798mv100/pq_hal_table_default.h): the motion
 * adaptation window first, then the film-mode detector's thresholds. Each
 * word is written whole, so the comment names the fields it carries.
 */
static const struct {
	unsigned int	off;
	u32		val;
} histb_vpss_dei_defaults[] = {
	{ 0x1004, 0x00000092 },	/* rec-mode, IIR, motion smooth, mf-max */
	{ 0x1008, 0xfec00200 },	/* ver_min_inten, dir_inten_ver */
	{ 0x100c, 0x00000002 },	/* range_scale */
	{ 0x1010, 0x0008021e },	/* check1 gain, range gain, max range */
	{ 0x1014, 0x0008021e },	/* check2 gain, range gain, max range */
	{ 0x1018, 0x1b201828 },	/* direction multipliers 0..3 */
	{ 0x101c, 0x0b0c0f12 },	/* direction multipliers 4..7 */
	{ 0x1020, 0x06070809 },	/* direction multipliers 8..11 */
	{ 0x1024, 0x00030505 },	/* direction multipliers 12..14 */
	{ 0x1034, 0x87766556 },	/* interpolation scale ratios 1..8 */
	{ 0x1038, 0x08888888 },	/* interpolation scale ratios 9..15 */
	{ 0x103c, 0x13880420 },	/* strength/dir thresholds, bc gain */
	{ 0x1040, 0x00000000 },	/* jitter and field-motion coring */
	{ 0x1044, 0x083eff00 },	/* field motion gain, slope, thresholds */
	{ 0x1048, 0xffd09010 },	/* motion-difference thresholds 0..3 */
	{ 0x104c, 0x4020ffff },	/* IIR range and thresholds 4..5 */
	{ 0x1050, 0x20000081 },	/* motion IIR curve slopes and ratio 0 */
	{ 0x1054, 0x40404030 },	/* motion IIR curve ratios 1..4 */
	{ 0x1058, 0x000d0011 },	/* history mode, rec-mode steps */
	{ 0x105c, 0x01000000 },	/* median filter */
	{ 0x1060, 0xa00a0fff },	/* comb-check limits */
	{ 0x1064, 0x00001e40 },	/* comb-check thresholds */
	{ 0x1068, 0xffff4808 },	/* frame-motion smooth thresholds 0..3 */
	{ 0x106c, 0x4000ffff },	/* frame-motion smooth range 4..5 */
	{ 0x1070, 0x00000008 },	/* frame-motion smooth slopes, ratio 0 */
	{ 0x1074, 0x40404040 },	/* frame-motion smooth ratios 1..4 */
	{ 0x1078, 0xffff4808 },	/* frame/field blend thresholds 0..3 */
	{ 0x107c, 0x4000ffff },	/* frame/field blend range 4..5 */
	{ 0x1080, 0x00000008 },	/* frame/field blend slopes, ratio 0 */
	{ 0x1084, 0x40404040 },	/* frame/field blend ratios 1..4 */
	{ 0x1088, 0x00800080 },	/* motion-adjust gains */
	{ 0x108c, 0x00100010 },	/* global-motion thresholds 0..1 */
	{ 0x1090, 0x00100010 },	/* global-motion thresholds 2..3 */
	{ 0x1094, 0x40402004 },	/* chroma combing removal */
	{ 0x1098, 0x040118c0 },	/* chroma combing thresholds */
	{ 0x1144, 0x00201008 },	/* film-mode histogram thresholds 0..2 */
	{ 0x1148, 0x00143214 },	/* motion coring: block, pixel, window */
	{ 0x1150, 0x02000006 },	/* edge smoothing, motion-to-rec shift */
	{ 0x1154, 0x00001e00 },	/* moving-block and still-block thresholds */
	{ 0x1158, 0x00000000 },	/* monitoring start and threshold */
	{ 0x115c, 0x00402008 },	/* static-detection thresholds */
	{ 0x1160, 0x0010105a },	/* per-pixel coring: block, pixel, window */
	{ 0x1164, 0x00000010 },	/* per-pixel horizontal threshold */
	{ 0x1168, 0x80402010 },	/* per-pixel vertical thresholds 0..3 */
	{ 0x116c, 0x20100804 },	/* pixel-difference thresholds 0..3 */
	{ 0x1170, 0x00200f14 },	/* edge/texture thresholds */
	{ 0x1174, 0x00000020 },	/* texture alpha, texture coring */
	{ 0x1178, 0x00000000 },	/* texture thresholds 0..3 */
	{ 0x117c, 0x00000000 },	/* detection region start and end */
};

static void histb_vpss_dei_free(struct histb_vpss_ctx *ctx)
{
	if (ctx->dei_fields)
		dma_free_coherent(ctx->vpss->dev,
				  ctx->dei_frame_bytes * VPSS_DEI_RING,
				  ctx->dei_fields, ctx->dei_fields_dma);
	if (ctx->dei_hist)
		dma_free_coherent(ctx->vpss->dev,
				  VPSS_DEI_HIST_PLANE * VPSS_DEI_HIST_PLANES,
				  ctx->dei_hist, ctx->dei_hist_dma);
	ctx->dei_fields = NULL;
	ctx->dei_hist = NULL;
	ctx->dei_frame_bytes = 0;
	ctx->dei_count = 0;
	ctx->dei_hist_wr = 0;
}

static int histb_vpss_dei_alloc(struct histb_vpss_ctx *ctx, size_t frame_bytes)
{
	struct histb_vpss *vpss = ctx->vpss;
	u32 *hist;
	unsigned int i;

	if (ctx->dei_frame_bytes == frame_bytes && ctx->dei_hist)
		return 0;
	histb_vpss_dei_free(ctx);

	ctx->dei_fields = dma_alloc_coherent(vpss->dev,
					     frame_bytes * VPSS_DEI_RING,
					     &ctx->dei_fields_dma, GFP_KERNEL);
	if (!ctx->dei_fields)
		return -ENOMEM;
	ctx->dei_hist = dma_alloc_coherent(vpss->dev,
					   VPSS_DEI_HIST_PLANE *
					   VPSS_DEI_HIST_PLANES,
					   &ctx->dei_hist_dma, GFP_KERNEL);
	if (!ctx->dei_hist) {
		histb_vpss_dei_free(ctx);
		return -ENOMEM;
	}
	/* The vendor hands the block a history of "no motion" words. */
	hist = ctx->dei_hist;
	for (i = 0; i < VPSS_DEI_HIST_PLANE * VPSS_DEI_HIST_PLANES / 4; i++)
		hist[i] = 0x007f007f;
	ctx->dei_frame_bytes = frame_bytes;
	ctx->dei_count = 0;
	ctx->dei_hist_wr = 0;
	return 0;
}

/*
 * A ring slot is one picture of its moment: the field that just arrived goes
 * into the half it occupies, and the other half holds the field that came
 * with it. That is the layout the block is given - it is handed the picture
 * (one line past its start for the bottom field) and reads every other line.
 */
static void histb_vpss_dei_store(struct histb_vpss_ctx *ctx, const void *src,
				 unsigned int stride, unsigned int h,
				 unsigned int field, bool bottom, u64 ts)
{
	unsigned int half = bottom;
	unsigned int slot = (field / 2) % VPSS_DEI_RING;
	u8 *pic = ctx->dei_fields + (size_t)slot * ctx->dei_frame_bytes;
	u8 *y = pic + (size_t)half * stride;
	u8 *c = pic + (size_t)stride * 2 * h + (size_t)half * stride;
	const u8 *row = src;
	unsigned int i;

	for (i = 0; i < h; i++)
		memcpy(y + (size_t)2 * i * stride, row + (size_t)i * stride,
		       stride);
	row += (size_t)stride * h;
	for (i = 0; i < h / 2; i++)
		memcpy(c + (size_t)2 * i * stride, row + (size_t)i * stride,
		       stride);
	ctx->dei_ts[slot][half] = ts;
}

static dma_addr_t histb_vpss_dei_field(struct histb_vpss_ctx *ctx,
				       unsigned int field, bool bottom,
				       unsigned int stride)
{
	unsigned int slot = (field / 2) % VPSS_DEI_RING;

	return ctx->dei_fields_dma + (size_t)slot * ctx->dei_frame_bytes +
	       (bottom ? stride : 0);
}

/*
 * One motion-adaptive job. A job reads the field it produces, the one before
 * it and the two after it out of the ring, so its picture is three fields
 * behind the buffer that triggered it, and its timestamp says so; the delay
 * is what makes sure every picture a job reads is complete. Until the output
 * field has a neighbour on both sides the block has nothing to interpolate
 * against, so that frame leaves as the doubled copy bob already produces.
 */
static int histb_vpss_dei_run(struct histb_vpss *vpss,
			      struct histb_vpss_ctx *ctx,
			      const struct histb_vpss_job *job,
			      const void *src, u64 ts, bool bottom,
			      u64 *out_ts)
{
	unsigned int h = job->height;
	size_t frame = (size_t)job->src_stride * 3 * h;
	dma_addr_t f[VPSS_DEI_RING];
	dma_addr_t hist_w, hist_r;
	unsigned int n, k, i;
	bool cur_bottom;
	u32 *s = vpss->shadow;
	int ret;

	ret = histb_vpss_dei_alloc(ctx, frame);
	if (ret)
		return ret;

	n = ctx->dei_count;
	/*
	 * The block pairs a field with the one before it, so a stream whose
	 * fields do not alternate has no pairing to give it; start the ring
	 * over instead of pairing the current field with a stale picture.
	 */
	if (n && bottom == ctx->dei_last_bottom) {
		dev_warn_ratelimited(vpss->dev,
				     "fields do not alternate, starting over\n");
		ctx->dei_count = 0;
		n = 0;
	}
	ctx->dei_last_bottom = bottom;
	if (!n)
		ctx->dei_first_bottom = bottom;
	histb_vpss_dei_store(ctx, src, job->src_stride, h, n, bottom, ts);
	ctx->dei_count++;

	if (ctx->dei_count <= 4) {
		*out_ts = ts;
		return histb_vpss_run(vpss, job);
	}

	k = n - 3;
	cur_bottom = ctx->dei_first_bottom ^ (k & 1);
	for (i = 0; i < VPSS_DEI_RING; i++) {
		unsigned int field = k - 1 + i;

		f[i] = histb_vpss_dei_field(ctx, field,
					    ctx->dei_first_bottom ^
					    (field & 1),
					    job->src_stride);
	}
	*out_ts = ctx->dei_ts[(k / 2) % VPSS_DEI_RING][cur_bottom];

	if (!s)
		return -ENODEV;
	if (upper_32_bits(f[0]) || upper_32_bits(f[3]))
		return -ERANGE;

	memset(s, 0, VPSS_REG_IMAGE_SIZE);
	s[VPSS_MISCELLANEOUS / 4] = 0x03003244;
	s[VPSS_INTMASK / 4] = 0xff;
	s[VPSS_TIMEOUT / 4] = 0xffffffff;

	/* Field input: the block reads one half of each frame-sized picture,
	 * which half the field it produces is, and which half the stream
	 * starts with.
	 */
	s[VPSS_CTRL / 4] = VPSS_CTRL_VHD_EN |
			  FIELD_PREP(VPSS_CTRL_PROT, 2) |
			  VPSS_CTRL_DEI_EN | VPSS_CTRL_BFIELD |
			  (ctx->dei_first_bottom ? VPSS_CTRL_BFIELD_FIRST : 0) |
			  (cur_bottom ? VPSS_CTRL_BFIELD_MODE : 0);
	s[VPSS_CTRL2 / 4] = FIELD_PREP(VPSS_CTRL2_IN_FORMAT, 0) |
			   FIELD_PREP(VPSS_CTRL2_VHD_FORMAT, 0);
	s[VPSS_IMGSIZE / 4] = FIELD_PREP(VPSS_IMGSIZE_WIDTH, job->in_w - 1) |
			     FIELD_PREP(VPSS_IMGSIZE_HEIGHT, 2 * h - 1);

	s[VPSS_CURYADDR / 4] = lower_32_bits(f[1]);
	s[VPSS_CURCADDR / 4] = lower_32_bits(f[1] +
					     (size_t)job->src_stride * 2 * h);
	s[VPSS_CURSTRIDE / 4] =
		FIELD_PREP(VPSS_CURSTRIDE_Y, job->src_stride) |
		FIELD_PREP(VPSS_CURSTRIDE_C, job->src_stride);
	s[VPSS_REFYADDR / 4] = lower_32_bits(f[0]);
	s[VPSS_REFCADDR / 4] = lower_32_bits(f[0] +
					     (size_t)job->src_stride * 2 * h);
	s[VPSS_REFSTRIDE / 4] =
		FIELD_PREP(VPSS_FIELDSTRIDE_Y, job->src_stride) |
		FIELD_PREP(VPSS_FIELDSTRIDE_C, job->src_stride);
	s[VPSS_NXT1YADDR / 4] = lower_32_bits(f[2]);
	s[VPSS_NXT1CADDR / 4] = lower_32_bits(f[2] +
					      (size_t)job->src_stride * 2 * h);
	s[VPSS_NXT1STRIDE / 4] =
		FIELD_PREP(VPSS_FIELDSTRIDE_Y, job->src_stride) |
		FIELD_PREP(VPSS_FIELDSTRIDE_C, job->src_stride);
	s[VPSS_NXT2YADDR / 4] = lower_32_bits(f[3]);
	s[VPSS_NXT2CADDR / 4] = lower_32_bits(f[3] +
					      (size_t)job->src_stride * 2 * h);
	s[VPSS_NXT2STRIDE / 4] =
		FIELD_PREP(VPSS_FIELDSTRIDE_Y, job->src_stride) |
		FIELD_PREP(VPSS_FIELDSTRIDE_C, job->src_stride);

	hist_w = ctx->dei_hist_dma + ctx->dei_hist_wr * VPSS_DEI_HIST_PLANE;
	hist_r = ctx->dei_hist_dma +
		 ((ctx->dei_hist_wr + 2) % VPSS_DEI_HIST_PLANES) *
		 VPSS_DEI_HIST_PLANE;
	s[VPSS_STWADDR / 4] = lower_32_bits(hist_w);
	s[VPSS_STRADDR / 4] = lower_32_bits(hist_r);
	s[VPSS_STSTRIDE / 4] =
		FIELD_PREP(VPSS_STSTRIDE_VAL, VPSS_DEI_HIST_STRIDE);
	/* The parameter block starts at the control word itself
	 * (VPSS_REG_SIZE_CALC(VPSS_CTRL, VPSS_DIECTRL) - 4 in the vendor
	 * header), which is inside this register image.
	 */
	s[VPSS_DEI_ADDR / 4] = lower_32_bits(vpss->shadow_dma + VPSS_DIECTRL);
	s[VPSS_DIECTRL / 4] = FIELD_PREP(VPSS_DIECTRL_LMODE, 1) |
			     FIELD_PREP(VPSS_DIECTRL_CMODE, 1);
	for (i = 0; i < ARRAY_SIZE(histb_vpss_dei_defaults); i++)
		s[histb_vpss_dei_defaults[i].off / 4] =
			histb_vpss_dei_defaults[i].val;

	s[VPSS_VHDCTRL / 4] = job->invert_uv ? VPSS_VHDCTRL_UV_INVERT : 0;
	s[VPSS_VHDSIZE / 4] = FIELD_PREP(VPSS_VHDSIZE_WIDTH, job->out_width - 1) |
			     FIELD_PREP(VPSS_VHDSIZE_HEIGHT, 2 * h - 1);
	s[VPSS_VHDYADDR / 4] = lower_32_bits(job->dst);
	s[VPSS_VHDCADDR / 4] = lower_32_bits(job->dst +
					     (size_t)job->dst_stride * 2 * h);
	s[VPSS_VHDSTRIDE / 4] = FIELD_PREP(VPSS_VHDSTRIDE_Y, job->dst_stride) |
			       FIELD_PREP(VPSS_VHDSTRIDE_C, job->dst_stride);

	s[VPSS_VHDLBA_BK / 4] = FIELD_PREP(GENMASK(15, 8), 0x10) |
				FIELD_PREP(GENMASK(23, 16), 0x80) |
				FIELD_PREP(GENMASK(31, 24), 0x80);
	s[VPSS_VHDCROP_SIZE / 4] =
		VPSS_CROP_EN |
		FIELD_PREP(VPSS_VHDCROP_WIDTH, job->out_width - 1) |
		FIELD_PREP(VPSS_VHDCROP_HEIGHT, 2 * h - 1);

	s[VPSS_PNEXT / 4] = 0;
	s[VPSS_START / 4] = 1;
	/* The image has to be in memory before the block is handed its head. */
	wmb();

	ret = histb_vpss_kick(vpss, vpss->shadow_dma);
	if (!ret)
		ctx->dei_hist_wr = (ctx->dei_hist_wr + 2) % VPSS_DEI_HIST_PLANES;
	return ret;
}

static void histb_vpss_device_run(void *priv)
{
	struct histb_vpss_ctx *ctx = priv;
	struct histb_vpss *vpss = ctx->vpss;
	struct vb2_v4l2_buffer *src, *dst;
	struct histb_vpss_job job = { 0 };
	unsigned int dei_mode = VPSS_DEI_NONE;
	int ret = -EINVAL;
	bool powered = false;
	u64 out_ts = 0;

	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	if (!src || !dst)
		goto finish;
	out_ts = src->vb2_buf.timestamp;

	if (vpss->faulted) {
		ret = -EIO;
		goto finish;
	}

	/*
	 * v4l2_m2m_suspend() waits for the current context only, so a job can
	 * still be scheduled while remove() runs; refuse it rather than touch a
	 * block whose clock is on its way out.
	 */
	if (READ_ONCE(vpss->removing)) {
		ret = -ENODEV;
		goto finish;
	}

	/*
	 * Nothing here reaches the block before this: the job description is
	 * built in the shadow image, and the reset on the way back up is what
	 * makes the registers meaningful again.
	 */
	ret = pm_runtime_resume_and_get(vpss->dev);
	if (ret) {
		dev_warn_ratelimited(vpss->dev, "cannot clock the block: %d\n",
				     ret);
		goto finish;
	}
	powered = true;

	job.src = vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0);
	job.dst = vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0);
	job.width = ctx->src_fmt.width;
	job.height = ctx->src_fmt.height;
	job.out_width = ctx->dst_fmt.width;
	job.out_height = ctx->dst_fmt.height;
	job.src_stride = ctx->src_fmt.bytesperline;
	job.dst_stride = ctx->dst_fmt.bytesperline;
	job.sharpness = ctx->sharpness ? ctx->sharpness->val : 0;
	{
		const struct histb_vpss_fmt *src_fmt =
			histb_vpss_find(histb_vpss_out_fmts,
					ARRAY_SIZE(histb_vpss_out_fmts),
					ctx->src_fmt.pixelformat);
		const struct histb_vpss_fmt *dst_fmt =
			histb_vpss_find(histb_vpss_cap_fmts,
					ARRAY_SIZE(histb_vpss_cap_fmts),
					ctx->dst_fmt.pixelformat);

		job.in_422 = src_fmt->fmt422;
		job.in_packed = src_fmt->packed;
		job.in_fmt = histb_vpss_in_format(src_fmt);
		job.out_422 = dst_fmt->fmt422;
		job.invert_uv = src_fmt->nv21 != dst_fmt->nv21;
		/*
		 * A packed pair reaches the sampler in the opposite order to the
		 * semi-planar one: with exact luma a YUYV/UYVY frame comes out
		 * with U and V exchanged, and the output inversion is the pair
		 * swap the rest of the path already uses.
		 */
		if (src_fmt->packed)
			job.invert_uv = !job.invert_uv;
	}
	dei_mode = ctx->dei_ctrl ? ctx->dei_ctrl->val : VPSS_DEI_NONE;
	job.in_x = ctx->crop.left;
	job.in_y = ctx->crop.top;
	job.in_w = ctx->crop.width;
	job.in_h = ctx->crop.height;
	/* The offset semantics are known for the planar layouts; in a packed
	 * row a start offset moves the pair boundary as well, so a windowed
	 * packed frame is refused.
	 */
	if (job.in_packed && (job.in_x || job.in_y)) {
		dev_warn_ratelimited(vpss->dev,
				     "packed input cannot be windowed\n");
		ret = -EINVAL;
		goto finish;
	}
	/* Bob doubles one field vertically through the scaler, so it only
	 * exists when the buffer is a field and the CAPTURE rectangle is twice
	 * its height; asking for it any other way is a mistake worth a warning
	 * rather than a silent plain conversion. A source window would shrink
	 * the field it doubles, so a cropped bob is refused too.
	 */
	if (dei_mode == VPSS_DEI_BOB &&
	    (ctx->dst_fmt.height != job.height * 2 ||
	     job.in_x || job.in_y ||
	     job.in_w != job.width || job.in_h != job.height ||
	     (src->field != V4L2_FIELD_TOP &&
	      src->field != V4L2_FIELD_BOTTOM))) {
		dev_warn_ratelimited(vpss->dev,
				     "bob needs a field and twice its height\n");
		ret = -EINVAL;
		goto finish;
	}
	job.hflip = ctx->hflip && ctx->hflip->val;
	job.vflip = ctx->vflip && ctx->vflip->val;
	job.angle = ctx->rotate ? ctx->rotate->val : 0;

	if (dei_mode == VPSS_DEI_MAD) {
		const void *vaddr = vb2_plane_vaddr(&src->vb2_buf, 0);

		/*
		 * The interpolation path reads 4:2:0 fields only, from memory
		 * the driver can copy, and it owns the whole conversion: no
		 * window, scaler, sharpen, flip or turn rides on top of it.
		 */
		if (job.in_422 || job.out_422 || job.in_packed ||
		    job.in_x || job.in_y ||
		    job.in_w != job.width || job.in_h != job.height ||
		    job.out_width != job.width ||
		    job.out_height != job.height * 2 ||
		    job.sharpness || job.hflip || job.vflip || job.angle ||
		    !vaddr ||
		    (src->field != V4L2_FIELD_TOP &&
		     src->field != V4L2_FIELD_BOTTOM)) {
			dev_warn_ratelimited(vpss->dev,
					     "motion-adaptive wants one 4:2:0 field and nothing "
					     "else\n");
			ret = -EINVAL;
			goto finish;
		}
		ret = histb_vpss_dei_run(vpss, ctx, &job, vaddr,
					 src->vb2_buf.timestamp,
					 src->field == V4L2_FIELD_BOTTOM,
					 &out_ts);
	} else if (job.angle == 180) {
		/* Half a turn is the port's mirror plus flip. Both are order
		 * reversals on independent axes, so they commute and a client's
		 * flip cancels the matching one here rather than being dropped.
		 */
		job.hflip = !job.hflip;
		job.vflip = !job.vflip;
		ret = histb_vpss_run(vpss, &job);
	} else if (job.angle) {
		struct histb_vpss_pass p = { 0 };

		histb_vpss_pass_fill(&p, &job);
		/* Quarter turns go through the block's rotate chain, which
		 * programs no crop, no scaler and no sharpening; a job asking for
		 * those alongside rotation is refused rather than quietly
		 * delivered without them. The port's mirror and flip do not
		 * compose with this chain either: an address-encoded frame showed
		 * the flip leaving the map untouched and the mirror moving the read
		 * window off the frame, and a 4:2:2 quarter turn writes only the
		 * first half of the chroma plane. Swapping the chroma order is inert
		 * here too (an NV12 request came back byte-identical to the NV21
		 * one), so all of it is refused.
		 */
		if (job.in_x || job.in_y || job.sharpness || job.hflip ||
		    job.vflip || job.invert_uv || job.in_422 || job.out_422 ||
		    job.in_w != job.out_height || job.in_h != job.out_width) {
			dev_warn_ratelimited(vpss->dev,
					     "rotation wants swapped 4:2:0 sizes and nothing "
					     "else\n");
			ret = -EINVAL;
			goto finish;
		}
		ret = histb_vpss_rotate_run(vpss, &p, job.angle);
	} else {
		ret = histb_vpss_run(vpss, &job);
	}
	if (ret)
		dev_warn_ratelimited(vpss->dev, "job failed: %d\n", ret);

finish:
	if (powered)
		pm_runtime_put_autosuspend(vpss->dev);
	if (src) {
		src = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		v4l2_m2m_buf_done(src, ret ? VB2_BUF_STATE_ERROR :
						 VB2_BUF_STATE_DONE);
	}
	if (dst) {
		dst = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (!ret) {
			vb2_set_plane_payload(&dst->vb2_buf, 0,
					      ctx->dst_fmt.sizeimage);
			if (src)
				dst->vb2_buf.timestamp = out_ts;
		}
		v4l2_m2m_buf_done(dst, ret ? VB2_BUF_STATE_ERROR :
						 VB2_BUF_STATE_DONE);
	}
	v4l2_m2m_job_finish(vpss->m2m_dev, ctx->fh.m2m_ctx);
}

static const struct v4l2_m2m_ops histb_vpss_m2m_ops = {
	.device_run = histb_vpss_device_run,
};

static int histb_vpss_queue_init(void *priv, struct vb2_queue *src_vq,
				 struct vb2_queue *dst_vq)
{
	struct histb_vpss_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->ops = &histb_vpss_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->vpss->lock;
	src_vq->dev = ctx->vpss->dev;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->ops = &histb_vpss_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->vpss->lock;
	dst_vq->dev = ctx->vpss->dev;

	return vb2_queue_init(dst_vq);
}

static const char *const histb_vpss_dei_menu[] = {
	"Off", "Bob", "Weave", "Motion adaptive", NULL,
};

static int histb_vpss_ctrls_create(struct histb_vpss_ctx *ctx)
{
	struct v4l2_ctrl_handler *hdl = &ctx->ctrl_handler;

	v4l2_ctrl_handler_init(hdl, 5);
	ctx->hflip = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_HFLIP, 0, 1, 1, 0);
	ctx->vflip = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_VFLIP, 0, 1, 1, 0);
	ctx->sharpness = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_SHARPNESS, 0, 255,
					   1, 0);
	ctx->rotate = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_ROTATE, 0, 270, 90, 0);
	ctx->dei_ctrl = v4l2_ctrl_new_std_menu_items(hdl, NULL,
						     V4L2_CID_DEINTERLACING_MODE,
						     VPSS_DEI_MAD, 0,
						     VPSS_DEI_NONE,
						     histb_vpss_dei_menu);
	if (hdl->error)
		return hdl->error;

	ctx->fh.ctrl_handler = hdl;
	return 0;
}

static int histb_vpss_open(struct file *file)
{
	struct histb_vpss *vpss = video_drvdata(file);
	struct histb_vpss_ctx *ctx;
	int ret;

	ctx = kzalloc_obj(*ctx);
	if (!ctx)
		return -ENOMEM;
	ctx->vpss = vpss;
	v4l2_fh_init(&ctx->fh, &vpss->vdev);
	file->private_data = &ctx->fh;

	ctx->src_fmt.width = 320;
	ctx->src_fmt.height = 240;
	/* The defaults carry no client bytesperline, so the stride bound
	 * above cannot trip here.
	 */
	histb_vpss_fill_fmt(&histb_vpss_out_fmts[0], &ctx->src_fmt);
	ctx->dst_fmt = ctx->src_fmt;
	histb_vpss_fill_fmt(&histb_vpss_cap_fmts[0], &ctx->dst_fmt);
	histb_vpss_reset_crop(ctx);

	ret = histb_vpss_ctrls_create(ctx);
	if (ret) {
		v4l2_fh_exit(&ctx->fh);
		kfree(ctx);
		return ret;
	}

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(vpss->m2m_dev, ctx,
					    histb_vpss_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		v4l2_ctrl_handler_free(&ctx->ctrl_handler);
		v4l2_fh_exit(&ctx->fh);
		kfree(ctx);
		return ret;
	}
	v4l2_fh_add(&ctx->fh, file);
	return 0;
}

static int histb_vpss_release(struct file *file)
{
	struct histb_vpss_ctx *ctx = container_of(file->private_data,
						  typeof(*ctx), fh);

	mutex_lock(&ctx->vpss->lock);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	histb_vpss_dei_free(ctx);
	mutex_unlock(&ctx->vpss->lock);
	kfree(ctx);
	return 0;
}

static const struct v4l2_file_operations histb_vpss_fops = {
	.owner = THIS_MODULE,
	.open = histb_vpss_open,
	.release = histb_vpss_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

static const struct v4l2_ioctl_ops histb_vpss_ioctl_ops = {
	.vidioc_querycap = histb_vpss_querycap,
	.vidioc_enum_fmt_vid_cap = histb_vpss_enum_fmt,
	.vidioc_enum_fmt_vid_out = histb_vpss_enum_fmt,
	.vidioc_g_fmt_vid_cap = histb_vpss_g_fmt,
	.vidioc_g_fmt_vid_out = histb_vpss_g_fmt,
	.vidioc_try_fmt_vid_cap = histb_vpss_try_fmt,
	.vidioc_try_fmt_vid_out = histb_vpss_try_fmt,
	.vidioc_s_fmt_vid_cap = histb_vpss_s_fmt,
	.vidioc_s_fmt_vid_out = histb_vpss_s_fmt,
	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
	.vidioc_g_selection = histb_vpss_g_selection,
	.vidioc_s_selection = histb_vpss_s_selection,
};

static const struct video_device histb_vpss_videodev = {
	.name = HISTB_VPSS_NAME,
	.vfl_dir = VFL_DIR_M2M,
	.fops = &histb_vpss_fops,
	.ioctl_ops = &histb_vpss_ioctl_ops,
	.minor = -1,
	.release = video_device_release_empty,
	.device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING,
};

/*
 * The block has no reset-done bit (VPSS_START only says that a job is running),
 * so the completion check is that its constant version word answers again: a
 * window that no longer decodes reads 0, and a reset that the CRG refused leaves
 * the block unclocked. A failed reset is latched rather than retried blind --
 * every later job would otherwise burn the whole 200 ms timeout on a block that
 * cannot answer, and the application would only see ETIMEDOUT.
 */
static int histb_vpss_reset(struct histb_vpss *vpss)
{
	unsigned long deadline;
	int ret;

	ret = reset_control_assert(vpss->rst);
	if (ret)
		goto faulted;
	udelay(1);
	ret = reset_control_deassert(vpss->rst);
	if (ret)
		goto faulted;

	if (!vpss->version)	/* first reset: nothing to compare against yet */
		return 0;

	deadline = jiffies + msecs_to_jiffies(20);
	do {
		if (readl(vpss->regs + VPSS_VERSION) == vpss->version)
			return 0;
		usleep_range(200, 400);
	} while (time_before(jiffies, deadline));

	ret = -EIO;
faulted:
	dev_err(vpss->dev, "reset failed (%d); VPSS stays unusable\n", ret);
	vpss->faulted = true;
	return ret;
}

static int histb_vpss_runtime_suspend(struct device *dev)
{
	struct histb_vpss *vpss = dev_get_drvdata(dev);

	clk_disable_unprepare(vpss->clk);
	return 0;
}

static int histb_vpss_runtime_resume(struct device *dev)
{
	struct histb_vpss *vpss = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(vpss->clk);
	if (ret)
		return ret;
	/*
	 * Coming back means the block lost its clock, so start from a known
	 * state instead of trusting registers that are now whatever the power
	 * domain left. The job itself re-programs everything it needs.
	 */
	return histb_vpss_reset(vpss);
}

static const struct dev_pm_ops histb_vpss_pm_ops = {
	RUNTIME_PM_OPS(histb_vpss_runtime_suspend, histb_vpss_runtime_resume,
		       NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

static void histb_vpss_v4l2_release(struct v4l2_device *v4l2_dev)
{
	struct histb_vpss *vpss = container_of(v4l2_dev, struct histb_vpss, v4l2_dev);

	v4l2_device_unregister(v4l2_dev);
	if (vpss->m2m_dev)
		v4l2_m2m_release(vpss->m2m_dev);
	kfree(vpss);
}

static void histb_vpss_v4l2_put(void *data)
{
	struct histb_vpss *vpss = data;

	v4l2_device_put(&vpss->v4l2_dev);
}

static int histb_vpss_probe(struct platform_device *pdev)
{
	struct histb_vpss *vpss;
	int ret;

	vpss = kzalloc_obj(*vpss);
	if (!vpss)
		return -ENOMEM;

	ret = v4l2_device_register(&pdev->dev, &vpss->v4l2_dev);
	if (ret) {
		kfree(vpss);
		return ret;
	}
	vpss->v4l2_dev.release = histb_vpss_v4l2_release;
	ret = devm_add_action_or_reset(&pdev->dev, histb_vpss_v4l2_put, vpss);
	if (ret)
		return ret;

	vpss->dev = &pdev->dev;
	mutex_init(&vpss->lock);
	init_completion(&vpss->done);

	vpss->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(vpss->regs))
		return PTR_ERR(vpss->regs);

	vpss->clk = devm_clk_get(&pdev->dev, "vpss");
	if (IS_ERR(vpss->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(vpss->clk),
				     "no vpss clock\n");
	vpss->rst = devm_reset_control_get_exclusive(&pdev->dev, "vpss");
	if (IS_ERR(vpss->rst))
		return dev_err_probe(&pdev->dev, PTR_ERR(vpss->rst),
				     "no vpss reset\n");

	vpss->shadow = dma_alloc_coherent(&pdev->dev,
					  VPSS_REG_IMAGE_SIZE * HISTB_VPSS_IMAGES,
					  &vpss->shadow_dma, GFP_KERNEL);
	if (!vpss->shadow)
		return -ENOMEM;
	vpss->coef = dma_alloc_coherent(&pdev->dev, VPSS_COEF_BYTES,
					&vpss->coef_dma, GFP_KERNEL);
	if (!vpss->coef) {
		ret = -ENOMEM;
		goto err_shadow;
	}

	/* The PM callbacks reach the device through drvdata, so publish it
	 * before the first runtime resume.
	 */
	platform_set_drvdata(pdev, vpss);

	pm_runtime_enable(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev,
					 HISTB_VPSS_AUTOSUSPEND_MS);
	pm_runtime_use_autosuspend(&pdev->dev);

	ret = pm_runtime_resume_and_get(&pdev->dev);
	if (ret) {
		dev_err_probe(&pdev->dev, ret, "cannot bring the block up\n");
		goto err_pm_disable;
	}
	/* Remember what the block answers with, so a later reset can tell
	 * "back up" from "the window stopped decoding".
	 */
	vpss->version = readl(vpss->regs + VPSS_VERSION);
	if (!vpss->version) {
		dev_err(&pdev->dev, "block does not answer (VPSS_VERSION reads 0)\n");
		ret = -ENODEV;
		goto err_pm;
	}

	vpss->m2m_dev = v4l2_m2m_init(&histb_vpss_m2m_ops);
	if (IS_ERR(vpss->m2m_dev)) {
		ret = PTR_ERR(vpss->m2m_dev);
		vpss->m2m_dev = NULL;
		goto err_pm;
	}

	vpss->vdev = histb_vpss_videodev;
	vpss->vdev.v4l2_dev = &vpss->v4l2_dev;
	vpss->vdev.lock = &vpss->lock;
	video_set_drvdata(&vpss->vdev, vpss);

	ret = video_register_device(&vpss->vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(&pdev->dev, "cannot register video device\n");
		goto err_pm;
	}

	dev_info(&pdev->dev, "video processor, %d x %d max\n",
		 VPSS_MAX_WIDTH, VPSS_MAX_HEIGHT);
	pm_runtime_put_autosuspend(&pdev->dev);
	return 0;

err_pm:
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_dont_use_autosuspend(&pdev->dev);
err_pm_disable:
	pm_runtime_disable(&pdev->dev);
	dma_free_coherent(vpss->dev, VPSS_COEF_BYTES, vpss->coef,
			  vpss->coef_dma);
err_shadow:
	dma_free_coherent(vpss->dev, VPSS_REG_IMAGE_SIZE * HISTB_VPSS_IMAGES,
			  vpss->shadow, vpss->shadow_dma);
	return ret;
}

static void histb_vpss_remove(struct platform_device *pdev)
{
	struct histb_vpss *vpss = platform_get_drvdata(pdev);

	/*
	 * v4l2_m2m_release() is only a kfree() in this tree, so nothing else here
	 * waits for a running job; pause the queue and let it finish first, or
	 * the shadow image freed below is still a DMA target for that job.
	 * This block reports through a polled status word, so there is no
	 * interrupt to mask.
	 */
	mutex_lock(&vpss->lock);
	WRITE_ONCE(vpss->removing, true);
	v4l2_m2m_suspend(vpss->m2m_dev);
	video_unregister_device(&vpss->vdev);
	mutex_unlock(&vpss->lock);
	/* Probe already dropped its own runtime reference, so force the idle
	 * state instead of putting one that is not held.
	 */
	pm_runtime_suspend(&pdev->dev);
	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	dma_free_coherent(vpss->dev, VPSS_REG_IMAGE_SIZE * HISTB_VPSS_IMAGES,
			  vpss->shadow, vpss->shadow_dma);
	dma_free_coherent(vpss->dev, VPSS_COEF_BYTES, vpss->coef,
			  vpss->coef_dma);
}

static const struct of_device_id histb_vpss_of_match[] = {
	{ .compatible = "hisilicon,hi3798mv100-vpss" },
	{ }
};
MODULE_DEVICE_TABLE(of, histb_vpss_of_match);

static struct platform_driver histb_vpss_driver = {
	.probe = histb_vpss_probe,
	.remove = histb_vpss_remove,
	.driver = {
		.name = HISTB_VPSS_NAME,
		.pm = pm_ptr(&histb_vpss_pm_ops),
		.of_match_table = histb_vpss_of_match,
	},
};
module_platform_driver(histb_vpss_driver);

MODULE_DESCRIPTION("HiSilicon HiSTB video processing subsystem driver");
MODULE_LICENSE("Dual MIT/GPL");
