// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon HiSTB V500 encoder: the V4L2 mem2mem face.
 *
 * Raw pictures go into the OUTPUT queue and H.264 or HEVC network abstraction
 * layer data comes out of the CAPTURE queue, so this is a stateful encoder: the
 * client hands over a picture and the coding parameters the controls express,
 * not slice headers or reference lists.
 *
 * The group of pictures is closed: the first picture of each group is an IDR
 * and the rest predict from it, and the rate loop moves the quantiser from the
 * bits the previous picture spent.
 *
 * The core serves one context at a time. Its vendor layer keeps state in two
 * globals and cannot be re-entered, and the reconstruction buffers, the command
 * image and the coded-stream ring here are one set per device, so a second
 * context is refused when it starts streaming rather than queued and silently
 * serialised.
 */

#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/videodev2.h>

#include <media/media-device.h>
#include <media/v4l2-ioctl.h>
#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>

#include "histb_venc_v5.h"
#include "histb-venc.h"

#define V500_TIMEOUT			msecs_to_jiffies(1000)
#define V500_MAX_OUTPUT_BUFFERS		8
#define V500_MAX_CAPTURE_BUFFERS	16

static const u32 histb_venc_v5_ctrls[] = {
	V4L2_CID_MPEG_VIDEO_BITRATE_MODE,
	V4L2_CID_MPEG_VIDEO_BITRATE,
	V4L2_CID_MPEG_VIDEO_H264_PROFILE,
	V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE,
	V4L2_CID_MPEG_VIDEO_H264_MIN_QP,
	V4L2_CID_MPEG_VIDEO_H264_MAX_QP,
	V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP,
	V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP,
	V4L2_CID_MPEG_VIDEO_HEVC_PROFILE,
	V4L2_CID_MPEG_VIDEO_HEVC_LEVEL,
	V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP,
	V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP,
	V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP,
	V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_QP,
	V4L2_CID_MPEG_VIDEO_GOP_SIZE,
	V4L2_CID_MPEG_VIDEO_HEADER_MODE,
	V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE,
	V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB,
	V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME,
};

static struct histb_venc_v5_ctx *file_to_ctx(struct file *filp)
{
	return container_of(file_to_v4l2_fh(filp), struct histb_venc_v5_ctx, fh);
}

/* --------------------------------------------------------------- formats */

/*
 * The input layouts this core stores pictures in. The two 4:2:2 ones are not
 * coded as 4:2:2: the image carries no chroma sampling ratio, so the core reads
 * the interleaved chroma plane at twice its own row pitch and therefore takes
 * every other row, which is the vendor's own treatment of these formats
 * (drv_venc_efl.c:4585-4589, whose comment says it out loud).
 */
#define V500_IN_NV12		0
#define V500_IN_YUV420		1
#define V500_IN_YUYV		2
#define V500_IN_NV21		3
#define V500_IN_NV16		4
#define V500_IN_NV61		5
#define V500_IN_UYVY		6

/*
 * Whether a fourcc names one of the layouts this core can be handed. The node
 * speaks only what ENUM_FMT lists: a picture in another layout is refused
 * rather than read as NV12, which would take its chroma from the wrong place.
 */
static bool histb_venc_v5_is_input(unsigned int fourcc)
{
	switch (fourcc) {
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21M:
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_YUV420M:
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_NV16M:
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_NV61M:
	case V4L2_PIX_FMT_NV61:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
		return true;
	default:
		return false;
	}
}

/*
 * Both the single-plane and the multi-plane spelling of a layout name the same
 * arrangement, and the node only ever speaks multi-plane.
 */
static unsigned int histb_venc_v5_input(unsigned int fourcc)
{
	switch (fourcc) {
	case V4L2_PIX_FMT_YUV420M:
	case V4L2_PIX_FMT_YUV420:
		return V500_IN_YUV420;
	case V4L2_PIX_FMT_YUYV:
		return V500_IN_YUYV;
	case V4L2_PIX_FMT_UYVY:
		return V500_IN_UYVY;
	case V4L2_PIX_FMT_NV21M:
	case V4L2_PIX_FMT_NV21:
		return V500_IN_NV21;
	case V4L2_PIX_FMT_NV16M:
	case V4L2_PIX_FMT_NV16:
		return V500_IN_NV16;
	case V4L2_PIX_FMT_NV61M:
	case V4L2_PIX_FMT_NV61:
		return V500_IN_NV61;
	default:
		return V500_IN_NV12;
	}
}

static unsigned int histb_venc_v5_input_fourcc(unsigned int input)
{
	switch (input) {
	case V500_IN_YUV420:
		return V4L2_PIX_FMT_YUV420M;
	case V500_IN_YUYV:
		return V4L2_PIX_FMT_YUYV;
	case V500_IN_UYVY:
		return V4L2_PIX_FMT_UYVY;
	case V500_IN_NV21:
		return V4L2_PIX_FMT_NV21M;
	case V500_IN_NV16:
		return V4L2_PIX_FMT_NV16M;
	case V500_IN_NV61:
		return V4L2_PIX_FMT_NV61M;
	default:
		return V4L2_PIX_FMT_NV12M;
	}
}

/* A packed layout is one plane of two bytes per luma sample and 4:2:2. */
static bool histb_venc_v5_is_packed(unsigned int input)
{
	return input == V500_IN_YUYV || input == V500_IN_UYVY;
}

/* Semi-planar 4:2:2: a chroma plane as tall as the luma one. */
static bool histb_venc_v5_is_semi422(unsigned int input)
{
	return input == V500_IN_NV16 || input == V500_IN_NV61;
}

/* The pitch a source plane of the negotiated picture is walked with. */
static unsigned int histb_venc_v5_src_stride(const struct histb_venc_v5_ctx *ctx)
{
	return ALIGN(ctx->src_w, 16);
}

/*
 * The rectangle the core codes: the selected one, or the whole picture when
 * nothing is selected. Selection is stored already aligned to the core's block
 * grid, so this is exact and the parameter set only trims the padding.
 */
static void histb_venc_v5_coded_dims(const struct histb_venc_v5_ctx *ctx,
				     unsigned int *width, unsigned int *height)
{
	if (ctx->crop.width && ctx->crop.height) {
		*width = ctx->crop.width;
		*height = ctx->crop.height;
	} else {
		*width = ctx->src_w;
		*height = ctx->src_h;
	}
}

/* The frame descriptor codes the selected rectangle, not the whole picture. */
static void histb_venc_v5_apply_geometry(struct histb_venc_v5_ctx *ctx)
{
	histb_venc_v5_coded_dims(ctx, &ctx->frame.width, &ctx->frame.height);
}

static int histb_venc_v5_try_fmt(struct histb_venc_v5_ctx *ctx,
				 struct v4l2_format *fmt)
{
	struct v4l2_pix_format_mplane *pix = &fmt->fmt.pix_mp;
	unsigned int width = ALIGN_DOWN(clamp(pix->width, 128u, 1920U), 2);
	unsigned int height = ALIGN_DOWN(clamp(pix->height, 128u, 1088U), 2);
	unsigned int stride = ALIGN(width, 16);
	unsigned int coded = ALIGN(height, 32);
	unsigned int chroma_stride;
	unsigned int input = V500_IN_NV12;

	pix->width = width;
	pix->height = height;
	if (fmt->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		if (!histb_venc_v5_is_input(pix->pixelformat))
			return -EINVAL;
		input = histb_venc_v5_input(pix->pixelformat);
		pix->pixelformat = histb_venc_v5_input_fourcc(input);
	} else if (pix->pixelformat != V4L2_PIX_FMT_HEVC) {
		pix->pixelformat = V4L2_PIX_FMT_H264;
	}
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = V4L2_COLORSPACE_SMPTE170M;
	pix->plane_fmt[0].bytesperline = 0;

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		pix->num_planes = 1;
		/*
		 * The ring is sized for an all-intra picture, but a client
		 * may ask for more room; the queue reports whichever is
		 * larger and the drain refuses a frame that does not fit.
		 */
		if (!pix->plane_fmt[0].sizeimage ||
		    pix->plane_fmt[0].sizeimage < width * height * 2)
			pix->plane_fmt[0].sizeimage = width * height * 2;
		return 0;
	}

	/*
	 * The core works on whole blocks, so the planes are reported as the
	 * coded picture rather than the visible one: the rows past the bottom
	 * of the picture are read, and a buffer that stops at the visible edge
	 * would leave those reads outside the allocation.
	 */
	chroma_stride = ALIGN(DIV_ROUND_UP(width, 2), 16);
	switch (input) {
	case V500_IN_YUV420:
		pix->num_planes = 3;
		pix->plane_fmt[0].bytesperline = stride;
		pix->plane_fmt[0].sizeimage = stride * coded;
		pix->plane_fmt[1].bytesperline = chroma_stride;
		pix->plane_fmt[1].sizeimage = chroma_stride * coded / 2;
		pix->plane_fmt[2].bytesperline = chroma_stride;
		pix->plane_fmt[2].sizeimage = chroma_stride * coded / 2;
		return 0;
	case V500_IN_YUYV:
	case V500_IN_UYVY:
		pix->num_planes = 1;
		pix->plane_fmt[0].bytesperline = ALIGN(width * 2, 16);
		pix->plane_fmt[0].sizeimage = ALIGN(width * 2, 16) * coded;
		return 0;
	default:
		break;
	}

	/*
	 * Semi-planar: one luma plane and one plane of interleaved chroma
	 * samples, the core reading both with the same pitch in bytes. A 4:2:2
	 * picture's chroma plane carries a row per luma row rather than one per
	 * pair, so it is exactly twice the size.
	 */
	pix->num_planes = 2;
	pix->plane_fmt[0].bytesperline = stride;
	pix->plane_fmt[0].sizeimage = stride * coded;
	pix->plane_fmt[1].bytesperline = stride;
	pix->plane_fmt[1].sizeimage = stride * coded /
		(histb_venc_v5_is_semi422(input) ? 1 : 2);

	return 0;
}

/*
 * The capture queue's format is the coded window, so it follows the output
 * picture whenever the queue is free to be set again. What the queue owns is
 * which syntax to code, and that carries over.
 */
static void histb_venc_v5_reset_cap_fmt(struct histb_venc_v5_ctx *ctx)
{
	struct v4l2_format *cap = &ctx->fmt[V500_FMT_CAPTURE];
	unsigned int fourcc = cap->fmt.pix_mp.pixelformat;

	memset(cap, 0, sizeof(*cap));
	cap->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	cap->fmt.pix_mp.pixelformat = fourcc;
	histb_venc_v5_coded_dims(ctx, &cap->fmt.pix_mp.width,
				 &cap->fmt.pix_mp.height);
	histb_venc_v5_try_fmt(ctx, cap);
}

static int histb_venc_v5_querycap(struct file *file, void *priv,
				struct v4l2_capability *cap)
{
	strscpy(cap->driver, HISTB_VENC_NAME, sizeof(cap->driver));
	strscpy(cap->card, "histb-venc", sizeof(cap->card));

	return 0;
}

static int histb_venc_v5_enum_fmt(struct file *file, void *priv,
				struct v4l2_fmtdesc *f)
{
	static const u32 inputs[] = {
		V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV21M, V4L2_PIX_FMT_YUV420M,
		V4L2_PIX_FMT_YUYV, V4L2_PIX_FMT_UYVY, V4L2_PIX_FMT_NV16M,
		V4L2_PIX_FMT_NV61M,
	};

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		if (f->index >= ARRAY_SIZE(inputs))
			return -EINVAL;
		f->pixelformat = inputs[f->index];
		return 0;
	}

	if (f->index > 1)
		return -EINVAL;
	f->pixelformat = f->index ? V4L2_PIX_FMT_HEVC : V4L2_PIX_FMT_H264;

	return 0;
}

static int histb_venc_v5_g_fmt(struct file *file, void *priv,
			     struct v4l2_format *f)
{
	struct histb_venc_v5_ctx *ctx = file_to_ctx(file);

	*f = ctx->fmt[V4L2_TYPE_IS_OUTPUT(f->type)];

	return 0;
}

static int histb_venc_v5_s_fmt(struct file *file, void *priv,
				     struct v4l2_format *f)
{
	struct histb_venc_v5_ctx *ctx = file_to_ctx(file);
	struct vb2_queue *q = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	struct vb2_queue *peer;
	u32 sizeimage;
	int ret;

	if (vb2_is_busy(q))
		return -EBUSY;

	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		ret = histb_venc_v5_try_fmt(ctx, f);
		if (ret)
			return ret;
		ctx->fmt[V500_FMT_OUTPUT] = *f;
		/* A selection is a rectangle in the picture it was set
		 * against; a new picture retires it.
		 */
		ctx->crop = (struct v4l2_rect){ 0 };
		ctx->src_w = f->fmt.pix_mp.width;
		ctx->src_h = f->fmt.pix_mp.height;
		histb_venc_v5_apply_geometry(ctx);

		/* The coded window follows the picture while it is free. */
		peer = v4l2_m2m_get_vq(ctx->fh.m2m_ctx,
				       V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
		if (!vb2_is_busy(peer))
			histb_venc_v5_reset_cap_fmt(ctx);
		return 0;
	}

	/*
	 * Only the coded window is the capture format's own; the geometry is
	 * the picture's and is changed through the output queue alone.
	 */
	sizeimage = f->fmt.pix_mp.plane_fmt[0].sizeimage;
	f->fmt.pix_mp.width = ctx->frame.width;
	f->fmt.pix_mp.height = ctx->frame.height;
	ret = histb_venc_v5_try_fmt(ctx, f);
	if (ret)
		return ret;
	if (sizeimage > f->fmt.pix_mp.plane_fmt[0].sizeimage)
		f->fmt.pix_mp.plane_fmt[0].sizeimage = sizeimage;
	ctx->hevc = f->fmt.pix_mp.pixelformat == V4L2_PIX_FMT_HEVC;
	ctx->fmt[V500_FMT_CAPTURE] = *f;

	return 0;
}

static int histb_venc_v5_try_fmt_vidioc(struct file *file, void *priv,
				      struct v4l2_format *f)
{
	return histb_venc_v5_try_fmt(file_to_ctx(file), f);
}

/*
 * The selected rectangle is the part of the source picture the core codes. It
 * belongs to the OUTPUT queue, whose buffers carry the whole picture, and the
 * core has no crop register: the origin is the address the planes are read
 * from and the extent the size the planes are walked for, so both are the
 * driver's to compute.
 */
static int histb_venc_v5_g_selection(struct file *file, void *priv,
				     struct v4l2_selection *s)
{
	struct histb_venc_v5_ctx *ctx = file_to_ctx(file);
	struct v4l2_rect full = { 0, 0, ctx->src_w, ctx->src_h };

	/* The core hands selection ioctls the plain buffer type, whatever the
	 * client asked for (v4l_s_selection), so this queue is named that way.
	 */
	if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		s->r = (ctx->crop.width && ctx->crop.height) ? ctx->crop : full;
		break;
	case V4L2_SEL_TGT_CROP_BOUNDS:
		s->r = full;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int histb_venc_v5_s_selection(struct file *file, void *priv,
				     struct v4l2_selection *s)
{
	struct histb_venc_v5_ctx *ctx = file_to_ctx(file);
	unsigned int align = V500_BLOCK_ALIGN;
	struct vb2_queue *vq;
	struct v4l2_rect r = s->r;

	if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT ||
	    s->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;
	if (!ctx->src_w || !ctx->src_h)
		return -EINVAL;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, s->type);
	if (!vq)
		return -EINVAL;
	if (vb2_is_busy(vq))
		return -EBUSY;

	/*
	 * The core walks the source in whole macroblocks, so the origin is
	 * macroblock aligned and the extent rounded down to one; the parameter
	 * set still trims the picture when the extent is not a whole number of
	 * them. A planar chroma plane is addressed at half the luma rate, so its
	 * horizontal origin has to stay a whole 16 bytes up too, which is the
	 * alignment the reference asks of every source address
	 * (drv_venc_efl.c:3761-3779).
	 */
	if (histb_venc_v5_input(ctx->fmt[V500_FMT_OUTPUT].fmt.pix_mp.pixelformat)
	    == V500_IN_YUV420)
		align = 2 * V500_BLOCK_ALIGN;
	if (r.left < 0 || r.top < 0)
		return -EINVAL;
	r.left = round_down(r.left, align);
	r.top = round_down(r.top, V500_BLOCK_ALIGN);
	r.width = round_down(r.width, V500_BLOCK_ALIGN);
	r.height = round_down(r.height, V500_BLOCK_ALIGN);
	if (r.width < (ctx->hevc ? 128u : 256u) || r.height < 128 ||
	    r.left + r.width > ctx->src_w || r.top + r.height > ctx->src_h)
		return -EINVAL;
	/*
	 * The core reads whole padded rows from wherever it was pointed, so a
	 * rectangle low in the picture has to leave those rows inside the buffer
	 * the queue was allocated with. The buffer is padded to 32, which covers
	 * both codecs' own rounding.
	 */
	if (r.top + ALIGN(r.height, 32) > ALIGN(ctx->src_h, 32))
		return -EINVAL;

	if (!r.left && !r.top && r.width == ctx->src_w &&
	    r.height == ctx->src_h)
		ctx->crop = (struct v4l2_rect){ 0 };
	else
		ctx->crop = r;
	s->r = r;

	histb_venc_v5_apply_geometry(ctx);
	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx,
			     V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	if (!vb2_is_busy(vq))
		histb_venc_v5_reset_cap_fmt(ctx);

	return 0;
}

static int histb_venc_v5_enum_framesizes(struct file *file, void *priv,
				       struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index > 0)
		return -EINVAL;
	if (!histb_venc_v5_is_input(fsize->pixel_format))
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	/*
	 * The block's own geometric face: two pixels either way, up to the
	 * picture it can hold. Which codec allows the small end is the capture
	 * format's business, and starting a stream says so.
	 */
	fsize->stepwise.min_width = 128;
	fsize->stepwise.max_width = 1920;
	fsize->stepwise.step_width = 2;
	fsize->stepwise.min_height = 128;
	fsize->stepwise.max_height = 1088;
	fsize->stepwise.step_height = 2;

	return 0;
}

/* --------------------------------------------------------------- buffers */

static int histb_venc_v5_queue_setup(struct vb2_queue *q, unsigned int *num_buffers,
				   unsigned int *num_planes, unsigned int sizes[],
				   struct device *alloc_devs[])
{
	struct histb_venc_v5_ctx *ctx = vb2_get_drv_priv(q);
	const struct v4l2_pix_format_mplane *pix =
		&ctx->fmt[V4L2_TYPE_IS_OUTPUT(q->type)].fmt.pix_mp;
	unsigned int i;

	if (*num_planes) {
		for (i = 0; i < *num_planes; i++) {
			if (sizes[i] < pix->plane_fmt[i].sizeimage)
				return -EINVAL;
		}
		return 0;
	}

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		if (*num_buffers > V500_MAX_OUTPUT_BUFFERS)
			return -EINVAL;
	} else if (*num_buffers > V500_MAX_CAPTURE_BUFFERS) {
		return -EINVAL;
	}

	*num_planes = pix->num_planes;
	for (i = 0; i < pix->num_planes; i++)
		sizes[i] = pix->plane_fmt[i].sizeimage;

	return 0;
}

static int histb_venc_v5_buf_prepare(struct vb2_buffer *vb)
{
	struct histb_venc_v5_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	const struct v4l2_pix_format_mplane *pix =
		&ctx->fmt[V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type)].fmt.pix_mp;
	unsigned int i;

	for (i = 0; i < vb->num_planes; i++) {
		if (vb2_plane_size(vb, i) < pix->plane_fmt[i].sizeimage)
			return -EINVAL;

		/*
		 * The core reads and writes whole 16-byte lines, and its
		 * addresses are declared in the image as plain words, so every
		 * plane has to start on that boundary.
		 */
		if (vb2_dma_contig_plane_dma_addr(vb, i) % 16)
			return -EINVAL;

		/*
		 * Those image words are 32 bits wide, so a plane ending above
		 * 4 GiB would be read from a truncated address. The device's
		 * 32-bit mask bounds its own allocations; an imported plane is
		 * checked here rather than trusted.
		 */
		if ((u64)vb2_dma_contig_plane_dma_addr(vb, i) +
		    vb2_plane_size(vb, i) - 1 > DMA_BIT_MASK(32))
			return -ERANGE;

		vb2_set_plane_payload(vb, i, 0);
	}

	return 0;
}

/* The end of a drained stream is said twice: in a buffer and as an event. */
static const struct v4l2_event histb_venc_v5_eos_event = {
	.type = V4L2_EVENT_EOS,
};

static int histb_venc_v5_subscribe_event(struct v4l2_fh *fh,
				       const struct v4l2_event_subscription *sub)
{
	if (sub->type == V4L2_EVENT_EOS)
		return v4l2_event_subscribe(fh, sub, 0, NULL);

	return v4l2_ctrl_subscribe_event(fh, sub);
}

static void histb_venc_v5_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct histb_venc_v5_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	unsigned int i;

	/*
	 * A drain asked for while nothing was queued has no picture to put the
	 * end marker on, so the first capture buffer the client offers after it
	 * is the marker: no bytes, the LAST flag, and the session stopped.
	 */
	if (V4L2_TYPE_IS_CAPTURE(vb->vb2_queue->type) &&
	    vb2_is_streaming(vb->vb2_queue) &&
	    v4l2_m2m_dst_buf_is_last(ctx->fh.m2m_ctx)) {
		for (i = 0; i < vb->num_planes; i++)
			vb2_set_plane_payload(vb, i, 0);
		vbuf->field = V4L2_FIELD_NONE;
		v4l2_m2m_last_buffer_done(ctx->fh.m2m_ctx, vbuf);
		v4l2_event_queue_fh(&ctx->fh, &histb_venc_v5_eos_event);
		return;
	}

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static int histb_venc_v5_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct histb_venc_v5_ctx *ctx = vb2_get_drv_priv(q);
	struct histb_venc_v5 *enc = histb_venc_v5_of_ctx(ctx);
	unsigned int lo = min(ctx->user_min_qp, ctx->user_max_qp);
	unsigned int hi = max(ctx->user_min_qp, ctx->user_max_qp);
	unsigned int min_width = ctx->hevc ? 128 : 256;
	unsigned int height = ctx->frame.height;
	unsigned int width = ctx->frame.width;
	int ret = 0;

	/*
	 * The codecs differ at the small end: H.264 codes whole macroblock
	 * pairs, so nothing below the picture it can name starts here. Say so
	 * at the point the geometry is frozen rather than code a broken stream.
	 */
	if (width < min_width || height < 128)
		return -EINVAL;

	mutex_lock(&enc->lock);
	if (!enc->ctx) {
		enc->ctx = ctx;
		/*
		 * A new stream starts its GOP, frame numbering and rate-control
		 * state over.
		 */
		ctx->sequence = 0;
		ctx->gop_left = 0;
		ctx->force_idr = false;
		ctx->frame.frame_num = 0;
		ctx->frame.idr_pic_id = 0;
		ctx->frame.poc = 0;
		ctx->rc_qp = (lo + hi) / 2;
		ctx->rc_credit = 0;
		ctx->gop_bits = 0;
	} else if (enc->ctx != ctx) {
		ret = -EBUSY;
	}
	mutex_unlock(&enc->lock);
	/* A stream restarted after a drain is not the tail of the old one. */
	if (!ret)
		v4l2_m2m_update_start_streaming_state(ctx->fh.m2m_ctx, q);

	return ret;
}

static void histb_venc_v5_stop_streaming(struct vb2_queue *q)
{
	struct histb_venc_v5_ctx *ctx = vb2_get_drv_priv(q);
	struct histb_venc_v5 *enc = histb_venc_v5_of_ctx(ctx);
	unsigned int i;

	while (true) {
		struct vb2_v4l2_buffer *vbuf;

		if (V4L2_TYPE_IS_OUTPUT(q->type))
			vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		else
			vbuf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

		if (!vbuf)
			break;

		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
	}

	/* A buffer the mem2mem lists no longer carry is still ours to return. */
	for (i = 0; i < vb2_get_num_buffers(q); i++) {
		struct vb2_buffer *vb = vb2_get_buffer(q, i);

		if (vb != NULL && vb->state == VB2_BUF_STATE_ACTIVE)
			vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
	}

	mutex_lock(&enc->lock);
	if (enc->ctx == ctx)
		enc->ctx = NULL;
	mutex_unlock(&enc->lock);
	/*
	 * A drain that had not finished still owes its caller the end marker,
	 * and the capture queue's own teardown is what clears the session state.
	 */
	v4l2_m2m_update_stop_streaming_state(ctx->fh.m2m_ctx, q);
}

static const struct vb2_ops histb_venc_v5_vb2_ops = {
	.queue_setup		= histb_venc_v5_queue_setup,
	.buf_prepare		= histb_venc_v5_buf_prepare,
	.buf_queue		= histb_venc_v5_buf_queue,
	.start_streaming	= histb_venc_v5_start_streaming,
	.stop_streaming		= histb_venc_v5_stop_streaming,
};

static int histb_venc_v5_queue_init(void *priv, struct vb2_queue *src_vq,
				  struct vb2_queue *dst_vq)
{
	struct histb_venc_v5_ctx *ctx = priv;
	struct histb_venc_v5 *enc = histb_venc_v5_of_ctx(ctx);
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->ops = &histb_venc_v5_vb2_ops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->dma_attrs = DMA_ATTR_ALLOC_SINGLE_PAGES;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &enc->vdev_lock;
	src_vq->dev = enc->v4l2.dev;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->ops = &histb_venc_v5_vb2_ops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	/* The driver copies the coded bytes in, so it needs a kernel mapping. */
	dst_vq->dma_attrs = DMA_ATTR_ALLOC_SINGLE_PAGES;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &enc->vdev_lock;
	dst_vq->dev = enc->v4l2.dev;

	return vb2_queue_init(dst_vq);
}

/* -------------------------------------------------------------- controls */

/*
 * The menu numbers the profiles in the syntax's order and the command image in
 * the vendor's, so the two need naming apart.
 */
static unsigned int histb_venc_v5_vendor_profile(unsigned int profile)
{
	switch (profile) {
	case V4L2_MPEG_VIDEO_H264_PROFILE_MAIN:
		return V500_H264_PROFILE_MAIN;
	case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH:
		return V500_H264_PROFILE_HIGH;
	default:
		return V500_H264_PROFILE_BASELINE;
	}
}

static int histb_venc_v5_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct histb_venc_v5_ctx *ctx =
		container_of(ctrl->handler, struct histb_venc_v5_ctx,
			     ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_BITRATE:
		ctx->bitrate = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MIN_QP:
	case V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP:
		ctx->user_min_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MAX_QP:
	case V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP:
		ctx->user_max_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
		ctx->gop = ctrl->val;
		break;
	/*
	 * The core splits at coding-unit rows, so the byte form has no exact
	 * image; the menu only offers the unit-count form.
	 */
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB:
		ctx->slice_mb = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE:
		if (ctrl->val == V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE)
			ctx->slice_mb = 0;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
		ctx->profile = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE:
		ctx->cabac = ctrl->val == V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP:
	case V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP:
		ctx->qp_i = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP:
	case V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_QP:
		ctx->qp_p = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME:
		ctx->force_idr = true;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_PROFILE:
		ctx->hevc_profile = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_LEVEL:
		ctx->hevc_level = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_BITRATE_MODE:
		ctx->bitrate_mode = ctrl->val;
		break;
	/* The menu only offers the one header mode this node implements. */
	case V4L2_CID_MPEG_VIDEO_HEADER_MODE:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops histb_venc_v5_ctrl_ops = {
	.s_ctrl = histb_venc_v5_s_ctrl,
};

static int histb_venc_v5_init_ctrls(struct histb_venc_v5_ctx *ctx)
{
	struct v4l2_ctrl_handler *hdl = &ctx->ctrl_handler;
	/* The two H.264 profiles this core is never told to code. */
	const u32 profile_mask =
		BIT(V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE) |
		BIT(V4L2_MPEG_VIDEO_H264_PROFILE_EXTENDED);

	v4l2_ctrl_handler_init(hdl, ARRAY_SIZE(histb_venc_v5_ctrls));

	v4l2_ctrl_new_std_menu(hdl, &histb_venc_v5_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_H264_PROFILE,
			       V4L2_MPEG_VIDEO_H264_PROFILE_HIGH, profile_mask,
			       V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE);
	/*
	 * The entropy coder is the core's own switch, not a consequence of the
	 * profile, so both forms are offered for every profile that allows it.
	 */
	v4l2_ctrl_new_std_menu(hdl, &histb_venc_v5_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE,
			       V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC, 0,
			       V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CAVLC);
	v4l2_ctrl_new_std_menu(hdl, &histb_venc_v5_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_BITRATE_MODE,
			       V4L2_MPEG_VIDEO_BITRATE_MODE_CQ, 0,
			       V4L2_MPEG_VIDEO_BITRATE_MODE_CBR);
	v4l2_ctrl_new_std(hdl, &histb_venc_v5_ctrl_ops, V4L2_CID_MPEG_VIDEO_BITRATE,
			  2000, 100000000, 1000, 4000000);
	/*
	 * The core's quantiser range is the syntax's, and the pair bounds the
	 * rate control rather than fixing a value: an all-intra picture is
	 * allowed the wider low end.
	 */
	v4l2_ctrl_new_std(hdl, &histb_venc_v5_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_MIN_QP, 1, 51, 1, 2);
	v4l2_ctrl_new_std(hdl, &histb_venc_v5_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_MAX_QP, 1, 51, 1, 51);
	/*
	 * The two quantisers the constant-quality mode codes with, one per
	 * picture type. The window above does not move them: nothing is looked
	 * up from the bits the last picture spent in that mode.
	 */
	v4l2_ctrl_new_std(hdl, &histb_venc_v5_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP, 1, 51, 1, 28);
	v4l2_ctrl_new_std(hdl, &histb_venc_v5_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP, 1, 51, 1, 28);
	v4l2_ctrl_new_std_menu(hdl, &histb_venc_v5_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_HEVC_PROFILE,
			       V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_STILL_PICTURE,
			       BIT(V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10),
			       V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN);
	v4l2_ctrl_new_std_menu(hdl, &histb_venc_v5_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_HEVC_LEVEL,
			       V4L2_MPEG_VIDEO_HEVC_LEVEL_6_2, 0,
			       V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1);
	v4l2_ctrl_new_std(hdl, &histb_venc_v5_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP, 0, 51, 1, 2);
	v4l2_ctrl_new_std(hdl, &histb_venc_v5_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP, 0, 51, 1, 51);
	v4l2_ctrl_new_std(hdl, &histb_venc_v5_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP, 0, 51, 1, 28);
	v4l2_ctrl_new_std(hdl, &histb_venc_v5_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_QP, 0, 51, 1, 28);
	v4l2_ctrl_new_std_menu(hdl, &histb_venc_v5_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_HEADER_MODE,
			       V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME,
			       0, V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME);
	v4l2_ctrl_new_std(hdl, &histb_venc_v5_ctrl_ops,
		V4L2_CID_MPEG_VIDEO_GOP_SIZE, 1, 255, 1, 30);
	v4l2_ctrl_new_std_menu(hdl, &histb_venc_v5_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE,
			       V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_MAX_MB,
			       BIT(V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_MAX_BYTES),
			       V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE);
	v4l2_ctrl_new_std(hdl, &histb_venc_v5_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB,
			  1, 8160, 1, 8160);
	/* A press, not a setting: the next picture is the one it names. */
	v4l2_ctrl_new_std(hdl, &histb_venc_v5_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME, 0, 0, 0, 0);
	if (hdl->error)
		return hdl->error;

	ctx->fh.ctrl_handler = hdl;

	return 0;
}

/* ------------------------------------------------------------------ run */

static void histb_venc_v5_fill_frame(struct histb_venc_v5_ctx *ctx,
				   struct vb2_v4l2_buffer *src,
				   struct histb_venc_v5_frame *frame)
{
	const struct v4l2_pix_format_mplane *pix =
		&ctx->fmt[V500_FMT_OUTPUT].fmt.pix_mp;
	unsigned int src_stride = histb_venc_v5_src_stride(ctx);
	unsigned int input = histb_venc_v5_input(pix->pixelformat);
	unsigned int packed = histb_venc_v5_is_packed(input);
	unsigned int semi422 = histb_venc_v5_is_semi422(input);
	unsigned int vu = input == V500_IN_NV21 || input == V500_IN_NV61;
	unsigned int lo, hi, qp;
	unsigned int i;

	frame->protocol = ctx->hevc ? V500_PROTOCOL_HEVC : V500_PROTOCOL_H264;
	frame->profile = histb_venc_v5_vendor_profile(ctx->profile);
	frame->cabac = !ctx->hevc && ctx->cabac;
	frame->hevc_profile = ctx->hevc_profile;
	frame->hevc_level = ctx->hevc_level;
	/* The parameter sets name the rate and the budget with these. */
	frame->bitrate = ctx->bitrate;
	frame->fps_num = ctx->fps_num;
	frame->fps_den = ctx->fps_den;

	/*
	 * The crop moves the read origin, and the picture the core codes is the
	 * selected rectangle. A packed row carries two bytes per luma sample, so
	 * its horizontal offset doubles; an interleaved chroma row is one byte per
	 * luma sample, so its offset is the luma offset itself, while a planar one
	 * covers two luma columns per sample. Vertically, a 4:2:2 chroma plane has
	 * a row per luma row and a 4:2:0 one a row per pair.
	 */
	histb_venc_v5_apply_geometry(ctx);
	frame->src_y = vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0) +
		       (dma_addr_t)ctx->crop.left * (packed ? 2 : 1) +
		       (dma_addr_t)pix->plane_fmt[0].bytesperline * ctx->crop.top;
	/*
	 * Where the chroma of each layout lives. Only the planar one has a third
	 * plane to name; a packed picture keeps its samples in the luma plane's
	 * rows, and the interleaved one keeps both orders pointed at the same
	 * plane, as the vendor build does.
	 */
	frame->src_v = frame->src_y;
	frame->src_c = frame->src_y;
	frame->src_stride_y = pix->plane_fmt[0].bytesperline;
	frame->src_stride_c = src_stride;
	switch (input) {
	case V500_IN_YUV420:
		frame->src_c = vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 1) +
			       ctx->crop.left / 2 +
			       (dma_addr_t)pix->plane_fmt[1].bytesperline *
				       (ctx->crop.top / 2);
		frame->src_v = vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 2) +
			       ctx->crop.left / 2 +
			       (dma_addr_t)pix->plane_fmt[2].bytesperline *
				       (ctx->crop.top / 2);
		frame->store = V500_STORE_PLANAR;
		frame->package = 0;
		frame->src_stride_y = pix->plane_fmt[0].bytesperline;
		frame->src_stride_c = pix->plane_fmt[1].bytesperline;
		break;
	case V500_IN_YUYV:
	case V500_IN_UYVY:
		frame->store = V500_STORE_PACKAGE;
		frame->package = input == V500_IN_UYVY ? V500_PACKAGE_UYVY :
			V500_PACKAGE_YUYV;
		frame->src_stride_y = pix->plane_fmt[0].bytesperline;
		break;
	default:
		frame->src_c = vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 1) +
			       ctx->crop.left +
			       (dma_addr_t)src_stride *
				       (semi422 ? ctx->crop.top :
						ctx->crop.top / 2);
		frame->src_v = frame->src_c;
		frame->store = V500_STORE_SEMI_PLANAR;
		/*
		 * The interleaved pair's order is the one bit of the format word
		 * that names it. The verified UV path writes that bit clear, so
		 * the VU path names it set; the board arm is the chroma-swapped
		 * fixture, which has to give the same stream as the other order.
		 */
		frame->package = vu ? V500_SEMI_VU : V500_SEMI_UV;
		/*
		 * A 4:2:2 chroma plane is walked at twice its own row pitch, which
		 * is how the reference feeds it and what makes the core take every
		 * other chroma row (drv_venc_efl.c:4588).
		 */
		frame->src_stride_c = semi422 ? 2 * src_stride : src_stride;
		break;
	}

	frame->recon[0] = ctx->enc->recon.dma;
	frame->recon[1] = ctx->enc->recon.dma + V500_RECON_BYTES;
	/*
	 * The reconstruction surface is the core's own, so it is walked with the
	 * stride of the picture it holds rather than the source's.
	 */
	frame->recon_stride_y = ALIGN(frame->width, 16);
	frame->recon_stride_c = ALIGN(frame->width, 16);

	/*
	 * One intra picture every GOP, and the first picture of the stream
	 * always is one: the reference chain starts from nothing. A key frame
	 * asked for mid-group is coded at once, and the group then counts from
	 * it rather than from where it was.
	 */
	frame->intra = ctx->gop_left == 0 || ctx->force_idr ||
		(ctx->hevc &&
		 ctx->hevc_profile ==
		 V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_STILL_PICTURE);
	ctx->force_idr = false;
	/* Only gates the copy of the parameter bytes into the command image. */
	frame->parameter_set = 1;

	/*
	 * The core splits between coding-unit rows, and its size word counts the
	 * rows a slice keeps together minus one: a size of zero puts every row in
	 * its own slice. The requested macroblock count therefore becomes a row
	 * count with the picture's own unit width, minus one. One slice per
	 * stream slot is the floor, so no picture asks for more slices than there
	 * are slots to write them into.
	 */
	frame->slice_split = 0;
	frame->slice_split_size = 0;
	if (ctx->slice_mb) {
		unsigned int unit = ctx->hevc ? 32 : 16;
		unsigned int rows = DIV_ROUND_UP(ctx->frame.height, unit);
		unsigned int per_row = DIV_ROUND_UP(ctx->frame.width, unit);
		unsigned int slice_rows =
			DIV_ROUND_UP(ctx->slice_mb, per_row) ?: 1;
		unsigned int floor = DIV_ROUND_UP(rows, V500_SLICE_BUFFERS);

		frame->slice_split = 1;
		frame->slice_split_size =
			max_t(unsigned int, slice_rows, floor) - 1;
	}

	/*
	 * The quantiser the picture is coded with is the one named on its own,
	 * so a request that pins a single value has to move it into the window
	 * too; the vendor build writes the same value into all three.
	 * Constant quality names the quantiser per picture type instead, and
	 * nothing bounds it: the mode is the reason the client named it.
	 */
	lo = min(ctx->user_min_qp, ctx->user_max_qp);
	hi = max(ctx->user_min_qp, ctx->user_max_qp);
	if (ctx->bitrate_mode == V4L2_MPEG_VIDEO_BITRATE_MODE_CQ)
		qp = frame->intra ? ctx->qp_i : ctx->qp_p;
	else
		qp = lo == hi ? lo : clamp_t(unsigned int, ctx->rc_qp, lo, hi);
	frame->start_qp = qp;
	frame->min_qp = frame->start_qp;
	frame->max_qp = frame->start_qp;

	/*
	 * The core's rate control works from the bits one picture may spend.
	 * CBR divides the configured rate by the frame rate. VBR spends the
	 * group's budget where the pictures need it: each one is offered the
	 * group's remaining bits over its remaining pictures, bounded to a
	 * quarter of the average below and four times it above.
	 */
	{
		u64 avg = div_u64((u64)ctx->bitrate * ctx->fps_den,
				  ctx->fps_num);

		if (frame->intra)
			ctx->gop_bits = 0;

		if (ctx->bitrate_mode == V4L2_MPEG_VIDEO_BITRATE_MODE_VBR &&
		    ctx->gop) {
			unsigned int left = ctx->gop_left + 1;
			u64 budget = avg * ctx->gop;
			u64 spend = budget > ctx->gop_bits ?
				    budget - ctx->gop_bits : 0;
			u64 target = clamp_t(u64, div64_u64(spend, left),
					     avg / 2, avg * 4);

			frame->target_bits = target;
		} else {
			frame->target_bits = avg;
		}
	}

	/*
	 * A split picture writes each slice into its own slot; unsplit work keeps
	 * the whole ring for slot zero.
	 */
	if (frame->slice_split) {
		unsigned int stride = histb_venc_v5_slot_stride(ctx->enc);

		for (i = 0; i < V500_SLICE_BUFFERS; i++) {
			frame->stream[i] = ctx->enc->stream.dma +
				V500_RING_OFFSET + i * stride;
			frame->stream_size[i] = stride;
		}
	} else {
		frame->stream[0] = ctx->enc->stream.dma + V500_RING_OFFSET;
		frame->stream_size[0] = ctx->enc->stream.size - V500_RING_OFFSET;
	}
	frame->stream_read = ctx->enc->stream.dma + V500_POINT_READ;
	frame->stream_write = ctx->enc->stream.dma + V500_POINT_WRITE;

	frame->param = ctx->enc->param.dma;

	frame->tmv[0] = ctx->enc->scratch.dma;
	frame->tmv[1] = ctx->enc->scratch.dma + SZ_128K;
	frame->pme[0] = ctx->enc->scratch.dma + SZ_256K;
	frame->pme[1] = ctx->enc->scratch.dma + SZ_384K;
	frame->pme_info[0] = ctx->enc->scratch.dma + SZ_512K;
	frame->pme_info[1] = ctx->enc->scratch.dma + SZ_512K * 2;
	frame->pme_info[2] = ctx->enc->scratch.dma + SZ_512K * 3;
	frame->tunnel_cell = ctx->enc->scratch.dma + SZ_512K * 4;
	frame->slice_header = ctx->enc->slice.dma;
}

/*
 * One picture's worth of the rate loop. The core gives no closed-loop CBR on
 * this part: the image's target word only steers the mode decision, so the
 * quantiser is chosen here from the bits the previous picture actually spent.
 * The credit bank is one second deep, which lets an intra picture borrow from
 * the pictures around it without letting a burst of them hold the average away
 * from the target. The user's QP window bounds the choice.
 */
static void histb_venc_v5_rate_update(struct histb_venc_v5_ctx *ctx,
				    unsigned int target_bits)
{
	unsigned int lo = min(ctx->user_min_qp, ctx->user_max_qp);
	unsigned int hi = max(ctx->user_min_qp, ctx->user_max_qp);
	unsigned int target = target_bits;
	unsigned int fps = max_t(unsigned int, ctx->fps_num / ctx->fps_den,
				 1);
	unsigned int step = 1;
	unsigned int depth = fps;
	unsigned int act = histb_venc_v5_pic_bits(ctx->enc);
	s64 limit;

	/* Equal bounds are the fixed-quantiser mode, with no loop at all. */
	if (lo >= hi)
		return;
	/* Constant quality took its quantiser from the client, not from bits. */
	if (ctx->bitrate_mode == V4L2_MPEG_VIDEO_BITRATE_MODE_CQ)
		return;
	if (!target)
		target = 1;
	/*
	 * VBR lets the rate swing: a deeper credit bank and larger quantiser
	 * moves let one picture borrow from its neighbours, which is the point
	 * of the mode. CBR keeps one step per picture.
	 */
	if (ctx->bitrate_mode == V4L2_MPEG_VIDEO_BITRATE_MODE_VBR) {
		step = 3;
		depth = fps * 2;
	}
	limit = (s64)target * depth;

	ctx->rc_credit += (s64)target - act;
	ctx->rc_credit = clamp_t(s64, ctx->rc_credit, -limit, limit);
	if (ctx->rc_credit > (s64)target) {
		ctx->rc_qp = ctx->rc_qp > lo + step ? ctx->rc_qp - step : lo;
		ctx->rc_credit -= (s64)target;
	} else if (ctx->rc_credit < -(s64)target) {
		ctx->rc_qp = ctx->rc_qp + step < hi ? ctx->rc_qp + step : hi;
		ctx->rc_credit += (s64)target;
	}
	ctx->rc_qp = clamp_t(unsigned int, ctx->rc_qp, lo, hi);
	dev_dbg(ctx->enc->dev, "rc: act %u target %u credit %lld qp %u\n",
		act, target, ctx->rc_credit, ctx->rc_qp);
}

static void histb_venc_v5_device_run(void *priv)
{
	struct histb_venc_v5_ctx *ctx = priv;
	struct histb_venc_v5 *enc = histb_venc_v5_of_ctx(ctx);
	struct vb2_v4l2_buffer *src, *dst;
	struct histb_venc_v5_frame *frame = &ctx->frame;
	struct histb_venc_v5_slice slice = { };
	struct dma_buf *dbuf;
	u8 *coded;
	size_t bytes;
	size_t head;
	int ret;
	bool pm = false;

	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	if (!src || !dst)
		return;

	if (READ_ONCE(enc->removing)) {
		v4l2_m2m_buf_done_and_job_finish(enc->m2m, ctx->fh.m2m_ctx,
						 VB2_BUF_STATE_ERROR);
		return;
	}

	ret = pm_runtime_resume_and_get(enc->dev);
	if (ret) {
		dev_dbg(enc->dev, "runtime resume failed: %d\n", ret);
		goto failed;
	}
	pm = true;

	histb_venc_v5_fill_frame(ctx, src, frame);
	frame->recon_index = ctx->sequence % V500_RECON_BUFFERS;
	enc->protocol = frame->protocol;

	/*
	 * The Baseline profile declares no context-adaptive coder, so a picture
	 * told to be both names no syntax. Hand the job back with the reason in
	 * the log instead of coding a stream no decoder is asked to accept.
	 */
	if (frame->profile == V500_H264_PROFILE_BASELINE && frame->cabac) {
		dev_err(enc->dev, "baseline names no cabac, picture refused\n");
		v4l2_m2m_buf_done_and_job_finish(enc->m2m, ctx->fh.m2m_ctx,
						 VB2_BUF_STATE_ERROR);
		pm_runtime_put(enc->dev);
		return;
	}

	ret = histb_venc_v5_parameter_sets(frame, enc->param.va,
						 enc->param.size, frame->param_reg);
	if (ret) {
		dev_err(enc->dev, "parameter sets refused (%d)\n", ret);
		goto failed;
	}

	/*
	 * The ring's two pointers are the core's to advance; both start at
	 * the beginning of the space it was given for this picture.
	 */
	((__le32 *)enc->stream.va)[V500_POINT_READ / 4] = 0;
	((__le32 *)enc->stream.va)[V500_POINT_WRITE / 4] = 0;

	histb_venc_v5_slice_header(frame, &slice);

	histb_venc_v5_image_pack(enc, frame, &slice);

	mutex_lock(&enc->lock);
	reinit_completion(&enc->done);
	histb_venc_v5_launch(enc, frame);
	if (!wait_for_completion_timeout(&enc->done, V500_TIMEOUT)) {
		mutex_unlock(&enc->lock);
		dev_err(enc->dev, "picture did not finish, status %#x\n",
			histb_venc_v5_status(enc));
		goto failed;
	}
	mutex_unlock(&enc->lock);

	if (enc->status & (V500_STATUS_TIMEOUT | V500_STATUS_BUF_FULL |
			   V500_STATUS_PBIT_OVERFLOW)) {
		dev_err(enc->dev, "picture ended on status %#x\n", enc->status);
		goto failed;
	}

	/* Any other report but the end of the picture is an unfinished block. */
	if (!(enc->status & V500_STATUS_PIC)) {
		dev_err(enc->dev, "no end of picture, status %#x\n",
			enc->status);
		goto failed;
	}

	coded = vb2_plane_vaddr(&dst->vb2_buf, 0);
	if (!coded) {
		dev_err(enc->dev, "capture plane has no kernel mapping\n");
		goto failed;
	}

	/* The coded bytes are written through the CPU, not the core. */
	dbuf = dst->vb2_buf.planes[0].dbuf;
	if (dbuf)
		dma_buf_begin_cpu_access(dbuf, DMA_TO_DEVICE);

	head = 0;
	if (frame->intra && frame->param_size + V500_PARAM_HEADROOM <=
		    vb2_plane_size(&dst->vb2_buf, 0)) {
		memcpy(coded, enc->param.va, frame->param_size);
		head = frame->param_size;
	}
	bytes = histb_venc_v5_drain(enc, frame, coded + head,
				  vb2_plane_size(&dst->vb2_buf, 0) - head);
	if (IS_ERR_VALUE(bytes)) {
		if (dbuf)
			dma_buf_end_cpu_access(dbuf, DMA_TO_DEVICE);
		dev_err(enc->dev, "no coded bytes: %zd\n", bytes);
		goto failed;
	}
	bytes += head;

	if (dbuf)
		dma_buf_end_cpu_access(dbuf, DMA_TO_DEVICE);

	v4l2_m2m_buf_copy_metadata(src, dst);
	dst->sequence = ctx->sequence;
	dst->flags |= frame->intra ? V4L2_BUF_FLAG_KEYFRAME :
		V4L2_BUF_FLAG_PFRAME;
	/* The picture the drain was waiting for carries the end marker. */
	if (v4l2_m2m_is_last_draining_src_buf(ctx->fh.m2m_ctx, src)) {
		dst->flags |= V4L2_BUF_FLAG_LAST;
		v4l2_event_queue_fh(&ctx->fh, &histb_venc_v5_eos_event);
		v4l2_m2m_mark_stopped(ctx->fh.m2m_ctx);
	}
	vb2_set_plane_payload(&dst->vb2_buf, 0, bytes);
	ctx->gop_bits += (u64)histb_venc_v5_pic_bits(enc);
	histb_venc_v5_rate_update(ctx, frame->target_bits);
	/*
	 * The group's position and the picture count move on only once the
	 * picture is handed over, so a job that failed is retried against the
	 * same reference and inside the same group slot, the way the v2 driver
	 * rolls its counters back when it drops a frame.
	 */
	ctx->gop_left = frame->intra ? ctx->gop - 1 : ctx->gop_left - 1;
	ctx->sequence++;
	v4l2_m2m_buf_done_and_job_finish(enc->m2m, ctx->fh.m2m_ctx,
					 VB2_BUF_STATE_DONE);
	if (pm)
		pm_runtime_put(enc->dev);

	return;

failed:
	if (pm)
		histb_venc_v5_stop_core(enc);
	v4l2_m2m_buf_done_and_job_finish(enc->m2m, ctx->fh.m2m_ctx,
					 VB2_BUF_STATE_ERROR);
	if (pm)
		pm_runtime_put(enc->dev);
}

/*
 * device_run is synchronous: it keeps both buffers on the ready
 * lists until it finishes, so STREAMOFF and close only wait for
 * the frame already in flight instead of completing it from a
 * second thread.
 */
static const struct v4l2_m2m_ops histb_venc_v5_m2m_ops = {
	.device_run	= histb_venc_v5_device_run,
};

static irqreturn_t histb_venc_v5_irq(int irq, void *dev_id)
{
	struct histb_venc_v5 *enc = dev_id;
	unsigned int status = histb_venc_v5_status(enc);

	/*
	 * The line is level-high and the enable word the vendor uses unmasks one
	 * source this driver does not name, so any report but an empty status is
	 * cleared and wakes the waiting job: an undecoded bit left on the line
	 * would keep re-firing until the kernel switched the interrupt off. What
	 * a report without the picture-end bit means is decided by the job.
	 */
	if (!status)
		return IRQ_NONE;

	writel(V500_INT_CLEAR_ALL, enc->base + V500_INT_CLEAR);
	enc->status = status;
	complete(&enc->done);

	return IRQ_HANDLED;
}

/* ------------------------------------------------------------- ioctls */

static int histb_venc_v5_g_parm(struct file *file, void *fh,
			      struct v4l2_streamparm *p)
{
	struct histb_venc_v5_ctx *ctx = file_to_ctx(file);

	if (p->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
	    p->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;

	p->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	p->parm.capture.timeperframe.numerator = ctx->fps_den;
	p->parm.capture.timeperframe.denominator = ctx->fps_num;

	return 0;
}

static int histb_venc_v5_s_parm(struct file *file, void *fh,
			      struct v4l2_streamparm *p)
{
	struct histb_venc_v5_ctx *ctx = file_to_ctx(file);
	struct v4l2_fract *t;

	if (p->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
	    p->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;

	t = &p->parm.capture.timeperframe;
	if (!t->numerator || !t->denominator ||
	    t->denominator / t->numerator < 1 ||
	    t->denominator / t->numerator > 120)
		return -EINVAL;

	ctx->fps_num = t->denominator;
	ctx->fps_den = t->numerator;
	p->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;

	return 0;
}

static int histb_venc_v5_encoder_cmd(struct file *file, void *priv,
				   struct v4l2_encoder_cmd *cmd)
{
	struct histb_venc_v5_ctx *ctx = file_to_ctx(file);
	int ret;

	/*
	 * The mem2m core owns the drain state: STOP records the last output
	 * buffer, or - when none is queued - asks for the next capture buffer to
	 * be the end marker, and START clears the stopped state so the same
	 * stream runs on. The picture that carries the marker is tagged where it
	 * completes, and the queue that is left empty afterwards is answered in
	 * the capture buffer's own queue callback.
	 */
	ret = v4l2_m2m_encoder_cmd(file, ctx->fh.m2m_ctx, cmd);
	/*
	 * A drain with nothing left to code is finished by the core inside this
	 * call, marker buffer and all, so the end of stream has to be said here:
	 * a session that codes no further picture would never reach the
	 * completion path that announces it.
	 */
	if (!ret && cmd->cmd == V4L2_ENC_CMD_STOP &&
	    v4l2_m2m_has_stopped(ctx->fh.m2m_ctx))
		v4l2_event_queue_fh(&ctx->fh, &histb_venc_v5_eos_event);

	return ret;
}

static const struct v4l2_ioctl_ops histb_venc_v5_ioctl_ops = {

	.vidioc_querycap = histb_venc_v5_querycap,
	.vidioc_enum_framesizes = histb_venc_v5_enum_framesizes,
	.vidioc_enum_fmt_vid_cap = histb_venc_v5_enum_fmt,
	.vidioc_enum_fmt_vid_out = histb_venc_v5_enum_fmt,
	.vidioc_g_fmt_vid_cap_mplane = histb_venc_v5_g_fmt,
	.vidioc_g_fmt_vid_out_mplane = histb_venc_v5_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane = histb_venc_v5_try_fmt_vidioc,
	.vidioc_try_fmt_vid_out_mplane = histb_venc_v5_try_fmt_vidioc,
	.vidioc_s_fmt_vid_cap_mplane = histb_venc_v5_s_fmt,
	.vidioc_s_fmt_vid_out_mplane = histb_venc_v5_s_fmt,
	.vidioc_g_selection = histb_venc_v5_g_selection,
	.vidioc_s_selection = histb_venc_v5_s_selection,
	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
	.vidioc_encoder_cmd = histb_venc_v5_encoder_cmd,
	.vidioc_try_encoder_cmd = v4l2_m2m_ioctl_try_encoder_cmd,
	.vidioc_g_parm = histb_venc_v5_g_parm,
	.vidioc_s_parm = histb_venc_v5_s_parm,
	.vidioc_subscribe_event = histb_venc_v5_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static int histb_venc_v5_open(struct file *filp)
{
	struct histb_venc_v5 *enc = video_drvdata(filp);
	struct histb_venc_v5_ctx *ctx;
	int ret;

	ctx = kzalloc_obj(*ctx);
	if (!ctx)
		return -ENOMEM;

	ctx->enc = enc;
	ctx->frame.start_qp = 28;
	ctx->frame.min_qp = 16;
	ctx->frame.max_qp = 48;
	ctx->bitrate = 4000000;
	ctx->bitrate_mode = V4L2_MPEG_VIDEO_BITRATE_MODE_CBR;
	ctx->profile = V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE;
	ctx->cabac = false;
	ctx->qp_i = 28;
	ctx->qp_p = 28;
	ctx->hevc_profile = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN;
	ctx->hevc_level = V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1;
	ctx->fps_num = 30;
	ctx->fps_den = 1;
	ctx->rc_qp = 28;
	ctx->user_min_qp = 2;
	ctx->user_max_qp = 51;
	ctx->gop = 30;

	v4l2_fh_init(&ctx->fh, video_devdata(filp));

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(enc->m2m, ctx,
					    histb_venc_v5_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto err_free;
	}

	ctx->fmt[V500_FMT_OUTPUT].type =
		V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	ctx->fmt[V500_FMT_OUTPUT].fmt.pix_mp.pixelformat =
		V4L2_PIX_FMT_NV12M;
	ctx->fmt[V500_FMT_CAPTURE].type =
		V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	histb_venc_v5_try_fmt(ctx, &ctx->fmt[V500_FMT_OUTPUT]);
	histb_venc_v5_try_fmt(ctx, &ctx->fmt[V500_FMT_CAPTURE]);
	ctx->src_w = ctx->fmt[V500_FMT_OUTPUT].fmt.pix_mp.width;
	ctx->src_h = ctx->fmt[V500_FMT_OUTPUT].fmt.pix_mp.height;
	histb_venc_v5_apply_geometry(ctx);

	ret = histb_venc_v5_init_ctrls(ctx);
	if (ret)
		goto err_m2m;

	v4l2_fh_add(&ctx->fh, filp);
	atomic_inc(&enc->open_count);

	return 0;

err_m2m:
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
err_free:
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return ret;
}

static int histb_venc_v5_release(struct file *filp)
{
	struct histb_venc_v5_ctx *ctx = file_to_ctx(filp);
	struct histb_venc_v5 *enc = ctx->enc;

	v4l2_fh_del(&ctx->fh, filp);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);

	mutex_lock(&enc->lock);
	if (enc->ctx == ctx)
		enc->ctx = NULL;
	mutex_unlock(&enc->lock);

	if (atomic_dec_and_test(&enc->open_count))
		wake_up_all(&enc->open_wq);

	return 0;
}

static const struct v4l2_file_operations histb_venc_v5_fops = {
	.owner		= THIS_MODULE,
	.open		= histb_venc_v5_open,
	.release	= histb_venc_v5_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

/* --------------------------------------------------------- registration */

void histb_venc_v5_v4l2_exit(struct histb_venc_v5 *enc)
{
	enc->removing = true;
	video_unregister_device(&enc->vfd);
	/*
	 * An open file handle still owns driver state, and its release
	 * needs the mem2mem context. Let the handles drain before the
	 * memory they use is given back.
	 */
	if (!wait_event_timeout(enc->open_wq,
				atomic_read(&enc->open_count) == 0,
				msecs_to_jiffies(10000)))
		dev_warn(enc->dev, "%d file handles still open at remove\n",
			 atomic_read(&enc->open_count));
	v4l2_m2m_suspend(enc->m2m);
	disable_irq(enc->irq);
	v4l2_m2m_unregister_media_controller(enc->m2m);
	v4l2_m2m_release(enc->m2m);
	v4l2_device_unregister(&enc->v4l2);
	media_device_cleanup(&enc->mdev);
}

int histb_venc_v5_v4l2_init(struct histb_venc_v5 *enc)
{
	struct video_device *vfd = &enc->vfd;
	int ret;

	mutex_init(&enc->vdev_lock);
	init_completion(&enc->done);
	atomic_set(&enc->open_count, 0);
	init_waitqueue_head(&enc->open_wq);

	ret = devm_request_irq(enc->dev, enc->irq, histb_venc_v5_irq, 0,
			       HISTB_VENC_NAME, enc);
	if (ret)
		return dev_err_probe(enc->dev, ret, "failed to get interrupt\n");

	ret = v4l2_device_register(enc->dev, &enc->v4l2);
	if (ret)
		return dev_err_probe(enc->dev, ret, "failed to register v4l2 device\n");
	enc->v4l2.mdev = &enc->mdev;

	strscpy(enc->mdev.driver_name, HISTB_VENC_NAME,
		sizeof(enc->mdev.driver_name));
	strscpy(enc->mdev.model, "Hi3798MV200H VEDU", sizeof(enc->mdev.model));
	snprintf(enc->mdev.bus_info, sizeof(enc->mdev.bus_info),
		 "platform:%s", dev_name(enc->dev));
	enc->mdev.dev = enc->dev;
	media_device_init(&enc->mdev);

	enc->m2m = v4l2_m2m_init(&histb_venc_v5_m2m_ops);
	if (IS_ERR(enc->m2m)) {
		ret = PTR_ERR(enc->m2m);
		goto err_v4l2;
	}

	vfd->v4l2_dev = &enc->v4l2;
	vfd->fops = &histb_venc_v5_fops;
	vfd->ioctl_ops = &histb_venc_v5_ioctl_ops;
	vfd->lock = &enc->vdev_lock;
	vfd->release = video_device_release_empty;
	vfd->vfl_dir = VFL_DIR_M2M;
	vfd->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	strscpy(vfd->name, HISTB_VENC_NAME, sizeof(vfd->name));
	video_set_drvdata(vfd, enc);

	ret = video_register_device(vfd, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_m2m;

	ret = v4l2_m2m_register_media_controller(enc->m2m, vfd,
		MEDIA_ENT_F_PROC_VIDEO_ENCODER);
	if (ret)
		goto err_vdev;

	ret = media_device_register(&enc->mdev);
	if (ret)
		goto err_mc;

	dev_info(enc->dev, "video encoder, Hi3798MV200H VEDU\n");

	return 0;

err_mc:
	v4l2_m2m_unregister_media_controller(enc->m2m);
err_vdev:
	video_unregister_device(vfd);
err_m2m:
	v4l2_m2m_release(enc->m2m);
err_v4l2:
	v4l2_device_unregister(&enc->v4l2);
	media_device_cleanup(&enc->mdev);
	return ret;
}
