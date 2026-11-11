// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * V4L2 mem2mem core of the HiSilicon HiSTB stateless video decoders.
 *
 * Userspace hands in fully parsed syntax through request controls; one job is
 * one OUTPUT (coded slice data) and one CAPTURE (a linear frame the backend
 * detiled from its own surface) buffer.  What lives here is everything the two
 * generations do the same way: the video node and its media entity, format
 * negotiation against the generation's own list, the vb2 queues, the control
 * handler and request validation.  The message ABI, the launch, the interrupt,
 * the surface geometry and the clock/reset glue stay with the backends.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <media/media-device.h>
#include <media/media-request.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

#include "histb_vdec_core.h"

static struct histb_vdec_common *histb_vdec_common_of_file(struct file *filp)
{
	return video_get_drvdata(video_devdata(filp));
}

static struct histb_vdec_core_ctx *histb_vdec_ctx_of_file(struct file *filp)
{
	struct v4l2_fh *fh = file_to_v4l2_fh(filp);

	return container_of(fh, struct histb_vdec_core_ctx, fh);
}

/* ---------------------------------------------------------------- formats */

/*
 * dev is passed explicitly, not looked up through ctx: ->open() calls this
 * before v4l2_fh_init(), so ctx->fh.vdev is still NULL there.
 */
static void histb_vdec_reset_fmts(struct histb_vdec_core_ctx *ctx,
				  const struct histb_vdec_common *common)
{
	const struct histb_vdec_fmt *def = common->gen->chip->fmts[0];
	unsigned int width = common->gen->default_width ?: common->gen->max_width;
	struct v4l2_format *out = &ctx->out_fmt;
	struct v4l2_format *cap = &ctx->cap_fmt;

	ctx->desc = def;

	memset(out, 0, sizeof(*out));
	out->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	out->fmt.pix_mp.pixelformat = def->fourcc;
	out->fmt.pix_mp.field = V4L2_FIELD_NONE;
	out->fmt.pix_mp.num_planes = 1;
	out->fmt.pix_mp.colorspace = V4L2_COLORSPACE_REC709;
	out->fmt.pix_mp.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	out->fmt.pix_mp.quantization = V4L2_QUANTIZATION_DEFAULT;
	out->fmt.pix_mp.xfer_func = V4L2_XFER_FUNC_DEFAULT;
	common->gen->set_out_size(ctx, out, width, 1088);
	out->fmt.pix_mp.width = width;
	out->fmt.pix_mp.height = 1088;
	out->fmt.pix_mp.plane_fmt[0].sizeimage = HISTB_VDEC_DEFAULT_OUTPUT_SIZE;

	memset(cap, 0, sizeof(*cap));
	cap->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	cap->fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV21;
	cap->fmt.pix_mp.field = V4L2_FIELD_NONE;
	cap->fmt.pix_mp.num_planes = 1;
	cap->fmt.pix_mp.colorspace = V4L2_COLORSPACE_REC709;
	cap->fmt.pix_mp.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	cap->fmt.pix_mp.quantization = V4L2_QUANTIZATION_DEFAULT;
	cap->fmt.pix_mp.xfer_func = V4L2_XFER_FUNC_DEFAULT;
	common->gen->set_cap_reset(ctx, cap, out->fmt.pix_mp.width,
				   out->fmt.pix_mp.height);
}

static void histb_vdec_reset_capture_fmt(struct histb_vdec_core_ctx *ctx)
{
	struct histb_vdec_common *common = histb_vdec_common_of_ctx(ctx);
	const struct v4l2_pix_format_mplane *out = &ctx->out_fmt.fmt.pix_mp;
	struct v4l2_format *cap = &ctx->cap_fmt;

	cap->fmt.pix_mp.field = V4L2_FIELD_NONE;
	/* Propagate the colourspace information of the bitstream to capture. */
	cap->fmt.pix_mp.colorspace = out->colorspace;
	cap->fmt.pix_mp.ycbcr_enc = out->ycbcr_enc;
	cap->fmt.pix_mp.quantization = out->quantization;
	cap->fmt.pix_mp.xfer_func = out->xfer_func;
	common->gen->set_cap_reset(ctx, cap, out->width, out->height);
}

static bool histb_vdec_cap_format(const struct histb_vdec_gen *gen, u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < gen->num_cap_formats; i++)
		if (gen->cap_formats[i] == fourcc)
			return true;

	return false;
}

static const struct histb_vdec_fmt *
histb_vdec_find_fmt(const struct histb_vdec_gen *gen, u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < gen->chip->num_fmts; i++)
		if (gen->chip->fmts[i]->fourcc == fourcc)
			return gen->chip->fmts[i];

	return NULL;
}

static int histb_vdec_querycap(struct file *file, void *priv,
			       struct v4l2_capability *cap)
{
	struct histb_vdec_common *common = histb_vdec_common_of_file(file);

	strscpy(cap->driver, common->gen->name, sizeof(cap->driver));
	strscpy(cap->card, common->gen->name, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(common->dev));
	return 0;
}

static int histb_vdec_enum_fmt_cap(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	struct histb_vdec_common *common = histb_vdec_common_of_file(file);

	if (f->index >= common->gen->num_cap_formats)
		return -EINVAL;

	f->pixelformat = common->gen->cap_formats[f->index];
	return 0;
}

static int histb_vdec_enum_fmt_out(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	struct histb_vdec_common *common = histb_vdec_common_of_file(file);

	if (f->index >= common->gen->chip->num_fmts)
		return -EINVAL;

	f->pixelformat = common->gen->chip->fmts[f->index]->fourcc;
	return 0;
}

static int histb_vdec_try_fmt_cap(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct histb_vdec_core_ctx *ctx = histb_vdec_ctx_of_file(file);
	struct histb_vdec_common *common = histb_vdec_common_of_ctx(ctx);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	unsigned int width, height;

	if (!histb_vdec_cap_format(common->gen, pix->pixelformat))
		pix->pixelformat = common->gen->cap_formats[0];

	/* A decoded frame can not be smaller than the stream resolution. */
	width = min_t(u32, pix->width, common->gen->max_width);
	height = min_t(u32, pix->height, common->gen->max_height);
	width = ALIGN(width, 16);
	height = ALIGN(height, 16);
	width = max(width, ctx->out_fmt.fmt.pix_mp.width);
	height = max(height, ctx->out_fmt.fmt.pix_mp.height);
	pix->width = width;
	pix->height = height;

	pix->field = V4L2_FIELD_NONE;
	pix->num_planes = 1;
	common->gen->set_cap_size(ctx, f, pix->width, pix->height);
	return 0;
}

static int histb_vdec_try_fmt_out(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct histb_vdec_common *common = histb_vdec_common_of_file(file);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;

	if (!histb_vdec_find_fmt(common->gen, pix->pixelformat))
		pix->pixelformat = common->gen->chip->fmts[0]->fourcc;

	pix->field = V4L2_FIELD_NONE;
	pix->num_planes = 1;
	pix->width = ALIGN(min_t(u32, pix->width, common->gen->max_width), 16);
	pix->height = ALIGN(min_t(u32, pix->height, common->gen->max_height),
			    16);

	/*
	 * The bitstream is consumed by the stream fetcher, not a plane
	 * formatter; the nominal size only bounds the buffer allocation and
	 * userspace may size it below the default.
	 */
	if (!pix->plane_fmt[0].sizeimage)
		pix->plane_fmt[0].sizeimage = HISTB_VDEC_DEFAULT_OUTPUT_SIZE;
	return 0;
}

static int histb_vdec_g_fmt_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct histb_vdec_core_ctx *ctx = histb_vdec_ctx_of_file(file);

	*f = ctx->cap_fmt;
	return 0;
}

static int histb_vdec_g_fmt_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct histb_vdec_core_ctx *ctx = histb_vdec_ctx_of_file(file);

	*f = ctx->out_fmt;
	return 0;
}

static int histb_vdec_s_fmt_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct histb_vdec_core_ctx *ctx = histb_vdec_ctx_of_file(file);
	struct vb2_queue *vq;
	int ret;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx,
			     V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	if (vb2_is_busy(vq))
		return -EBUSY;

	ret = histb_vdec_try_fmt_cap(file, priv, f);
	if (ret)
		return ret;

	ctx->cap_fmt = *f;
	return 0;
}

static int histb_vdec_s_fmt_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct histb_vdec_core_ctx *ctx = histb_vdec_ctx_of_file(file);
	struct histb_vdec_common *common = histb_vdec_common_of_ctx(ctx);
	const struct histb_vdec_fmt *desc;
	struct vb2_queue *vq, *peer_vq;
	int ret;

	/*
	 * A protocol change resets the per-protocol hardware state, and the
	 * frame slot geometry follows the stream resolution, so neither can
	 * change once the queues are in use.
	 */
	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx,
			     V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	if (vb2_is_streaming(vq) ||
	    (vb2_is_busy(vq) &&
	     f->fmt.pix_mp.pixelformat != ctx->out_fmt.fmt.pix_mp.pixelformat))
		return -EBUSY;

	peer_vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx,
				  V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	if (vb2_is_busy(peer_vq))
		return -EBUSY;

	ret = histb_vdec_try_fmt_out(file, priv, f);
	if (ret)
		return ret;

	desc = histb_vdec_find_fmt(common->gen, f->fmt.pix_mp.pixelformat);
	if (!desc)
		return -EINVAL;

	if (ctx->desc != desc) {
		ctx->desc = desc;
		if (common->gen->protocol_changed)
			common->gen->protocol_changed(ctx);
	}
	ctx->out_fmt = *f;
	histb_vdec_reset_capture_fmt(ctx);
	return 0;
}

static int histb_vdec_enum_framesizes(struct file *file, void *priv,
				      struct v4l2_frmsizeenum *fsize)
{
	struct histb_vdec_common *common = histb_vdec_common_of_file(file);
	unsigned int min_width = common->gen->min_coded_width ?
				 common->gen->min_coded_width(fsize->pixel_format) :
				 16;

	if (fsize->index)
		return -EINVAL;

	if (!histb_vdec_cap_format(common->gen, fsize->pixel_format) &&
	    !histb_vdec_find_fmt(common->gen, fsize->pixel_format))
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = min_width;
	fsize->stepwise.max_width = common->gen->max_width;
	fsize->stepwise.step_width = 16;
	fsize->stepwise.min_height = 16;
	fsize->stepwise.max_height = common->gen->max_height;
	fsize->stepwise.step_height = 16;
	return 0;
}

static int histb_vdec_g_selection(struct file *file, void *priv,
				  struct v4l2_selection *s)
{
	struct histb_vdec_core_ctx *ctx = histb_vdec_ctx_of_file(file);
	const struct v4l2_pix_format_mplane *pix = histb_vdec_cap_pix(ctx);

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
	    s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE_PADDED:
		break;
	default:
		return -EINVAL;
	}

	/*
	 * The stateless controls carry no cropping information, so the coded
	 * size is the visible size here; tiles past it are never written.
	 */
	s->r.left = 0;
	s->r.top = 0;
	s->r.width = pix->width;
	s->r.height = pix->height;
	return 0;
}

/* ------------------------------------------------------------ vb2 queues */

static int histb_vdec_queue_setup(struct vb2_queue *vq,
				  unsigned int *num_buffers,
				  unsigned int *num_planes,
				  unsigned int sizes[],
				  struct device *alloc_devs[])
{
	struct histb_vdec_core_ctx *ctx = vb2_get_drv_priv(vq);
	struct histb_vdec_common *common = histb_vdec_common_of_ctx(ctx);
	const struct histb_vdec_gen *gen = common->gen;
	const struct v4l2_format *f;

	if (V4L2_TYPE_IS_OUTPUT(vq->type))
		f = &ctx->out_fmt;
	else
		f = &ctx->cap_fmt;

	/*
	 * The picture message names references through the capture buffer
	 * index, so a queue deeper than the slot table could never have every
	 * buffer addressable as a reference. The count includes the buffers the
	 * queue already holds: VIDIOC_REQBUFS reaches this op with an empty
	 * queue, VIDIOC_CREATE_BUFS with the existing ones in place.
	 */
	if (!V4L2_TYPE_IS_OUTPUT(vq->type)) {
		unsigned int have = vb2_get_num_buffers(vq);

		if (have >= gen->max_cap_buffers)
			return gen->cap_full_einval ? -EINVAL : -ENOBUFS;
		if (*num_buffers > gen->max_cap_buffers - have) {
			if (gen->cap_clamp)
				*num_buffers = gen->max_cap_buffers - have;
			else
				return gen->cap_full_einval ? -EINVAL :
							      -ENOBUFS;
		}
	}

	if (*num_planes) {
		if (*num_planes != 1)
			return -EINVAL;
		if (sizes[0] < f->fmt.pix_mp.plane_fmt[0].sizeimage)
			return -EINVAL;
		return 0;
	}

	*num_planes = 1;
	sizes[0] = f->fmt.pix_mp.plane_fmt[0].sizeimage;
	return 0;
}

static int histb_vdec_buf_prepare(struct vb2_buffer *vb)
{
	struct histb_vdec_core_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct histb_vdec_common *common = histb_vdec_common_of_ctx(ctx);
	const struct v4l2_format *f;
	int ret;

	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type))
		f = &ctx->out_fmt;
	else
		f = &ctx->cap_fmt;

	if (vb2_plane_size(vb, 0) < f->fmt.pix_mp.plane_fmt[0].sizeimage)
		return -EINVAL;

	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type)) {
		if (common->gen->buf_prepare) {
			ret = common->gen->buf_prepare(ctx, vb);
			if (ret)
				return ret;
		}
		return 0;
	}

	/* CAPTURE buffers are always fully written by the decoder. */
	vb2_set_plane_payload(vb, 0, f->fmt.pix_mp.plane_fmt[0].sizeimage);

	return 0;
}

static int histb_vdec_buf_init(struct vb2_buffer *vb)
{
	struct histb_vdec_core_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct histb_vdec_common *common = histb_vdec_common_of_ctx(ctx);

	if (common->gen->buf_init)
		return common->gen->buf_init(ctx, vb);
	return 0;
}

static void histb_vdec_buf_cleanup(struct vb2_buffer *vb)
{
	struct histb_vdec_core_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct histb_vdec_common *common = histb_vdec_common_of_ctx(ctx);

	if (common->gen->buf_cleanup)
		common->gen->buf_cleanup(ctx, vb);
}

static int histb_vdec_buf_out_validate(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	vbuf->field = V4L2_FIELD_NONE;
	return 0;
}

static void histb_vdec_buf_queue(struct vb2_buffer *vb)
{
	struct histb_vdec_core_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static void histb_vdec_buf_request_complete(struct vb2_buffer *vb)
{
	struct histb_vdec_core_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_ctrl_request_complete(vb->req_obj.req, &ctx->ctrl_hdl);
}

static int histb_vdec_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct histb_vdec_core_ctx *ctx = vb2_get_drv_priv(q);
	struct histb_vdec_common *common = histb_vdec_common_of_ctx(ctx);

	if (common->gen->start)
		return common->gen->start(ctx, q);
	return 0;
}

static void histb_vdec_queue_cleanup(struct histb_vdec_core_ctx *ctx,
				     struct vb2_queue *q, u32 state)
{
	unsigned int i;

	while (true) {
		struct vb2_v4l2_buffer *vbuf;

		if (V4L2_TYPE_IS_OUTPUT(q->type))
			vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		else
			vbuf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

		if (!vbuf)
			break;

		v4l2_ctrl_request_complete(vbuf->vb2_buf.req_obj.req,
					   &ctx->ctrl_hdl);
		v4l2_m2m_buf_done(vbuf, state);
	}

	/* A buffer the mem2mem lists no longer carry is still ours to return. */
	for (i = 0; i < vb2_get_num_buffers(q); i++) {
		struct vb2_buffer *vb = vb2_get_buffer(q, i);

		if (!vb || vb->state != VB2_BUF_STATE_ACTIVE)
			continue;
		v4l2_ctrl_request_complete(vb->req_obj.req, &ctx->ctrl_hdl);
		vb2_buffer_done(vb, state);
	}
}

static void histb_vdec_stop_streaming(struct vb2_queue *q)
{
	struct histb_vdec_core_ctx *ctx = vb2_get_drv_priv(q);
	struct histb_vdec_common *common = histb_vdec_common_of_ctx(ctx);

	/*
	 * The mem2mem core drains the running job before stop_streaming can
	 * run, so the hardware is idle here.
	 */
	if (common->gen->stop)
		common->gen->stop(ctx, q);

	histb_vdec_queue_cleanup(ctx, q, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops histb_vdec_queue_ops = {
	.queue_setup = histb_vdec_queue_setup,
	.buf_init = histb_vdec_buf_init,
	.buf_prepare = histb_vdec_buf_prepare,
	.buf_out_validate = histb_vdec_buf_out_validate,
	.buf_queue = histb_vdec_buf_queue,
	.buf_cleanup = histb_vdec_buf_cleanup,
	.buf_request_complete = histb_vdec_buf_request_complete,
	.start_streaming = histb_vdec_start_streaming,
	.stop_streaming = histb_vdec_stop_streaming,
};

static int histb_vdec_queue_init(void *priv, struct vb2_queue *src_vq,
				 struct vb2_queue *dst_vq)
{
	struct histb_vdec_core_ctx *ctx = priv;
	struct histb_vdec_common *common = histb_vdec_common_of_ctx(ctx);
	const struct histb_vdec_gen *gen = common->gen;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->ops = &histb_vdec_queue_ops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	/*
	 * The driver reads the bitstream in the kernel (slice delimitation and
	 * header parsing), so the plane must keep a kernel mapping;
	 * DMA_ATTR_NO_KERNEL_MAPPING would leave vb2_plane_vaddr() NULL.
	 */
	src_vq->dma_attrs = DMA_ATTR_ALLOC_SINGLE_PAGES;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &common->vdev_lock;
	src_vq->dev = gen->dma_dev(common);
	src_vq->supports_requests = true;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->ops = &histb_vdec_queue_ops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	/* The completion writes the linear frame through a kernel mapping. */
	dst_vq->dma_attrs = DMA_ATTR_ALLOC_SINGLE_PAGES;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &common->vdev_lock;
	dst_vq->dev = gen->dma_dev(common);
	dst_vq->min_queued_buffers = gen->min_queued_buffers;

	return vb2_queue_init(dst_vq);
}

/* --------------------------------------------------------------- controls */

static int histb_vdec_try_ctrl(struct v4l2_ctrl *ctrl)
{
	struct histb_vdec_core_ctx *ctx =
		container_of(ctrl->handler, struct histb_vdec_core_ctx, ctrl_hdl);
	struct histb_vdec_common *common = histb_vdec_common_of_ctx(ctx);

	if (common->gen->try_ctrl)
		return common->gen->try_ctrl(ctx->desc, ctrl);

	return 0;
}

static const struct v4l2_ctrl_ops histb_vdec_ctrl_ops = {
	.try_ctrl = histb_vdec_try_ctrl,
};

/*
 * The control handlers of all file handles carry the controls of every
 * protocol, like rkvdec; a protocol switch then never needs a handler rebuild.
 */
static int histb_vdec_init_ctrls(struct histb_vdec_core_ctx *ctx)
{
	const struct histb_vdec_chip *chip = histb_vdec_common_of_ctx(ctx)->gen->chip;
	unsigned int i, j, nctrls = 0;
	int ret;

	for (i = 0; i < chip->num_fmts; i++)
		nctrls += chip->fmts[i]->num_ctrls;

	v4l2_ctrl_handler_init(&ctx->ctrl_hdl, nctrls);

	for (i = 0; i < chip->num_fmts; i++) {
		for (j = 0; j < chip->fmts[i]->num_ctrls; j++) {
			struct v4l2_ctrl_config cfg = chip->fmts[i]->ctrls[j];

			cfg.ops = &histb_vdec_ctrl_ops;
			v4l2_ctrl_new_custom(&ctx->ctrl_hdl, &cfg, ctx);
			if (ctx->ctrl_hdl.error) {
				ret = ctx->ctrl_hdl.error;
				goto err_free_handler;
			}
		}
	}

	ret = v4l2_ctrl_handler_setup(&ctx->ctrl_hdl);
	if (ret)
		goto err_free_handler;

	ctx->fh.ctrl_handler = &ctx->ctrl_hdl;
	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(&ctx->ctrl_hdl);
	return ret;
}

/* --------------------------------------------------------------- ioctls */

static const struct v4l2_ioctl_ops histb_vdec_ioctl_ops = {
	.vidioc_querycap = histb_vdec_querycap,
	.vidioc_enum_framesizes = histb_vdec_enum_framesizes,

	.vidioc_enum_fmt_vid_cap = histb_vdec_enum_fmt_cap,
	.vidioc_enum_fmt_vid_out = histb_vdec_enum_fmt_out,
	.vidioc_g_fmt_vid_cap_mplane = histb_vdec_g_fmt_cap,
	.vidioc_g_fmt_vid_out_mplane = histb_vdec_g_fmt_out,
	.vidioc_try_fmt_vid_cap_mplane = histb_vdec_try_fmt_cap,
	.vidioc_try_fmt_vid_out_mplane = histb_vdec_try_fmt_out,
	.vidioc_s_fmt_vid_cap_mplane = histb_vdec_s_fmt_cap,
	.vidioc_s_fmt_vid_out_mplane = histb_vdec_s_fmt_out,
	.vidioc_g_selection = histb_vdec_g_selection,

	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,

	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,

	.vidioc_decoder_cmd = v4l2_m2m_ioctl_stateless_decoder_cmd,
	.vidioc_try_decoder_cmd = v4l2_m2m_ioctl_stateless_try_decoder_cmd,

	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static int histb_vdec_open(struct file *filp)
{
	struct histb_vdec_common *common = histb_vdec_common_of_file(filp);
	const struct histb_vdec_gen *gen = common->gen;
	struct histb_vdec_core_ctx *ctx;
	int ret;

	ctx = kzalloc(gen->ctx_size, GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	histb_vdec_reset_fmts(ctx, common);
	v4l2_fh_init(&ctx->fh, video_devdata(filp));

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(common->m2m_dev, ctx,
					    histb_vdec_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto err_free_fh;
	}

	ret = histb_vdec_init_ctrls(ctx);
	if (ret)
		goto err_release_m2m;

	if (gen->open) {
		ret = gen->open(ctx);
		if (ret)
			goto err_free_ctrls;
	}

	v4l2_fh_add(&ctx->fh, filp);
	return 0;

err_free_ctrls:
	v4l2_ctrl_handler_free(&ctx->ctrl_hdl);
err_release_m2m:
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
err_free_fh:
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return ret;
}

static int histb_vdec_release(struct file *filp)
{
	struct histb_vdec_core_ctx *ctx = histb_vdec_ctx_of_file(filp);
	struct histb_vdec_common *common = histb_vdec_common_of_ctx(ctx);

	mutex_lock(&common->vdev_lock);
	v4l2_fh_del(&ctx->fh, filp);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	if (common->gen->release)
		common->gen->release(ctx);
	v4l2_ctrl_handler_free(&ctx->ctrl_hdl);
	v4l2_fh_exit(&ctx->fh);
	mutex_unlock(&common->vdev_lock);
	kfree(ctx);

	return 0;
}

static const struct v4l2_file_operations histb_vdec_fops = {
	.owner = THIS_MODULE,
	.open = histb_vdec_open,
	.release = histb_vdec_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

/* ------------------------------------------------------------- requests */

static int histb_vdec_request_validate(struct media_request *req)
{
	struct media_request_object *obj;
	struct vb2_buffer *out = NULL;
	struct v4l2_ctrl_handler *req_hdl;
	struct histb_vdec_common *common;
	struct histb_vdec_core_ctx *ctx;
	const u32 *required;
	unsigned int count, num, i;

	count = vb2_request_buffer_cnt(req);
	/*
	 * A request drives one decode job, and a job is the pair of one OUTPUT
	 * and one CAPTURE buffer, so the request must bind an OUTPUT buffer to
	 * be queueable at all and never more than one: the engine is programmed
	 * through a single stream base and one slice chain per run, so two
	 * coded buffers cannot be mapped onto the one job this request
	 * schedules.
	 */
	if (!count)
		return -ENOENT;
	if (count > 1)
		return -EINVAL;

	list_for_each_entry(obj, &req->objects, list) {
		struct vb2_buffer *vb;

		if (!vb2_request_object_is_buffer(obj))
			continue;
		vb = container_of(obj, struct vb2_buffer, req_obj);
		if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type)) {
			out = vb;
			break;
		}
	}
	if (!out)
		return -ENOENT;

	ctx = vb2_get_drv_priv(out->vb2_queue);
	common = histb_vdec_common_of_ctx(ctx);
	required = common->gen->required_ctrls ?
		   common->gen->required_ctrls(ctx, &num) : NULL;
	if (required) {
		req_hdl = v4l2_ctrl_request_hdl_find(req, &ctx->ctrl_hdl);
		if (!req_hdl)
			return -ENOENT;
		for (i = 0; i < num; i++) {
			if (v4l2_ctrl_request_hdl_ctrl_find(req_hdl,
							    required[i]))
				continue;
			v4l2_ctrl_request_hdl_put(req_hdl);
			return -ENOENT;
		}
		v4l2_ctrl_request_hdl_put(req_hdl);
	}

	return vb2_request_validate(req);
}

static const struct media_device_ops histb_vdec_media_ops = {
	.req_validate = histb_vdec_request_validate,
	.req_queue = v4l2_m2m_request_queue,
};

/* ------------------------------------------------------------- v4l2 init */

int histb_vdec_core_init(struct histb_vdec_common *common,
			 struct platform_device *pdev,
			 const struct histb_vdec_gen *gen)
{
	struct video_device *vdev;
	int ret;

	common->dev = &pdev->dev;
	common->gen = gen;
	mutex_init(&common->vdev_lock);
	spin_lock_init(&common->job_lock);

	ret = v4l2_device_register(&pdev->dev, &common->v4l2_dev);
	if (ret)
		return ret;

	common->v4l2_dev.mdev = &common->mdev;
	strscpy(common->mdev.driver_name, gen->name,
		sizeof(common->mdev.driver_name));
	strscpy(common->mdev.model, gen->chip->model,
		sizeof(common->mdev.model));
	snprintf(common->mdev.bus_info, sizeof(common->mdev.bus_info),
		 "platform:%s", dev_name(&pdev->dev));
	media_device_init(&common->mdev);
	common->mdev.dev = &pdev->dev;
	common->mdev.ops = &histb_vdec_media_ops;

	common->m2m_dev = v4l2_m2m_init(gen->m2m_ops);
	if (IS_ERR(common->m2m_dev)) {
		ret = PTR_ERR(common->m2m_dev);
		common->m2m_dev = NULL;
		v4l2_err(&common->v4l2_dev, "failed to init mem2mem device\n");
		goto err_unregister;
	}

	vdev = video_device_alloc();
	if (!vdev) {
		ret = -ENOMEM;
		goto err_m2m;
	}
	common->vdev = vdev;

	vdev->v4l2_dev = &common->v4l2_dev;
	vdev->fops = &histb_vdec_fops;
	vdev->ioctl_ops = &histb_vdec_ioctl_ops;
	vdev->lock = &common->vdev_lock;
	vdev->release = video_device_release;
	vdev->vfl_dir = VFL_DIR_M2M;
	vdev->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	strscpy(vdev->name, gen->name, sizeof(vdev->name));
	video_set_drvdata(vdev, common);

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		v4l2_err(&common->v4l2_dev, "failed to register video device\n");
		video_device_release(vdev);
		common->vdev = NULL;
		goto err_m2m;
	}

	/*
	 * The mem2mem core creates the decoder entity and the device-node link;
	 * without them the media node has an empty topology and userspace cannot
	 * tie /dev/video0 to /dev/media0.
	 */
	ret = v4l2_m2m_register_media_controller(common->m2m_dev, vdev,
						 MEDIA_ENT_F_PROC_VIDEO_DECODER);
	if (ret) {
		v4l2_err(&common->v4l2_dev, "failed to register media controller\n");
		goto err_unregister_vdev;
	}

	ret = media_device_register(&common->mdev);
	if (ret) {
		dev_err(common->dev, "failed to register media device\n");
		goto err_unregister_mc;
	}

	return 0;

err_unregister_mc:
	v4l2_m2m_unregister_media_controller(common->m2m_dev);
err_unregister_vdev:
	histb_vdec_core_stop(common);
err_m2m:
	if (common->m2m_dev)
		v4l2_m2m_release(common->m2m_dev);
err_unregister:
	media_device_cleanup(&common->mdev);
	v4l2_device_unregister(&common->v4l2_dev);
	return ret;
}

void histb_vdec_core_stop(struct histb_vdec_common *common)
{
	mutex_lock(&common->mdev.req_queue_mutex);
	mutex_lock(&common->vdev_lock);
	WRITE_ONCE(common->removing, true);
	if (common->vdev)
		video_unregister_device(common->vdev);
	if (common->m2m_dev)
		v4l2_m2m_suspend(common->m2m_dev);
	mutex_unlock(&common->vdev_lock);
	mutex_unlock(&common->mdev.req_queue_mutex);
}

void histb_vdec_core_exit(struct histb_vdec_common *common)
{
	media_device_unregister(&common->mdev);
	if (common->m2m_dev) {
		v4l2_m2m_unregister_media_controller(common->m2m_dev);
		v4l2_m2m_release(common->m2m_dev);
	}
	media_device_cleanup(&common->mdev);
	v4l2_device_unregister(&common->v4l2_dev);
}

/* -------------------------------------------------------------- src dma */

void *histb_vdec_src_va(struct vb2_buffer *vb)
{
	void *va = vb2_plane_vaddr(vb, 0);

	return va ? va + vb->planes[0].data_offset : NULL;
}

dma_addr_t histb_vdec_src_dma(struct vb2_buffer *vb)
{
	return vb2_dma_contig_plane_dma_addr(vb, 0) + vb->planes[0].data_offset;
}

int histb_vdec_src_cpu_begin(struct vb2_buffer *vb)
{
	struct dma_buf *dbuf = vb->planes[0].dbuf;

	/*
	 * The bitstream may have been written by whoever exported the plane, and
	 * videobuf2 does not bracket a driver's CPU use of it, so the window
	 * belongs here. A plane this device allocated itself carries no dma_buf
	 * and has nothing to sync.
	 */
	if (dbuf)
		return dma_buf_begin_cpu_access(dbuf, DMA_FROM_DEVICE);

	return 0;
}

void histb_vdec_src_cpu_end(struct vb2_buffer *vb)
{
	struct dma_buf *dbuf = vb->planes[0].dbuf;

	if (dbuf)
		dma_buf_end_cpu_access(dbuf, DMA_FROM_DEVICE);
}

/* ---------------------------------------------------------------- driver */

static int histb_vdec_probe(struct platform_device *pdev)
{
	const struct histb_vdec_variant *variant;

	variant = of_device_get_match_data(&pdev->dev);
	if (!variant || !variant->probe)
		return -ENODEV;

	return variant->probe(pdev, variant->gen);
}

static void histb_vdec_remove(struct platform_device *pdev)
{
	const struct histb_vdec_variant *variant;

	variant = of_device_get_match_data(&pdev->dev);
	if (variant && variant->remove)
		variant->remove(pdev);
}

static int histb_vdec_runtime_suspend(struct device *dev)
{
	const struct histb_vdec_variant *variant =
		of_device_get_match_data(dev);

	if (variant && variant->runtime_suspend)
		return variant->runtime_suspend(dev);
	return 0;
}

static int histb_vdec_runtime_resume(struct device *dev)
{
	const struct histb_vdec_variant *variant =
		of_device_get_match_data(dev);

	if (variant && variant->runtime_resume)
		return variant->runtime_resume(dev);
	return 0;
}

static int histb_vdec_suspend(struct device *dev)
{
	const struct histb_vdec_variant *variant =
		of_device_get_match_data(dev);

	if (variant && variant->system_suspend)
		return variant->system_suspend(dev);
	return 0;
}

static int histb_vdec_resume(struct device *dev)
{
	const struct histb_vdec_variant *variant =
		of_device_get_match_data(dev);

	if (variant && variant->system_resume)
		return variant->system_resume(dev);
	return 0;
}

static const struct dev_pm_ops histb_vdec_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(histb_vdec_suspend, histb_vdec_resume)
	SET_RUNTIME_PM_OPS(histb_vdec_runtime_suspend, histb_vdec_runtime_resume,
			   NULL)
};

static const struct of_device_id histb_vdec_of_match[] = {
	{ .compatible = "hisilicon,hi3798mv100-vdec",
	  .data = &histb_vdec_v2_mv100_variant },
	{ .compatible = "hisilicon,hi3716cv200-vdec",
	  .data = &histb_vdec_v2_cv200_variant },
	{ .compatible = "hisilicon,hi3798mv300-vdec",
	  .data = &histb_vdec_v2_mv300_variant },
	{ .compatible = "hisilicon,hi3798mv200-vdec",
	  .data = &histb_vdec_v5_mv200h_variant },
	{ }
};
MODULE_DEVICE_TABLE(of, histb_vdec_of_match);

static struct platform_driver histb_vdec_driver = {
	.probe = histb_vdec_probe,
	.remove = histb_vdec_remove,
	.driver = {
		.name = "histb-vdec",
		.of_match_table = histb_vdec_of_match,
		.pm = pm_ptr(&histb_vdec_pm_ops),
	},
};
module_platform_driver(histb_vdec_driver);

MODULE_AUTHOR("David Yang <mmyangfl@gmail.com>");
MODULE_DESCRIPTION("HiSilicon HiSTB stateless video decoder driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("DMA_BUF");
