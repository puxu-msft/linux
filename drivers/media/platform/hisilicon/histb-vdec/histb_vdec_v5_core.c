// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon HiSTB V5R6 stateless decoder backend.
 *
 * This generation decodes out of a command-image pool: the protocol sources
 * pack the message slots, the launch programs the core and the job thread
 * waits for the interrupt the platform driver owns. The V4L2 node, the queues,
 * the format negotiation and the request validation live in histb_vdec_core.c.
 */

#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/media.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <media/media-device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

#include "histb_vdec_v5.h"

/* The core casts the allocation to its prefix; both layouts must agree. */
static_assert(offsetof(struct histb_vdec_v5_ctx, fh) == offsetof(struct histb_vdec_core_ctx, fh));
static_assert(offsetof(struct histb_vdec_v5_ctx, ctrl_hdl) ==
	      offsetof(struct histb_vdec_core_ctx, ctrl_hdl));
static_assert(offsetof(struct histb_vdec_v5_ctx, desc) ==
	      offsetof(struct histb_vdec_core_ctx, desc));
static_assert(offsetof(struct histb_vdec_v5_ctx, out_fmt) ==
	      offsetof(struct histb_vdec_core_ctx, out_fmt));
static_assert(offsetof(struct histb_vdec_v5_ctx, cap_fmt) ==
	      offsetof(struct histb_vdec_core_ctx, cap_fmt));

/*
 * The core hands v4l2 the common prefix and this backend casts it back, which
 * holds while that prefix sits at offset zero.
 */
static_assert(offsetof(struct histb_vdec_v5, common) == 0);


static bool histb_vdec_v5_frames_empty(const struct histb_vdec_v5_ctx *ctx);

static const struct histb_vdec_v5_codec_ops *
histb_vdec_v5_ops(const void *p)
{
	const struct histb_vdec_core_ctx *ctx = p;

	return ctx->desc->ops;
}

struct histb_vdec_v5 *histb_vdec_v5_of_ctx(const struct histb_vdec_v5_ctx *ctx)
{
	return video_get_drvdata(ctx->fh.vdev);
}

/*
 * This generation is one part, so the protocol list the node negotiates is the
 * one the block was built for, and its first entry is the fourcc a freshly
 * opened file starts on.
 */
static const histb_vdec_v5_fmt_desc *const histb_vdec_v5_fmt_list[] = {
	&histb_vdec_v5_h264_fmt,
	&histb_vdec_v5_hevc_fmt,
};

static const struct histb_vdec_chip histb_vdec_v5_chip_mv200h = {
	.model = "HI3798MV200H VDH",
	.fmts = histb_vdec_v5_fmt_list,
	.num_fmts = ARRAY_SIZE(histb_vdec_v5_fmt_list),
};

unsigned int histb_vdec_v5_frame_size(const struct histb_vdec_v5_ctx *ctx,
				    unsigned int width, unsigned int height)
{
	return V5R6_BAND_PITCH(width) *
	       ALIGN(ALIGN(height, ctx->desc->grid_align), 32) * 3 / 2;
}

unsigned int histb_vdec_v5_linear_size(unsigned int width, unsigned int height)
{
	return ALIGN(width, 16) * height * 3 / 2;
}

void histb_vdec_v5_buf_geom(const struct histb_vdec_v5_ctx *ctx,
			  unsigned int *pitch, unsigned int *lines,
			  unsigned int *uv_offset)
{
	const struct v4l2_pix_format_mplane *pix = histb_vdec_cap_pix(ctx);

	*pitch = V5R6_BAND_PITCH(pix->width);
	*lines = ALIGN(pix->height, ctx->desc->grid_align);
	*uv_offset = *pitch * ALIGN(*lines, 32);
}

static int histb_vdec_v5_buf_init(struct vb2_buffer *vb)
{
	struct histb_vdec_v5_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct histb_vdec_v5 *dec = histb_vdec_v5_of_ctx(ctx);
	struct histb_vdec_v5_frame *frame;
	unsigned int pitch, lines, uv_offset;

	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type))
		return 0;

	frame = &ctx->frames[vb->index];
	frame->index = vb->index;
	histb_vdec_v5_buf_geom(ctx, &pitch, &lines, &uv_offset);
	frame->pitch = pitch;
	frame->height = lines;

	/*
	 * The core writes the tiled surface; the buffer userspace owns is the
	 * linear frame the completion path de-tiles into.
	 */
	frame->size = histb_vdec_v5_frame_size(ctx, histb_vdec_cap_pix(ctx)->width,
					     histb_vdec_cap_pix(ctx)->height);
	frame->va = dma_alloc_coherent(dec->dev, frame->size, &frame->dma,
				       GFP_KERNEL);
	if (!frame->va)
		return -ENOMEM;

	/* The slot ids are the protocol's to give, when a message needs them. */
	frame->used = true;

	return 0;
}

static void histb_vdec_v5_buf_cleanup(struct vb2_buffer *vb)
{
	struct histb_vdec_v5_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type))
		return;

	if (ctx->desc && histb_vdec_v5_ops(ctx)->buf_release)
		histb_vdec_v5_ops(ctx)->buf_release(ctx, vb->index);
	if (ctx->frames[vb->index].va) {
		struct histb_vdec_v5 *dec = histb_vdec_v5_of_ctx(ctx);

		dma_free_coherent(dec->dev, ctx->frames[vb->index].size,
				  ctx->frames[vb->index].va,
				  ctx->frames[vb->index].dma);
	}
	ctx->frames[vb->index] = (struct histb_vdec_v5_frame){};

	if (histb_vdec_v5_frames_empty(ctx)) {
		ctx->apc_next = 0;
		ctx->pmv_next = 0;
	}
}

struct histb_vdec_v5_frame *
histb_vdec_v5_frame_of(struct histb_vdec_v5_ctx *ctx, unsigned int index)
{
	if (index >= V5R6_MAX_FRAMES || !ctx->frames[index].used)
		return NULL;

	return &ctx->frames[index];
}

static bool histb_vdec_v5_frames_empty(const struct histb_vdec_v5_ctx *ctx)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ctx->frames); i++) {
		if (ctx->frames[i].used)
			return false;
	}

	return true;
}

static unsigned int histb_vdec_v5_coded_mbs(const struct histb_vdec_v5_ctx *ctx)
{
	const struct v4l2_pix_format_mplane *pix = histb_vdec_cap_pix(ctx);

	return (pix->width / 16) *
	       (ALIGN(pix->height, ctx->desc->grid_align) / 16);
}

static bool histb_vdec_v5_job_take(struct histb_vdec_common *common,
				 struct histb_vdec_job *out)
{
	unsigned long flags;

	spin_lock_irqsave(&common->job_lock, flags);
	if (!common->job_running) {
		spin_unlock_irqrestore(&common->job_lock, flags);
		return false;
	}
	*out = common->job;
	common->job_running = false;
	common->job = (struct histb_vdec_job){};
	spin_unlock_irqrestore(&common->job_lock, flags);
	return true;
}

static void histb_vdec_v5_job_finish(struct histb_vdec_v5 *dec,
				   const struct histb_vdec_job *job,
				   enum vb2_buffer_state state)
{
	v4l2_m2m_buf_done_and_job_finish(dec->common.m2m_dev, job->ctx->fh.m2m_ctx, state);
	pm_runtime_put(dec->dev);
}

static void histb_vdec_v5_job_failed(struct histb_vdec_v5 *dec,
				   const struct histb_vdec_job *job)
{
	struct histb_vdec_common *common = &dec->common;
	struct vb2_v4l2_buffer *dst;

	cancel_delayed_work(&common->watchdog);

	/*
	 * A launch that was rung but did not report leaves the core writing
	 * into the surface this job named, so the next job would be programmed
	 * against a busy pipeline. The pulse costs nothing on an idle core,
	 * which is what a run() that refused before the doorbell leaves behind.
	 */
	histb_vdec_v5_abort(dec);

	/*
	 * Nothing was copied out of the internal surface, so the capture buffer
	 * holds the picture before it, and the ERROR flag alone cannot tell that
	 * apart: an empty payload says the engine delivered nothing, the same
	 * rule the older backend's refusals follow.
	 */
	dst = v4l2_m2m_next_dst_buf(job->ctx->fh.m2m_ctx);
	if (dst)
		vb2_set_plane_payload(&dst->vb2_buf, 0, 0);

	histb_vdec_v5_job_finish(dec, job, VB2_BUF_STATE_ERROR);
}

static void histb_vdec_v5_recover(struct histb_vdec_common *common, const char *why)
{
	struct histb_vdec_v5 *dec = common->priv;
	struct histb_vdec_job job;

	if (!histb_vdec_v5_job_take(common, &job))
		return;

	dev_err(dec->dev,
		"%s: state=%#010x int=%#010x cycles=%u rd=%u wr=%u\n",
		why, dec->last_state, dec->int_state,
		histb_vdec_v5_read(dec, V5R6_DEC_CYCLES),
		histb_vdec_v5_read(dec, V5R6_RD_REQ_PER_PIC),
		histb_vdec_v5_read(dec, V5R6_WR_REQ_PER_PIC));

	histb_vdec_v5_job_failed(dec, &job);
}

void histb_vdec_v5_watchdog(struct work_struct *work)
{
	struct histb_vdec_common *common =
		container_of(to_delayed_work(work), struct histb_vdec_common,
			     watchdog);

	histb_vdec_v5_recover(common, "frame decode timed out");
}

static bool histb_vdec_v5_short_coverage(struct histb_vdec_v5 *dec,
				       const struct histb_vdec_v5_ctx *ctx)
{
	const u32 *rep = histb_vdec_v5_slot(dec, V5R6_SLOT_UP);
	unsigned int i, slices = ctx->mbs ? ctx->slices : 0;
	u32 max_end = 0;

	if (!slices || !ctx->mbs)
		return false;

	for (i = 0; i < slices && i < V5R6_MAX_SLICES; i++)
		max_end = max(max_end, rep[4 * i + 2]);

	if (max_end + 1 == ctx->mbs)
		return false;

	dev_err(dec->dev, "picture reports %u of %u macroblocks\n",
		max_end + 1, ctx->mbs);
	return true;
}

static inline unsigned int histb_vdec_v5_tile_luma(unsigned int row,
						 unsigned int col,
						 unsigned int pitch)
{
	unsigned int band = row / 16, r = row % 16;
	unsigned int tile = col / 256, c = col % 256;
	unsigned int group = c / 64, inner = c % 64;
	unsigned int sub = r / 4, line = r % 4;
	unsigned int block = (4 * group + sub) ^ group;

	return band * pitch * 16 + tile * 4096 + block * 256 + line * 64 + inner;
}

static inline unsigned int histb_vdec_v5_tile_chroma(unsigned int row,
						   unsigned int col,
						   unsigned int pitch,
						   unsigned int uv)
{
	unsigned int band = row / 8, r = row % 8;
	unsigned int byte = col * 2;
	unsigned int tile = byte / 256, c = byte % 256;
	unsigned int group = c / 64, inner = c % 64;
	unsigned int sub = r / 4, line = r % 4;
	unsigned int block = 2 * group + (sub ^ (group & 1));

	return uv + band * pitch * 8 + tile * 2048 + block * 256 + line * 64 +
	       inner;
}

static void histb_vdec_v5_detile_frame(const u8 *src, u8 *dst, unsigned int width,
				     unsigned int height, unsigned int pitch,
				     unsigned int uv, unsigned int stride)
{
	unsigned int row, col;

	for (row = 0; row < height; row++)
		for (col = 0; col < width; col++)
			dst[row * stride + col] =
				src[histb_vdec_v5_tile_luma(row, col, pitch)];
	for (row = 0; row < height / 2; row++)
		for (col = 0; col < width / 2; col++) {
			unsigned int off =
				histb_vdec_v5_tile_chroma(row, col, pitch, uv);
			u8 *p = dst + stride * height + row * stride + 2 * col;

			p[0] = src[off];
			p[1] = src[off + 1];
		}
}

static bool histb_vdec_v5_detile_dst(struct histb_vdec_v5_ctx *ctx,
				   struct vb2_v4l2_buffer *dst)
{
	struct histb_vdec_v5_frame *frame =
		histb_vdec_v5_frame_of(ctx, dst->vb2_buf.index);
	const struct v4l2_pix_format_mplane *pix = histb_vdec_cap_pix(ctx);
	struct dma_buf *dbuf = dst->vb2_buf.planes[0].dbuf;
	unsigned int pitch, lines, uv, stride = ALIGN(pix->width, 16);
	u8 *out;
	int ret;

	if (frame == NULL || frame->va == NULL)
		return false;
	out = vb2_plane_vaddr(&dst->vb2_buf, 0);
	if (out == NULL)
		return false;

	histb_vdec_v5_buf_geom(ctx, &pitch, &lines, &uv);
	if (dbuf) {
		ret = dma_buf_begin_cpu_access(dbuf, DMA_TO_DEVICE);
		if (ret)
			return false;
	}
	histb_vdec_v5_detile_frame(frame->va, out, pix->width, pix->height,
				 pitch, uv, stride);
	if (dbuf)
		dma_buf_end_cpu_access(dbuf, DMA_TO_DEVICE);

	return true;
}

static void histb_vdec_v5_device_run(void *priv)
{
	struct histb_vdec_v5_ctx *ctx = priv;
	struct histb_vdec_v5 *dec = histb_vdec_v5_of_ctx(ctx);
	struct histb_vdec_common *common = &dec->common;
	struct histb_vdec_job job;
	struct vb2_v4l2_buffer *src, *dst;
	unsigned int gen;
	bool failed, short_cov, timed_out;
	unsigned long flags;
	int ret;

	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	if (WARN_ON(!src) || WARN_ON(!dst))
		return;

	if (READ_ONCE(dec->common.removing)) {
		v4l2_ctrl_request_complete(src->vb2_buf.req_obj.req,
					   &ctx->ctrl_hdl);
		vb2_set_plane_payload(&dst->vb2_buf, 0, 0);
		v4l2_m2m_buf_done_and_job_finish(dec->common.m2m_dev, ctx->fh.m2m_ctx,
						 VB2_BUF_STATE_ERROR);
		return;
	}

	ret = pm_runtime_resume_and_get(dec->dev);
	if (ret) {
		dev_dbg(dec->dev, "runtime resume failed: %d\n", ret);
		goto err_no_pm;
	}

	mutex_lock(&dec->hw_lock);

	spin_lock_irqsave(&common->job_lock, flags);
	if (WARN_ON(common->job_running)) {
		spin_unlock_irqrestore(&common->job_lock, flags);
		mutex_unlock(&dec->hw_lock);
		pm_runtime_put(dec->dev);
		return;
	}
	common->job_running = true;
	common->job_gen++;
	common->job = (struct histb_vdec_job){ (struct histb_vdec_core_ctx *)ctx, common->job_gen };
	gen = common->job.gen;
	spin_unlock_irqrestore(&common->job_lock, flags);

	/*
	 * Apply the controls the source buffer's request carries; the message
	 * assembly below consumes their values, so the request object can be
	 * completed as soon as that has run.
	 */
	ret = v4l2_ctrl_request_setup(src->vb2_buf.req_obj.req,
				      &ctx->ctrl_hdl);
	if (ret) {
		dev_dbg(dec->dev, "request setup failed: %d\n", ret);
		goto err_job;
	}

	v4l2_m2m_buf_copy_metadata(src, dst);

	/*
	 * One deadline covers the whole frame. The protocol layer packs the
	 * message slots and launches the core under hw_lock; if the frame that
	 * started is still unreported when the deadline passes, the watchdog
	 * rather than this thread owns its recovery.
	 */
	schedule_delayed_work(&common->watchdog, V5R6_TIMEOUT);

	/*
	 * The protocol layer walks the bitstream with the CPU to pack the
	 * message slots, so the whole of that walk goes inside one access window
	 * on the source plane.
	 */
	ret = histb_vdec_src_cpu_begin(&src->vb2_buf);
	if (ret) {
		dev_dbg(dec->dev, "source cpu access failed: %d\n", ret);
		goto err_job;
	}
	ret = histb_vdec_v5_ops(ctx)->run(ctx, src, dst);
	histb_vdec_src_cpu_end(&src->vb2_buf);
	if (ret) {
		dev_dbg(dec->dev, "run refused: %d (out fourcc %4.4s)\n", ret,
			(char *)&ctx->desc->fourcc);
		/*
		 * A refusal has to be readable without the buffer flag: a
		 * picture the core flagged as failed comes back with
		 * V4L2_BUF_FLAG_ERROR set just like this one, so the payload
		 * length is what says the surface holds nothing. Dropping the
		 * contents is the client's job once bytesused is zero.
		 */
		vb2_set_plane_payload(&dst->vb2_buf, 0, 0);
		goto err_job;
	}

	v4l2_ctrl_request_complete(src->vb2_buf.req_obj.req, &ctx->ctrl_hdl);

	/*
	 * The wait stays inside hw_lock: the message pool, the registers and the
	 * completion are one pipeline, so a second job may not pack its slots or
	 * ring its doorbell until this one has reported.
	 */
	timed_out = wait_for_completion_timeout(&dec->done, V5R6_TIMEOUT) == 0;
	mutex_unlock(&dec->hw_lock);

	if (timed_out) {
		histb_vdec_v5_recover(common, "frame decode timed out");
		return;
	}

	if (!histb_vdec_v5_job_take(common, &job))
		return;

	dec->last_state = histb_vdec_v5_read(dec, V5R6_STATE);
	failed = (dec->int_state & V5R6_INT_DEC_ERR) ||
		 (dec->last_state & V5R6_STATE_DEC_ERR);
	short_cov = histb_vdec_v5_short_coverage(dec, ctx);
	if (!histb_vdec_v5_detile_dst(ctx, dst)) {
		dev_err(dec->dev, "capture buffer has no linear mapping\n");
		vb2_set_plane_payload(&dst->vb2_buf, 0, 0);
		failed = true;
	}

	dev_dbg(dec->dev, "job %u done: int=%#010x state=%#010x short=%u reports=%u\n",
		gen, dec->int_state, dec->last_state, short_cov,
		histb_vdec_v5_read(dec, V5R6_RPT_CNT));

	cancel_delayed_work(&common->watchdog);

	/*
	 * The core keeps no clean state after a picture it flagged, and a
	 * picture whose reports stop short of the last macroblock is in the
	 * same position: the next launch is programmed against areas the
	 * core wrote while the picture was going wrong. Reset here, as the
	 * older engine's driver does.
	 */
	if (failed || short_cov)
		histb_vdec_v5_abort(dec);

	/*
	 * What the client is told follows the older engine's rule: a flagged
	 * picture is delivered as an error even though its pixels are there,
	 * because the flag is the only sign the engine gave that something in
	 * the stream was not consumed as written. A picture that merely fell
	 * short of a macroblock total nobody asked it to report is not.
	 */
	if (failed)
		dev_info(dec->dev,
			 "picture delivered with the core's error flag (int %#x state %#x)\n",
			 dec->int_state, dec->last_state);

	histb_vdec_v5_job_finish(dec, &job,
			       failed ? VB2_BUF_STATE_ERROR :
					VB2_BUF_STATE_DONE);
	return;

err_job:
	v4l2_ctrl_request_complete(src->vb2_buf.req_obj.req, &ctx->ctrl_hdl);
	mutex_unlock(&dec->hw_lock);
	if (histb_vdec_v5_job_take(common, &job))
		histb_vdec_v5_job_failed(dec, &job);
	return;

err_no_pm:
	v4l2_ctrl_request_complete(src->vb2_buf.req_obj.req, &ctx->ctrl_hdl);
	vb2_set_plane_payload(&dst->vb2_buf, 0, 0);
	v4l2_m2m_buf_done_and_job_finish(dec->common.m2m_dev, ctx->fh.m2m_ctx,
					 VB2_BUF_STATE_ERROR);
}

static const struct v4l2_m2m_ops histb_vdec_v5_m2m_ops = {
	.device_run = histb_vdec_v5_device_run,
};


/* ------------------------------------------------------------- gen ops */

static const u32 histb_vdec_v5_cap_formats[] = {
	V4L2_PIX_FMT_NV21,
};

static struct device *histb_vdec_v5_dma_dev(struct histb_vdec_common *common)
{
	return common->dev;
}

static void histb_vdec_v5_set_size(struct histb_vdec_core_ctx *ctx,
				   struct v4l2_format *f,
				   unsigned int width, unsigned int height)
{
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;

	pix->width = width;
	pix->height = height;
	pix->plane_fmt[0].bytesperline = ALIGN(width, 16);
	pix->plane_fmt[0].sizeimage = histb_vdec_v5_linear_size(width, height);
}

static void histb_vdec_v5_geom_hook(struct histb_vdec_core_ctx *cctx,
				    struct histb_vdec_geom *g)
{
	histb_vdec_v5_buf_geom((struct histb_vdec_v5_ctx *)cctx, &g->pitch,
			       &g->lines, &g->uv_offset);
}

static int histb_vdec_v5_buf_prepare_hook(struct histb_vdec_core_ctx *cctx,
					  struct vb2_buffer *vb)
{
	dma_addr_t dma = vb2_dma_contig_plane_dma_addr(vb, 0);

	/*
	 * The core reads for the length the client reported, starting where the
	 * client said its data starts, so that window has to stay inside the
	 * plane. The mem2mem core bounds the report by the plane, not by the
	 * plane left over after the start.
	 */
	if (vb->planes[0].data_offset + vb2_get_plane_payload(vb, 0) >
	    vb2_plane_size(vb, 0))
		return -EINVAL;

	/*
	 * The message slots name the bitstream start in one 32-bit field, so a
	 * plane ending above 4 GiB would be read from a truncated address. The
	 * device's 32-bit DMA mask keeps its own allocations inside that window;
	 * say it here rather than rely on the mask alone.
	 */
	if ((u64)dma + vb2_plane_size(vb, 0) - 1 > DMA_BIT_MASK(32))
		return -ERANGE;

	return 0;
}

static int histb_vdec_v5_buf_init_hook(struct histb_vdec_core_ctx *cctx,
				       struct vb2_buffer *vb)
{
	return histb_vdec_v5_buf_init(vb);
}

static void histb_vdec_v5_buf_cleanup_hook(struct histb_vdec_core_ctx *cctx,
					   struct vb2_buffer *vb)
{
	histb_vdec_v5_buf_cleanup(vb);
}

static int histb_vdec_v5_start(struct histb_vdec_core_ctx *cctx,
			       const struct vb2_queue *q)
{
	struct histb_vdec_v5_ctx *ctx = (struct histb_vdec_v5_ctx *)cctx;

	if (V4L2_TYPE_IS_OUTPUT(q->type))
		return 0;

	/*
	 * The motion-history areas are per reference surface of this stream, so
	 * they come into existence with the capture queue that holds them and
	 * not before: a job can only be scheduled once both queues are
	 * streaming.
	 */
	return histb_vdec_v5_pmv_alloc(histb_vdec_v5_of_ctx(ctx), ctx,
				       histb_vdec_v5_coded_mbs(ctx));
}

static void histb_vdec_v5_stop(struct histb_vdec_core_ctx *cctx,
			       const struct vb2_queue *q)
{
	struct histb_vdec_v5_ctx *ctx = (struct histb_vdec_v5_ctx *)cctx;

	if (V4L2_TYPE_IS_OUTPUT(q->type)) {
		if (histb_vdec_v5_ops(ctx)->stop)
			histb_vdec_v5_ops(ctx)->stop(ctx);
	} else {
		histb_vdec_v5_pmv_free(ctx);
	}
}

static int histb_vdec_v5_open_ctx(struct histb_vdec_core_ctx *cctx)
{
	struct histb_vdec_v5_ctx *ctx = (struct histb_vdec_v5_ctx *)cctx;
	struct histb_vdec_v5 *dec = histb_vdec_v5_of_ctx(ctx);

	atomic_inc(&dec->open_count);
	return 0;
}

static void histb_vdec_v5_release_ctx(struct histb_vdec_core_ctx *cctx)
{
	struct histb_vdec_v5_ctx *ctx = (struct histb_vdec_v5_ctx *)cctx;
	struct histb_vdec_v5 *dec = histb_vdec_v5_of_ctx(ctx);

	histb_vdec_v5_pmv_free(ctx);
	if (atomic_dec_and_test(&dec->open_count))
		wake_up_all(&dec->open_wq);
}

static const u32 *histb_vdec_v5_required_ctrls(
				const struct histb_vdec_core_ctx *ctx,
				unsigned int *num)
{
	*num = ctx->desc->num_req;
	return ctx->desc->req_ctrls ? ctx->desc->req_ctrls : NULL;
}

static int histb_vdec_v5_try_ctrl_hook(const struct histb_vdec_fmt *fmt,
				       struct v4l2_ctrl *ctrl)
{
	const struct histb_vdec_v5_codec_ops *ops = fmt->ops;

	if (ops->try_ctrl)
		return ops->try_ctrl(ctrl);

	return 0;
}

static const struct histb_vdec_gen histb_vdec_v5_gen = {
	.name			= "histb-vdec",
	.ctx_size		= sizeof(struct histb_vdec_v5_ctx),
	.chip			= &histb_vdec_v5_chip_mv200h,
	.max_width		= V5R6_MAX_WIDTH,
	.max_height		= V5R6_MAX_HEIGHT,
	.max_cap_buffers	= V5R6_MAX_DPB_SLOTS,
	.min_queued_buffers	= 4,
	.cap_full_einval	= true,
	.cap_formats		= histb_vdec_v5_cap_formats,
	.num_cap_formats	= ARRAY_SIZE(histb_vdec_v5_cap_formats),
	.dma_dev		= histb_vdec_v5_dma_dev,
	.set_out_size		= histb_vdec_v5_set_size,
	.set_cap_size		= histb_vdec_v5_set_size,
	.set_cap_reset		= histb_vdec_v5_set_size,
	.buf_geom		= histb_vdec_v5_geom_hook,
	.buf_prepare		= histb_vdec_v5_buf_prepare_hook,
	.buf_init		= histb_vdec_v5_buf_init_hook,
	.buf_cleanup		= histb_vdec_v5_buf_cleanup_hook,
	.start			= histb_vdec_v5_start,
	.stop			= histb_vdec_v5_stop,
	.open			= histb_vdec_v5_open_ctx,
	.release		= histb_vdec_v5_release_ctx,
	.required_ctrls		= histb_vdec_v5_required_ctrls,
	.try_ctrl		= histb_vdec_v5_try_ctrl_hook,
	.m2m_ops		= &histb_vdec_v5_m2m_ops,
};

const struct histb_vdec_variant histb_vdec_v5_mv200h_variant = {
	.gen			= &histb_vdec_v5_gen,
	.probe			= histb_vdec_v5_probe,
	.remove			= histb_vdec_v5_remove,
	.runtime_suspend	= histb_vdec_v5_runtime_suspend,
	.runtime_resume		= histb_vdec_v5_runtime_resume,
	.system_suspend		= pm_runtime_force_suspend,
	.system_resume		= pm_runtime_force_resume,
};
