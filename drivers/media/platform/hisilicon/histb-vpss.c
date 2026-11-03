// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * histb-vpss.c - HiSilicon HiSTB video processing subsystem, mem2mem form.
 *
 * The block sits between a decoder and an encoder or a display: it converts
 * formats, scales and de-interlaces a frame. Everything the block can do
 * that has no V4L2 interface on a mem2mem device (tone mapping, temporal noise
 * reduction, a second output port, a write-back capture channel) is
 * deliberately not implemented: what this driver
 * offers is the format conversion, the two-axis scaling implied by the OUTPUT
 * and CAPTURE sizes, and the deinterlacing mode control.
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

#define HISTB_VPSS_NAME		"histb-vpss"

#define VPSS_MIN_WIDTH		32
#define VPSS_MIN_HEIGHT		32
#define VPSS_MAX_WIDTH		1920
#define VPSS_MAX_HEIGHT		1088
#define VPSS_ALIGN		16

/* Deinterlacing modes offered through V4L2_CID_DEINTERLACING_MODE. */
#define VPSS_DEI_NONE		0
#define VPSS_DEI_BOB		1
#define VPSS_DEI_WEAVE		2

struct histb_vpss_fmt {
	u32	fourcc;
	bool	nv21;
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
	void			*shadow;	/* register image the block runs */
	dma_addr_t		shadow_dma;
	void			*coef;		/* scaler phase tables for one bucket */
	dma_addr_t		coef_dma;
	bool			busy;
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
	bool		invert_uv;
	bool		hflip;
	bool		vflip;
	unsigned int	crop_x;
	unsigned int	crop_y;
	unsigned int	crop_w;
	unsigned int	crop_h;
};

static const struct histb_vpss_fmt histb_vpss_out_fmts[] = {
	{ V4L2_PIX_FMT_NV12, false },
	{ V4L2_PIX_FMT_NV21, true },
};

static const struct histb_vpss_fmt histb_vpss_cap_fmts[] = {
	{ V4L2_PIX_FMT_NV12, false },
	{ V4L2_PIX_FMT_NV21, true },
};

static void histb_vpss_reset(struct histb_vpss *vpss);
static void histb_vpss_reset_crop(struct histb_vpss_ctx *ctx);

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

static void histb_vpss_fill_fmt(const struct histb_vpss_fmt *f,
				struct v4l2_pix_format *pix)
{
	unsigned int width = clamp_t(u32, ALIGN(pix->width, VPSS_ALIGN),
				     VPSS_MIN_WIDTH, VPSS_MAX_WIDTH);
	unsigned int height = clamp_t(u32, ALIGN(pix->height, VPSS_ALIGN),
				      VPSS_MIN_HEIGHT, VPSS_MAX_HEIGHT);
	unsigned int stride;

	pix->width = width;
	pix->height = height;
	pix->pixelformat = f->fourcc;
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = V4L2_COLORSPACE_SMPTE170M;

	stride = histb_vpss_stride(width, pix->bytesperline);
	pix->bytesperline = stride;
	pix->sizeimage = stride * height * 3 / 2;
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

	histb_vpss_fill_fmt(fmt, &f->fmt.pix);
	return 0;
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
	} else {
		ctx->dst_fmt = f->fmt.pix;
		histb_vpss_reset_crop(ctx);
	}
	return 0;
}

static void histb_vpss_reset_crop(struct histb_vpss_ctx *ctx)
{
	ctx->crop.left = 0;
	ctx->crop.top = 0;
	ctx->crop.width = ctx->dst_fmt.width;
	ctx->crop.height = ctx->dst_fmt.height;
}

static int histb_vpss_g_selection(struct file *file, void *priv,
				  struct v4l2_selection *sel)
{
	struct histb_vpss_ctx *ctx = histb_vpss_ctx_of(file);

	if (!V4L2_TYPE_IS_CAPTURE(sel->type))
		return -EINVAL;
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r = ctx->crop;
		return 0;
	default:
		return -EINVAL;
	}
}

/*
 * The block accepts a crop rectangle only when it covers the whole frame: a
 * smaller window leaves the pipeline waiting for a completion that never
 * arrives, and the
 * vendor's own port configuration never asks for one. A client that wants a
 * smaller picture scales into a smaller CAPTURE size instead.
 */
static int histb_vpss_s_selection(struct file *file, void *priv,
				  struct v4l2_selection *sel)
{
	struct histb_vpss_ctx *ctx = histb_vpss_ctx_of(file);
	struct v4l2_rect r = sel->r;

	if (!V4L2_TYPE_IS_CAPTURE(sel->type) ||
	    sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;
	if (r.left || r.top || r.width != ctx->dst_fmt.width ||
	    r.height != ctx->dst_fmt.height)
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

static void histb_vpss_load_coef(struct histb_vpss *vpss, unsigned int b,
				 dma_addr_t addr[4])
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
	static const unsigned int words[4] = { 51, 51, 34, 34 };
	const u32 *tab[4] = { lh[b], lv[b], ch[b], cv[b] };
	unsigned int i;

	for (i = 0; i < 4; i++) {
		memcpy(vpss->coef + i * 256, tab[i], words[i] * 4);
		addr[i] = vpss->coef_dma + i * 256;
	}
}

/*
 * A frame is a pass over one register image; the block transforms the luma
 * and the chroma planes separately (img_pro_mode selects which one).
 */
struct histb_vpss_pass {
	dma_addr_t	in;
	dma_addr_t	out;
	unsigned int	in_w;
	unsigned int	in_h;
	unsigned int	out_w;
	unsigned int	out_h;
	unsigned int	in_stride;
	unsigned int	out_stride;
	bool		invert_uv;
	bool		hflip;
	bool		vflip;
	unsigned int	crop_x;
	unsigned int	crop_y;
	unsigned int	crop_w;
	unsigned int	crop_h;
};

static int histb_vpss_pass_run(struct histb_vpss *vpss,
			       const struct histb_vpss_pass *p)
{
	u32 *s = vpss->shadow;
	unsigned int src_c, dst_c, stride;
	unsigned long deadline;
	u32 st;

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

	s[VPSS_CTRL2 / 4] = FIELD_PREP(VPSS_CTRL2_IN_FORMAT, 0) |
			   FIELD_PREP(VPSS_CTRL2_VHD_FORMAT, 0);
	s[VPSS_IMGSIZE / 4] = FIELD_PREP(VPSS_IMGSIZE_WIDTH, p->in_w - 1) |
			     FIELD_PREP(VPSS_IMGSIZE_HEIGHT, p->in_h - 1);

	src_c = lower_32_bits(p->in) +
		p->in_stride * p->in_h;
	s[VPSS_CURYADDR / 4] = lower_32_bits(p->in);
	s[VPSS_CURCADDR / 4] = src_c;
	stride = p->in_stride;	/* both layouts take the pitch in bytes */
	s[VPSS_CURSTRIDE / 4] = FIELD_PREP(VPSS_CURSTRIDE_Y, stride) |
				FIELD_PREP(VPSS_CURSTRIDE_C, stride);

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

	if (p->out_w != p->in_w || p->out_h != p->in_h) {
		dma_addr_t caddr[4];
		unsigned int b = histb_vpss_zme_bucket(p->in_w, p->out_w);

		histb_vpss_load_coef(vpss, b, caddr);
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
			FIELD_PREP(VPSS_ZME_IN_FMT, 1) |
			FIELD_PREP(VPSS_ZME_OUT_FMT, 1);
	}

	/* A window smaller than the frame leaves the rest to the letterbox
	 * fill, so the background has to be programmed and enabled with it.
	 */
	s[VPSS_VHDLBA_BK / 4] = FIELD_PREP(GENMASK(15, 8), 0x10) |
				FIELD_PREP(GENMASK(23, 16), 0x80) |
				FIELD_PREP(GENMASK(31, 24), 0x80);
	s[VPSS_VHDLBA_VFPOS / 4] = FIELD_PREP(GENMASK(12, 0), p->crop_x) |
				   FIELD_PREP(GENMASK(28, 16), p->crop_y);
	if (p->crop_w != p->out_w || p->crop_h != p->out_h)
		s[VPSS_CTRL / 4] |= VPSS_CTRL_VHD_LBA_EN;
	s[VPSS_VHDCROP_POS / 4] =
		FIELD_PREP(VPSS_VHDCROP_X, p->crop_x) |
		FIELD_PREP(VPSS_VHDCROP_Y, p->crop_y);
	s[VPSS_VHDCROP_SIZE / 4] =
		VPSS_CROP_EN |
		FIELD_PREP(VPSS_VHDCROP_WIDTH, p->crop_w - 1) |
		FIELD_PREP(VPSS_VHDCROP_HEIGHT, p->crop_h - 1);

	s[VPSS_PNEXT / 4] = 0;
	s[VPSS_START / 4] = 1;
	wmb();

	writel(lower_32_bits(vpss->shadow_dma), vpss->regs + VPSS_PNEXT);
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
		usleep_range(200, 400);
	} while (time_before(jiffies, deadline));

	dev_warn(vpss->dev, "job timed out, intstate %#x\n", st);
	writel(0xffffffff, vpss->regs + VPSS_INTCLR);
	histb_vpss_reset(vpss);
	return -ETIMEDOUT;
}

static void histb_vpss_pass_fill(struct histb_vpss_pass *p,
				 const struct histb_vpss_job *job)
{
	p->in = job->src;
	p->out = job->dst;
	p->in_w = job->width;
	p->in_h = job->height;
	p->out_w = job->out_width;
	p->out_h = job->out_height;
	p->in_stride = job->src_stride;
	p->out_stride = job->dst_stride;
	p->invert_uv = job->invert_uv;
	p->hflip = job->hflip;
	p->vflip = job->vflip;
	p->crop_x = job->crop_x;
	p->crop_y = job->crop_y;
	p->crop_w = job->crop_w;
	p->crop_h = job->crop_h;
}

static int histb_vpss_run(struct histb_vpss *vpss,
			  const struct histb_vpss_job *job)
{
	struct histb_vpss_pass p = { 0 };

	histb_vpss_pass_fill(&p, job);
	return histb_vpss_pass_run(vpss, &p);
}

static void histb_vpss_device_run(void *priv)
{
	struct histb_vpss_ctx *ctx = priv;
	struct histb_vpss *vpss = ctx->vpss;
	struct vb2_v4l2_buffer *src, *dst;
	struct histb_vpss_job job = { 0 };
	unsigned int dei_mode = VPSS_DEI_NONE;
	int ret = -EINVAL;

	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	if (!src || !dst)
		goto finish;

	job.src = vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0);
	job.dst = vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0);
	job.width = ctx->src_fmt.width;
	job.height = ctx->src_fmt.height;
	job.out_width = ctx->dst_fmt.width;
	job.out_height = ctx->dst_fmt.height;
	job.src_stride = ctx->src_fmt.bytesperline;
	job.dst_stride = ctx->dst_fmt.bytesperline;
	job.invert_uv = histb_vpss_find(histb_vpss_out_fmts,
					ARRAY_SIZE(histb_vpss_out_fmts),
					ctx->src_fmt.pixelformat)->nv21 !=
			histb_vpss_find(histb_vpss_cap_fmts,
					ARRAY_SIZE(histb_vpss_cap_fmts),
					ctx->dst_fmt.pixelformat)->nv21;
	dei_mode = ctx->dei_ctrl ? ctx->dei_ctrl->val : VPSS_DEI_NONE;
	job.crop_x = ctx->crop.left;
	job.crop_y = ctx->crop.top;
	job.crop_w = ctx->crop.width;
	job.crop_h = ctx->crop.height;
	/* Bob doubles one field vertically through the scaler, so it only
	 * exists when the buffer is a field and the CAPTURE rectangle is twice
	 * its height; asking for it any other way is a mistake worth a warning
	 * rather than a silent plain conversion.
	 */
	if (dei_mode == VPSS_DEI_BOB &&
	    (ctx->dst_fmt.height != job.height * 2 ||
	     (src->field != V4L2_FIELD_TOP &&
	      src->field != V4L2_FIELD_BOTTOM))) {
		dev_warn_ratelimited(vpss->dev,
				     "bob needs a field and twice its height\n");
		goto finish;
	}
	job.hflip = ctx->hflip && ctx->hflip->val;
	job.vflip = ctx->vflip && ctx->vflip->val;

	ret = histb_vpss_run(vpss, &job);
	if (ret)
		dev_warn_ratelimited(vpss->dev, "job failed: %d\n", ret);

finish:
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
				dst->vb2_buf.timestamp = src->vb2_buf.timestamp;
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
	"Off", "Bob", "Weave", NULL,
};

static int histb_vpss_ctrls_create(struct histb_vpss_ctx *ctx)
{
	struct v4l2_ctrl_handler *hdl = &ctx->ctrl_handler;

	v4l2_ctrl_handler_init(hdl, 3);
	ctx->hflip = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_HFLIP, 0, 1, 1, 0);
	ctx->vflip = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_VFLIP, 0, 1, 1, 0);
	ctx->dei_ctrl = v4l2_ctrl_new_std_menu_items(hdl, NULL,
						     V4L2_CID_DEINTERLACING_MODE,
						     VPSS_DEI_WEAVE, 0,
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

	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
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

static void histb_vpss_reset(struct histb_vpss *vpss)
{
	reset_control_assert(vpss->rst);
	udelay(1);
	reset_control_deassert(vpss->rst);
}

static int histb_vpss_probe(struct platform_device *pdev)
{
	struct histb_vpss *vpss;
	int ret;

	vpss = devm_kzalloc(&pdev->dev, sizeof(*vpss), GFP_KERNEL);
	if (!vpss)
		return -ENOMEM;
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

	vpss->shadow = dma_alloc_coherent(&pdev->dev, VPSS_REG_IMAGE_SIZE,
					  &vpss->shadow_dma, GFP_KERNEL);
	if (!vpss->shadow)
		return -ENOMEM;
	vpss->coef = dma_alloc_coherent(&pdev->dev, 4 * 256, &vpss->coef_dma,
					GFP_KERNEL);
	if (!vpss->coef)
		return -ENOMEM;

	ret = clk_prepare_enable(vpss->clk);
	if (ret)
		return ret;
	histb_vpss_reset(vpss);

	ret = v4l2_device_register(&pdev->dev, &vpss->v4l2_dev);
	if (ret)
		goto err_clk;

	vpss->m2m_dev = v4l2_m2m_init(&histb_vpss_m2m_ops);
	if (IS_ERR(vpss->m2m_dev)) {
		ret = PTR_ERR(vpss->m2m_dev);
		goto err_v4l2;
	}

	vpss->vdev = histb_vpss_videodev;
	vpss->vdev.v4l2_dev = &vpss->v4l2_dev;
	vpss->vdev.lock = &vpss->lock;
	video_set_drvdata(&vpss->vdev, vpss);

	ret = video_register_device(&vpss->vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(&pdev->dev, "cannot register video device\n");
		goto err_m2m;
	}

	platform_set_drvdata(pdev, vpss);
	dev_info(&pdev->dev, "video processor, %d x %d max\n",
		 VPSS_MAX_WIDTH, VPSS_MAX_HEIGHT);
	return 0;

err_m2m:
	v4l2_m2m_release(vpss->m2m_dev);
err_v4l2:
	v4l2_device_unregister(&vpss->v4l2_dev);
err_clk:
	clk_disable_unprepare(vpss->clk);
	return ret;
}

static void histb_vpss_remove(struct platform_device *pdev)
{
	struct histb_vpss *vpss = platform_get_drvdata(pdev);

	video_unregister_device(&vpss->vdev);
	v4l2_m2m_release(vpss->m2m_dev);
	v4l2_device_unregister(&vpss->v4l2_dev);
	clk_disable_unprepare(vpss->clk);
	dma_free_coherent(vpss->dev, VPSS_REG_IMAGE_SIZE, vpss->shadow,
			  vpss->shadow_dma);
	dma_free_coherent(vpss->dev, 4 * 256, vpss->coef, vpss->coef_dma);
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
		.of_match_table = histb_vpss_of_match,
	},
};
module_platform_driver(histb_vpss_driver);

MODULE_DESCRIPTION("HiSilicon HiSTB video processing subsystem driver");
MODULE_LICENSE("Dual MIT/GPL");
