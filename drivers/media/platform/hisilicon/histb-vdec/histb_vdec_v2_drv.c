// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * HiSilicon Hi3798MV100 / Hi3716CV200 / Hi3798MV300 无状态解码后端。
 *
 * This generation decodes out of the VDM message pool: the protocol sources
 * pack the parsed syntax into fixed message slots, the VDM is programmed and
 * the doorbell rung, and a threaded interrupt finishes the frame.  The V4L2
 * node, the queues and the request validation live in histb_vdec_core.c.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>
#include <media/media-device.h>
#include <media/media-request.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

#include "histb_vdec_v2.h"

/* The core casts the allocation to its prefix; both layouts must agree. */
static_assert(offsetof(struct histb_vdec_ctx, fh) == offsetof(struct histb_vdec_core_ctx, fh));
static_assert(offsetof(struct histb_vdec_ctx, ctrl_hdl) ==
	      offsetof(struct histb_vdec_core_ctx, ctrl_hdl));
static_assert(offsetof(struct histb_vdec_ctx, desc) == offsetof(struct histb_vdec_core_ctx, desc));
static_assert(offsetof(struct histb_vdec_ctx, out_fmt) ==
	      offsetof(struct histb_vdec_core_ctx, out_fmt));
static_assert(offsetof(struct histb_vdec_ctx, cap_fmt) ==
	      offsetof(struct histb_vdec_core_ctx, cap_fmt));

/*
 * The core hands v4l2 the common prefix and this backend casts it back, which
 * holds while that prefix sits at offset zero.
 */
static_assert(offsetof(struct histb_vdec_dev, common) == 0);


static const struct histb_vdec_codec_ops *
histb_vdec_v2_ops(const struct histb_vdec_ctx *ctx)
{
	return ctx->desc->ops;
}

/*
 * The codec descriptors are extern objects, so C can not aggregate them into
 * the by-value histb_vdec_fmts[] array the driver contract declares; the list
 * is carried as pointers instead.
 */
static const histb_vdec_fmt_desc *const histb_vdec_fmt_list[] = {
	&histb_vdec_h264_fmt,
	&histb_vdec_hevc_fmt,
	&histb_vdec_mpeg2_fmt,
	&histb_vdec_mpeg4_fmt,
	&histb_vdec_h263_fmt,
};

/*
 * Hi3716CV200 runs an older VDH generation than Hi3798MV100. Its register
 * offsets and every message block this driver programs are the same, but HEVC
 * is missing outright: that generation has no HEVC header at all, its report
 * word keeps the two CTB-size bits reserved, and the chip's own capability
 * list is the Hi3798MV100 list without HEVC. So this list is the one above
 * minus HEVC, and the first entry stays the default fourcc for both.
 */
static const histb_vdec_fmt_desc *const histb_vdec_fmt_list_cv200[] = {
	&histb_vdec_h264_fmt,
	&histb_vdec_mpeg2_fmt,
	&histb_vdec_mpeg4_fmt,
	&histb_vdec_h263_fmt,
};

static const struct histb_vdec_chip histb_vdec_v2_chip_mv100 = {
	.model = "HI3798MV100 VDH",
	.fmts = histb_vdec_fmt_list,
	.num_fmts = ARRAY_SIZE(histb_vdec_fmt_list),
};

static const struct histb_vdec_chip histb_vdec_v2_chip_cv200 = {
	.model = "HI3716CV200 VDH",
	.fmts = histb_vdec_fmt_list_cv200,
	.num_fmts = ARRAY_SIZE(histb_vdec_fmt_list_cv200),
};

/*
 * Hi3798MV300 运行 VDH V5R2C1 这一代（VDH_DIR := HiVDHV5R2C1，固件标签为 HiVDHFMWV500R001）。它保留了 VDM/SCD 寄存器窗口和本后端编程的消息块，所以沿用 Hi3798MV100 的格式列表；目前在它上面逐字节正确解码的协议是 H.264 与 HEVC Main，VP9 与 10-bit 输出尚未接通。
 */
static const struct histb_vdec_chip histb_vdec_v2_chip_mv300 = {
	.model = "HI3798MV300 VDH",
	.fmts = histb_vdec_fmt_list,
	.num_fmts = ARRAY_SIZE(histb_vdec_fmt_list),
};

/*
 * 本后端的一个型号：core 所依据的代描述，加上后端自身需要知道的该型号信息。
 */
struct histb_vdec_v2_part {
	struct histb_vdec_gen	gen;
	bool			raster;	/* 见 histb_vdec_dev::raster */
	bool			direct_capture; /* 见 histb_vdec_dev 同名字段 */
	bool			crg_gates; /* VDH 门控在 CRG30：需要 hisilicon,crg */
	bool			h264_chain_null_end; /* 见 histb_vdec_dev 同名字段 */
	bool			cabac_seg_stop_bit; /* 见 histb_vdec_dev 同名字段 */
	bool			qmatrix_vfmw5; /* 见 histb_vdec_dev 同名字段 */
	bool			mv_out; /* 见 histb_vdec_dev 同名字段 */
	bool			hevc_tiles; /* 见 histb_vdec_dev 同名字段 */
	bool			h264_interlace_vfmw5; /* 见 histb_vdec_dev 同名字段 */
	bool			mpeg4_vfmw5; /* 见 histb_vdec_dev 同名字段 */
	bool			mpeg_cov_reports; /* 见 histb_vdec_dev 同名字段 */
	bool			covered_err_keeps_engine; /* 见 histb_vdec_dev 同名字段 */
};

static struct histb_vdec_dev *histb_vdec_dev_of(const struct histb_vdec_ctx *ctx)
{
	return video_get_drvdata(ctx->fh.vdev);
}

static unsigned int histb_vdec_stride(const struct histb_vdec_ctx *ctx,
				      unsigned int width)
{
	return ctx->desc->stride(width);
}

static bool histb_vdec_direct_capture_fmt(const struct histb_vdec_dev *dev,
					  u32 fourcc)
{
	return dev->direct_capture && fourcc == V4L2_PIX_FMT_NV21;
}

static bool histb_vdec_direct_capture(struct histb_vdec_ctx *ctx)
{
	return histb_vdec_direct_capture_fmt(histb_vdec_dev_of(ctx),
					     ctx->cap_fmt.fmt.pix_mp.pixelformat);
}

/*
 * 色度平面相对亮度起点的偏移。V2/V3 的 tile 帧沿用厂商布局：亮度 band 阵列之后先是每对齐行 16 字节的 head-info 区，再是色度。direct_capture 的型号把色度紧接在亮度之后、不留 head-info 区，capture 才是标准的 NV21（色度从 bytesperline * height 起）。
 *
 * head-info 是厂商为帧压缩准备的区（vfmw_adapter.h 的 head 字段注释为 “YC head info stride when compress is used”，v5 后端同样只在压缩写路径上用它），本驱动不置 VDM_CFG1_COMPRESS_EN，引擎在不开压缩时不写它。Hi3798MV300 哨兵实测：capture 整块填 0xA5 后解码，按厂商布局时 H.264 1080p、HEVC 1080p 与 H.264 2560x1440 的 head-info 区一个字节都没被写；色度紧跟亮度、HEAD_INF_OFFSET 写 0 时三者都只写到色度末尾，6/6 帧逐字节一致。
 */
unsigned int histb_vdec_uv_offset(const struct histb_vdec_dev *dev,
				  unsigned int stride, unsigned int lines)
{
	unsigned int luma = histb_vdec_luma_bytes(stride, lines);

	return dev->direct_capture ? luma :
				     luma + histb_vdec_head_inf_bytes(lines);
}

/* 一帧帧缓冲的字节数，布局见 histb_vdec_uv_offset()。 */
static unsigned int histb_vdec_frame_store_bytes(const struct histb_vdec_dev *dev,
						 unsigned int stride,
						 unsigned int lines)
{
	return dev->direct_capture ?
	       histb_vdec_luma_bytes(stride, lines) / 2 * 3 :
	       histb_vdec_frame_bytes(stride, lines);
}

/* 本帧的写出几何：亮度起点、16 行 band 间距、色度偏移与 head-info 偏移。四个协议共用。 */
void histb_vdec_write_frame_regs(struct histb_vdec_dev *dev, dma_addr_t dst,
				 unsigned int stride, unsigned int lines)
{
	vdm_write(dev, VDM_YSTADDR_1D, lower_32_bits(dst));
	vdm_write(dev, VDM_YSTRIDE_1D, histb_vdec_band_bytes(stride));
	vdm_write(dev, VDM_UVOFFSET_1D, histb_vdec_uv_offset(dev, stride, lines));
	vdm_write(dev, VDM_HEAD_INF_OFFSET,
		  dev->direct_capture ? 0 : histb_vdec_head_inf_bytes(lines));
}

static unsigned int histb_vdec_frame_size(const struct histb_vdec_ctx *ctx,
					  unsigned int width,
					  unsigned int height)
{
	return histb_vdec_frame_store_bytes(histb_vdec_dev_of(ctx),
					    histb_vdec_stride(ctx, width),
					    ALIGN(height, ctx->desc->grid_align));
}

static void histb_vdec_set_frame_size(struct histb_vdec_ctx *ctx,
				      struct v4l2_format *f,
				      unsigned int width, unsigned int height)
{
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;

	pix->width = width;
	pix->height = height;
	pix->plane_fmt[0].bytesperline = histb_vdec_stride(ctx, width);
	pix->plane_fmt[0].sizeimage = histb_vdec_frame_size(ctx, width, height);
}

/*
 * direct_capture 的 capture 几何：bytesperline 是引擎行距（不受客户端要求影响），高度是帧缓冲的行数（按协议的网格再按 32 行对齐，与 histb_vdec_luma_bytes() 一致，所以可以超过 ENUM_FRAMESIZES 报的最大编码高度，例如 HEVC 2560x1440 为 1472 行），色度从 bytesperline * height 起。可见区由 G_SELECTION 的 COMPOSE 给出（OUTPUT 上声明的编码尺寸）。
 */
static void histb_vdec_set_direct_size(struct histb_vdec_ctx *ctx,
				       struct v4l2_format *f,
				       unsigned int width, unsigned int height)
{
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	unsigned int stride = histb_vdec_stride(ctx, width);
	unsigned int lines = ALIGN(ALIGN(height, ctx->desc->grid_align), 32);

	pix->width = width;
	pix->height = lines;
	pix->plane_fmt[0].bytesperline = stride;
	pix->plane_fmt[0].sizeimage =
		histb_vdec_frame_store_bytes(histb_vdec_dev_of(ctx), stride, lines);
}

static void histb_vdec_set_cap_size(struct histb_vdec_ctx *ctx,
				    struct v4l2_format *f,
				    unsigned int width, unsigned int height)
{
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	unsigned int bpl;

	if (histb_vdec_direct_capture_fmt(histb_vdec_dev_of(ctx),
					  pix->pixelformat)) {
		histb_vdec_set_direct_size(ctx, f, width, height);
		return;
	}

	bpl = max_t(u32, ALIGN(width, 16),
		    round_up(pix->plane_fmt[0].bytesperline, 16));
	pix->width = width;
	pix->height = height;
	pix->plane_fmt[0].bytesperline = bpl;
	pix->plane_fmt[0].sizeimage = (size_t)bpl * height * 3 / 2;
}

int histb_vdec_cap_dims(struct histb_vdec_ctx *ctx, unsigned int *w,
			unsigned int *h)
{
	const struct v4l2_pix_format_mplane *pix =
		&ctx->cap_fmt.fmt.pix_mp;

	*w = pix->width;
	*h = pix->height;
	return 0;
}

void histb_vdec_buf_geom(struct histb_vdec_ctx *ctx, unsigned int *stride,
			 unsigned int *lines)
{
	unsigned int w, h;

	histb_vdec_cap_dims(ctx, &w, &h);
	*stride = histb_vdec_stride(ctx, w);
	*lines = ALIGN(h, ctx->desc->grid_align);
}

static struct histb_vdec_frame *
histb_vdec_frame_get(struct histb_vdec_ctx *ctx, unsigned int index)
{
	struct histb_vdec_dev *dev = histb_vdec_dev_of(ctx);
	struct histb_vdec_frame *frame;
	unsigned int stride, lines, w, h;
	size_t size;

	if (index >= HISTB_VDEC_MAX_DPB_SLOTS)
		return NULL;
	frame = &ctx->frame[index];
	histb_vdec_buf_geom(ctx, &stride, &lines);
	histb_vdec_cap_dims(ctx, &w, &h);
	size = histb_vdec_frame_size(ctx, w, h);
	if (frame->dma && frame->stride == stride && frame->lines == lines &&
	    frame->size >= size)
		return frame;

	if (frame->dma)
		dma_free_noncoherent(dev->memdev, frame->size, frame->va,
				     frame->dma, DMA_BIDIRECTIONAL);
	memset(frame, 0, sizeof(*frame));
	/*
	 * 内部帧走可缓存映射（见 histb_vdec_frame）。dma_alloc_noncoherent() 用 CPU 缓存写零清空这块内存，这里清一次缓存把这些脏行写回；此后 CPU 只读不写，缓存里只会有干净行，每个 job 在读之前按 DMA_FROM_DEVICE 失效即可（histb_vdec_frame_to_linear()）。每个 job 把帧交回引擎前不再做 for_device，是对 Documentation/core-api/dma-api.rst“交给设备前 for_device”约定的有意省略：arm64 与 ARM32 的 for_device 只做 clean，而这些帧在缓存里没有脏行可写回；将来若加 CPU 写（例如出错时清帧），必须在交回引擎前补 dma_sync_single_for_device()。
	 */
	frame->va = dma_alloc_noncoherent(dev->memdev, size, &frame->dma,
					  DMA_BIDIRECTIONAL, GFP_KERNEL);
	if (!frame->va) {
		memset(frame, 0, sizeof(*frame));
		return NULL;
	}
	dma_sync_single_for_device(dev->memdev, frame->dma, size,
				   DMA_BIDIRECTIONAL);
	frame->size = size;
	frame->stride = stride;
	frame->lines = lines;
	return frame;
}

dma_addr_t histb_vdec_frame_dma(struct histb_vdec_ctx *ctx,
				struct vb2_buffer *vb)
{
	struct histb_vdec_frame *frame;

	if (histb_vdec_direct_capture(ctx)) {
		dma_addr_t dma;

		/*
		 * 消息字与 YSTADDR 只带地址低 32 位，MPEG-2/4 还按 16 字节截取帧地址；不满足的 buffer（只可能来自导入的 DMABUF）按没有帧处理，由调用方拒收这个 job。
		 */
		if (!vb2_plane_cookie(vb, 0))
			return 0;
		dma = vb2_dma_contig_plane_dma_addr(vb, 0);
		if ((dma & 15) || (u64)dma + vb2_plane_size(vb, 0) - 1 >
				  DMA_BIT_MASK(32))
			return 0;
		return dma;
	}
	frame = histb_vdec_frame_get(ctx, vb->index);

	return frame ? frame->dma : 0;
}



void histb_vdec_frames_free(struct histb_vdec_ctx *ctx)
{
	struct histb_vdec_dev *dev = histb_vdec_dev_of(ctx);
	unsigned int i;

	for (i = 0; i < HISTB_VDEC_MAX_DPB_SLOTS; i++) {
		struct histb_vdec_frame *frame = &ctx->frame[i];

		if (!frame->dma)
			continue;
		dma_free_noncoherent(dev->memdev, frame->size, frame->va,
				     frame->dma, DMA_BIDIRECTIONAL);
		memset(frame, 0, sizeof(*frame));
	}
}

/*
 * 返回 buf[pos..end) 中下一个 00 00 @last 三字节序列的位置，H.264 与 HEVC 的起始码扫描和防竞争字节扫描共用。一致性分配的 OUTPUT 平面是 write-combining（normal non-cacheable）映射，逐字节扫描每字节约 150 ns，在 1080p60 下占去了每帧预算的大头；改用整字加载加零字节检测，每次加载跳过八个字节。没有匹配时返回 @end。
 */
u32 histb_vdec_scan_triple(const u8 *buf, u32 end, u32 pos, u8 last)
{
	u32 i = pos;

	while (i + 8 <= end) {
		u64 v = get_unaligned_le64(buf + i), z, pair;
		u32 j;

		/*
		 * 窗口的第 k 个字节是 v 的第 8k..8k+7 位。这里求逐字节精确的零标志（常用的 has-zero 借位技巧会把零字节后面那个字节也报成零）：清掉每个字节的 bit 7 后加 0x7f，零字节在两种视图里的 bit 7 都为 0。不会有进位跨过字节边界。
		 */
		z = ~(((v & 0x7f7f7f7f7f7f7f7fULL) +
		       0x7f7f7f7f7f7f7f7fULL) | v) & 0x8080808080808080ULL;
		pair = z & (z >> 8);
		while (pair) {
			j = __ffs64(pair) >> 3;
			if (i + j + 2 < end && buf[i + j + 2] == last)
				return i + j;
			pair &= pair - 1;
		}
		if ((z & (1ULL << 63)) && i + 9 < end && !buf[i + 8] &&
		    buf[i + 9] == last)
			return i + 7;
		i += 8;
	}
	for (; i + 2 < end; i++)
		if (!buf[i] && !buf[i + 1] && buf[i + 2] == last)
			return i;

	return end;
}

static void histb_vdec_copy_chroma(u8 *d, const u8 *s, unsigned int len,
				   bool swap_uv)
{
	unsigned int i;

	if (!swap_uv) {
		memcpy(d, s, len);
		return;
	}
	for (i = 0; i + 1 < len; i += 2) {
		d[i] = s[i + 1];
		d[i + 1] = s[i];
	}
}

/* V2/V3：亮度 tile 为 256 字节 × 16 行，色度 tile 为 256 字节 × 8 行。 */
static void histb_vdec_copy_tiled(u8 *out, unsigned int bpl, const u8 *src,
				  unsigned int uv, unsigned int stride,
				  unsigned int w, unsigned int h, bool swap_uv)
{
	unsigned int x, y, cy;

	for (y = 0; y < h; y++) {
		const u8 *srow = src + (y / 16) * stride * 16 + (y % 16) * 256;
		u8 *drow = out + (size_t)y * bpl;

		for (x = 0; x < w; x += 256)
			memcpy(drow + x, srow + (x / 256) * 4096,
			       min(256u, w - x));
	}
	for (cy = 0; cy < h / 2; cy++) {
		const u8 *srow = src + uv + (cy / 8) * stride * 8 +
				 (cy % 8) * 256;
		u8 *drow = out + (size_t)bpl * h + (size_t)cy * bpl;

		for (x = 0; x < w; x += 256)
			histb_vdec_copy_chroma(drow + x, srow + (x / 256) * 2048,
					       min(256u, w - x), swap_uv);
	}
}

/* V5R2C1：两个平面都是每 @stride 字节一行画面。 */
static void histb_vdec_copy_raster(u8 *out, unsigned int bpl, const u8 *src,
				   unsigned int uv, unsigned int stride,
				   unsigned int w, unsigned int h, bool swap_uv)
{
	unsigned int y, cy;

	for (y = 0; y < h; y++)
		memcpy(out + (size_t)y * bpl, src + (size_t)y * stride, w);
	for (cy = 0; cy < h / 2; cy++)
		histb_vdec_copy_chroma(out + (size_t)bpl * h + (size_t)cy * bpl,
				       src + uv + (size_t)cy * stride, w,
				       swap_uv);
}

static void histb_vdec_frame_to_linear(struct histb_vdec_ctx *ctx,
				       struct vb2_v4l2_buffer *dst)
{
	struct v4l2_pix_format_mplane *pix =
		&ctx->cap_fmt.fmt.pix_mp;
	struct histb_vdec_frame *frame;
	struct vb2_buffer *vb = &dst->vb2_buf;
	struct dma_buf *dbuf = vb->planes[0].dbuf;
	unsigned int w = pix->width, h = pix->height;
	unsigned int bpl = pix->plane_fmt[0].bytesperline;
	/*
	 * The VDM's tile chroma is Cr first, i.e. NV21; measured on a stream
	 * whose U and V differ (a red frame and a flat 64/192 fixture, both
	 * checked against an ffmpeg decode of the same bitstream: V=191, U=65,
	 * and the tile frame carries bf 41). An NV12 frame therefore swaps the
	 * pair and an NV21 frame takes it as is.
	 */
	bool swap_uv = pix->pixelformat == V4L2_PIX_FMT_NV12;
	unsigned int uv;
	const u8 *src;
	u8 *out;

	if (histb_vdec_direct_capture(ctx)) {
		/* 引擎已经写在这个 buffer 里了 */
		vb2_set_plane_payload(vb, 0, (size_t)bpl * h * 3 / 2);
		return;
	}
	if (vb->index >= HISTB_VDEC_MAX_DPB_SLOTS)
		goto no_frame;
	frame = &ctx->frame[vb->index];
	out = vb2_plane_vaddr(vb, 0);
	if (!frame->va || !out)
		goto no_frame;

	uv = histb_vdec_uv_offset(histb_vdec_dev_of(ctx), frame->stride,
				  frame->lines);
	src = frame->va;
	/* The CPU writes this plane; whoever imported it reads it afterwards. */
	if (dbuf && dma_buf_begin_cpu_access(dbuf, DMA_TO_DEVICE))
		goto no_frame;

	/* 引擎刚写完这一帧：先丢掉缓存里（含预取进来的）旧行再读。 */
	dma_sync_single_for_cpu(histb_vdec_dev_of(ctx)->memdev, frame->dma,
				frame->size, DMA_FROM_DEVICE);
	if (histb_vdec_dev_of(ctx)->raster)
		histb_vdec_copy_raster(out, bpl, src, uv, frame->stride, w, h,
				       swap_uv);
	else
		histb_vdec_copy_tiled(out, bpl, src, uv, frame->stride, w, h,
				      swap_uv);

	if (dbuf)
		dma_buf_end_cpu_access(dbuf, DMA_TO_DEVICE);

	vb2_set_plane_payload(vb, 0, (size_t)bpl * h * 3 / 2);
	return;

no_frame:
	/* No deliverable copy: say so with an empty payload instead of stale
	 * bytes from the previous job (a refusal has to be visible).
	 */
	vb2_set_plane_payload(vb, 0, 0);
}

static bool histb_vdec_job_take(struct histb_vdec_dev *dev,
				struct histb_vdec_job *out)
{
	struct histb_vdec_common *common = &dev->common;
	unsigned long flags;

	spin_lock_irqsave(&common->job_lock, flags);
	if (!dev->common.job_running) {
		spin_unlock_irqrestore(&common->job_lock, flags);
		return false;
	}
	*out = common->job;
	dev->common.job_running = false;
	common->job = (struct histb_vdec_job){};
	spin_unlock_irqrestore(&common->job_lock, flags);
	return true;
}

/*
 * @produced tells the two error paths apart. The engine raising DEC_ERR over a
 * picture it wrote is not the same event as a job that never reached the
 * engine: the stateless API asks the driver to hand back the (likely corrupted)
 * frame in the first case and nothing at all in the second, and only the second
 * is what a refusal looks like from the client side.
 */
static void histb_vdec_job_complete(struct histb_vdec_dev *dev,
				    const struct histb_vdec_job *job,
				    enum vb2_buffer_state state, bool produced)
{
	struct histb_vdec_ctx *ctx = (struct histb_vdec_ctx *)job->ctx;
	struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	struct vb2_v4l2_buffer *dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	bool hold = src && (src->flags & V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF);

	if (dst) {
		u32 bit = BIT(dst->vb2_buf.index % 32);

		if (produced)
			histb_vdec_frame_to_linear(ctx, dst);
		else
			vb2_set_plane_payload(&dst->vb2_buf, 0, 0);
		/* 一对场里任何一场出错，整帧交出时都带 ERROR（见 histb_vdec_ctx.held_err） */
		if (hold && (state == VB2_BUF_STATE_ERROR || !produced))
			ctx->held_err |= bit;
		if (!hold && (ctx->held_err & bit)) {
			ctx->held_err &= ~bit;
			state = VB2_BUF_STATE_ERROR;
		}
	}
	v4l2_m2m_buf_done_and_job_finish(dev->common.m2m_dev, job->ctx->fh.m2m_ctx,
					 state);
	pm_runtime_put_autosuspend(dev->dev);
}

static unsigned int histb_vdec_count_slices(u32 state)
{
	unsigned int num = FIELD_GET(VDM_STATE_SLICE_NUM, state);

	/*
	 * 上行消息池里每个解出的 slice 有一条 4 字报告。厂商 VDMHAL_V5R2C1_MakeDecReport 把字 1、字 2 当作该 slice 的起、止宏块（CTB）号取走（tmp/media/vfmw_v5/vdm_hal.S，读汇编）。字 0 是状态字，厂商只取第 0 条报告字 0 的 bit 31 存成 Error_Flag（vdm_hal.S 的 MakeDecReport，vdm_drv.h 注释为 logic decode err flag），其余各位含义未知。Hi3798MV300 上实测，带 DEC_ERR 的帧里出错 slice 的字 0：链走进空槽 98230000、HEVC 段长去掉停止位 80430000、CABAC 段尾多出不足 1 字节 14230000/14210000/00210000（后一类由 histb_vdec_tail_slack_only() 放行）；干净的 H.264/HEVC slice 为 14020000/00020000（非末 slice 14000000/00000000）。这里的计数供调试输出与覆盖检查使用。
	 */
	return min_t(unsigned int, num, HISTB_VDEC_MAX_UP_SLICES);
}

static void histb_vdec_dump_up(struct histb_vdec_dev *dev, unsigned int slices,
			       bool failed)
{
	const u32 *up = histb_vdec_up_msgs(&dev->pool);
	unsigned int n = min_t(unsigned int, slices, 16);
	unsigned int i;

	for (i = 0; i < n; i++)
		dev_dbg(dev->dev, "up%u=%08x %08x %08x %08x\n", i,
			up[4 * i], up[4 * i + 1], up[4 * i + 2],
			up[4 * i + 3]);
	if (failed && slices)
		dev_err_ratelimited(dev->dev, "failed frame: %u slice reports\n",
				    slices);
}

/*
 * 打开 cabac_seg_stop_bit 的型号上，CABAC slice 的段尾停止位恰好是一个字节的最低位时，引擎常常把段长算短若干位（停在停止位之前的那个 1 上），于是报 DEC_ERR，而像素完全正确。Hi3798MV300 实测（x264 High CABAC 1080p 三条码流 90 帧，段长按厂商规则保留停止位）：停止位不在字节最低位的 35 帧全部干净；在最低位的 55 帧里 46 帧报 DEC_ERR，逐帧把段长缩短 1~7 位扫一遍，每帧恰有一个缩短量能消掉它，46 帧里正好等于停止位到它前面最近一个 1 的距离（例：末字节 10000001 要缩 7 位，11000001 要缩 6 位），另有 9 帧不缩才干净，取决于算术解码的状态，从字节上算不出来。HEVC 也见到同一签名：每帧 4 个 slice 的码流里一个 I 帧的前 3 个 slice 都报，其中只有 1 个的停止位在字节最低位，像素同样逐字节一致。这种帧的 slice 报告字 0 是 0x14230000（H.264 末 slice）、0x14210000/0x00210000（非末 slice），宏块全覆盖。
 *
 * 只放行这一种：DEC_ERR、每条 slice 报告字 0 的 bit 31（厂商 MakeDecReport 存成 Error_Flag 的那一位）为 0，且各 slice 的起止宏块首尾相接、恰好覆盖整幅图像。同机实测其余各类 DEC_ERR 的字 0 都带 bit 31：段长少报 1 位 94430000，H.264 CABAC 码流中段改 48 字节 0xff 94230000、末字节附近改 1 位 94030000，HEVC 4 slice 码流中段改坏 80210028/80230003、末字节附近改 1 位 80410008，链走进空槽 98230000，HEVC 段长去掉停止位 80430000；改坏的帧也都覆盖不全（少 1 到 2183 个宏块/CTB）。位级负对照：全 I 帧的 H.264 CABAC 与 HEVC 各 30 帧，每帧翻转停止位之前第 1~16 位中的一位、共 240 帧，解错的帧（H.264 45 帧、HEVC 58 帧）除 HEVC 1 帧外都带 ERROR，那 1 帧引擎根本没报 DEC_ERR（发行模块同样不带），放行规则没有放过任何解错的帧。放行的帧照旧复位引擎，只是不再交出 V4L2_BUF_FLAG_ERROR、不打印失败日志。
 */
static bool histb_vdec_tail_slack_only(struct histb_vdec_dev *dev,
				       unsigned int slices)
{
	const u32 *up = histb_vdec_up_msgs(&dev->pool);
	u32 next = 0;
	unsigned int i;

	if (!dev->tail_slack_total || !slices ||
	    slices >= HISTB_VDEC_MAX_UP_SLICES)
		return false;

	for (i = 0; i < slices; i++) {
		if (up[4 * i] & BIT(31))
			return false;
		if (up[4 * i + 1] != next || up[4 * i + 2] < next ||
		    up[4 * i + 2] >= dev->tail_slack_total)
			return false;
		next = up[4 * i + 2] + 1;
	}

	return next == dev->tail_slack_total;
}

/*
 * 打开 mpeg_cov_reports 的型号上，MPEG-1/2 与 MPEG-4 Part 2（含 H.263 短头）的前端按 slice 报告的宏块覆盖判帧好坏；MPEG-1/2 与 H.263 的 DEC_ERR 本身不算失败，与厂商 V5R2C1 固件同一口径（读汇编，tmp/media/vfmw_v5/ 与 build/scratch-vdec74/review-cov/vendor-dl/）：VDMHAL_V5R2C1_MakeDecReport 在 VDM_STATE 同时带 DEC_OVER 与 DEC_ERR 时，只对 MPEG-2（VID_STD_E 3）把 RetType 记成 0（vdm_hal.S 的 ubfx x0, x1, 17, 2 / ccmp w21, 3 一段）；MPEG-2 与 MPEG-4 的帧错误率都由 PostPro_GetMp24MbGroup 按各 slice 报告字 1/字 2 的起止宏块之间的缺口算（postprocess.S），没有缺口时只看修复参数里的 ErrFrame_Flag（手头的反汇编里找不到它的写入方，全局变量零初始化），不读 RetType，也不读第 0 条报告字 0 的 bit 31。H.263 在厂商那里走同一条 MPEG-4 路径（VDMHAL_V5R2C1_PrepareDec 对 VFMW_H263 不派发，短头由 MPEG-4 语法层 Mp4_Syntax_ShortHeader/Mp4_Syntax_Slice_SH 处理）。
 *
 * Hi3798MV300 上这两种协议带 DEC_ERR 而像素与软解只差反变换的帧（07 #109）：MPEG-2 每帧都报；H.263 也每帧都报，它的 GOB 段在宏块数据之后总还有位（现行段长到图像末尾；止于下一 GOB 起始码时仍带补到字节边界的 GSTUF 零位，单靠字节分不出它与宏块数据末尾的 0 位）。07 #109 ④ 试过的三种段尾口径（止于下一 GOB、去末尾零字节、再去末字节低位零）都没消掉它，后一种同时会裁掉数据末尾的 0 位，所以不是反证；推断与 MPEG-4 段尾带着填充位就报 DEC_ERR、去掉即不报是同一机理，未上机证实。MPEG-4（非短头）去掉填充位之后干净的帧不再报 DEC_ERR，那里 DEC_ERR 仍是有用的出错信号，所以它照旧算失败，覆盖不全另外也算（比厂商严）。
 *
 * 判据：报告从宏块 0 起、相邻 slice 之间没有缺口（允许重叠，厂商同样只找缺口）、每条的止点不越出画面，并且最终覆盖到最后一个宏块。比厂商严一处：厂商把止于最后 10 个宏块之内的帧也当作完整（PostPro_GetMp24MbGroup 的 sub w1, w5, #10），这里要求覆盖到底。slice 数超出上行报告区（HISTB_VDEC_MAX_UP_SLICES）时报告不全、无从判断，histb_vdec_coverage_judged() 让这样的帧退回 DEC_ERR 判据。
 */
static bool histb_vdec_reports_cover(struct histb_vdec_dev *dev,
				     unsigned int slices)
{
	const u32 *up = histb_vdec_up_msgs(&dev->pool);
	u32 next = 0;
	unsigned int i;

	if (!dev->mb_total || !slices || slices > HISTB_VDEC_MAX_UP_SLICES)
		return false;

	for (i = 0; i < slices; i++) {
		u32 start = up[4 * i + 1], end = up[4 * i + 2];

		if (start > next || end < start || end >= dev->mb_total)
			return false;
		next = max(next, end + 1);
	}

	return next == dev->mb_total;
}

/* 这一帧按 histb_vdec_reports_cover() 判覆盖：打开 mpeg_cov_reports 的型号上的 MPEG-1/2 与 MPEG-4/H.263，且报告没有溢出上行报告区 */
static bool histb_vdec_coverage_judged(const struct histb_vdec_dev *dev,
				       const struct histb_vdec_fmt *fmt, u32 state)
{
	return dev->mpeg_cov_reports && dev->mb_total &&
	       FIELD_GET(VDM_STATE_SLICE_NUM, state) <= HISTB_VDEC_MAX_UP_SLICES &&
	       (fmt->vidstd == HISTB_VIDSTD_MPEG2 ||
		fmt->vidstd == HISTB_VIDSTD_MPEG4);
}

/* DEC_ERR 在宏块全覆盖时不算失败的协议：MPEG-1/2 与 H.263（V4L2_PIX_FMT_MPEG4 的短头 VOP 被前端拒收，H.263 只经 V4L2_PIX_FMT_H263 进来） */
static bool histb_vdec_dec_err_benign(const struct histb_vdec_fmt *fmt)
{
	return fmt->vidstd == HISTB_VIDSTD_MPEG2 ||
	       fmt->fourcc == V4L2_PIX_FMT_H263;
}

static void histb_vdec_report_failure(struct histb_vdec_dev *dev, u32 state,
				      unsigned int slices)
{
	const u32 *up = histb_vdec_up_msgs(&dev->pool);
	unsigned int n = min_t(unsigned int, slices, HISTB_VDEC_MAX_UP_SLICES);
	unsigned int i, lines = 0;
	u32 max_end = 0;
	u32 off;

	for (i = 0; i < n; i++)
		max_end = max(max_end, up[4 * i + 2]);

	dev_err(dev->dev,
		"engine-flagged frame: state=%#010x slices=%u end_mb=%u/%u\n",
		state, slices, max_end, dev->mb_total);

	for (off = VDM_SED_STA; off <= VDM_SED_LAST && lines < 4; off += 0x10) {
		u32 a = vdm_read(dev, off), b = vdm_read(dev, off + 4);
		u32 c = vdm_read(dev, off + 8), d = vdm_read(dev, off + 12);

		if (!(a | b | c | d))
			continue;
		dev_err(dev->dev,
			"sed %03x=%08x %03x=%08x %03x=%08x %03x=%08x\n",
			off, a, off + 4, b, off + 8, c, off + 12, d);
		lines++;
	}
}

irqreturn_t histb_vdec_irq_handler(int irq, void *priv)
{
	struct histb_vdec_dev *dev = priv;
	struct histb_vdec_job job;
	unsigned int slices;
	bool short_cov, deliver_err, slack_only, cov_judged, benign;
	u32 state;

	/*
	 * Use the state the primary handler captured: clearing INT_STATE drops
	 * the latched frame-done condition, so a re-read here would already
	 * show an idle VDM and the job would never be completed.
	 */
	state = dev->irq_state;
	if (!(state & VDM_STATE_DEC_OVER))
		return IRQ_HANDLED;

	if (!histb_vdec_job_take(dev, &job))
		return IRQ_HANDLED;

	slices = histb_vdec_count_slices(state);
	dev_dbg(dev->dev, "frame done: %u slices%s\n", slices,
		(state & VDM_STATE_DEC_ERR) ? ", decode error" : "");
	slack_only = (state & VDM_STATE_DEC_ERR) &&
		     histb_vdec_tail_slack_only(dev, slices);

	/*
	 * One 4-word report per slice; word 1 is the first and word 2 the last
	 * macroblock the slice covered (measured: a 300-MB picture reports 0x12b,
	 * and a stream cut mid-slice reports the shorter end; the vendor's
	 * MakeDecReport copies the same two words into SliceMbRange). Coverage is
	 * judged whenever the front end reports a macroblock total, and a frame
	 * whose reports stop short is always recovered from below. On parts with
	 * mpeg_cov_reports, for MPEG-1/2 and MPEG-4/H.263 a gap also fails the
	 * frame (histb_vdec_reports_cover()), and for MPEG-1/2 and H.263 coverage
	 * alone decides it (histb_vdec_dec_err_benign()). Elsewhere MPEG-1/2 is
	 * decided by the end-only check, because that front end raises DEC_ERR
	 * on pictures the check can prove complete, and the other front ends keep
	 * the DEC_ERR rule.
	 */
	cov_judged = histb_vdec_coverage_judged(dev, job.ctx->desc, state);
	benign = cov_judged && histb_vdec_dec_err_benign(job.ctx->desc);
	short_cov = false;
	if (cov_judged) {
		short_cov = !histb_vdec_reports_cover(dev, slices);
		if (short_cov)
			dev_err_ratelimited(dev->dev,
					    "slice reports do not cover all %u macroblocks (%u reports)\n",
					    dev->mb_total,
					    (unsigned int)FIELD_GET(VDM_STATE_SLICE_NUM, state));
	} else if (dev->mb_total && slices) {
		const u32 *up = histb_vdec_up_msgs(&dev->pool);
		u32 max_end = 0;
		bool bad = false;
		unsigned int i;

		for (i = 0; i < slices && i < HISTB_VDEC_MAX_UP_SLICES; i++) {
			u32 end = up[4 * i + 2];

			if (end >= dev->mb_total)
				bad = true;
			max_end = max(max_end, end);
		}
		if (!bad && max_end != dev->mb_total - 1)
			bad = true;
		if (bad) {
			dev_err_ratelimited(dev->dev,
					    "frame decoded only %u of %u macroblocks\n",
					    max_end + 1, dev->mb_total);
			short_cov = true;
		}
	}

	if (cov_judged)
		deliver_err = short_cov ||
			      ((state & VDM_STATE_DEC_ERR) && !benign);
	else if (dev->mb_total && slices &&
		 job.ctx->desc->vidstd == HISTB_VIDSTD_MPEG2)
		deliver_err = short_cov;
	else
		deliver_err = (state & VDM_STATE_DEC_ERR) && !slack_only;
	histb_vdec_dump_up(dev, slices,
			   cov_judged ? deliver_err :
				    (state & VDM_STATE_DEC_ERR) && !slack_only);

	/*
	 * A picture the engine flagged leaves the VDM pipeline in a state the next
	 * frame cannot be programmed out of: measured on the board, an MPEG-2
	 * picture that ended with DEC_ERR set made every subsequent frame hang
	 * (H.264 alone 4/4 clean, H.264 right after one MPEG-2 picture 8/8 timed
	 * out with the engine not reading a single macroblock), and only the CRG
	 * reset recovered it. Reset here so an errored stream cannot poison the
	 * next job, whatever opened it; parts with covered_err_keeps_engine skip
	 * it for the frames the coverage rule accepts, as the vendor firmware
	 * does. Under the coverage rule the one-per-stream diagnostic is kept for
	 * the frames that fail it.
	 */
	if ((state & VDM_STATE_DEC_ERR) || short_cov) {
		if ((cov_judged ? deliver_err : !slack_only) &&
		    !((struct histb_vdec_ctx *)job.ctx)->diag_reported) {
			histb_vdec_report_failure(dev, state, slices);
			((struct histb_vdec_ctx *)job.ctx)->diag_reported = true;
		}
		if (!(benign && !short_cov && dev->covered_err_keeps_engine))
			histb_vdec_hw_reset(dev);
	}

	cancel_delayed_work(&dev->watchdog);
	histb_vdec_job_complete(dev, &job,
				deliver_err ? VB2_BUF_STATE_ERROR :
					      VB2_BUF_STATE_DONE, true);
	return IRQ_HANDLED;
}

static bool histb_vdec_hw_irq_status(struct histb_vdec_dev *dev, u32 *state)
{
	u32 val = vdm_read(dev, VDM_STATE);

	if (!(val & VDM_STATE_DEC_OVER))
		return false;

	*state = val;
	vdm_write(dev, VDM_INT_STATE, 0xffffffff);
	return true;
}

static irqreturn_t histb_vdec_hardirq(int irq, void *priv)
{
	struct histb_vdec_dev *dev = priv;
	u32 state;

	if (!histb_vdec_hw_irq_status(dev, &state))
		return IRQ_NONE;

	dev->irq_state = state;
	return IRQ_WAKE_THREAD;
}

static void histb_vdec_watchdog(struct work_struct *work)
{
	struct histb_vdec_dev *dev =
		container_of(to_delayed_work(work), struct histb_vdec_dev,
			     watchdog);
	struct histb_vdec_job job;

	if (!histb_vdec_job_take(dev, &job))
		return;

	/* Claimed: the frame is ours to recover, and no IRQ thread can complete
	 * it from under us any more.
	 *
	 * The state and the per-picture counters have to be read here: the
	 * reset below wipes them, and a timeout whose engine never asked for a
	 * single read is a different bug from one that decoded the whole
	 * picture without ever raising end-of-video.
	 */
	dev_err(dev->dev,
		"frame decode timed out (state=%#010x int=%#010x cycle=%u rd=%u wr=%u), resetting "
		"the VDM\n",
		vdm_read(dev, VDM_STATE), vdm_read(dev, VDM_INT_STATE),
		vdm_read(dev, VDM_DEC_CYCLE_PER_PIC),
		vdm_read(dev, VDM_RD_REQ_PER_PIC),
		vdm_read(dev, VDM_WR_REQ_PER_PIC));
	histb_vdec_hw_reset(dev);
	histb_vdec_job_complete(dev, &job, VB2_BUF_STATE_ERROR, false);
}

static void histb_vdec_device_run(void *priv)
{
	struct histb_vdec_ctx *ctx = priv;
	struct histb_vdec_dev *dev = histb_vdec_dev_of(ctx);
	struct histb_vdec_common *common = &dev->common;
	struct histb_vdec_job job_record;
	struct vb2_v4l2_buffer *src, *dst;
	unsigned int gen;
	unsigned long flags;
	int ret;

	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	if (WARN_ON(!src) || WARN_ON(!dst))
		return;

	if (READ_ONCE(dev->common.removing)) {
		/*
		 * v4l2_m2m_suspend() waits for the context that is current, not
		 * for every one that is queued, so a job can still be scheduled
		 * while the teardown runs. Hand the buffers back rather than
		 * doorbell a block that is about to lose its clock.
		 */
		goto err_finish_no_pm;
	}

	ret = pm_runtime_resume_and_get(dev->dev);
	if (ret) {
		dev_dbg(dev->dev, "pm resume failed: %d\n", ret);
		goto err_finish_no_pm;
	}

	mutex_lock(&dev->hw_lock);

	spin_lock_irqsave(&common->job_lock, flags);
	if (WARN_ON(dev->common.job_running)) {
		spin_unlock_irqrestore(&common->job_lock, flags);
		mutex_unlock(&dev->hw_lock);
		pm_runtime_put_autosuspend(dev->dev);
		return;
	}
	dev->common.job_running = true;
	common->job_gen++;
	common->job = (struct histb_vdec_job){ (struct histb_vdec_core_ctx *)ctx, common->job_gen };
	gen = common->job.gen;
	spin_unlock_irqrestore(&common->job_lock, flags);

	/*
	 * Apply the controls attached to the source buffer's request; the
	 * request is completed right away because the message assembly below
	 * has already consumed the control values.
	 */
	/* The front end reports the macroblocks it submitted, if it knows. */
	dev->mb_total = 0;
	dev->tail_slack_total = 0;
	ret = v4l2_ctrl_request_setup(src->vb2_buf.req_obj.req,
				      &ctx->ctrl_hdl);
	if (ret) {
		dev_dbg(dev->dev, "request setup failed: %d\n", ret);
		goto err_job;
	}

	/* 不是接着一个被扣住的 buffer 解第二场：清掉这个下标上一帧留下的出错记录 */
	if (!dst->is_held)
		ctx->held_err &= ~BIT(dst->vb2_buf.index % 32);
	v4l2_m2m_buf_copy_metadata(src, dst);

	/*
	 * The protocol front ends walks the bitstream with the CPU to pack the
	 * message slots, so the whole of that walk goes inside one access window
	 * on the source plane.
	 */
	ret = histb_vdec_src_cpu_begin(&src->vb2_buf);
	if (ret) {
		dev_dbg(dev->dev, "source cpu access failed: %d\n", ret);
		goto err_job;
	}
	ret = histb_vdec_v2_ops(ctx)->run(ctx, src, dst);
	histb_vdec_src_cpu_end(&src->vb2_buf);
	if (ret) {
		dev_dbg(dev->dev, "run refused: %d (out fourcc %4.4s)\n", ret,
			(char *)&ctx->desc->fourcc);
		/*
		 * The empty payload this path ends up delivering is the only
		 * signal that separates a refusal from a picture the engine
		 * flagged: both carry V4L2_BUF_FLAG_ERROR, and a client that
		 * reads the flag alone would display the previous contents of
		 * the buffer as new output.
		 */
		goto err_job;
	}

	/*
	 * The message assembly above consumed this request's control values, so
	 * its control object can be completed right away; the buffer object is
	 * completed when the job finishes.
	 */
	v4l2_ctrl_request_complete(src->vb2_buf.req_obj.req, &ctx->ctrl_hdl);

	/* hw_lock only guards the assembly and the doorbell; completion paths
	 * never take it, so it must not stay held across the wait.
	 */
	mutex_unlock(&dev->hw_lock);
	schedule_delayed_work(&dev->watchdog, HISTB_VDEC_TIMEOUT);
	dev_dbg(dev->dev, "job %u programmed\n", gen);
	return;

err_job:
	v4l2_ctrl_request_complete(src->vb2_buf.req_obj.req, &ctx->ctrl_hdl);
	mutex_unlock(&dev->hw_lock);
	if (histb_vdec_job_take(dev, &job_record))
		histb_vdec_job_complete(dev, &job_record,
					VB2_BUF_STATE_ERROR, false);
	return;

err_finish_no_pm:
	v4l2_ctrl_request_complete(src->vb2_buf.req_obj.req, &ctx->ctrl_hdl);
	v4l2_m2m_buf_done_and_job_finish(dev->common.m2m_dev, ctx->fh.m2m_ctx,
					 VB2_BUF_STATE_ERROR);
}

static void histb_vdec_job_abort(void *priv)
{
	struct histb_vdec_ctx *ctx = priv;
	struct histb_vdec_dev *dev = histb_vdec_dev_of(ctx);
	struct histb_vdec_job job;
	bool got;

	/*
	 * hw_lock keeps the claim from landing in the middle of device_run's
	 * assembly: the job is marked running before the registers are written, so
	 * without it the reset below could interleave with those writes and the
	 * doorbell could still be rung afterwards.
	 */
	mutex_lock(&dev->hw_lock);
	got = histb_vdec_job_take(dev, &job);
	mutex_unlock(&dev->hw_lock);
	if (!got)
		return;

	cancel_delayed_work_sync(&dev->watchdog);
	dev_dbg(dev->dev, "job %u aborted\n", job.gen);
	histb_vdec_hw_reset(dev);
	histb_vdec_job_complete(dev, &job, VB2_BUF_STATE_ERROR, false);
}

static const struct v4l2_m2m_ops histb_vdec_m2m_ops = {
	.device_run = histb_vdec_device_run,
	.job_abort = histb_vdec_job_abort,
};

static int histb_vdec_runtime_suspend(struct device *dev)
{
	struct histb_vdec_dev *vdec = dev_get_drvdata(dev);

	/* Stop first, then make sure no latched report can fire on the way
	 * down; the clocks are the last thing to go.
	 */
	vdm_write(vdec, VDM_START, 0);
	vdm_write(vdec, VDM_INT_MASK, 0xffffffff);
	vdm_write(vdec, VDM_INT_STATE, 0xffffffff);
	clk_bulk_disable_unprepare(vdec->num_clks, vdec->clks);
	return 0;
}

static int histb_vdec_runtime_resume(struct device *dev)
{
	struct histb_vdec_dev *vdec = dev_get_drvdata(dev);

	int ret;

	ret = clk_bulk_prepare_enable(vdec->num_clks, vdec->clks);
	if (ret)
		return ret;

	/* Registers return whatever the power domain left; mask and clear so a
	 * stale report cannot be taken for the next job's completion.
	 */
	return histb_vdec_hw_init(vdec);
}

static int histb_vdec_pool_alloc(struct histb_vdec_dev *dev)
{
	struct histb_vdec_msgpool *pool = &dev->pool;

	/*
	 * pic slot, slice slots, upstream report slots and the protocol table
	 * area (CABAC M/N tables, tile info, ppfd and the band-line scratch
	 * blocks) must all fit in the single pool.
	 */
	BUILD_BUG_ON(ALIGN(HISTB_VDEC_SLOT_BYTES +
			   HISTB_VDEC_MAX_SLICE_SLOTS * HISTB_VDEC_SLOT_BYTES +
			   HISTB_VDEC_MAX_UP_SLICES *
			   HISTB_VDEC_UP_MSG_WORDS * 4, 64) +
		     HISTB_TABLE_OFF_SCRATCH(HISTB_VDEC_NUM_SCRATCH - 1) +
		     HISTB_VDEC_SCRATCH_SIZE > HISTB_VDEC_POOL_SIZE);

	pool->size = HISTB_VDEC_POOL_SIZE;
	pool->va = dma_alloc_wc(dev->memdev, pool->size, &pool->dma, GFP_KERNEL);
	if (!pool->va)
		return -ENOMEM;

	pool->pic_off = 0;
	pool->slice_off = HISTB_VDEC_SLOT_BYTES;
	pool->up_off = pool->slice_off +
		       HISTB_VDEC_MAX_SLICE_SLOTS * HISTB_VDEC_SLOT_BYTES;
	pool->table_off = ALIGN(pool->up_off + HISTB_VDEC_MAX_UP_SLICES *
				HISTB_VDEC_UP_MSG_WORDS * 4, 64);

	memset(pool->va, 0, pool->size);
	return 0;
}

static void histb_vdec_pool_free(void *data)
{
	struct histb_vdec_dev *dev = data;

	dma_free_wc(dev->memdev, dev->pool.size, dev->pool.va, dev->pool.dma);
	dev->pool.va = NULL;
}

static const u32 *histb_vdec_required_ctrls(const struct histb_vdec_ctx *ctx,
					    unsigned int *num)
{
	static const u32 h264[] = {
		V4L2_CID_STATELESS_H264_SPS,
		V4L2_CID_STATELESS_H264_PPS,
		V4L2_CID_STATELESS_H264_DECODE_PARAMS,
		V4L2_CID_STATELESS_H264_SLICE_PARAMS,
	};
	static const u32 hevc[] = {
		V4L2_CID_STATELESS_HEVC_SPS,
		V4L2_CID_STATELESS_HEVC_PPS,
		V4L2_CID_STATELESS_HEVC_DECODE_PARAMS,
		V4L2_CID_STATELESS_HEVC_SLICE_PARAMS,
	};
	static const u32 mpeg2[] = {
		V4L2_CID_STATELESS_MPEG2_SEQUENCE,
		V4L2_CID_STATELESS_MPEG2_PICTURE,
		V4L2_CID_STATELESS_MPEG2_QUANTISATION,
	};

	switch (ctx->desc->fourcc) {
	case V4L2_PIX_FMT_H264_SLICE:
		*num = ARRAY_SIZE(h264);
		return h264;
	case V4L2_PIX_FMT_HEVC_SLICE:
		*num = ARRAY_SIZE(hevc);
		return hevc;
	case V4L2_PIX_FMT_MPEG2_SLICE:
		*num = ARRAY_SIZE(mpeg2);
		return mpeg2;
	default:
		*num = 0;
		return NULL;
	}
}
/* ---------------------------------------------------------------- probe */

static void histb_vdec_v2_memdev_put(void *data)
{
	struct histb_vdec_dev *dev = data;

	put_device(dev->memdev);
}

int histb_vdec_v2_probe(struct platform_device *pdev,
			const struct histb_vdec_gen *gen)
{
	const struct histb_vdec_v2_part *part =
		container_of_const(gen, struct histb_vdec_v2_part, gen);
	struct histb_vdec_dev *dev;
	struct platform_device *mem;
	int ret;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->dev = &pdev->dev;
	dev->chip = gen->chip;
	dev->raster = part->raster;
	dev->direct_capture = part->direct_capture;
	dev->h264_chain_null_end = part->h264_chain_null_end;
	dev->cabac_seg_stop_bit = part->cabac_seg_stop_bit;
	dev->qmatrix_vfmw5 = part->qmatrix_vfmw5;
	dev->mv_out = part->mv_out;
	dev->hevc_tiles = part->hevc_tiles;
	dev->h264_interlace_vfmw5 = part->h264_interlace_vfmw5;
	dev->mpeg4_vfmw5 = part->mpeg4_vfmw5;
	dev->mpeg_cov_reports = part->mpeg_cov_reports;
	dev->covered_err_keeps_engine = part->covered_err_keeps_engine;
	dev->common.priv = dev;
	platform_set_drvdata(pdev, dev);

	mutex_init(&dev->hw_lock);
	mutex_init(&dev->common.vdev_lock);
	spin_lock_init(&dev->common.job_lock);
	INIT_DELAYED_WORK(&dev->watchdog, histb_vdec_watchdog);

	dev->vdm = devm_platform_ioremap_resource_byname(pdev, "vdm");
	if (IS_ERR(dev->vdm))
		return PTR_ERR(dev->vdm);

	dev->scd = devm_platform_ioremap_resource_byname(pdev, "scd");
	if (IS_ERR(dev->scd))
		return PTR_ERR(dev->scd);

	ret = devm_clk_bulk_get_all(&pdev->dev, &dev->clks);
	if (ret < 0)
		return ret;
	dev->num_clks = ret;

	/*
	 * Both reset lines have to be released: the subsystem-wide one keeps
	 * the whole VDM register window reading zero while asserted.
	 */
	dev->rst = devm_reset_control_array_get_exclusive(&pdev->dev);
	if (IS_ERR(dev->rst))
		return PTR_ERR(dev->rst);

	/*
	 * VDH 门控位于 CRG30 的型号（Hi3798MV300）由节点给出 CRG syscon，histb_vdec_hw_init() 在首次访问 VDM 寄存器之前打开这些门控。
	 */
	dev->crg = syscon_regmap_lookup_by_phandle_optional(pdev->dev.of_node,
							     "hisilicon,crg");
	if (IS_ERR(dev->crg))
		return dev_err_probe(&pdev->dev, PTR_ERR(dev->crg),
				     "failed to look up the CRG syscon\n");
	/* 不开门控时整个 VDM 窗口都读回 0。 */
	if (part->crg_gates && !dev->crg)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "hisilicon,crg is required on this part\n");

	ret = platform_get_irq(pdev, 0);
	if (ret < 0)
		return ret;
	dev->irq = ret;

	ret = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "failed to set DMA mask\n");
		return ret;
	}
	mem = platform_device_alloc("histb-vdec-dma", PLATFORM_DEVID_NONE);
	if (!mem)
		return -ENOMEM;
	dev->memdev = &mem->dev;
	platform_device_set_of_node_from_dev(mem, &pdev->dev);
	ret = dev_set_name(dev->memdev, "%s-dma", dev_name(&pdev->dev));
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(&pdev->dev, histb_vdec_v2_memdev_put,
				       dev);
	if (ret)
		return ret;
	ret = of_dma_configure(dev->memdev, dev->memdev->of_node, true);
	if (ret)
		return ret;
	ret = dma_set_mask_and_coherent(dev->memdev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	vb2_dma_contig_set_max_seg_size(dev->memdev, DMA_BIT_MASK(32));
	/* Exported buffers retain their allocator even after unbind and rebind. */
	ret = devm_of_reserved_mem_device_init(dev->memdev);
	if (ret && ret != -ENODEV) {
		dev_err(&pdev->dev, "failed to bind reserved memory\n");
		return ret;
	}

	ret = histb_vdec_pool_alloc(dev);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(&pdev->dev, histb_vdec_pool_free, dev);
	if (ret)
		return ret;

	histb_vdec_hw_init_tables(dev);

	pm_runtime_set_autosuspend_delay(&pdev->dev, 100);
	pm_runtime_use_autosuspend(&pdev->dev);
	/* The block must come up through the resume callback: the gates the
	 * bootloader may have left on are not ours to rely on.
	 */
	pm_runtime_set_suspended(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	ret = pm_runtime_resume_and_get(&pdev->dev);
	if (ret)
		goto err_disable_pm;

	/*
	 * The decoder comes up held in reset; release it, mask every interrupt
	 * source and only then register the handler.
	 */
	ret = reset_control_deassert(dev->rst);
	if (ret)
		goto err_disable_pm;

	ret = histb_vdec_hw_init(dev);
	if (ret)
		goto err_assert_reset;

	ret = devm_request_threaded_irq(&pdev->dev, dev->irq,
					histb_vdec_hardirq,
					histb_vdec_irq_handler,
					IRQF_ONESHOT, dev_name(&pdev->dev), dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to request irq\n");
		goto err_assert_reset;
	}

	ret = histb_vdec_core_init(&dev->common, pdev, gen);
	if (ret)
		goto err_assert_reset;


	dev_info(&pdev->dev, "video decoder, %s\n", dev->chip->model);
	pm_runtime_put_autosuspend(&pdev->dev);

	return 0;

err_assert_reset:
	reset_control_assert(dev->rst);
	pm_runtime_put_sync(&pdev->dev);
err_disable_pm:
	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
	return ret;
}

void histb_vdec_v2_remove(struct platform_device *pdev)
{
	struct histb_vdec_dev *dev = platform_get_drvdata(pdev);

	histb_vdec_core_stop(&dev->common);
	disable_irq(dev->irq);
	cancel_delayed_work_sync(&dev->watchdog);
	histb_vdec_core_exit(&dev->common);
	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	reset_control_assert(dev->rst);
	if (!pm_runtime_status_suspended(&pdev->dev))
		histb_vdec_runtime_suspend(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
}


/* ------------------------------------------------------------- gen ops */

static const u32 histb_vdec_v2_cap_formats[] = {
	V4L2_PIX_FMT_NV21,
	V4L2_PIX_FMT_NV12,
};

static struct device *histb_vdec_v2_dma_dev(struct histb_vdec_common *common)
{
	struct histb_vdec_dev *dev = common->priv;

	return dev->memdev;
}

static void histb_vdec_v2_buf_geom_hook(struct histb_vdec_core_ctx *cctx,
					struct histb_vdec_geom *g)
{
	struct histb_vdec_ctx *ctx = (struct histb_vdec_ctx *)cctx;

	histb_vdec_buf_geom(ctx, &g->pitch, &g->lines);
	g->uv_offset = histb_vdec_uv_offset(histb_vdec_dev_of(ctx), g->pitch,
					    g->lines);
}

static int histb_vdec_v2_buf_prepare_hook(struct histb_vdec_core_ctx *cctx,
					  struct vb2_buffer *vb)
{
	dma_addr_t dma = vb2_dma_contig_plane_dma_addr(vb, 0);

	/*
	 * The bitstream segment descriptors carry the length in 24 bits;
	 * anything larger would wrap around silently.
	 */
	if (vb2_get_plane_payload(vb, 0) > BIT(24) - 1)
		return -EINVAL;
	if (vb->planes[0].data_offset + vb2_get_plane_payload(vb, 0) >
	    vb2_plane_size(vb, 0))
		return -EINVAL;

	/*
	 * The message words carry the bitstream start in one 32-bit field
	 * (lower_32_bits), so a plane that ends above 4 GiB would be read from a
	 * truncated address. This device's mask keeps its own allocations inside
	 * that window; say it here rather than rely on the mask alone.
	 */
	if ((u64)dma + vb2_plane_size(vb, 0) - 1 > DMA_BIT_MASK(32))
		return -ERANGE;

	return 0;
}

static unsigned int histb_vdec_v2_min_coded_width(u32 fourcc)
{
	/*
	 * An H.264 picture one macroblock column wide cannot be delivered at
	 * all: the shared luma write path needs two macroblock columns, measured
	 * on the board with a 32-wide stream declared as one column.
	 */
	return fourcc == V4L2_PIX_FMT_H264_SLICE ? 32 : 16;
}

static int histb_vdec_v2_start(struct histb_vdec_core_ctx *cctx,
			       const struct vb2_queue *q)
{
	struct histb_vdec_ctx *ctx = (struct histb_vdec_ctx *)cctx;
	if (!V4L2_TYPE_IS_OUTPUT(q->type))
		return 0;

	ctx->diag_reported = false;
	if (histb_vdec_v2_ops(ctx)->start)
		return histb_vdec_v2_ops(ctx)->start(ctx);

	return 0;
}

static void histb_vdec_v2_stop(struct histb_vdec_core_ctx *cctx,
			       const struct vb2_queue *q)
{
	struct histb_vdec_ctx *ctx = (struct histb_vdec_ctx *)cctx;
	if (V4L2_TYPE_IS_OUTPUT(q->type) && histb_vdec_v2_ops(ctx)->stop)
		histb_vdec_v2_ops(ctx)->stop(ctx);

	/* With the stream stopped no dpb index survives, so neither do the
	 * tile frames that stood in for the client buffers.
	 */
	histb_vdec_frames_free(ctx);
}

static void histb_vdec_v2_release_ctx(struct histb_vdec_core_ctx *cctx)
{
	histb_vdec_frames_free((struct histb_vdec_ctx *)cctx);
}

static void histb_vdec_v2_protocol_changed(struct histb_vdec_core_ctx *cctx)
{
	struct histb_vdec_ctx *ctx = (struct histb_vdec_ctx *)cctx;

	/* The dpb -> pmv slot map is per-protocol. */
	memset(ctx->pmv_map, 0, sizeof(ctx->pmv_map));
	ctx->pmv_next = 0;
}

static int histb_vdec_v2_try_ctrl(const struct histb_vdec_fmt *fmt,
				  struct v4l2_ctrl *ctrl)
{
	const struct histb_vdec_codec_ops *ops = fmt->ops;

	if (ops->try_ctrl)
		return ops->try_ctrl(ctrl);

	return 0;
}

static void histb_vdec_v2_set_out_size(struct histb_vdec_core_ctx *cctx,
				       struct v4l2_format *f,
				       unsigned int width, unsigned int height)
{
	histb_vdec_set_frame_size((struct histb_vdec_ctx *)cctx, f, width,
				  height);
}

/*
 * OUTPUT 格式一变就重置 capture：direct_capture 的 NV21 取引擎直写的几何，其余沿用内部帧的几何（bytesperline 为引擎行距，色度紧跟亮度、由拷贝写出）。
 */
static void histb_vdec_v2_set_cap_reset(struct histb_vdec_core_ctx *cctx,
					struct v4l2_format *f,
					unsigned int width, unsigned int height)
{
	struct histb_vdec_ctx *ctx = (struct histb_vdec_ctx *)cctx;

	if (histb_vdec_direct_capture_fmt(histb_vdec_dev_of(ctx),
					  f->fmt.pix_mp.pixelformat))
		histb_vdec_set_direct_size(ctx, f, width, height);
	else
		histb_vdec_set_frame_size(ctx, f, width, height);
}

static void histb_vdec_v2_set_cap_size(struct histb_vdec_core_ctx *cctx,
				       struct v4l2_format *f,
				       unsigned int width, unsigned int height)
{
	histb_vdec_set_cap_size((struct histb_vdec_ctx *)cctx, f, width,
				height);
}

static const u32 *histb_vdec_v2_required_ctrls(
				const struct histb_vdec_core_ctx *cctx,
				unsigned int *num)
{
	return histb_vdec_required_ctrls((const struct histb_vdec_ctx *)cctx,
					 num);
}

#define HISTB_VDEC_V2_GEN(_chip, _caps, _max_w, _max_h, _min_queued, _hold)	\
	{								\
		.name			= "histb-vdec",			\
		.ctx_size		= sizeof(struct histb_vdec_ctx),	\
		.chip			= _chip,				\
		.max_width		= _max_w,				\
		.max_height		= _max_h,				\
		.default_width		= 1920,					\
		.max_cap_buffers	= HISTB_VDEC_MAX_APC_SLOTS,	\
		.min_queued_buffers	= _min_queued,				\
		.cap_clamp		= true,				\
		.hold_capture		= _hold,				\
		.cap_formats		= _caps,				\
		.num_cap_formats	= ARRAY_SIZE(_caps),			\
		.dma_dev		= histb_vdec_v2_dma_dev,		\
		.set_out_size		= histb_vdec_v2_set_out_size,		\
		.set_cap_size		= histb_vdec_v2_set_cap_size,		\
		.set_cap_reset		= histb_vdec_v2_set_cap_reset,		\
		.buf_geom		= histb_vdec_v2_buf_geom_hook,		\
		.buf_prepare		= histb_vdec_v2_buf_prepare_hook,	\
		.min_coded_width	= histb_vdec_v2_min_coded_width,	\
		.start			= histb_vdec_v2_start,			\
		.stop			= histb_vdec_v2_stop,			\
		.release		= histb_vdec_v2_release_ctx,		\
		.required_ctrls		= histb_vdec_v2_required_ctrls,		\
		.try_ctrl		= histb_vdec_v2_try_ctrl,		\
		.protocol_changed	= histb_vdec_v2_protocol_changed,	\
		.m2m_ops		= &histb_vdec_m2m_ops,			\
	}

static const struct histb_vdec_v2_part histb_vdec_v2_part_mv100 = {
	.gen = HISTB_VDEC_V2_GEN(&histb_vdec_v2_chip_mv100,
				 histb_vdec_v2_cap_formats, 1920, 1920, 4, false),
};
static const struct histb_vdec_v2_part histb_vdec_v2_part_cv200 = {
	.gen = HISTB_VDEC_V2_GEN(&histb_vdec_v2_chip_cv200,
				 histb_vdec_v2_cap_formats, 1920, 1920, 4, false),
};

/*
 * Hi3798MV300 的引擎最高可解 4096x2304（厂商能力表，424,673,280 像素/秒），但消息池的 band-line scratch、内部帧和 capture buffer 都随画面增大，4K 需要的 CMA 超过默认大小的区域所能容纳。2560x1440 是目前跑通过的最大几何；再往上提还需要同时增大 HISTB_VDEC_MAX_WIDTH（scratch 跟随它）、HISTB_VDEC_POOL_SIZE 和板级 CMA。
 *
 * capture 下限两个是单参考帧逐行码流的最低需求，不是 DPB 深度策略：深度是码流属性，没有哪个格式能携带，所以参考 N 帧的客户端得自己保持 N+1 个 buffer 轮转。下限压低在 1440p 下很要紧，那里每个 capture buffer 占 5.5 MB（NV12 时背后另有同样大的内部帧）。
 */
static const struct histb_vdec_v2_part histb_vdec_v2_part_mv300 = {
	.gen = HISTB_VDEC_V2_GEN(&histb_vdec_v2_chip_mv300,
				 histb_vdec_v2_cap_formats, 2560, 1440, 2, true),
	.raster = true,
	.direct_capture = true,
	.crg_gates = true,
	.h264_chain_null_end = true,
	.cabac_seg_stop_bit = true,
	.qmatrix_vfmw5 = true,
	.mv_out = true,
	.hevc_tiles = true,
	.h264_interlace_vfmw5 = true,
	.mpeg4_vfmw5 = true,
	.mpeg_cov_reports = true,
	.covered_err_keeps_engine = true,
};

const struct histb_vdec_variant histb_vdec_v2_mv100_variant = {
	.gen			= &histb_vdec_v2_part_mv100.gen,
	.probe			= histb_vdec_v2_probe,
	.remove			= histb_vdec_v2_remove,
	.runtime_suspend	= histb_vdec_runtime_suspend,
	.runtime_resume		= histb_vdec_runtime_resume,
	.system_suspend		= pm_runtime_force_suspend,
	.system_resume		= pm_runtime_force_resume,
};

const struct histb_vdec_variant histb_vdec_v2_cv200_variant = {
	.gen			= &histb_vdec_v2_part_cv200.gen,
	.probe			= histb_vdec_v2_probe,
	.remove			= histb_vdec_v2_remove,
	.runtime_suspend	= histb_vdec_runtime_suspend,
	.runtime_resume		= histb_vdec_runtime_resume,
	.system_suspend		= pm_runtime_force_suspend,
	.system_resume		= pm_runtime_force_resume,
};

const struct histb_vdec_variant histb_vdec_v2_mv300_variant = {
	.gen			= &histb_vdec_v2_part_mv300.gen,
	.probe			= histb_vdec_v2_probe,
	.remove			= histb_vdec_v2_remove,
	.runtime_suspend	= histb_vdec_runtime_suspend,
	.runtime_resume		= histb_vdec_runtime_resume,
	.system_suspend		= pm_runtime_force_suspend,
	.system_resume		= pm_runtime_force_resume,
};
