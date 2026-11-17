/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Shared contract between the HiSTB stateless decoder core and its
 * per-generation backends.
 *
 * Both generations expose the same V4L2 stateless mem2mem interface: fully
 * parsed syntax arrives through request controls, one OUTPUT and one CAPTURE
 * buffer per job, and the capture buffer receives a linear frame (written by
 * the engine itself where the generation can, otherwise copied out of an
 * internal surface). The core owns that interface, the node and media
 * registration and the request validation; a backend owns the message ABI,
 * the launch, the interrupt and the surface geometry, which are where the two
 * generations actually differ. 两代逐字相同的消息打包（HEVC 量化矩阵图像）例外地放在 core，见 histb_vdec_hevc_pack_scaling()。
 */
#ifndef __HISTB_VDEC_CORE_H__
#define __HISTB_VDEC_CORE_H__

/* Nominal OUTPUT (bitstream) plane size both generations start from. */
#define HISTB_VDEC_DEFAULT_OUTPUT_SIZE	(3 * 1024 * 1024)

#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include <media/media-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-v4l2.h>

struct histb_vdec_common;
struct histb_vdec_core_ctx;
struct histb_vdec_gen;

/*
 * One protocol's description of itself. Both generations share the layout so
 * that the core can negotiate formats without knowing which block it drives;
 * @ops points at the protocol's own message assembly, which stays with the
 * backend.
 */
struct histb_vdec_fmt {
	u32				fourcc;
	u32				vidstd;
	unsigned int			num_ctrls;
	const struct v4l2_ctrl_config	*ctrls;
	const void			*ops;
	unsigned int			(*stride)(unsigned int width);
	unsigned int			grid_align;
	const u32			*req_ctrls;
	unsigned int			num_req;
};

struct histb_vdec_chip {
	const char *model;
	const struct histb_vdec_fmt *const *fmts;
	unsigned int num_fmts;
};

/*
 * Prefix every backend context has to start with.  The core allocates it and
 * the backend's own structure follows it in the same allocation, so the
 * protocol sources keep addressing their fields by name.
 */
struct histb_vdec_core_ctx {
	struct v4l2_fh			fh;
	struct v4l2_ctrl_handler	ctrl_hdl;
	const struct histb_vdec_fmt	*desc;
	struct v4l2_format		out_fmt;
	struct v4l2_format		cap_fmt;
};

struct histb_vdec_job {
	struct histb_vdec_core_ctx	*ctx;
	unsigned int		gen;
};

struct histb_vdec_common {
	struct device			*dev;
	const struct histb_vdec_gen	*gen;
	void				*priv;	/* backend device */

	struct v4l2_device		v4l2_dev;
	struct media_device		mdev;
	struct video_device		*vdev;
	struct v4l2_m2m_dev		*m2m_dev;

	/* serialises the V4L2 ioctls: video node and both vb2 queues */
	struct mutex			vdev_lock;
	/* guards @job and @job_running against the completion paths */
	spinlock_t			job_lock;
	struct histb_vdec_job		job;
	unsigned int			job_gen;
	bool				job_running;
	bool				removing;
	struct delayed_work		watchdog;
};

struct histb_vdec_geom {
	unsigned int	pitch;		/* tile band pitch */
	unsigned int	lines;		/* luma rows of the coding grid */
	unsigned int	uv_offset;	/* chroma start behind the luma base */
};

struct histb_vdec_gen {
	const char			*name;
	size_t				ctx_size;
	const struct histb_vdec_chip	*chip;
	unsigned int			max_width;
	unsigned int			max_height;
	/* 新 context 起始的 OUTPUT 宽度；为 0 时取 @max_width。 */
	unsigned int			default_width;
	unsigned int			max_cap_buffers;
	unsigned int			min_queued_buffers;
	/* A capture queue deeper than the slot table is refused or clamped. */
	bool				cap_full_einval;
	bool				cap_clamp;

	const u32			*cap_formats;
	unsigned int			num_cap_formats;

	/* The device exported capture buffers are allocated against. */
	struct device *(*dma_dev)(struct histb_vdec_common *common);

	/* Frame geometry, which is where the two blocks differ. */
	void (*set_out_size)(struct histb_vdec_core_ctx *ctx, struct v4l2_format *f,
			     unsigned int width, unsigned int height);
	void (*set_cap_size)(struct histb_vdec_core_ctx *ctx, struct v4l2_format *f,
			     unsigned int width, unsigned int height);
	void (*set_cap_reset)(struct histb_vdec_core_ctx *ctx, struct v4l2_format *f,
			      unsigned int width, unsigned int height);
	void (*buf_geom)(struct histb_vdec_core_ctx *ctx, struct histb_vdec_geom *g);
	int (*buf_prepare)(struct histb_vdec_core_ctx *ctx, struct vb2_buffer *vb);
	int (*buf_init)(struct histb_vdec_core_ctx *ctx, struct vb2_buffer *vb);
	void (*buf_cleanup)(struct histb_vdec_core_ctx *ctx, struct vb2_buffer *vb);
	unsigned int (*min_coded_width)(u32 fourcc);

	/* Stream and buffer lifetime. */
	int (*start)(struct histb_vdec_core_ctx *ctx, const struct vb2_queue *q);
	void (*stop)(struct histb_vdec_core_ctx *ctx, const struct vb2_queue *q);
	void (*buf_release)(struct histb_vdec_core_ctx *ctx, unsigned int index);
	int (*open)(struct histb_vdec_core_ctx *ctx);
	void (*release)(struct histb_vdec_core_ctx *ctx);

	const u32 *(*required_ctrls)(const struct histb_vdec_core_ctx *ctx,
				     unsigned int *num);
	int (*try_ctrl)(const struct histb_vdec_fmt *fmt,
			struct v4l2_ctrl *ctrl);
	void (*protocol_changed)(struct histb_vdec_core_ctx *ctx);

	const struct v4l2_m2m_ops	*m2m_ops;
};

struct histb_vdec_variant {
	const struct histb_vdec_gen	*gen;
	int (*probe)(struct platform_device *pdev,
		     const struct histb_vdec_gen *gen);
	void (*remove)(struct platform_device *pdev);
	int (*runtime_suspend)(struct device *dev);
	int (*runtime_resume)(struct device *dev);
	int (*system_suspend)(struct device *dev);
	int (*system_resume)(struct device *dev);
};

extern const struct histb_vdec_variant histb_vdec_v2_mv100_variant;
extern const struct histb_vdec_variant histb_vdec_v2_cv200_variant;
extern const struct histb_vdec_variant histb_vdec_v2_mv300_variant;
extern const struct histb_vdec_variant histb_vdec_v5_mv200h_variant;

/* Registration and teardown of the media node; called by a backend's probe. */
int histb_vdec_core_init(struct histb_vdec_common *common,
			 struct platform_device *pdev,
			 const struct histb_vdec_gen *gen);
/* Quiesce the node: unregister the video device and stop new jobs. */
void histb_vdec_core_stop(struct histb_vdec_common *common);
/* Release the media controller, the m2m device and the media device. */
void histb_vdec_core_exit(struct histb_vdec_common *common);

static inline struct histb_vdec_common *
histb_vdec_common_of_ctx(const void *p)
{
	const struct histb_vdec_core_ctx *ctx = p;
	struct v4l2_device *v4l2_dev = ctx->fh.vdev->v4l2_dev;

	return container_of(v4l2_dev, struct histb_vdec_common, v4l2_dev);
}

/* Geometry helpers the core and the backends both use. */
static inline const struct v4l2_pix_format_mplane *
histb_vdec_cap_pix(const void *p)
{
	const struct histb_vdec_core_ctx *ctx = p;

	return &ctx->cap_fmt.fmt.pix_mp;
}

static inline const struct v4l2_pix_format_mplane *
histb_vdec_out_pix(const void *p)
{
	const struct histb_vdec_core_ctx *ctx = p;

	return &ctx->out_fmt.fmt.pix_mp;
}

static inline void histb_vdec_get_dims(const void *p,
				       unsigned int *width,
				       unsigned int *height)
{
	*width = histb_vdec_cap_pix(p)->width;
	*height = histb_vdec_cap_pix(p)->height;
}

void *histb_vdec_src_va(struct vb2_buffer *vb);
dma_addr_t histb_vdec_src_dma(struct vb2_buffer *vb);
/* Window for the CPU reads a backend makes of an imported OUTPUT plane. */
int histb_vdec_src_cpu_begin(struct vb2_buffer *vb);
void histb_vdec_src_cpu_end(struct vb2_buffer *vb);

/* HEVC 量化矩阵图像（256 个消息字），两代后端共用，见 histb_vdec_core.c */
void histb_vdec_hevc_pack_scaling(u32 *dst,
				  const struct v4l2_ctrl_hevc_scaling_matrix *m);

#endif /* __HISTB_VDEC_CORE_H__ */
