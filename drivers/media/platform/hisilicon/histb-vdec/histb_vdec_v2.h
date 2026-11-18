/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * HiSilicon VDH v2 无状态视频解码后端，适用于 Hi3716CV200（HiVDHV200R004）、Hi3798MV100（HiVDHV300R001）与 Hi3798MV300（HiVDHV5R2C1，固件标签 HiVDHFMWV500R001）。
 *
 * The hardware decoder consumes fully parsed syntax: the userspace submits
 * per-frame parsing results through V4L2 stateless controls, and the driver
 * packs them into the hardware's downstream message pool, programs the VDM
 * registers and rings the doorbell. Frame storage uses a table+index model:
 * capture buffers are registered by index and referenced by id from the message
 * pool.
 * V2/V3 的引擎把 NV21 帧按 256x16 分块写进每个 dpb index 各自的内部帧（见 histb_vdec_frame），job 完成时再拷出到线性的 CAPTURE buffer；V5R2C1 行优先写，NV21 直接写进 CAPTURE buffer（见 histb_vdec_dev.direct_capture），只有 NV12 还经内部帧拷出。
 *
 * The register layouts, message slot encodings and buffer geometries recorded
 * below were derived from the vendor decoder firmware and then checked against
 * the hardware.
 */
#ifndef HISTB_VDEC_H
#define HISTB_VDEC_H

#include <linux/delay.h>
#include <linux/platform_device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-v4l2.h>

#include "histb_vdec_core.h"
#include "histb_vdec_v2_regs.h"

struct regmap;

#define HISTB_VDEC_NAME			"histb-vdec"

/* One downstream message slot: 5 x 64-word bursts. */
#define HISTB_VDEC_SLOT_WORDS		320
#define HISTB_VDEC_SLOT_BYTES		(HISTB_VDEC_SLOT_WORDS * 4)
#define HISTB_VDEC_MAX_SLICE_SLOTS	140
#define HISTB_VDEC_UP_MSG_WORDS		4
#define HISTB_VDEC_MAX_UP_SLICES	200

#define HISTB_VDEC_MAX_DPB_SLOTS	32
#define HISTB_VDEC_MAX_APC_SLOTS	16
#define HISTB_VDEC_MAX_PMV_SLOTS	20
/*
 * 本后端任一型号能接受的最大画面：Hi3716CV200 与 Hi3798MV100 为 1920x1920，Hi3798MV300 为 2560x1440。band-line scratch 与消息池按这个包络分配；格式层对外通告并钳位到的是各型号自己的宽高（histb_vdec_gen::max_width）。
 */
#define HISTB_VDEC_MAX_WIDTH		2560
#define HISTB_VDEC_MAX_HEIGHT		1920

/* Watchdog, matching the fixed timeout the vendor firmware arms here. */
#define HISTB_VDEC_TIMEOUT		msecs_to_jiffies(500)

/* Sizes of the fixed per-protocol lookup tables (extracted from the vendor
 * firmware's constant pools).
 */
#define HISTB_VDEC_CABAC_MN_H264_SIZE	5120
#define HISTB_VDEC_CABAC_MN_HEVC_SIZE	928
/* The tile/segment info image that picture message D55 points at, sized by the
 * reservation the vendor firmware makes for its own copy of the same area (64 *
 * 4 * 128 bytes); growing it from the 2 KB first used here changed no board
 * reading, so this is fidelity to the vendor layout rather than a fix.
 */
#define HISTB_VDEC_TILE_INFO_SIZE	(64 * 4 * 128)
#define HISTB_VDEC_PPFD_SIZE		(64 * 4 * 800)

/* band-line scratch 缓冲区按厂商布局定尺寸：每块 64 * 4 * width 字节，因此跟随所支持的最大宽度。
 * Which of these a protocol actually references is fixed per protocol (see the codec sources).
 */
#define HISTB_VDEC_SCRATCH_SIZE		(64 * 4 * HISTB_VDEC_MAX_WIDTH)
enum histb_vdec_scratch {
	HISTB_SCRATCH_SED_TOP,
	HISTB_SCRATCH_PMV_TOP,
	HISTB_SCRATCH_RCNTOP,
	HISTB_SCRATCH_DBLK_TOP,
	HISTB_SCRATCH_SAO_TOP,
	HISTB_SCRATCH_PMV_LEFT,
	HISTB_SCRATCH_SAO_LEFT,
	HISTB_SCRATCH_DBLK_LEFT,
	HISTB_SCRATCH_ITRANS_TOP,
	HISTB_VDEC_NUM_SCRATCH,
};

/*
 * Single downstream/upstream message pool, allocated once per device: the VDM
 * decodes one frame at a time, so the pool is owned by the device and borrowed
 * by whichever job is running. Write-combining mapping: both the CPU (message
 * producer/consumer) and the VDM DMA master access it, and the vendor firmware
 * uses uncached mappings for the same purpose.
 *
 * Layout:
 *   [0]                          picture slot
 *   [slot 1 .. N]                slice slots
 *   [after slices]               upstream report messages
 *   [aligned]                    per-protocol tables & scratch
 */
struct histb_vdec_msgpool {
	dma_addr_t	dma;
	void		*va;
	size_t		size;
	/* Fixed offsets inside va, initialised at allocation time. */
	unsigned int	pic_off;
	unsigned int	slice_off;
	unsigned int	up_off;
	unsigned int	table_off;
};

/*
 * 照搬厂商固件对 1D 写路径施加的亮度 stride 规则：已是 256 倍数的 stride 保持不变，否则向上取整到下一个 256 字节 tile 列数，再跳到奇数列数。
 * 320 -> 768, 640 -> 768, 192 -> 256, 512 -> 512, 1920 -> 2304.
 */
static inline unsigned int histb_vdec_tile_stride(unsigned int width)
{
	unsigned int cols = width / 256;

	if (!(width % 256))
		return width;
	cols += (cols % 2) ? 2 : 1;

	return cols * 256;
}

/*
 * HEVC's band pitch is a plain ALIGN(width, 256), not the odd/even column rule
 * above: 320 -> 512 and 1920 -> 2048. Board-measured line by line - the HEVC
 * corpus only untiles byte-exactly at this pitch.
 */
static inline unsigned int histb_vdec_hevc_stride(unsigned int width)
{
	return ALIGN(width, 256);
}

/*
 * 1D 写路径的帧缓冲几何，在板上实测，并与固件为 H.264 编程的几何一致。VDM_YSTRIDE_1D 给出的是 16 行 band *之间* 的间距。V2/V3 沿用厂商布局，色度 band 从整个亮度 band 阵列再加一个逐画面 head-info 区（每个对齐行 16 字节）之后开始；V5R2C1 色度紧跟亮度、不留 head-info 区（histb_vdec_uv_offset()）。
 * The band count rounds up to an even number because field coding reserves a pair.
 *
 * band 内部各代不同。V2/V3 写 256 字节 × 16 画面行的 tile，所以那里的行距就是 tile 宽度。V5R2C1 按行优先写同样的帧，以 tile stride 为行距：在 Hi3798MV300 上以 1920x1088 实测，H.264（行距 2304 字节）与 HEVC（行距 2048 字节）的帧读回来都是 NV21 光栅图，与软件解码逐字节一致。色度相对亮度的实际偏移由 histb_vdec_uv_offset() 按型号决定（V5R2C1 色度紧跟亮度、head-info 区在色度之后，引擎不写它）。
 */
static inline unsigned int histb_vdec_band_bytes(unsigned int tile_stride)
{
	return tile_stride * 16;
}

static inline unsigned int histb_vdec_head_inf_bytes(unsigned int lines)
{
	return 16 * ALIGN(lines, 32);
}

static inline unsigned int histb_vdec_luma_bytes(unsigned int tile_stride,
						 unsigned int lines)
{
	return tile_stride * ALIGN(lines, 32);
}

/* V2/V3 的帧缓冲：亮度 band、head-info 区与色度 band（亮度的一半）之和；V5R2C1 不留 head-info 区，见 histb_vdec_frame_store_bytes()。 */
static inline unsigned int histb_vdec_frame_bytes(unsigned int tile_stride,
						  unsigned int lines)
{
	return histb_vdec_luma_bytes(tile_stride, lines) / 2 * 3 +
	       histb_vdec_head_inf_bytes(lines);
}

static inline void *histb_vdec_pic_slot(const struct histb_vdec_msgpool *p)
{
	return p->va + p->pic_off;
}

static inline dma_addr_t
histb_vdec_pic_slot_dma(const struct histb_vdec_msgpool *p)
{
	return p->dma + p->pic_off;
}

static inline void *histb_vdec_slice_slot(const struct histb_vdec_msgpool *p,
					  unsigned int i)
{
	return p->va + p->slice_off + i * HISTB_VDEC_SLOT_BYTES;
}

static inline dma_addr_t
histb_vdec_slice_slot_dma(const struct histb_vdec_msgpool *p, unsigned int i)
{
	return p->dma + p->slice_off + i * HISTB_VDEC_SLOT_BYTES;
}

static inline void *histb_vdec_up_msgs(const struct histb_vdec_msgpool *p)
{
	return p->va + p->up_off;
}

static inline dma_addr_t
histb_vdec_up_msgs_dma(const struct histb_vdec_msgpool *p)
{
	return p->dma + p->up_off;
}

static inline void *histb_vdec_tables_base(const struct histb_vdec_msgpool *p)
{
	return p->va + p->table_off;
}

static inline dma_addr_t
histb_vdec_tables_dma(const struct histb_vdec_msgpool *p)
{
	return p->dma + p->table_off;
}

struct histb_vdec_dev {
	struct histb_vdec_common common;
	struct device		*dev;
	struct device		*memdev;
	const struct histb_vdec_chip *chip;
	void __iomem		*vdm;
	void __iomem		*scd;
	struct clk_bulk_data	*clks;
	int			num_clks;
	struct reset_control	*rst;
	int			irq;

	struct delayed_work	watchdog;

	/* VDM state captured by the primary handler for the IRQ thread */
	u32			irq_state;
	/* Macroblocks the submitted picture covers, 0 when the front end does
	 * not report them; the slice reports are checked against it.
	 */
	u32			mb_total;
	/*
	 * 本 job 的 DEC_ERR 可能只是 CABAC 段尾的计数偏差（见 histb_vdec_tail_slack_only()）：非 0 时是图像的宏块/CTB 总数，slice 报告必须首尾相接地覆盖它；0 表示不放行。由 H.264（CABAC）与 HEVC 的 run() 在打开 cabac_seg_stop_bit 的型号上设置，每个 job 开始前清零。
	 */
	u32			tail_slack_total;

	/* protects the message pool and the VDM against concurrent jobs */
	struct mutex		hw_lock;
	struct histb_vdec_msgpool pool;
	bool			job_running;

	/*
	 * Hi3798MV300 把 VDH/DSP 时钟门控与 VDH/SCD/MFD 软复位放在 CRG30，而不是放在已建模的 clock provider 里；门控是 clock 的型号上为 NULL。
	 */
	struct regmap		*crg;
	/*
	 * V5R2C1 的 1D 写路径：帧是行优先的（见 histb_vdec_band_bytes()），HEVC 只有在置位 VDM_CFG1_DBLK_1D_EN 时才走这条路径。
	 */
	bool			raster;
	/*
	 * 引擎直接写进客户端的 capture buffer（NV21），不经内部帧与 CPU 拷贝：capture 的 bytesperline 取引擎行距、高度取按协议网格再按 32 行对齐的帧高，色度紧跟亮度（histb_vdec_uv_offset()），sizeimage 与厂商帧缓冲同大（histb_vdec_frame_bytes()）。参考帧也直接从 capture buffer 读。NV12 仍走内部帧加拷贝（引擎只写 NV21）。
	 *
	 * Hi3798MV300 实测（#45 内核上的出树模块，25 例测试包全部逐字节一致、错误标志 0）：拷贝每帧约 3.4 ms 省掉，H.264 1080p60 91 → 140 fps（7.3 时期 SN21 零拷贝为 141 fps）、2560x1440 58 → 83 fps、HEVC P 87 → 176 fps、B-TMVP 136 → 242 fps。
	 */
	bool			direct_capture;
	/*
	 * 仅 H.264：V5R2C1 的 slice 链以最后一个 slice 的 D63 = 0 结束，不能指向一个清空的后继槽（见 histb_h264_assemble_slice()）。HEVC 在所有型号上本来就以 0 结束链，不读这个开关。
	 */
	bool			h264_chain_null_end;
	/*
	 * V5R2C1 上 CABAC slice（HEVC 全部、H.264 的 entropy_coding_mode_flag=1）的段长要包含 rbsp_stop_one_bit，只去掉其后的对齐零位；CAVLC 仍连停止位一起去掉。厂商 H.264 语法层 CalcStreamBits 对 CABAC 的 PPS 把尾部长度减 1（留下停止位）、把头长向上取整到字节（厂商 vfmw v5.0 Hi3798MV200 h264.S，读汇编），HEVC 段长的实测在 histb_hevc_assemble_slice()，H.264 的在 histb_h264_assemble_slice()。
	 *
	 * 打开它的型号还会在 CABAC 段尾停止位恰好落在字节最低位时报出不影响像素的 DEC_ERR，由 tail_slack_total 与 histb_vdec_tail_slack_only() 放行。
	 */
	bool			cabac_seg_stop_bit;
	/*
	 * 量化矩阵按厂商 vfmw v5.0 语法层（V5R2C1 这一代）的布局写进图像消息，并接受任意矩阵：H.264 每张表逐行写、每行偶数列在前奇数列在后（histb_h264_pack_scaling()），HEVC 按列写并带 DC 字、CFG0 置 VDM_CFG0_LD_QMATRIX（histb_hevc_pack_scaling()）。不打开的型号沿用原来的光栅序布局与 histb_h264_scaling_supported() 的 H.264 判据（每表一个值、三张帧内 4x4 表相同），HEVC 量化矩阵照旧拒收。
	 */
	bool			qmatrix_vfmw5;
	/*
	 * 运动信息写回：V5R2C1 要置 VDM_CFG1_MV_OUT_EN，引擎才把当前帧的运动信息写进图像消息给它的 PMV 槽（H.264 为 D7，HEVC 为 D6），以它为 collocated 图像的后续帧（H.264 B slice 的空间与时间 direct、HEVC 的 TMVP）读的就是这个槽；不置位时引擎根本不写，槽一直是分配时的零。H.264 与 HEVC 的 histb_*_program_regs() 都读这个开关。
	 *
	 * 依据（读汇编，厂商 V5R2C1 固件 tmp/media/vfmw_v5/）：H264HAL_V5R2C1_CfgVdmReg 与 HEVCHAL_V5R2C1_CfgVdmReg 每帧都用 orr w0, w0, -128 置 CFG1 字节 1 的最高位（bit 15），再经 MFDE_ConfigReg 写 CFG1；两个函数写的寄存器偏移里都没有 VDM_DNR_MBINFO_STADDR（0x90），所以这一代置这一位不需要 0x90 指向的缓冲。Hi3798MV300 上 0x90 写入后立即读回为 0，哨兵法证明不了引擎碰不碰它指向的地址，这里只以厂商不写为据。Hi3798MV100/Hi3716CV200 保持不置：V300 的 HEVCHAL_V300R001_CfgVdmReg 用 bic #32768 显式清掉这一位，这两代的 H.264 HAL 手头没有，H.264 上置不置未读、未实测。
	 *
	 * Hi3798MV300 实测（不置位 → 置位）：解完后各 PMV 槽非零字数 0 → 数千到数万；HEVC 开 TMVP 的 I+P 码流以 P 帧为 collocated 的帧亮度 PSNR 27.3–27.9 dB → 逐字节一致，B slice 的 collocated 取自 L1 的 30 帧码流置位后同样逐字节一致；H.264 空间 direct 与时间 direct 的 30 帧 IPBPB 码流各 14 个 B 帧全部失配 → 30 帧逐字节一致，无 B 帧的码流两种取值都一致。写回量：HEVC 按每 16x16 块 16 字节写（由非零记录推断：1080p 的非零数据写到 130556 字节为止；上界取自厂商 VDMHAL_V5R2C1_CalcFsSize 给 HEVC 的 W_mb * H_mb * 16 = 130560），H.264 按每宏块 32 字节写（1080p/1440p 的非零数据恰好写到 261120/460800 字节即槽尾为止，槽后到页尾的 1024/2048 字节保持为零），都在本驱动分配的槽内。
	 */
	bool			mv_out;
	/*
	 * HEVC tiles：每帧按厂商 V5R2C1 的布局写 tile info image（图像消息 D55 指向，HEVC_WriteTileInfo），slice 消息的 D10/D43/D44 用 tile 扫描地址（HEVCHAL_V5R2C1_StartDec），见 histb_hevc_pack_tile_info() 与 histb_hevc_slice_ctb_range()。不打开的型号照旧拒收 tiles（Hi3798MV100 bring-up 时 tiles 图像都只解出不完整的第一帧就停），也不写 image。
	 */
	bool			hevc_tiles;
	/*
	 * H.264 场图像（field_pic_flag=1）与 MBAFF 帧按厂商 V5R2C1 HAL 的写法编程（histb_vdec_v2_h264.c）：图像消息 D0 高度与 CFG0 宏块数取本图像（场为半帧）、底场的当前 PMV 写半槽、参考结构码与 APC 号带场编码与奇偶、collocated PMV 按参考是否按场解出取半槽、VDM_REF_PIC_TYPE 按参考是否按场解出置位、VDM_FF_APT_EN 恒 0、末 slice 的 D44 对 MBAFF 取宏块对数；同时放开场/MBAFF 的 B slice。依据在 histb_vdec_v2_h264.c 各处注释（读厂商汇编）；Hi3798MV300 上 MBAFF、场编码与帧/场自适应码流（含 B）都与软解逐字节一致。不打开的型号保持原来的写法。
	 */
	bool			h264_interlace_vfmw5;
	/*
	 * MPEG-4 与 H.263 按厂商 V5R2C1 编程（histb_vdec_v2_mpeg4.c）：CFG1 写厂商的 0x2001C002 一组位（DDR_STRIDE 为 0、MV_OUT_EN 等）；MPEG-4 每个视频包的段长止于下一包的 resync marker，并去掉 next_resync_marker()/next_start_code() 的填充位（histb_vdec_mpeg4_strip_stuffing()）；H.263 短头图像的 fcode 取 1。不打开的型号保持 MV100 bring-up 时的写法。
	 */
	bool			mpeg4_vfmw5;
	/*
	 * MPEG-1/2 与 MPEG-4/H.263 帧按上行 slice 报告的起止宏块（字 1/字 2）判覆盖，有缺口或没覆盖到最后一个宏块即失败；MPEG-1/2 与 H.263 的 DEC_ERR 在覆盖完整时不算失败（histb_vdec_reports_cover()、histb_vdec_dec_err_benign()）。字 1/字 2 的这一用法读自厂商 V5R2C1 的 VDMHAL_V5R2C1_MakeDecReport，只在打开它的型号上用；不打开的型号保持原来的写法：MPEG-1/2 只看报告止点覆盖到底，MPEG-4/H.263 按 DEC_ERR 判。
	 */
	bool			mpeg_cov_reports;
	/*
	 * 宏块全覆盖、只带 DEC_ERR 的 MPEG-1/2 与 H.263 帧（histb_vdec_reports_cover()、histb_vdec_dec_err_benign()）解完不复位引擎。厂商 V5R2C1 固件对 DEC_ERR 从不复位（读汇编）：VDMDRV_IsrProcess → VDMDRV_AfterDec 一路没有复位，复位只发生在 VDMDRV_CheckDevice（超过 1000 ms 的超时）与 VCTRL_ResetHardware/VCTRL_CloseHardware/VCTRL_DestroyChan（经 VDMDRV_ResetVDH）里，经 HAL 表 +152 的 VDMHAL_V5R2C1_ResetVdm 或 VDH_Reset_Module（后者本体不在手头反汇编里，VDH_Reset_Mfde_Module 同样经 +152 调 ResetVdm，转录待亲验）。不打开的型号保持每个 DEC_ERR 都复位：那是 Hi3798MV100 上的实测（带 DEC_ERR 的 MPEG-2 图像之后紧接的 H.264 8/8 超时，见 histb_vdec_irq_handler()）。
	 */
	bool			covered_err_keeps_engine;
};

/*
 * One internal frame: the engine only writes 16x16 tiles, while CAPTURE is a
 * linear format, so every dpb index owns a tile buffer here and the job's
 * completion detiles it into the client's buffer. References are read from
 * these buffers too, which is why they have to outlive the job.
 *
 * direct_capture 的型号上 NV21 不用内部帧（引擎直写 capture，见 histb_vdec_dev.direct_capture），只有 NV12（引擎只写 NV21，要拷贝时交换 U/V）还走这里。
 *
 * 内部帧用可缓存映射（dma_alloc_noncoherent()）：每个 job 完成时 CPU 要把整帧读一遍，而这颗 SoC 的 DMA 不一致，dma_alloc_wc() 给的是 Normal-NC 映射，A53 在上面读得极慢。Hi3798MV300 实测：1080p 一帧的拷贝 29.9 ms → 3.0~3.4 ms（另加 0.3~0.5 ms 失效缓存），2560x1440 53 ms → 4.3~4.7 ms；H.264 1080p60 解码吞吐 26.7 → 92 fps。
 */
struct histb_vdec_frame {
	void		*va;
	dma_addr_t	dma;
	size_t		size;
	unsigned int	stride;
	unsigned int	lines;
};

struct histb_vdec_ctx {
	struct v4l2_fh		fh;
	struct v4l2_ctrl_handler ctrl_hdl;
	const struct histb_vdec_fmt *desc;
	struct v4l2_format	out_fmt;
	struct v4l2_format	cap_fmt;

	/* dpb (capture buffer) index -> pmv slot id, filled on demand by the
	 * codec layers; pmv slots are a private 20-entry pool.
	 */
	u8			pmv_map[HISTB_VDEC_MAX_DPB_SLOTS];
	u8			pmv_next;

	struct histb_vdec_frame	frame[HISTB_VDEC_MAX_DPB_SLOTS];

	/* MPEG-2 front end: the submitted stream carries no sequence
	 * extension, so the block has to be told MPEG-1 syntax.
	 */
	bool			mpeg1;

	/* One verbose engine-flag report per stream: the flag is set on frames
	 * that still traverse every macroblock, so a per-frame report is noise.
	 */
	bool			diag_reported;

	/*
	 * capture 下标位图：这个 buffer 被 V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF 扣住期间（一对场的第一场）有 job 出错或被拒收。v4l2_m2m_buf_done_and_job_finish() 对扣住的 buffer 不记完成状态，第二场正常完成时整帧会以 DONE 交出，这里让它带上 ERROR。buffer 开始一帧新的解码（不是接着扣住的那一帧）时清零。
	 */
	u32			held_err;

	void			*priv;
};

struct histb_vdec_codec_ops {
	int (*start)(struct histb_vdec_ctx *ctx);
	void (*stop)(struct histb_vdec_ctx *ctx);
	/*
	 * Assemble the message pool and program the VDM for one frame, then
	 * ring the doorbell via histb_vdec_hw_kick(). Runs with dev->hw_lock
	 * held and the runtime PM reference taken.
	 */
	int (*run)(struct histb_vdec_ctx *ctx, struct vb2_v4l2_buffer *src,
		   struct vb2_v4l2_buffer *dst);
	int (*try_ctrl)(struct v4l2_ctrl *ctrl);
};

/* One protocol's descriptor, shared with the core. */
typedef struct histb_vdec_fmt histb_vdec_fmt_desc;
typedef struct histb_vdec_chip histb_vdec_chip_t;

/* Per-protocol descriptors, defined by the codec sources. */
extern const histb_vdec_fmt_desc histb_vdec_h264_fmt;
extern const histb_vdec_fmt_desc histb_vdec_hevc_fmt;
extern const histb_vdec_fmt_desc histb_vdec_mpeg2_fmt;
extern const histb_vdec_fmt_desc histb_vdec_mpeg4_fmt;
extern const histb_vdec_fmt_desc histb_vdec_h263_fmt;

/* drv.c */
int histb_vdec_v2_probe(struct platform_device *pdev,
			const struct histb_vdec_gen *gen);
void histb_vdec_v2_remove(struct platform_device *pdev);
irqreturn_t histb_vdec_irq_handler(int irq, void *priv);
int histb_vdec_cap_dims(struct histb_vdec_ctx *ctx, unsigned int *w,
			unsigned int *h);
void histb_vdec_buf_geom(struct histb_vdec_ctx *ctx, unsigned int *stride,
			 unsigned int *lines);
unsigned int histb_vdec_uv_offset(const struct histb_vdec_dev *dev,
				  unsigned int stride, unsigned int lines);
void histb_vdec_write_frame_regs(struct histb_vdec_dev *dev, dma_addr_t dst,
				 unsigned int stride, unsigned int lines);
dma_addr_t histb_vdec_frame_dma(struct histb_vdec_ctx *ctx,
				struct vb2_buffer *vb);
void histb_vdec_frames_free(struct histb_vdec_ctx *ctx);
u32 histb_vdec_scan_triple(const u8 *buf, u32 end, u32 pos, u8 last);
/* Bitstream start, honouring the plane's data_offset. */
void *histb_vdec_src_va(struct vb2_buffer *vb);
dma_addr_t histb_vdec_src_dma(struct vb2_buffer *vb);

/* hw.c */
int histb_vdec_hw_init(struct histb_vdec_dev *dev);
void histb_vdec_hw_reset(struct histb_vdec_dev *dev);
void histb_vdec_hw_kick(struct histb_vdec_dev *dev,
			enum histb_vdec_vidstd std);
/* Program the constant tables once at probe. */
void histb_vdec_hw_init_tables(struct histb_vdec_dev *dev);

static inline void vdm_write(struct histb_vdec_dev *dev, unsigned int reg,
			     u32 val)
{
	writel(val, dev->vdm + reg);
}

static inline u32 vdm_read(struct histb_vdec_dev *dev, unsigned int reg)
{
	return readl(dev->vdm + reg);
}

/* Convert an OUTPUT plane into the vendor's bitstream segment descriptor: a
 * (byte offset from the aligned stream base, bit offset, bit length) triple.
 * The hardware fetches on 16-byte granularity, so the bits inside the granule
 * join the bit offset, which is what the vendor firmware programs for every
 * segment it hands over. data_addr must be the address of the segment's first
 * bit, not of the plane start.
 */
static inline void histb_vdec_seg_pack(u32 *words, dma_addr_t stream_base,
				       dma_addr_t data_addr,
				       unsigned int bit_offset,
				       unsigned int bit_len)
{
	unsigned int bytes = (unsigned int)(data_addr - (stream_base & ~15u));

	words[0] = bytes & ~15u;
	words[1] = bit_offset + ((bytes & 15u) * 8u);
	words[2] = bit_len;
}

/*
 * Zero bits at the tail of a segment, counted from the end of the byte range
 * backwards: whole zero bytes, then the zeros below the lowest set bit of the
 * last non-zero byte (the stream is MSB-first, so the tail of the last byte is
 * the last thing written). This is the same scan the vendor firmware runs over
 * a segment before handing it to the engine; the '1' stop bit above those zeros
 * belongs to the rbsp trailing too, so callers add one. The VDM counts every
 * bit of a segment as macroblock layer, so a length that still covers the
 * padding makes it report surplus bits at the end of the segment (measured on
 * the board: one bit of padding in an interior slice is enough to set the error
 * flag).
 */
static inline u32 histb_vdec_trail_zero_bits(const u8 *va, u32 end_bit)
{
	u32 bits = 0;
	u32 byte = end_bit / 8;

	if (!va)
		return 0;
	while (byte--) {
		u8 value = va[byte];

		if (value)
			return bits + __ffs(value);
		bits += 8;
	}
	return bits;
}

/* The VDM's tile geometry, kept internal; CAPTURE itself is linear. */

/* Nominal OUTPUT (bitstream) plane size. */
#define HISTB_VDEC_OUTPUT_SIZE_DEFAULT	(3 * 1024 * 1024)

/* 消息池分配大小：要容纳随宽度缩放的 scratch 块。 */
#define HISTB_VDEC_POOL_SIZE		(8 * 1024 * 1024)

/*
 * Layout of the per-protocol table area, as offsets from
 * histb_vdec_msgpool::table_off. The CABAC M/N tables are constant and are
 * programmed once at probe, the tile info image is zeroed there (parts with
 * histb_vdec_dev.hevc_tiles rewrite it for every HEVC picture), and the
 * remaining areas are filled per frame by the message assembly code. The scratch blocks follow in enum histb_vdec_scratch
 * order, HISTB_VDEC_SCRATCH_SIZE bytes each.
 */
#define HISTB_TABLE_OFF_CABAC_MN_H264	0
#define HISTB_TABLE_OFF_CABAC_MN_HEVC	5120
#define HISTB_TABLE_OFF_TILE_INFO	6048
#define HISTB_TABLE_OFF_PPFD		(HISTB_TABLE_OFF_TILE_INFO + \
					 HISTB_VDEC_TILE_INFO_SIZE)
#define HISTB_TABLE_OFF_SCRATCH_BASE	286720
#define HISTB_TABLE_OFF_SCRATCH(i)	(HISTB_TABLE_OFF_SCRATCH_BASE + \
					 (i) * HISTB_VDEC_SCRATCH_SIZE)

#endif /* HISTB_VDEC_H */
