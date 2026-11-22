// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon Hi3798MV200 HDMI transmitter
 *
 * The on-chip HDMI 2.0 transmitter of the Hi3798MV200 family: it takes the
 * pixel stream of a VDP HD channel and turns it into a TMDS link. It is a
 * drm_bridge, the node the VDP's of_graph port@0 points at; the connector
 * belongs to the bridge connector the VDP master builds over the chain, so
 * this driver creates none of its own.
 *
 * The transmitter is two register windows -- the digital blocks and the PHY --
 * plus two divider fields and the resets inside the SoC clock generator, which
 * this driver reaches through that node's syscon regmap.
 *
 * Hi3798MV300（hisilicon,hi3798mv300-hdmi）是同一个发送器，PHY 与发送器的编程照本机实际运行的 fastboot（fastboot-hi3798mv3dmw-64.bin，与原厂 3.18 内核同一套 HDMI 代码、表的版本更新）对 MV300（芯片号 5）走的分支：分频表、驱动电流/终端表与展频表都是它的，见“Hi3798MV300 变体”一节；两种芯片经 struct histb_hdmi_variant 分派。
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/hdmi.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/mutex.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_bridge.h>
#include <drm/display/drm_hdmi_state_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>
#include <drm/drm_edid.h>
#include <drm/drm_print.h>
#include <sound/hdmi-codec.h>

/*
 * Digital window. Offsets and bit layouts are the ones in the SDK's register
 * file for this part; the reset value of TX_PWD_RST_CTRL, where only the
 * HDCP 2.2 MCU request bit is set, was read back from a board running the
 * vendor's own system.
 */
#define TX_PWD_RST_CTRL		0x0010
#define  TX_PWD_SRST		BIT(0)
#define  TX_SYS_SRST		BIT(1)
#define  TX_VID_SRST		BIT(2)
#define  TX_HDMI_SRST		BIT(3)
#define  TX_HDCP1X_SRST		BIT(4)
#define  TX_PHY_SRST		BIT(5)
#define  TX_AUD_SRST		BIT(6)
#define  TX_ACR_SRST		BIT(7)
#define  TX_AFIFO_SRST		BIT(8)
#define  TX_HDCP2X_SRST		BIT(9)
#define  TX_MCU_SRST		BIT(10)
/* HDCP is not implemented here, so those two request bits stay set. */
#define  TX_PWD_HELD		(TX_HDCP2X_SRST | TX_MCU_SRST)

#define VIDEO_DMUX_CTRL		0x09a8
/*
 * Video path (TX + 0x800) -- hal/ctrl/hisiv100/regs/hdmi_reg_video_path.h.
 * The two halves of this block decide the question the panel cannot answer for
 * us on this board: VIDEO_PATH_CTRL bit 0 blanks the outgoing video (so a set
 * bit means black even with a live source), and the FORMAT DETECTOR measures the
 * timing that actually arrives from the VDP -- hsync_active_cnt [12:0] and
 * hsync_total_cnt [25:13] at +0x8c0, the vertical pair at +0x8c4, flags
 * interlaced[0]/hsync[1]/vsync[2] at +0x8bc.  FDET_STATUS is latched: the
 * vendor re-arms it by clearing bit 3 of +0x8b8 and raising it again.
 */
#define TIMING_GEN_CTRL		0x0800
#define   TG_GEN_EN			BIT(0)
#define   TG_EXTMODE			BIT(1)
#define   TG_TIMING_SEL			GENMASK(7, 2)
#define VIDEO_PATH_CTRL		0x0830
#define   VIDEO_BLANK_EN		BIT(0)
#define FORMAT_DET_CONFIG	0x08b8
#define   FDET_SYNC_FORCE		BIT(0)
#define   FDET_HSYNC_POL_VAL		BIT(1)
#define   FDET_VSYNC_POL_VAL		BIT(2)
#define   FDET_STATUS_CLEAR		BIT(3)
#define   FDET_PIXEL_CNT_THREHOLD	GENMASK(7, 4)
#define FDET_STATUS			0x08bc
#define   FDET_INTERLACED		BIT(0)
#define FDET_HORI_RES		0x08c0
#define FDET_VERT_RES		0x08c4
#define   FDET_ACTIVE			GENMASK(12, 0)
#define   FDET_TOTAL			GENMASK(25, 13)

#define  DMUX_Y_SEL		GENMASK(2, 0)
#define  DMUX_CB_SEL		GENMASK(5, 3)
#define  DMUX_CR_SEL		GENMASK(8, 6)
#define  DMUX_SYNCMASK_EN	GENMASK(24, 21)
#define  DMUX_INVER_SYNC	GENMASK(28, 25)
#define  DMUX_INVER_V		BIT(0)
#define  DMUX_INVER_H		BIT(1)
#define  DMUX_INVER_DE		BIT(3)

#define VIDEO_DITHER_CTRL	0x08ec
#define  DITHER_MODE_8BIT	(2 << 1)
#define VIDEO_CSC_CTRL		0x0934
#define  CSC_MODE_RGB8		(0x88 << 4)
#define  CSC_EN			BIT(0)
#define  CSC_SATURATE_EN	BIT(3)
#define VIDEO_DWSM_CTRL		0x0980
#define  DWSM_VERT_BYPASS	BIT(3)
#define VIDEO_DATA_ALIGN	0x098c
#define PACK_FIFO_CTRL		0x1800
#define  PACK_FIFO_AUTO_RST_EN	BIT(2)
#define  PACK_FIFO_CK_DET_EN	BIT(4)
#define  PACK_FIFO_DELAY_CNT	GENMASK(15, 8)
/* 原厂只写 tmds_pack_mode [1:0]，其余字段是复位值：fastboot 出图时读到 0x7f14 */
#define  PACK_FIFO_DELAY_RESET	0x7f
#define PACK_FIFO_ST		0x1804
#define  PACK_FIFO_PCLK2TCLK_STABLE	BIT(0)
/* R->Cr, G->Y, B->Cb puts native RGB lanes on the three TMDS channels. */
#define  DMUX_NATIVE_RGB	0x11
/* 原厂 HAL_HDMI_CtrlVideoPathSet 对 4:4:4 输入（RGB 与 YCbCr 都一样）写的直通映射 */
#define  DMUX_IDENTITY		(FIELD_PREP_CONST(DMUX_Y_SEL, 0) | \
				 FIELD_PREP_CONST(DMUX_CB_SEL, 1) | \
				 FIELD_PREP_CONST(DMUX_CR_SEL, 2))
#define AVI_PACKET		0x1818
#define AVI_PACKET_CFG		0x19a4
#define HDMI_VS_PACKET		0x1980
#define HDMI_VS_PACKET_CFG	0x19d0
#define  PACKET_CFG_DISABLE	0
#define  PACKET_CFG_ENABLE	3
#define AVMUTE_CTRL		0x1a0c
#define  AVMUTE_SET		BIT(0)
#define  AVMUTE_CLR		BIT(1)
#define AVMIXER_CONFIG		0x1a08
#define  AVMIX_HDMI_MODE	BIT(0)
#define  AVMIX_NULL_PKT_EN	BIT(2)
/* bit8 原厂没有访问器、从不改写，fastboot 出图时为 1，是复位值；字段含义未知 */
#define  AVMIX_RESET_BIT8	BIT(8)

#define HDMI_ENC_CTRL		0x1a60
#define  ENC_HDMI2_ON		BIT(0)
#define  ENC_HDMI2_SEL		BIT(1)	/* set by the 2.0 timing rows only */
#define  ENC_BYPASS		BIT(5)
#define  ENC_CK_DIV_SEL		GENMASK(7, 6)

/*
 * PHY window. The power-down fields are active-high in name and active-low in
 * value: the vendor's PHY_POWER_ON is 0, and the values written below are the
 * ones a board running the vendor system at 3840x2160@30 was found holding
 * (PHY_TOP_PD 0x0830 out of a reset value of 0x17ffff), with the de-emphasis,
 * BIST and termination bits left as that system leaves them.
 */
#define PHY_DPHY_RST		0x0010
#define  DPHY_SRST_REQ		BIT(0)

#define PHY_AFIFO_DATA_SEL	0x0050
#define  AFIFO_DATA_CLK_H	GENMASK(18, 9)

#define PHY_AFIFO_CLK		0x0054
#define  AFIFO_DATA_CLK_L	GENMASK(29, 0)

#define PHY_TOP_PD		0x0064
#define  PHY_GC_PD		GENMASK(3, 0)
#define  PHY_GC_PD_BIST		BIT(4)
#define  PHY_GC_PD_DE		GENMASK(8, 5)
#define  PHY_GC_PD_LDO		GENMASK(10, 9)
#define  PHY_GC_PD_RTERM	GENMASK(18, 11)
#define  PHY_GC_PD_RXSENSE	BIT(19)
#define  PHY_GC_TXPLL_PD	BIT(20)
#define  PHY_DIVSEL		GENMASK(22, 21)
#define  PHY_PD_ALL		FIELD_PREP(PHY_GC_PD, 0xf)
#define  PHY_PD_BIST		FIELD_PREP(PHY_GC_PD_BIST, 1)
#define  PHY_PD_DE		FIELD_PREP(PHY_GC_PD_DE, 1)
#define  PHY_PD_LDO		FIELD_PREP(PHY_GC_PD_LDO, 3)
#define  PHY_PD_RTERM		FIELD_PREP(PHY_GC_PD_RTERM, 1)
#define  PHY_PD_RXSENSE		PHY_GC_PD_RXSENSE
#define  PHY_PD_TXPLL		PHY_GC_TXPLL_PD

/*
 * 驱动电流、预加重、终端与测试字（原厂 HDMI_APHY_DRIVER_*、RTERM_CTRL、TEST_CTRL 访问器的位段），每路一个字段：clk 与三条数据通道。
 */
#define PHY_DRIVER_IMAIN	0x0068
#define  IMAIN_CLK		GENMASK(5, 0)
#define  IMAIN_D0		GENMASK(11, 6)
#define  IMAIN_D1		GENMASK(17, 12)
#define  IMAIN_D2		GENMASK(23, 18)
#define  IMAIN_DE_CLK		GENMASK(29, 24)
#define PHY_DRIVER_IPRE		0x006c
#define  IPRE_DE_D0		GENMASK(5, 0)
#define  IPRE_DE_D1		GENMASK(11, 6)
#define  IPRE_DE_D2		GENMASK(17, 12)
#define PHY_DRIVER_RPRE		0x0074
#define  RPRE_CLK		GENMASK(8, 6)
#define  RPRE_D0		GENMASK(11, 9)
#define  RPRE_D1		GENMASK(14, 12)
#define  RPRE_D2		GENMASK(17, 15)
#define  RPRE_FLAG		BIT(2)	/* 每个 rsel_pre 字段的最高位，原厂表单列一个开关 */
#define PHY_RTERM_CTRL		0x0078
#define  RT_CLK			GENMASK(7, 0)
#define  RT_D0			GENMASK(15, 8)
#define  RT_D1			GENMASK(23, 16)
#define  RT_D2			GENMASK(31, 24)
#define PHY_TEST_CTRL		0x0080
#define  TEST_REG		GENMASK(15, 0)
#define PHY_TXPLL_CTRL		0x008c
#define  TXPLL_EN_SSCDIV	BIT(19)
#define PHY_TXPLL_TEST		0x0090
#define  TXPLL_TEST		GENMASK(19, 0)
#define  TXPLL_TEST_SSC		0x30000	/* 原厂展频打开时写的值 */

#define PHY_PLL_LOCK_STATUS	0x0098
#define  PHY_CLK_FB_EXIST	BIT(0)
#define  PHY_CLK_LOCK		BIT(20)

#define PHY_FBSSC_SET		0x00b0
#define  FBSSC_SET_FB		GENMASK(23, 0)
#define  FBSSC_SET_LOAD		BIT(24)
#define  FBSSC_SSC_MODE		BIT(27)
/*
 * The loop divider is written as (multiplier * 4) << 16, which is why the
 * vendor names it after 25. Every 24-bit row of its table uses this one.
 */
#define  FBSSC_LOOP_DIV_25	0x640000

#define PHY_FBSSC_STEP		0x00b8
#define  FBSSC_STEP		GENMASK(9, 0)
#define  FBSSC_SPAN		GENMASK(23, 10)

/*
 * Clock generator fields this device programs. Both hold a divider minus one.
 * They are picked from the mode rather than from a rate relation, which is why
 * they are not modelled as clocks (see histb_hdmi_parse_dt()).
 */
#define CRG_HDMI_CTRL		0x10c
#define  CRG_SSC_CLK_DIV	GENMASK(13, 10)
#define CRG_HDMI_PHY		0x110
#define  CRG_PHY_SRST_REQ		BIT(4)
#define  CRG_TMDS_CLK_DIV	GENMASK(8, 6)

/**
 * struct histb_hdmi_phy_cfg - one row of the PHY mode table
 * @min_khz: lowest pixel clock, in kHz, this row serves
 * @ssc_div: value for the clock generator's ssc_clk_div field, divider minus one
 * @tmds_div: value for the clock generator's tmds_clk_div field, divider minus one
 * @divsel: PHY interpolation code: 0 for 1x, 1 for 2x, 2 for 4x, 3 for 8x
 * @clk_l: per-lane pattern for the AFIFO data clock, low 30 bits
 * @clk_h: the same pattern's tenth bit of each lane, in the select register
 * @hdmi2: run the encoder in HDMI 2.0 timing
 * @ref_mul: MV300 展频计算用的参考倍数（原厂分频表第 4 列，16/8/4/2/1），mv200 的行不用
 *
 * The rows are the vendor's, unchanged: the same fields for the same pixel
 * clock bands. The 200001 kHz row is the one a board was found using at
 * 3840x2160@30 -- ssc_clk_div 9, tmds_clk_div 0, interpolation 0, and both
 * AFIFO pattern fields holding the 1:10 value below -- and 30- and 36-bit
 * modes have their own vendor rows that nothing here can ask for yet, since no
 * part of this driver programs bpc.
 */
struct histb_hdmi_phy_cfg {
	unsigned int min_khz;
	u32 ssc_div;
	u32 tmds_div;
	u32 divsel;
	u32 clk_l;
	u32 clk_h;
	bool hdmi2;
	u8 ref_mul;
};

/*
 * Each lane field of the AFIFO pattern word is ten bits wide, so a 1:10 ratio
 * is three times 0x1f at strides of ten, not of eight: 0x01f07c1f is what the
 * transmitter holds while it is driving a panel (0xf8d30054, read live).
 */
#define PHY_RATIO_1_10_L	0x01f07c1f
#define PHY_RATIO_1_40_L	0x000fffff

/*
 * One row per pixel-clock band, at 24-bit colour, which is the only depth this
 * pipeline is driven at.  ssc_div is the bypass divider minus one (CRG67's
 * field), tmds_div the TMDS Nx divider minus one (CRG68's field), divsel the
 * interpolation code (0 = 1X), and clk_l/clk_h the lane patterns of the clock
 * ratio.
 *
 * The band below 200 MHz runs 1X with tmds_div 0 because that is what this part
 * was measured to need: the system driving the attached capture card holds
 * CRG68 = 0x00000001 and PHY_TOP_PD divsel = 1X at the board's 74.25 MHz pixel
 * clock (CRG54 = 0x400d0019 in both), while the CV200's table -- which this one
 * was copied from -- interpolates 2X/4X/8X in that range.  Handing this PHY the
 * CV200 values left the sink decoding only some of the frames, so the
 * interpolated rows are this part's absence, not a smaller band.  The HDMI 2.0
 * row is the CV200's and has not been read back here.
 */
static const struct histb_hdmi_phy_cfg histb_hdmi_phy_cfgs[] = {
	{     25000, 9, 0, 0, PHY_RATIO_1_10_L, 0x1f, false, },
	{    200001, 9, 0, 0, PHY_RATIO_1_10_L, 0x1f, false, },
	{    340001, 4, 0, 0, PHY_RATIO_1_40_L, 0x00, true,  },
};

/*
 * Hi3798MV300 的行：原厂 PhyDivCfgGet 表的 24 位色行（fastboot VA 0xcc47e4，与原厂 3.18 内核 0xc1370800 的表逐字相同），列依次是 ssc_clk_div、tmds_clk_div（都已减一）、插值码、AFIFO 图样、HDMI 2.0、参考倍数。插值码与 tmds_clk_div 成对：PHY 插值 2^divsel 倍，CRG68 就分 2^divsel，把 VCO 留在同一区间。1080p60 落在 100001 行：CRG68[8:6] = 1、PHY_TOP_PD 插值码 1，fastboot 出图时读到的正是这两个值（captures/20261122-hdmi/A-fastboot.txt 的 0xf8a22110 = 0x41、0xf8d30064 = 0x00200830）；按 mv200 行写成 0/0 时 ACR 实测 CTS 翻倍、PACK_FIFO_ST 报 pclk 到 tclk 不稳，显示器无信号（同目录 B-driver-1080p.txt）。
 */
static const struct histb_hdmi_phy_cfg histb_hdmi_mv300_phy_cfgs[] = {
	{     25000, 9, 7, 3, PHY_RATIO_1_10_L, 0x1f, false, 16 },
	{     50001, 9, 3, 2, PHY_RATIO_1_10_L, 0x1f, false,  8 },
	{    100001, 9, 1, 1, PHY_RATIO_1_10_L, 0x1f, false,  4 },
	{    200001, 9, 0, 0, PHY_RATIO_1_10_L, 0x1f, false,  2 },
	{    340001, 4, 0, 0, PHY_RATIO_1_40_L, 0x00, true,   1 },
};

/**
 * struct histb_hdmi_mv300_drive - 一个 TMDS 时钟区间的驱动电流、预加重与终端
 * @min_khz: 本行起始的 TMDS 时钟，kHz
 * @de_clk: 时钟通道去加重电流（原厂电流码，写寄存器前经 histb_hdmi_mv300_cur() 换算）
 * @de_data: 三条数据通道的去加重电流
 * @main_clk: 时钟通道主驱动电流
 * @main_data: 数据通道主驱动电流
 * @pre_clk: 时钟通道 rsel_pre 的最高位
 * @pre_data: 数据通道 rsel_pre 的最高位
 * @rsel_clk: 时钟通道预加重电阻码
 * @rsel_data: 数据通道预加重电阻码
 * @rterm_mode_clk: 时钟通道终端模式 0–3，决定 PHY_TOP_PD 里该通道的两位终端关断
 * @rterm_mode_data: 数据通道终端模式
 * @rt_clk: 时钟通道终端电阻码
 * @rt_data: 数据通道终端电阻码
 * @test: PHY 测试字
 *
 * 原厂 MV300 表（fastboot VA 0xcc4cc8，PhySpecSet 对芯片号 5 选这张）每行给四条通道各一列，三条数据通道在每行都相等，所以这里合成一列。
 */
struct histb_hdmi_mv300_drive {
	unsigned int min_khz;
	u8 de_clk, de_data;
	u8 main_clk, main_data;
	bool pre_clk, pre_data;
	u8 rsel_clk, rsel_data;
	u8 rterm_mode_clk, rterm_mode_data;
	u8 rt_clk, rt_data;
	u16 test;
};

static const struct histb_hdmi_mv300_drive histb_hdmi_mv300_drives[] = {
	{  25000, 0, 0, 0x16, 0x1c, false, false, 1, 1, 2, 3, 0x60, 0x20, 0x51 },
	{ 100001, 0, 3, 0x16, 0x1d, false, false, 0, 0, 2, 3, 0x60, 0x20, 0x51 },
	{ 165001, 0, 2, 0x16, 0x1d, false, true,  2, 2, 2, 3, 0x60, 0x10, 0x51 },
	{ 340001, 0, 2, 0x16, 0x25, false, false, 1, 1, 2, 2, 0x60, 0x80, 0x51 },
};

/**
 * struct histb_hdmi_mv300_ssc - 一个 TMDS 时钟区间的展频参数
 * @min_khz: 本行起始的 TMDS 时钟，kHz
 * @amp: 展频幅度，单位 0.001 %（步长 × 跨度 ≈ 环路分频 × amp / 100000）
 * @freq_hz: 调制频率
 *
 * 原厂 MV300 表（fastboot VA 0xcc5028）。fastboot 出图时展频是开着的（A-fastboot.txt：FBSSC_SET bit27、TXPLL_CTRL bit19、TXPLL_TEST 0x30000，FBSSC_STEP 0x52813 即步长 19、跨度 330），这里照它打开；幅度为 0 的行仍进展频模式，只是步长与跨度为 0。
 */
struct histb_hdmi_mv300_ssc {
	unsigned int min_khz;
	u16 amp;
	u16 freq_hz;
};

static const struct histb_hdmi_mv300_ssc histb_hdmi_mv300_sscs[] = {
	{  25000,   0,     0 },
	{  70001, 150, 45000 },
	{  90001,  70, 45000 },
	{ 110001,  50, 45000 },
	{ 145001, 100, 45000 },
	{ 180001,  40, 45000 },
	{ 220001,  40, 45000 },
	{ 250001,  40, 45000 },
	{ 300001,   0,     0 },
};

/*
 * DDC: the transmitter's own I2C master, which the SDK names PWD.
 *
 * The master is not a general-purpose one: a transfer is described by a slave
 * address plus a segment pointer plus an offset, and the hardware emits the
 * pointer write and the repeated start itself. That is exactly the shape of an
 * EDID read, so the adapter below answers the two message forms drm_do_get_edid
 * uses -- a one or two byte pointer write followed by a block read -- plus plain
 * writes, and nothing else.
 *
 * Timeouts and the order of the status clears are the vendor's
 * (hal/ctrl/hisiv100/hdmi_hal_ddc.c). Its DDC speed counter in the always-on
 * block is left alone: the vendor never writes it either, and nothing here
 * documents what unit it counts in.
 */
#define PWD_FIFO_RDATA		0x0038
#define  PWD_FIFO_DATA_OUT	GENMASK(7, 0)
#define PWD_FIFO_WDATA		0x003c
#define  PWD_FIFO_DATA_IN	GENMASK(7, 0)
#define PWD_DATA_CNT		0x0040
#define  PWD_FIFO_DATA_CNT	GENMASK(4, 0)
#define  PWD_DATA_OUT_CNT	GENMASK(17, 8)
#define PWD_SLAVE_CFG		0x0044
#define  PWD_SLAVE_ADDR		GENMASK(7, 0)
#define  PWD_SLAVE_OFFSET	GENMASK(15, 8)
#define  PWD_SLAVE_SEG		GENMASK(23, 16)
#define PWD_MST_STATE		0x0048
#define  PWD_NO_ACK		BIT(0)
#define  PWD_BUS_LOW		BIT(1)
#define  PWD_IN_PROG		BIT(2)
#define  PWD_FIFO_EMPTY		BIT(5)
#define PWD_MST_CMD		0x004c
#define  PWD_CMD_READ_MULTI	0x2
#define  PWD_CMD_READ_SEGMENT	0x4
#define  PWD_CMD_WRITE_MULTI	0x6
#define  PWD_CMD_FIFO_CLR	0x9
#define  PWD_CMD_ABORT		0xf
#define  PWD_CMD		GENMASK(3, 0)
#define PWD_MST_CLR		0x0050
#define  PWD_CLR_STATUS		(PWD_BUS_LOW | PWD_NO_ACK)
#define DDC_MST_ARB_CTRL	0x0064
#define  DDC_AUTO_ABORT		BIT(1)
#define DDC_MST_ARB_REQ		0x0068
#define  DDC_REQ		BIT(0)
#define DDC_MST_ARB_ACK		0x006c
#define  DDC_REQ_ACK		BIT(0)
/* the always-on block's DDC status, used to check the bus is idle */
#define AON_DDC_MAN_CTRL	0x405c
#define  DDC_SCL_HIGH		BIT(0)
#define  DDC_SDA_HIGH		BIT(1)

/*
 * Hot plug lives in the always-on block too.  The reported state is inverted
 * when the polarity bit says so; the override bit is left at its reset value
 * of 0, so the pin and not a software value decides what bit 0 reports.
 */
#define AON_HOTPLUG_ST_CFG	0x4020
#define  HPD_OVERRIDE_EN	BIT(1)
#define  HPD_POLARITY_CTL	BIT(2)
#define  HPD_SOFT_VALUE		BIT(3)
#define AON_TX_AON_STATE	0x4028
#define  HPD_STATE		BIT(0)
#define  PHY_RX_SENSE		BIT(1)

#define DDC_EDID_ADDR		0xa0
#define DDC_FIFO_DEPTH		16
#define DDC_TIMEOUT_ARB_MS	100
#define DDC_TIMEOUT_SDA_MS	30
#define DDC_TIMEOUT_INPROG_MS	20
#define DDC_TIMEOUT_DATA_MS	20

struct histb_hdmi_ddc {
	struct i2c_adapter adapter;
	struct mutex lock;
};


#define HISTB_HDMI_MAX_KHZ	600000

struct histb_hdmi;

/**
 * struct histb_hdmi_variant - 芯片之间不同的编程
 * @phy_cfgs: 按像素时钟分段的 PHY/CRG 分频表，升序
 * @num_phy_cfgs: @phy_cfgs 的行数
 * @set_phy: 按 @phy_cfgs 的一行与模式时钟（kHz）配置 PHY 并等锁定，通道驱动保持关闭
 * @phy_drive_on: 视频通路配好之后打开 TMDS 通道驱动
 * @vmux: VIDEO_DMUX_CTRL，4:4:4 输入
 * @pack_fifo: PACK_FIFO_CTRL
 * @avmixer: AVMIXER_CONFIG
 * @avmute: AVMUTE_CTRL（General Control 包的 Set/Clear_AVMUTE）
 * @pwd_running: 出图期间 TX_PWD_RST_CTRL 保持置位的复位请求
 * @product: drm_bridge 的产品名
 */
struct histb_hdmi_variant {
	const struct histb_hdmi_phy_cfg *phy_cfgs;
	unsigned int num_phy_cfgs;
	int (*set_phy)(struct histb_hdmi *hdmi,
		       const struct histb_hdmi_phy_cfg *cfg, unsigned int khz);
	void (*phy_drive_on)(struct histb_hdmi *hdmi, unsigned int khz);
	u32 vmux;
	u32 pack_fifo;
	u32 avmixer;
	u32 avmute;
	u32 pwd_running;
	const char *product;
};

struct histb_hdmi {
	const struct histb_hdmi_variant *variant;
	struct device *dev;
	void __iomem *regs;
	void __iomem *phy;
	struct regmap *crg;
	struct clk *clk_pix;
	struct clk *clk_tmds;
	struct reset_control *resets;
	struct histb_hdmi_ddc ddc;
	unsigned int audio_tmds_khz;
	struct drm_bridge bridge;
};

static struct histb_hdmi *bridge_to_histb_hdmi(struct drm_bridge *bridge)
{
	return container_of(bridge, struct histb_hdmi, bridge);
}

static u32 histb_hdmi_read(struct histb_hdmi *hdmi, u32 offset)
{
	return readl_relaxed(hdmi->regs + offset);
}

static void histb_hdmi_write(struct histb_hdmi *hdmi, u32 offset, u32 val)
{
	writel_relaxed(val, hdmi->regs + offset);
}

static u32 histb_hdmi_phy_read(struct histb_hdmi *hdmi, u32 offset)
{
	return readl_relaxed(hdmi->phy + offset);
}

static void histb_hdmi_phy_write(struct histb_hdmi *hdmi, u32 offset, u32 val)
{
	writel_relaxed(val, hdmi->phy + offset);
}

static const struct histb_hdmi_phy_cfg *
histb_hdmi_phy_cfg(const struct histb_hdmi *hdmi, u32 khz)
{
	const struct histb_hdmi_variant *variant = hdmi->variant;
	const struct histb_hdmi_phy_cfg *best = NULL;
	unsigned int i;

	for (i = 0; i < variant->num_phy_cfgs; i++)
		if (khz >= variant->phy_cfgs[i].min_khz)
			best = &variant->phy_cfgs[i];

	return best;
}

/*
 * Bring the PHY up for one mode. The order is the vendor's and the board shows
 * what it leaves behind: divider fields first, then a wait, then a pulse of the
 * PHY's soft reset, then the interpolation and pattern fields, then the loop
 * divider with its load strobe.
 *
 * Spreading spectrum is left off: the writes below take the shape of the
 * vendor's disabled branch, whose step and span registers stay at zero, and the
 * enabled branch needs a span and step computed from the mode that nothing here
 * derives yet.
 */
static int histb_hdmi_set_phy(struct histb_hdmi *hdmi,
			       const struct histb_hdmi_phy_cfg *cfg,
			       unsigned int khz)
{
	u32 val;
	int ret;

	ret = regmap_update_bits(hdmi->crg, CRG_HDMI_CTRL, CRG_SSC_CLK_DIV,
				 FIELD_PREP(CRG_SSC_CLK_DIV, cfg->ssc_div));
	if (ret)
		return ret;

	ret = regmap_update_bits(hdmi->crg, CRG_HDMI_PHY, CRG_TMDS_CLK_DIV,
				 FIELD_PREP(CRG_TMDS_CLK_DIV, cfg->tmds_div));
	if (ret)
		return ret;

	/* The vendor waits after the clock generator before touching the PHY. */
	usleep_range(1000, 1200);

	/*
	 * Then it pulses the PHY's own soft reset, on this part only, so that
	 * what the loop locks to is the new divider. That request bit is a
	 * field of the same register the dividers live in and is written here
	 * rather than handed through the reset framework, because the pulse
	 * belongs to the divider change and to nothing else.
	 */
	ret = regmap_update_bits(hdmi->crg, CRG_HDMI_PHY, CRG_PHY_SRST_REQ,
				 CRG_PHY_SRST_REQ);
	if (ret)
		return ret;
	udelay(10);
	ret = regmap_update_bits(hdmi->crg, CRG_HDMI_PHY, CRG_PHY_SRST_REQ, 0);
	if (ret)
		return ret;
	usleep_range(1000, 1200);

	histb_hdmi_phy_write(hdmi, PHY_TOP_PD,
			     PHY_PD_BIST | PHY_PD_DE | PHY_PD_RTERM |
			     FIELD_PREP(PHY_DIVSEL, cfg->divsel));

	histb_hdmi_phy_write(hdmi, PHY_AFIFO_CLK,
			     FIELD_PREP(AFIFO_DATA_CLK_L, cfg->clk_l));
	val = histb_hdmi_phy_read(hdmi, PHY_AFIFO_DATA_SEL);
	histb_hdmi_phy_write(hdmi, PHY_AFIFO_DATA_SEL,
			     (val & ~AFIFO_DATA_CLK_H) |
			     FIELD_PREP(AFIFO_DATA_CLK_H, cfg->clk_h));

	/*
	 * The loop divider takes a load strobe: the value without it, then with
	 * it, then without again, which is the state the board was found in
	 * (its reset value leaves the strobe set).
	 */
	val = FIELD_PREP(FBSSC_SET_FB, FBSSC_LOOP_DIV_25);
	histb_hdmi_phy_write(hdmi, PHY_FBSSC_SET, val);
	histb_hdmi_phy_write(hdmi, PHY_FBSSC_SET, val | FBSSC_SET_LOAD);
	histb_hdmi_phy_write(hdmi, PHY_FBSSC_SET, val);

	return readl_relaxed_poll_timeout(hdmi->phy + PHY_PLL_LOCK_STATUS, val,
					  (val & PHY_CLK_LOCK) &&
					  (val & PHY_CLK_FB_EXIST), 100, 20000);
}

/* Drive the pins last: from here a sink can find a clock. */
static void histb_hdmi_phy_drive_on(struct histb_hdmi *hdmi, unsigned int khz)
{
	histb_hdmi_phy_write(hdmi, PHY_TOP_PD,
			     histb_hdmi_phy_read(hdmi, PHY_TOP_PD) & ~PHY_GC_PD);
}

/* ------------------------------------------------------------------ */
/* Hi3798MV300 变体                                                    */
/* ------------------------------------------------------------------ */

/*
 * 依据是本机 p1 上实际运行的 fastboot（firmware/histb/TTL-mv300-64/fastboot-hi3798mv3dmw-64.bin，A32，VA = 文件偏移 + 0xc00000）的 HDMI PHY 代码，函数按与原厂 3.18 内核（有符号）逐条对照认出：HAL_HDMI_PhyTmdsSet @0xc6c15c、HAL_HDMI_PhySpecSet @0xc6b5c4、HAL_HDMI_PhySscSet @0xc6bd58、HAL_HDMI_PhyPowerSet @0xc6b14c、HAL_HDMI_PhyOeSet @0xc6b208、DRV_HDMI_ProdCrgDivSet @0xc70244、DRV_HDMI_ProdChipGet @0xc6fb24（Hi3798M + 版本 0x300 → 芯片号 5）。与 mv200 路径的差别：分频按原厂表成对设插值与 CRG68 分频；PHY 复位用 PHY 窗口里的 DPHY_RST，不脉冲 CRG68 bit4（原厂只对芯片号 3 即 MV200 脉冲后者，它会把模拟 PHY 的驱动与终端寄存器清回复位值）；写驱动电流、预加重、终端与测试字；打开展频。
 */

/* 原厂 PhyTransferCur2Hex：电流码到寄存器码 */
static u32 histb_hdmi_mv300_cur(u8 cur)
{
	if (cur <= 15)
		return cur;
	if (cur <= 27)
		return cur + 4;
	if (cur <= 31)
		return cur + 16;
	if (cur <= 43)
		return cur + 20;
	return 0;
}

static const struct histb_hdmi_mv300_drive *histb_hdmi_mv300_drive(unsigned int khz)
{
	const struct histb_hdmi_mv300_drive *best = NULL;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(histb_hdmi_mv300_drives); i++)
		if (khz >= histb_hdmi_mv300_drives[i].min_khz)
			best = &histb_hdmi_mv300_drives[i];

	return best;
}

static const struct histb_hdmi_mv300_ssc *histb_hdmi_mv300_ssc(unsigned int khz)
{
	const struct histb_hdmi_mv300_ssc *best = NULL;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(histb_hdmi_mv300_sscs); i++)
		if (khz >= histb_hdmi_mv300_sscs[i].min_khz)
			best = &histb_hdmi_mv300_sscs[i];

	return best;
}

static void histb_hdmi_phy_rmw(struct histb_hdmi *hdmi, u32 offset, u32 mask,
			       u32 val)
{
	histb_hdmi_phy_write(hdmi, offset,
			     (histb_hdmi_phy_read(hdmi, offset) & ~mask) | val);
}

/*
 * 原厂 PhySscSet 的打开分支：参考频率 = 像素时钟 × 参考倍数 ÷ (ssc_clk_div + 1)，跨度 = 参考频率 ÷ 调制频率 ÷ 4，步长 = 幅度 × (环路分频 ÷ 100) × (调制频率 ÷ 1000) ÷ (参考频率 ÷ 4)，整数运算的次序照原厂。1080p60：参考 59.4 MHz，跨度 330、步长 19。
 */
static void histb_hdmi_mv300_set_ssc(struct histb_hdmi *hdmi,
				     const struct histb_hdmi_phy_cfg *cfg,
				     unsigned int khz)
{
	const struct histb_hdmi_mv300_ssc *ssc = histb_hdmi_mv300_ssc(khz);
	u32 ref = (u32)div_u64((u64)khz * cfg->ref_mul * 1000, cfg->ssc_div + 1);
	u32 step = 0, span = 0;

	if (ssc && ssc->freq_hz && ref >= 4) {
		span = ref / ssc->freq_hz / 4;
		step = (u32)div_u64((u64)ssc->amp * (FBSSC_LOOP_DIV_25 / 100) *
				    (ssc->freq_hz / 1000), ref / 4);
	}

	histb_hdmi_phy_rmw(hdmi, PHY_FBSSC_STEP, FBSSC_STEP | FBSSC_SPAN,
			   FIELD_PREP(FBSSC_STEP, step) |
			   FIELD_PREP(FBSSC_SPAN, span));
	histb_hdmi_phy_rmw(hdmi, PHY_FBSSC_SET, FBSSC_SSC_MODE | FBSSC_SET_LOAD,
			   FBSSC_SSC_MODE);
	histb_hdmi_phy_rmw(hdmi, PHY_FBSSC_SET, FBSSC_SET_LOAD, FBSSC_SET_LOAD);
	histb_hdmi_phy_rmw(hdmi, PHY_FBSSC_SET, FBSSC_SET_LOAD, 0);
	histb_hdmi_phy_rmw(hdmi, PHY_TXPLL_TEST, TXPLL_TEST, TXPLL_TEST_SSC);
	histb_hdmi_phy_rmw(hdmi, PHY_TXPLL_CTRL, TXPLL_EN_SSCDIV, TXPLL_EN_SSCDIV);
}

/* 原厂 PhySpecSet 的芯片号 5 分支（PhyHWEnhanceSpecSet）；表按 TMDS 时钟查，24 位色下即像素时钟 */
static void histb_hdmi_mv300_set_drive(struct histb_hdmi *hdmi,
				       const struct histb_hdmi_mv300_drive *drv)
{
	u32 main_d = histb_hdmi_mv300_cur(drv->main_data);
	u32 de_d = histb_hdmi_mv300_cur(drv->de_data);
	u32 rsel_d = drv->rsel_data | (drv->pre_data ? RPRE_FLAG : 0);

	histb_hdmi_phy_rmw(hdmi, PHY_DRIVER_IMAIN,
			   IMAIN_CLK | IMAIN_D0 | IMAIN_D1 | IMAIN_D2 | IMAIN_DE_CLK,
			   FIELD_PREP(IMAIN_CLK, histb_hdmi_mv300_cur(drv->main_clk)) |
			   FIELD_PREP(IMAIN_D0, main_d) |
			   FIELD_PREP(IMAIN_D1, main_d) |
			   FIELD_PREP(IMAIN_D2, main_d) |
			   FIELD_PREP(IMAIN_DE_CLK, histb_hdmi_mv300_cur(drv->de_clk)));
	histb_hdmi_phy_rmw(hdmi, PHY_DRIVER_IPRE,
			   IPRE_DE_D0 | IPRE_DE_D1 | IPRE_DE_D2,
			   FIELD_PREP(IPRE_DE_D0, de_d) |
			   FIELD_PREP(IPRE_DE_D1, de_d) |
			   FIELD_PREP(IPRE_DE_D2, de_d));
	histb_hdmi_phy_rmw(hdmi, PHY_DRIVER_RPRE,
			   RPRE_CLK | RPRE_D0 | RPRE_D1 | RPRE_D2,
			   FIELD_PREP(RPRE_CLK, drv->rsel_clk |
				      (drv->pre_clk ? RPRE_FLAG : 0)) |
			   FIELD_PREP(RPRE_D0, rsel_d) |
			   FIELD_PREP(RPRE_D1, rsel_d) |
			   FIELD_PREP(RPRE_D2, rsel_d));
	histb_hdmi_phy_write(hdmi, PHY_RTERM_CTRL,
			     FIELD_PREP(RT_CLK, drv->rt_clk) |
			     FIELD_PREP(RT_D0, drv->rt_data) |
			     FIELD_PREP(RT_D1, drv->rt_data) |
			     FIELD_PREP(RT_D2, drv->rt_data));
	histb_hdmi_phy_rmw(hdmi, PHY_TEST_CTRL, TEST_REG,
			   FIELD_PREP(TEST_REG, drv->test));
}

/*
 * 原厂 PhyTmdsSet 的次序：PHY 上电（txpll、rxsense、ldo 打开，bist 保持关）、插值码、CRG 两个分频、DPHY 软复位脉冲、等 1 ms、AFIFO 图样、环路分频、展频、驱动与终端。之后等 TX PLL 锁定（原厂不等，这里沿用 mv200 路径的判据）。进来先把四路驱动关掉：fastboot 留下的通道可能正在发。
 */
static int histb_hdmi_mv300_set_phy(struct histb_hdmi *hdmi,
				    const struct histb_hdmi_phy_cfg *cfg,
				    unsigned int khz)
{
	const struct histb_hdmi_mv300_drive *drv = histb_hdmi_mv300_drive(khz);
	u32 val;
	int ret;

	if (!drv)
		return -EINVAL;

	histb_hdmi_phy_rmw(hdmi, PHY_TOP_PD, PHY_GC_PD, PHY_PD_ALL);
	histb_hdmi_phy_rmw(hdmi, PHY_TOP_PD,
			   PHY_PD_TXPLL | PHY_PD_RXSENSE | PHY_GC_PD_LDO |
			   PHY_GC_PD_BIST, PHY_PD_BIST);
	histb_hdmi_phy_rmw(hdmi, PHY_TOP_PD, PHY_DIVSEL,
			   FIELD_PREP(PHY_DIVSEL, cfg->divsel));

	ret = regmap_update_bits(hdmi->crg, CRG_HDMI_CTRL, CRG_SSC_CLK_DIV,
				 FIELD_PREP(CRG_SSC_CLK_DIV, cfg->ssc_div));
	if (ret)
		return ret;
	ret = regmap_update_bits(hdmi->crg, CRG_HDMI_PHY, CRG_TMDS_CLK_DIV,
				 FIELD_PREP(CRG_TMDS_CLK_DIV, cfg->tmds_div));
	if (ret)
		return ret;

	histb_hdmi_phy_rmw(hdmi, PHY_DPHY_RST, DPHY_SRST_REQ, DPHY_SRST_REQ);
	histb_hdmi_phy_rmw(hdmi, PHY_DPHY_RST, DPHY_SRST_REQ, 0);
	usleep_range(1000, 1200);

	histb_hdmi_phy_rmw(hdmi, PHY_AFIFO_DATA_SEL, AFIFO_DATA_CLK_H,
			   FIELD_PREP(AFIFO_DATA_CLK_H, cfg->clk_h));
	histb_hdmi_phy_rmw(hdmi, PHY_AFIFO_CLK, AFIFO_DATA_CLK_L,
			   FIELD_PREP(AFIFO_DATA_CLK_L, cfg->clk_l));
	histb_hdmi_phy_rmw(hdmi, PHY_FBSSC_SET, FBSSC_SET_FB,
			   FIELD_PREP(FBSSC_SET_FB, FBSSC_LOOP_DIV_25));

	histb_hdmi_mv300_set_ssc(hdmi, cfg, khz);
	histb_hdmi_mv300_set_drive(hdmi, drv);

	return readl_relaxed_poll_timeout(hdmi->phy + PHY_PLL_LOCK_STATUS, val,
					  (val & PHY_CLK_LOCK) &&
					  (val & PHY_CLK_FB_EXIST), 100, 20000);
}

/*
 * 原厂 PhyOeSet(1)：去加重电流为 0 的通道关掉去加重（clk 是 bit0、数据通道依次 bit1–3），终端按各通道的模式关断（模式 m 关掉该通道两位里的 3 - m 那几位，模式 3 全开），最后打开四路驱动。
 */
static void histb_hdmi_mv300_phy_drive_on(struct histb_hdmi *hdmi,
					  unsigned int khz)
{
	const struct histb_hdmi_mv300_drive *drv = histb_hdmi_mv300_drive(khz);
	u32 de_pd = 0, rterm_pd;

	if (!drv)
		return;

	if (!drv->de_clk)
		de_pd |= BIT(0);
	if (!drv->de_data)
		de_pd |= BIT(1) | BIT(2) | BIT(3);

	rterm_pd = (3 - drv->rterm_mode_clk) |
		   (3 - drv->rterm_mode_data) << 2 |
		   (3 - drv->rterm_mode_data) << 4 |
		   (3 - drv->rterm_mode_data) << 6;

	histb_hdmi_phy_rmw(hdmi, PHY_TOP_PD,
			   PHY_GC_PD_DE | PHY_GC_PD_RTERM | PHY_GC_PD,
			   FIELD_PREP(PHY_GC_PD_DE, de_pd) |
			   FIELD_PREP(PHY_GC_PD_RTERM, rterm_pd));
}

static const struct histb_hdmi_variant histb_hdmi_mv200_variant = {
	.phy_cfgs	= histb_hdmi_phy_cfgs,
	.num_phy_cfgs	= ARRAY_SIZE(histb_hdmi_phy_cfgs),
	.set_phy	= histb_hdmi_set_phy,
	.phy_drive_on	= histb_hdmi_phy_drive_on,
	.vmux		= DMUX_NATIVE_RGB,
	.pack_fifo	= PACK_FIFO_AUTO_RST_EN | PACK_FIFO_CK_DET_EN,
	.avmixer	= AVMIX_HDMI_MODE | AVMIX_NULL_PKT_EN,
	.avmute		= 0,
	.pwd_running	= 0,
	.product	= "Hi3798MV200 HDMI",
};

/*
 * 发送器侧的几个字照 fastboot 出图时的读数（A-fastboot.txt），各自的原厂出处：VIDEO_DMUX_CTRL 直通（HAL_HDMI_CtrlVideoPathSet 的 4:4:4 分支，VDP 送 RGB 时同样直通）；PACK_FIFO_CTRL 保留复位值 0x7f14（原厂只写 tmds_pack_mode）；AVMIXER_CONFIG 只置 hdmi_mode（HAL_HDMI_CtrlTmdsModeSet），空包不开、bit8 保持复位值；General Control 包发 Clear_AVMUTE（CtrlAvmuteSet）；HDCP 2.2 的 MCU 一直按在复位里（只有 HDCP22_AuthStart 才放开）。
 */
static const struct histb_hdmi_variant histb_hdmi_mv300_variant = {
	.phy_cfgs	= histb_hdmi_mv300_phy_cfgs,
	.num_phy_cfgs	= ARRAY_SIZE(histb_hdmi_mv300_phy_cfgs),
	.set_phy	= histb_hdmi_mv300_set_phy,
	.phy_drive_on	= histb_hdmi_mv300_phy_drive_on,
	.vmux		= DMUX_IDENTITY,
	.pack_fifo	= FIELD_PREP_CONST(PACK_FIFO_DELAY_CNT, PACK_FIFO_DELAY_RESET) |
			  PACK_FIFO_AUTO_RST_EN | PACK_FIFO_CK_DET_EN,
	.avmixer	= AVMIX_HDMI_MODE | AVMIX_RESET_BIT8,
	.avmute		= AVMUTE_CLR,
	.pwd_running	= TX_MCU_SRST,
	.product	= "Hi3798MV300 HDMI",
};

/*
 * A mode survives only if both halves of the clock chain accept it: the PHY has
 * a divider set for its band, and the display FPLL can synthesise the pixel
 * clock itself.  The FPLL solver only tolerates 0.03 %, which is what rejects a
 * mode like 1024x768@60 reduced blanking (65.12256 MHz has no configuration)
 * here instead of at commit time.
 */
static enum drm_mode_status histb_hdmi_mode_valid(struct drm_bridge *bridge,
						  const struct drm_display_info *info,
						  const struct drm_display_mode *mode)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);

	if (!histb_hdmi_phy_cfg(hdmi, mode->clock))
		return MODE_CLOCK_RANGE;

	if (mode->clock > HISTB_HDMI_MAX_KHZ)
		return MODE_CLOCK_HIGH;

	if (clk_round_rate(hdmi->clk_pix, mode->clock * 1000) < 0)
		return MODE_NOCLOCK;

	return MODE_OK;
}

/*
 * Colour format.  The transmitter's own converter is what makes YCbCr444 out
 * of the RGB the display interface hands over; the mode byte selects the
 * direction, colourimetry and quantization range (bit 7 RGB in, bit 6 full
 * range in, bits [5:4] colourimetry in, bit 3 RGB out, bit 2 full range out,
 * bits [1:0] colourimetry out, at [11:4] in the register).  RGB is declared
 * with the converter off.  The YCbCr output is limited range: asking the
 * converter for full range stops the packer on this part, so the mode leaves
 * bit 2 clear and the infoframe says limited.
 */
static void histb_hdmi_color_csc(struct histb_hdmi *hdmi, bool yuv444, u32 khz)
{
	u32 val;

	histb_hdmi_write(hdmi, VIDEO_DMUX_CTRL, hdmi->variant->vmux);

	if (!yuv444) {
		val = CSC_MODE_RGB8;
	} else {
		u32 cm = khz > 27000 ? 1 : 0;

		val = ((BIT(7) | BIT(6) | cm << 4 | cm) << 4) |
		      CSC_EN | CSC_SATURATE_EN;
	}

	histb_hdmi_write(hdmi, VIDEO_CSC_CTRL, val);
}

static int histb_hdmi_atomic_check(struct drm_bridge *bridge,
				   struct drm_bridge_state *bridge_state,
				   struct drm_crtc_state *crtc_state,
				   struct drm_connector_state *conn_state)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);
	const struct drm_display_mode *mode = &crtc_state->mode;

	if (!histb_hdmi_phy_cfg(hdmi, mode->clock) ||
	    mode->clock > HISTB_HDMI_MAX_KHZ)
		return -EINVAL;

	/* The converter always produces limited-range YCbCr. */
	if (conn_state->hdmi.output_format == DRM_OUTPUT_COLOR_FORMAT_YCBCR444)
		conn_state->hdmi.is_limited_range = true;

	return 0;
}


static int histb_hdmi_write_avi_infoframe(struct drm_bridge *bridge,
					  const u8 *buffer, size_t length);

static void histb_hdmi_atomic_enable(struct drm_bridge *bridge,
				     struct drm_atomic_commit *commit)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);
	struct drm_device *drm = bridge->dev;
	struct drm_connector *connector;
	struct drm_connector_state *conn_state;
	const struct drm_display_mode *mode;
	const struct histb_hdmi_phy_cfg *cfg;
	bool yuv444;
	int ret;

	mode = &bridge->encoder->crtc->state->adjusted_mode;
	cfg = histb_hdmi_phy_cfg(hdmi, mode->clock);
	if (!cfg) {
		drm_err(drm, "no HDMI PHY setting for %u kHz\n", mode->clock);
		return;
	}

	/*
	 * The ratio bit that could halve the TMDS clock against the pixel clock
	 * is pinned to 1:1 by the pixel clock's .prepare, since the VDP register
	 * file cannot be reached otherwise, so the mode clock is also the TMDS
	 * clock here -- which is what the audio ACR needs in order to pick N.
	 */
	hdmi->audio_tmds_khz = mode->clock;

	ret = clk_prepare_enable(hdmi->clk_pix);
	if (ret) {
		drm_err(drm, "failed to enable the pixel clock: %d\n", ret);
		return;
	}

	ret = clk_prepare_enable(hdmi->clk_tmds);
	if (ret) {
		drm_err(drm, "failed to enable the TMDS clock: %d\n", ret);
		goto err_pix;
	}

	ret = reset_control_deassert(hdmi->resets);
	if (ret) {
		drm_err(drm, "failed to release the transmitter resets: %d\n", ret);
		goto err_tmds;
	}

	/* The controller's own block resets are released here, not in the
	 * clock generator: everything but HDCP comes out of reset at once.
	 */
	histb_hdmi_write(hdmi, TX_PWD_RST_CTRL, TX_PWD_HELD);

	ret = hdmi->variant->set_phy(hdmi, cfg, mode->clock);
	if (ret) {
		drm_err(drm, "HDMI PHY did not lock for %u kHz: %d\n",
			mode->clock, ret);
		goto err_resets;
	}

	connector = drm_atomic_get_new_connector_for_encoder(commit,
							     bridge->encoder);
	conn_state = connector ?
		drm_atomic_get_new_connector_state(commit, connector) : NULL;
	yuv444 = conn_state &&
		 conn_state->hdmi.output_format ==
		 DRM_OUTPUT_COLOR_FORMAT_YCBCR444;
	histb_hdmi_color_csc(hdmi, yuv444, mode->clock);

	/*
	 * Video path: 8-bit components straight into the encoder, dither and the
	 * transmitter's own conversion off, one data reset, then the packer's
	 * clock detection.  Without that last bit the packer emits nothing and a
	 * sink sees a clock with no data on it.
	 */
	histb_hdmi_write(hdmi, VIDEO_DITHER_CTRL, DITHER_MODE_8BIT);
	histb_hdmi_write(hdmi, VIDEO_DWSM_CTRL, DWSM_VERT_BYPASS);
	histb_hdmi_write(hdmi, VIDEO_DATA_ALIGN, 0);
	histb_hdmi_write(hdmi, TIMING_GEN_CTRL,
			 histb_hdmi_read(hdmi, TIMING_GEN_CTRL) & ~TG_GEN_EN);
	histb_hdmi_write(hdmi, VIDEO_PATH_CTRL, 0);
	histb_hdmi_write(hdmi, HDMI_ENC_CTRL,
			 cfg->hdmi2 ? ENC_HDMI2_ON | ENC_HDMI2_SEL : 0);
	histb_hdmi_write(hdmi, AVMIXER_CONFIG, hdmi->variant->avmixer);
	histb_hdmi_write(hdmi, AVMUTE_CTRL, hdmi->variant->avmute);

	histb_hdmi_write(hdmi, TX_PWD_RST_CTRL,
			 hdmi->variant->pwd_running |
			 TX_HDMI_SRST | TX_AUD_SRST | TX_ACR_SRST |
			 TX_AFIFO_SRST);
	udelay(5);
	histb_hdmi_write(hdmi, TX_PWD_RST_CTRL, hdmi->variant->pwd_running);
	histb_hdmi_write(hdmi, PACK_FIFO_CTRL, hdmi->variant->pack_fifo);
	usleep_range(1000, 2000);

	/*
	 * 原厂 HAL_HDMI_CtrlTmdsStableGet 的判据：像素时钟到 TMDS 时钟的打包 FIFO 稳定。MV300 上分频与插值不配对时它读 0，显示器那头是无信号，所以这里报出来。
	 */
	if (!(histb_hdmi_read(hdmi, PACK_FIFO_ST) & PACK_FIFO_PCLK2TCLK_STABLE))
		drm_warn(drm, "HDMI pack FIFO not stable at %u kHz: %08x\n",
			 mode->clock, histb_hdmi_read(hdmi, PACK_FIFO_ST));

	/*
	 * Push the infoframes the sink needs to interpret the lanes; without
	 * them it has to guess, and the guess is not RGB.
	 */
	if (connector) {
		ret = drm_atomic_helper_connector_hdmi_update_infoframes(
			connector, commit);
		if (ret)
			drm_err(drm, "failed to update infoframes: %d\n", ret);
	}

	hdmi->variant->phy_drive_on(hdmi, mode->clock);

	return;

err_resets:
	reset_control_assert(hdmi->resets);
err_tmds:
	clk_disable_unprepare(hdmi->clk_tmds);
err_pix:
	clk_disable_unprepare(hdmi->clk_pix);
}

static void histb_hdmi_atomic_disable(struct drm_bridge *bridge,
				      struct drm_atomic_commit *commit)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);
	u32 val;


	/* Stop driving before the PLL goes down, the vendor's order. */
	val = histb_hdmi_phy_read(hdmi, PHY_TOP_PD);
	histb_hdmi_phy_write(hdmi, PHY_TOP_PD, val | PHY_PD_ALL);

	val |= PHY_PD_TXPLL | PHY_PD_RXSENSE | PHY_PD_LDO;
	histb_hdmi_phy_write(hdmi, PHY_TOP_PD, val);

	histb_hdmi_write(hdmi, TX_PWD_RST_CTRL,
			 histb_hdmi_read(hdmi, TX_PWD_RST_CTRL) |
			 (TX_PWD_SRST | TX_SYS_SRST | TX_VID_SRST | TX_HDMI_SRST |
			  TX_HDCP1X_SRST | TX_PHY_SRST | TX_AUD_SRST | TX_ACR_SRST |
			  TX_AFIFO_SRST));

	reset_control_assert(hdmi->resets);
	clk_disable_unprepare(hdmi->clk_tmds);
	clk_disable_unprepare(hdmi->clk_pix);
}

static int histb_hdmi_attach(struct drm_bridge *bridge,
			     struct drm_encoder *encoder,
			     enum drm_bridge_attach_flags flags)
{
	/*
	 * The connector is the bridge connector the VDP master builds over the
	 * chain, and it reads EDID through this bridge's ddc adapter.  HPD is
	 * polled, not interrupt-driven: the datasheet names a line for this block
	 * (GIC SPI 147, "HDMITX_AON"), but the vendor hdmi_2_0 driver never
	 * requests it -- HAL_HDMI_IrqEnableSet() is an empty stub there and HPD
	 * runs as a 10 ms polling machine that only touches TX_AON_INTR_STATE/MASK.
	 * With no measurement showing the line carry anything, it stays out of the
	 * binding, because an interrupt number published in DT cannot be taken back.
	 */
	if (!(flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR))
		return -EINVAL;

	return 0;
}

static int histb_hdmi_ddc_arb(struct histb_hdmi *hdmi, bool acquire)
{
	u32 want = acquire ? DDC_REQ_ACK : 0;
	u32 val;

	val = histb_hdmi_read(hdmi, DDC_MST_ARB_REQ);
	histb_hdmi_write(hdmi, DDC_MST_ARB_REQ,
			 acquire ? (val | DDC_REQ) : (val & ~DDC_REQ));

	return readl_relaxed_poll_timeout(hdmi->regs + DDC_MST_ARB_ACK, val,
					  (val & DDC_REQ_ACK) == want, 1000,
					  DDC_TIMEOUT_ARB_MS * USEC_PER_MSEC);
}

static int histb_hdmi_ddc_idle(struct histb_hdmi *hdmi)
{
	u32 val;
	int ret;

	ret = readl_relaxed_poll_timeout(hdmi->regs + AON_DDC_MAN_CTRL, val,
					 val & DDC_SCL_HIGH, 1000,
					 USEC_PER_MSEC);
	if (ret)
		return ret;

	return readl_relaxed_poll_timeout(hdmi->regs + AON_DDC_MAN_CTRL, val,
					  val & DDC_SDA_HIGH, 1000,
					  DDC_TIMEOUT_SDA_MS * USEC_PER_MSEC);
}

/*
 * The master stretches the clock while the FIFO needs feeding, so writing the
 * whole payload in before it drains is safe, and the empty flag rather than the
 * count is what says a byte has arrived.
 */
static int histb_hdmi_ddc_write(struct histb_hdmi *hdmi, const u8 *buf, int len)
{
	int i;
	u32 val;

	for (i = 0; i < len; i++) {
		if (readl_relaxed_poll_timeout(hdmi->regs + PWD_DATA_CNT, val,
					       FIELD_GET(PWD_FIFO_DATA_CNT, val) <
					       DDC_FIFO_DEPTH, 1000,
					       DDC_TIMEOUT_DATA_MS * USEC_PER_MSEC))
			return -ETIMEDOUT;

		writel(FIELD_PREP(PWD_FIFO_DATA_IN, buf[i]),
		       hdmi->regs + PWD_FIFO_WDATA);
	}

	return i;
}

static int histb_hdmi_ddc_read(struct histb_hdmi *hdmi, u8 *buf, int len)
{
	int i;
	u32 val;

	for (i = 0; i < len; i++) {
		if (readl_relaxed_poll_timeout(hdmi->regs + PWD_MST_STATE, val,
					       !(val & PWD_FIFO_EMPTY), 1000,
					       DDC_TIMEOUT_DATA_MS * USEC_PER_MSEC))
			return -ETIMEDOUT;

		buf[i] = FIELD_GET(PWD_FIFO_DATA_OUT,
				   readl(hdmi->regs + PWD_FIFO_RDATA));
	}

	return i;
}

/* One addressed transfer: the pointer the master has to emit, then the payload. */
static int histb_hdmi_ddc_msg(struct histb_hdmi *hdmi, u8 addr, u8 seg, u8 offset,
			      u8 cmd, u8 *buf, int len, bool read)
{
	u32 val;
	int ret, done;

	writel(PWD_CMD_FIFO_CLR, hdmi->regs + PWD_MST_CMD);
	writel(FIELD_PREP(PWD_SLAVE_ADDR, addr) |
	       FIELD_PREP(PWD_SLAVE_OFFSET, offset) |
	       FIELD_PREP(PWD_SLAVE_SEG, seg), hdmi->regs + PWD_SLAVE_CFG);
	/* Read-modify-written because the low bits are the fill count itself. */
	val = readl(hdmi->regs + PWD_DATA_CNT);
	writel((val & ~PWD_DATA_OUT_CNT) | FIELD_PREP(PWD_DATA_OUT_CNT, len),
	       hdmi->regs + PWD_DATA_CNT);
	writel(cmd, hdmi->regs + PWD_MST_CMD);

	if (read)
		done = histb_hdmi_ddc_read(hdmi, buf, len);
	else
		done = histb_hdmi_ddc_write(hdmi, buf, len);
	if (done < 0) {
		ret = done;
		goto out;
	}

	writel(PWD_CLR_STATUS, hdmi->regs + PWD_MST_CLR);
	ret = readl_relaxed_poll_timeout(hdmi->regs + PWD_MST_STATE, val,
					 !(val & PWD_IN_PROG), 1000,
					 DDC_TIMEOUT_INPROG_MS * USEC_PER_MSEC);
	if (ret)
		goto out;

	if (readl(hdmi->regs + PWD_MST_STATE) & PWD_NO_ACK) {
		ret = -ENXIO;
		goto out;
	}

	return done;

out:
	writel(PWD_CMD_ABORT, hdmi->regs + PWD_MST_CMD);
	writel(PWD_CLR_STATUS, hdmi->regs + PWD_MST_CLR);
	return ret;
}

#define DDC_SEGMENT_ADDR	0x30

static int histb_hdmi_ddc_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs,
			       int num)
{
	struct histb_hdmi *hdmi = i2c_get_adapdata(adapter);
	bool pointer = false;
	u8 offset = 0, seg = 0;
	unsigned int i = 0;
	int total = num;
	int ret;

	if (num < 1 || num > 3)
		return -EOPNOTSUPP;

	/*
	 * A pointer write followed by a read is one transaction here, not two:
	 * the offset and the segment the master has to emit are fields of the
	 * same configuration register. That covers the shapes drm_do_probe_ddc_edid
	 * sends -- write 0x30 then the segment byte for EDID blocks past the
	 * second, write the start offset, then read.
	 */
	if (num == 3 && msgs[0].addr == DDC_SEGMENT_ADDR &&
	    !(msgs[0].flags & I2C_M_RD) && msgs[0].len == 1) {
		seg = msgs[0].buf[0];
		msgs++;
		num--;
	}
	if (num == 2 && !(msgs[0].flags & I2C_M_RD) && msgs[0].len == 1 &&
	    (msgs[1].flags & I2C_M_RD) && msgs[0].addr == msgs[1].addr) {
		offset = msgs[0].buf[0];
		pointer = true;
	}

	mutex_lock(&hdmi->ddc.lock);

	ret = histb_hdmi_ddc_arb(hdmi, true);
	if (ret)
		goto unlock;

	ret = histb_hdmi_ddc_idle(hdmi);
	if (ret)
		goto release;

	if (pointer) {
		ret = histb_hdmi_ddc_msg(hdmi, msgs[1].addr << 1, seg, offset,
					 seg ? PWD_CMD_READ_SEGMENT :
					       PWD_CMD_READ_MULTI,
					 msgs[1].buf, msgs[1].len, true);
		if (ret < 0)
			goto release;
		i = 2;
	}

	for (; i < num; i++) {
		if (msgs[i].flags & I2C_M_RD)
			ret = histb_hdmi_ddc_msg(hdmi, msgs[i].addr << 1, 0, 0,
						 PWD_CMD_READ_MULTI,
						 msgs[i].buf, msgs[i].len, true);
		else
			ret = histb_hdmi_ddc_msg(hdmi, msgs[i].addr << 1, 0, 0,
						 PWD_CMD_WRITE_MULTI,
						 msgs[i].buf, msgs[i].len, false);
		if (ret < 0)
			goto release;
	}

	/* i2c_transfer() 的调用者按全部消息计数，segment 写入也算一条。 */
	ret = total;

release:
	histb_hdmi_ddc_arb(hdmi, false);
unlock:
	mutex_unlock(&hdmi->ddc.lock);

	return ret;
}

static u32 histb_hdmi_ddc_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C;
}

static const struct i2c_algorithm histb_hdmi_ddc_algo = {
	.master_xfer = histb_hdmi_ddc_xfer,
	.functionality = histb_hdmi_ddc_func,
};

static int histb_hdmi_ddc_init(struct histb_hdmi *hdmi)
{
	struct i2c_adapter *adapter = &hdmi->ddc.adapter;

	mutex_init(&hdmi->ddc.lock);

	strscpy(adapter->name, "histb-hdmi-ddc", sizeof(adapter->name));
	adapter->owner = THIS_MODULE;
	adapter->algo = &histb_hdmi_ddc_algo;
	adapter->dev.parent = hdmi->dev;
	i2c_set_adapdata(adapter, hdmi);

	return devm_i2c_add_adapter(hdmi->dev, adapter);
}

static enum drm_connector_status histb_hdmi_detect(struct drm_bridge *bridge,
						   struct drm_connector *connector)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);
	bool connected = histb_hdmi_read(hdmi, AON_TX_AON_STATE) & HPD_STATE;

	if (histb_hdmi_read(hdmi, AON_HOTPLUG_ST_CFG) & HPD_POLARITY_CTL)
		connected = !connected;

	return connected ? connector_status_connected :
			   connector_status_disconnected;
}

static const struct drm_edid *histb_hdmi_edid_read(struct drm_bridge *bridge,
						   struct drm_connector *connector)
{
	return drm_edid_read_ddc(connector, bridge->ddc);
}

/*
 * Info packets.  The packer takes the three header bytes plus four seven-byte
 * subpackets, laid out as alternating four- and three-byte words; the config
 * word enables transmission.  Without an AVI infoframe a sink has to guess how
 * to read the lanes, and the answer it picks is not the one this pipeline
 * sends.
 */
static int histb_hdmi_write_packet(struct histb_hdmi *hdmi, u32 packet,
				   u32 cfg, const u8 *buffer, size_t length)
{
	unsigned int sub, byte, index;
	u32 word;

	if (length < 4 || length > 31)
		return -EINVAL;

	histb_hdmi_write(hdmi, cfg, PACKET_CFG_DISABLE);
	histb_hdmi_write(hdmi, packet,
			 buffer[0] | buffer[1] << 8 | buffer[2] << 16);
	for (sub = 0; sub < 8; sub++) {
		word = 0;
		for (byte = 0; byte < (sub & 1 ? 3 : 4); byte++) {
			index = 3 + (sub / 2) * 7 + (sub & 1 ? 4 : 0) + byte;
			if (index < length)
				word |= (u32)buffer[index] << (byte * 8);
		}
		histb_hdmi_write(hdmi, packet + 4 + sub * 4, word);
	}
	histb_hdmi_write(hdmi, cfg, PACKET_CFG_ENABLE);

	return 0;
}

static int histb_hdmi_write_avi_infoframe(struct drm_bridge *bridge,
					  const u8 *buffer, size_t length)
{
	return histb_hdmi_write_packet(bridge_to_histb_hdmi(bridge),
				       AVI_PACKET, AVI_PACKET_CFG,
				       buffer, length);
}

static int histb_hdmi_clear_avi_infoframe(struct drm_bridge *bridge)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);

	histb_hdmi_write(hdmi, AVI_PACKET_CFG, PACKET_CFG_DISABLE);

	return 0;
}

/*
 * The transmitter has no HDMI vendor-specific packet slot this driver knows
 * how to fill; the callbacks exist because the bridge connector requires them,
 * and accept silently.
 */
static int histb_hdmi_write_hdmi_infoframe(struct drm_bridge *bridge,
					   const u8 *buffer, size_t length)
{
	return 0;
}

static int histb_hdmi_clear_hdmi_infoframe(struct drm_bridge *bridge)
{
	return 0;
}

static const struct drm_bridge_funcs histb_hdmi_bridge_funcs = {
	.attach = histb_hdmi_attach,
	.detect = histb_hdmi_detect,
	.edid_read = histb_hdmi_edid_read,
	.mode_valid = histb_hdmi_mode_valid,
	.atomic_check = histb_hdmi_atomic_check,
	.atomic_enable = histb_hdmi_atomic_enable,
	.atomic_disable = histb_hdmi_atomic_disable,
	.atomic_create_state = drm_atomic_helper_bridge_create_state,
	.hdmi_write_avi_infoframe = histb_hdmi_write_avi_infoframe,
	.hdmi_clear_avi_infoframe = histb_hdmi_clear_avi_infoframe,
	.hdmi_write_hdmi_infoframe = histb_hdmi_write_hdmi_infoframe,
	.hdmi_clear_hdmi_infoframe = histb_hdmi_clear_hdmi_infoframe,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
};

/*
 * The transmitter configures itself from the atomic paths, so its bind is the
 * empty one the component framework asks for.
 */
static int histb_hdmi_component_bind(struct device *comp, struct device *master,
				     void *master_data)
{
	return 0;
}

static void histb_hdmi_component_unbind(struct device *comp, struct device *master,
					void *master_data)
{
}

static const struct component_ops histb_hdmi_component_ops = {
	.bind = histb_hdmi_component_bind,
	.unbind = histb_hdmi_component_unbind,
};

/*
 * Audio: the transmitter's I2S receiver, channel status memory and ACR
 * generator, in the first window at 0x1000..0x1050.
 *
 * This is not a codec in the usual sense: the samples arrive over I2S from the
 * AIAO block, and the transmitter only has to open its input, describe the
 * stream in the channel status bytes, and give the ACR generator the N value
 * for this sample rate and TMDS clock -- CTS itself comes from the hardware
 * (acr_cts_hw_sw_sel stays clear).  The order below, including the pulse on the
 * afifo/acr/aud reset requests, is the one the vendor path uses, because those
 * fields are clocked by what that pulse restarts.
 *
 * The four I2S input enables are all raised and the fifo map is left at its
 * reset value, because which AIAO transmit port reaches which input is not
 * documented anywhere local; the vendor build does the same.
 */
#define TX_AUDIO_CTRL		0x1000
#define  AUDIO_IN_EN		BIT(0)
#define  AUDIO_MUTE_EN		BIT(1)
#define  AUDIO_LAYOUT		BIT(2)	/* clear for two channels */
#define  AUDIO_I2S_EN		GENMASK(7, 4)	/* one enable per input */
#define AUD_I2S_CTRL		0x1004
#define  I2S_HBRA_ON		BIT(0)
#define  I2S_LENGTH		GENMASK(11, 8)
#define AUD_CHST_CFG0		0x100c
#define  CHST_PROF		BIT(0)
#define  CHST_COMPRESS		BIT(1)
#define  CHST_FS		GENMASK(27, 24)
#define  CHST_CLK_ACCURACY	GENMASK(31, 28)	/* level 2, the reset value */
#define AUD_CHST_CFG1		0x1010
#define  CHST_LENGTH		GENMASK(3, 0)
#define  CHST_ORG_FS		GENMASK(7, 4)
#define AUD_FIFO_CTRL		0x1018
#define  FIFO_TEST		GENMASK(4, 0)
#define  FIFO_HBR_MASK		GENMASK(11, 8)
#define AUD_ACR_CTRL		0x1040
#define  ACR_CTS_HW_SW_SEL	BIT(1)
#define ACR_N_VAL_SW		0x1048
#define  ACR_N_MAX		GENMASK(19, 0)

#define AUDIO_SRST		(TX_AUD_SRST | TX_ACR_SRST | TX_AFIFO_SRST)

struct histb_hdmi_audio_rate {
	unsigned int hz;
	u8 fs;		/* CHST_FS, channel status byte 3 */
	u8 org;		/* CHST_ORG_FS, byte 4's high nibble */
};

/*
 * Only these seven rates have a channel status code on this part; the table the
 * register set was generated from carries no entry for 64 or 128 kHz.
 */
static const struct histb_hdmi_audio_rate histb_hdmi_audio_rates[] = {
	{  32000, 0x04, 0x0c },
	{  44100, 0x00, 0x0f },
	{  48000, 0x02, 0x0d },
	{  88200, 0x08, 0x07 },
	{  96000, 0x0a, 0x05 },
	{ 176400, 0x0c, 0x03 },
	{ 192000, 0x0e, 0x01 },
};

/* AUD_I2S_CTRL's length field and CHST_LENGTH share this encoding. */
struct histb_hdmi_audio_bits {
	unsigned int width;
	u8 code;
};

static const struct histb_hdmi_audio_bits histb_hdmi_audio_bits[] = {
	{ 16, 0x02 },
	{ 17, 0x0c },
	{ 18, 0x04 },
	{ 19, 0x08 },
	{ 20, 0x0a },
	{ 21, 0x0d },
	{ 22, 0x05 },
	{ 23, 0x09 },
	{ 24, 0x0b },
};

/*
 * N, and the CTS the hardware is expected to reach, per rate and TMDS clock.  A
 * row with tmds 0 is the fallback the vendor uses for modes it did not list.
 */
struct histb_hdmi_audio_acr {
	unsigned int hz;
	unsigned int tmds_khz;
	u32 n;
};

static const struct histb_hdmi_audio_acr histb_hdmi_audio_acr[] = {
	{  32000,      0,   4096 },
	{  32000,  25174,   4576 },
	{  32000,  25200,   4096 },
	{  32000,  27000,   4096 },
	{  32000,  27027,   4096 },
	{  32000,  54000,   4096 },
	{  32000,  54054,   4096 },
	{  32000,  74175,  11648 },
	{  32000,  74250,   4096 },
	{  32000, 148351,  11648 },
	{  32000, 148500,   4096 },
	{  32000, 296703,   5824 },
	{  32000, 297000,   3072 },
	{  32000, 593406,   5824 },
	{  32000, 594000,   3072 },
	{  44100,      0,   6272 },
	{  44100,  25174,   7007 },
	{  44100,  25200,   6272 },
	{  44100,  27000,   6272 },
	{  44100,  27027,   6272 },
	{  44100,  54000,   6272 },
	{  44100,  54054,   6272 },
	{  44100,  74175,  17836 },
	{  44100,  74250,   6272 },
	{  44100, 148351,   8918 },
	{  44100, 148500,   6272 },
	{  44100, 296703,   4459 },
	{  44100, 297000,   4704 },
	{  44100, 593406,   8918 },
	{  44100, 594000,   9408 },
	{  48000,      0,   6144 },
	{  48000,  25174,   6864 },
	{  48000,  25200,   6144 },
	{  48000,  27000,   6144 },
	{  48000,  27027,   6144 },
	{  48000,  54000,   6144 },
	{  48000,  54054,   6144 },
	{  48000,  74175,  11648 },
	{  48000,  74250,   6144 },
	{  48000, 148351,   5824 },
	{  48000, 148500,   6144 },
	{  48000, 296703,   5824 },
	{  48000, 297000,   5120 },
	{  48000, 593406,   5824 },
	{  48000, 594000,   6144 },
	{  64000,      0,   8192 },
	{  64000,  25174,   9152 },
	{  64000,  25200,   8192 },
	{  64000,  27000,   8192 },
	{  64000,  27027,   8192 },
	{  64000,  54000,   8192 },
	{  64000,  54054,   8192 },
	{  64000,  74175,  23296 },
	{  64000,  74250,   8192 },
	{  64000, 148351,  23296 },
	{  64000, 148500,   8192 },
	{  64000, 296703,  11648 },
	{  64000, 297000,   8192 },
	{  64000, 593406,  11648 },
	{  64000, 594000,   8192 },
	{  88200,      0,  12544 },
	{  88200,  25174,  14014 },
	{  88200,  25200,  12544 },
	{  88200,  27000,  12544 },
	{  88200,  27027,  12544 },
	{  88200,  54000,  12544 },
	{  88200,  54054,  12544 },
	{  88200,  74175,  35672 },
	{  88200,  74250,  12544 },
	{  88200, 148351,  17836 },
	{  88200, 148500,  12544 },
	{  88200, 296703,   8918 },
	{  88200, 297000,   9408 },
	{  88200, 593406,  17836 },
	{  88200, 594000,  18816 },
	{  96000,      0,  12288 },
	{  96000,  25174,  13728 },
	{  96000,  25200,  12288 },
	{  96000,  27000,  12288 },
	{  96000,  27027,  12288 },
	{  96000,  54000,  12288 },
	{  96000,  54054,  12288 },
	{  96000,  74175,  23296 },
	{  96000,  74250,  12288 },
	{  96000, 148351,  11648 },
	{  96000, 148500,  12288 },
	{  96000, 296703,  11648 },
	{  96000, 297000,  10240 },
	{  96000, 593406,  11648 },
	{  96000, 594000,  12288 },
	{ 128000,      0,  16384 },
	{ 128000,  25174,  18304 },
	{ 128000,  25200,  16384 },
	{ 128000,  27000,  16384 },
	{ 128000,  27027,  16384 },
	{ 128000,  54000,  16384 },
	{ 128000,  54054,  16384 },
	{ 128000,  74175,  46592 },
	{ 128000,  74250,  16384 },
	{ 128000, 148351,  46592 },
	{ 128000, 148500,  16384 },
	{ 128000, 296703,  23296 },
	{ 128000, 297000,  16384 },
	{ 128000, 593406,  23296 },
	{ 128000, 594000,  16384 },
	{ 176400,      0,  25088 },
	{ 176400,  25174,  28028 },
	{ 176400,  25200,  25088 },
	{ 176400,  27000,  25088 },
	{ 176400,  27027,  25088 },
	{ 176400,  54000,  25088 },
	{ 176400,  54054,  25088 },
	{ 176400,  74175,  71344 },
	{ 176400,  74250,  25088 },
	{ 176400, 148351,  35672 },
	{ 176400, 148500,  25088 },
	{ 176400, 296703,  17836 },
	{ 176400, 297000,  18816 },
	{ 176400, 593406,  35672 },
	{ 176400, 594000,  27632 },
	{ 192000,      0,  24576 },
	{ 192000,  25174,  27456 },
	{ 192000,  25200,  24576 },
	{ 192000,  27000,  24576 },
	{ 192000,  27027,  24576 },
	{ 192000,  54000,  24576 },
	{ 192000,  54054,  24576 },
	{ 192000,  74175,  46592 },
	{ 192000,  74250,  24576 },
	{ 192000, 148351,  23296 },
	{ 192000, 148500,  24576 },
	{ 192000, 296703,  23296 },
	{ 192000, 297000,  20480 },
	{ 192000, 593406,  23296 },
	{ 192000, 594000,  24576 },
};

static void histb_hdmi_audio_rmw(struct histb_hdmi *hdmi, u32 offset, u32 mask,
				  u32 val)
{
	u32 cur = histb_hdmi_read(hdmi, offset);

	histb_hdmi_write(hdmi, offset, (cur & ~mask) | val);
}

static const struct histb_hdmi_audio_rate *histb_hdmi_audio_rate(unsigned int hz)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(histb_hdmi_audio_rates); i++)
		if (histb_hdmi_audio_rates[i].hz == hz)
			return &histb_hdmi_audio_rates[i];

	return NULL;
}

static u32 histb_hdmi_audio_n(unsigned int hz, unsigned int tmds_khz)
{
	const struct histb_hdmi_audio_acr *fallback = NULL;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(histb_hdmi_audio_acr); i++) {
		const struct histb_hdmi_audio_acr *row = &histb_hdmi_audio_acr[i];

		if (row->hz != hz)
			continue;
		if (row->tmds_khz == tmds_khz)
			return row->n;
		if (!row->tmds_khz)
			fallback = row;
	}

	return fallback ? fallback->n : 0;
}

static u8 histb_hdmi_audio_bits_code(unsigned int width)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(histb_hdmi_audio_bits); i++)
		if (histb_hdmi_audio_bits[i].width == width)
			return histb_hdmi_audio_bits[i].code;

	return 0xff;
}

static int histb_hdmi_audio_setup(struct histb_hdmi *hdmi, unsigned int rate,
				   unsigned int width, unsigned int channels)
{
	const struct histb_hdmi_audio_rate *ar = histb_hdmi_audio_rate(rate);
	u8 bits = histb_hdmi_audio_bits_code(width);
	u32 mask = AUDIO_IN_EN | AUDIO_MUTE_EN | AUDIO_I2S_EN;
	u32 n;

	if (!ar || bits == 0xff)
		return -EINVAL;

	n = histb_hdmi_audio_n(rate, hdmi->audio_tmds_khz);
	if (!n)
		return -EINVAL;

	/* Off and muted while the fields are rewritten. */
	histb_hdmi_audio_rmw(hdmi, TX_AUDIO_CTRL, mask, AUDIO_MUTE_EN);

	histb_hdmi_audio_rmw(hdmi, TX_PWD_RST_CTRL, AUDIO_SRST, AUDIO_SRST);
	usleep_range(500, 600);
	histb_hdmi_audio_rmw(hdmi, TX_PWD_RST_CTRL, AUDIO_SRST, 0);

	histb_hdmi_audio_rmw(hdmi, TX_AUDIO_CTRL, AUDIO_LAYOUT,
			     channels > 2 ? AUDIO_LAYOUT : 0);
	histb_hdmi_audio_rmw(hdmi, AUD_I2S_CTRL, I2S_LENGTH | I2S_HBRA_ON,
			     FIELD_PREP(I2S_LENGTH, bits));
	histb_hdmi_audio_rmw(hdmi, AUD_CHST_CFG0,
			     CHST_FS | CHST_PROF | CHST_COMPRESS,
			     FIELD_PREP(CHST_FS, ar->fs));
	histb_hdmi_audio_rmw(hdmi, AUD_CHST_CFG1, CHST_LENGTH | CHST_ORG_FS,
			     FIELD_PREP(CHST_LENGTH, bits) |
			     FIELD_PREP(CHST_ORG_FS, ar->org));
	histb_hdmi_audio_rmw(hdmi, AUD_FIFO_CTRL, FIFO_TEST | FIFO_HBR_MASK, 0);
	histb_hdmi_audio_rmw(hdmi, AUD_ACR_CTRL, ACR_CTS_HW_SW_SEL, 0);
	histb_hdmi_write(hdmi, ACR_N_VAL_SW, n & ACR_N_MAX);

	histb_hdmi_audio_rmw(hdmi, TX_AUDIO_CTRL, mask,
			     AUDIO_I2S_EN | AUDIO_IN_EN);

	return 0;
}

static void histb_hdmi_audio_disable(struct histb_hdmi *hdmi)
{
	histb_hdmi_audio_rmw(hdmi, TX_AUDIO_CTRL,
			     AUDIO_IN_EN | AUDIO_MUTE_EN | AUDIO_I2S_EN,
			     AUDIO_MUTE_EN);
}

static int histb_hdmi_audio_prepare(struct device *dev, void *data,
				     struct hdmi_codec_daifmt *fmt,
				     struct hdmi_codec_params *hparms)
{
	struct histb_hdmi *hdmi = data;

	if (fmt->fmt != HDMI_I2S)
		return -EINVAL;

	return histb_hdmi_audio_setup(hdmi, hparms->sample_rate,
				      hparms->sample_width,
				      hparms->channels);
}

static void histb_hdmi_audio_shutdown(struct device *dev, void *data)
{
	histb_hdmi_audio_disable(data);
}

static int histb_hdmi_audio_mute(struct device *dev, void *data,
				 bool enable, int direction)
{
	struct histb_hdmi *hdmi = data;

	if (direction != SNDRV_PCM_STREAM_PLAYBACK)
		return -EINVAL;

	histb_hdmi_audio_rmw(hdmi, TX_AUDIO_CTRL, AUDIO_MUTE_EN,
			     enable ? AUDIO_MUTE_EN : 0);

	return 0;
}

static const struct hdmi_codec_ops histb_hdmi_codec_ops = {
	.prepare = histb_hdmi_audio_prepare,
	.audio_shutdown = histb_hdmi_audio_shutdown,
	.mute_stream = histb_hdmi_audio_mute,
	/*
	 * No get_eld: the ELD lives on the connector, which the VDP master
	 * creates over the bridge chain, so handing it out is a display-side
	 * step and not this device's to invent.
	 */
};

static void histb_hdmi_audio_unregister(void *data)
{
	platform_device_unregister(data);
}

static int histb_hdmi_audio_register(struct histb_hdmi *hdmi)
{
	struct hdmi_codec_pdata pdata = {
		.ops = &histb_hdmi_codec_ops,
		.data = hdmi,
		.i2s = 1,
		.max_i2s_channels = 2,
	};
	struct platform_device *pdev;

	pdev = platform_device_register_data(hdmi->dev, HDMI_CODEC_DRV_NAME,
					     PLATFORM_DEVID_AUTO, &pdata,
					     sizeof(pdata));
	if (IS_ERR(pdev))
		return PTR_ERR(pdev);

	/*
	 * The codec device must not carry our of_node: ASoC reaches it from the
	 * sound card's phandle through soc_component_to_node(), which falls back
	 * to the parent device's node, and setting it here made our own
	 * of_match_table bind the codec (-EINVAL "invalid resource (null)"
	 * measured on the board).
	 */
	return devm_add_action_or_reset(hdmi->dev, histb_hdmi_audio_unregister,
					pdev);
}

static int histb_hdmi_parse_dt(struct histb_hdmi *hdmi)
{
	struct platform_device *pdev = to_platform_device(hdmi->dev);
	struct device *dev = &pdev->dev;

	hdmi->regs = devm_platform_ioremap_resource_byname(pdev, "tx");
	if (IS_ERR(hdmi->regs))
		return PTR_ERR(hdmi->regs);

	hdmi->phy = devm_platform_ioremap_resource_byname(pdev, "phy");
	if (IS_ERR(hdmi->phy))
		return PTR_ERR(hdmi->phy);

	/*
	 * The two divider fields and the resets live in the clock generator,
	 * whose node is a syscon. Those fields are not modelled as clocks: both
	 * are chosen from the mode, and the factor that turns the pixel clock
	 * into the PHY's reference clock -- the interpolation ratio -- has no
	 * field in the clock generator at all, so no rate relation could be
	 * reported honestly for them.
	 */
	hdmi->crg = syscon_regmap_lookup_by_phandle(dev->of_node,
						    "hisilicon,crg");
	if (IS_ERR(hdmi->crg))
		return dev_err_probe(dev, PTR_ERR(hdmi->crg), "no crg regmap\n");

	hdmi->clk_pix = devm_clk_get(dev, "pix");
	if (IS_ERR(hdmi->clk_pix))
		return dev_err_probe(dev, PTR_ERR(hdmi->clk_pix), "no pixel clock\n");

	hdmi->clk_tmds = devm_clk_get(dev, "tmds");
	if (IS_ERR(hdmi->clk_tmds))
		return dev_err_probe(dev, PTR_ERR(hdmi->clk_tmds), "no TMDS clock\n");

	hdmi->resets = devm_reset_control_array_get_exclusive(dev);
	if (IS_ERR(hdmi->resets))
		return dev_err_probe(dev, PTR_ERR(hdmi->resets), "no resets\n");

	return 0;
}

static int histb_hdmi_probe(struct platform_device *pdev)
{
	struct histb_hdmi *hdmi;
	int ret;

	/*
	 * The bridge must come from devm_drm_bridge_alloc(): that is where its
	 * refcount, list head and container pointer are set up, and
	 * drm_bridge_add() faults without them.  The macro takes the enclosing
	 * struct, so the bridge stays embedded here.
	 */
	hdmi = devm_drm_bridge_alloc(&pdev->dev, struct histb_hdmi, bridge,
				     &histb_hdmi_bridge_funcs);
	if (IS_ERR(hdmi))
		return PTR_ERR(hdmi);

	hdmi->dev = &pdev->dev;
	hdmi->variant = device_get_match_data(&pdev->dev);
	if (!hdmi->variant)
		return -ENODEV;
	platform_set_drvdata(pdev, hdmi);

	ret = histb_hdmi_parse_dt(hdmi);
	if (ret)
		return ret;

	ret = histb_hdmi_ddc_init(hdmi);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to add the DDC adapter\n");

	hdmi->bridge.of_node = pdev->dev.of_node;
	hdmi->bridge.type = DRM_MODE_CONNECTOR_HDMIA;
	hdmi->bridge.ops = DRM_BRIDGE_OP_DETECT | DRM_BRIDGE_OP_EDID |
			 DRM_BRIDGE_OP_HDMI;
	hdmi->bridge.supported_formats =
		BIT(DRM_OUTPUT_COLOR_FORMAT_RGB444) |
		BIT(DRM_OUTPUT_COLOR_FORMAT_YCBCR444);
	hdmi->bridge.ddc = &hdmi->ddc.adapter;
	hdmi->bridge.vendor = "HISI";
	hdmi->bridge.product = hdmi->variant->product;

	ret = devm_drm_bridge_add(&pdev->dev, &hdmi->bridge);
	if (ret)
		return ret;

	ret = histb_hdmi_audio_register(hdmi);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "no audio codec\n");

	/*
	 * Membership of the VDP's aggregate is what makes the master bind at
	 * all -- it reads its own of-graph endpoint to build the match list --
	 * and it is what takes the DRM device down before this driver's clocks,
	 * regmap and MMIO are released.  Bind has nothing left to do: the
	 * bridge is on the global list by then.
	 */
	return component_add(&pdev->dev, &histb_hdmi_component_ops);
}

static void histb_hdmi_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &histb_hdmi_component_ops);
}

static const struct of_device_id histb_hdmi_of_table[] = {
	{ .compatible = "hisilicon,hi3798mv200-hdmi",
	  .data = &histb_hdmi_mv200_variant, },
	{ .compatible = "hisilicon,hi3798mv300-hdmi",
	  .data = &histb_hdmi_mv300_variant, },
	{ }
};
MODULE_DEVICE_TABLE(of, histb_hdmi_of_table);

static struct platform_driver histb_hdmi_driver = {
	.probe = histb_hdmi_probe,
	.remove = histb_hdmi_remove,
	.driver = {
		.name = "histb-hdmi",
		.of_match_table = histb_hdmi_of_table,
	},
};
module_platform_driver(histb_hdmi_driver);

MODULE_AUTHOR("David Yang <mmyangfl@gmail.com>");
MODULE_DESCRIPTION("HiSilicon Hi3798MV200 HDMI transmitter driver");
MODULE_LICENSE("GPL");
