// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSTB TianLai internal audio DAC (ADAC).
 *
 * The analog output stage of the SoC: it takes the I2S stream that an AIAO
 * transmit port puts on its serial lane and drives a speaker/line output.
 * 这条串行线路不可选——ADAC 寄存器里没有选源位——所以 machine 驱动必须把这个 codec 与厂商端口映射指定的端口配对（aiao_v1_1/ao/drv_ao_op.c:30-88）：S40 的芯片（hi3716cv200、hi3798mv100）上是 AIAO 发送口 2，S28 的芯片（hi3798mv200、hi3798mv300）上是发送口 1。
 *
 * 驱动支持两代，每一代都只用它自己的厂商 HAL 用到的寄存器与操作序列，此外一概不碰：
 *
 *  - V500（hi3716cv200）：两个字，LOUT 与 CTRL。HAL 写 LOUT 的 popfree 位、两个掉电位、两个静音位与两个音量字段，以及 CTRL 的软静音位对和 sample_sel；clkdgesel、clksel2、adj_refbf、adj_ctcm、adj_dac、rst、data_bits 与 mute_rate 在该文件里没有写入者，而板上读到它们本来就有值（在我方任何代码运行之前 CTRL 读到 0x00E06755，data_bits=3），所以完全保持 bootloader 留下的值。输出级在每个流打开时上电。
 *
 *  - V700（hi3798mv300）：两个窗口共四个字，ADAC0/ADAC1 在 PERICTRL 0x110，ADAC2/ADAC3 在 PERICTRL 0x160（hi3798mv200/hi_reg_peri.h 的 bits_mv300 布局）。操作序列取自 hal_tianlai_adac_v700.c 在定义 HI_ADAC_INTERNAL_AMP_SUPPORT 时（即使用片内 line driver 及其 charge pump）编出的分支，因为 CM201-2 原厂 Android 内核跑的就是这一支：它的 ADAC_TIANLAI_V700_Init/DeInit 反汇编出来与该分支逐条写入一致。上电过程要睡眠约 600 ms，并经 pop 电路让输出缓升，所以输出级跟随 DAPM bias level（idle_bias_on 让它在两次播放之间保持上电），而不是每次打开流都上电。
 *
 * ADAC0 的音量码按 1 dB 一步：0x00 为 +6 dB，0x06 为 0 dB，0x7f 为 -121 dB（hal_tianlai_adac_v500.c:45-50；两代的字段布局相同）；控件把码值取反，使调大即变响。
 */
#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <sound/soc.h>
#include <sound/tlv.h>

/* ADAC0（V700）/ LOUT（V500）：数字音量与静音字段相同 */
#define HISTB_ADAC_LOUT		0x00
#define  ADAC_DACR_VOL		GENMASK(6, 0)
#define  ADAC_DACL_VOL		GENMASK(14, 8)
#define  ADAC_VOL_0DB		0x06
#define  ADAC_VOL_MIN		0x7f
#define  ADAC_DACR_PATH		BIT(22)
#define  ADAC_DACL_PATH		BIT(23)
#define  ADAC_POPFREER		BIT(24)
#define  ADAC_POPFREEL		BIT(25)
#define  ADAC_PD_VREF		BIT(27)
#define  ADAC_MUTE_DACR		BIT(28)
#define  ADAC_MUTE_DACL		BIT(29)
#define  ADAC_PD_DACR		BIT(30)
#define  ADAC_PD_DACL		BIT(31)
#define  ADAC_POWEROFF		(ADAC_PD_DACL | ADAC_PD_DACR | ADAC_MUTE_DACL | \
				 ADAC_MUTE_DACR | ADAC_PD_VREF | ADAC_POPFREEL | \
				 ADAC_POPFREER)

/* ADAC1 (V700) / CTRL (V500) */
#define HISTB_ADAC_CTRL		0x04
#define  ADAC1_ADJ_REFBF	GENMASK(2, 0)
#define  ADAC1_ADJ_CTCM		GENMASK(4, 3)
#define  ADAC1_ADJ_DAC		GENMASK(7, 5)
#define  ADAC_SAMPLE_SEL	GENMASK(21, 19)
#define  ADAC1_DATA_BITS	GENMASK(23, 22)
#define  ADAC_SUNMUTER		BIT(28)
#define  ADAC_SUNMUTEL		BIT(29)
#define  ADAC_SMUTER		BIT(30)
#define  ADAC_SMUTEL		BIT(31)

/* V700 模拟窗口：ADAC2 在 +0x0，ADAC3 在 +0x4 */
#define HISTB_ADAC2		0x00
#define  ADAC2_PD_VREF		BIT(1)
#define  ADAC2_PDB_CTCM_IBIAS	BIT(2)
#define  ADAC2_PD_CTCM		BIT(3)
#define  ADAC2_PD_IBIAS		BIT(4)
#define  ADAC2_MUTE_LINEOUT	BIT(5)
#define  ADAC2_PD_DACR		BIT(6)
#define  ADAC2_PD_DACL		BIT(7)
#define  ADAC2_POP_PU_PULLB	BIT(8)
#define  ADAC2_POP_PULLOUT_EN	BIT(9)
#define  ADAC2_ADJ_DAC_LOW_BIAS	BIT(10)
#define  ADAC2_GAIN		GENMASK(15, 13)
#define HISTB_ADAC3		0x04
#define  ADAC3_CHOP_BPS_LINEOUT	BIT(0)
#define  ADAC3_RST_POP		BIT(5)
#define  ADAC3_POP_DIS		BIT(6)
#define  ADAC3_PD_CLKRAMP	BIT(7)
#define  ADAC3_CP_DT_CTRL	GENMASK(11, 10)
#define  ADAC3_CP_CLK_SEL	GENMASK(13, 12)
#define  ADAC3_CP_EN_SOFT	BIT(14)
#define  ADAC3_CP_PD		BIT(15)
#define  ADAC3_CP_BUFFER_PD	BIT(16)
#define  ADAC3_PD_LINEOUT	BIT(17)
#define  ADAC3_MSEL		BIT(18)
#define  ADAC3_RST		BIT(21)
#define  ADAC3_CLKSEL		BIT(23)
#define  ADAC3_CTRL7		GENMASK(31, 24)

struct histb_adac {
	struct regmap *regmap;	/* LOUT/CTRL 或 ADAC0/ADAC1 */
	struct regmap *ana;	/* 仅 V700：ADAC2/ADAC3 */
	struct clk *clk;
	struct reset_control *rst;
};

struct histb_adac_variant {
	const struct snd_soc_component_driver *component;
	struct snd_soc_dai_driver *dai;
	bool v700;
};

static const DECLARE_TLV_DB_SCALE(histb_adac_vol_tlv, -12100, 100, 0);

static const struct snd_kcontrol_new histb_adac_controls[] = {
	SOC_DOUBLE_TLV("DAC Playback Volume", HISTB_ADAC_LOUT, 8, 0, 0x7f, 1,
		       histb_adac_vol_tlv),
};

static const struct snd_soc_dapm_widget histb_adac_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("DAC Out"),
};

static const struct snd_soc_dapm_route histb_adac_dapm_routes[] = {
	{ "DAC Out", NULL, "DAC HiFi" },
};

/*
 * The rate bucket the vendor HAL selects (Digfi_DacSetSampleRate). The field
 * covers one of five bands and the band is shared by the neighbouring rates, so
 * 48 kHz and 44.1 kHz are the same code.
 */
static int histb_adac_sample_sel(unsigned int rate)
{
	if (rate <= 12000)
		return 0;
	if (rate <= 24000)
		return 1;
	if (rate <= 48000)
		return 2;
	if (rate <= 96000)
		return 3;

	return 4;
}

static int histb_adac_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct histb_adac *adac = snd_soc_dai_get_drvdata(dai);

	return regmap_update_bits(adac->regmap, HISTB_ADAC_CTRL, ADAC_SAMPLE_SEL,
				  FIELD_PREP(ADAC_SAMPLE_SEL,
					     histb_adac_sample_sel(params_rate(params))));
}

/*
 * V500 上电，按厂商的顺序（Digfi_DacPoweup 中不含 HI_SND_MUTECTL_SUPPORT 分支的那条路径，它没有需要猜测的时序钩子）：先软静音，使模拟级上电期间没有信号到达引脚；基准与驱动上电前后用 popfree（抑制咔哒声的斜坡）包住；最后解除两种静音。
 *
 * Soft-unmute is an RMW over just the four mute bits: sample_sel lives in the
 * same register, so writing the whole register here would undo whatever rate
 * .hw_params picked.
 */
static void histb_adac_soft_unmute(struct histb_adac *adac)
{
	regmap_update_bits(adac->regmap, HISTB_ADAC_CTRL,
			   ADAC_SMUTEL | ADAC_SMUTER | ADAC_SUNMUTEL | ADAC_SUNMUTER,
			   ADAC_SUNMUTEL | ADAC_SUNMUTER);
}

static int histb_adac_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct histb_adac *adac = snd_soc_dai_get_drvdata(dai);

	regmap_set_bits(adac->regmap, HISTB_ADAC_CTRL, ADAC_SMUTEL | ADAC_SMUTER);
	regmap_set_bits(adac->regmap, HISTB_ADAC_LOUT, ADAC_POPFREEL | ADAC_POPFREER);
	regmap_clear_bits(adac->regmap, HISTB_ADAC_LOUT, ADAC_PD_VREF);
	regmap_clear_bits(adac->regmap, HISTB_ADAC_LOUT, ADAC_PD_DACL | ADAC_PD_DACR);
	regmap_clear_bits(adac->regmap, HISTB_ADAC_LOUT, ADAC_POPFREEL | ADAC_POPFREER);
	regmap_clear_bits(adac->regmap, HISTB_ADAC_LOUT, ADAC_MUTE_DACL | ADAC_MUTE_DACR);
	histb_adac_soft_unmute(adac);

	return 0;
}

static void histb_adac_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct histb_adac *adac = snd_soc_dai_get_drvdata(dai);

	regmap_set_bits(adac->regmap, HISTB_ADAC_CTRL, ADAC_SMUTEL | ADAC_SMUTER);
	/*
	 * One write for the whole analog power-down: every bit of the off state
	 * is asserted together, which is what the block shows cold (the board
	 * read ADAC0 = 0xF8400606 with pd_*, mute_* and pd_vref all set).
	 */
	regmap_set_bits(adac->regmap, HISTB_ADAC_LOUT, ADAC_POWEROFF);
}

static int histb_adac_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct histb_adac *adac = snd_soc_dai_get_drvdata(dai);

	if (mute)
		regmap_set_bits(adac->regmap, HISTB_ADAC_CTRL,
				ADAC_SMUTEL | ADAC_SMUTER);
	else
		histb_adac_soft_unmute(adac);

	return 0;
}

static const struct snd_soc_dai_ops histb_adac_ops = {
	.hw_params = histb_adac_hw_params,
	.startup = histb_adac_startup,
	.shutdown = histb_adac_shutdown,
	.mute_stream = histb_adac_mute_stream,
};

static const struct snd_soc_component_driver histb_adac_component = {
	.controls = histb_adac_controls,
	.num_controls = ARRAY_SIZE(histb_adac_controls),
	.dapm_widgets = histb_adac_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(histb_adac_dapm_widgets),
	.dapm_routes = histb_adac_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(histb_adac_dapm_routes),
	.idle_bias_on = true,
};

static struct snd_soc_dai_driver histb_adac_dai = {
	.name = "histb-adac",
	.playback = {
		.stream_name = "DAC HiFi",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &histb_adac_ops,
};

/*
 * V700 上电：定义 HI_ADAC_INTERNAL_AMP_SUPPORT 时的 ADAC_TIANLAI_V700_Init，即 Digfi_ADACEnable、Digfi_DacInitDigital、Digfi_DacSetAdj、Digfi_DacSetPd、Digfi_Dac3、Digfi_DacPoweup、Digfi_DacSetDataBits，再加 0 dB 音量；厂商的每条语句对应一次寄存器更新，顺序与厂商一致。CM201-2 原厂内核执行的正是这一序列（其 3.18 镜像中 c0867174 处的 ADAC_TIANLAI_V700_Init），只有采样率除外，采样率在这里归 .hw_params 管。上电后输出保持数字静音，流 prepare 之后由 .mute_stream 解除。
 */
static int histb_adac_v700_power_on(struct histb_adac *adac)
{
	struct regmap *dig = adac->regmap, *ana = adac->ana;
	int ret;

	ret = clk_prepare_enable(adac->clk);
	if (ret)
		return ret;
	udelay(10);

	/* Digfi_ADACEnable：模拟核复位脉冲、时钟选择、CRG 复位 */
	regmap_set_bits(ana, HISTB_ADAC3, ADAC3_RST);
	regmap_clear_bits(ana, HISTB_ADAC3, ADAC3_RST);
	regmap_clear_bits(ana, HISTB_ADAC3, ADAC3_CLKSEL);
	reset_control_assert(adac->rst);
	reset_control_deassert(adac->rst);
	msleep(1);

	/* Digfi_DacInitDigital：以最低音量打开数字部分 */
	regmap_clear_bits(dig, HISTB_ADAC_LOUT, ADAC_PD_DACL | ADAC_PD_DACR);
	regmap_clear_bits(dig, HISTB_ADAC_LOUT, ADAC_MUTE_DACL | ADAC_MUTE_DACR);
	regmap_update_bits(dig, HISTB_ADAC_LOUT, ADAC_DACL_VOL | ADAC_DACR_VOL,
			   FIELD_PREP(ADAC_DACL_VOL, ADAC_VOL_MIN) |
			   FIELD_PREP(ADAC_DACR_VOL, ADAC_VOL_MIN));
	regmap_update_bits(dig, HISTB_ADAC_LOUT, ADAC_DACL_PATH | ADAC_DACR_PATH,
			   ADAC_DACR_PATH);

	/* Digfi_DacSetAdj */
	regmap_update_bits(dig, HISTB_ADAC_CTRL,
			   ADAC1_ADJ_REFBF | ADAC1_ADJ_CTCM | ADAC1_ADJ_DAC,
			   FIELD_PREP(ADAC1_ADJ_REFBF, 2) |
			   FIELD_PREP(ADAC1_ADJ_CTCM, 1) |
			   FIELD_PREP(ADAC1_ADJ_DAC, 4));
	regmap_set_bits(ana, HISTB_ADAC2, ADAC2_ADJ_DAC_LOW_BIAS);

	/* Digfi_DacSetPd */
	regmap_update_bits(ana, HISTB_ADAC2,
			   ADAC2_PD_VREF | ADAC2_PDB_CTCM_IBIAS | ADAC2_PD_IBIAS |
			   ADAC2_PD_DACR | ADAC2_PD_DACL | ADAC2_PD_CTCM,
			   ADAC2_PDB_CTCM_IBIAS);

	/* Digfi_Dac3 */
	regmap_update_bits(ana, HISTB_ADAC3,
			   ADAC3_CTRL7 | ADAC3_CP_BUFFER_PD | ADAC3_CP_CLK_SEL |
			   ADAC3_CP_DT_CTRL | ADAC3_PD_CLKRAMP |
			   ADAC3_CHOP_BPS_LINEOUT,
			   ADAC3_CP_BUFFER_PD | FIELD_PREP(ADAC3_CP_DT_CTRL, 3) |
			   ADAC3_CHOP_BPS_LINEOUT);

	/* Digfi_DacPoweup 的片内放大器（"2VRM"）分支 */
	regmap_set_bits(ana, HISTB_ADAC2, ADAC2_POP_PU_PULLB);
	regmap_clear_bits(ana, HISTB_ADAC2, ADAC2_PD_DACR | ADAC2_PD_DACL);
	regmap_set_bits(ana, HISTB_ADAC3, ADAC3_CHOP_BPS_LINEOUT);
	regmap_set_bits(ana, HISTB_ADAC3, ADAC3_MSEL);
	regmap_clear_bits(ana, HISTB_ADAC3, ADAC3_POP_DIS);
	regmap_clear_bits(dig, HISTB_ADAC_LOUT, ADAC_PD_DACL | ADAC_PD_DACR);
	regmap_set_bits(ana, HISTB_ADAC2, ADAC2_POP_PULLOUT_EN);
	regmap_clear_bits(ana, HISTB_ADAC2, ADAC2_PD_IBIAS);
	regmap_clear_bits(ana, HISTB_ADAC2, ADAC2_PD_CTCM | ADAC2_PDB_CTCM_IBIAS);
	regmap_clear_bits(ana, HISTB_ADAC3, ADAC3_RST_POP);
	regmap_clear_bits(ana, HISTB_ADAC3, ADAC3_CP_BUFFER_PD);
	/* charge pump：先软启动，再进入正常运行 */
	regmap_update_bits(ana, HISTB_ADAC3, ADAC3_CP_EN_SOFT | ADAC3_CP_PD,
			   ADAC3_CP_EN_SOFT);
	msleep(200);
	regmap_clear_bits(ana, HISTB_ADAC3, ADAC3_CP_EN_SOFT);
	msleep(200);
	regmap_clear_bits(ana, HISTB_ADAC2, ADAC2_PD_DACR | ADAC2_PD_DACL);
	regmap_clear_bits(ana, HISTB_ADAC2, ADAC2_PD_VREF);
	regmap_clear_bits(ana, HISTB_ADAC3, ADAC3_PD_LINEOUT);
	msleep(200);
	regmap_update_bits(ana, HISTB_ADAC2,
			   ADAC2_ADJ_DAC_LOW_BIAS | ADAC2_POP_PULLOUT_EN,
			   ADAC2_ADJ_DAC_LOW_BIAS);
	regmap_set_bits(ana, HISTB_ADAC2, ADAC2_PDB_CTCM_IBIAS);
	regmap_clear_bits(ana, HISTB_ADAC2, ADAC2_MUTE_LINEOUT);
	regmap_clear_bits(dig, HISTB_ADAC_LOUT, ADAC_MUTE_DACL | ADAC_MUTE_DACR);
	regmap_set_bits(ana, HISTB_ADAC3, ADAC3_POP_DIS);
	/* HI_SND_SUPPORT_INTERNAL_OP_MAXGAIN：该 HAL 里定义了这个宏 */
	regmap_update_bits(ana, HISTB_ADAC2, ADAC2_GAIN,
			   FIELD_PREP(ADAC2_GAIN, 2));

	/* Digfi_DacSetDataBits：16 bit 接口 */
	regmap_clear_bits(dig, HISTB_ADAC_CTRL, ADAC1_DATA_BITS);

	regmap_update_bits(dig, HISTB_ADAC_LOUT, ADAC_DACL_VOL | ADAC_DACR_VOL,
			   FIELD_PREP(ADAC_DACL_VOL, ADAC_VOL_0DB) |
			   FIELD_PREP(ADAC_DACR_VOL, ADAC_VOL_0DB));
	regmap_set_bits(dig, HISTB_ADAC_LOUT, ADAC_MUTE_DACL | ADAC_MUTE_DACR);

	return 0;
}

/*
 * V700 下电：定义 HI_ADAC_INTERNAL_AMP_SUPPORT 时的 ADAC_TIANLAI_V700_DeInit（Digfi_DacPowedown、Digfi_DacDeInitDigital、Digfi_ADACDisable），与原厂内核在 c0867500 处执行的一致。
 */
static void histb_adac_v700_power_off(struct histb_adac *adac)
{
	struct regmap *dig = adac->regmap, *ana = adac->ana;

	regmap_update_bits(ana, HISTB_ADAC2, ADAC2_GAIN, 0);
	regmap_set_bits(dig, HISTB_ADAC_LOUT, ADAC_MUTE_DACL | ADAC_MUTE_DACR);
	regmap_clear_bits(ana, HISTB_ADAC3, ADAC3_POP_DIS);
	regmap_set_bits(ana, HISTB_ADAC2, ADAC2_POP_PULLOUT_EN);
	msleep(100);
	regmap_set_bits(ana, HISTB_ADAC3, ADAC3_PD_LINEOUT);
	msleep(200);
	regmap_set_bits(ana, HISTB_ADAC2, ADAC2_PD_VREF);
	msleep(200);
	regmap_set_bits(ana, HISTB_ADAC2, ADAC2_PD_DACL | ADAC2_PD_DACR);
	regmap_set_bits(ana, HISTB_ADAC3,
			ADAC3_CP_PD | ADAC3_CP_BUFFER_PD | ADAC3_CP_EN_SOFT);
	regmap_set_bits(ana, HISTB_ADAC2, ADAC2_PD_CTCM | ADAC2_PD_IBIAS);
	regmap_set_bits(ana, HISTB_ADAC2, ADAC2_PD_DACL | ADAC2_PD_DACR);
	regmap_set_bits(ana, HISTB_ADAC2, ADAC2_MUTE_LINEOUT);
	regmap_update_bits(ana, HISTB_ADAC3, ADAC3_POP_DIS | ADAC3_RST_POP,
			   ADAC3_RST_POP);
	regmap_clear_bits(ana, HISTB_ADAC2, ADAC2_POP_PU_PULLB);

	/* Digfi_DacDeInitDigital */
	regmap_set_bits(dig, HISTB_ADAC_LOUT, ADAC_MUTE_DACL | ADAC_MUTE_DACR);
	regmap_set_bits(dig, HISTB_ADAC_LOUT, ADAC_PD_DACL | ADAC_PD_DACR);

	/* Digfi_ADACDisable */
	regmap_set_bits(ana, HISTB_ADAC3, ADAC3_RST);
	reset_control_assert(adac->rst);
	clk_disable_unprepare(adac->clk);
}

static int histb_adac_v700_set_bias_level(struct snd_soc_component *component,
					  enum snd_soc_bias_level level)
{
	struct histb_adac *adac = snd_soc_component_get_drvdata(component);
	enum snd_soc_bias_level old =
		snd_soc_dapm_get_bias_level(snd_soc_component_to_dapm(component));

	if (level == SND_SOC_BIAS_STANDBY && old == SND_SOC_BIAS_OFF)
		return histb_adac_v700_power_on(adac);
	if (level == SND_SOC_BIAS_OFF && old != SND_SOC_BIAS_OFF)
		histb_adac_v700_power_off(adac);

	return 0;
}

static int histb_adac_v700_mute_stream(struct snd_soc_dai *dai, int mute,
				       int stream)
{
	struct histb_adac *adac = snd_soc_dai_get_drvdata(dai);

	return regmap_update_bits(adac->regmap, HISTB_ADAC_LOUT,
				  ADAC_MUTE_DACL | ADAC_MUTE_DACR,
				  mute ? ADAC_MUTE_DACL | ADAC_MUTE_DACR : 0);
}

/*
 * 音量控件在 ADAC0，即第一个窗口。必须显式指定：dev_get_regmap(dev, NULL) 取的是最后注册的 regmap（这里是模拟窗口 "ana"），不指定的话音量控件会写到 ADAC2 的模拟电源位与增益上。
 */
static int histb_adac_v700_probe(struct snd_soc_component *component)
{
	struct histb_adac *adac = snd_soc_component_get_drvdata(component);

	snd_soc_component_init_regmap(component, adac->regmap);
	return 0;
}

static const struct snd_soc_dai_ops histb_adac_v700_ops = {
	.hw_params = histb_adac_hw_params,
	.mute_stream = histb_adac_v700_mute_stream,
};

static const struct snd_soc_component_driver histb_adac_v700_component = {
	.probe = histb_adac_v700_probe,
	.set_bias_level = histb_adac_v700_set_bias_level,
	.controls = histb_adac_controls,
	.num_controls = ARRAY_SIZE(histb_adac_controls),
	.dapm_widgets = histb_adac_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(histb_adac_dapm_widgets),
	.dapm_routes = histb_adac_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(histb_adac_dapm_routes),
	.idle_bias_on = true,
};

/* Digfi_DacSetSampleRate 列出的采样率；64 kHz 没有自己的档位。 */
static struct snd_soc_dai_driver histb_adac_v700_dai = {
	.name = "histb-adac",
	.playback = {
		.stream_name = "DAC HiFi",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
			 SNDRV_PCM_RATE_12000 | SNDRV_PCM_RATE_16000 |
			 SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_24000 |
			 SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
			 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
			 SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
			 SNDRV_PCM_RATE_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &histb_adac_v700_ops,
};

/* 每个窗口两个字；其后的寄存器都不属于 DAC。 */
static const struct regmap_config histb_adac_regmap = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = HISTB_ADAC_CTRL,
};

static const struct regmap_config histb_adac_ana_regmap = {
	.name = "ana",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = HISTB_ADAC3,
};

static int histb_adac_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct histb_adac_variant *variant = of_device_get_match_data(dev);
	struct histb_adac *adac;
	void __iomem *base;
	int ret;

	adac = devm_kzalloc(dev, sizeof(*adac), GFP_KERNEL);
	if (!adac)
		return -ENOMEM;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	adac->regmap = devm_regmap_init_mmio(dev, base, &histb_adac_regmap);
	if (IS_ERR(adac->regmap))
		return dev_err_probe(dev, PTR_ERR(adac->regmap), "no regmap\n");

	if (variant->v700) {
		base = devm_platform_ioremap_resource(pdev, 1);
		if (IS_ERR(base))
			return PTR_ERR(base);

		adac->ana = devm_regmap_init_mmio(dev, base,
						  &histb_adac_ana_regmap);
		if (IS_ERR(adac->ana))
			return dev_err_probe(dev, PTR_ERR(adac->ana),
					     "no analog regmap\n");

		/* 此后门控时钟与复位由 bias level 掌管。 */
		adac->clk = devm_clk_get(dev, NULL);
		if (IS_ERR(adac->clk))
			return dev_err_probe(dev, PTR_ERR(adac->clk),
					     "no adac clock\n");

		adac->rst = devm_reset_control_get_exclusive(dev, NULL);
		if (IS_ERR(adac->rst))
			return dev_err_probe(dev, PTR_ERR(adac->rst),
					     "no adac reset\n");
	} else {
		adac->clk = devm_clk_get_enabled(dev, NULL);
		if (IS_ERR(adac->clk))
			return dev_err_probe(dev, PTR_ERR(adac->clk),
					     "no adac clock\n");
	}

	platform_set_drvdata(pdev, adac);

	ret = devm_snd_soc_register_component(dev, variant->component,
					      variant->dai, 1);
	if (ret)
		return dev_err_probe(dev, ret, "could not register the codec\n");

	return 0;
}

static const struct histb_adac_variant hi3716cv200_adac = {
	.component = &histb_adac_component,
	.dai = &histb_adac_dai,
};

static const struct histb_adac_variant hi3798mv300_adac = {
	.component = &histb_adac_v700_component,
	.dai = &histb_adac_v700_dai,
	.v700 = true,
};

static const struct of_device_id histb_adac_of_match[] = {
	{ .compatible = "hisilicon,hi3716cv200-adac", .data = &hi3716cv200_adac },
	{ .compatible = "hisilicon,hi3798mv300-adac", .data = &hi3798mv300_adac },
	{ }
};
MODULE_DEVICE_TABLE(of, histb_adac_of_match);

static struct platform_driver histb_adac_driver = {
	.probe = histb_adac_probe,
	.driver = {
		.name = "histb-adac",
		.of_match_table = histb_adac_of_match,
	},
};
module_platform_driver(histb_adac_driver);

MODULE_DESCRIPTION("HiSTB TianLai internal audio DAC");
MODULE_LICENSE("GPL");
