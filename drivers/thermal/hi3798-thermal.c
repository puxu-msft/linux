// SPDX-License-Identifier: GPL-2.0
/*
 * Temperature sensor for HiSilicon Hi3798MV200/MV300 SoCs.
 *
 * The sensor lives in the PMC block and has nothing in common with the
 * tsensor that drivers/thermal/hisi_thermal.c drives on Hi6220 and Hi3660 --
 * different registers, different sampling scheme, different conversion. It is
 * not described by any binding either; the vendor device tree carries a bare
 *
 *	hisi-sensor@0 { compatible = "arm,hisi-thermal"; };
 *
 * with no reg at all, because the vendor driver hardcodes the address.
 *
 * Reading it means enabling the sensor, waiting for it to settle, collecting
 * eight 10-bit samples packed two per word, and applying a linear fit. All of
 * that, including the magic enable value and the fit coefficients, is taken
 * from drivers/hisilicon/thermal/hi3798cvx-thermal.c in the HiSilicon
 * HiSTBLinuxV100R005C00 BSP.
 *
 * Copyright (c) 2026
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>

/*
 * 寄存器窗口从 PMC+0x28 开始（DT reg = <0x8a23028 0x18>），不覆盖 PMC+0x18..0x20 的 PMC6–PMC8 调压寄存器，那几个字由 histb-regulator 各自映射。
 */
#define HI3798_TSENSOR_CTRL		0x00	/* PMC+0x28 */
#define HI3798_TSENSOR_DATA		0x08	/* PMC+0x30..+0x3c */

/* Vendor magic. Bit 0 looks like the enable; the rest is undocumented. */
#define HI3798_TSENSOR_ENABLE		0x6005
#define HI3798_TSENSOR_DISABLE		0x0

/* Four words, two 10-bit samples each. */
#define HI3798_TSENSOR_WORDS		4
#define HI3798_TSENSOR_SAMPLES		(HI3798_TSENSOR_WORDS * 2)
#define HI3798_TSENSOR_SAMPLE_MASK	0x3ff

/*
 * 每次读数前使能的时长，沿用厂商驱动的 mdelay(16)；这里在 thermal 核心的工作队列里运行，可以睡眠。使能期间约每 12 ms 换进一个新采样（见 HI3798_TSENSOR_PRIME_MS），所以每次读数大约只换进一个，八个采样是最近八次读数各自换进的值：常规 1 s 轮询下跨约 8 s，passive 100 ms 轮询下跨约 0.8 s。上报温度因此是这段时间的平均，升温时最多滞后约一个窗口。
 */
#define HI3798_TSENSOR_SETTLE_US	16000

/*
 * 八个采样是一个移位缓冲区，使能期间大约每 12 ms 换进一个新采样；关掉再开不清空它。冷启动时缓冲区里是上电残留（CM201-2 2026-11-15 实测码值 0x000、0x00a、0x168），16 ms 的稳定时间只换进一两个新采样，离散检查要连续失败好几次才会过。probe 先使能这么久把八个采样全部换新：实测 50 ms 时仍有残留，200 ms 时八个全部一致。热重启后缓冲区里本来就是有效值，所以之前只在热重启上测试时没暴露。
 */
#define HI3798_TSENSOR_PRIME_MS		200

/* Linear fit: millicelsius = ((avg - 125) * 165 / 806 - 40) * 1000 */
#define HI3798_TSENSOR_OFFSET		125
#define HI3798_TSENSOR_MUL		165
#define HI3798_TSENSOR_DIV		806
#define HI3798_TSENSOR_BIAS		40

/*
 * 误读必须拒收而不是上报：zone 的 critical trip 会让板子关机，一次虚假读数不应触发它。即使是全 1 采样（码值 1023），拟合结果也不超过 143 degC，对换算结果做范围检查永远抓不到真正要紧的那种读数，所以改为检查采样本身。码值钉在 10 位范围的任一端，是传感器失效或没有时钟（或总线读回全 1）的表现；八个采样（最近八次读数换进的值）若相差超过 64 个码值（每码约 0.2 degC，约合 13 degC），视为无效：冷启动残留或传感器失效时如此，一个窗口内升温超过约 13 degC 时也会如此，后者由 thermal 核心按 250 ms 重试，每次至少换进一个新采样，约 2 s 内收敛。低于拟合下端 -40 degC 的结果同样拒收。
 */
#define HI3798_TSENSOR_SPREAD_MAX	64
#define HI3798_TEMP_MIN_MC		(-40000)

struct hi3798_thermal {
	void __iomem *regs;
	struct mutex lock;	/* serialises the enable/sample/disable cycle */
};

static int hi3798_thermal_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct hi3798_thermal *data = thermal_zone_device_priv(tz);
	unsigned int sum = 0, lo = HI3798_TSENSOR_SAMPLE_MASK, hi = 0;
	int i, j, millicelsius;

	mutex_lock(&data->lock);

	writel(HI3798_TSENSOR_ENABLE, data->regs + HI3798_TSENSOR_CTRL);
	usleep_range(HI3798_TSENSOR_SETTLE_US, HI3798_TSENSOR_SETTLE_US + 2000);

	for (i = 0; i < HI3798_TSENSOR_WORDS; i++) {
		u32 val = readl(data->regs + HI3798_TSENSOR_DATA + i * 4);

		for (j = 0; j < 2; j++) {
			unsigned int s = (val >> (16 * j)) &
					 HI3798_TSENSOR_SAMPLE_MASK;

			sum += s;
			lo = min(lo, s);
			hi = max(hi, s);
		}
	}

	writel(HI3798_TSENSOR_DISABLE, data->regs + HI3798_TSENSOR_CTRL);

	mutex_unlock(&data->lock);

	if (lo == 0 || hi == HI3798_TSENSOR_SAMPLE_MASK ||
	    hi - lo > HI3798_TSENSOR_SPREAD_MAX)
		return -EAGAIN;

	sum /= HI3798_TSENSOR_SAMPLES;

	millicelsius = ((int)sum - HI3798_TSENSOR_OFFSET) * HI3798_TSENSOR_MUL;
	millicelsius = millicelsius / HI3798_TSENSOR_DIV - HI3798_TSENSOR_BIAS;
	millicelsius *= 1000;

	if (millicelsius < HI3798_TEMP_MIN_MC)
		return -EAGAIN;

	*temp = millicelsius;

	return 0;
}

static const struct thermal_zone_device_ops hi3798_thermal_ops = {
	.get_temp = hi3798_thermal_get_temp,
};

static int hi3798_thermal_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct thermal_zone_device *tzd;
	struct hi3798_thermal *data;
	int temp, ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(data->regs))
		return PTR_ERR(data->regs);

	mutex_init(&data->lock);
	platform_set_drvdata(pdev, data);

	writel(HI3798_TSENSOR_ENABLE, data->regs + HI3798_TSENSOR_CTRL);
	msleep(HI3798_TSENSOR_PRIME_MS);
	writel(HI3798_TSENSOR_DISABLE, data->regs + HI3798_TSENSOR_CTRL);

	tzd = devm_thermal_of_zone_register(dev, 0, data, &hi3798_thermal_ops);
	if (IS_ERR(tzd))
		return dev_err_probe(dev, PTR_ERR(tzd),
				     "failed to register thermal zone\n");

	/*
	 * 注册 zone 时已经读过一次传感器并评估了 trip；这一次只是把第一个温度记进日志。读不出有效值也保留 zone：之后每次轮询都会再读，缓冲区换满新采样就恢复；probe 失败则整次启动都没有温度监控与降频。
	 */
	ret = hi3798_thermal_get_temp(tzd, &temp);
	if (ret) {
		dev_warn(dev, "tsensor registered, first reading invalid (%d)\n", ret);
		return 0;
	}

	dev_info(dev, "tsensor registered, %d.%03d degC\n",
		 temp / 1000, abs(temp % 1000));

	return 0;
}

static const struct of_device_id hi3798_thermal_of_match[] = {
	{ .compatible = "hisilicon,hi3798mv200-tsensor" },
	{ }
};
MODULE_DEVICE_TABLE(of, hi3798_thermal_of_match);

static struct platform_driver hi3798_thermal_driver = {
	.probe = hi3798_thermal_probe,
	.driver = {
		.name = "hi3798-thermal",
		.of_match_table = hi3798_thermal_of_match,
		/* probe 里要等 HI3798_TSENSOR_PRIME_MS，异步探测免得拖慢启动 */
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
};
module_platform_driver(hi3798_thermal_driver);

MODULE_DESCRIPTION("HiSilicon Hi3798MV200/MV300 temperature sensor");
MODULE_LICENSE("GPL");
