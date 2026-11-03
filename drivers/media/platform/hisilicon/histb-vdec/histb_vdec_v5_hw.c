// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon HiSTB V5R6 decoder: message pool and launch.
 *
 * The register order is the one the vendor's own build for this family runs;
 * the ABI note lists it with its sources.
 */
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/jiffies.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include "histb_vdec_v5.h"
#include "histb_vdec_v5_tables_h264.h"
#include "histb_vdec_v5_tables_hevc.h"

int histb_vdec_v5_pool_alloc(struct histb_vdec_v5 *dec)
{
	struct histb_vdec_v5_pool *pool = &dec->pool;

	/*
	 * The layout below is absolute from the pool base, so the base only has
	 * to be the 1024 bytes the vendor aligns to; a coherent allocation is
	 * page-aligned already. Both the CPU and the core's DMA master touch it,
	 * so it is allocated coherent rather than cached and flushed by hand. It
	 * goes with the device, because a launch keeps pointing at it until the
	 * node is gone.
	 */
	pool->va = dmam_alloc_coherent(dec->dev, V5R6_POOL_BYTES, &pool->dma,
				       GFP_KERNEL);
	if (!pool->va)
		return -ENOMEM;

	pool->size = V5R6_POOL_BYTES;
	pool->slot_off = 0;

	histb_vdec_v5_pool_seed(dec);

	return 0;
}

/*
 * Fill the tables a launch is told to read. The neighbour, deblocking and
 * motion areas start empty, because the core takes the samples and the
 * coefficient counts of the macroblocks above and left of the picture from
 * them, and the CABAC initialisation image goes in at the address the picture
 * message carries.
 */
void histb_vdec_v5_pool_seed(struct histb_vdec_v5 *dec)
{
	struct histb_vdec_v5_pool *pool = &dec->pool;

	memset(pool->va + V5R6_POOL_SED_LEFT_PHY, 0,
	       V5R6_TBL_END - V5R6_POOL_SED_LEFT_PHY);
	memcpy(pool->va + V5R6_POOL_CABAC_MN_H264, histb_vdec_v5_cabac_mn,
	       sizeof(histb_vdec_v5_cabac_mn));
	memcpy(pool->va + V5R6_POOL_CABAC_MN_HEVC, histb_vdec_v5_cabac_mn_hevc,
	       sizeof(histb_vdec_v5_cabac_mn_hevc));
}

/*
 * Program the core for one frame and ring the doorbell. The caller has already
 * filled the picture slot and the slice chain, and has registered the frame the
 * core is about to write.
 */
/*
 * Program the core for one frame and ring the doorbell. The protocol layer has
 * already filled the picture slot and the slice chain.
 */
/*
 * The SoC's clock and reset generator holds a selector and a pulse-skipping
 * configuration for the decode core, and the vendor build programs both on
 * every launch: select, clear the skip count, then pulse the load enable so
 * the choice actually latches. This tree has no clock for those bits because
 * nothing documents what the selector's codes give on this part, so the launch
 * reaches for the CRG the way the vendor's own MV310 build does.
 */
#define V5R6_CRG_VDH		0x78
#define V5R6_CRG_VDH_SEL	GENMASK(9, 8)
#define V5R6_CRG_VDH_SKIP	GENMASK(16, 12)
#define V5R6_CRG_VDH_LOAD	BIT(17)

/*
 * The sub-block clock enables are two bits wide, and V5R6_CLKSEL_RUN is
 * the value each one carries on a board that has decoded pictures. The decode
 * core, its MMU and the stream distributor are the three a picture needs; the
 * DSP, bitplane and MFDE units keep whatever the boot gave them until a format
 * asks for them.
 */
void histb_vdec_v5_clocks(struct histb_vdec_v5 *dec, bool on)
{
	static const unsigned int units[] = {
		V5R6_CLKSEL_VDEC, V5R6_CLKSEL_MMU, V5R6_CLKSEL_SCD,
	};
	u32 val = histb_vdec_v5_read(dec, V5R6_CLKSEL);
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(units); i++) {
		u32 field = V5R6_CLKSEL_FIELD(units[i]);

		val &= ~field;
		if (on)
			val |= V5R6_CLKSEL_RUN << (units[i] * 2);
	}

	histb_vdec_v5_write(dec, V5R6_CLKSEL, val);
}


static int histb_vdec_v5_clock_select(struct histb_vdec_v5 *dec)
{
	struct regmap *crg = dec->crg;
	int ret;

	if (IS_ERR(crg))
		return PTR_ERR(crg);

	ret = regmap_update_bits(crg, V5R6_CRG_VDH, V5R6_CRG_VDH_SEL,
				 FIELD_PREP(V5R6_CRG_VDH_SEL, 2));
	if (ret)
		return ret;

	ret = regmap_update_bits(crg, V5R6_CRG_VDH,
				 V5R6_CRG_VDH_SKIP | V5R6_CRG_VDH_LOAD, 0);
	if (ret)
		return ret;

	return regmap_update_bits(crg, V5R6_CRG_VDH, V5R6_CRG_VDH_LOAD,
				  V5R6_CRG_VDH_LOAD);
}

int histb_vdec_v5_launch(struct histb_vdec_v5 *dec, const struct histb_vdec_v5_job *job)
{
	unsigned int i;
	u32 *rep;
	int ret;

	ret = histb_vdec_v5_clock_select(dec);
	if (ret)
		return ret;
	histb_vdec_v5_clocks(dec, true);

	histb_vdec_v5_write(dec, V5R6_AVM_ADDR,
			  histb_vdec_v5_addr_at(job->pic_dma, job->reg_shift));
	histb_vdec_v5_write(dec, V5R6_VAM_ADDR,
			  histb_vdec_v5_addr_at(job->up_dma, job->reg_shift));
	histb_vdec_v5_write(dec, V5R6_STREAM_BASE,
			  histb_vdec_v5_addr_at(job->stream, job->reg_shift));

	/* Reconstructed picture: luma start, band pitch, chroma offset. */
	histb_vdec_v5_write(dec, V5R6_YSTADDR_1D,
			  histb_vdec_v5_addr_at(job->recon, job->reg_shift));
	/*
	 * The pitch goes in as bytes per line. The older engine wants sixteen
	 * lines of the pitch here, so this word is not portable between them.
	 */
	histb_vdec_v5_write(dec, V5R6_YSTRIDE_1D, job->pitch);
	histb_vdec_v5_write(dec, V5R6_UVOFFSET_1D, job->uv_offset);
	/* The head-info image only exists on the compressed write path. */
	histb_vdec_v5_write(dec, V5R6_HEAD_INF_OFFSET, 0);

	histb_vdec_v5_write(dec, V5R6_CFG0, job->cfg0);
	histb_vdec_v5_write(dec, V5R6_CFG1, job->cfg1);

	histb_vdec_v5_write(dec, V5R6_DOWN_CLK_CFG, V5R6_DOWN_CLK_VALUE);

	/*
	 * The seven stage watchdogs. Reset leaves them at zero, which times a stage
	 * out as soon as it starts, so a launch that never writes them can only end
	 * with an error and nothing decoded. The word ahead of the run of seven is
	 * not a watchdog and stays untouched.
	 */
	for (i = 0; i < V5R6_TIMEOUT_STAGES; i++)
		histb_vdec_v5_write(dec, V5R6_SED_TO + i * 4, job->to_val);

	/*
	 * The stream distributor is programmed at every launch. Its reference
	 * cache is used up to the part's whole width range, so the enable is
	 * kept set rather than re-derived from the frame width.
	 */
	histb_vdec_v5_write(dec, V5R6_SCD_AVS_FLAG, 0);
	histb_vdec_v5_write(dec, V5R6_SCD_EMAR_CFG, V5R6_SCD_EMAR_ENABLE);
	histb_vdec_v5_write(dec, V5R6_SCD_VDH_SELRST, 1);
	histb_vdec_v5_write(dec, V5R6_SCD_CLOCK_GATE, V5R6_SCD_GATE_VALUE);

	/*
	 * Translation stays off, which is what lets the messages and the
	 * reference surfaces carry physical addresses. The fault latches share
	 * this core's interrupt line, so they are cleared along with it.
	 */
	histb_vdec_v5_write(dec, V5R6_SMMU_SCR, 0);
	histb_vdec_v5_write(dec, V5R6_SMMU_INTMASK, V5R6_SMMU_FAULT_BITS);
	histb_vdec_v5_write(dec, V5R6_SMMU_INTCLR_S, V5R6_SMMU_FAULT_BITS);
	histb_vdec_v5_write(dec, V5R6_SMMU_INTCLR_NS, V5R6_SMMU_FAULT_BITS);

	histb_vdec_v5_write(dec, V5R6_INT_STATE, ~0U);
	/*
	 * The mask word disables by setting a bit, so the two lines that end a
	 * picture are the ones left clear: the completion, and the error the
	 * core raises instead of it when it stops on something it could not
	 * consume. Both are waited for through dec->done.
	 */
	histb_vdec_v5_write(dec, V5R6_INT_MASK,
			  U32_MAX ^ (V5R6_INT_DEC_OVER | V5R6_INT_DEC_ERR));

	reinit_completion(&dec->done);

	/*
	 * The picture report lands in this slot, and clearing it first is what
	 * makes the end-of-picture mark the core writes belong to this launch
	 * rather than to the one before it.
	 */
	rep = histb_vdec_v5_slot(dec, V5R6_SLOT_UP);
	rep[0] = 0;
	rep[1] = 0;
	rep[2] = 0;
	rep[3] = 0;
	wmb();

	/* The doorbell is a pulse held long enough for the core to see it. */
	udelay(30);
	wmb();
	histb_vdec_v5_write(dec, V5R6_START, 0);
	udelay(1);
	histb_vdec_v5_write(dec, V5R6_START, V5R6_START_GO);
	udelay(30);
	histb_vdec_v5_write(dec, V5R6_START, 0);

	/*
	 * The core reports the picture through its interrupt, which is masked
	 * and latched: waiting on the state word instead would read a bit that
	 * does not track the picture. The caller waits on dec->done.
	 */

	return 0;
}

/*
 * The motion-history area one surface gets, as this revision's firmware sizes
 * it: one macroblock grid of 64 bytes each, rounded up past the row the engine
 * reads ahead of the picture. Half of that block is the bottom-field view, so
 * the collocated pointer for a field pair is this size shifted by one.
 */
#define V5R6_PMV_PER_MB		64
#define V5R6_PMV_ROUND		143

int histb_vdec_v5_pmv_alloc(struct histb_vdec_v5 *dec, struct histb_vdec_v5_ctx *ctx,
			  unsigned int mbs)
{
	struct histb_vdec_v5_pmv *pmv = &ctx->pmv;
	size_t bytes;

	if (pmv->va && pmv->slots == V5R6_MAX_DPB_SLOTS &&
	    pmv->slot_bytes >= ((mbs * V5R6_PMV_PER_MB + V5R6_PMV_ROUND) &
				  ~127U))
		return 0;

	histb_vdec_v5_pmv_free(ctx);

	pmv->dev = dec->dev;
	pmv->slot_bytes = (mbs * V5R6_PMV_PER_MB + V5R6_PMV_ROUND) & ~127U;
	pmv->slots = V5R6_MAX_DPB_SLOTS;
	bytes = pmv->slot_bytes * pmv->slots;

	pmv->va = dma_alloc_coherent(dec->dev, bytes, &pmv->dma, GFP_KERNEL);
	if (!pmv->va) {
		pmv->slots = 0;
		return -ENOMEM;
	}

	return 0;
}

void histb_vdec_v5_pmv_free(struct histb_vdec_v5_ctx *ctx)
{
	struct histb_vdec_v5_pmv *pmv = &ctx->pmv;

	if (!pmv->va)
		return;

	dma_free_coherent(pmv->dev, pmv->slot_bytes * pmv->slots, pmv->va,
			  pmv->dma);
	pmv->va = NULL;
	pmv->dma = 0;
	pmv->slot_bytes = 0;
	pmv->slots = 0;
}

void histb_vdec_v5_abort(struct histb_vdec_v5 *dec)
{
	u32 state = 0;
	int ret;

	histb_vdec_v5_clocks(dec, true);
	histb_vdec_v5_write(dec, V5R6_INT_MASK, ~0U);
	histb_vdec_v5_write(dec, V5R6_RESET, V5R6_RESET_CORE);
	ret = readl_relaxed_poll_timeout(dec->base + V5R6_RST_STATE,
					   state, state & V5R6_RST_STATE_CORE,
					   10, 30000);
	histb_vdec_v5_write(dec, V5R6_RESET, 0);
	if (ret)
		dev_err(dec->dev, "abort reset not acknowledged (%#x)\n",
			state);
}
