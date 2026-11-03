// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon HiSTB V5R6 stateless H.264 decoder.
 *
 * One job decodes one frame. The core takes fully parsed syntax: it never reads
 * a slice header, and the only bitstream it fetches is slice_data(), addressed
 * per slice as a stream segment. The picture slot of the message pool carries
 * the frame-level state -- the coded size, the picture order counts, the
 * reference-picture table and the quantiser matrix -- and the slices follow it
 * as a chain of slots, each naming its successor, the chain ending on a zero
 * link.
 *
 * Three encodings are what the packers below exist to produce: an address word
 * is a 16-byte granule index rather than a byte address; a reference picture is
 * named by the picture-container slot the driver assigned to the surface that
 * holds it, so that id is kept per capture buffer next to the motion-history
 * slot; and a segment runs to the end of its slice's NAL, trailing bits and
 * all, because nothing in the tail of a CABAC slice tells data from padding.
 */

#include <linux/bitfield.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>

#include <media/v4l2-h264.h>
#include <media/videobuf2-dma-contig.h>

#include "histb_vdec_v5.h"

/* A DPB entry that owns neither a picture-container nor a motion slot. */
#define V5R6_H264_SLOT_NONE		0xffu

/*
 * Slice-type codes the engine counts motion history by, indexed by slice_type %
 * 5: P, B, I, SP, SI. The switch types share the P and I codes.
 */
static const u8 histb_vdec_v5_h264_slice_type[5] = { 1, 2, 0, 1, 0 };

/*
 * Reference-entry flags, indexed by 2 * picture structure + MBAFF, of which the
 * low two bits are kept. Only the frame-coding entry is reachable, because
 * run() refuses the other shapes.
 */
static const u8 histb_vdec_v5_h264_list_struct[8] = { 0, 1, 3, 3, 3, 3, 2, 2 };

/* Assembly state one frame shares between the picture and the slice packers. */
struct histb_vdec_v5_h264_run {
	const struct v4l2_ctrl_h264_sps *sps;
	const struct v4l2_ctrl_h264_pps *pps;
	const struct v4l2_ctrl_h264_decode_params *dec;
	const struct v4l2_ctrl_h264_pred_weights *weights;
	const struct v4l2_ctrl_h264_scaling_matrix *scaling;
	const struct v4l2_ctrl_h264_slice_params *slices;
	struct histb_vdec_v5_frame *frame;
	unsigned int		num_slices;
	unsigned int		num_valid;
	unsigned int		width_mbs;
	unsigned int		height_mbs;
	unsigned int		mbs;
	u32			payload_bits;	/* end of the OUTPUT span */
	u32			stream_base;	/* granule of the first slice */
	const u8		*src_va;
	dma_addr_t		src_dma;
	dma_addr_t		dst_dma;
	u8			dpb2apc[V4L2_H264_NUM_DPB_ENTRIES];
	u8			dpb2pmv[V4L2_H264_NUM_DPB_ENTRIES];
	u32			img_addr[V5R6_MAX_DPB_SLOTS];
	s32			apc_poc[2][V5R6_MAX_DPB_SLOTS];
	struct v4l2_h264_reference reflist_p[V4L2_H264_REF_LIST_LEN];
	struct v4l2_h264_reference reflist_b0[V4L2_H264_REF_LIST_LEN];
	struct v4l2_h264_reference reflist_b1[V4L2_H264_REF_LIST_LEN];
	u32			slice_starts[V5R6_MAX_SLICES + 1];
};

/*
 * Per-context state of this protocol. The two slot tables are kept offset by
 * one so that a zeroed entry says "no slot given yet", and the assembly state
 * lives here rather than on the kernel stack.
 */
struct histb_vdec_v5_h264 {
	u8			apc_map[V5R6_MAX_FRAMES];
	u8			pmv_map[V5R6_MAX_FRAMES];
	void			*rbsp_va;
	dma_addr_t		rbsp_dma;
	size_t			rbsp_size;
	struct histb_vdec_v5_h264_run run;
};

/* Address word of a device-pool area, as the messages want it. */
static u32 histb_vdec_v5_h264_pool_addr(struct histb_vdec_v5 *dec, unsigned int off)
{
	return histb_vdec_v5_addr_at(dec->pool.dma + off, V5R6_ADDR_SHIFT);
}

/* The motion-history area one slot of this context's pool holds. */
static dma_addr_t histb_vdec_v5_h264_pmv_dma(const struct histb_vdec_v5_ctx *ctx,
					   unsigned int slot)
{
	return ctx->pmv.dma + (dma_addr_t)slot * ctx->pmv.slot_bytes;
}

/*
 * Slot ids of one capture buffer, taken modulo the pool each one wraps over.
 * The picture-container slot is what the reference tables of every slice name;
 * the motion-history slot indexes the per-context areas, which the caller has
 * given this context by the time it is asked for.
 */
static unsigned int histb_vdec_v5_h264_apc_alloc(struct histb_vdec_v5_ctx *ctx,
					       unsigned int index)
{
	struct histb_vdec_v5_h264 *h264 = ctx->priv;
	u8 id = h264->apc_map[index];

	if (id == 0) {
		id = ctx->apc_next % V5R6_MAX_DPB_SLOTS + 1;
		ctx->apc_next++;
		h264->apc_map[index] = id;
	}

	return id - 1;
}

static unsigned int histb_vdec_v5_h264_pmv_alloc(struct histb_vdec_v5_ctx *ctx,
					       unsigned int index)
{
	struct histb_vdec_v5_h264 *h264 = ctx->priv;
	u8 id = h264->pmv_map[index];

	if (id == 0) {
		id = ctx->pmv_next % ctx->pmv.slots + 1;
		ctx->pmv_next++;
		h264->pmv_map[index] = id;
	}

	return id - 1;
}

/*
 * Bit position of every slice NAL in the OUTPUT plane: nal_unit_type 1 or 5,
 * the two a message slot can describe, data partitions among them not. A
 * four-byte start code matches once, because its extra leading zero never forms
 * a 00 00 01 triple with the prefix. The count returned is the number found,
 * which may exceed @max.
 */
static unsigned int histb_vdec_v5_h264_find_nals(const u8 *buf, u32 size,
					       u32 *starts, unsigned int max)
{
	unsigned int n = 0;
	u32 i;

	for (i = 0; i + 3 < size; i++) {
		u8 nal_type = buf[i + 3] & 0x1f;

		if (buf[i] != 0 || buf[i + 1] != 0 || buf[i + 2] != 1)
			continue;
		if (nal_type != 1 && nal_type != 5)
			continue;
		if (n < max)
			starts[n] = (i + 3) * 8;
		n++;
	}

	return n;
}

/* The first bit of a slice's data, behind the header the core never reads. */
static u32 histb_vdec_v5_h264_data_bit(const struct histb_vdec_v5_h264_run *run,
				     unsigned int index)
{
	return run->slice_starts[index] + run->slices[index].header_bit_size;
}

/*
 * End of one slice's data inside the span handed to the core. Slice NAL units
 * are separated by annex-b start codes, and 00 00 01 cannot occur inside a NAL
 * unit because emulation prevention rewrites any 00 00 followed by 00 or 01, so
 * the start code before the next NAL is the one byte pattern that marks where
 * this slice stops. Everything between the last coded bin and that start code
 * stays in the segment: the core takes it and only flags surplus bits.
 */
static u32 histb_vdec_v5_h264_trim_nal(const u8 *va, u32 end_bit)
{
	if (va == NULL || end_bit < 24)
		return end_bit;
	if (va[end_bit / 8 - 3] == 0 && va[end_bit / 8 - 2] == 0 &&
	    va[end_bit / 8 - 1] == 1)
		return end_bit - 24;

	return end_bit;
}

/*
 * Stream segment of one slice, slice words 0 to 2. The core fetches on 16-byte
 * granules: word 0 is the granule holding the first data bit, as a distance
 * from the granule the stream base was programmed at; word 1 is the bit offset
 * inside that granule; and word 2 the number of valid bits.
 */
static void histb_vdec_v5_h264_pack_segment(u32 *w, u32 data_bit, u32 base,
					  u32 valid_bits)
{
	w[0] = (((data_bit / 8) & ~15u) - base) >> V5R6_ADDR_SHIFT;
	w[1] = data_bit & 127u;
	w[2] = valid_bits;
}

static u32 histb_vdec_v5_h264_qm_word(const u8 *coeffs)
{
	return coeffs[0] | coeffs[1] << 8 | coeffs[2] << 16 | coeffs[3] << 24;
}

/*
 * One scaling list in raster order, four coefficients per word, least
 * significant byte first. Without a picture matrix the spec's default of 16 per
 * coefficient applies.
 */
static u32 *histb_vdec_v5_h264_pack_qm_list(u32 *dst, const u8 *list,
					 unsigned int words, bool present)
{
	unsigned int i;

	for (i = 0; i < words; i++) {
		if (present)
			*dst++ = histb_vdec_v5_h264_qm_word(list + 4 * i);
		else
			*dst++ = 0x10101010u;
	}

	return dst;
}

/*
 * Quantiser matrix, picture words 64 to 119: the six 4x4 lists followed by the
 * two 8x8 luma lists the 4:2:0 path uses.
 */
static void
histb_vdec_v5_h264_pack_qm(u32 *dst,
			 const struct v4l2_ctrl_h264_scaling_matrix *m,
			 bool present)
{
	unsigned int i;

	for (i = 0; i < 6; i++)
		dst = histb_vdec_v5_h264_pack_qm_list(dst, m->scaling_list_4x4[i],
						    16, present);
	for (i = 0; i < 2; i++)
		dst = histb_vdec_v5_h264_pack_qm_list(dst, m->scaling_list_8x8[i],
						    64, present);
}

/* Whether one scaling list repeats a single coefficient through the row. */
static bool histb_vdec_v5_h264_qm_flat(const u8 *list, unsigned int len)
{
	unsigned int i;

	for (i = 1; i < len; i++) {
		if (list[i] != list[0])
			return false;
	}

	return true;
}

/*
 * Which matrices this core takes as the protocol says. It applies the six 4x4
 * lists and the two luma 8x8 lists, but a picture whose three intra 4x4 lists
 * differ from each other comes back wrong, so only the shape known to match is
 * accepted here.
 */
static bool
histb_vdec_v5_h264_qm_ok(const struct v4l2_ctrl_h264_scaling_matrix *m)
{
	u8 v = m->scaling_list_4x4[0][0];
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(m->scaling_list_4x4); i++) {
		if (!histb_vdec_v5_h264_qm_flat(m->scaling_list_4x4[i], 16))
			return false;
		if (i < 3 && m->scaling_list_4x4[i][0] != v)
			return false;
	}

	for (i = 0; i < 2; i++) {
		if (!histb_vdec_v5_h264_qm_flat(m->scaling_list_8x8[i], 64))
			return false;
	}

	return true;
}

/*
 * Prediction weights of one list, slice words 64 to 255: six blocks of 32
 * words, the luma and the two chroma components of list 0 then of list 1. Luma
 * and the first chroma component pack as offset, weight and denominator; the
 * second chroma component as weight and offset. Weights keep nine bits, offsets
 * eight.
 */
static void
histb_vdec_v5_h264_pack_weights(u32 *slot,
			      const struct v4l2_ctrl_h264_pred_weights *pw,
			      unsigned int count, bool list1)
{
	const struct v4l2_h264_weight_factors *f = &pw->weight_factors[list1];
	unsigned int base = list1 ? 96u : 64u;
	unsigned int i;

	for (i = 0; i < count; i++) {
		slot[base + i] = ((u32)(u8)f->luma_offset[i] << 12) |
				 ((u32)f->luma_weight[i] & 0x1ff) << 3 |
				 (pw->luma_log2_weight_denom & 7);
		slot[base + 64 + i] =
			((u32)(u8)f->chroma_offset[i][0] << 12) |
			((u32)f->chroma_weight[i][0] & 0x1ff) << 3 |
			(pw->chroma_log2_weight_denom & 7);
		slot[base + 128 + i] =
			((u32)f->chroma_weight[i][1] & 0x1ff) |
			((u32)(u8)f->chroma_offset[i][1] << 9);
	}
}

/*
 * Reference list of one slice. The entry flags take four bits each, eight per
 * word from word 12 for list 0 and word 16 for list 1; the picture-container
 * ids take five bits each, four per word from word 20 for list 0 and word 28
 * for list 1, holding the slot number and the bottom-field marker, which frame
 * coding leaves clear.
 */
static void
histb_vdec_v5_h264_pack_declist(u32 *slot, const struct v4l2_h264_reference *list,
			      unsigned int count, unsigned int distinct,
			      const u8 *dpb2apc,
			      const struct v4l2_ctrl_h264_decode_params *dec,
			      bool list1)
{
	unsigned int flag_word = list1 ? 16 : 12;
	unsigned int apc_word = list1 ? 28 : 20;
	unsigned int i;

	for (i = 0; i < count; i++) {
		const struct v4l2_h264_dpb_entry *dpb;
		const struct v4l2_h264_reference *ref;
		unsigned int pos = i;
		u32 flags;

		/*
		 * A slice may declare more list positions than the pictures its
		 * list carries; the padded positions repeat the last picture
		 * that is really there, which is what encoders that declare the
		 * extra position assume.
		 */
		if (distinct != 0 && pos >= distinct)
			pos = distinct - 1;
		ref = &list[pos];
		dpb = &dec->dpb[ref->index];

		/*
		 * The table entry of a whole picture in a frame-coded stream,
		 * with the DPB entry's long-term flag on top of it.
		 */
		flags = histb_vdec_v5_h264_list_struct[0] & 3;
		if (dpb->flags & V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM)
			flags |= 8;

		slot[flag_word + (i >> 3)] |= flags << (4 * (i & 7));
		slot[apc_word + (i >> 2)] |=
			((u32)dpb2apc[ref->index] << 1) << (5 * (i & 3));
	}
}

/*
 * Whether any slice needs the prediction weight table: implicit weighted
 * prediction is a mode the core derives by itself and hands it nothing.
 */
static bool
histb_vdec_v5_h264_needs_weights(const struct histb_vdec_v5_h264_run *run)
{
	unsigned int i;

	for (i = 0; i < run->num_slices; i++) {
		const struct v4l2_ctrl_h264_slice_params *slice =
			&run->slices[i];

		if (V4L2_H264_CTRL_PRED_WEIGHTS_REQUIRED(run->pps, slice))
			return true;
	}

	return false;
}

/*
 * A reference list is usable only when every entry the slice reads names an
 * active DPB entry: a client that left the arrays zeroed would otherwise be
 * taken as meaning "DPB entry 0, no fields".
 */
static bool
histb_vdec_v5_h264_ref_list_ok(const struct v4l2_h264_reference *list,
			     unsigned int count,
			     const struct v4l2_ctrl_h264_decode_params *dec)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		if ((list[i].fields & V4L2_H264_FRAME_REF) == 0)
			return false;
		if (list[i].index >= V4L2_H264_NUM_DPB_ENTRIES)
			return false;
		if ((dec->dpb[list[i].index].flags &
		     V4L2_H264_DPB_ENTRY_FLAG_ACTIVE) == 0)
			return false;
	}

	return true;
}

/*
 * The lists one slice uses: the submitted ones when they name active DPB
 * entries, because only they can carry the reordering of 8.2.4.3, else the
 * initial construction of 8.2.4.2 prepared during job assembly. Returns true
 * when the submitted lists were taken.
 */
static bool
histb_vdec_v5_h264_slice_lists(const struct histb_vdec_v5_h264_run *run,
			     unsigned int index, unsigned int count0,
			     unsigned int count1, bool is_b,
			     const struct v4l2_h264_reference **pl0,
			     const struct v4l2_h264_reference **pl1)
{
	const struct v4l2_ctrl_h264_slice_params *slice = &run->slices[index];
	bool usable;

	*pl0 = is_b ? run->reflist_b0 : run->reflist_p;
	*pl1 = run->reflist_b1;

	usable = histb_vdec_v5_h264_ref_list_ok(slice->ref_pic_list0, count0,
					      run->dec);
	if (is_b)
		usable = usable &&
			histb_vdec_v5_h264_ref_list_ok(slice->ref_pic_list1,
						     count1, run->dec);
	if (!usable)
		return false;

	*pl0 = slice->ref_pic_list0;
	*pl1 = slice->ref_pic_list1;

	return true;
}

static void histb_vdec_v5_h264_pack_pic(struct histb_vdec_v5 *dec,
				      const struct histb_vdec_v5_ctx *ctx,
				      const struct histb_vdec_v5_h264_run *run,
				      u32 *pic)
{
	const struct v4l2_ctrl_h264_pps *pps = run->pps;
	const struct v4l2_ctrl_h264_decode_params *par = run->dec;
	dma_addr_t pmv;
	unsigned int i;

	memset(pic, 0, V5R6_SLOT_BYTES);

	/*
	 * Word 0 is the coded size in macroblocks, the flags of the picture
	 * parameter set and the reference flag. Bits 15:14 hold the picture
	 * structure, of which frame coding is the zero this path can express,
	 * and the 4:2:0 flag stands for every picture it accepts.
	 */
	pic[0] = FIELD_PREP(GENMASK(8, 0), run->width_mbs - 1) |
		 FIELD_PREP(GENMASK(24, 16), run->height_mbs - 1) |
		 BIT(25) |
		 ((pps->flags & V4L2_H264_PPS_FLAG_CONSTRAINED_INTRA_PRED) ?
		  BIT(26) : 0) |
		 ((pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE) ?
		  BIT(27) : 0) |
		 ((pps->flags & V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE) ?
		  BIT(28) : 0) |
		 (par->nal_ref_idc != 0 ? BIT(31) : 0);
	pic[1] = histb_vdec_v5_addr_at(run->dst_dma, V5R6_ADDR_SHIFT);

	/*
	 * Word 2 is the picture's order count: 8.5.1.1 gives the greater of the
	 * two field counts, which for the frame coding this path accepts is the
	 * top one. Words 3 and 4 are the per-field counts, which this
	 * generation's HAL writes only for macroblock-adaptive frame-field
	 * pictures -- refused here, so they stay zero as the reset leaves them.
	 */
	pic[2] = max_t(s32, par->top_field_order_cnt,
		       par->bottom_field_order_cnt);

	/* The neighbour tables, then this picture's own motion-history area. */
	pic[5] = histb_vdec_v5_h264_pool_addr(dec, V5R6_TBL_SED_TOP);
	pic[6] = histb_vdec_v5_h264_pool_addr(dec, V5R6_TBL_PMV_TOP);
	pmv = histb_vdec_v5_h264_pmv_dma(ctx, run->frame->pmv_slot);
	pic[7] = histb_vdec_v5_addr_at(pmv, V5R6_ADDR_SHIFT);
	pic[8] = histb_vdec_v5_h264_pool_addr(dec, V5R6_TBL_RCNT_TOP);

	/* Head of the slice chain. */
	pic[9] = histb_vdec_v5_addr_at(histb_vdec_v5_slot_dma(dec, V5R6_SLOT_SLICE),
				     V5R6_ADDR_SHIFT);

	/* Reference addresses by slot, and the POC pair of each slot. */
	for (i = 0; i < V5R6_MAX_DPB_SLOTS; i++) {
		pic[10 + i] = run->img_addr[i];
		pic[27 + 2 * i] = (u32)run->apc_poc[0][i];
		pic[28 + 2 * i] = (u32)run->apc_poc[1][i];
	}

	pic[26] = histb_vdec_v5_h264_pool_addr(dec, V5R6_POOL_CABAC_MN_H264);
	pic[59] = histb_vdec_v5_h264_pool_addr(dec, V5R6_TBL_DBLK_TOP);

	histb_vdec_v5_h264_pack_qm(&pic[64], run->scaling,
				 pps->flags &
				 V4L2_H264_PPS_FLAG_SCALING_MATRIX_PRESENT);
}

static int histb_vdec_v5_h264_pack_slice(struct histb_vdec_v5 *dec,
				       const struct histb_vdec_v5_ctx *ctx,
				       const struct histb_vdec_v5_h264_run *run,
				       unsigned int index)
{
	const struct v4l2_ctrl_h264_slice_params *slice = &run->slices[index];
	const struct v4l2_ctrl_h264_pps *pps = run->pps;
	const struct v4l2_h264_reference *l0, *l1, *col;
	unsigned int type = slice->slice_type % 5;
	unsigned int last = run->num_slices - 1;
	unsigned int count0 = 0, count1 = 0;
	unsigned int distinct0 = 0, distinct1 = 0;
	bool is_b = type == V4L2_H264_SLICE_TYPE_B;
	bool from_app, weighted;
	unsigned int wmode, qp;
	u32 data_bit, end_bit, valid;
	u32 *slot = histb_vdec_v5_slot(dec, V5R6_SLOT_SLICE + index);

	memset(slot, 0, V5R6_SLOT_BYTES);

	if (type != V4L2_H264_SLICE_TYPE_I && type != V4L2_H264_SLICE_TYPE_SI) {
		count0 = min_t(unsigned int,
			       slice->num_ref_idx_l0_active_minus1 + 1,
			       V4L2_H264_REF_LIST_LEN);
	}
	if (is_b) {
		count1 = min_t(unsigned int,
			       slice->num_ref_idx_l1_active_minus1 + 1,
			       V4L2_H264_REF_LIST_LEN);
	}

	from_app = histb_vdec_v5_h264_slice_lists(run, index, count0, count1,
						is_b, &l0, &l1);

	/*
	 * How many positions of the list carry a picture. A submitted list is
	 * only taken when all of its positions name an active entry, so all of
	 * them carry a picture -- including a position naming the same picture
	 * as its neighbour does, which 8.2.4.3 allows. The in-kernel list has
	 * no such marker: it fills its tail with DPB entry 0, and only its
	 * first num_valid positions carry a picture.
	 */
	if (count0 != 0) {
		distinct0 = from_app ? count0 :
			min_t(unsigned int, count0, run->num_valid);
		if (distinct0 == 0)
			count0 = 0;
	}
	if (count1 != 0) {
		distinct1 = from_app ? count1 :
			min_t(unsigned int, count1, run->num_valid);
		if (distinct1 == 0)
			count1 = 0;
	}

	/*
	 * Collocated picture for direct prediction. 8.8.2 picks it by the
	 * slice's own flag, but this generation addresses the collocated motion
	 * vectors from the first list 1 entry whatever the direct mode is, so
	 * that is the one handed over.
	 */
	col = l1;

	end_bit = run->payload_bits;
	if (index < last)
		end_bit = run->slice_starts[index + 1];
	end_bit = histb_vdec_v5_h264_trim_nal(run->src_va, end_bit);
	data_bit = histb_vdec_v5_h264_data_bit(run, index);
	if (data_bit >= end_bit) {
		dev_dbg(dec->dev, "slice %u leaves no data bits\n", index);
		return -EINVAL;
	}

	/*
	 * The segment runs to the end of this slice's NAL, rbsp_trailing_bits
	 * included. Walking the tail back to a stop bit is not possible here: a
	 * CABAC slice's last byte is arithmetic-coded data, so the lowest set bit
	 * is a bin as often as it is the stop bit, and cutting at it truncates
	 * the entropy stream -- measured as garbage in the last macroblocks of
	 * the picture. The core takes the padding and only sets its surplus-bits
	 * error flag.
	 */
	valid = end_bit - data_bit;
	histb_vdec_v5_h264_pack_segment(&slot[0], data_bit, run->stream_base,
				      valid);

	/*
	 * Word 6 holds the slice's first macroblock, its cabac initial idc and
	 * the absolute slice qp, the spec's SliceQPy.
	 */
	qp = clamp_t(int, 26 + pps->pic_init_qp_minus26 + slice->slice_qp_delta,
		     0, 51);
	slot[6] = (slice->first_mb_in_slice & 0xfffffu) | ((qp & 63) << 26) |
		  ((slice->cabac_init_idc & 3) << 24);

	/*
	 * Word 7 is the slice type, the two list sizes the entries below carry,
	 * the declared lengths and the macroblock-level flags.
	 */
	slot[7] = histb_vdec_v5_h264_slice_type[type] |
		  (count0 << 2) | (count1 << 8) |
		  ((slice->num_ref_idx_l0_active_minus1 & 31) << 16) |
		  ((slice->num_ref_idx_l1_active_minus1 & 31) << 21);
	if (run->sps->flags & V4L2_H264_SPS_FLAG_DIRECT_8X8_INFERENCE)
		slot[7] |= BIT(14);
	if (slice->flags & V4L2_H264_SLICE_FLAG_DIRECT_SPATIAL_MV_PRED)
		slot[7] |= BIT(15);

	/*
	 * Words 8 and 9 address the collocated picture's motion history, the
	 * second its far half; both stay zero unless the access unit has a B
	 * slice, because nothing else reads them.
	 */
	if (is_b && count1 != 0) {
		unsigned int col_slot = run->dpb2pmv[col[0].index];
		dma_addr_t pmv = histb_vdec_v5_h264_pmv_dma(ctx, col_slot);

		slot[8] = histb_vdec_v5_addr_at(pmv, V5R6_ADDR_SHIFT);
		slot[9] = histb_vdec_v5_addr_at(pmv + (ctx->pmv.slot_bytes >> 1),
					      V5R6_ADDR_SHIFT);
	}

	/*
	 * Weighting mode: the PPS's bipred mode for a B slice, its weighted
	 * prediction flag for an inter slice of any other type. Mode 2 is
	 * implicit weighted prediction, which the core carries out by itself,
	 * so only mode 1 has a table to hand down.
	 */
	if (is_b) {
		wmode = pps->weighted_bipred_idc & 3;
	} else {
		wmode = ((pps->flags & V4L2_H264_PPS_FLAG_WEIGHTED_PRED) &&
			 (type == V4L2_H264_SLICE_TYPE_P ||
			  type == V4L2_H264_SLICE_TYPE_SP)) ? 1 : 0;
	}
	weighted = wmode == 1;

	slot[10] = (((u32)pps->chroma_qp_index_offset & 31) << 5) |
		   ((u32)pps->second_chroma_qp_index_offset & 31) |
		   (wmode << 16);
	slot[11] = (((u32)slice->slice_alpha_c0_offset_div2 & 15) << 16) |
		   (((u32)slice->slice_beta_offset_div2 & 15) << 8) |
		   (slice->disable_deblocking_filter_idc & 0xff);

	histb_vdec_v5_h264_pack_declist(slot, l0, count0, distinct0, run->dpb2apc,
				      run->dec, false);
	histb_vdec_v5_h264_pack_declist(slot, l1, count1, distinct1, run->dpb2apc,
				      run->dec, true);

	/*
	 * Word 44 is the last macroblock this slice covers, because the core
	 * starts the next one at the macroblock after it; word 63 links the
	 * next slot, and a zero there ends the chain.
	 */
	slot[44] = run->mbs - 1;
	slot[63] = 0;
	if (index < last) {
		unsigned int next = V5R6_SLOT_SLICE + index + 1;

		slot[44] = run->slices[index + 1].first_mb_in_slice - 1;
		slot[63] = histb_vdec_v5_addr_at(histb_vdec_v5_slot_dma(dec, next),
					       V5R6_ADDR_SHIFT);
	}

	if (weighted) {
		const struct v4l2_ctrl_h264_pred_weights *pw = run->weights;

		histb_vdec_v5_h264_pack_weights(slot, pw, count0, false);
		if (is_b)
			histb_vdec_v5_h264_pack_weights(slot, pw, count1, true);
	}

	return 0;
}

/*
 * Whether this submission discards every stored picture, which is what makes
 * recycling the slot ids here lose nothing: an IDR starts a new reference chain
 * and the uAPI hands it with no entry still flagged as a stored picture.
 */
static bool
histb_vdec_v5_h264_flush(const struct v4l2_ctrl_h264_decode_params *dec)
{
	unsigned int i;

	if ((dec->flags & V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC) == 0)
		return false;

	for (i = 0; i < V4L2_H264_NUM_DPB_ENTRIES; i++) {
		if (dec->dpb[i].flags & (V4L2_H264_DPB_ENTRY_FLAG_VALID |
					 V4L2_H264_DPB_ENTRY_FLAG_ACTIVE))
			return false;
	}

	return true;
}

/*
 * Syntax this core cannot be handed. Each of these leaves a message word with
 * no value that means what the bitstream says, so the picture is refused rather
 * than programmed from a guess.
 */
static int histb_vdec_v5_h264_check_syntax(struct histb_vdec_v5 *dec,
					 const struct histb_vdec_v5_h264_run *run)
{
	const struct v4l2_ctrl_h264_sps *sps = run->sps;
	const struct v4l2_ctrl_h264_pps *pps = run->pps;

	if (V4L2_H264_SPS_HAS_CHROMA_FORMAT(sps) &&
	    (sps->chroma_format_idc != 1 || sps->bit_depth_luma_minus8 != 0 ||
	     sps->bit_depth_chroma_minus8 != 0 ||
	     (sps->flags & V4L2_H264_SPS_FLAG_SEPARATE_COLOUR_PLANE))) {
		dev_dbg(dec->dev, "only 8-bit 4:2:0 reaches this core\n");
		return -EINVAL;
	}

	if (sps->flags & V4L2_H264_SPS_FLAG_QPPRIME_Y_ZERO_TRANSFORM_BYPASS) {
		dev_dbg(dec->dev, "the core has no transform bypass path\n");
		return -EINVAL;
	}

	/*
	 * Slice-group partitioning moves where a slice's data starts and the
	 * PPS carries no map for it; a redundant picture would have to be
	 * presented as a picture of its own.
	 */
	if (pps->num_slice_groups_minus1 != 0 ||
	    (pps->flags & V4L2_H264_PPS_FLAG_REDUNDANT_PIC_CNT_PRESENT)) {
		dev_dbg(dec->dev,
			"no message for slice groups or redundant pictures\n");
		return -EINVAL;
	}

	if (pps->flags & V4L2_H264_PPS_FLAG_SCALING_MATRIX_PRESENT) {
		if (run->scaling == NULL ||
		    !histb_vdec_v5_h264_qm_ok(run->scaling)) {
			dev_dbg(dec->dev,
				"quantiser matrix this core cannot apply\n");
			return -EINVAL;
		}
	}

	if (sps->max_num_ref_frames > V5R6_MAX_DPB_SLOTS) {
		dev_dbg(dec->dev, "%u reference frames exceed %u slots\n",
			sps->max_num_ref_frames, V5R6_MAX_DPB_SLOTS);
		return -EINVAL;
	}

	if (histb_vdec_v5_h264_needs_weights(run) && run->weights == NULL) {
		dev_dbg(dec->dev, "no weight table for this slice\n");
		return -EINVAL;
	}

	return 0;
}

/*
 * Hand the engine the slice spans with the emulation prevention bytes of the
 * NAL units taken out. Whether the core undoes that escaping itself is not
 * settled for this generation, and the copy is right either way: a span that
 * still carries 00 00 03 is wrong for a core that reads its bytes literally,
 * and taking them out is inert for one that undoes them.
 *
 * The copy starts at the first slice's NAL header byte and the slice offsets
 * are rebased onto it, leaving the OUTPUT plane alone; a span with no
 * prevention byte is used where it lies, so the common case pays no copy.
 */
static int histb_vdec_v5_h264_unpack(struct histb_vdec_v5 *dec,
				   struct histb_vdec_v5_ctx *ctx,
				   struct histb_vdec_v5_h264_run *run)
{
	struct histb_vdec_v5_h264 *h264 = ctx->priv;
	unsigned int first = run->slice_starts[0] / 8;
	unsigned int end = run->payload_bits / 8;
	unsigned int esc_next = first;
	unsigned int out = 0, idx = 0, zeros = 0;
	unsigned int i;
	u8 *dst;

	if (run->src_va == NULL || first >= end)
		return 0;

	for (i = first; i + 2 < end; i++) {
		if (run->src_va[i] == 0 && run->src_va[i + 1] == 0 &&
		    run->src_va[i + 2] == 0x03)
			break;
	}
	if (i + 2 >= end)
		return 0;

	if (h264->rbsp_size < end - first) {
		size_t want = round_up(end - first, 64 * 1024);

		if (h264->rbsp_va != NULL)
			dma_free_coherent(dec->dev, h264->rbsp_size,
					  h264->rbsp_va, h264->rbsp_dma);
		h264->rbsp_va = dma_alloc_coherent(dec->dev, want,
						   &h264->rbsp_dma, GFP_KERNEL);
		if (h264->rbsp_va == NULL) {
			h264->rbsp_size = 0;
			return -ENOMEM;
		}
		h264->rbsp_size = want;
	}
	dst = h264->rbsp_va;

	for (i = first; i < end; i++) {
		u8 byte = run->src_va[i];

		if (i == esc_next) {
			esc_next = idx + 1 < run->num_slices ?
				run->slice_starts[idx + 1] / 8 : end;
			run->slice_starts[idx++] = out * 8;
			zeros = 0;
		}
		if (zeros >= 2 && byte == 0x03) {
			zeros = 0;
			continue;
		}
		dst[out++] = byte;
		zeros = byte != 0 ? 0 : zeros + 1;
	}

	run->payload_bits = out * 8;
	run->src_va = dst;
	run->src_dma = h264->rbsp_dma;

	return 0;
}

/*
 * Map the DPB onto the reference table: every active entry has to name a
 * capture buffer of this context, and the picture-container slot given to that
 * buffer is what the messages refer to. An entry with no buffer, or a buffer
 * with no record, leaves a reference the core would fetch from nowhere.
 */
static int histb_vdec_v5_h264_map_refs(struct histb_vdec_v5 *dec,
				     struct histb_vdec_v5_ctx *ctx,
				     struct histb_vdec_v5_h264_run *run,
				     struct vb2_v4l2_buffer *dst)
{
	struct histb_vdec_v5_h264 *h264 = ctx->priv;
	struct histb_vdec_v5_frame *frame;
	struct vb2_queue *cap_q;
	unsigned long owned = 0;
	unsigned int i;

	if (dst->vb2_buf.index >= V5R6_MAX_FRAMES) {
		dev_dbg(dec->dev, "capture buffer %u beyond the frame table\n",
			dst->vb2_buf.index);
		return -EINVAL;
	}

	/*
	 * A fresh chain may reuse every slot, so recycle the ids before any of
	 * this picture's own gets handed out.
	 */
	if (histb_vdec_v5_h264_flush(run->dec)) {
		memset(h264->apc_map, 0, sizeof(h264->apc_map));
		memset(h264->pmv_map, 0, sizeof(h264->pmv_map));
		ctx->apc_next = 0;
		ctx->pmv_next = 0;
	}

	frame = histb_vdec_v5_frame_of(ctx, dst->vb2_buf.index);
	if (frame == NULL) {
		dev_dbg(dec->dev, "no frame record for buffer %u\n",
			dst->vb2_buf.index);
		return -EINVAL;
	}

	frame->apc_slot = histb_vdec_v5_h264_apc_alloc(ctx, dst->vb2_buf.index);
	frame->pmv_slot = histb_vdec_v5_h264_pmv_alloc(ctx, dst->vb2_buf.index);
	run->frame = frame;
	owned = BIT(frame->apc_slot);

	/*
	 * Until a surface owns a slot it holds this picture's address, which is
	 * what the tail of the fill loop below repeats.
	 */
	for (i = 0; i < V5R6_MAX_DPB_SLOTS; i++) {
		run->img_addr[i] = histb_vdec_v5_addr_at(run->dst_dma,
						       V5R6_ADDR_SHIFT);
		run->apc_poc[0][i] = 0;
		run->apc_poc[1][i] = 0;
	}

	cap_q = v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx);
	for (i = 0; i < V4L2_H264_NUM_DPB_ENTRIES; i++) {
		const struct v4l2_h264_dpb_entry *entry = &run->dec->dpb[i];
		struct histb_vdec_v5_frame *ref;
		struct vb2_buffer *vb;

		run->dpb2apc[i] = V5R6_H264_SLOT_NONE;
		run->dpb2pmv[i] = V5R6_H264_SLOT_NONE;
		if ((entry->flags & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE) == 0)
			continue;

		vb = vb2_find_buffer(cap_q, entry->reference_ts);
		if (vb == NULL || vb->index >= V5R6_MAX_FRAMES) {
			dev_dbg(dec->dev, "reference %u names no buffer\n", i);
			return -EINVAL;
		}
		ref = histb_vdec_v5_frame_of(ctx, vb->index);
		if (ref == NULL) {
			dev_dbg(dec->dev, "reference %u has no record\n", i);
			return -EINVAL;
		}

		run->dpb2apc[i] = histb_vdec_v5_h264_apc_alloc(ctx, vb->index);
		run->dpb2pmv[i] = histb_vdec_v5_h264_pmv_alloc(ctx, vb->index);
		ref->apc_slot = run->dpb2apc[i];
		ref->pmv_slot = run->dpb2pmv[i];
		owned |= BIT(run->dpb2apc[i]);
		run->img_addr[run->dpb2apc[i]] =
			histb_vdec_v5_addr_at(ref->dma, V5R6_ADDR_SHIFT);
		run->apc_poc[0][run->dpb2apc[i]] = entry->top_field_order_cnt;
		run->apc_poc[1][run->dpb2apc[i]] =
			entry->bottom_field_order_cnt;
	}

	/*
	 * A slot no surface owns still has to hold an address the core may
	 * lawfully fetch: a zero word is a real DMA target, so those entries
	 * repeat the first one.
	 */
	for (i = 0; i < V5R6_MAX_DPB_SLOTS; i++) {
		if ((owned & BIT(i)) == 0)
			run->img_addr[i] = run->img_addr[0];
	}

	return 0;
}


/*
 * An imported OUTPUT buffer is read through the CPU to find the slice NALs,
 * and its exporter may be a cached one. Copy the access unit into the driver's
 * own coherent buffer, so that every later read, and the core's own fetch, work
 * on memory whose contents the CPU has already observed. The caller holds the
 * CPU access window on the source plane for the whole job, which is what makes
 * this copy safe; a locally allocated buffer needs no window and no copy.
 */
static int histb_vdec_v5_h264_copy_import(struct histb_vdec_v5 *dec,
					struct histb_vdec_v5_h264 *h264,
					struct vb2_v4l2_buffer *src,
					struct histb_vdec_v5_h264_run *run)
{
	struct dma_buf *dbuf = src->vb2_buf.planes[0].dbuf;
	size_t len = run->payload_bits / 8;
	u8 *dst;

	if (dbuf == NULL)
		return 0;
	if (h264 == NULL || run->src_va == NULL)
		return -EINVAL;

	if (h264->rbsp_size < len) {
		size_t want = round_up(len, 64 * 1024);

		if (h264->rbsp_va != NULL)
			dma_free_coherent(dec->dev, h264->rbsp_size,
					  h264->rbsp_va, h264->rbsp_dma);
		h264->rbsp_va = dma_alloc_coherent(dec->dev, want,
						   &h264->rbsp_dma, GFP_KERNEL);
		if (h264->rbsp_va == NULL) {
			h264->rbsp_size = 0;
			return -ENOMEM;
		}
		h264->rbsp_size = want;
	}

	dst = h264->rbsp_va;
	memcpy(dst, run->src_va, len);

	run->src_va = dst;
	run->src_dma = h264->rbsp_dma;

	return 0;
}

static int histb_vdec_v5_h264_run(struct histb_vdec_v5_ctx *ctx,
			       struct vb2_v4l2_buffer *src,
			       struct vb2_v4l2_buffer *dst)
{
	struct histb_vdec_v5 *dec = histb_vdec_v5_of_ctx(ctx);
	struct histb_vdec_v5_h264_run *run;
	struct v4l2_h264_reflist_builder builder;
	struct histb_vdec_v5_job job = {
		.to_val		= V5R6_TIMEOUT_DEFAULT,
		.reg_shift	= V5R6_ADDR_SHIFT,
	};
	struct histb_vdec_v5_h264 *h264 = ctx->priv;
	struct histb_vdec_v5_frame *dst_frame;
	struct v4l2_ctrl *ctrl;
	unsigned int found, i, pitch, lines, uv_offset, n_slices;
	unsigned long bytes, start_code;
	u32 *pic;
	const u32 *sl0;
	int ret;

	if (h264 == NULL) {
		h264 = kzalloc_obj(*h264);
		if (h264 == NULL)
			return -ENOMEM;
		ctx->priv = h264;
	}
	run = &h264->run;

	/*
	 * The driver core has applied the request to the control handler before
	 * scheduling this job, so p_cur carries this frame's values. One OUTPUT
	 * buffer carries the whole access unit, so the slice array is what says
	 * how many slice slots the chain needs.
	 */
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_H264_SPS);
	run->sps = ctrl != NULL ? ctrl->p_cur.p_h264_sps : NULL;
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_H264_PPS);
	run->pps = ctrl != NULL ? ctrl->p_cur.p_h264_pps : NULL;
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl,
			      V4L2_CID_STATELESS_H264_DECODE_PARAMS);
	run->dec = ctrl != NULL ? ctrl->p_cur.p_h264_decode_params : NULL;
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl,
			      V4L2_CID_STATELESS_H264_PRED_WEIGHTS);
	run->weights = ctrl != NULL ? ctrl->p_cur.p_h264_pred_weights : NULL;
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl,
			      V4L2_CID_STATELESS_H264_SCALING_MATRIX);
	run->scaling = ctrl != NULL ? ctrl->p_cur.p_h264_scaling_matrix : NULL;
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl,
			      V4L2_CID_STATELESS_H264_SLICE_PARAMS);
	run->slices = ctrl != NULL ? ctrl->p_cur.p_h264_slice_params : NULL;
	n_slices = ctrl != NULL ? ctrl->new_elems : 0;
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl,
			      V4L2_CID_STATELESS_H264_START_CODE);
	start_code = ctrl != NULL ? ctrl->cur.val :
		V4L2_STATELESS_H264_START_CODE_ANNEX_B;

	/*
	 * Every word of the picture message comes from these controls: the
	 * frame-level state has no other source, and the slice chain is built
	 * from the slice array, so a missing or empty one leaves words that
	 * nothing can write.
	 */
	if (run->sps == NULL || run->pps == NULL || run->dec == NULL ||
	    run->slices == NULL || n_slices == 0) {
		dev_dbg(dec->dev, "no controls for this access unit\n");
		return -EINVAL;
	}
	if (n_slices > V5R6_MAX_SLICES) {
		dev_dbg(dec->dev, "%u slices exceed the %u a chain holds\n",
			n_slices, V5R6_MAX_SLICES);
		return -EINVAL;
	}
	run->num_slices = n_slices;

	bytes = vb2_get_plane_payload(&src->vb2_buf, 0);
	if (bytes <= src->vb2_buf.planes[0].data_offset) {
		dev_dbg(dec->dev, "submission carries no bitstream\n");
		return -EINVAL;
	}
	bytes -= src->vb2_buf.planes[0].data_offset;
	run->payload_bits = (u32)bytes * 8;
	run->src_dma = vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0) +
			       src->vb2_buf.planes[0].data_offset;
	dst_frame = histb_vdec_v5_frame_of(ctx, dst->vb2_buf.index);
	if (dst_frame == NULL)
		return -EINVAL;
	run->dst_dma = dst_frame->dma;
	run->src_va = vb2_plane_vaddr(&src->vb2_buf, 0);
	if (run->src_va != NULL)
		run->src_va += src->vb2_buf.planes[0].data_offset;

	ret = histb_vdec_v5_h264_copy_import(dec, h264, src, run);
	if (ret)
		return ret;

	ret = histb_vdec_v5_h264_check_syntax(dec, run);
	if (ret)
		return ret;

	/*
	 * Field and field/frame coding is refused outright. A stored field pair
	 * keeps half of its motion history in each half of an area and this API
	 * hands every field picture its own DPB entry and capture buffer, so
	 * the half a B slice needs cannot be named; delivering the picture
	 * without it would leave a surface wrong that later pictures still
	 * reference.
	 */
	if (run->dec->flags & V4L2_H264_DECODE_PARAM_FLAG_FIELD_PIC) {
		dev_dbg(dec->dev, "field pictures are not expressible\n");
		return -EINVAL;
	}
	if (run->sps->flags & V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD) {
		dev_dbg(dec->dev,
			"macroblock-adaptive frame/field is not expressible\n");
		return -EINVAL;
	}

	/*
	 * The coded grid, and the capture geometry the launch is programmed
	 * with. The engine writes the coded picture rather than the visible
	 * rectangle, and sizes the motion-history areas from the capture grid,
	 * so a coded picture has to stay inside that grid and inside the buffer
	 * the grid was agreed for. Comparing the coded width through the band
	 * pitch is what makes that exact: the pitch says which 256-column class
	 * the negotiated width falls in, and a coded width in a higher class
	 * needs columns the grid does not offer.
	 */
	run->width_mbs = run->sps->pic_width_in_mbs_minus1 + 1;
	run->height_mbs = run->sps->pic_height_in_map_units_minus1 + 1;
	if ((run->sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY) == 0)
		run->height_mbs *= 2;
	run->mbs = run->width_mbs * run->height_mbs;
	if (run->width_mbs * 16 > V5R6_MAX_WIDTH ||
	    run->height_mbs * 16 > V5R6_MAX_HEIGHT) {
		dev_dbg(dec->dev, "coded size %ux%u beyond this core\n",
			run->width_mbs * 16, run->height_mbs * 16);
		return -EINVAL;
	}
	histb_vdec_v5_buf_geom(ctx, &pitch, &lines, &uv_offset);
	if (V5R6_BAND_PITCH(run->width_mbs * 16) > pitch ||
	    run->height_mbs * 16 > lines ||
	    vb2_get_plane_payload(&dst->vb2_buf, 0) <
	    histb_vdec_v5_linear_size(run->width_mbs * 16,
				    run->height_mbs * 16)) {
		dev_dbg(dec->dev, "capture grid too small for %ux%u coded\n",
			run->width_mbs * 16, run->height_mbs * 16);
		return -EINVAL;
	}

	ret = histb_vdec_v5_pmv_alloc(dec, ctx, run->mbs);
	if (ret)
		return ret;

	if (start_code == V4L2_STATELESS_H264_START_CODE_ANNEX_B) {
		/*
		 * Locating the slices reads the OUTPUT plane through the kernel
		 * mapping, so a mapping that came back unavailable leaves every
		 * slice boundary unknown and the job is refused rather than
		 * programmed from offsets nothing verified.
		 */
		if (run->src_va == NULL) {
			dev_dbg(dec->dev, "OUTPUT queue has no mapping\n");
			return -EINVAL;
		}
		found = histb_vdec_v5_h264_find_nals(run->src_va,
					run->payload_bits / 8,
					run->slice_starts,
					ARRAY_SIZE(run->slice_starts));
		/*
		 * The scan has to find exactly one NAL per declared slice: each
		 * declared slice gets one slot, addressed by the start found
		 * here, so a different count would mean programming spans the
		 * control does not describe.
		 */
		if (found != n_slices) {
			dev_dbg(dec->dev, "%u slice NALs for %u declared\n",
				found, n_slices);
			return -EINVAL;
		}
	} else {
		/*
		 * Without start codes nothing in the buffer separates one slice
		 * from the next, so a single-slice submission is the only
		 * layout that can be consumed. The NAL header bits belong to
		 * the offset either way: a segment is computed from the first
		 * data bit.
		 */
		if (n_slices != 1) {
			dev_dbg(dec->dev,
				"no start codes to separate %u slices\n",
				n_slices);
			return -EINVAL;
		}
		run->slice_starts[0] = 0;
	}

	ret = histb_vdec_v5_h264_unpack(dec, ctx, run);
	if (ret)
		return ret;

	ret = histb_vdec_v5_h264_map_refs(dec, ctx, run, dst);
	if (ret)
		return ret;

	v4l2_h264_init_reflist_builder(&builder, run->dec, run->sps,
				       run->dec->dpb);
	run->num_valid = builder.num_valid;
	v4l2_h264_build_p_ref_list(&builder, run->reflist_p);
	v4l2_h264_build_b_ref_lists(&builder, run->reflist_b0, run->reflist_b1);

	/*
	 * The stream base is the granule the first slice's data starts in, so
	 * every segment is a distance from there.
	 */
	run->stream_base = (histb_vdec_v5_h264_data_bit(run, 0) / 8) & ~15u;

	pic = histb_vdec_v5_slot(dec, V5R6_SLOT_PIC);
	histb_vdec_v5_h264_pack_pic(dec, ctx, run, pic);

	for (i = 0; i < n_slices; i++) {
		ret = histb_vdec_v5_h264_pack_slice(dec, ctx, run, i);
		if (ret)
			return ret;
	}

	/*
	 * The last real slot links to this one, so it has to read as an empty
	 * segment even where an earlier job left a longer chain here.
	 */
	memset(histb_vdec_v5_slot(dec, V5R6_SLOT_SLICE + n_slices), 0,
	       V5R6_SLOT_BYTES);

	/*
	 * The neighbour-history areas the core reads back are consumed by a
	 * launch, so they have to be seeded again every time it is given one.
	 */
	histb_vdec_v5_pool_seed(dec);

	job.cfg0 = ((run->mbs - 1) & V5R6_CFG0_MBAMT) |
		   V5R6_CFG0_COEFIDX_DETECT | V5R6_CFG0_LD_QMATRIX;
	/*
	 * One slice group per picture, which is every picture a stateless
	 * decoder is handed, and the motion-vector output the reference chain
	 * of the next picture reads back.
	 */
	job.cfg1 = FIELD_PREP(V5R6_CFG1_VID_STD, ctx->desc->vidstd) |
		   FIELD_PREP(V5R6_CFG1_MAX_SLCGRP_NUM, 3) |
		   V5R6_CFG1_MV_OUT | V5R6_CFG1_FST_SLC_GRP;
	job.pitch = pitch;
	job.stream = run->src_dma + run->stream_base;
	job.recon = run->dst_dma;
	job.uv_offset = uv_offset;
	job.pic_dma = histb_vdec_v5_slot_dma(dec, V5R6_SLOT_PIC);
	job.up_dma = histb_vdec_v5_slot_dma(dec, V5R6_SLOT_UP);
	job.pic = pic;
	job.slice = histb_vdec_v5_slot(dec, V5R6_SLOT_SLICE);
	ctx->mbs = run->mbs;
	ctx->slices = n_slices;

	/*
	 * The words the engine is about to read. A refused picture or a wrong
	 * one has nothing else to look at: these are derived here and reach no
	 * other interface.
	 */
	sl0 = (const u32 *)job.slice;
	dev_dbg(dec->dev, "pic %08x %08x %08x %08x %08x %08x %08x %08x\n",
		pic[0], pic[1], pic[2], pic[7], pic[9], pic[10], pic[11],
		pic[26]);
	dev_dbg(dec->dev, "slc %08x %08x %08x %08x %08x %08x\n",
		sl0[0], sl0[2], sl0[6], sl0[7], sl0[8], sl0[11]);

	ret = histb_vdec_v5_launch(dec, &job);
	if (ret) {
		dev_err(dec->dev, "launch failed: int %#x state %#x\n",
			dec->int_state, dec->last_state);
		return ret;
	}

	return 0;
}

/*
 * The picture and slice messages of this protocol take 8-bit 4:2:0 frames and
 * nothing else. The geometry is not judged here: the handler holds an all-zero
 * default, which TRY_EXT_CTRLS reads back as though a client had set it, and no
 * picture size can be judged before one is submitted, so run() refuses those.
 */
static int histb_vdec_v5_h264_try_ctrl(struct v4l2_ctrl *ctrl)
{
	const struct v4l2_ctrl_h264_sps *sps;

	if (ctrl->id != V4L2_CID_STATELESS_H264_SPS)
		return 0;

	sps = ctrl->p_new.p_h264_sps;
	if (V4L2_H264_SPS_HAS_CHROMA_FORMAT(sps) &&
	    (sps->chroma_format_idc != 1 || sps->bit_depth_luma_minus8 != 0 ||
	     sps->bit_depth_chroma_minus8 != 0 ||
	     (sps->flags & V4L2_H264_SPS_FLAG_SEPARATE_COLOUR_PLANE)))
		return -EINVAL;

	return 0;
}

static void histb_vdec_v5_h264_stop(struct histb_vdec_v5_ctx *ctx)
{
	struct histb_vdec_v5_h264 *h264 = ctx->priv;
	struct histb_vdec_v5 *dec;

	if (h264 == NULL)
		return;

	dec = histb_vdec_v5_of_ctx(ctx);
	histb_vdec_v5_pmv_free(ctx);
	if (h264->rbsp_va != NULL)
		dma_free_coherent(dec->dev, h264->rbsp_size, h264->rbsp_va,
				  h264->rbsp_dma);
	kfree(h264);
	ctx->priv = NULL;
	ctx->apc_next = 0;
	ctx->pmv_next = 0;
}

static const struct v4l2_ctrl_config histb_vdec_v5_h264_ctrls[] = {
	{
		.id = V4L2_CID_STATELESS_H264_SPS,
	},
	{
		.id = V4L2_CID_STATELESS_H264_PPS,
	},
	{
		.id = V4L2_CID_STATELESS_H264_SCALING_MATRIX,
	},
	{
		.id = V4L2_CID_STATELESS_H264_PRED_WEIGHTS,
	},
	{
		.id = V4L2_CID_STATELESS_H264_DECODE_PARAMS,
	},
	{
		.id = V4L2_CID_STATELESS_H264_SLICE_PARAMS,
		/*
		 * v4l2_ctrl_new_custom() only preserves cfg->flags when the
		 * config carries an explicit name: otherwise it runs
		 * v4l2_ctrl_fill(), which zeroes the flags and would drop the
		 * dynamic-array property.
		 */
		.name = "H264 Slice Parameters",
		.type = V4L2_CTRL_TYPE_H264_SLICE_PARAMS,
		.dims = { 8 },
		.flags = V4L2_CTRL_FLAG_DYNAMIC_ARRAY,
	},
	{
		.id = V4L2_CID_STATELESS_H264_DECODE_MODE,
		.min = V4L2_STATELESS_H264_DECODE_MODE_FRAME_BASED,
		.max = V4L2_STATELESS_H264_DECODE_MODE_FRAME_BASED,
		.def = V4L2_STATELESS_H264_DECODE_MODE_FRAME_BASED,
	},
	{
		.id = V4L2_CID_STATELESS_H264_START_CODE,
		.min = V4L2_STATELESS_H264_START_CODE_NONE,
		.max = V4L2_STATELESS_H264_START_CODE_ANNEX_B,
		.def = V4L2_STATELESS_H264_START_CODE_ANNEX_B,
	},
};

/* A request without these can not be built into a picture message. */
static const u32 histb_vdec_v5_h264_req_ctrls[] = {
	V4L2_CID_STATELESS_H264_SPS,
	V4L2_CID_STATELESS_H264_PPS,
	V4L2_CID_STATELESS_H264_DECODE_PARAMS,
	V4L2_CID_STATELESS_H264_SLICE_PARAMS,
};

/* A surface the capture queue dropped no longer holds slots of either kind.
 */
static void histb_vdec_v5_h264_buf_release(struct histb_vdec_v5_ctx *ctx,
					   unsigned int index)
{
	struct histb_vdec_v5_h264 *h264 = ctx->priv;

	/* Before the first job there are no slot tables to give back. */
	if (h264 == NULL || index >= V5R6_MAX_FRAMES)
		return;

	h264->apc_map[index] = 0;
	h264->pmv_map[index] = 0;
}

static const struct histb_vdec_v5_codec_ops histb_vdec_v5_h264_ops = {
	.run		= histb_vdec_v5_h264_run,
	.try_ctrl	= histb_vdec_v5_h264_try_ctrl,
	.stop		= histb_vdec_v5_h264_stop,
	.buf_release	= histb_vdec_v5_h264_buf_release,
};

/* The band pitch this protocol's pictures are written at, in bytes per line. */
static unsigned int histb_vdec_v5_h264_stride(unsigned int width)
{
	return V5R6_BAND_PITCH(width);
}

const histb_vdec_v5_fmt_desc histb_vdec_v5_h264_fmt = {
	.fourcc		= V4L2_PIX_FMT_H264_SLICE,
	.vidstd		= 0,
	.num_ctrls	= ARRAY_SIZE(histb_vdec_v5_h264_ctrls),
	.ctrls		= histb_vdec_v5_h264_ctrls,
	.ops		= &histb_vdec_v5_h264_ops,
	.stride		= histb_vdec_v5_h264_stride,
	.grid_align	= 32,
	.req_ctrls	= histb_vdec_v5_h264_req_ctrls,
	.num_req	= ARRAY_SIZE(histb_vdec_v5_h264_req_ctrls),
};
