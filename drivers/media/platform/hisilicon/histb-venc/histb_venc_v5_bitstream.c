// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon HiSTB V500 encoder: the syntax the core does not generate.
 *
 * The core codes the macroblocks and the picture-level fields it is given in
 * the command image, but it takes the parameter sets and the fixed part of the
 * slice header from the driver. The parameter sets reach it as Annex-B bytes
 * cut into words; the slice header reaches it in groups of up to 32 bits, the
 * first bit of a full group at its top and a short last group right-aligned --
 * the arrangement the core's own bit reader expects.
 *
 * The core adds the slice address and the fields that follow it (quantiser
 * parameter, deblocking, entropy flags), the NAL header of each slice, and the
 * escaping between what the driver writes and what it codes, so none of those
 * appear here.
 */

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "histb_venc_v5.h"

#define NAL_SPS			7
#define NAL_PPS			8
#define NAL_HEVC_VPS		32
#define NAL_HEVC_SPS		33
#define NAL_HEVC_PPS		34

struct histb_venc_v5_bits {
	u8 data[96];
	unsigned int count;
	bool overflow;
};

static void histb_venc_v5_put(struct histb_venc_v5_bits *bits, u32 value,
			    unsigned int count)
{
	unsigned int i;

	if (bits->count + count > sizeof(bits->data) * 8) {
		bits->overflow = true;
		return;
	}

	for (i = 0; i < count; i++) {
		unsigned int index = bits->count + i;
		unsigned int shift = count - i - 1;

		if ((value >> shift) & 1)
			bits->data[index / 8] |= BIT(7 - index % 8);
	}

	bits->count += count;
}

/*
 * Unsigned Exp-Golomb: as many zero bits as the length of value plus one
 * demands, then that length's bits of value plus one. ue(2) is therefore the
 * three bits 011, which is how the vendor's own writers spell it.
 */
static void histb_venc_v5_ue(struct histb_venc_v5_bits *bits, u32 value)
{
	u32 length = value + 1;
	unsigned int zeros = fls(length) - 1;

	histb_venc_v5_put(bits, 0, zeros);
	histb_venc_v5_put(bits, length, zeros + 1);
}

static void histb_venc_v5_se(struct histb_venc_v5_bits *bits, s32 value)
{
	histb_venc_v5_ue(bits, value <= 0 ? -2 * (u32)value : 2 * (u32)value - 1);
}

static void histb_venc_v5_trailing(struct histb_venc_v5_bits *bits)
{
	histb_venc_v5_put(bits, 1, 1);

	if (bits->count % 8)
		histb_venc_v5_put(bits, 0, 8 - bits->count % 8);
}

/*
 * Turn a segment into the words the image carries. Bits keep their order: the
 * first of the segment is the top bit of the first word, and the word holding
 * the short last group is shifted down so its own first bit ends up lowest.
 */
static void histb_venc_v5_words(struct histb_venc_v5_bits *bits, u32 *words,
			      unsigned int word_count)
{
	unsigned int i;

	memset(words, 0, word_count * sizeof(*words));

	for (i = 0; i < bits->count; i++) {
		u32 bit = (bits->data[i / 8] >> (7 - i % 8)) & 1;

		words[i / 32] |= bit << (31 - i % 32);
	}

	if (bits->count > word_count * 32)
		bits->overflow = true;
	else if (bits->count % 32)
		words[bits->count / 32] >>= 32 - bits->count % 32;
}

/*
 * The level that names this picture, with the direct-8x8 flag that goes with
 * it: ITU-T H.264 Table A-1, checked over the frame size, the macroblock rate
 * and the bit rate at once. A picture the table cannot name is refused rather
 * than given a level it may not be played at.
 */
struct venc_v2_level {
	unsigned int max_fs;
	unsigned int max_mbps;
	unsigned int max_br_kbps;
	u8 level_idc;
};

/*
 * MaxBR is the Baseline/Main/Extended column, which the High column equals at
 * every level this encoder reaches.
 */
static const struct venc_v2_level venc_v2_levels[] = {
	{    99,    1485,     64, 10 }, {   396,    3000,    192, 11 },
	{   396,    6000,    384, 12 }, {   396,   11880,    768, 13 },
	{   396,   11880,   2000, 20 }, {   792,   19800,   4000, 21 },
	{  1620,   40500,   4000, 22 }, {  1620,   40500,  10000, 30 },
	{  3600,  108000,  14000, 31 }, {  5120,  216000,  20000, 32 },
	{  8192,  245760,  20000, 40 }, {  8192,  245760,  50000, 41 },
	{  8704,  522240,  50000, 42 }, { 22080,  589824, 135000, 50 },
	{ 36864,  983040, 240000, 51 },
};

static u8 histb_venc_v5_level(unsigned int total_mb, unsigned int mbps,
			      unsigned int br_kbps, bool *direct8x8)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(venc_v2_levels); i++) {
		if (total_mb <= venc_v2_levels[i].max_fs &&
		    mbps <= venc_v2_levels[i].max_mbps &&
		    br_kbps <= venc_v2_levels[i].max_br_kbps) {
			*direct8x8 = total_mb >= 1620;
			return venc_v2_levels[i].level_idc;
		}
	}

	return 0;
}

/*
 * The syntax's own profile number, in the order the image names the profiles.
 */
static u8 histb_venc_v5_profile_idc(unsigned int profile)
{
	static const u8 idc[] = { 66, 77, 88, 100 };

	return profile < ARRAY_SIZE(idc) ? idc[profile] : 66;
}

static void histb_venc_v5_sps(struct histb_venc_v5_bits *bits,
			      const struct histb_venc_v5_frame *frame,
			      u8 level, bool direct8x8)
{
	unsigned int mbw = DIV_ROUND_UP(frame->width, 16);
	unsigned int mbh = DIV_ROUND_UP(frame->height, 16);
	unsigned int width = frame->width;
	unsigned int height = frame->height;
	unsigned int fps_den = max_t(unsigned int, frame->fps_den, 1);
	unsigned int fps = frame->fps_num;
	unsigned int profile = frame->profile;

	histb_venc_v5_put(bits, histb_venc_v5_profile_idc(profile), 8);
	histb_venc_v5_put(bits, 0, 8);		/* constraint and reserved */
	histb_venc_v5_put(bits, level, 8);
	histb_venc_v5_ue(bits, 0);		/* seq_parameter_set_id */
	/*
	 * Only the High family declares chroma format, depth and scaling lists;
	 * the vendor's own High writer leaves three of those five out, so this
	 * one follows the syntax rather than the vendor.
	 */
	if (profile == V500_H264_PROFILE_HIGH) {
		histb_venc_v5_ue(bits, 1);	/* chroma_format_idc: 4:2:0 */
		histb_venc_v5_ue(bits, 0);	/* bit_depth_luma_minus8 */
		histb_venc_v5_ue(bits, 0);	/* bit_depth_chroma_minus8 */
		histb_venc_v5_put(bits, 0, 1);	/* qpprime_y_zero_transform_bypass */
		histb_venc_v5_put(bits, 0, 1);	/* seq_scaling_matrix_present_flag */
	}
	/*
	 * Neither Main nor High needs anything else before the picture geometry,
	 * and the core codes no scaling list, so the declaration stops there.
	 */
	histb_venc_v5_ue(bits, 0);		/* log2_max_frame_num_minus4 */
	histb_venc_v5_ue(bits, 2);		/* pic_order_cnt_type: implicit */
	histb_venc_v5_ue(bits, 1);		/* num_ref_frames */
	histb_venc_v5_put(bits, 0, 1);		/* gaps_in_frame_num_value */
	histb_venc_v5_ue(bits, mbw - 1);
	histb_venc_v5_ue(bits, mbh - 1);
	histb_venc_v5_put(bits, 1, 1);		/* frame_mbs_only_flag */
	histb_venc_v5_put(bits, direct8x8, 1);	/* direct_8x8_inference_flag */

	/* The flag itself is unconditional; only its four offsets are. */
	histb_venc_v5_put(bits, mbw * 16 != width || mbh * 16 != height, 1);
	if (mbw * 16 != width || mbh * 16 != height) {
		histb_venc_v5_ue(bits, 0);	/* left */
		histb_venc_v5_ue(bits, (mbw * 16 - width) / 2);
		histb_venc_v5_ue(bits, 0);	/* top */
		histb_venc_v5_ue(bits, (mbh * 16 - height) / 2);
	}

	/*
	 * A timing-only VUI: it names the picture rate and no signalling, so a
	 * reader knows what frame interval the coded rate was matched to. One
	 * tick per field, twice the frame rate of ticks in a second.
	 */
	histb_venc_v5_put(bits, 1, 1);	/* vui_parameters_present */
	histb_venc_v5_put(bits, 0, 1);	/* aspect_ratio_info_present */
	histb_venc_v5_put(bits, 0, 1);	/* overscan_info_present */
	histb_venc_v5_put(bits, 0, 1);	/* video_signal_type_present */
	histb_venc_v5_put(bits, 0, 1);	/* chroma_loc_info_present */
	histb_venc_v5_put(bits, 1, 1);	/* timing_info_present */
	histb_venc_v5_put(bits, 1, 32);	/* num_units_in_tick */
	histb_venc_v5_put(bits, 2 * fps / fps_den, 32);	/* time_scale */
	histb_venc_v5_put(bits, 1, 1);	/* fixed_frame_rate */
	histb_venc_v5_put(bits, 0, 1);	/* nal_hrd_parameters_present */
	histb_venc_v5_put(bits, 0, 1);	/* vcl_hrd_parameters_present */
	histb_venc_v5_put(bits, 0, 1);	/* pic_struct_present */
	histb_venc_v5_put(bits, 0, 1);	/* bitstream_restriction */
	histb_venc_v5_trailing(bits);
}

/*
 * The picture parameters. The core's High arrangement codes its first macroblock
 * with the 8x8 luma prediction mode -- one bit longer a mb_type than the 16x16
 * the other profiles use, and the same count of prediction modes after it -- so
 * the High form has to carry the optional tail that lets a decoder read that
 * type: the flag, no scaling matrices, and the second chroma offset it brings.
 */
static void histb_venc_v5_pps(struct histb_venc_v5_bits *bits, bool cabac, bool high)
{
	histb_venc_v5_ue(bits, 0);		/* pic_parameter_set_id */
	histb_venc_v5_ue(bits, 0);		/* seq_parameter_set_id */
	histb_venc_v5_put(bits, cabac, 1);	/* entropy_coding_mode_flag */
	histb_venc_v5_put(bits, 0, 1);		/* pic_order_present_flag */
	histb_venc_v5_ue(bits, 0);		/* num_slice_groups_minus1 */
	histb_venc_v5_ue(bits, 0);		/* num_ref_idx_l0_active_minus1 */
	histb_venc_v5_ue(bits, 0);		/* num_ref_idx_l1_active_minus1 */
	histb_venc_v5_put(bits, 0, 1);		/* weighted_pred_flag */
	histb_venc_v5_put(bits, 0, 2);		/* weighted_bipred_idc */
	histb_venc_v5_se(bits, 0);		/* pic_init_qp_minus26 */
	histb_venc_v5_se(bits, 0);		/* pic_init_qs_minus26 */
	histb_venc_v5_se(bits, 0);		/* chroma_qp_index_offset */
	histb_venc_v5_put(bits, 1, 1);		/* deblocking_filter_control */
	histb_venc_v5_put(bits, 0, 1);		/* constrained_intra_pred_flag */
	histb_venc_v5_put(bits, 0, 1);	/* redundant_pic_cnt_present_flag */
	if (high) {
		histb_venc_v5_put(bits, 1, 1);	/* transform_8x8_mode_flag */
		histb_venc_v5_put(bits, 0, 1);	/* pic_scaling_matrix_present */
		histb_venc_v5_se(bits, 0);	/* second_chroma_qp_index_offset */
	}
	histb_venc_v5_trailing(bits);
}

/*
 * The profile_tier_level block both HEVC parameter sets carry: Main profile and
 * Main tier. The core's own writer leaves the level at zero, which names no
 * level at all, so this one says 5.1.
 */
static void histb_venc_v5_hevc_ptl(struct histb_venc_v5_bits *bits, bool progressive,
				 bool msp, unsigned int level_idc)
{
	histb_venc_v5_put(bits, 0, 2);		/* general_profile_space */
	histb_venc_v5_put(bits, 0, 1);		/* general_tier_flag: Main */
	histb_venc_v5_put(bits, msp ? 3 : 1, 5);	/* general_profile_idc */
	histb_venc_v5_put(bits, msp ? 0x70000000 : 0x60000000, 32);
	histb_venc_v5_put(bits, progressive, 1);
	histb_venc_v5_put(bits, 0, 1);		/* general_interlaced_source_flag */
	histb_venc_v5_put(bits, 0, 1);		/* general_non_packed_constraint */
	histb_venc_v5_put(bits, 0, 1);		/* general_frame_only_constraint */
	histb_venc_v5_put(bits, 0, 32);		/* reserved constraint bits */
	histb_venc_v5_put(bits, 0, 12);		/* the rest of the 44 bits */
	histb_venc_v5_put(bits, level_idc, 8);
}

/* Level idc is 30 times the level; the menu order is the spec's. */
static unsigned int histb_venc_v5_hevc_level_idc(unsigned int level)
{
	static const unsigned int idc[] = {
		30, 60, 63, 90, 93, 120, 123, 150, 153, 156, 180, 183, 186,
	};

	return level < ARRAY_SIZE(idc) ? idc[level] : 153;
}

static void histb_venc_v5_hevc_vps(struct histb_venc_v5_bits *bits, bool msp,
				 unsigned int level_idc)
{
	histb_venc_v5_put(bits, 0, 4);		/* vps_video_parameter_set_id */
	histb_venc_v5_put(bits, 3, 2);		/* base layer internal/available */
	histb_venc_v5_put(bits, 0, 6);		/* vps_max_layers_minus1 */
	histb_venc_v5_put(bits, 0, 3);		/* vps_max_sub_layers_minus1 */
	histb_venc_v5_put(bits, 1, 1);		/* vps_temporal_id_nesting_flag */
	histb_venc_v5_put(bits, 0xffff, 16);
	histb_venc_v5_hevc_ptl(bits, true, msp, level_idc);
	histb_venc_v5_put(bits, 1, 1);		/* vps_sub_layer_ordering_present */
	histb_venc_v5_ue(bits, msp ? 0 : 1);	/* vps_max_dec_pic_buffering_minus1 */
	histb_venc_v5_ue(bits, 0);		/* vps_max_num_reorder_pics */
	histb_venc_v5_ue(bits, 0);		/* vps_max_latency_increase_plus1 */
	histb_venc_v5_put(bits, 0, 6);		/* vps_max_layer_id */
	histb_venc_v5_ue(bits, 0);		/* vps_num_layer_sets_minus1 */
	/*
	 * The picture rate is not signalled here: the VPS timing block is the
	 * one part of this syntax the published revisions disagree about, and
	 * the reader-side convention is to take the rate from the sequence
	 * parameter set's VUI (which is where x265 writes it too).
	 */
	histb_venc_v5_put(bits, 0, 1);		/* vps_timing_info_present_flag */
	histb_venc_v5_put(bits, 0, 1);		/* vps_extension_flag */
	histb_venc_v5_trailing(bits);
}

/*
 * The coding tree is 32x32 and the transform tree stops at 16x16, which is
 * what the core's sizes require; one short-term reference one picture back,
 * temporal motion vectors on, and PCM on because the vendor stream declares it.
 */
static void histb_venc_v5_hevc_sps(struct histb_venc_v5_bits *bits,
				 unsigned int width, unsigned int height,
				 bool msp, unsigned int level_idc,
				 unsigned int fps_num, unsigned int fps_den)
{
	unsigned int coded_w = ALIGN(width, 8);
	unsigned int coded_h = ALIGN(height, 8);
	bool cropped = coded_w != width || coded_h != height;

	histb_venc_v5_put(bits, 0, 4);		/* sps_video_parameter_set_id */
	histb_venc_v5_put(bits, 0, 3);		/* sps_max_sub_layers_minus1 */
	histb_venc_v5_put(bits, 1, 1);		/* sps_temporal_id_nesting_flag */
	histb_venc_v5_hevc_ptl(bits, false, msp, level_idc);
	histb_venc_v5_ue(bits, 0);		/* sps_seq_parameter_set_id */
	histb_venc_v5_ue(bits, 1);		/* chroma_format_idc: 4:2:0 */
	histb_venc_v5_ue(bits, coded_w);
	histb_venc_v5_ue(bits, coded_h);
	histb_venc_v5_put(bits, cropped, 1);	/* conformance_window_flag */
	if (cropped) {
		histb_venc_v5_ue(bits, 0);	/* conf_win_left_offset */
		histb_venc_v5_ue(bits, (coded_w - width) / 2);
		histb_venc_v5_ue(bits, 0);	/* conf_win_top_offset */
		histb_venc_v5_ue(bits, (coded_h - height) / 2);
	}
	histb_venc_v5_ue(bits, 0);		/* bit_depth_luma_minus8 */
	histb_venc_v5_ue(bits, 0);		/* bit_depth_chroma_minus8 */
	histb_venc_v5_ue(bits, 12);		/* log2_max_pic_order_cnt_lsb_minus4 */
	histb_venc_v5_put(bits, 1, 1);		/* sub_layer_ordering_present */
	histb_venc_v5_ue(bits, msp ? 0 : 1);	/* sps_max_dec_pic_buffering_minus1 */
	histb_venc_v5_ue(bits, 0);		/* sps_max_num_reorder_pics */
	histb_venc_v5_ue(bits, 0);		/* sps_max_latency_increase_plus1 */
	histb_venc_v5_ue(bits, 0);		/* log2_min_luma_coding_block_size_minus3 */
	histb_venc_v5_ue(bits, 2);		/* log2_diff_max_min_luma_coding_block_size */
	histb_venc_v5_ue(bits, 0);		/* log2_min_luma_transform_block_size_minus2 */
	histb_venc_v5_ue(bits, 2);		/* log2_diff_max_min_transform_block_size */
	histb_venc_v5_ue(bits, 0);		/* max_transform_hierarchy_depth_inter */
	histb_venc_v5_ue(bits, 0);		/* max_transform_hierarchy_depth_intra */
	histb_venc_v5_put(bits, 0, 1);		/* scaling_list_enabled_flag */
	histb_venc_v5_put(bits, 0, 1);		/* amp_enabled_flag */
	histb_venc_v5_put(bits, 1, 1);		/* sample_adaptive_offset_enabled */
	histb_venc_v5_put(bits, 1, 1);		/* pcm_enabled_flag */
	histb_venc_v5_put(bits, 7, 4);		/* pcm_sample_bit_depth_luma_minus1 */
	histb_venc_v5_put(bits, 7, 4);		/* pcm_sample_bit_depth_chroma_minus1 */
	histb_venc_v5_ue(bits, 0);		/* log2_min_pcm_luma_block_size_minus3 */
	histb_venc_v5_ue(bits, 0);		/* log2_diff_max_min_pcm_luma_block */
	histb_venc_v5_put(bits, 0, 1);		/* pcm_loop_filter_disabled_flag */
	if (!msp) {
		histb_venc_v5_ue(bits, 1);	/* num_short_term_ref_pic_sets */
		histb_venc_v5_ue(bits, 1);	/* num_negative_pics */
		histb_venc_v5_ue(bits, 0);	/* num_positive_pics */
		histb_venc_v5_ue(bits, 0);	/* delta_poc_s0_minus1 */
		histb_venc_v5_put(bits, 1, 1);	/* used_by_curr_pic_s0_flag */
	} else {
		histb_venc_v5_ue(bits, 0);	/* num_short_term_ref_pic_sets */
	}
	histb_venc_v5_put(bits, 0, 1);		/* long_term_ref_pics_present_flag */
	histb_venc_v5_put(bits, msp ? 0 : 1, 1);	/* sps_temporal_mvp_enabled_flag */
	histb_venc_v5_put(bits, 0, 1);		/* strong_intra_smoothing_enabled */
	/*
	 * A timing-only VUI, the sequence parameter set's counterpart of the
	 * H.264 one: the clock is the picture rate itself, there is no signalling
	 * and no buffering model, so a reader learns the interval the coded rate
	 * was matched to and nothing else.
	 */
	histb_venc_v5_put(bits, 1, 1);		/* vui_parameters_present_flag */
	histb_venc_v5_put(bits, 0, 1);		/* aspect_ratio_info_present */
	histb_venc_v5_put(bits, 0, 1);		/* overscan_info_present */
	histb_venc_v5_put(bits, 0, 1);		/* video_signal_type_present */
	histb_venc_v5_put(bits, 0, 1);		/* chroma_loc_info_present */
	histb_venc_v5_put(bits, 0, 1);		/* neutral_chroma_indication */
	histb_venc_v5_put(bits, 0, 1);		/* field_seq_flag */
	histb_venc_v5_put(bits, 0, 1);		/* frame_field_info_present */
	histb_venc_v5_put(bits, 0, 1);		/* default_display_window_flag */
	histb_venc_v5_put(bits, 1, 1);		/* vui_timing_info_present */
	histb_venc_v5_put(bits, max_t(unsigned int, fps_den, 1), 32);
	histb_venc_v5_put(bits, fps_num, 32);	/* num_units_in_tick, time_scale */
	histb_venc_v5_put(bits, 0, 1);		/* vui_poc_proportional_to_timing */
	histb_venc_v5_put(bits, 0, 1);		/* vui_hrd_parameters_present */
	histb_venc_v5_put(bits, 0, 1);		/* bitstream_restriction_flag */
	histb_venc_v5_put(bits, 0, 1);		/* sps_extension_present_flag */
	histb_venc_v5_trailing(bits);
}

/*
 * One slice per picture, with the quantiser delta and the deblocking flags in
 * the slice header; the deblocking control is present and the picture-level
 * filter is left off, which the slice then overrides back on.
 */
static void histb_venc_v5_hevc_pps(struct histb_venc_v5_bits *bits)
{
	histb_venc_v5_ue(bits, 0);		/* pps_pic_parameter_set_id */
	histb_venc_v5_ue(bits, 0);		/* pps_seq_parameter_set_id */
	histb_venc_v5_put(bits, 0, 1);		/* dependent_slice_segments_enabled */
	histb_venc_v5_put(bits, 0, 1);		/* output_flag_present_flag */
	histb_venc_v5_put(bits, 0, 3);		/* num_extra_slice_header_bits */
	histb_venc_v5_put(bits, 0, 1);		/* sign_data_hiding_enabled_flag */
	histb_venc_v5_put(bits, 1, 1);		/* cabac_init_present_flag */
	histb_venc_v5_ue(bits, 0);		/* num_ref_idx_l0_default_active_minus1 */
	histb_venc_v5_ue(bits, 0);		/* num_ref_idx_l1_default_active_minus1 */
	histb_venc_v5_se(bits, 0);		/* init_qp_minus26 */
	histb_venc_v5_put(bits, 0, 1);		/* constrained_intra_pred_flag */
	histb_venc_v5_put(bits, 0, 1);		/* transform_skip_enabled_flag */
	/*
	 * The core codes no per-block quantiser delta, so the syntax must not
	 * announce one: a decoder that expects it reads the residual bits as
	 * deltas and the picture falls apart.
	 */
	histb_venc_v5_put(bits, 0, 1);		/* cu_qp_delta_enabled_flag */
	histb_venc_v5_se(bits, 0);		/* pps_cb_qp_offset */
	histb_venc_v5_se(bits, 0);		/* pps_cr_qp_offset */
	histb_venc_v5_put(bits, 0, 1);		/* pps_slice_chroma_qp_offsets */
	histb_venc_v5_put(bits, 0, 1);		/* weighted_pred_flag */
	histb_venc_v5_put(bits, 0, 1);		/* weighted_bipred_flag */
	histb_venc_v5_put(bits, 0, 1);		/* transquant_bypass_enabled_flag */
	histb_venc_v5_put(bits, 0, 1);		/* tiles_enabled_flag */
	histb_venc_v5_put(bits, 0, 1);		/* entropy_coding_sync_enabled_flag */
	histb_venc_v5_put(bits, 1, 1);		/* loop_filter_across_slices */
	histb_venc_v5_put(bits, 1, 1);		/* deblocking_filter_control_present */
	histb_venc_v5_put(bits, 1, 1);		/* deblocking_filter_override_enabled */
	histb_venc_v5_put(bits, 1, 1);		/* pps_deblocking_filter_disabled */
	histb_venc_v5_put(bits, 0, 1);		/* pps_scaling_list_data_present */
	histb_venc_v5_put(bits, 0, 1);		/* lists_modification_present_flag */
	histb_venc_v5_ue(bits, 0);		/* log2_parallel_merge_level_minus2 */
	histb_venc_v5_put(bits, 0, 1);		/* slice_segment_header_ext_present */
	histb_venc_v5_put(bits, 0, 1);		/* pps_extension_present_flag */
	histb_venc_v5_trailing(bits);
}

/*
 * Append one NAL with a four-byte start code, escaping a run of two zero bytes
 * that would otherwise read as a start code. HEVC carries a two-byte header
 * whose type sits above the layer and temporal fields; H.264 one byte holding
 * the reference index above the type. Returns the bytes written, or zero when
 * the buffer cannot hold the unit.
 */
static size_t histb_venc_v5_nal(u8 *dst, size_t room, unsigned int nal_type,
			      bool hevc, const struct histb_venc_v5_bits *bits)
{
	size_t out = 0;
	unsigned int zeros = 0;
	unsigned int i;

	if (room < 6 + bits->count / 8)
		return 0;

	dst[out++] = 0;
	dst[out++] = 0;
	dst[out++] = 0;
	dst[out++] = 1;
	if (hevc) {
		/* forbidden 0, type, layer 0, temporal id plus one. */
		dst[out++] = nal_type << 1;
		dst[out++] = 1;
	} else {
		dst[out++] = nal_type | 0x60;	/* nal_ref_idc 3 */
	}

	for (i = 0; i * 8 < bits->count; i++) {
		u8 value = bits->data[i];

		dst[out++] = value;
		if (value) {
			zeros = 0;
		} else if (++zeros == 2) {
			dst[out++] = 3;
			zeros = 0;
		}
	}

	return out;
}

/*
 * Build this picture's parameter sets into the device's parameter area, record
 * their size, and cut the bytes into the words of the command image. The core
 * does not prepend them on this part, so the driver joins these bytes in front
 * of the coded slice; the words are kept because the vendor's build fills them.
 */
int histb_venc_v5_parameter_sets(struct histb_venc_v5_frame *frame,
			       void *param_va, size_t room, u32 *words)
{
	struct histb_venc_v5_bits sps = { };
	struct histb_venc_v5_bits pps = { };
	struct histb_venc_v5_bits vps = { };
	bool hevc = frame->protocol == V500_PROTOCOL_HEVC;
	bool msp = frame->hevc_profile == V500_HEVC_PROFILE_MSP;
	unsigned int level_idc = histb_venc_v5_hevc_level_idc(frame->hevc_level);
	unsigned int first;
	size_t used = 0;

	if (hevc) {
		histb_venc_v5_hevc_vps(&vps, msp, level_idc);
		histb_venc_v5_hevc_sps(&sps, frame->width, frame->height, msp,
				     level_idc,
				     max_t(unsigned int, frame->fps_num, 1),
				     frame->fps_den);
		histb_venc_v5_hevc_pps(&pps);
	} else {
		unsigned int mbw = DIV_ROUND_UP(frame->width, 16);
		unsigned int mbh = DIV_ROUND_UP(frame->height, 16);
		unsigned int mbps = mbw * mbh * frame->fps_num /
			max_t(unsigned int, frame->fps_den, 1);
		bool direct8x8;
		u8 level;

		level = histb_venc_v5_level(mbw * mbh, mbps,
					frame->bitrate / 1000, &direct8x8);
		if (!level)
			return -E2BIG;
		histb_venc_v5_sps(&sps, frame, level, direct8x8);
		histb_venc_v5_pps(&pps, frame->cabac,
				frame->profile == V500_H264_PROFILE_HIGH);
	}
	if (vps.overflow || sps.overflow || pps.overflow)
		return -EINVAL;

	if (hevc) {
		used = histb_venc_v5_nal(param_va, room, NAL_HEVC_VPS, true, &vps);
		if (!used)
			return -ENOSPC;
	}

	first = histb_venc_v5_nal((u8 *)param_va + used, room - used,
				hevc ? NAL_HEVC_SPS : NAL_SPS, hevc, &sps);
	if (!first)
		return -ENOSPC;
	used += first;

	first = histb_venc_v5_nal((u8 *)param_va + used, room - used,
				hevc ? NAL_HEVC_PPS : NAL_PPS, hevc, &pps);
	if (!first)
		return -ENOSPC;
	used += first;

	if (used > V500_PARAM_WORDS * 4)
		return -ENOSPC;

	memset((u8 *)param_va + used, 0, room - used);
	memcpy(words, param_va, used);
	frame->param_size = used;

	return 0;
}

/*
 * The HEVC slice header in the two pieces the core takes it in: the first
 * segment, whose first-slice flag and address the core writes itself, and the
 * rest, which travels with the CABAC side. The picture order count starts over
 * at each IDR and advances by one, and the slice names it in sixteen bits.
 */
static void histb_venc_v5_hevc_slice_header(struct histb_venc_v5_frame *frame,
					  struct histb_venc_v5_slice *slice)
{
	struct histb_venc_v5_bits part1 = { };
	struct histb_venc_v5_bits cabac = { };
	u32 word;

	if (frame->intra)
		frame->poc = 0;
	else
		frame->poc = (frame->poc + 1) & 0xffff;

	if (frame->intra)
		histb_venc_v5_put(&part1, 0, 1);	/* no_output_of_prior_pics_flag */
	histb_venc_v5_ue(&part1, 0);		/* slice_pic_parameter_set_id */
	histb_venc_v5_words(&part1, &word, 1);
	slice->part1 = word << (16 - part1.count);
	slice->bits[0] = part1.count;

	histb_venc_v5_ue(&cabac, frame->intra ? 2 : 1);	/* slice_type */
	if (!frame->intra) {
		histb_venc_v5_put(&cabac, frame->poc, 16);
		histb_venc_v5_put(&cabac, 1, 1);	/* short_term_ref_pic_set_sps */
		histb_venc_v5_put(&cabac, 1, 1);	/* slice_temporal_mvp_enabled */
	}
	histb_venc_v5_put(&cabac, 1, 1);		/* slice_sao_luma_flag */
	histb_venc_v5_put(&cabac, 1, 1);		/* slice_sao_chroma_flag */
	if (!frame->intra) {
		histb_venc_v5_put(&cabac, 1, 1);	/* num_ref_idx_active_override */
		histb_venc_v5_ue(&cabac, 0);	/* num_ref_idx_l0_active_minus1 */
		histb_venc_v5_put(&cabac, 0, 1);	/* cabac_init_flag */
		/* One reference, so the collocated picture is the only l0 entry. */
		histb_venc_v5_ue(&cabac, 3);	/* five_minus_max_num_merge_cand */
	}
	histb_venc_v5_se(&cabac, (s32)frame->start_qp - 26);
	histb_venc_v5_put(&cabac, 1, 1);		/* deblocking_filter_override_flag */
	histb_venc_v5_put(&cabac, 0, 1);		/* slice_deblocking_filter_disabled */
	histb_venc_v5_se(&cabac, 0);		/* slice_beta_offset_div2 */
	histb_venc_v5_se(&cabac, 0);		/* slice_tc_offset_div2 */
	histb_venc_v5_put(&cabac, 1, 1);		/* loop_filter_across_slices */

	histb_venc_v5_words(&cabac, slice->cabac, ARRAY_SIZE(slice->cabac));
	slice->bits[1] = cabac.count;
}

/*
 * The slice-header groups of one picture. An intra picture is always an IDR
 * here, so it carries no reference-index override and its frame number is zero;
 * both counters wrap modulo sixteen, the width the four-bit field gives them.
 */
void histb_venc_v5_slice_header(struct histb_venc_v5_frame *frame,
			      struct histb_venc_v5_slice *slice)
{
	struct histb_venc_v5_bits core = { };

	if (frame->protocol == V500_PROTOCOL_HEVC) {
		histb_venc_v5_hevc_slice_header(frame, slice);
		return;
	}

	struct histb_venc_v5_bits reorder = { };
	struct histb_venc_v5_bits marking = { };

	/*
	 * frame_num names this picture in decode order, so an inter picture
	 * advances the counter before writing it; an IDR starts the count over
	 * at zero, which is also what makes the next P picture differ from it.
	 */
	if (frame->intra)
		frame->frame_num = 0;
	else
		frame->frame_num = (frame->frame_num + 1) & 0xf;

	histb_venc_v5_ue(&core, frame->intra ? 2 : 0);	/* slice_type */
	histb_venc_v5_ue(&core, 0);			/* pic_parameter_set_id */
	histb_venc_v5_put(&core, frame->intra ? 0 : frame->frame_num, 4);
	if (frame->intra)
		histb_venc_v5_ue(&core, frame->idr_pic_id);
	else
		histb_venc_v5_put(&core, 0, 1);	/* num_ref_idx override off */

	/* RefPicListReordering: the single flag, saying there is none. */
	histb_venc_v5_put(&reorder, 0, 1);

	/* DecRefPicMarking: two flags for an IDR, one for the rest. */
	histb_venc_v5_put(&marking, 0, frame->intra ? 2 : 1);

	histb_venc_v5_words(&core, slice->header, ARRAY_SIZE(slice->header));
	histb_venc_v5_words(&reorder, slice->reorder,
			  ARRAY_SIZE(slice->reorder));
	histb_venc_v5_words(&marking, slice->marking,
			  ARRAY_SIZE(slice->marking));
	slice->bits[0] = core.count | reorder.count << 8 |
			 marking.count << 16;

	if (frame->intra)
		frame->idr_pic_id = (frame->idr_pic_id + 1) & 0xf;
}
