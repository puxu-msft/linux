// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon HiSTB V500 encoder: command image and launch.
 *
 * A picture is described by one 4096 byte image in DMA memory, packed field by
 * field here, plus the register writes that point at it and start the core. The
 * ABI note carries the offsets with their sources; the fields this generation
 * does not use are left at the zero the image is allocated with, as the vendor
 * build leaves them for an intra picture.
 */

#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/slab.h>

#include "histb_venc_v5.h"

/*
 * The image is little endian and word addressed, as the core reads it with the
 * same word order the vendor's hardware layer stored it with.
 */
static void histb_venc_v5_word(void *image, unsigned int offset, u32 value)
{
	((__le32 *)image)[offset / 4] = cpu_to_le32(value);
}

/* Image offsets, as the core's own layout names them. */
/* Some regions below are named by their word index; IMGW turns one into
 * the byte offset the writer takes.
 */
#define IMGW(x)			((x) * 4)

#define IMG_GEOMETRY		0x000
#define IMG_SIZE		0x004
#define IMG_SEARCH_RANGE	0x014
#define IMG_SLICE_SPLIT		0x008
#define IMG_QP			0x00c
#define IMG_LAMBDA_SSD		0xad
#define IMG_LAMBDA_SAD		0xd5
#define IMG_RC_QP			0x270
#define IMG_RC_TARGET		0x274
#define IMG_STORE_FMT		0x3b	/* word index; the writer takes bytes */
#define IMG_SRC_Y		0x068
#define IMG_SRC_C		0x06c
#define IMG_SRC_V		0x070
#define IMG_SRC_STRIDES		0x07c
#define IMG_RECON_Y		0x080
#define IMG_RECON_C		0x084
#define IMG_RECON_STRIDES	0x088
#define IMG_REF_RECON_Y		0x098
#define IMG_REF_RECON_C		0x09c
#define IMG_REF_RECON_STRIDES	0x0a0
#define IMG_STREAM_BASE		0x0e0
#define IMG_STREAM_WRITE	0x0e4
#define IMG_STREAM_READ		0x0e8
#define IMG_STREAM_SIZES	0x14f
#define IMG_STREAM_ADDRS	0x15f
#define IMG_PARAM_ADDR		IMGW(0x16f)
#define IMG_PARAM_GATE		IMGW(0x170)
#define IMG_PARAM_WORDS		0x171
#define IMG_PICTURE_TYPE		0x125
#define IMG_SLICE_BITS		0x126
#define IMG_SLICE_HEADERS		0x13a
#define IMG_SLICE_REORDER	0x13e
#define IMG_SLICE_MARKING	0x140
#define IMG_SLICE_COUNTS		0x142
#define IMG_PME_INFO			0x16
#define IMG_TUNNEL_CELL		0x064
#define IMG_PME_OTHER		0x0b0
#define IMG_PME_CURRENT		0x0d0
#define IMG_TMV_CURRENT		0x0d8
#define IMG_TMV_OTHER		0x0dc


/*
 * The V500R001 lambda defaults for H.264: 40 SSD then 40 SAD words, indexed by
 * the quantiser. The core costs its mode decision with these, so an image that
 * leaves them zero has no quantiser-dependent decision to make.
 */
static const u32 v500_lambda_ssd[V500_LAMBDA_WORDS] = {
	1, 1, 1, 2, 2, 3, 3, 4,
	5, 7, 9, 11, 14, 17, 22, 27,
	34, 43, 54, 69, 86, 109, 137, 173,
	218, 274, 345, 435, 548, 691, 870, 1097,
	1382, 1741, 2193, 2763, 3482, 4095, 4095, 4095,
};

static const u32 v500_lambda_sad[V500_LAMBDA_WORDS] = {
	1, 1, 1, 1, 2, 2, 2, 2,
	3, 3, 3, 4, 4, 4, 5, 6,
	6, 7, 8, 9, 10, 11, 13, 14,
	16, 18, 20, 23, 25, 29, 32, 36,
	40, 45, 51, 57, 64, 72, 81, 91,
};

/* The same two tables for HEVC, whose quantiser costs start higher. */
static const u32 v500_lambda_ssd_hevc[V500_LAMBDA_WORDS] = {
	18, 22, 28, 36, 45, 57, 72, 91,
	115, 145, 183, 231, 291, 367, 463, 583,
	735, 926, 1167, 1470, 1853, 2334, 2941, 3706,
	4669, 5883, 7412, 9338, 11766, 14824, 18677, 23532,
	29649, 37355, 47065, 59298, 74711, 94130, 118596, 131071,
};

static const u32 v500_lambda_sad_hevc[V500_LAMBDA_WORDS] = {
	193, 216, 243, 273, 306, 344, 386, 433,
	487, 546, 613, 688, 773, 867, 974, 1093,
	1227, 1377, 1546, 1735, 1948, 2186, 2454, 2755,
	3092, 3471, 3896, 4373, 4908, 5510, 6184, 6942,
	7792, 8746, 9817, 11020, 12369, 13884, 15584, 17493,
};



/*
 * Words the vendor's hardware layer writes with the same value for every
 * H.264 picture: read out of the reference backend and confirmed by
 * sweeping size, quantiser, picture type and buffer layout, so only the
 * words that move with the picture are packed below. Their fields are not
 * attributed, so they are replayed exactly as the reference writes them.
 */
static const struct {
	u16 word;
	u32 value;
} v500_image_defaults[] = {
	{ 0x0005, 0x00000003 },
	{ 0x0006, 0x00000001 },
	{ 0x0007, 0x00007715 },
	{ 0x0008, 0x00000003 },
	{ 0x0009, 0x00000013 },
	{ 0x000a, 0x006401f4 },
	{ 0x000b, 0x05050f0f },
	{ 0x000c, 0x02020000 },
	{ 0x000d, 0x0000000f },
	{ 0x000e, 0x00000002 },
	{ 0x000f, 0x00000003 },
	{ 0x0014, 0x000f001f },
	{ 0x003b, 0x00010000 },
	{ 0x0053, 0x00030080 },
	{ 0x0056, 0x00030040 },
	{ 0x0057, 0x00000001 },
	{ 0x0059, 0x00040010 },
	{ 0x005a, 0x00000001 },
	{ 0x005c, 0x00040010 },
	{ 0x005d, 0x00000001 },
	{ 0x005f, 0x00040010 },
	{ 0x0060, 0x00000001 },
	{ 0x0072, 0x00000100 },
	{ 0x0079, 0xff00ff00 },
	{ 0x007a, 0x00000001 },
	{ 0x007b, 0x00000014 },
	{ 0x007c, 0x00000032 },
	{ 0x007d, 0x00000096 },
	{ 0x007f, 0x00301004 },
	{ 0x0080, 0x000c0804 },
	{ 0x0081, 0x0041e246 },
	{ 0x0083, 0x0f0f0505 },
	{ 0x0084, 0x83a06483 },
	{ 0x0085, 0x0000000a },
	{ 0x0086, 0x000002ff },
	{ 0x0087, 0x00063220 },
	{ 0x0088, 0x00017a03 },
	{ 0x0089, 0x00800308 },
	{ 0x008a, 0x00001026 },
	{ 0x008b, 0x00080008 },
	{ 0x008c, 0x00080008 },
	{ 0x008d, 0x00100010 },
	{ 0x008e, 0x00180018 },
	{ 0x0094, 0x80800000 },
	{ 0x0095, 0x003c0500 },
	{ 0x0096, 0x00140000 },
	{ 0x009e, 0x00000a20 },
	{ 0x009f, 0x00002c0a },
	{ 0x00a0, 0x0000300a },
	{ 0x00a1, 0x0000300a },
	{ 0x00a2, 0x0000300a },
	{ 0x00a3, 0x0002330a },
	{ 0x00a4, 0x00003300 },
	{ 0x00a5, 0x00843300 },
	{ 0x00a7, 0x07070700 },
	{ 0x00a8, 0x120e0b09 },
	{ 0x00a9, 0xffffff19 },
	{ 0x00aa, 0x55555555 },
	{ 0x00ab, 0x00000008 },
	{ 0x00fe, 0x00044005 },
	{ 0x0104, 0x00000002 },
	{ 0x0105, 0x000f000f },
	{ 0x0106, 0x000003c9 },
	{ 0x0110, 0xffffffff },
	{ 0x0111, 0xffffffff },
	{ 0x0112, 0x00000007 },
	{ 0x0113, 0xffffffff },
	{ 0x0114, 0x00000007 },
	{ 0x0115, 0x0000000f },
	{ 0x011c, 0x00000001 },
	{ 0x011d, 0x000055ab },
	{ 0x014c, 0x007f3f3f },
	{ 0x014e, 0x003d0900 },
};

int histb_venc_v5_buffers_alloc(struct histb_venc_v5 *enc)
{
	struct device *dev = enc->dev;
	void *va;

	/*
	 * The image holds the bus addresses of everything else as plain words,
	 * so the whole set has to sit below 4 GiB; the probe installs a 32-bit
	 * DMA mask for that reason and these allocations honour it.
	 */
	va = dma_alloc_coherent(dev, V500_IMAGE_BYTES, &enc->image.dma,
				 GFP_KERNEL);
	if (!va)
		return -ENOMEM;

	enc->image.va = va;
	enc->image.size = V500_IMAGE_BYTES;
	memset(va, 0, V500_IMAGE_BYTES);

	va = dma_alloc_coherent(dev, V500_STREAM_BYTES, &enc->stream.dma,
				GFP_KERNEL);
	if (!va)
		goto free_image;

	enc->stream.va = va;
	enc->stream.size = V500_STREAM_BYTES;

	/*
	 * Two reconstruction buffers: the core writes the picture it codes and
	 * reads back the one before it, and the picture it uses is selected by
	 * an index in the image rather than by a second pair of addresses.
	 */
	va = dma_alloc_coherent(dev, V500_RECON_BYTES * V500_RECON_BUFFERS,
				&enc->recon.dma, GFP_KERNEL);
	if (!va)
		goto free_stream;

	enc->recon.va = va;
	enc->recon.size = V500_RECON_BYTES * V500_RECON_BUFFERS;

	va = dma_alloc_coherent(dev, V500_PARAM_BYTES, &enc->param.dma,
				GFP_KERNEL);
	if (!va)
		goto free_recon;

	enc->param.va = va;
	enc->param.size = V500_PARAM_BYTES;

	va = dma_alloc_coherent(dev, V500_SLICE_BYTES, &enc->slice.dma,
				GFP_KERNEL);
	if (!va)
		goto free_param;

	enc->slice.va = va;
	enc->slice.size = V500_SLICE_BYTES;

	/*
	 * The core is handed addresses for the inter-picture areas even when the
	 * picture is intra, so those get a real area rather than zero.
	 */
	va = dma_alloc_coherent(dev, V500_SCRATCH_BYTES, &enc->scratch.dma,
				GFP_KERNEL);
	if (!va)
		goto free_slice;

	enc->scratch.va = va;
	enc->scratch.size = V500_SCRATCH_BYTES;

	return 0;

free_slice:
	dma_free_coherent(dev, enc->slice.size, enc->slice.va, enc->slice.dma);
free_param:
	dma_free_coherent(dev, enc->param.size, enc->param.va, enc->param.dma);
free_recon:
	dma_free_coherent(dev, enc->recon.size, enc->recon.va,
			  enc->recon.dma);
free_stream:
	dma_free_coherent(dev, enc->stream.size, enc->stream.va,
			  enc->stream.dma);
free_image:
	dma_free_coherent(dev, enc->image.size, enc->image.va,
			  enc->image.dma);
	return -ENOMEM;
}

void histb_venc_v5_buffers_free(struct histb_venc_v5 *enc)
{
	struct device *dev = enc->dev;

	dma_free_coherent(dev, enc->slice.size, enc->slice.va, enc->slice.dma);
	dma_free_coherent(dev, enc->param.size, enc->param.va, enc->param.dma);
	dma_free_coherent(dev, enc->scratch.size, enc->scratch.va,
			  enc->scratch.dma);
	dma_free_coherent(dev, enc->recon.size, enc->recon.va, enc->recon.dma);
	dma_free_coherent(dev, enc->stream.size, enc->stream.va,
			  enc->stream.dma);
	dma_free_coherent(dev, enc->image.size, enc->image.va, enc->image.dma);
}

/*
 * Fill the image for one picture. The geometry reaches the core twice: once as
 * the pixel size, once as macroblock counts, and the counts are what the units
 * that read the reconstruction work from.
 */
void histb_venc_v5_image_pack(struct histb_venc_v5 *enc,
			    const struct histb_venc_v5_frame *frame,
			    const struct histb_venc_v5_slice *slice)
{
	unsigned int i;
	unsigned int coded = histb_venc_v5_coded_height(frame);
	bool hevc = frame->protocol == V500_PROTOCOL_HEVC;

	void *image = enc->image.va;
	u32 word;

	for (i = 0; i < ARRAY_SIZE(v500_image_defaults); i++)
		histb_venc_v5_word(image,
			 IMGW(v500_image_defaults[i].word),
			 v500_image_defaults[i].value);

	/*
	 * Two of the vendor's fixed words move with the protocol: the motion
	 * search window, and the low bits of the decision word that the search
	 * depth rides in. The vendor's packing gives High and HEVC depth 7 and
	 * every other H.264 profile 5. The second H.264 slice-header set is not
	 * read at all for HEVC.
	 */
	histb_venc_v5_word(image, IMGW(0x0007),
			 hevc || frame->profile == V500_H264_PROFILE_HIGH ?
			 0x00007717 : 0x00007715);
	histb_venc_v5_word(image, IMGW(IMG_SEARCH_RANGE),
			 hevc ? 0x0013001b : 0x000f001f);
	if (hevc) {
		/*
		 * The three decision words HEVC codes with: the vendor swaps
		 * its per-size quantiser costs for the HEVC set, and the last
		 * of them saturates.
		 */
		histb_venc_v5_word(image, IMGW(0x0087), 0x00063260);
		histb_venc_v5_word(image, IMGW(0x00a7), 0x05050303);
		histb_venc_v5_word(image, IMGW(0x00a8), 0xff140f0a);
		histb_venc_v5_word(image, IMGW(0x00a9), 0xffffffff);
		histb_venc_v5_word(image, IMGW(0x014c), 0);
	}

	/*
	 * The core takes the coded size in macroblock units from this word as
	 * well; the reference build leaves it zero, and the core then stops
	 * coding inside the first macroblock row.
	 */
	if (!hevc) {
		word = FIELD_PREP(GENMASK(24, 16),
				  DIV_ROUND_UP(frame->height, 16) - 1) |
		       FIELD_PREP(GENMASK(8, 0),
				  DIV_ROUND_UP(frame->width, 16) - 1);
		histb_venc_v5_word(image, IMG_GEOMETRY, word);
	}

	word = FIELD_PREP(GENMASK(28, 16), frame->height - 1) |
	       FIELD_PREP(GENMASK(12, 0), frame->width - 1);
	histb_venc_v5_word(image, IMG_SIZE, word);

	/*
	 * Slice splitting: the enable sits above a sixteen bit size, and bit 16
	 * of that word is set by the vendor's packing even when splitting is
	 * off.
	 */
	word = FIELD_PREP(BIT(17), frame->slice_split ? 1 : 0) |
	       BIT(16) |
	       FIELD_PREP(GENMASK(15, 0), frame->slice_split_size);
	histb_venc_v5_word(image, IMG_SLICE_SPLIT, word);

	word = FIELD_PREP(GENMASK(15, 10), frame->start_qp);
	histb_venc_v5_word(image, IMG_QP, word);

	/*
	 * The rate-control bounds go in twice: the picture's own quantiser window
	 * above, and here as the limits the search is allowed to move between,
	 * with the per-macroblock bit target that the rate controller works from.
	 */
	histb_venc_v5_word(image, IMG_RC_QP,
			 FIELD_PREP(GENMASK(13, 8), frame->max_qp) |
			 FIELD_PREP(GENMASK(5, 0), frame->min_qp));
	/* Rate control splits the picture by coding tree, 32 pixels for HEVC. */
	word = frame->target_bits / histb_venc_v5_block_rows(frame);
	histb_venc_v5_word(image, IMG_RC_TARGET,
			 FIELD_PREP(GENMASK(30, 6), word) | BIT(1));

	for (i = 0; i < V500_LAMBDA_WORDS; i++) {
		histb_venc_v5_word(image, IMGW(IMG_LAMBDA_SSD + i),
			 (hevc ? v500_lambda_ssd_hevc[i] : v500_lambda_ssd[i]) &
			 GENMASK(19, 0));
		histb_venc_v5_word(image, IMGW(IMG_LAMBDA_SAD + i),
			 (hevc ? v500_lambda_sad_hevc[i] : v500_lambda_sad[i]) &
			 GENMASK(19, 0));
	}

	/*
	 * The input layout in the vendor's own encoding. A semi-planar picture
	 * only ever names which sample of the interleaved pair comes first, in
	 * one bit; the other two layouts carry the whole order byte there, and
	 * YUYV is the value 0xd8 in it.
	 */
	word = FIELD_PREP(GENMASK(9, 6), frame->store);
	if (frame->store == V500_STORE_SEMI_PLANAR)
		word |= FIELD_PREP(BIT(16), frame->package == 0);
	else
		word |= FIELD_PREP(GENMASK(23, 16), frame->package);
	histb_venc_v5_word(image, IMGW(IMG_STORE_FMT), word);

	histb_venc_v5_word(image, IMG_SRC_Y, frame->src_y);
	histb_venc_v5_word(image, IMG_SRC_C, frame->src_c);
	/* Only a planar input has a third plane to point at. */
	histb_venc_v5_word(image, IMG_SRC_V, frame->src_v);

	/*
	 * The source strides put the luma one high; the reconstruction strides
	 * put it low, which is how the vendor's own packing writes the two
	 * words.
	 */
	word = FIELD_PREP(GENMASK(31, 16), frame->src_stride_y) |
	       FIELD_PREP(GENMASK(15, 0), frame->src_stride_c);
	histb_venc_v5_word(image, IMG_SRC_STRIDES, word);

	/*
	 * The chroma plane follows the luma one by the core's own coded height,
	 * which is the negotiated one rounded up to its block grid.
	 */
	histb_venc_v5_word(image, IMG_RECON_Y, frame->recon[frame->recon_index]);
	histb_venc_v5_word(image, IMG_RECON_C,
			 frame->recon[frame->recon_index] +
			 frame->recon_stride_y * coded);

	word = FIELD_PREP(GENMASK(15, 0), frame->recon_stride_y) |
	       FIELD_PREP(GENMASK(31, 16), frame->recon_stride_c);
	histb_venc_v5_word(image, IMG_RECON_STRIDES, word);

	/*
	 * The picture the core predicts from: the other reconstruction slot, with
	 * its own address pair and its stride pair, whose halves sit in the
	 * opposite order to the pair above. The chroma plane of a reconstruction
	 * slot always starts at the same coded height, so the reference follows
	 * the same offset the picture being written was given.
	 */
	histb_venc_v5_word(image, IMG_REF_RECON_Y,
			 frame->recon[1 - frame->recon_index]);
	histb_venc_v5_word(image, IMG_REF_RECON_C,
			 frame->recon[1 - frame->recon_index] +
			 frame->recon_stride_y * coded);
	word = FIELD_PREP(GENMASK(31, 16), frame->recon_stride_y) |
	       FIELD_PREP(GENMASK(15, 0), frame->recon_stride_c);
	histb_venc_v5_word(image, IMG_REF_RECON_STRIDES, word);

	histb_venc_v5_word(image, IMG_STREAM_BASE, frame->stream[0]);
	histb_venc_v5_word(image, IMG_STREAM_WRITE, frame->stream_write);
	histb_venc_v5_word(image, IMG_STREAM_READ, frame->stream_read);

	/*
	 * The inter-picture areas are read by rotation: the picture being coded
	 * uses the entry the index names, the one it looks back at the other.
	 */
	histb_venc_v5_word(image, IMGW(IMG_PME_INFO),
		 frame->pme_info[frame->recon_index]);
	histb_venc_v5_word(image, IMGW(IMG_PME_INFO + 1),
		 frame->pme_info[(frame->recon_index + 2) % 3]);
	histb_venc_v5_word(image, IMGW(IMG_PME_INFO + 2),
		 frame->pme_info[(frame->recon_index + 1) % 3]);
	histb_venc_v5_word(image, IMG_TUNNEL_CELL, frame->tunnel_cell);
	histb_venc_v5_word(image, IMG_PME_CURRENT,
			 frame->pme[frame->recon_index]);
	histb_venc_v5_word(image, IMG_PME_OTHER,
			 frame->pme[1 - frame->recon_index]);
	histb_venc_v5_word(image, IMG_TMV_CURRENT,
			 frame->tmv[frame->recon_index]);
	histb_venc_v5_word(image, IMG_TMV_OTHER,
			 frame->tmv[1 - frame->recon_index]);

	for (i = 0; i < V500_STREAM_SLOTS; i++) {
		histb_venc_v5_word(image, IMGW(IMG_STREAM_SIZES + i),
				 frame->stream_size[i]);
		histb_venc_v5_word(image, IMGW(IMG_STREAM_ADDRS + i),
				 frame->stream[i]);
	}

	/*
	 * The parameter sets go in twice: the address of the area holding their
	 * bytes, and the bytes themselves as words, so the core can prepend them
	 * without reading the area at all. The gate says which of the two this
	 * picture wants and how many bytes are worth taking.
	 */
	histb_venc_v5_word(image, IMG_PARAM_ADDR, frame->param);
	/*
	 * Only the constant bit goes here: with the core-side prepend armed, a
	 * launch stops reporting anything at all on this part -- no interrupt, a
	 * status word that stays zero -- while the driver joining the parameter
	 * sets in front of the coded slice is what the header mode the node
	 * advertises means. The address and the words stay written either way,
	 * as the vendor's build fills them too.
	 */
	histb_venc_v5_word(image, IMG_PARAM_GATE, BIT(8));
	if (frame->parameter_set) {
		for (i = 0; i < V500_PARAM_WORDS; i++)
			histb_venc_v5_word(image,
					 IMGW(IMG_PARAM_WORDS + i),
					 frame->param_reg[i]);
	}

	histb_venc_v5_word(image, IMGW(IMG_PICTURE_TYPE),
		 frame->intra ? V500_PICTURE_INTRA : V500_PICTURE_INTER);

	/*
	 * The two bit counts travel in one word either way: the first segment
	 * for HEVC, the whole header for H.264. What follows differs: H.264
	 * hands over three groups of words, HEVC the first segment and then the
	 * CABAC-side rest, each word top-aligned on the header's bit position.
	 */
	histb_venc_v5_word(image, IMGW(IMG_SLICE_BITS),
			 FIELD_PREP(GENMASK(4, 0), slice->bits[0] & 0x1f) |
			 FIELD_PREP(GENMASK(21, 16), slice->bits[1]));

	if (hevc) {
		/*
		 * HEVC's header words sit straight after the count word: the
		 * first segment, then the CABAC-side rest eight words wide.
		 */
		histb_venc_v5_word(image, IMGW(IMG_SLICE_BITS + 1), slice->part1);
		for (i = 0; i < ARRAY_SIZE(slice->cabac); i++) {
			u32 value = slice->cabac[i];
			int room = (int)((i + 1) * 32) - (int)slice->bits[1];

			if (slice->bits[1] <= i * 32 || room >= 32)
				value = 0;
			else if (room > 0)
				value <<= room;
			histb_venc_v5_word(image,
					 IMGW(IMG_SLICE_BITS + 2 + i), value);
		}
	} else {
		/*
		 * Each of the three groups gets its bit count here, one below
		 * the count, with the packed word's own low field taking the
		 * whole triple less one.
		 */
		word = slice->bits[0];
		histb_venc_v5_word(image, IMGW(IMG_SLICE_COUNTS),
			 ((word - 1) & 0x7f) << 16 |
			 (((word >> 8) - 1) & 0x3f) << 8 |
			 (((word >> 16) - 1) & 0x3f));

		for (i = 0; i < 4; i++)
			histb_venc_v5_word(image, IMGW(IMG_SLICE_HEADERS + i),
					 slice->header[i]);
		for (i = 0; i < 2; i++) {
			histb_venc_v5_word(image, IMGW(IMG_SLICE_REORDER + i),
					 slice->reorder[i]);
			histb_venc_v5_word(image, IMGW(IMG_SLICE_MARKING + i),
					 slice->marking[i]);
		}
	}
}

/*
 * Arm the core. The block also needs its own control word, not just the image:
 * the mode bits that say a picture is coming from memory, that this is
 * H.264 in the entropy arrangement the driver's parameter sets declare, and
 * the per-macroblock bit target the rate controller divides its decisions by.
 * Fields the vendor's build fills from its tuning table are left at the reset
 * value, and the ABI note says which those are.
 */

/*
 * Register words the vendor's layer writes with a fixed value for this
 * protocol; the reference backend emits them for every picture and
 * their fields are not attributed. Section 10 of the ABI note lists
 * where the values were read from.
 */
static const struct {
	u16 offset;
	u32 value;
} v500_fixed_regs[] = {
	{ V500_TUNE_WORD(0x06), 0x02625a00 },
	{ V500_TUNE_WORD(0x0d), 0x00370006 },
	{ V500_TUNE_WORD(0x0e), 0x0000007f },
	{ V500_TUNE_WORD(0x20), 0x00010001 },
	{ V500_TUNE_WORD(0x94), 0x00000003 },
};

void histb_venc_v5_launch(struct histb_venc_v5 *enc,
			const struct histb_venc_v5_frame *frame)
{
	void __iomem *base = enc->base;
	bool hevc = frame->protocol == V500_PROTOCOL_HEVC;
	unsigned int mbs = histb_venc_v5_blocks(frame);
	unsigned int i;
	u32 ctrl;

	writel(V500_INT_CLEAR_ALL, base + V500_INT_CLEAR);
	writel(V500_INT_ENABLE_DEFAULT, base + V500_INT_ENABLE);

	/*
	 * The protocol lives in its own bit and clears the three words that only
	 * mean something to H.264: the profile constraint pair, the entropy
	 * arrangement and the negative "not HEVC" marker itself.
	 */
	ctrl = V500_CORE_ALWAYS | V500_CORE_DDR_MODE;
	if (hevc) {
		ctrl |= V500_CORE_HEVC;
	} else {
		ctrl |= V500_CORE_NOT_HEVC;
		if (frame->profile != V500_H264_PROFILE_HIGH)
			ctrl |= FIELD_PREP(V500_CORE_NOT_HIGH, 1);
		if (!frame->cabac)
			ctrl |= V500_CORE_CAVLC;
	}
	ctrl |= frame->intra ? V500_CORE_INTRA : V500_CORE_INTER;
	if (frame->low_latency)
		ctrl |= V500_CORE_LOW_DELAY;

	writel(ctrl, base + V500_CORE_CTRL);
	writel(V500_SEARCH_CFG_DEFAULT, base + V500_SEARCH_CFG);
	for (i = 0; i < ARRAY_SIZE(v500_fixed_regs); i++)
		writel(v500_fixed_regs[i].value,
			 base + v500_fixed_regs[i].offset);
	writel((frame->target_bits / mbs) & GENMASK(19, 0),
	       base + V500_MB_TARGET_BITS);

	writel(lower_32_bits(enc->image.dma), base + V500_CMD_ADDR);
	writel(1, base + V500_START);
}

unsigned int histb_venc_v5_status(struct histb_venc_v5 *enc)
{
	return readl(enc->base + V500_STATUS);
}

/*
 * The vendor layer has no stop path, and the block's reset request is the one
 * bit that costs the neighbouring decoder its launches. Mask the lines, drop
 * the status and release the start bit so the next picture starts from a known
 * interrupt state; whether the engine itself drains is not settled here.
 */
void histb_venc_v5_stop_core(struct histb_venc_v5 *enc)
{
	writel(0, enc->base + V500_INT_ENABLE);
	writel(V500_INT_CLEAR_ALL, enc->base + V500_INT_CLEAR);
	writel(0, enc->base + V500_START);
}

/* Code length of the finished picture, in bits. */
/*
 * The counter holds bytes, not bits: the vendor's read-back multiplies it by
 * eight to get the picture's length in bits, and taking the register value as
 * bits hands back an eighth of the picture.
 */
unsigned int histb_venc_v5_pic_bits(struct histb_venc_v5 *enc)
{
	void __iomem *reg = enc->protocol == V500_PROTOCOL_HEVC ?
		enc->base + V500_PIC_BITS_HEVC : enc->base + V500_PIC_BITS_H264;

	return readl(reg) << 3;
}

/* Stream slot geometry: the ring is cut into the vendor's slice buffers. */
unsigned int histb_venc_v5_slot_stride(struct histb_venc_v5 *enc)
{
	unsigned int ring = enc->stream.size - V500_RING_OFFSET -
			    V500_SLICE_BUFFERS * 64;

	return ALIGN_DOWN(ring / V500_SLICE_BUFFERS, 64);
}

/*
 * Copy the bytes the picture wrote into the ring. Without splitting the length
 * is the count the core reports in its own register, which is where the vendor
 * reads it from. With splitting each slice sits in its own slot and reports its
 * own length; the copy walks the slots in slice order and stops at the one the
 * core marks last.
 */
/*
 * Four sample words of each reconstruction slot, printed around a launch. The
 * core's own output layout is what the reference picture has to agree with, and
 * the slots are the only place that question can be settled: the sample offsets
 * name the luma start, the luma middle, the plane boundary and the chroma.
 */
ssize_t histb_venc_v5_drain(struct histb_venc_v5 *enc,
			  const struct histb_venc_v5_frame *frame,
			  void *dst, size_t size)
{
	unsigned int stride = histb_venc_v5_slot_stride(enc);
	unsigned int slot;
	size_t total = 0;

	if (!frame->slice_split) {
		unsigned int bits = histb_venc_v5_pic_bits(enc);
		size_t bytes = (bits + 7) / 8;

		if (!bytes)
			return -EIO;

		if (bytes > size ||
		    bytes > enc->stream.size - V500_RING_OFFSET) {
			dev_err(enc->dev, "picture wrote %zu bytes, room is %zu\n",
				bytes, size);
			return -ENOSPC;
		}

		memcpy(dst, enc->stream.va + V500_RING_OFFSET, bytes);

		return bytes;
	}

	for (slot = 0; slot < V500_SLICE_BUFFERS; slot++) {
		unsigned int first = readl(enc->base + V500_SLICE_BITS(slot));
		unsigned int second = readl(enc->base + V500_SLICE_ADJUST(slot));
		unsigned int bytes = (first & V500_SLICE_BITS_MASK) -
				     (second & V500_SLICE_ADJUST_MASK);

		if (!bytes)
			break;
		if (bytes > stride || total + bytes > size) {
			dev_err(enc->dev,
				"slice %u wrote %u bytes, room is %zu\n",
				slot, bytes, size - total);
			return -ENOSPC;
		}

		memcpy(dst + total,
		       enc->stream.va + V500_RING_OFFSET + slot * stride, bytes);
		total += bytes;

		if (second & V500_SLICE_LAST)
			break;
	}

	return total ? total : -EIO;
}
