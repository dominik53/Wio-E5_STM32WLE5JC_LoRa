/*
 * p2p_encryption.c
 *
 *  Created on: Aug 27, 2025
 *      Author: Dominik
 */

#include "p2p_encryption.h"
#include <string.h>
#include "cmox_crypto.h" /* ST CMOX public API (mac & cipher one-shot) */


const uint8_t nwk_s_key[16] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90, 0x00, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60};
const uint8_t app_s_key[16] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90, 0x00, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60};


/* ====== Small CMOX one-time init guard (follows ST example init flow) ====== */
static int s_cmox_inited = 0;
static void p2penc_ensure_cmox_init(void)
{
	if (!s_cmox_inited)
	{
		cmox_init_arg_t init =
		{ CMOX_INIT_TARGET_AUTO, NULL };
		(void) cmox_initialize(&init); /* per ST example pattern *//* fileciteturn0file0 */
		s_cmox_inited = 1;
	}
}

/* ==== Utils: LE writes ==== */
static inline void wr32_le(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t) v;
	p[1] = (uint8_t) (v >> 8);
	p[2] = (uint8_t) (v >> 16);
	p[3] = (uint8_t) (v >> 24);
}

/* Build B0 (MIC) and Ai (CTR) as per LoRaWAN v1.0.x */
static void build_b0(uint8_t b0[16], uint8_t dir, uint32_t devaddr,
		uint32_t fcnt, uint8_t len_msg)
{
	memset(b0, 0, 16);
	b0[0] = 0x49; /* 0x49 for data MIC */
	b0[5] = (uint8_t) (dir & 1);
	wr32_le(&b0[6], devaddr);
	wr32_le(&b0[10], fcnt);
	b0[15] = len_msg; /* length of msg (<=255) */
}

static void build_ai(uint8_t ai[16], uint8_t dir, uint32_t devaddr,
		uint32_t fcnt, uint8_t i)
{
	memset(ai, 0, 16);
	ai[0] = 0x01;
	ai[5] = (uint8_t) (dir & 1);
	wr32_le(&ai[6], devaddr);
	wr32_le(&ai[10], fcnt);
	ai[15] = i; /* 1..N */
}

void p2penc_init(p2penc_ctx_t *ctx, uint32_t devaddr, uint8_t dir,
		const uint8_t nwkskey[16], const uint8_t appskey[16])
{
	if (!ctx)
		return;
	memset(ctx, 0, sizeof(*ctx));
	ctx->devaddr = devaddr;
	ctx->dir = dir ? 1u : 0u;
	if (nwkskey)
		memcpy(ctx->nwkskey, nwkskey, 16);
	if (appskey)
		memcpy(ctx->appskey, appskey, 16);
}

size_t p2penc_overhead_bytes(void)
{
	size_t ov = P2PENC_MIC_LEN + 4 /* FCnt */;
#if P2PENC_INCLUDE_DEVADDR
	ov += 4;
#endif
#if P2PENC_INCLUDE_FPORT
	ov += 1;
#endif
	return ov;
}

size_t p2penc_max_encryptable(size_t max_segment_bytes)
{
	size_t ov = p2penc_overhead_bytes();
	if (max_segment_bytes <= ov)
		return 0;
	size_t avail = max_segment_bytes - ov;
#if P2PENC_ALIGN_FRMPAYLOAD_16
avail = (avail/16u)*16u;
#endif
	return avail;
}

/* === AES-ECB one-shot helper (CTR uses ECB on Ai blocks) === */
static int aes128_ecb_encrypt_block(const uint8_t key[16], const uint8_t in[16],
		uint8_t out[16])
{
	p2penc_ensure_cmox_init();
	size_t out_len = 0;
	cmox_cipher_retval_t ret = cmox_cipher_encrypt(
	CMOX_AESFAST_ECB_ENC_ALGO, /* Default AES implementation via meta-define, CMOX_AES_ECB_ENC_ALGO */
	in, 16, /* PT (one block) */
	key, 16, /* 128-bit key */
	NULL, 0, /* no IV in ECB */
	out, &out_len /* CT (one block) */
	); /* API shape mirrors ST AES-CBC example usage. fileciteturn0file1 */
	return (ret == CMOX_CIPHER_SUCCESS && out_len == 16) ? P2PENC_OK : P2PENC_ERR_CRYPTO;
}

int p2penc_ctr_crypt(const p2penc_ctx_t *ctx, uint32_t fcnt, const uint8_t *in,
		uint8_t *out, size_t len)
{
	if (!ctx || (!in && len) || (!out && len))
		return P2PENC_ERR_PARAMS;
	uint8_t ai[16], ks[16];
	uint32_t blk = 1;
	size_t off = 0;
	while (off < len)
	{
		size_t chunk = (len - off > 16) ? 16 : (len - off);
		build_ai(ai, ctx->dir, ctx->devaddr, fcnt, (uint8_t) blk);
		if (aes128_ecb_encrypt_block(ctx->appskey, ai, ks) != P2PENC_OK)
			return P2PENC_ERR_CRYPTO;
		for (size_t i = 0; i < chunk; ++i)
			out[off + i] = in[off + i] ^ ks[i];
		off += chunk;
		blk++;
	}
	return P2PENC_OK;
}

int p2penc_compute_mic(const p2penc_ctx_t *ctx, uint32_t fcnt,
		const uint8_t *msg, size_t msg_len, uint8_t out_mic[P2PENC_MIC_LEN])
{
	if (!ctx || !msg || !out_mic)
		return P2PENC_ERR_PARAMS;
	if (msg_len > 255)
		return P2PENC_ERR_PARAMS; /* LoRa B0 len field is 1 byte */

	p2penc_ensure_cmox_init();

	uint8_t b0[16];
	build_b0(b0, ctx->dir, ctx->devaddr, fcnt, (uint8_t) msg_len);

	/* Concatenate B0 || msg into temp buffer (<= 271 bytes) */
	uint8_t buf[16 + 255];
	memcpy(buf, b0, 16);
	memcpy(buf + 16, msg, msg_len);

	uint8_t tag16[16];
	size_t tag_len = 0;
	/* One-shot CMAC, per ST example API (cmox_mac_compute). fileciteturn0file0 */
	cmox_mac_retval_t ret = cmox_mac_compute(
	CMOX_CMAC_AES_ALGO, /* AES-CMAC (default impl) */
	buf, 16 + msg_len, /* message (B0||msg) */
	ctx->nwkskey, 16, /* key */
	NULL, 0, /* custom data (unused) */
	tag16, sizeof(tag16), /* full 16B tag */
	&tag_len);
	if (ret != CMOX_MAC_SUCCESS || tag_len != sizeof(tag16))
		return P2PENC_ERR_CRYPTO;
	memcpy(out_mic, tag16, P2PENC_MIC_LEN);
	return P2PENC_OK;
}

int p2penc_build_frame(const p2penc_ctx_t *ctx, uint32_t fcnt, uint8_t fport,
		const uint8_t *in_payload, size_t in_len, size_t max_segment_bytes,
		uint8_t *out_frame, size_t *out_len, size_t *used_payload_len)
{
	if (!ctx || !out_frame || !out_len)
		return P2PENC_ERR_PARAMS;

	size_t max_frm = p2penc_max_encryptable(max_segment_bytes);
	size_t take = (in_len < max_frm) ? in_len : max_frm;

	size_t off = 0;
#if P2PENC_INCLUDE_DEVADDR
	wr32_le(&out_frame[off], ctx->devaddr); off += 4;
#endif
	wr32_le(&out_frame[off], fcnt);
	off += 4;
#if P2PENC_INCLUDE_FPORT
	out_frame[off++] = fport;
#endif

	/* Encrypt payload into frame buffer */
	int rc = p2penc_ctr_crypt(ctx, fcnt, in_payload, &out_frame[off], take);
	if (rc != P2PENC_OK)
		return rc;
	size_t ct_len = take;

	/* MIC over header + ciphertext */
	uint8_t mic[P2PENC_MIC_LEN];
	rc = p2penc_compute_mic(ctx, fcnt, out_frame, off + ct_len, mic);
	if (rc != P2PENC_OK)
		return rc;

	memcpy(&out_frame[off + ct_len], mic, P2PENC_MIC_LEN);
	*out_len = off + ct_len + P2PENC_MIC_LEN;
	if (used_payload_len)
		*used_payload_len = take;
	return P2PENC_OK;
}

int p2penc_parse_and_decrypt(const p2penc_ctx_t *ctx, const uint8_t *frame,
		size_t frame_len, uint8_t *out_plain, size_t *out_plain_len,
		uint32_t *out_fcnt, uint8_t *out_fport)
{
	if (!ctx || !frame || frame_len < p2penc_overhead_bytes())
		return P2PENC_ERR_PARAMS;

	size_t off = 0;
#if P2PENC_INCLUDE_DEVADDR
	uint32_t devaddr_rx = (uint32_t)frame[0] | ((uint32_t)frame[1]<<8) | ((uint32_t)frame[2]<<16) | ((uint32_t)frame[3]<<24);
	off += 4; (void)devaddr_rx;
#endif
	uint32_t fcnt_rx = (uint32_t) frame[off] | ((uint32_t) frame[off + 1] << 8)
			| ((uint32_t) frame[off + 2] << 16)
			| ((uint32_t) frame[off + 3] << 24);
	off += 4;
	uint8_t fport_rx = 0;
#if P2PENC_INCLUDE_FPORT
	fport_rx = frame[off++];
#endif

	if (frame_len < off + P2PENC_MIC_LEN)
		return P2PENC_ERR_BUFFER;

	size_t ct_len = frame_len - off - P2PENC_MIC_LEN;
	const uint8_t *ct = &frame[off];
	const uint8_t *mic = &frame[off + ct_len];

	uint8_t mic_calc[P2PENC_MIC_LEN];
	int rc = p2penc_compute_mic(ctx, fcnt_rx, frame, off + ct_len, mic_calc);
	if (rc != P2PENC_OK)
		return rc;
	if (memcmp(mic, mic_calc, P2PENC_MIC_LEN) != 0)
		return P2PENC_ERR_MIC;

	if (!out_plain || !out_plain_len || *out_plain_len < ct_len)
		return P2PENC_ERR_BUFFER;

	rc = p2penc_ctr_crypt(ctx, fcnt_rx, ct, out_plain, ct_len);
	if (rc != P2PENC_OK)
		return rc;

	*out_plain_len = ct_len;
	if (out_fcnt)
		*out_fcnt = fcnt_rx;
	if (out_fport)
		*out_fport = fport_rx;
	return P2PENC_OK;
}

