/*
 * p2p_encryption.h
 *
 *  Created on: Aug 27, 2025
 *      Author: Dominik
 */

#ifndef INC_P2P_ENCRYPTION_H_
#define INC_P2P_ENCRYPTION_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DEVICE_ADDRESS	0x12345678
#define F_PORT	1

extern const uint8_t nwk_s_key[16];
extern const uint8_t app_s_key[16];

/* ==== Build-time options (tune to your PHY frame budget) ==== */
#ifndef P2PENC_INCLUDE_DEVADDR
#define P2PENC_INCLUDE_DEVADDR 0 /* 1: prepend 4B DevAddr (LE); 0: omit */
#endif
#ifndef P2PENC_INCLUDE_FPORT
#define P2PENC_INCLUDE_FPORT 0 /* 1: include 1B FPort; 0: omit */
#endif
#ifndef P2PENC_ALIGN_FRMPAYLOAD_16
#define P2PENC_ALIGN_FRMPAYLOAD_16 0 /* 1: cap encryptable len to 16B mult */
#endif

#define P2PENC_DIR_UPLINK 0u
#define P2PENC_DIR_DOWNLINK 1u

#define P2PENC_MIC_LEN 4u

/* Error codes */
#define P2PENC_OK 0
#define P2PENC_ERR_PARAMS -1
#define P2PENC_ERR_BUFFER -2
#define P2PENC_ERR_CRYPTO -3
#define P2PENC_ERR_MIC -4

/* Context: addressing + keys (LoRaWAN-analogous naming) */
typedef struct
{
	uint32_t devaddr; /* host-endian */
	uint8_t dir; /* 0 uplink / 1 downlink */
	uint8_t nwkskey[16]; /* CMAC key for MIC */
	uint8_t appskey[16]; /* CTR payload key (ECB on Ai blocks) */
} p2penc_ctx_t;

/* Initialize context */
void p2penc_init(p2penc_ctx_t *ctx, uint32_t devaddr, uint8_t dir,
		const uint8_t nwkskey[16], const uint8_t appskey[16]);

/* Header+MIC overhead with current compile-time flags */
size_t p2penc_overhead_bytes(void);

/* Given max frame segment (e.g., radio FIFO budget), return max encryptable bytes */
size_t p2penc_max_encryptable(size_t max_segment_bytes);

/* High-level API: build P2P frame (header + ciphertext + MIC) within size cap. */
int p2penc_build_frame(const p2penc_ctx_t *ctx, uint32_t fcnt, uint8_t fport,
		const uint8_t *in_payload, size_t in_len, size_t max_segment_bytes,
		uint8_t *out_frame, size_t *out_len, size_t *used_payload_len);

/* Parse + verify MIC + decrypt. Outputs plaintext, fcnt, fport. */
int p2penc_parse_and_decrypt(const p2penc_ctx_t *ctx, const uint8_t *frame,
		size_t frame_len, uint8_t *out_plain, size_t *out_plain_len,
		uint32_t *out_fcnt, uint8_t *out_fport);

/* Low-level helpers (LoRa-style CTR and CMAC) */
int p2penc_ctr_crypt(const p2penc_ctx_t *ctx, uint32_t fcnt, const uint8_t *in,
		uint8_t *out, size_t len);

int p2penc_compute_mic(const p2penc_ctx_t *ctx, uint32_t fcnt,
		const uint8_t *msg, size_t msg_len, uint8_t out_mic[P2PENC_MIC_LEN]);

#ifdef __cplusplus
}
#endif

#endif /* INC_P2P_ENCRYPTION_H_ */
