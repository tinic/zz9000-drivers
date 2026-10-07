/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "ext.h"

static uint16_t be16(const uint8_t *p)
{
	return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

uint32_t zznet_ext_offer(uint16_t rx_meta, int tx_async)
{
	uint32_t offer = ZZNET_EXT_RX_DIRECT | ZZNET_EXT_RX_LINK_HDR |
	                 ZZNET_EXT_RX_VERIFIED | ZZNET_EXT_RX_POLL |
	                 ZZNET_EXT_RX_CAPACITY | ZZNET_EXT_TX_QUICK |
	                 ZZNET_EXT_RX_BATCH;

	/* VERIFIED needs no firmware help: without the GEM verdict the
	 * driver verifies from the sum it takes while copying. */
	if (rx_meta & ZZNET_RX_META_TX_CSUM)
		offer |= ZZNET_EXT_TX_CSUM_TCP | ZZNET_EXT_TX_CSUM_UDP;
	if (tx_async)
		offer |= ZZNET_EXT_TX_MORE;
	return offer;
}

int zznet_ext_accept(uint16_t version, uint16_t size, uint32_t request,
                     uint32_t offer, unsigned have, uint32_t *accepted)
{
	uint32_t a;

	if (version != ZZNET_EXT_VERSION || size < ZZNET_EXT_MIN_SIZE)
		return 0;
	a = request & offer;
	if (!(have & ZZNET_EXT_HAVE_RX_CB) || !(a & ZZNET_EXT_RX_DIRECT))
		a &= ~(ZZNET_EXT_RX_DIRECT | ZZNET_EXT_RX_LINK_HDR |
		       ZZNET_EXT_RX_VERIFIED | ZZNET_EXT_RX_BATCH);
	if (!(a & ZZNET_EXT_RX_LINK_HDR) || !(have & ZZNET_EXT_HAVE_COPYTO))
		a &= ~ZZNET_EXT_RX_BATCH;
	if (!(have & ZZNET_EXT_HAVE_TX_CB))
		a &= ~(ZZNET_EXT_TX_CSUM_TCP | ZZNET_EXT_TX_CSUM_UDP |
		       ZZNET_EXT_TX_MORE);
	*accepted = a;
	return 1;
}

uint32_t zznet_ext_rx_capacity(uint16_t rx_frames)
{
	uint32_t frames = rx_frames & ZZNET_RX_FRAMES_COUNT;

	if (!(rx_frames & ZZNET_RX_FRAMES_PRESENT) || frames == 0)
		frames = ZZNET_RX_FRAMES_LEGACY;
	return frames * 1514;
}

int zznet_ext_batch_ok(uint16_t version, uint16_t size, uint16_t count,
                       unsigned cookie_size)
{
	return version == ZZNET_EXT_BATCH_VERSION && count != 0 &&
	       count <= ZZNET_EXT_BATCH_MAX &&
	       (uint32_t)size >= 8u + (uint32_t)count * cookie_size;
}

uint16_t zznet_ext_fold(uint32_t acc)
{
	acc = (acc & 0xffff) + (acc >> 16);
	acc = (acc & 0xffff) + (acc >> 16);
	return (uint16_t)acc;
}

/* IPv4 without options, unfragmented, length exact, TCP or UDP with a
 * complete header; for UDP the length matches and a checksum is present.
 * Returns the protocol or 0. */
static unsigned ip4_shape(const uint8_t *ip, uint32_t len)
{
	uint32_t tlen;

	if (len < 20 || ip[0] != 0x45 || be16(ip + 2) != len)
		return 0;
	if ((ip[6] & 0x3f) != 0 || ip[7] != 0)
		return 0;
	tlen = len - 20;
	if (ip[9] == 6)
		return tlen >= 20 ? 6 : 0;
	if (ip[9] == 17)
		return (tlen >= 8 && be16(ip + 24) == tlen &&
		        be16(ip + 26) != 0) ? 17 : 0;
	return 0;
}

uint8_t zznet_ext_rx_trust4(const uint8_t *ip, uint32_t len,
                            unsigned verdict)
{
	unsigned proto = ip4_shape(ip, len);

	if ((verdict == ZZNET_RX_META_TCP && proto == 6) ||
	    (verdict == ZZNET_RX_META_UDP && proto == 17))
		return ZZNET_EXT_RXF_VERIFIED;
	return 0;
}

uint8_t zznet_ext_rx_verify4(const uint8_t *ip, uint32_t len, uint32_t sum)
{
	unsigned proto = ip4_shape(ip, len);
	uint32_t acc = 0;
	unsigned i;

	if (!proto)
		return 0;
	for (i = 0; i < 20; i += 2)
		acc += be16(ip + i);
	if (zznet_ext_fold(acc) != 0xffff)
		return 0;
	/* The payload sum includes the IP header, which sums to 0xffff, a
	 * ones-complement zero; adding the pseudo-header leaves the
	 * transport sum, which is 0xffff when its checksum is right. */
	acc = zznet_ext_fold(sum);
	acc += be16(ip + 12) + be16(ip + 14) + be16(ip + 16) + be16(ip + 18);
	acc += proto + (len - 20);
	return zznet_ext_fold(acc) == 0xffff ? ZZNET_EXT_RXF_VERIFIED : 0;
}

uint8_t zznet_ext_rx_verify6(const uint8_t *ip, uint32_t len, uint32_t sum)
{
	uint32_t tlen, acc;
	unsigned nh;

	if (len < 40 || (ip[0] >> 4) != 6)
		return 0;
	tlen = be16(ip + 4);
	if (tlen + 40 != len)
		return 0;
	nh = ip[6];
	if (nh == 17) {
		if (tlen < 8 || be16(ip + 44) != tlen || be16(ip + 46) == 0)
			return 0;
	} else if (nh != 6 || tlen < 20) {
		return 0;
	}
	/* The payload sum covers the fixed header too; take its first eight
	 * bytes back out, leaving the addresses for the pseudo-header. */
	acc = zznet_ext_fold(sum);
	acc += (uint16_t)~zznet_ext_fold((uint32_t)be16(ip) + be16(ip + 2) +
	                                 be16(ip + 4) + be16(ip + 6));
	acc += tlen + nh;
	return zznet_ext_fold(acc) == 0xffff ? ZZNET_EXT_RXF_VERIFIED : 0;
}

uint8_t zznet_ext_tx_csum4(const uint8_t *frame, uint32_t have, uint32_t len,
                           uint8_t allowed, uint16_t *offset)
{
	const uint8_t *ip = frame + 14;
	uint32_t ihl, total, tlen, off;
	uint8_t flag;

	if (have > len)
		have = len;
	if (have < 34 || frame[12] != 0x08 || frame[13] != 0x00 ||
	    (ip[0] & 0xf0) != 0x40)
		return 0;
	ihl = (uint32_t)(ip[0] & 0x0f) << 2;
	total = be16(ip + 2);
	if (ihl < 20 || ihl > len - 14 || total < ihl || total > len - 14 ||
	    (ip[6] & 0x3f) != 0 || ip[7] != 0)
		return 0;
	tlen = total - ihl;
	if (ip[9] == 6 && (allowed & ZZNET_EXT_TXF_TCP)) {
		if (tlen < 20 || 14 + ihl + 13 > have || (ip[ihl + 12] >> 4) < 5)
			return 0;
		off = 14 + ihl + 16;
		flag = ZZNET_EXT_TXF_TCP;
	} else if (ip[9] == 17 && (allowed & ZZNET_EXT_TXF_UDP)) {
		if (tlen < 8 || 14 + ihl + 6 > have || be16(ip + ihl + 4) != tlen)
			return 0;
		off = 14 + ihl + 6;
		flag = ZZNET_EXT_TXF_UDP;
	} else {
		return 0;
	}
	*offset = (uint16_t)off;
	return flag;
}
