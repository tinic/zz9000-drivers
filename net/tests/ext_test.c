/* SPDX-License-Identifier: GPL-3.0-or-later */
/*
 * Host coverage for the AmiNetXDuo SANA-II extension decisions: record
 * negotiation, the RX capacity answer, RX batch records, the VERIFIED
 * receive flag from the GEM verdict and from software, and the TX
 * checksum field the GEM is to fill. Build/run:
 * make -C net/tests test
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ext.h"

#define CHECK(expr) do { \
	if (!(expr)) { \
		fprintf(stderr, "%s:%d: check failed: %s\n", \
		        __FILE__, __LINE__, #expr); \
		return EXIT_FAILURE; \
	} \
} while (0)

#define ALL 0xffffffffu
#define HAVE_ALL (ZZNET_EXT_HAVE_RX_CB | ZZNET_EXT_HAVE_TX_CB | \
                  ZZNET_EXT_HAVE_COPYTO)
#define RX_ALL (ZZNET_EXT_RX_DIRECT | ZZNET_EXT_RX_LINK_HDR | \
                ZZNET_EXT_RX_VERIFIED | ZZNET_EXT_RX_BATCH)
#define TX_CSUM (ZZNET_EXT_TX_CSUM_TCP | ZZNET_EXT_TX_CSUM_UDP)

static uint32_t sum16(const uint8_t *p, uint32_t n)
{
	uint32_t acc = 0, i;

	for (i = 0; i + 1 < n; i += 2)
		acc += (uint32_t)(p[i] << 8 | p[i + 1]);
	if (n & 1)
		acc += (uint32_t)p[n - 1] << 8;
	return acc;
}

static void put16(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v >> 8);
	p[1] = (uint8_t)v;
}

/* An IPv4 packet with correct header and transport checksums. */
static uint32_t make_ip4(uint8_t *p, uint8_t proto, uint32_t data)
{
	uint32_t thl = proto == 6 ? 20 : 8;
	uint32_t len = 20 + thl + data, i, acc;

	memset(p, 0, 1600);
	p[0] = 0x45;
	put16(p + 2, len);
	p[6] = 0x40;
	p[8] = 64;
	p[9] = proto;
	p[12] = 10; p[15] = 1; p[16] = 10; p[19] = 2;
	put16(p + 10, (uint16_t)~zznet_ext_fold(sum16(p, 20)));
	for (i = 0; i < data; i++)
		p[20 + thl + i] = (uint8_t)(i * 7 + 3);
	if (proto == 6)
		p[32] = 0x50;
	else
		put16(p + 24, thl + data);
	acc = sum16(p + 20, thl + data) + sum16(p + 12, 8) + proto + thl + data;
	put16(p + 20 + (proto == 6 ? 16 : 6), (uint16_t)~zznet_ext_fold(acc));
	return len;
}

static uint32_t make_ip6(uint8_t *p, uint8_t nh, uint32_t data)
{
	uint32_t thl = nh == 6 ? 20 : 8;
	uint32_t tlen = thl + data, i, acc;

	memset(p, 0, 1600);
	p[0] = 0x60;
	put16(p + 4, tlen);
	p[6] = nh;
	p[7] = 64;
	for (i = 8; i < 40; i++)
		p[i] = (uint8_t)i;
	for (i = 0; i < data; i++)
		p[40 + thl + i] = (uint8_t)(i * 5 + 1);
	if (nh == 6)
		p[52] = 0x50;
	else
		put16(p + 44, tlen);
	acc = sum16(p + 40, tlen) + sum16(p + 8, 32) + nh + tlen;
	put16(p + 40 + (nh == 6 ? 16 : 6), (uint16_t)~zznet_ext_fold(acc));
	return 40 + tlen;
}

int main(void)
{
	uint32_t a = 0xdeadbeef;
	uint32_t offer;
	static uint8_t ip[1600], fr[1600];
	uint32_t len;
	uint16_t off = 0;

	/* Offer. */
	offer = zznet_ext_offer(0, 0);
	CHECK((offer & RX_ALL) == RX_ALL);
	CHECK(offer & ZZNET_EXT_RX_POLL);
	CHECK(offer & ZZNET_EXT_RX_CAPACITY);
	CHECK(offer & ZZNET_EXT_TX_QUICK);
	CHECK(!(offer & (TX_CSUM | ZZNET_EXT_TX_MORE)));
	CHECK((zznet_ext_offer(ZZNET_RX_META_TX_CSUM, 0) & TX_CSUM) == TX_CSUM);
	CHECK(zznet_ext_offer(0, 1) & ZZNET_EXT_TX_MORE);
	offer = zznet_ext_offer(ZZNET_RX_META_TX_CSUM, 1);

	/* Only Version 3 with a full record is valid. */
	CHECK(!zznet_ext_accept(2, 24, ALL, offer, HAVE_ALL, &a));
	CHECK(!zznet_ext_accept(4, 24, ALL, offer, HAVE_ALL, &a));
	CHECK(!zznet_ext_accept(3, 23, ALL, offer, HAVE_ALL, &a));
	CHECK(a == 0xdeadbeef);

	/* Intersection; a longer record is fine. */
	CHECK(zznet_ext_accept(3, 32, ALL, offer, HAVE_ALL, &a));
	CHECK(a == offer);
	CHECK(zznet_ext_accept(3, 24, ZZNET_EXT_RX_CAPACITY, offer, HAVE_ALL, &a));
	CHECK(a == ZZNET_EXT_RX_CAPACITY);
	CHECK(zznet_ext_accept(3, 24, 0, offer, 0, &a));
	CHECK(a == 0);

	/* Receive features need both callbacks and RX_DIRECT. */
	CHECK(zznet_ext_accept(3, 24, ALL, offer,
	                       ZZNET_EXT_HAVE_TX_CB | ZZNET_EXT_HAVE_COPYTO, &a));
	CHECK(!(a & RX_ALL));
	CHECK(zznet_ext_accept(3, 24, ALL & ~ZZNET_EXT_RX_DIRECT, offer,
	                       HAVE_ALL, &a));
	CHECK(!(a & RX_ALL));

	/* A batch also needs the link header and CopyToBuff. */
	CHECK(zznet_ext_accept(3, 24, ALL & ~ZZNET_EXT_RX_LINK_HDR, offer,
	                       HAVE_ALL, &a));
	CHECK(!(a & ZZNET_EXT_RX_BATCH) && (a & ZZNET_EXT_RX_DIRECT));
	CHECK(zznet_ext_accept(3, 24, ALL, offer,
	                       ZZNET_EXT_HAVE_RX_CB | ZZNET_EXT_HAVE_TX_CB, &a));
	CHECK(!(a & ZZNET_EXT_RX_BATCH) && (a & ZZNET_EXT_RX_LINK_HDR));

	/* TX checksum and MORE need TxFlags. */
	CHECK(zznet_ext_accept(3, 24, ALL, offer,
	                       ZZNET_EXT_HAVE_RX_CB | ZZNET_EXT_HAVE_COPYTO, &a));
	CHECK(!(a & (TX_CSUM | ZZNET_EXT_TX_MORE)));
	CHECK(a & ZZNET_EXT_RX_BATCH);

	/* Capacity: register absent or zero is the 32-frame ring. */
	CHECK(zznet_ext_rx_capacity(0) == 32 * 1514);
	CHECK(zznet_ext_rx_capacity(56) == 32 * 1514);
	CHECK(zznet_ext_rx_capacity(ZZNET_RX_FRAMES_PRESENT) == 32 * 1514);
	CHECK(zznet_ext_rx_capacity(ZZNET_RX_FRAMES_PRESENT | 56) == 56 * 1514);

	/* Batch records: version 1, 1..32 cookies, Size covering them. */
	CHECK(zznet_ext_batch_ok(1, 8 + 4 * 4, 4, 4));
	CHECK(!zznet_ext_batch_ok(1, 8 + 4 * 3, 4, 4));
	CHECK(!zznet_ext_batch_ok(2, 8 + 4 * 4, 4, 4));
	CHECK(!zznet_ext_batch_ok(1, 8, 0, 4));
	CHECK(zznet_ext_batch_ok(1, 8 + 32 * 4, 32, 4));
	CHECK(!zznet_ext_batch_ok(1, 8 + 33 * 4, 33, 4));

	/* VERIFIED from the GEM verdict: structure only. */
	len = make_ip4(ip, 6, 100);
	CHECK(zznet_ext_rx_trust4(ip, len, ZZNET_RX_META_TCP) == ZZNET_EXT_RXF_VERIFIED);
	CHECK(zznet_ext_rx_trust4(ip, len, ZZNET_RX_META_UDP) == 0);
	CHECK(zznet_ext_rx_trust4(ip, len, 1) == 0);
	CHECK(zznet_ext_rx_trust4(ip, len + 6, ZZNET_RX_META_TCP) == 0);   /* padding */
	ip[0] = 0x46;
	CHECK(zznet_ext_rx_trust4(ip, len, ZZNET_RX_META_TCP) == 0);       /* options */
	len = make_ip4(ip, 6, 100);
	ip[6] = 0x20;
	CHECK(zznet_ext_rx_trust4(ip, len, ZZNET_RX_META_TCP) == 0);       /* MF */
	len = make_ip4(ip, 17, 33);
	CHECK(zznet_ext_rx_trust4(ip, len, ZZNET_RX_META_UDP) == ZZNET_EXT_RXF_VERIFIED);
	put16(ip + 24, 40);
	CHECK(zznet_ext_rx_trust4(ip, len, ZZNET_RX_META_UDP) == 0);       /* UDP length */
	len = make_ip4(ip, 17, 33);
	ip[26] = ip[27] = 0;
	CHECK(zznet_ext_rx_trust4(ip, len, ZZNET_RX_META_UDP) == 0);       /* no csum */

	/* VERIFIED from the copy's sum. */
	len = make_ip4(ip, 6, 101);
	CHECK(zznet_ext_rx_verify4(ip, len, sum16(ip, len)) == ZZNET_EXT_RXF_VERIFIED);
	ip[60] ^= 1;
	CHECK(zznet_ext_rx_verify4(ip, len, sum16(ip, len)) == 0);
	len = make_ip4(ip, 17, 7);
	CHECK(zznet_ext_rx_verify4(ip, len, sum16(ip, len)) == ZZNET_EXT_RXF_VERIFIED);
	ip[10] ^= 1;                                                     /* IP header */
	CHECK(zznet_ext_rx_verify4(ip, len, sum16(ip, len)) == 0);
	len = make_ip6(ip, 6, 64);
	CHECK(zznet_ext_rx_verify6(ip, len, sum16(ip, len)) == ZZNET_EXT_RXF_VERIFIED);
	ip[8] ^= 0x10;
	CHECK(zznet_ext_rx_verify6(ip, len, sum16(ip, len)) == 0);
	len = make_ip6(ip, 17, 9);
	CHECK(zznet_ext_rx_verify6(ip, len, sum16(ip, len)) == ZZNET_EXT_RXF_VERIFIED);
	CHECK(zznet_ext_rx_verify6(ip, len + 2, sum16(ip, len)) == 0);    /* padding */
	ip[46] = ip[47] = 0;
	CHECK(zznet_ext_rx_verify6(ip, len, sum16(ip, len)) == 0);

	/* TX checksum field. */
	memset(fr, 0, sizeof(fr));
	fr[12] = 0x08;
	len = make_ip4(ip, 6, 50);
	memcpy(fr + 14, ip, len);
	CHECK(zznet_ext_tx_csum4(fr, 94, 14 + len, ZZNET_EXT_TXF_TCP, &off) == ZZNET_EXT_TXF_TCP);
	CHECK(off == 14 + 20 + 16);
	CHECK(zznet_ext_tx_csum4(fr, 94, 14 + len, ZZNET_EXT_TXF_UDP, &off) == 0);
	CHECK(zznet_ext_tx_csum4(fr, 40, 14 + len, ZZNET_EXT_TXF_TCP, &off) == 0);
	fr[14] = 0x46;                                                   /* options */
	CHECK(zznet_ext_tx_csum4(fr, 94, 14 + len, ZZNET_EXT_TXF_TCP, &off) == ZZNET_EXT_TXF_TCP);
	CHECK(off == 14 + 24 + 16);
	fr[14] = 0x45;
	fr[20] = 0x20;                                                   /* MF */
	CHECK(zznet_ext_tx_csum4(fr, 94, 14 + len, ZZNET_EXT_TXF_TCP, &off) == 0);
	len = make_ip4(ip, 17, 50);
	memcpy(fr + 14, ip, len);
	CHECK(zznet_ext_tx_csum4(fr, 94, 14 + len, ZZNET_EXT_TXF_UDP | ZZNET_EXT_TXF_TCP, &off) == ZZNET_EXT_TXF_UDP);
	CHECK(off == 14 + 20 + 6);
	CHECK(zznet_ext_tx_csum4(fr, 94, 14 + len - 1, ZZNET_EXT_TXF_UDP, &off) == 0);
	fr[13] = 0xdd;
	CHECK(zznet_ext_tx_csum4(fr, 94, 14 + len, ZZNET_EXT_TXF_UDP, &off) == 0);

	printf("ext_test: ok\n");
	return EXIT_SUCCESS;
}
