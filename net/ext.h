/* SPDX-License-Identifier: GPL-3.0-or-later */
/*
 * Driver-side decisions for the AmiNetXDuo SANA-II extension (anxs2ext.h),
 * kept free of Amiga headers so the host tests run this exact code.
 * device.c checks the mirrored constants against anxs2ext.h.
 *
 * Firmware registers consulted:
 *   ETH_RX_FRAMES (0x6C)  bit 15 = present, bits 14..0 = frames a sender
 *                         may put on the wire at once; older firmware reads
 *                         0 and has a 32-frame receive ring
 *   ETH_RX_META (0xA6)    for the frame presented in the RX window, valid
 *                         from its header read until its ack: bit 15 =
 *                         receive checksum engine on, bits 1..0 = GEM
 *                         verdict (0 none, 1 IP header, 2 IP+TCP,
 *                         3 IP+UDP); the GEM discards frames it found bad.
 *                         Bit 14 = transmit checksum insertion on: a zeroed
 *                         TCP/UDP checksum field of an IPv4 frame is filled
 *                         in by the GEM.
 */
#ifndef ZZNET_EXT_H
#define ZZNET_EXT_H

#include <stdint.h>

#define ZZNET_EXT_VERSION       3
#define ZZNET_EXT_MIN_SIZE      24   /* ANXD_S2_EXTENSION_SIZE on m68k */

/* ANXD_S2F_* */
#define ZZNET_EXT_RX_DIRECT     (1UL << 0)
#define ZZNET_EXT_RX_LINK_HDR   (1UL << 1)
#define ZZNET_EXT_RX_VERIFIED   (1UL << 2)
#define ZZNET_EXT_TX_CSUM_TCP   (1UL << 3)
#define ZZNET_EXT_TX_CSUM_UDP   (1UL << 4)
#define ZZNET_EXT_RX_POLL       (1UL << 5)
#define ZZNET_EXT_RX_CAPACITY   (1UL << 6)
#define ZZNET_EXT_TX_QUICK      (1UL << 7)
#define ZZNET_EXT_RX_BATCH      (1UL << 8)
#define ZZNET_EXT_TX_MORE       (1UL << 9)

/* ANXD_S2_RXF_* and ANXD_S2_TXF_* */
#define ZZNET_EXT_RXF_SUMMED    0x01
#define ZZNET_EXT_RXF_VERIFIED  0x02
#define ZZNET_EXT_TXF_TCP       0x01
#define ZZNET_EXT_TXF_UDP       0x02
#define ZZNET_EXT_TXF_MORE      0x40

/* ANXD_S2_RX_BATCH_VERSION, ANXD_S2_RX_BATCH_MAX */
#define ZZNET_EXT_BATCH_VERSION 1
#define ZZNET_EXT_BATCH_MAX     32

#define ZZNET_RX_META           0xA6
#define ZZNET_RX_META_PRESENT   0x8000
#define ZZNET_RX_META_TX_CSUM   0x4000
#define ZZNET_RX_META_VERDICT   0x0003
#define ZZNET_RX_META_TCP       2
#define ZZNET_RX_META_UDP       3

#define ZZNET_RX_FRAMES         0x6C
#define ZZNET_RX_FRAMES_PRESENT 0x8000
#define ZZNET_RX_FRAMES_COUNT   0x7fff
#define ZZNET_RX_FRAMES_LEGACY  32

/* What the driver can offer on this card, from ETH_RX_META read at init.
 * tx_async: the asynchronous TX path is present, so a start can be held
 * (TX_MORE). */
uint32_t zznet_ext_offer(uint16_t rx_meta, int tx_async);

/* What an opener supplied besides Request. */
#define ZZNET_EXT_HAVE_RX_CB    0x01  /* RxDirect and RxFilled */
#define ZZNET_EXT_HAVE_TX_CB    0x02  /* TxFlags */
#define ZZNET_EXT_HAVE_COPYTO   0x04  /* S2_CopyToBuff */

/* One ANXD_S2_EXTENSION record. Returns 0 for a record the driver must
 * ignore (unknown Version, short Size), else 1 with *accepted set to the
 * request intersected with the offer, less what the opener cannot use:
 * the receive features need both receive callbacks and RX_DIRECT; a batch
 * also needs the link header and CopyToBuff; TX checksum and MORE need
 * TxFlags. */
int zznet_ext_accept(uint16_t version, uint16_t size, uint32_t request,
                     uint32_t offer, unsigned have, uint32_t *accepted);

/* ANXD_CMD_RX_CAPACITY answer in bytes from ETH_RX_FRAMES. */
uint32_t zznet_ext_rx_capacity(uint16_t rx_frames);

/* An ANXD_CMD_RX_BATCH record header: 1 when the driver may fill it. */
int zznet_ext_batch_ok(uint16_t version, uint16_t size, uint16_t count,
                       unsigned cookie_size);

/* Fold a 32-bit ones-complement accumulator to 16 bits. */
uint16_t zznet_ext_fold(uint32_t acc);

/* ANXD_S2_RXF_VERIFIED or 0 for an IPv4 payload of `len` bytes in RAM at
 * `ip`, given the GEM verdict (ETH_RX_META bits 1..0, 2 TCP, 3 UDP). The
 * verdict covers the checksums; this checks the structural promises the
 * flag makes: no IP options, not a fragment, total length equal to len,
 * the protocol the verdict names with a complete header, UDP with its
 * length matching and a checksum present. */
uint8_t zznet_ext_rx_trust4(const uint8_t *ip, uint32_t len,
                            unsigned verdict);

/* The same flag from software: `sum` is the uncomplemented ones-complement
 * sum of the `len` payload bytes (any 32-bit accumulation of it). */
uint8_t zznet_ext_rx_verify4(const uint8_t *ip, uint32_t len, uint32_t sum);
uint8_t zznet_ext_rx_verify6(const uint8_t *ip, uint32_t len, uint32_t sum);

/* For a whole Ethernet frame of `len` bytes (header first; only the first
 * bytes need be present, up to the transport checksum field): the offset
 * of the TCP or UDP checksum field the GEM is to fill, when the frame is
 * IPv4, not a fragment, its total length fits the frame and its transport
 * header is complete, and the protocol is in `allowed` (ZZNET_EXT_TXF_*).
 * Returns the ZZNET_EXT_TXF_* flag, or 0 for a frame to send as written. */
uint8_t zznet_ext_tx_csum4(const uint8_t *frame, uint32_t have, uint32_t len,
                           uint8_t allowed, uint16_t *offset);

#endif /* ZZNET_EXT_H */
