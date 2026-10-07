/*
 * MNT ZZ9000 Network Driver (ZZ9000Net.device)
 * Copyright (C) 2016-2026, Lucie L. Hartmann <lucie@mntre.com>
 *                          MNT Research GmbH, Berlin
 *                          https://mntre.com
 *
 * Based on code copyright (C) 2018 Henryk Richter <henryk.richter@gmx.net>
 * Released under GPLv3+ with permission.
 *
 * 2026 GCC port, bug fixes and performance work:
 *   Copyright (C) 2026, Dimitris Panokostas <midwan@gmail.com>
 *
 * More Info: https://mntre.com/zz9000
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * GNU General Public License v3.0 or later
 *
 * https://spdx.org/licenses/GPL-3.0-or-later.html
 */

#define DEVICE_MAIN

#include <proto/exec.h>
#include <proto/utility.h>
#include <proto/dos.h>
#include <proto/expansion.h>
#include <proto/timer.h>
#include <clib/exec_protos.h>
#include <clib/alib_protos.h>
#include <dos/dostags.h>
#include <utility/tagitem.h>
#include <exec/lists.h>
#include <exec/errors.h>
#include <exec/interrupts.h>
#include <exec/tasks.h>
#include <exec/execbase.h>
#include <hardware/intbits.h>
#include <string.h>
#include "sana2.h"

#ifdef HAVE_VERSION_H
#include "version.h"
#endif
/* NSD support is optional */
#ifdef NEWSTYLE
#include <devices/newstyle.h>
#endif /* NEWSTYLE */
#ifdef DEVICES_NEWSTYLE_H

const UWORD dev_supportedcmds[] = {
	NSCMD_DEVICEQUERY,
	CMD_READ,
	CMD_WRITE,
	S2_ADDMULTICASTADDRESS,
	S2_DELMULTICASTADDRESS,
	S2_MULTICAST,
	/* ... add all cmds here that are supported by BeginIO */
	0
};

const struct NSDeviceQueryResult NSDQueryAnswer = {
	0,
	16, /* up to SupportedCommands (inclusive) TODO: correct number */
	NSDEVTYPE_SANA2, /* TODO: proper device type */
	0,  /* subtype */
	(UWORD*)dev_supportedcmds
};
#endif /* DEVICES_NEWSTYLE_H */

/* Before device.h: its SysBase/ExpansionBase macros would rewrite the
 * library-base names in this header's helpers. */
#include "zz9000_hw.h"
#include "device.h"
#include "zzcfg_query.h"
#include "macros.h"
#include "rx.h"
#include "tx.h"
#include "ext.h"

#if ZZNET_S2ERR_NO_RESOURCES != S2ERR_NO_RESOURCES || \
    ZZNET_S2ERR_BAD_STATE != S2ERR_BAD_STATE || \
    ZZNET_S2ERR_BAD_ADDRESS != S2ERR_BAD_ADDRESS || \
    ZZNET_S2ERR_NOT_SUPPORTED != S2ERR_NOT_SUPPORTED || \
    ZZNET_RXF_MCAST != SANA2IOF_MCAST || \
    ZZNET_RXF_BCAST != SANA2IOF_BCAST || \
    ZZNET_MCAST_ADDR_LEN != HW_ADDRFIELDSIZE
#error multicast model drifted from the SANA-II driver contract
#endif

#if ZZNET_RX_MIN_FRAME != HW_ETH_HDR_SIZE || ZZNET_RX_MAX_FRAME != HW_ETH_MAX_RAW
#error RX slot model drifted from the driver frame size bounds
#endif

#if ZZNET_EXT_VERSION != ANXD_S2_ABI_VERSION || \
    ZZNET_EXT_RX_DIRECT != ANXD_S2F_RX_DIRECT || \
    ZZNET_EXT_RX_LINK_HDR != ANXD_S2F_RX_LINK_HDR || \
    ZZNET_EXT_RX_VERIFIED != ANXD_S2F_RX_VERIFIED || \
    ZZNET_EXT_TX_CSUM_TCP != ANXD_S2F_TX_CSUM_TCP || \
    ZZNET_EXT_TX_CSUM_UDP != ANXD_S2F_TX_CSUM_UDP || \
    ZZNET_EXT_RX_CAPACITY != ANXD_S2F_RX_CAPACITY || \
    ZZNET_EXT_TX_QUICK != ANXD_S2F_TX_QUICK || \
    ZZNET_EXT_RX_POLL != ANXD_S2F_RX_POLL || \
    ZZNET_EXT_RX_BATCH != ANXD_S2F_RX_BATCH || \
    ZZNET_EXT_TX_MORE != ANXD_S2F_TX_MORE || \
    ZZNET_EXT_RXF_SUMMED != ANXD_S2_RXF_SUMMED || \
    ZZNET_EXT_RXF_VERIFIED != ANXD_S2_RXF_VERIFIED || \
    ZZNET_EXT_TXF_TCP != ANXD_S2_TXF_TCP || \
    ZZNET_EXT_TXF_UDP != ANXD_S2_TXF_UDP || \
    ZZNET_EXT_TXF_MORE != ANXD_S2_TXF_MORE || \
    ZZNET_EXT_BATCH_VERSION != ANXD_S2_RX_BATCH_VERSION || \
    ZZNET_EXT_BATCH_MAX != ANXD_S2_RX_BATCH_MAX
#error extension model drifted from anxs2ext.h
#endif
_Static_assert(ZZNET_EXT_MIN_SIZE == ANXD_S2_EXTENSION_SIZE,
               "extension model drifted from anxs2ext.h");
_Static_assert(ANXD_S2_RX_BATCH_SIZE(0) == 8,
               "extension model drifted from anxs2ext.h");

// FIXME get rid of global var!
static ULONG ZZ9K_REGS = 0;
#define ZZ9K_RX 0x2000
#define ZZ9K_TX 0x8000
#define ZZ9K_TX_STATUS 0x68

/* Status reads (about 1 us each, served by the ARM) a writer spends waiting
 * for one of the four TX slots to retire before it gives up on the frame.
 * Four full-size frames leave the wire in ~0.5 ms at 100 Mbit/s and ~5 ms at
 * 10 Mbit/s; the firmware retires refused frames as well, so only a hung
 * firmware reaches this. */
#define ZZNET_TX_WAIT_READS 100000UL

struct Sana2DeviceStats global_stats;
BOOL is_online;

/* issue #29: count of empty (firmware-cleared) RX slots the framer skipped
 * instead of acking. Surfaced via S2_GETSPECIALSTATS as "RxEmptySlot".
 * After this fix it should be large while BadData/Overruns drop to ~0,
 * confirming the old counts were the empty-slot artifact, not real loss. */
ULONG rxv_empty_slot = 0;
#define ZZSS_RX_EMPTY_SLOT 0x5A5A0021UL

/* issue #29 residual-stall ACK probe (diagnostic).
 *
 * The residual stall is a TCP send-side wedge: the Amiga stops advancing
 * snd_una on an UPLOAD. The live monitor proved the card keeps delivering
 * inbound frames with zero loss during the wedge, so the open question is
 * whether the server's advancing cumulative ACK actually reaches the stack.
 *
 * We parse the TCP headers of the SMB (port 445) connection from the exact
 * bytes on each side of the SANA-II boundary:
 *   - inbound (server -> Amiga, src port 445): the server's cumulative ACK
 *     = how far the server has received the Amiga's upload. Parsed from the
 *     staged copy handed to the stack in read_frame.
 *   - outbound (Amiga -> server, dst port 445): the Amiga's own send seq,
 *     parsed from the staged TX-window frame in write_frame.
 *
 * DECISIVE read at the stall: if the Amiga keeps (re)transmitting at a seq
 * BELOW the server's cumulative ACK (rxv_tx_seq < rxv_srv_ack), it is
 * resending data the server has already acknowledged -> the stack is
 * ignoring a valid ACK (send-side / stack bug), NOT the card dropping it.
 * If instead rxv_srv_ack never reaches rxv_tx_seq_max, the server never
 * confirmed that data -> a genuine delivery gap to chase card-side. */
volatile ULONG rxv_srv_ack     = 0;   /* last server cumulative ack (in, src 445) */
volatile ULONG rxv_srv_ack_upd = 0;   /* # times srv ack advanced forward         */
volatile ULONG rxv_p445_in     = 0;   /* inbound TCP frames parsed (src 445)       */
volatile ULONG rxv_tx_seq      = 0;   /* last outbound seq (out, dst 445)          */
volatile ULONG rxv_tx_seq_max  = 0;   /* highest outbound seq+payload (dst 445)    */
volatile ULONG rxv_p445_out    = 0;   /* outbound TCP frames parsed (dst 445)      */
#define ZZSS_RX_SRV_ACK      0x5A5A0022UL
#define ZZSS_RX_SRV_ACK_UPD  0x5A5A0023UL
#define ZZSS_RX_P445_IN      0x5A5A0024UL
#define ZZSS_RX_TX_SEQ       0x5A5A0025UL
#define ZZSS_RX_TX_SEQ_MAX   0x5A5A0026UL
#define ZZSS_RX_P445_OUT     0x5A5A0027UL

/* Minimal, bounds-checked IPv4/TCP header parse over `ip` (the IP header),
 * `len` bytes available. Reads byte-wise through a volatile pointer so it is
 * safe on both RAM (db_RxStage) and MMIO (FPGA TX/RX window) sources and the
 * compiler cannot coalesce a byte-combine into an unaligned wide load (the
 * documented Cortex-A9 strongly-ordered fault applies firmware-side; byte
 * reads are the portable rule for slot memory on both ends). Returns 1 and
 * fills the requested out-params for an IPv4/TCP packet, else 0. */
static int zznet_parse_ip_tcp(volatile const UBYTE *ip, ULONG len,
                              USHORT *sport, USHORT *dport,
                              ULONG *seq, ULONG *ack, ULONG *paylen)
{
	UBYTE  ihl;
	ULONG  ip_hl, tcp_off, total_len, hdrs;
	volatile const UBYTE *tcp;

	if (len < 20)              return 0;   /* room for a min IPv4 header  */
	if ((ip[0] >> 4) != 4)     return 0;   /* IPv4                        */
	ihl   = ip[0] & 0x0f;
	ip_hl = (ULONG)ihl * 4;
	if (ip_hl < 20 || ip_hl > len) return 0;
	if (ip[9] != 6)            return 0;   /* TCP                         */

	total_len = ((ULONG)ip[2] << 8) | ip[3];
	if (total_len > len) total_len = len;  /* clamp to what we actually have */
	if (total_len < ip_hl + 20) return 0;  /* room for a min TCP header   */

	tcp = ip + ip_hl;
	if (sport) *sport = ((USHORT)tcp[0] << 8) | tcp[1];
	if (dport) *dport = ((USHORT)tcp[2] << 8) | tcp[3];
	if (seq)   *seq   = ((ULONG)tcp[4] << 24) | ((ULONG)tcp[5] << 16) |
	                    ((ULONG)tcp[6] << 8)  |  (ULONG)tcp[7];
	if (ack)   *ack   = ((ULONG)tcp[8] << 24) | ((ULONG)tcp[9] << 16) |
	                    ((ULONG)tcp[10] << 8) |  (ULONG)tcp[11];
	tcp_off = (ULONG)(tcp[12] >> 4) * 4;
	if (tcp_off < 20) tcp_off = 20;
	if (paylen) {
		hdrs = ip_hl + tcp_off;
		*paylen = (total_len > hdrs) ? (total_len - hdrs) : 0;
	}
	return 1;
}

/* Staging buffer for RX payload copies.
 *
 * Roadshow's BufferManagement bm_CopyToBuffer is a generic memcpy. When
 * we hand it `frame + 18` (the non-RAW payload offset inside the FPGA
 * RX window), the source is only word-aligned (18 mod 4 == 2), so its
 * memcpy falls back to move.w at best. Every RX frame then burns
 * ~750 word-sized Zorro bus cycles instead of ~375 longword cycles.
 *
 * Staging the payload through a driver-owned RAM buffer lets us issue
 * the MMIO reads ourselves as longwords (one Z3 bus cycle per 4 bytes)
 * and then hand Roadshow a RAM source, where its generic memcpy is
 * essentially free relative to the MMIO read cost.
 *
 * 1536 covers any MTU up to 1514 plus a 2-byte alignment offset. The
 * buffer is single-use per frame and only touched from frame_proc
 * (single worker process per devbase, serialized by AmigaOS Forbid()
 * semantics on OpenDevice/CloseDevice), so no locking is needed. The
 * pointer lives on the devbase — see `struct devbase::db_RxStage`. */
#define RX_STAGE_SIZE 1536

SAVEDS void frame_proc();
static void zznet_tx_flush(DEVBASETYPE *db);
char *frame_proc_name = "ZZ9000NetFramer";

/* ZZ9000 interrupt server (INT6 default, optional INT2).
 * Reads the status once, masks+acks the ethernet bit, signals frame_proc.
 * Returns non-zero when this interrupt was ours so Exec short-circuits
 * the server chain; chains through (returns 0) when it isn't.
 */
SAVEDS ULONG dev_isr(struct devbase* db __asm("a1")) {
  volatile USHORT* status_reg = (volatile USHORT*)(ZZ9K_REGS+0x04);
  USHORT status = *status_reg;

  if (!(status & 1)) {
    return 0;
  }

  /* Disable the ethernet IRQ bit, then ack it. frame_proc re-enables bit 0. */
  *status_reg = status & 0xfffe;
  *status_reg = 8|16;

  if (db->db_Proc) {
    Signal((struct Task*)db->db_Proc, SIGBREAKF_CTRL_F);
  }
  return 1;
}

/* TX_MORE backstop at vertical blank: wake frame_proc to start frames
 * whose start was held. Never ours alone, so always chains on. */
SAVEDS ULONG zznet_vbl(struct devbase* db __asm("a1")) {
  if (db->db_Tx.held && db->db_Proc)
    Signal((struct Task*)db->db_Proc, SIGBREAKF_CTRL_E);
  return 0;
}

static UBYTE HW_MAC[] = {0x00,0x00,0x00,0x00,0x00,0x00};

static int zznet_mcast_hw_capable(void *ctx)
{
	(void)ctx;
	return (*(volatile USHORT *)(ZZ9K_REGS + ZZNET_ETH_CONFIG) &
	        ZZNET_ETH_CONFIG_CAP_MCAST_HASH) != 0;
}

static void zznet_mcast_hw_command(void *ctx, uint16_t command)
{
	(void)ctx;
	*(volatile USHORT *)(ZZ9K_REGS + ZZNET_ETH_CONFIG) = (USHORT)command;
}

static const struct zznet_mcast_io zznet_mcast_hw = {
	zznet_mcast_hw_capable,
	zznet_mcast_hw_command,
	0
};

static LONG zznet_mcast_change(DEVBASETYPE *db, const UBYTE *addr, BOOL add)
{
	LONG result;

	ObtainSemaphore(&db->db_McastSem);
	result = zznet_mcast_update(&db->db_Mcast, &zznet_mcast_hw,
	                            (const uint8_t *)addr, add ? 1 : 0);
	ReleaseSemaphore(&db->db_McastSem);
	return result;
}

/* Unicast and broadcast do not consult the group table, so they do not
 * take db_McastSem. Group frames do: exact membership is the collision filter. */
/* The presented frame's Ethernet header, read from the window once per
 * frame by frame_proc (3 longwords and a word) and used from RAM. Only
 * frame_proc touches it. */
static ULONG zznet_rx_hdr[4];

static struct zznet_rx_plan zznet_frame_plan(DEVBASETYPE *db,
                                             volatile const UBYTE *addr)
{
	uint8_t local[ZZNET_MCAST_ADDR_LEN];
	unsigned i;
	struct zznet_rx_plan plan;

	for (i = 0; i < ZZNET_MCAST_ADDR_LEN; i++)
		local[i] = addr[i];
	if (!zznet_mcast_membership_required(local))
		return zznet_mcast_rx_plan(0, local);
	ObtainSemaphore(&db->db_McastSem);
	plan = zznet_mcast_rx_plan(&db->db_Mcast, local);
	ReleaseSemaphore(&db->db_McastSem);
	return plan;
}

void set_mac_from_string(UBYTE* buf) {
  int k=0;
  for (int i=0; i<6; i++) {
    int c = buf[k];
    int v = 0;

    if (c>='0' && c<='9') c-='0';
    else if (c>='a' && c<='f') c=c+10-'a';
    else if (c>='A' && c<='F') c=c+10-'A';

    v = c<<4;
    c = buf[k+1];

    if (c>='0' && c<='9') c-='0';
    else if (c>='a' && c<='f') c=c+10-'a';
    else if (c>='A' && c<='F') c=c+10-'A';

    HW_MAC[i] = v+c;

    k+=3;
  }
}

/* AmiNetXDuo SANA-II extension (anxs2ext.h): the first ANXD_S2_EXTENSION
 * record of a known Version and Size in the open taglist is answered with
 * what this card can honour; later records are ignored. An opener without
 * one, or with an invalid one, is a plain SANA-II opener. */
static void zznet_ext_negotiate(DEVBASETYPE *db, struct BufferManagement *bm,
                                struct TagItem *tags)
{
	struct TagItem *state = tags;
	struct TagItem *ti;

	while ((ti = NextTagItem(&state))) {
		AnxdS2Extension *ext = (AnxdS2Extension *)ti->ti_Data;
		ULONG accepted;

		if (ti->ti_Tag != ANXD_S2_EXTENSION || !ext)
			continue;
		if (!zznet_ext_accept(ext->Version, ext->Size, ext->Request,
		                      db->db_ExtOffer,
		                      ((ext->RxDirect && ext->RxFilled) ?
		                       ZZNET_EXT_HAVE_RX_CB : 0) |
		                      (ext->TxFlags ? ZZNET_EXT_HAVE_TX_CB : 0) |
		                      (bm->bm_CopyToBuffer ? ZZNET_EXT_HAVE_COPYTO : 0),
		                      &accepted))
			continue;
		ext->Accepted = accepted;
		bm->bm_Ext = ext;
		bm->bm_ExtAccepted = accepted;
		D(("ZZ9000Net: extension accepted %lx\n", accepted));
		break;
	}

	/* A held TX start waits one vertical blank at most. */
	if ((bm->bm_ExtAccepted & ANXD_S2F_TX_MORE) && !db->db_VblOn) {
		db->db_VblInt.is_Node.ln_Type = NT_INTERRUPT;
		db->db_VblInt.is_Node.ln_Pri = 0;
		db->db_VblInt.is_Node.ln_Name = "ZZ9000Net TX";
		db->db_VblInt.is_Data = (APTR)db;
		db->db_VblInt.is_Code = (void(*)())zznet_vbl;
		AddIntServer(INTB_VERTB, &db->db_VblInt);
		db->db_VblOn = 1;
	}
}

/* Wake frame_proc when it left a frame in the card for a read that may
 * now be there, or for an opener set that changed. */
static void zznet_rx_kick(DEVBASETYPE *db)
{
	if (db->db_RxBehind && db->db_Proc)
		Signal((struct Task*)db->db_Proc, SIGBREAKF_CTRL_F);
}

/* Reply every queued request of `bm` that `match` selects (all of them
 * when match is 0, else RX batches only) with `err`; under db_ReadListSem
 * for the list walk, replied outside it. */
static void zznet_reply_reads(DEVBASETYPE *db, struct BufferManagement *bm,
                              BOOL batches_only, BYTE err, ULONG wire)
{
	struct Node *done = NULL, *n, *next;

	ObtainSemaphore(&db->db_ReadListSem);
	for (n = db->db_ReadList.lh_Head; (next = n->ln_Succ); n = next) {
		struct IOSana2Req *io = (struct IOSana2Req *)n;

		if ((bm && io->ios2_BufferManagement != bm) ||
		    (batches_only && io->ios2_Req.io_Command != ANXD_CMD_RX_BATCH))
			continue;
		Remove(n);
		n->ln_Succ = done;   /* chained for the reply below */
		done = n;
	}
	ReleaseSemaphore(&db->db_ReadListSem);
	while ((n = done)) {
		struct IOSana2Req *io = (struct IOSana2Req *)n;

		done = n->ln_Succ;
		io->ios2_Req.io_Error = err;
		io->ios2_WireError = wire;
		ReplyMsg((struct Message *)io);
	}
}

/* The end of an RX pass: every batch that took a frame and is still
 * queued is answered now, so no frame waits in a batch for a later pass.
 * Only frame_proc calls this. */
static void zznet_batch_flush(DEVBASETYPE *db)
{
	struct Node *done = NULL, *n, *next;

	if (!db->db_BatchPending)
		return;
	db->db_BatchPending = 0;
	ObtainSemaphore(&db->db_ReadListSem);
	for (n = db->db_ReadList.lh_Head; (next = n->ln_Succ); n = next) {
		struct IOSana2Req *io = (struct IOSana2Req *)n;

		if (io->ios2_Req.io_Command == ANXD_CMD_RX_BATCH &&
		    ((AnxdS2RxBatch *)io->ios2_Data)->Filled != 0) {
			Remove(n);
			n->ln_Succ = done;
			done = n;
		}
	}
	ReleaseSemaphore(&db->db_ReadListSem);
	while ((n = done)) {
		done = n->ln_Succ;
		((struct IOSana2Req *)n)->ios2_Req.io_Error = 0;
		((struct IOSana2Req *)n)->ios2_WireError = 0;
		ReplyMsg((struct Message *)n);
	}
}

/* Remember a packet type a direct-receive opener reads. Under
 * db_ReadListSem. */
static void zznet_note_type(struct BufferManagement *bm, USHORT type)
{
	UWORD i;

	if (!(bm->bm_ExtAccepted & ANXD_S2F_RX_DIRECT))
		return;
	for (i = 0; i < bm->bm_NTypes; i++)
		if (bm->bm_Types[i] == type)
			return;
	if (bm->bm_NTypes < sizeof(bm->bm_Types) / sizeof(bm->bm_Types[0]))
		bm->bm_Types[bm->bm_NTypes++] = type;
}

/* A frame of `type` found no posted read. Leave it in the card when the
 * unit's sole opener receives directly and has been reading this type:
 * its reader is behind rather than gone, and the card's ring holds the
 * frame until the read comes. Never with a second opener, whose traffic
 * the held frame would block. Under db_ReadListSem. */
static BOOL zznet_rx_hold(DEVBASETYPE *db, USHORT type)
{
	struct BufferManagement *bm =
	    (struct BufferManagement *)db->db_Openers.lh_Head;
	UWORD i;

	if (!bm->bm_Node.mln_Succ || bm->bm_Node.mln_Succ->mln_Succ)
		return FALSE;
	for (i = 0; i < bm->bm_NTypes; i++)
		if (bm->bm_Types[i] == type)
			return TRUE;
	return FALSE;
}

struct ProcInit
{
   struct Message msg;
   struct devbase *db;
   BOOL  error;
   UBYTE pad[2];
};

SAVEDS struct Device *DevInit( ASMR(d0) DEVBASEP                  ASMREG(d0),
                                   ASMR(a0) BPTR seglist              ASMREG(a0),
				   ASMR(a6) struct Library *_SysBase  ASMREG(a6) )
{
	UBYTE*p;
	ULONG i;
	LONG  ok;

	p = ((UBYTE*)db) + sizeof(struct Library);
	i = sizeof(DEVBASETYPE)-sizeof(struct Library);
	while( i-- )
		*p++ = 0;

	db->db_SysBase = _SysBase;
	db->db_SegList = seglist;
	db->db_Flags   = 0;

	ok = 0;
	if( (DOSBase = OpenLibrary("dos.library", 36)) ) {
		if( (UtilityBase = OpenLibrary("utility.library", 37)) ) {
			ok = 0;

      struct ConfigDev* cd = NULL;

      if ((ExpansionBase = OpenLibrary("expansion.library", 0)) ) {
        // Find Z2 or Z3 model of MNT ZZ9000
        if ((cd = (struct ConfigDev*)FindConfigDev(cd,0x6d6e,0x4)) || (cd = (struct ConfigDev*)FindConfigDev(cd,0x6d6e,0x3))) {
          BPTR fh;

          D(("ZZ9000Net: MNT ZZ9000 found.\n"));
          ZZ9K_REGS = (ULONG)cd->cd_BoardAddr;

          BOOL have_env_mac = FALSE;

          if ((fh=Open("ENV:ZZ9K_MAC",MODE_OLDFILE))) {
            UBYTE char_buf[32];
            char* res = FGets(fh,char_buf,18);
            if (!res || strlen(char_buf)<17) {
              D(("ZZ9000Net: MAC address in ENV:ZZ9K_MAC has invalid syntax.\n"));
            } else {
              D(("ZZ9000Net: Setting MAC address from ENV:ZZ9K_MAC.\n"));
              set_mac_from_string(char_buf);
              have_env_mac = TRUE;
            }
            Close(fh);
          }

          if (have_env_mac) {
            // ENV override wins: push it into the firmware
            // FIXME
            *(volatile USHORT*)(ZZ9K_REGS+0x84) = (HW_MAC[0]<<8)|HW_MAC[1];
            *(volatile USHORT*)(ZZ9K_REGS+0x84) = (HW_MAC[0]<<8)|HW_MAC[1];
            *(volatile USHORT*)(ZZ9K_REGS+0x86) = (HW_MAC[2]<<8)|HW_MAC[3];
            *(volatile USHORT*)(ZZ9K_REGS+0x86) = (HW_MAC[2]<<8)|HW_MAC[3];
            *(volatile USHORT*)(ZZ9K_REGS+0x88) = (HW_MAC[4]<<8)|HW_MAC[5];
          } else {
            // Adopt the firmware's current MAC instead of forcing the
            // old built-in default: honors a `mac` line in ZZ9000.CFG
            // (applied at cold boot, firmware 2.3+) and reads back the
            // same 68:82:F2:00:01:00 default otherwise.
            USHORT mac_hi  = *(volatile USHORT*)(ZZ9K_REGS+0x84);
            USHORT mac_mid = *(volatile USHORT*)(ZZ9K_REGS+0x86);
            USHORT mac_lo  = *(volatile USHORT*)(ZZ9K_REGS+0x88);
            HW_MAC[0] = mac_hi >> 8;
            HW_MAC[1] = mac_hi & 0xff;
            HW_MAC[2] = mac_mid >> 8;
            HW_MAC[3] = mac_mid & 0xff;
            HW_MAC[4] = mac_lo >> 8;
            HW_MAC[5] = mac_lo & 0xff;
            D(("ZZ9000Net: Using firmware MAC.\n"));
          }

          /* Firmware with the asynchronous TX path reports it in bit 15 of
           * ETH_TX_STATUS; older firmware reads 0 there and keeps the
           * synchronous send. */
          InitSemaphore(&db->db_TxSem);
          InitSemaphore(&db->db_ReadListSem);
          NEWLIST(&db->db_Openers);
          if (zznet_tx_reset(&db->db_Tx,
                             *(volatile USHORT*)(ZZ9K_REGS+ZZ9K_TX_STATUS))) {
            D(("ZZ9000Net: Using asynchronous TX.\n"));
            db->db_Flags |= DEVF_TXASYNC;
          }

          /* Firmware without these registers reads 0: no checksum verdict,
           * and the 32-frame receive ring. */
          {
            USHORT rx_meta = *(volatile USHORT*)(ZZ9K_REGS+ZZNET_RX_META);

            if ((db->db_Flags & DEVF_TXASYNC) &&
                (rx_meta & ZZNET_RX_META_TX_OFFSET2))
              db->db_Flags |= DEVF_TXSHIFT;
            db->db_ExtOffer = zznet_ext_offer(rx_meta,
                (db->db_Flags & DEVF_TXASYNC) != 0);
          }
          db->db_RxCapacity = zznet_ext_rx_capacity(
              *(volatile USHORT*)(ZZ9K_REGS+ZZNET_RX_FRAMES));

          ok = 1;

        } else {
          D(("ZZ9000Net: MNT ZZ9000 not found!\n"));
        }
				CloseLibrary(ExpansionBase);
      } else {
        D(("ZZ9000Net: failed to open expansion.library!\n"));
      }

			if (!ok) {
				CloseLibrary(DOSBase);
				CloseLibrary(UtilityBase);
			}
		}
		else {
			D(("ZZ9000Net: Could not open utility.library.\n"));
			CloseLibrary(DOSBase);
		}
	}
	else {
		D(("ZZ9000Net: Could not open dos.library.\n"));
	}

	{
		BPTR fh;
		UWORD cfg_present = 0;
		if ((fh=Open("ENV:ZZ9K_INT2",MODE_OLDFILE))) {
			D(("ZZ9000Net: Using INT2 mode (ENV).\n"));
			Close(fh);
			db->db_Flags |= DEVF_INT2MODE;
		} else if (ok && ZZ9K_REGS &&
				zzcfg_query(ZZ9K_REGS, ZZ_CFG_KEY_INT2, &cfg_present) && cfg_present) {
			// `int2 = on` in ZZ9000.CFG (firmware 2.3+)
			D(("ZZ9000Net: Using INT2 mode (ZZ9000.CFG).\n"));
			db->db_Flags |= DEVF_INT2MODE;
		} else {
			D(("ZZ9000Net: Using INT6 mode (default).\n"));
		}
	}

	/* no hardware found, reject init */
	return (ok > 0) ? (struct Device*)db : (0);
}

SAVEDS LONG DevOpen( ASMR(a1) struct IOSana2Req *ioreq           ASMREG(a1),
                         ASMR(d0) ULONG unit                         ASMREG(d0),
                         ASMR(d1) ULONG flags                        ASMREG(d1),
                         ASMR(a6) DEVBASEP                           ASMREG(a6) )
{
	LONG ok = 0,ret = IOERR_OPENFAIL;
  struct BufferManagement *bm;

	D(("ZZ9000Net: DevOpen for %ld\n",unit));

	db->db_Lib.lib_OpenCnt++; /* avoid Expunge, see below for separate "unit" open count */

  /* Multi-open support: the original driver rejected any opener past the
   * first. That blocks diagnostics like ZZNetStats from attaching while
   * Roadshow has the device open. SANA-II allows multiple openers with
   * independent BufferManagement — we do the heavy one-time init (RX
   * list, worker process, interrupt server, stats reset) only on the
   * very first open, and just hand the per-opener BM out on subsequent
   * opens. */
  if (unit==0 && db->db_Lib.lib_OpenCnt >= 1) {
    BOOL first_open = (db->db_Lib.lib_OpenCnt == 1);

    if ((bm = (struct BufferManagement*)AllocVec(sizeof(struct BufferManagement), MEMF_CLEAR|MEMF_PUBLIC))) {
      bm->bm_CopyToBuffer = (BMFunc)GetTagData(S2_CopyToBuff, 0, (struct TagItem *)ioreq->ios2_BufferManagement);
      bm->bm_CopyFromBuffer = (BMFunc)GetTagData(S2_CopyFromBuff, 0, (struct TagItem *)ioreq->ios2_BufferManagement);
      zznet_ext_negotiate(db, bm, (struct TagItem *)ioreq->ios2_BufferManagement);

      ioreq->ios2_BufferManagement = (VOID *)bm;
      ioreq->ios2_Req.io_Error = 0;
      ioreq->ios2_Req.io_Unit = (struct Unit *)unit; // not a real pointer, but id integer
      ioreq->ios2_Req.io_Device = (struct Device *)db;

      if (!first_open) {
        /* Secondary opener — hardware and worker process are already up.
         * Defensive: explicitly verify first-open init actually completed
         * (db_Proc and db_interrupt set) instead of trusting lib_OpenCnt
         * as a proxy. Under AmigaOS Forbid()-serialized OpenDevice this
         * race isn't reachable, but the explicit check is cheap and
         * protects against a future caller that opens without Forbid(). */
        if (db->db_Proc && db->db_interrupt) {
          ok  = 1;
          ret = 0;
        } else {
          D(("ZZ9000Net: secondary open rejected, first-open init incomplete\n"));
          FreeVec(bm);
          ioreq->ios2_BufferManagement = NULL;
          ret = IOERR_OPENFAIL;
          ok  = 0;
        }
      } else {

      memset(&global_stats, 0, sizeof(global_stats));
      /* Reset the file-scope diagnostic counters alongside global_stats so a
       * close/reopen presents a consistent baseline: S2_GETGLOBALSTATS starts
       * from zero here, and S2_GETSPECIALSTATS (RxEmptySlot) must too, else it
       * would report totals accumulated across previous device sessions. */
      rxv_empty_slot = 0;
      rxv_srv_ack = rxv_srv_ack_upd = rxv_p445_in = 0;
      rxv_tx_seq = rxv_tx_seq_max = rxv_p445_out = 0;

      NEWLIST(&db->db_ReadList);
      InitSemaphore(&db->db_ReadListSem);
      InitSemaphore(&db->db_McastSem);
      zznet_mcast_reset(&db->db_Mcast, &zznet_mcast_hw);

      struct ProcInit init;
      struct MsgPort *port;

      if (port = CreateMsgPort()) {
        D(("ZZ9000Net: Starting Process\n"));
        if ((db->db_Proc = CreateNewProcTags(NP_Entry, (ULONG)frame_proc, NP_Name,
                                             (ULONG)frame_proc_name, NP_Priority, 10, TAG_DONE))) {
          InitSemaphore(&db->db_ProcExitSem);

          init.error = 1;
          init.db = db;
          init.msg.mn_Length = sizeof(init);
          init.msg.mn_ReplyPort = port;

          D(("ZZ9000Net: handover db: %lx\n",init.db));

          PutMsg(&db->db_Proc->pr_MsgPort, (struct Message*)&init);
          WaitPort(port);

          if (!init.error) {
            ok = 1;

            // Register Interrupt server
            if ((db->db_interrupt = AllocMem(sizeof(struct Interrupt), MEMF_PUBLIC|MEMF_CLEAR))) {
              db->db_interrupt->is_Node.ln_Type = NT_INTERRUPT;
              db->db_interrupt->is_Node.ln_Pri = 125;
              db->db_interrupt->is_Node.ln_Name = "ZZ9000Net";
              db->db_interrupt->is_Data = (APTR)db;
              db->db_interrupt->is_Code = (void(*)())dev_isr;

              Disable();
              AddIntServer((db->db_Flags & DEVF_INT2MODE) ? INTB_PORTS : INTB_EXTER, db->db_interrupt);
              Enable();

              D(("ZZ9000Net: Interrupt server registered, using INT%ld\n",(db->db_Flags & DEVF_INT2MODE) ? 2L : 6L));
              ret = 0;
              ok = 1;

              /* Allocate the RX payload staging buffer in Fast RAM. This
               * runs only after all fallible first-open init has
               * succeeded, so a failure above cannot leak the buffer.
               * Failure here is non-fatal: read_frame falls back to a
               * direct bm_CopyToBuffer from MMIO when db_RxStage is
               * NULL. AllocVec is expected to return 8-byte alignment
               * per the Exec contract; the explicit runtime check keeps
               * us honest about zznet_mmio_read_block's longword
               * alignment precondition. */
              if (!db->db_RxStage) {
                db->db_RxStage = AllocVec(RX_STAGE_SIZE, MEMF_FAST);
                if (db->db_RxStage && ((ULONG)db->db_RxStage & 3)) {
                  D(("ZZ9000Net: AllocVec returned misaligned pointer %lx; refusing staging\n",
                     (ULONG)db->db_RxStage));
                  FreeVec(db->db_RxStage);
                  db->db_RxStage = NULL;
                }
                if (!db->db_RxStage) {
                  D(("ZZ9000Net: db_RxStage unavailable; falling back to direct MMIO copy\n"));
                }
              }

              // enable HW interrupt
              *(volatile USHORT*)(ZZ9K_REGS+0x04) = 1;

              D(("ZZ9000Net: ZZ interrupt enabled\n"));
            } else {
              D(("ZZ9000Net: failed to alloc struct Interrupt\n"));
              ret = IOERR_OPENFAIL;
              ok = 0;

              Signal((struct Task*)db->db_Proc, SIGBREAKF_CTRL_C);
              // this will block until the process has really quit and released the semaphore
              ObtainSemaphore(&db->db_ProcExitSem);
              ReleaseSemaphore(&db->db_ProcExitSem);
            }
          } else {
            D(("ZZ9000Net:process startup error\n"));
            ret = IOERR_OPENFAIL;
            ok = 0;
          }
        } else {
          D(("ZZ9000Net:couldn't create process\n"));
          ret = IOERR_OPENFAIL;
          ok = 0;
        }
        DeleteMsgPort(port);
      }
      } /* end first_open init */
    }
  } else {
    ret = IOERR_OPENFAIL;
    ok = 0;
  }

	if (ok) {
		ret = 0;
    db->db_Lib.lib_Flags &= ~LIBF_DELEXP;

    ObtainSemaphore(&db->db_ReadListSem);
    AddTail(&db->db_Openers, (struct Node *)bm);
    zznet_rx_kick(db);
    ReleaseSemaphore(&db->db_ReadListSem);
	}

	if (ret == IOERR_OPENFAIL) {
		ioreq->ios2_Req.io_Unit   = (0);
		ioreq->ios2_Req.io_Device = (0);
		ioreq->ios2_Req.io_Error  = ret;
		db->db_Lib.lib_OpenCnt--;
	}
	ioreq->ios2_Req.io_Message.mn_Node.ln_Type = NT_REPLYMSG;

	D(("ZZ9000Net: DevOpen return code %ld\n",ret));

	return ret;
}

SAVEDS BPTR DevClose(   ASMR(a1) struct IORequest *ioreq        ASMREG(a1),
                            ASMR(a6) DEVBASEP                       ASMREG(a6) )
{
	/* ULONG unit; */
	BPTR  ret = (0);

	D(("ZZ9000Net: DevClose open count %ld\n",db->db_Lib.lib_OpenCnt));

	if (!ioreq)
		return ret;

	/* Free this opener's BufferManagement. Each OpenDevice allocates one;
	 * previously this was leaked on every close. */
	{
		struct IOSana2Req *s2 = (struct IOSana2Req *)ioreq;
		struct BufferManagement *bm = s2->ios2_BufferManagement;
		if (bm) {
			ObtainSemaphore(&db->db_ReadListSem);
			Remove((struct Node *)bm);
			zznet_rx_kick(db);
			ReleaseSemaphore(&db->db_ReadListSem);
			/* anxs2ext.h: CloseDevice answers an extension opener's
			 * reads and batches still queued. */
			if (bm->bm_Ext)
				zznet_reply_reads(db, bm, FALSE, IOERR_ABORTED, 0);
			FreeVec(s2->ios2_BufferManagement);
			s2->ios2_BufferManagement = NULL;
		}
	}

	db->db_Lib.lib_OpenCnt--;

  if (db->db_Lib.lib_OpenCnt == 0) {
    /* Last opener gone — disable HW IRQ, tear down worker process.
     * Previously the IRQ was disabled on every close, which killed
     * Roadshow's RX if a diagnostic tool opened+closed the device. */
    *(volatile USHORT*)(ZZ9K_REGS+0x04) = 0;
    D(("ZZ9000Net: ZZ interrupt disabled\n"));

    if (db->db_interrupt) {
      D(("ZZ9000Net: Remove IntServer...\n"));
      Forbid();
      RemIntServer((db->db_Flags & DEVF_INT2MODE) ? INTB_PORTS : INTB_EXTER, db->db_interrupt);
      db->db_interrupt = 0;
      Permit();
    }
    if (db->db_Proc) {
      D(("ZZ9000Net: End Proc...\n"));
      Signal((struct Task*)db->db_Proc, SIGBREAKF_CTRL_C);
      db->db_Proc = 0;

      ObtainSemaphore(&db->db_ProcExitSem);
      ReleaseSemaphore(&db->db_ProcExitSem);
    }

    if (db->db_VblOn) {
      RemIntServer(INTB_VERTB, &db->db_VblInt);
      db->db_VblOn = 0;
    }
    /* Nobody is left to flush a held TX run. */
    ObtainSemaphore(&db->db_TxSem);
    zznet_tx_flush(db);
    ReleaseSemaphore(&db->db_TxSem);

    /* No RX worker can be consulting the group table past this point. */
    zznet_mcast_reset(&db->db_Mcast, &zznet_mcast_hw);

    /* Staging buffer is freed only after the worker process has
     * exited above (ObtainSemaphore/ReleaseSemaphore pair). That
     * guarantees no read_frame can still be in flight referencing
     * db_RxStage, because all RX paths run inside frame_proc. */
    if (db->db_RxStage) {
      FreeVec(db->db_RxStage);
      db->db_RxStage = NULL;
    }
  }

	ioreq->io_Device = (0);
	ioreq->io_Unit   = (struct Unit *)(-1);

	if (db->db_Lib.lib_Flags & LIBF_DELEXP)
		ret = DevExpunge(db);

	return ret;
}

SAVEDS BPTR DevExpunge( ASMR(a6) DEVBASEP                        ASMREG(a6) )
{
	BPTR seglist = db->db_SegList;

	if( db->db_Lib.lib_OpenCnt )
	{
		db->db_Lib.lib_Flags |= LIBF_DELEXP;
		return (0);
	}

  D(("ZZ9000Net: Remove Device Node...\n"));
  Remove((struct Node*)db);

	CloseLibrary(DOSBase);
	CloseLibrary(UtilityBase);
	FreeMem( ((BYTE*)db)-db->db_Lib.lib_NegSize,(ULONG)(db->db_Lib.lib_PosSize + db->db_Lib.lib_NegSize));

	return seglist;
}

struct Device *TimerBase;
static void set_last_start()
{
  struct { void *db_SysBase; } *db = (void*)0x4;
  struct IORequest req;
  memset(&req, 0, sizeof(req));
  req.io_Message.mn_Length = sizeof(req);

  if (OpenDevice(TIMERNAME, UNIT_MICROHZ, &req, 0) == 0)
  {
    TimerBase = req.io_Device;
    GetSysTime(&global_stats.LastStart);
    CloseDevice(&req);
  }
}

ULONG read_frame(DEVBASETYPE *db, struct IOSana2Req *req, volatile UBYTE *frame, USHORT sz, USHORT tp);
ULONG write_frame(DEVBASETYPE *db, struct IOSana2Req *req, UBYTE *frame, int slot, UBYTE txf);
static ULONG write_frame_async(DEVBASETYPE *db, struct IOSana2Req *req, UBYTE txf);
static ULONG zznet_rx_batch(struct IOSana2Req *req, volatile UBYTE *frame,
                            USHORT sz, USHORT tp);

SAVEDS VOID DevBeginIO( ASMR(a1) struct IOSana2Req *ioreq       ASMREG(a1),
                            ASMR(a6) DEVBASEP                       ASMREG(a6) )
{
	ULONG unit = (ULONG)ioreq->ios2_Req.io_Unit;
	(void)unit;

	ioreq->ios2_Req.io_Message.mn_Node.ln_Type = NT_MESSAGE;
  ioreq->ios2_Req.io_Error = S2ERR_NO_ERROR;
  ioreq->ios2_WireError = S2WERR_GENERIC_ERROR;

	//D(("BeginIO command %ld unit %ld\n",(LONG)ioreq->ios2_Req.io_Command,unit));

	switch( ioreq->ios2_Req.io_Command ) {
  case CMD_READ:
    if (ioreq->ios2_BufferManagement == NULL) {
      ioreq->ios2_Req.io_Error = S2ERR_BAD_ARGUMENT;
      ioreq->ios2_WireError = S2WERR_BUFF_ERROR;
    }
    else {
      // not quick, add request to reader list
      // will be handled on interrupts by frame_proc
      ioreq->ios2_Req.io_Flags &= ~SANA2IOF_QUICK;
      ObtainSemaphore(&db->db_ReadListSem);
      AddHead((struct List*)&db->db_ReadList, (struct Node*)ioreq);
      zznet_note_type(ioreq->ios2_BufferManagement, ioreq->ios2_PacketType);
      zznet_rx_kick(db);
      ReleaseSemaphore(&db->db_ReadListSem);
      ioreq = NULL;
    }
    break;
  case S2_BROADCAST:
    /* set broadcast addr: ff:ff:ff:ff:ff:ff */
    if (ioreq->ios2_DstAddr) {
      memset(ioreq->ios2_DstAddr, 0xff, HW_ADDRFIELDSIZE);
    } else {
      D(("bcast: invalid dst addr\n"));
    }
    /* fall through */
  case S2_MULTICAST:
    if (!zznet_mcast_send_ok(ioreq->ios2_DstAddr)) {
      ioreq->ios2_Req.io_Error = S2ERR_BAD_ADDRESS;
      ioreq->ios2_WireError = S2WERR_BAD_MULTICAST;
      break;
    }
    /* fall through */
  case CMD_WRITE: {
    struct BufferManagement *bm = (struct BufferManagement *)ioreq->ios2_BufferManagement;
    UBYTE txf = 0;
    ULONG res;

    /* anxs2ext.h: per-write metadata from the opener's TxFlags, only when
     * it was granted a transmit feature the flags govern. */
    if (bm->bm_ExtAccepted & (ANXD_S2F_TX_CSUM_TCP | ANXD_S2F_TX_CSUM_UDP |
                              ANXD_S2F_TX_MORE)) {
      UBYTE f = bm->bm_Ext->TxFlags(ioreq->ios2_Data);

      if (bm->bm_ExtAccepted & ANXD_S2F_TX_CSUM_TCP) txf |= f & ANXD_S2_TXF_TCP;
      if (bm->bm_ExtAccepted & ANXD_S2F_TX_CSUM_UDP) txf |= f & ANXD_S2_TXF_UDP;
      if (bm->bm_ExtAccepted & ANXD_S2F_TX_MORE)     txf |= f & ANXD_S2_TXF_MORE;
    }
    res = (db->db_Flags & DEVF_TXASYNC)
        ? write_frame_async(db, ioreq, txf)
        : write_frame(db, ioreq, (UBYTE*)(ZZ9K_REGS+ZZ9K_TX), -1, txf);
    if (res!=0) {
      ioreq->ios2_Req.io_Error = S2ERR_NO_RESOURCES;
      ioreq->ios2_WireError = S2WERR_GENERIC_ERROR;
    } else {
      ioreq->ios2_Req.io_Error = 0;
      global_stats.PacketsSent++;
    }
    break;
  }

  case S2_ADDMULTICASTADDRESS:
  case S2_DELMULTICASTADDRESS:
    if ((ioreq->ios2_SrcAddr[0] & 1) == 0) {
      ioreq->ios2_Req.io_Error = S2ERR_BAD_ADDRESS;
      ioreq->ios2_WireError = S2WERR_BAD_MULTICAST;
    } else {
      LONG result = zznet_mcast_change(db, ioreq->ios2_SrcAddr,
          ioreq->ios2_Req.io_Command == S2_ADDMULTICASTADDRESS);
      if (result != 0) {
        ioreq->ios2_Req.io_Error = result;
        ioreq->ios2_WireError =
            (result == S2ERR_NO_RESOURCES) ? S2WERR_MULTICAST_FULL :
            (result == S2ERR_BAD_STATE) ? S2WERR_BAD_MULTICAST :
            S2WERR_GENERIC_ERROR;
      } else {
        ioreq->ios2_WireError = 0;
      }
    }
    break;

  case S2_READORPHAN:
    if (!ioreq->ios2_BufferManagement)
			{
				ioreq->ios2_Req.io_Error = S2ERR_BAD_ARGUMENT;
				ioreq->ios2_WireError = S2WERR_BUFF_ERROR;
			}
    else
			{
        ioreq->ios2_Req.io_Flags &= ~SANA2IOF_QUICK;
        // FIXME do we need this list?
        //ObtainSemaphore(&db->db_Units[unit].du_Sem);
        //AddHead((struct List*)&db->db_ReadOrphans,(struct Node*)ioreq);
        //ReleaseSemaphore(&db->db_Units[unit].du_Sem);
        ioreq = NULL;
			}
    break;

  case S2_CONFIGINTERFACE:   /* forward request */
    /* fall through */
  case S2_ONLINE:
    set_last_start();
    is_online = TRUE;
    break;
  case S2_OFFLINE:
    is_online = FALSE;
    /* anxs2ext.h: going offline answers queued RX batches. */
    zznet_reply_reads(db, NULL, TRUE, S2ERR_OUTOFSERVICE, S2WERR_UNIT_OFFLINE);
    break;

  case S2_GETSTATIONADDRESS:
    memcpy(ioreq->ios2_SrcAddr, HW_MAC, HW_ADDRFIELDSIZE); /* current */
    memcpy(ioreq->ios2_DstAddr, HW_MAC, HW_ADDRFIELDSIZE); /* default */
    break;
  case S2_DEVICEQUERY:
    {
      struct Sana2DeviceQuery *devquery;

      devquery = ioreq->ios2_StatData;
      devquery->DevQueryFormat = 0;        /* "this is format 0" */
      devquery->DeviceLevel = 0;           /* "this spec defines level 0" */

      if (devquery->SizeAvailable >= 18) devquery->AddrFieldSize = HW_ADDRFIELDSIZE * 8; /* Bits! */
      if (devquery->SizeAvailable >= 22) devquery->MTU           = 1500;
      if (devquery->SizeAvailable >= 26) devquery->BPS           = 1000*1000*100;
      if (devquery->SizeAvailable >= 30) devquery->HardwareType  = S2WireType_Ethernet;

      devquery->SizeSupplied = (devquery->SizeAvailable<30?devquery->SizeAvailable:30);
    }
    break;
  case S2_GETGLOBALSTATS:
    if (ioreq->ios2_StatData) {
      memcpy(ioreq->ios2_StatData, &global_stats, sizeof(struct Sana2DeviceStats));
    }
    break;
  case S2_GETSPECIALSTATS:
    {
      /* issue #29: expose RxEmptySlot — empty (firmware-cleared) RX slots the
       * framer skipped instead of acking. ZZNetStats prints it by name. */
      struct Sana2SpecialStatHeader *s2ssh = (struct Sana2SpecialStatHeader *)ioreq->ios2_StatData;
      if (s2ssh) {
        struct Sana2SpecialStatRecord *rec =
            (struct Sana2SpecialStatRecord *)(s2ssh + 1);
        ULONG max = s2ssh->RecordCountMax;
        ULONG n = 0;
        if (n < max) { rec[n].Type = ZZSS_RX_EMPTY_SLOT; rec[n].Count = rxv_empty_slot;   rec[n].String = (char*)"RxEmptySlot"; n++; }
        /* issue #29 ACK probe (send-wedge diagnostic) */
        if (n < max) { rec[n].Type = ZZSS_RX_P445_IN;    rec[n].Count = rxv_p445_in;      rec[n].String = (char*)"P445In";     n++; }
        if (n < max) { rec[n].Type = ZZSS_RX_SRV_ACK;    rec[n].Count = rxv_srv_ack;      rec[n].String = (char*)"SrvAck";     n++; }
        if (n < max) { rec[n].Type = ZZSS_RX_SRV_ACK_UPD;rec[n].Count = rxv_srv_ack_upd;  rec[n].String = (char*)"SrvAckUpd";  n++; }
        if (n < max) { rec[n].Type = ZZSS_RX_P445_OUT;   rec[n].Count = rxv_p445_out;     rec[n].String = (char*)"P445Out";    n++; }
        if (n < max) { rec[n].Type = ZZSS_RX_TX_SEQ;     rec[n].Count = rxv_tx_seq;       rec[n].String = (char*)"TxSeq";      n++; }
        if (n < max) { rec[n].Type = ZZSS_RX_TX_SEQ_MAX; rec[n].Count = rxv_tx_seq_max;   rec[n].String = (char*)"TxSeqMax";   n++; }
        s2ssh->RecordCountSupplied = n;
      }
    }
    break;
  case ANXD_CMD_RX_BATCH:
    {
      /* Many frames for one request (anxs2ext.h): queued like a CMD_READ
       * of its type, each frame into the next cookie through the direct
       * path, replied when full or at the end of the pass that filled
       * its first slot. Refused at once for a bad record, a raw request
       * or an opener that was not granted it. */
      struct BufferManagement *bm = (struct BufferManagement *)ioreq->ios2_BufferManagement;
      AnxdS2RxBatch *b = (AnxdS2RxBatch *)ioreq->ios2_Data;

      if (!bm || !(bm->bm_ExtAccepted & ANXD_S2F_RX_BATCH) ||
          (ioreq->ios2_Req.io_Flags & SANA2IOF_RAW)) {
        ioreq->ios2_Req.io_Error = S2ERR_NOT_SUPPORTED;
        ioreq->ios2_WireError = S2WERR_GENERIC_ERROR;
        break;
      }
      if (!b) {
        ioreq->ios2_Req.io_Error = S2ERR_BAD_ARGUMENT;
        ioreq->ios2_WireError = S2WERR_NULL_POINTER;
        break;
      }
      if (!zznet_ext_batch_ok(b->Version, b->Size, b->Count, sizeof(APTR))) {
        ioreq->ios2_Req.io_Error = S2ERR_BAD_ARGUMENT;
        ioreq->ios2_WireError = S2WERR_GENERIC_ERROR;
        break;
      }
      b->Filled = 0;
      ioreq->ios2_DataLength = 0;
      ioreq->ios2_Req.io_Flags &= ~SANA2IOF_QUICK;
      ObtainSemaphore(&db->db_ReadListSem);
      AddHead((struct List*)&db->db_ReadList, (struct Node*)ioreq);
      zznet_note_type(bm, ioreq->ios2_PacketType);
      zznet_rx_kick(db);
      ReleaseSemaphore(&db->db_ReadListSem);
      ioreq = NULL;
      break;
    }
  case ANXD_CMD_RX_POLL:
    {
      /* The opener re-posted its reads and is about to sleep: if a frame
       * waits in the card for one, wake the framer to deliver it now. The
       * framer is the only context that drains the card here, so the
       * poll hands it the work instead of draining from the caller. */
      struct BufferManagement *bm = (struct BufferManagement *)ioreq->ios2_BufferManagement;

      if (bm && (bm->bm_ExtAccepted & ANXD_S2F_RX_POLL)) {
        if (is_online) {
          ObtainSemaphore(&db->db_ReadListSem);
          zznet_rx_kick(db);
          ReleaseSemaphore(&db->db_ReadListSem);
        }
        break;
      }
      ioreq->ios2_Req.io_Error = S2ERR_NOT_SUPPORTED;
      ioreq->ios2_WireError = S2WERR_GENERIC_ERROR;
      break;
    }
  case ANXD_CMD_TX_FLUSH:
    {
      /* Start what a run of ANXD_S2_TXF_MORE writes left held. Holding
       * is per unit: every opener's held frames start. */
      struct BufferManagement *bm = (struct BufferManagement *)ioreq->ios2_BufferManagement;

      if (bm && (bm->bm_ExtAccepted & ANXD_S2F_TX_MORE)) {
        ObtainSemaphore(&db->db_TxSem);
        zznet_tx_flush(db);
        ReleaseSemaphore(&db->db_TxSem);
        break;
      }
      ioreq->ios2_Req.io_Error = S2ERR_NOT_SUPPORTED;
      ioreq->ios2_WireError = S2WERR_GENERIC_ERROR;
      break;
    }
  case ANXD_CMD_RX_CAPACITY:
    {
      struct BufferManagement *bm = (struct BufferManagement *)ioreq->ios2_BufferManagement;

      /* Only for an opener that negotiated it; anyone else gets the
       * unknown-command answer below. */
      if (bm && (bm->bm_ExtAccepted & ANXD_S2F_RX_CAPACITY)) {
        ioreq->ios2_DataLength = db->db_RxCapacity;
        break;
      }
    }
    /* fall through */
  default:
    {
      ioreq->ios2_Req.io_Error = S2ERR_NOT_SUPPORTED;
      ioreq->ios2_WireError = S2WERR_GENERIC_ERROR;
      break;
    }
	}

	if (ioreq) {
		DevTermIO(db, (struct IORequest*)ioreq);
  }
}

SAVEDS LONG DevAbortIO( ASMR(a1) struct IORequest *ioreq        ASMREG(a1),
                            ASMR(a6) DEVBASEP                       ASMREG(a6) )
{
	struct IOSana2Req* ios2 = (struct IOSana2Req*)ioreq;
	struct Node* n;
	LONG ret = -1;

	D(("ZZ9000Net: AbortIO on %lx\n",(ULONG)ioreq));

	/* Walk the read list under the semaphore to make sure the IO is still
	 * pending (and not already being serviced by frame_proc). Only then is
	 * it safe to Remove()/Reply it; otherwise the caller gets -1 meaning
	 * "IO was not abortable". */
	ObtainSemaphore(&db->db_ReadListSem);
	for (n = db->db_ReadList.lh_Head; n->ln_Succ; n = n->ln_Succ) {
		if (n == (struct Node*)ioreq) {
			Remove(n);
			ret = 0;
			break;
		}
	}
	ReleaseSemaphore(&db->db_ReadListSem);

	if (ret == 0) {
		ioreq->io_Error = IOERR_ABORTED;
		ios2->ios2_WireError = 0;
		ReplyMsg((struct Message*)ioreq);
	}
	return ret;
}

void DevTermIO( DEVBASEP, struct IORequest *ioreq )
{
  struct IOSana2Req* ios2 = (struct IOSana2Req*)ioreq;

  if (!(ios2->ios2_Req.io_Flags & SANA2IOF_QUICK)) {
    ReplyMsg((struct Message *)ioreq);
  } else {
    ioreq->io_Message.mn_Node.ln_Type = NT_REPLYMSG;
  }
}

/* Frame header layout in the ZZ9000 RX window (MMIO-backed):
 *   +0..+1   USHORT  total size
 *   +2..+3   USHORT  serial (increments each new frame)
 *   +4..+9   UBYTE[] destination MAC
 *   +10..+15 UBYTE[] source MAC
 *   +16..+17 USHORT  ethertype
 *   +18..    payload
 */

/* Fetch [size:2][serial:2] in one bus cycle on Z3 (32-bit) — the two
 * values always move together and live in adjacent words, so there is
 * no reason to poke the card twice. */
static uint32_t zznet_rx_hw_header(void *ctx)
{
	(void)ctx;
	return *(volatile ULONG *)(ZZ9K_REGS + ZZ9K_RX);
}

static uint16_t zznet_rx_hw_status(void *ctx)
{
	(void)ctx;
	return *(volatile USHORT *)(ZZ9K_REGS + ZZ_REG_ETH_RX_STATUS);
}

/* 32 bytes per block, eight registers in and out: on a 68060 reading the
 * Zorro III RX window the per-longword loop spends as much on instructions
 * as on the bus.  `blocks` is at least 1. */
static void zznet_copy_blocks(ULONG *dst, volatile ULONG *src, ULONG blocks)
{
	register ULONG *d __asm__("a0") = dst;
	register volatile ULONG *s __asm__("a1") = src;
	register ULONG n __asm__("d0") = blocks;

	__asm__ volatile(
		"1:	movem.l (%1)+,%%d1-%%d7/%%a2\n"
		"	movem.l %%d1-%%d7/%%a2,(%0)\n"
		"	lea 32(%0),%0\n"
		"	subq.l #1,%2\n"
		"	bne.s 1b\n"
		: "+a"(d), "+a"(s), "+d"(n)
		:
		: "d1", "d2", "d3", "d4", "d5", "d6", "d7", "a2", "cc", "memory");
}

static const struct zznet_rx_io zznet_rx_hw = {
	zznet_rx_hw_header,
	zznet_rx_hw_status,
	0
};

/* Bulk MMIO→RAM copy for an RX payload.
 *
 * Source is the FPGA RX window (word-aligned for non-RAW at frame+18,
 * longword-aligned for RAW at frame+4). We place the destination
 * inside `base` at the same 2-byte phase as the source so the body of
 * the copy can be done in aligned longword reads on both sides — the
 * expensive side (MMIO) drops to 1 bus cycle per 4 bytes instead of 2.
 *
 * Returns the start of the valid bytes inside `base`, which the caller
 * then passes to Roadshow's CopyToBuffer as a RAM source.
 */
static inline UBYTE* zznet_mmio_read_block(volatile UBYTE *src, UBYTE *base, ULONG n) {
	/* Caller contract: `base` is longword-aligned (enforced at AllocVec
	 * time in DevOpen). `src` is word-aligned and its 2-byte phase
	 * picks whether the body of the copy starts at `base` (RAW, src
	 * 4-aligned) or `base + 2` (non-RAW, src at frame+18). After the
	 * leading-word branch fires (if any), both src and dst are
	 * longword-aligned, and they stay that way through the longword
	 * loop — so every typed wide access in this function lands on an
	 * aligned address. */
	UBYTE *dst = base + ((ULONG)src & 2);   /* match source 2-byte phase */
	UBYTE *dst_start = dst;

	if (n == 0) return dst_start;

	/* Consume a leading 2-byte misalignment once — but only when at
	 * least 2 bytes remain. A stray 1-byte payload (runt frame) would
	 * otherwise underflow the unsigned `n` and walk the longword loop
	 * across the whole FPGA window. For n == 1 we fall straight through
	 * to the final single-byte copy below. */
	if (((ULONG)src & 2) && n >= 2) {
		*(USHORT*)dst = *(volatile USHORT*)src;
		src += 2; dst += 2; n -= 2;
	}

	/* Bulk longword stream runs only when src is actually 4-aligned. If
	 * the leading-word branch was skipped because n < 2, src is still
	 * 2-aligned and we drop straight to the byte tail. */
	if (((ULONG)src & 3) == 0) {
		volatile ULONG *ls = (volatile ULONG*)src;
		ULONG          *ld = (ULONG*)dst;
		ULONG longs = n >> 2;
		if (longs >= 8) {
			zznet_copy_blocks(ld, ls, longs >> 3);
			ld += longs & ~7UL;
			ls += longs & ~7UL;
			longs &= 7;
		}
		while (longs--) *ld++ = *ls++;
		src = (volatile UBYTE*)ls;
		dst = (UBYTE*)ld;
		n &= 3;
	}

	if (n >= 2) {
		*(USHORT*)dst = *(volatile USHORT*)src;
		src += 2; dst += 2; n -= 2;
	}
	if (n) {
		*dst = *src;
	}

	return dst_start;
}

/* MMIO payload copy straight into the opener's RxDirect buffer, returning
 * the payload's ones-complement sum (big-endian 16-bit lanes, an odd last
 * byte padded with a zero low byte, accumulated in 32 bits with end-around
 * carry) when `sum` is set. `src` is the payload in the RX window at
 * frame+18, 2 mod 4; `dst` is only promised even. One leading word puts
 * the source on a longword, so every Zorro read in the bulk is an aligned
 * longword; the 68020+ takes the possibly misaligned RAM store in its
 * stride. Every lane starts at an even payload offset, so summing words
 * and longwords folds to the same 16-bit result. */
static ULONG zznet_mmio_copy(UBYTE *dst, volatile UBYTE *src, ULONG n, BOOL sum)
{
	ULONG acc = 0, v;

	if (((ULONG)src & 2) && n >= 2) {
		v = *(volatile USHORT*)src;
		*(USHORT*)dst = (USHORT)v;
		acc = v;
		src += 2; dst += 2; n -= 2;
	}
	if (((ULONG)src & 3) == 0) {
		volatile ULONG *ls = (volatile ULONG*)src;
		ULONG longs = n >> 2;

		if (sum) {
			while (longs--) {
				v = *ls++;
				*(ULONG*)dst = v;
				dst += 4;
				acc += v;
				if (acc < v)
					acc++;
			}
		} else {
			if (longs >= 8) {
				zznet_copy_blocks((ULONG*)dst, ls, longs >> 3);
				dst += (longs & ~7UL) << 2;
				ls  += longs & ~7UL;
				longs &= 7;
			}
			while (longs--) {
				*(ULONG*)dst = *ls++;
				dst += 4;
			}
		}
		src = (volatile UBYTE*)ls;
		n &= 3;
	}
	if (n >= 2) {
		v = *(volatile USHORT*)src;
		*(USHORT*)dst = (USHORT)v;
		acc += v;
		if (acc < v)
			acc++;
		src += 2; dst += 2; n -= 2;
	}
	if (n) {
		v = (ULONG)*src << 8;
		*dst = *src;
		acc += v;
		if (acc < v)
			acc++;
	}
	return acc;
}

/* The direct receive of one frame (anxs2ext.h): the payload goes from
 * the RX window into the buffer RxDirect(cookie) names, with the Ethernet
 * header in the 14 bytes before it when `link_hdr` is set, and
 * RxFilled(cookie) reports it. Returns 0 when RxDirect declined. Called
 * before the frame is acked, so ETH_RX_META still describes it.
 *
 * Every frame goes up SUMMED unless the GEM's verdict already proves it:
 * when the opener asked for VERIFIED and the GEM checked an IPv4 TCP or
 * UDP frame, the payload is copied without summing and only the
 * structural promises are checked; otherwise the copy carries the sum and
 * VERIFIED, when asked for, comes from software over IPv4 or IPv6. A
 * frame the GEM verdict cannot cover is copied again with the sum. */
static int zznet_rx_fill(struct BufferManagement *bm, APTR cookie,
                         volatile UBYTE *frame, ULONG plen, USHORT tp,
                         BOOL link_hdr)
{
	volatile UBYTE *payload = frame + 4 + HW_ETH_HDR_SIZE;
	BOOL verify = (bm->bm_ExtAccepted & ANXD_S2F_RX_VERIFIED) != 0;
	UBYTE flags = 0;
	ULONG sum = 0;
	UBYTE *dst;

	dst = bm->bm_Ext->RxDirect(cookie, plen);
	if (!dst)
		return 0;

	if (verify && tp == 0x0800) {
		USHORT meta = *(volatile USHORT*)(ZZ9K_REGS+ZZNET_RX_META);

		if (meta & ZZNET_RX_META_PRESENT) {
			unsigned verdict = meta & ZZNET_RX_META_VERDICT;

			if (verdict == ZZNET_RX_META_TCP || verdict == ZZNET_RX_META_UDP) {
				zznet_mmio_copy(dst, payload, plen, FALSE);
				flags = zznet_ext_rx_trust4(dst, plen, verdict);
			}
		}
	}
	if (!flags) {
		sum = zznet_mmio_copy(dst, payload, plen, TRUE);
		flags = ANXD_S2_RXF_SUMMED;
		if (verify && tp == 0x0800)
			flags |= zznet_ext_rx_verify4(dst, plen, sum);
		else if (verify && tp == 0x86dd)
			flags |= zznet_ext_rx_verify6(dst, plen, sum);
	}

	if (link_hdr) {
		/* The header frame_proc copied from the window (zznet_rx_hdr);
		 * dst is only even, so the stores are words. */
		const ULONG *h = zznet_rx_hdr;
		USHORT *w = (USHORT*)(dst - HW_ETH_HDR_SIZE);
		ULONG m;
		int i;

		for (i = 0; i < 3; i++) {
			m = h[i];
			w[2*i]   = (USHORT)(m >> 16);
			w[2*i+1] = (USHORT)m;
		}
		w[6] = (USHORT)(zznet_rx_hdr[3] >> 16);
	}
	bm->bm_Ext->RxFilled(cookie, plen, sum, flags);
	return 1;
}

/* Direct receive for a non-raw CMD_READ of an opener with
 * ANXD_S2F_RX_DIRECT. Returns 0 when the frame is not for the direct path;
 * the caller then copies it through S2_CopyToBuff as usual. */
static int zznet_rx_direct(struct BufferManagement *bm, struct IOSana2Req *req,
                           volatile UBYTE *frame, ULONG datasize, USHORT tp)
{
	if (!(bm->bm_ExtAccepted & ANXD_S2F_RX_DIRECT) ||
	    (req->ios2_Req.io_Flags & SANA2IOF_RAW))
		return 0;
	if (!zznet_rx_fill(bm, req->ios2_Data, frame, datasize, tp,
	                   (bm->bm_ExtAccepted & ANXD_S2F_RX_LINK_HDR) != 0))
		return 0;
	req->ios2_Req.io_Error = req->ios2_WireError = 0;
	return 1;
}

/* One frame into an RX batch's next cookie, link header always (the batch
 * has no per-frame request fields). Returns 0 when it was filled. A frame
 * that does not fit a non-raw read, or that RxDirect declines, is lost to
 * the batch; there is no other buffer to copy it to. */
static ULONG zznet_rx_batch(struct IOSana2Req *req, volatile UBYTE *frame,
                            USHORT sz, USHORT tp)
{
	struct BufferManagement *bm = (struct BufferManagement *)req->ios2_BufferManagement;
	AnxdS2RxBatch *b = (AnxdS2RxBatch *)req->ios2_Data;

	if (sz < HW_ETH_HDR_SIZE || sz > HW_ETH_MAX_STD ||
	    !zznet_rx_fill(bm, b->Cookie[b->Filled], frame,
	                   (ULONG)sz - HW_ETH_HDR_SIZE, tp, TRUE))
		return 1;
	b->Filled++;
	return 0;
}

ULONG read_frame(DEVBASETYPE *db, struct IOSana2Req *req, volatile UBYTE *frame, USHORT sz, USHORT tp)
{
	struct BufferManagement *bm;
	volatile UBYTE *frame_ptr;
	ULONG datasize;
	ULONG err = 0;

	/* Size policy split by listener type:
	 *   - RAW   : accept up to HW_ETH_MAX_RAW  (1518, VLAN-tagged).
	 *   - non-RAW : accept up to HW_ETH_MAX_STD (1514, untagged), since
	 *               we advertise MTU = 1500 to the SANA-II client.
	 *
	 * frame_proc already drops wire-level garbage (sz < 18 or sz >
	 * HW_ETH_MAX_RAW) before matching a listener, so these checks only
	 * fire for frames that are protocol-shaped but don't fit the
	 * listener's contract — e.g. a VLAN-tagged frame delivered to a
	 * non-RAW listener. In that case returning an error is correct:
	 * the oversized frame genuinely cannot be delivered through the
	 * non-RAW path without overrunning the client's MTU-sized buffer. */
	if (req->ios2_Req.io_Flags & SANA2IOF_RAW) {
		if (sz > HW_ETH_MAX_RAW) {
			req->ios2_Req.io_Error = S2ERR_SOFTWARE;
			req->ios2_WireError    = S2WERR_BUFF_ERROR;
			return 1;
		}
		frame_ptr = frame + 4;
		datasize  = sz;
		req->ios2_Req.io_Flags = SANA2IOF_RAW;
	} else {
		if (sz < HW_ETH_HDR_SIZE || sz > HW_ETH_MAX_STD) {
			req->ios2_Req.io_Error = S2ERR_SOFTWARE;
			req->ios2_WireError    = S2WERR_BUFF_ERROR;
			return 1;
		}
		frame_ptr = frame + 4 + HW_ETH_HDR_SIZE;
		datasize  = (ULONG)sz - HW_ETH_HDR_SIZE;   /* ≤ HW_ETH_MTU by the check above */
		req->ios2_Req.io_Flags = 0;
	}

	/* Internal assertion: staging buffer must fit datasize + 2-byte
	 * phase offset. With the protocol caps above, datasize ≤ 1518
	 * (RAW, VLAN-tagged) and RX_STAGE_SIZE is 1536, so this is always
	 * true — the check exists only as a backstop for future MTU /
	 * staging-size changes. */
	if (datasize + 2 > RX_STAGE_SIZE) {
		req->ios2_Req.io_Error = S2ERR_SOFTWARE;
		req->ios2_WireError    = S2WERR_BUFF_ERROR;
		return 1;
	}

	req->ios2_DataLength = datasize;

	bm = (struct BufferManagement *)req->ios2_BufferManagement;
	if (!zznet_rx_direct(bm, req, frame, datasize, tp)) {
		/* SANA-II contract: bm_CopyToBuffer is a synchronous copy — it
		 * reads `datasize` bytes from `source` into the client-owned
		 * destination and returns success/failure of a completed copy.
		 * It is not a descriptor submission. This routine (and the
		 * pre-rev-20 direct-MMIO path before it) already depended on
		 * that contract: on return we write `rx_accept`, which hands
		 * the backlog slot back to the firmware for reuse by the next
		 * inbound frame. If the callback deferred consumption, the
		 * hardware slot would be overwritten by incoming traffic long
		 * before the client finished copying — so reusing db_RxStage
		 * across successive frames adds no new lifetime assumption
		 * beyond what the SANA-II spec already requires.
		 *
		 * Fast path: stage the payload through Fast RAM so Roadshow's
		 * generic memcpy sees a longword-aligned RAM source and can copy
		 * at memory speed instead of stalling the Zorro bus word by
		 * word. Fallback: if no staging buffer is available (Fast RAM
		 * allocation failed at open time), hand Roadshow the MMIO
		 * source directly — slower, but functionally identical to the
		 * pre-rev-20 behavior. */
		void *copy_src = db->db_RxStage
			? (void *)zznet_mmio_read_block(frame_ptr, db->db_RxStage, datasize)
			: (void *)frame_ptr;

		if (!(*bm->bm_CopyToBuffer)((void*)req->ios2_Data, copy_src, datasize)) {
			req->ios2_Req.io_Error = S2ERR_SOFTWARE;
			req->ios2_WireError    = S2WERR_BUFF_ERROR;
			err = 1;
		} else {
			req->ios2_Req.io_Error = req->ios2_WireError = 0;

			/* issue #29 ACK probe: for the SMB (445) connection, record the
			 * server's cumulative ack from the SAME bytes the stack receives
			 * (copy_src = the staged IP packet, non-RAW). src port 445 =
			 * server -> Amiga. */
			if (tp == 0x0800) {
				USHORT sp = 0, dp = 0;
				ULONG  ack = 0;
				if (zznet_parse_ip_tcp((volatile const UBYTE *)copy_src,
				                       datasize, &sp, &dp, NULL, &ack, NULL) &&
				    sp == 445) {
					rxv_p445_in++;
					if (ack != rxv_srv_ack) {
						if ((LONG)(ack - rxv_srv_ack) > 0)
							rxv_srv_ack_upd++;
						rxv_srv_ack = ack;
					}
				}
			}
		}
	}

	/* Coalesce the 12-byte dst+src MAC header into three longword MMIO reads
	 * instead of six word reads (and a second pass of three for the broadcast
	 * check). On Z3 that's 3 bus cycles instead of 9. The destination
	 * ios2_DstAddr / ios2_SrcAddr fields are word-aligned per SANA-II, so
	 * splitting each long back into two word stores is safe. */
	{
		ULONG m0 = *(volatile ULONG*)(frame + 4);    /* dst[0..3]           */
		ULONG m1 = *(volatile ULONG*)(frame + 8);    /* dst[4..5] src[0..1] */
		ULONG m2 = *(volatile ULONG*)(frame + 12);   /* src[2..5]           */

		USHORT *wd = (USHORT*)req->ios2_DstAddr;
		USHORT *ws = (USHORT*)req->ios2_SrcAddr;
		wd[0] = (USHORT)(m0 >> 16);
		wd[1] = (USHORT)(m0 & 0xFFFF);
		wd[2] = (USHORT)(m1 >> 16);
		ws[0] = (USHORT)(m1 & 0xFFFF);
		ws[1] = (USHORT)(m2 >> 16);
		ws[2] = (USHORT)(m2 & 0xFFFF);

		req->ios2_Req.io_Flags |= zznet_mcast_rx_flags(req->ios2_DstAddr);
	}

	req->ios2_PacketType = tp;

	return err;
}

/* Checksum offload for a frame already in the TX window at `start`
 * (anxs2ext.h ANXD_S2_TXF_TCP/UDP): the opener left the pseudo-header sum
 * in the TCP/UDP checksum field, and the GEM's insertion wants it zero.
 * The field's offset comes from the frame's first bytes, taken again from
 * the opener's buffer into RAM rather than read back over Zorro; only the
 * card copy changes, since the stack may retransmit its own. */
static void zznet_tx_csum(struct BufferManagement *bm, struct IOSana2Req *req,
                          UBYTE *start, USHORT sz, UBYTE txf)
{
	UBYTE peek[HW_ETH_HDR_SIZE + 80];
	ULONG have;
	USHORT off;

	if (req->ios2_Req.io_Flags & SANA2IOF_RAW) {
		have = sz < sizeof(peek) ? sz : sizeof(peek);
		if (!(*bm->bm_CopyFromBuffer)(peek, req->ios2_Data, have))
			return;
	} else {
		have = sz < sizeof(peek) ? sz : sizeof(peek);
		peek[12] = (UBYTE)(req->ios2_PacketType >> 8);
		peek[13] = (UBYTE)req->ios2_PacketType;
		if (!(*bm->bm_CopyFromBuffer)(peek + HW_ETH_HDR_SIZE, req->ios2_Data,
		                              have - HW_ETH_HDR_SIZE))
			return;
	}
	if (zznet_ext_tx_csum4(peek, have, sz, txf, &off))
		*(volatile USHORT*)(start + off) = 0;
}

/* slot < 0: the synchronous send from the start of the TX window. slot
 * 0..3: an asynchronous send from that slot, which the caller has taken
 * from db_Tx under db_TxSem. txf: the ANXD_S2_TXF_* flags granted for
 * this write (0 for a plain opener). */
ULONG write_frame(DEVBASETYPE *db, struct IOSana2Req *req, UBYTE *frame, int slot, UBYTE txf)
{
	struct BufferManagement *bm;
	UBYTE *start = frame;
	USHORT sz = 0;
	ULONG  rc = 0;

	if (req->ios2_Req.io_Flags & SANA2IOF_RAW) {
		sz = req->ios2_DataLength;
	} else {
		sz = req->ios2_DataLength + HW_ETH_HDR_SIZE;

		/* Build the 14-byte Ethernet header. Using memcpy (non-volatile
		 * frame pointer) lets libc / the compiler emit move.l where the
		 * alignment permits; forcing word stores here costs Zorro III
		 * bandwidth versus the baseline. The reg write below is volatile
		 * and serves as the commit barrier before we kick TX. */
		*((USHORT*)(frame + 12)) = (USHORT)req->ios2_PacketType;
		memcpy(frame,     req->ios2_DstAddr, HW_ADDRFIELDSIZE);
		memcpy(frame + 6, HW_MAC,            HW_ADDRFIELDSIZE);
		frame += HW_ETH_HDR_SIZE;
	}

	if (sz == 0) {
		return 0;
	}

	bm = (struct BufferManagement *)req->ios2_BufferManagement;
	if (!(*bm->bm_CopyFromBuffer)(frame, req->ios2_Data, req->ios2_DataLength)) {
		return 1;
	}

	/* issue #29 ACK probe: for the SMB (445) connection, record the Amiga's
	 * outbound send seq from the staged TX-window frame (non-RAW: `frame` now
	 * points at the IP packet). dst port 445 = Amiga -> server. Parsed before
	 * the kick so it never races the FPGA DMA. Compared against rxv_srv_ack at
	 * the stall: rxv_tx_seq < rxv_srv_ack ⇒ resending already-acked data. */
	if (!(req->ios2_Req.io_Flags & SANA2IOF_RAW) &&
	    (USHORT)req->ios2_PacketType == 0x0800) {
		USHORT sp = 0, dp = 0;
		ULONG  seq = 0, paylen = 0;
		if (zznet_parse_ip_tcp((volatile const UBYTE *)frame,
		                       req->ios2_DataLength, &sp, &dp,
		                       &seq, NULL, &paylen) &&
		    dp == 445) {
			ULONG end = seq + paylen;
			rxv_p445_out++;
			rxv_tx_seq = seq;
			if ((LONG)(end - rxv_tx_seq_max) > 0)
				rxv_tx_seq_max = end;
		}
	}

	/* A shifted frame's checksum field is zeroed by the firmware on
	 * consent (ETH_TX bit 13), without the 68k touching the window. */
	if ((txf & (ANXD_S2_TXF_TCP | ANXD_S2_TXF_UDP)) &&
	    !(slot >= 0 && (db->db_Flags & DEVF_TXSHIFT)))
		zznet_tx_csum(bm, req, start, sz, txf);

	if (slot >= 0) {
		/* Queued, not sent: the bus cycle returns at once and the slot
		 * stays the firmware's until ETH_TX_STATUS retires it. A refused
		 * submission is retired the same way and cannot be told apart
		 * from a sent one, so there is no result to read back.
		 *
		 * ANXD_S2_TXF_MORE: the opener sends a run back to back, so the
		 * start is held until the run ends (ANXD_CMD_TX_FLUSH), a write
		 * without the flag, all four slots holding frames, or the next
		 * vertical blank; the frames then leave together. */
		USHORT word = zznet_tx_word(slot, sz);

		if (db->db_Flags & DEVF_TXSHIFT) {
			word |= ZZNET_TX_OFFSET2;
			if (txf & (ANXD_S2_TXF_TCP | ANXD_S2_TXF_UDP))
				word |= ZZNET_TX_CSUM;
		}
		if (txf & ANXD_S2_TXF_MORE) {
			zznet_tx_hold(&db->db_Tx, word);
			if (db->db_Tx.held >= ZZNET_TX_SLOTS)
				zznet_tx_flush(db);
			return 0;
		}
		zznet_tx_flush(db);
		*(volatile USHORT*)(ZZ9K_REGS+0x80) = word;
		zznet_tx_submitted(&db->db_Tx);
		return 0;
	}

	{
		volatile USHORT *reg = (volatile USHORT*)(ZZ9K_REGS+0x80);
		*reg = sz;      /* kick the TX engine */
		rc   = *reg;    /* read back hardware status */
		if (rc) {
			D(("tx err: %d\n",rc));
		}
	}

	return rc;
}

/* Asynchronous send: take the next TX slot in submission order, waiting
 * for the firmware to retire the oldest one while all four are in flight.
 * A slot is reused only after ETH_TX_STATUS has counted its frame, so the
 * copy into it can never overwrite a frame the GEM is still reading. */
/* Start every held frame, oldest first. Under db_TxSem. */
static void zznet_tx_flush(DEVBASETYPE *db)
{
	USHORT word;

	while ((word = zznet_tx_unhold(&db->db_Tx)))
		*(volatile USHORT*)(ZZ9K_REGS+0x80) = word;
}

static ULONG write_frame_async(DEVBASETYPE *db, struct IOSana2Req *req, UBYTE txf)
{
	ULONG rc = 1;
	ULONG reads;
	int slot;

	/* A frame longer than a slot would run into the next one. */
	if (req->ios2_DataLength > HW_ETH_MAX_RAW)
		return 1;

	ObtainSemaphore(&db->db_TxSem);
	for (reads = 0; (slot = zznet_tx_slot(&db->db_Tx)) < 0 &&
	                reads < ZZNET_TX_WAIT_READS; reads++) {
		/* Held frames only retire once started. */
		zznet_tx_flush(db);
		zznet_tx_reclaim(&db->db_Tx,
		                 *(volatile USHORT*)(ZZ9K_REGS+ZZ9K_TX_STATUS));
	}
	if (slot >= 0) {
		/* Shifted, the frame starts 2 bytes into the slot and the
		 * stack's CopyFromBuff writes the IP payload to a longword
		 * aligned card address. */
		rc = write_frame(db, req,
		                 (UBYTE*)(ZZ9K_REGS + ZZ9K_TX +
		                          (ULONG)slot * ZZNET_TX_SLOT_SIZE +
		                          ((db->db_Flags & DEVF_TXSHIFT) ? 2 : 0)),
		                 slot, txf);
	} else {
		D(("tx: no slot retired\n"));
	}
	ReleaseSemaphore(&db->db_TxSem);
	return rc;
}

SAVEDS void frame_proc() {
  ULONG wmask;

  D(("ZZ9000Net: frame_proc()\n"));

  struct ProcInit* init;

  {
    struct { void *db_SysBase; } *db = (void*)0x4;
    struct Process* proc;

    proc = (struct Process*)FindTask(NULL);
    WaitPort(&proc->pr_MsgPort);
    init = (struct ProcInit*)GetMsg(&proc->pr_MsgPort);
  }

  struct devbase* db = init->db;

  init->error = 0;
  db = init->db;
  ObtainSemaphore(&db->db_ProcExitSem);
  ReplyMsg((struct Message*)init);

  wmask = SIGBREAKF_CTRL_F | SIGBREAKF_CTRL_C | SIGBREAKF_CTRL_E;

  struct zznet_rx_state rx_state;
  zznet_rx_reset(&rx_state,
                 *(volatile USHORT *)(ZZ9K_REGS + ZZ_REG_FW_VERSION));
  ULONG  recv          = Wait(wmask);   /* wait for first packet */

  volatile UBYTE*  frm       = (volatile UBYTE*)(ZZ9K_REGS+ZZ9K_RX);
  volatile USHORT* rx_accept = (volatile USHORT*)(ZZ9K_REGS+0x82);
  volatile USHORT* irq_ctrl  = (volatile USHORT*)(ZZ9K_REGS+0x04);
  UWORD pass = 0;   /* frames since the RX batches were last answered */

  while (1) {
    struct IOSana2Req *ior;

    if (recv & SIGBREAKF_CTRL_C) {
      D(("ZZ9000Net: process end\n"));
      break;
    }

    /* TX_MORE backstop from the vertical blank: start held frames. */
    if (db->db_Tx.held &&
        (SetSignal(0, SIGBREAKF_CTRL_E) & SIGBREAKF_CTRL_E)) {
      ObtainSemaphore(&db->db_TxSem);
      zznet_tx_flush(db);
      ReleaseSemaphore(&db->db_TxSem);
    }

    /* A long pass still answers its RX batches regularly. */
    if (++pass >= ZZNET_EXT_BATCH_MAX) {
      pass = 0;
      zznet_batch_flush(db);
    }

    struct zznet_rx_state rx_prev = rx_state;
    struct zznet_rx_decision d = zznet_rx_next(&rx_state, &zznet_rx_hw);
    USHORT sz = d.size, serial = d.serial;

    if (d.empty)
      rxv_empty_slot++;
    global_stats.BadData  += d.bad_data;
    global_stats.Overruns += d.overruns;

    if (d.action == ZZNET_RX_WAIT) {
      /* Nothing new or an empty slot. Re-enable the ethernet IRQ so the
       * ISR can wake us, then sleep. Enable-before-wait is correct: if a
       * frame raced in between our header read and the enable, the ISR
       * will signal and Wait returns immediately. */
      zznet_batch_flush(db);
      *irq_ctrl = 1;
      recv = Wait(wmask);
      continue;
    }
    if (d.action == ZZNET_RX_DROP) {
      /* Ack with the frame's serial so the firmware RX-accept handshake
       * advances past exactly this (bad) frame and nothing else. */
      *rx_accept = serial;
      continue;
    }

    zznet_rx_hdr[0] = *(volatile ULONG*)(frm + 4);
    zznet_rx_hdr[1] = *(volatile ULONG*)(frm + 8);
    zznet_rx_hdr[2] = *(volatile ULONG*)(frm + 12);
    zznet_rx_hdr[3] = (ULONG)*(volatile USHORT*)(frm + 16) << 16;
    USHORT packet_type = (USHORT)(zznet_rx_hdr[3] >> 16);
    struct IOSana2Req *match = NULL;

    /* GEM hash collisions are not unknown packet types. Ack and drop
     * the exact miss before reader selection; UnknownTypesReceived
     * stays reserved for an accepted frame with no reader or a failed
     * read. */
    {
      struct zznet_rx_plan plan = zznet_frame_plan(db, (const UBYTE *)zznet_rx_hdr);

      if (!plan.select_reader) {
        if (plan.count_unknown)
          global_stats.UnknownTypesReceived++;
        *rx_accept = serial;
        continue;
      }
    }

    /* Walk the read list only long enough to find a matching listener
     * and detach it. Doing the payload copy (read_frame) and ReplyMsg
     * outside the semaphore keeps DevAbortIO / CMD_READ unblocked for
     * the duration of the Zorro bus copy. */
    ObtainSemaphore(&db->db_ReadListSem);
    db->db_RxBehind = 0;
    for (ior = (struct IOSana2Req *)db->db_ReadList.lh_Head;
         ior->ios2_Req.io_Message.mn_Node.ln_Succ;
         ior = (struct IOSana2Req *)ior->ios2_Req.io_Message.mn_Node.ln_Succ) {
      if (ior->ios2_PacketType == packet_type) {
        Remove((struct Node*)ior);
        match = ior;
        break;
      }
    }
    if (!match && zznet_rx_hold(db, packet_type))
      db->db_RxBehind = 1;
    ReleaseSemaphore(&db->db_ReadListSem);

    if (db->db_RxBehind) {
      /* Leave the frame presented and unacked for the sole opener's next
       * read (anxs2ext.h, ANXD_CMD_RX_POLL). Its counters are taken when
       * it is delivered, and the model forgets it was seen. The card IRQ
       * stays masked, since the firmware would raise it again at once;
       * posting a read, polling, opening or closing wakes us instead. */
      rx_state = rx_prev;
      global_stats.BadData  -= d.bad_data;
      global_stats.Overruns -= d.overruns;
      zznet_batch_flush(db);
      recv = Wait(wmask);
      continue;
    }

    if (match && match->ios2_Req.io_Command == ANXD_CMD_RX_BATCH) {
      if (zznet_rx_batch(match, frm, sz, packet_type) == 0) {
        global_stats.PacketsReceived++;
      } else {
        global_stats.UnknownTypesReceived++;
      }
      if (((AnxdS2RxBatch *)match->ios2_Data)->Filled >=
          ((AnxdS2RxBatch *)match->ios2_Data)->Count) {
        match->ios2_Req.io_Error = match->ios2_WireError = 0;
        ReplyMsg((struct Message *)match);
      } else {
        /* Back at the head, still first for its type; answered at the
         * end of this pass if nothing fills it first. */
        ObtainSemaphore(&db->db_ReadListSem);
        AddHead((struct List*)&db->db_ReadList, (struct Node*)match);
        ReleaseSemaphore(&db->db_ReadListSem);
        if (((AnxdS2RxBatch *)match->ios2_Data)->Filled)
          db->db_BatchPending = 1;
      }
    } else if (match) {
      ULONG res = read_frame(db, match, frm, sz, packet_type);
      if (res == 0) {
        global_stats.PacketsReceived++;
      } else {
        /* read_frame already set io_Error/ios2_WireError; reply so the
         * caller learns the request failed instead of leaving it on
         * a now-dangling list entry. */
        D(("RERR %ld\n", res));
        global_stats.UnknownTypesReceived++;
      }
      ReplyMsg((struct Message *)match);
    } else {
      /* No listener matched — frame dropped. A future change could
       * route these to S2_READORPHAN requests. */
      global_stats.UnknownTypesReceived++;
    }

    /* Release the FPGA RX slot so the next frame can land. We ack with the
     * frame's own serial so the firmware RX-accept handshake advances past
     * exactly the frame we just read (a stray ack of an empty/other slot is
     * rejected firmware-side). We do NOT re-enable the ethernet IRQ here —
     * staying masked lets us drain any already-queued frames via the next
     * loop iteration without paying for an IRQ we'd ignore anyway. */
    *rx_accept = serial;
  }
  // disable interrupt
  *(volatile USHORT*)(ZZ9K_REGS+0x04) = 0;

  Forbid();
  ReleaseSemaphore(&db->db_ProcExitSem);
}
