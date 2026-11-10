/* data.c — IPSC GROUP_DATA/PVT_DATA <-> HBP data bursts (CSBK, data header, rate 1/2 and 3/4 blocks).
 *
 * IPSC carries each burst's PDU without FEC (layout as node-dmr-lib DMRDataPayload.js, matching
 * captures/data-ipsc.pcap); HBP carries the full 33-byte burst.  One more step on the way up:
 * MOTOTRBO radios may send confirmed data with a DMR-standard compressed UDP/IP header (SAP 3),
 * which HBP data servers such as D-APRS only accept as a plain IPv4 datagram (SAP 4).  Those
 * messages are held until their last block, decompressed and sent on as new blocks. */
#include "data.h"
#include "ipsc.h"
#include "hbp.h"
#include "translate.h"
#include "ipsc_const.h"
#include "hbp_const.h"
#include "dmr/dmr.h"
#include "log.h"
#include <stdlib.h>
#include <string.h>
#include <sys/random.h>

#define LOGN "data"

enum { DT_CSBK = 3, DT_HEADER = 6, DT_RATE12 = 7, DT_RATE34 = 8 };

#define IPSC_PAYLOAD_OFF 30          /* data type; PDU size in bits at +6; PDU at +8 */
#define CSBKO_PREAMBLE   0x3D
#define DPF_RESPONSE     0x1
#define DPF_UNCONFIRMED  0x2
#define DPF_CONFIRMED    0x3
#define SAP_UDP_COMPRESSED 0x3
#define SAP_IP           0x4
#define RATE34_CRC9_MASK 0x1FF
#define HEADER_CRC_MASK  0xCCCC
#define MAX_BLOCKS       127         /* blocks-to-follow is 7 bits */
#define MAX_BLOCK_DATA   18
#define DEFAULT_CC       1
#define RF_QUIET_S       0.5         /* node-dmr-lib IPSCPeer waits this long after the repeater's last packet:
                                        data sent sooner (an ACK right after the radio's message) is not keyed */
#define BURST_S          0.06        /* one burst per TDMA frame and slot */
#define DN_QUEUE         128
#define DN_PKT_MAX       (IPSC_PAYLOAD_OFF + 8 + 18 + 2)
#define MAX_HELD_PRE     16

typedef struct {
    uint8_t dmrd[DMRD_CORE_LEN];
    uint8_t pdu[12];
    int     cc;
} held_pre;

typedef struct {
    uint8_t b[DN_PKT_MAX];
    int     n, ts, dt, last;
    double  queued;
} dn_pkt;

struct datapath;

typedef struct {
    /* IPSC -> HBP */
    int      up_active;
    uint8_t  up_key[7];              /* src, dst, IPSC call sequence */
    uint8_t  up_stream[4];
    uint8_t  up_seq;
    int      held;                   /* compressed header waiting for its blocks */
    uint8_t  held_hdr[12];
    int      held_want, held_n, held_cc, held_dt;
    uint8_t  held_blk[MAX_BLOCKS][18];

    /* HBP -> IPSC */
    int      dn_active;
    uint8_t  dn_stream[4];
    uint8_t  dn_call_seq;
    uint8_t  dn_ctrl[4];
    uint16_t dn_rtp_seq;
    uint32_t dn_rtp_ts;
    int      dn_blocks_left;
    held_pre pre[MAX_HELD_PRE];      /* preambles wait to see whether a response header follows */
    int      pre_n;
    dn_pkt   q[DN_QUEUE];
    int      q_head, q_len;
    double   next_tx;
    ev_timer *tx_timer;
    struct datapath *dp;
} slot_state;

struct datapath {
    const Config *cfg;
    ev_loop *loop;
    double rf_last;                  /* last data packet from the repeater */
    struct ipsc *ip;
    struct hbp *hb;
    struct translator *tr;
    slot_state s[3];                 /* [1] TS1, [2] TS2 */
};

datapath *datapath_new(const Config *cfg, ev_loop *loop)
{
    datapath *dp = calloc(1, sizeof *dp);
    if (!dp) return NULL;
    dp->cfg = cfg;
    dp->loop = loop;
    dp->rf_last = -RF_QUIET_S;
    for (int ts = 1; ts <= 2; ts++) dp->s[ts].dp = dp;
    return dp;
}

void datapath_set_protocols(datapath *dp, struct ipsc *ip, struct hbp *hb, struct translator *tr)
{
    dp->ip = ip; dp->hb = hb; dp->tr = tr;
}

void datapath_free(datapath *dp)
{
    if (!dp) return;
    for (int ts = 1; ts <= 2; ts++)
        if (dp->s[ts].tx_timer) ev_timer_cancel(dp->loop, dp->s[ts].tx_timer);
    free(dp);
}

static void dn_pump(slot_state *s);

static void dn_timer_cb(ev_loop *loop, void *ud)
{
    (void)loop;
    slot_state *s = ud;
    s->tx_timer = NULL;
    dn_pump(s);
}

static void dn_pump(slot_state *s)
{
    datapath *dp = s->dp;
    while (s->q_len > 0 && !s->tx_timer) {
        double now = ev_now(dp->loop);
        double due = dp->rf_last + RF_QUIET_S > s->next_tx ? dp->rf_last + RF_QUIET_S : s->next_tx;
        if (now < due) { s->tx_timer = ev_timer_at(dp->loop, due, dn_timer_cb, s); return; }
        dn_pkt *p = &s->q[s->q_head];
        ipsc_send_voice(dp->ip, p->b, p->n);
        LOGD(LOGN, "HBP->IPSC data sent %s len=%d ts=%d dt=%d end=%d waited=%dms  raw=%s",
             p->b[0] == PVT_DATA ? "PVT_DATA" : "GROUP_DATA", p->n, p->ts, p->dt, p->last,
             (int)((now - p->queued) * 1000), log_hex(p->b, p->n));
        s->q_head = (s->q_head + 1) % DN_QUEUE;
        s->q_len--;
        s->next_tx = now + BURST_S;
    }
}

static void rand_bytes(uint8_t *out, int n)
{
    if (getrandom(out, (size_t)n, 0) != n)
        for (int i = 0; i < n; i++) out[i] = (uint8_t)rand();
}

static unsigned u24(const uint8_t *p) { return (unsigned)(p[0] << 16 | p[1] << 8 | p[2]); }

/* ---------------- IPSC -> HBP ---------------- */

static void send_burst(datapath *dp, int ts, int is_private, const uint8_t src[3], const uint8_t dst[3],
                       int dt, int cc, const uint8_t *pdu)
{
    slot_state *s = &dp->s[ts];
    dmr_bit info[196];
    if (dt == DT_RATE34) dmr_trellis34_encode(pdu, info);
    else dmr_bptc_encode_data(pdu, info);

    uint32_t st = dmr_slot_type_encode(cc, dt);
    dmr_bit fb[264];
    memcpy(fb, info, 98);
    for (int i = 0; i < 10; i++) { fb[98 + i] = (st >> (19 - i)) & 1; fb[156 + i] = (st >> (9 - i)) & 1; }
    memcpy(fb + 108, DMR_BS_DATA_SYNC, 48);
    memcpy(fb + 166, info + 98, 98);

    uint8_t dmrd[DMRD_LEN];
    uint32_t rid = translator_repeater_id(dp->tr);
    memcpy(dmrd, "DMRD", 4);
    dmrd[DMRD_SEQ_OFF] = s->up_seq++;
    memcpy(dmrd + DMRD_SRC_OFF, src, 3);
    memcpy(dmrd + DMRD_DST_OFF, dst, 3);
    dmrd[DMRD_RPTR_OFF] = (uint8_t)(rid >> 24); dmrd[DMRD_RPTR_OFF + 1] = (uint8_t)(rid >> 16);
    dmrd[DMRD_RPTR_OFF + 2] = (uint8_t)(rid >> 8); dmrd[DMRD_RPTR_OFF + 3] = (uint8_t)rid;
    dmrd[DMRD_FLAGS_OFF] = (uint8_t)((ts == 2 ? HBPF_TGID_TS2 : 0) | (is_private ? HBPF_TGID_CALL_P : 0) |
                                     HBPF_FRAMETYPE_DATASYNC | dt);
    memcpy(dmrd + DMRD_STREAM_OFF, s->up_stream, 4);
    dmr_bits_to_bytes(fb, 264, dmrd + DMRD_PAYLOAD_OFF);
    dmrd[DMRD_BER_OFF] = 0; dmrd[DMRD_BER_OFF + 1] = 0;
    hbp_send_dmrd(dp->hb, dmrd, DMRD_LEN);
}

static int moto_port(int id)
{
    switch (id - 94) {               /* DMR-standard compression on MOTOTRBO: port IDs offset by 94 */
        case 1: return 4001;         /* LRRP */
        case 3: return 4005;         /* ARS */
        case 4: return 4007;         /* TMS */
        case 5: return 4008;         /* telemetry */
        default: return -1;
    }
}

static int ip_prefix(int address_id)
{
    switch (address_id) {
        case 0: return 12;           /* radio network */
        case 1: return 13;           /* server (gateway) network */
        case 2: return 225;          /* group */
        default: return -1;
    }
}

static uint32_t ones_sum(const uint8_t *d, int n, uint32_t sum)
{
    for (int i = 0; i + 1 < n; i += 2) sum += (uint32_t)(d[i] << 8 | d[i + 1]);
    if (n & 1) sum += (uint32_t)d[n - 1] << 8;
    return sum;
}

static uint16_t ones_fold(uint32_t sum)
{
    while (sum >> 16) sum = (sum & 0xFFFF) + (sum >> 16);
    return (uint16_t)~sum;
}

/* The held confirmed rate 3/4 message with its UDP/IP header restored, as a new header and blocks.
 * Returns the block count, or -1 to forward the original untouched. */
static int confirmed(const uint8_t hdr[12]) { return (hdr[0] & 0xF) == DPF_CONFIRMED; }

/* User octets per block; 0 for confirmed rate 1/2, whose IPSC block layout no capture shows yet. */
static int block_data(int is_confirmed, int dt)
{
    if (dt == DT_RATE34) return is_confirmed ? 16 : 18;
    if (dt == DT_RATE12) return is_confirmed ? 0 : 12;
    return 0;
}

static int decompress(const slot_state *s, uint8_t hdr_out[12], uint8_t blk_out[MAX_BLOCKS][18])
{
    int conf = confirmed(s->held_hdr), bd = block_data(conf, s->held_dt), lead = conf ? 2 : 0;
    uint8_t user[MAX_BLOCKS * MAX_BLOCK_DATA];
    int n = s->held_want * bd;
    for (int k = 0; k < s->held_want; k++)   /* the message CRC-32 below covers every block */
        memcpy(user + k * bd, s->held_blk[k] + lead, (size_t)bd);
    uint32_t crc32 = (uint32_t)(user[n - 4] | user[n - 3] << 8 | user[n - 2] << 16 | (uint32_t)user[n - 1] << 24);
    if (dmr_crc32_data(user, n - 4) != crc32) return -1;

    const uint8_t *h = s->held_hdr;
    int pad = ((h[0] >> 4) & 1) << 4 | (h[1] & 0xF);
    int ulen = n - 4 - pad;
    if (ulen < 5) return -1;
    int src_net = ip_prefix(user[2] >> 4), dst_net = ip_prefix(user[2] & 0xF);
    int spid = user[3], dpid = user[4], off = 5;
    if (src_net < 0 || dst_net < 0 || (spid | dpid) & 0x80) return -1;
    int sport = spid ? moto_port(spid) : (off + 2 <= ulen ? (user[off] << 8 | user[off + 1]) : -1);
    if (!spid) off += 2;
    int dport = dpid ? moto_port(dpid) : (off + 2 <= ulen ? (user[off] << 8 | user[off + 1]) : -1);
    if (!dpid) off += 2;
    if (sport < 0 || dport < 0 || off > ulen) return -1;

    int plen = ulen - off;
    int iplen = 28 + plen;
    int nb = (iplen + 4 + bd - 1) / bd;
    if (nb > MAX_BLOCKS) return -1;
    int npad = nb * bd - 4 - iplen;

    uint8_t buf[MAX_BLOCKS * MAX_BLOCK_DATA];
    memset(buf, 0, sizeof buf);
    uint8_t *ip = buf, *udp = buf + 20;
    ip[0] = 0x45; ip[2] = (uint8_t)(iplen >> 8); ip[3] = (uint8_t)iplen;
    ip[4] = user[0]; ip[5] = user[1];                       /* the compressed header keeps the IPv4 id */
    ip[8] = 64; ip[9] = 17;
    ip[12] = (uint8_t)src_net; memcpy(ip + 13, h + 5, 3);    /* addresses from the data header's IDs */
    ip[16] = (uint8_t)dst_net; memcpy(ip + 17, h + 2, 3);
    uint16_t ipck = ones_fold(ones_sum(ip, 20, 0));
    ip[10] = (uint8_t)(ipck >> 8); ip[11] = (uint8_t)ipck;
    udp[0] = (uint8_t)(sport >> 8); udp[1] = (uint8_t)sport;
    udp[2] = (uint8_t)(dport >> 8); udp[3] = (uint8_t)dport;
    udp[4] = (uint8_t)((8 + plen) >> 8); udp[5] = (uint8_t)(8 + plen);
    memcpy(udp + 8, user + off, (size_t)plen);
    uint8_t pseudo[12] = {0};
    memcpy(pseudo, ip + 12, 8); pseudo[9] = 17; pseudo[10] = udp[4]; pseudo[11] = udp[5];
    uint16_t uck = ones_fold(ones_sum(udp, 8 + plen, ones_sum(pseudo, 12, 0)));
    if (!uck) uck = 0xFFFF;
    udp[6] = (uint8_t)(uck >> 8); udp[7] = (uint8_t)uck;

    int body = nb * bd - 4;
    uint32_t c = dmr_crc32_data(buf, body);
    buf[body] = (uint8_t)c; buf[body + 1] = (uint8_t)(c >> 8); buf[body + 2] = (uint8_t)(c >> 16); buf[body + 3] = (uint8_t)(c >> 24);
    for (int k = 0; k < nb; k++) {
        if (conf) {
            uint16_t c9 = dmr_crc9(buf + k * bd, bd, k) ^ RATE34_CRC9_MASK;
            blk_out[k][0] = (uint8_t)(k << 1 | c9 >> 8);
            blk_out[k][1] = (uint8_t)c9;
        }
        memcpy(blk_out[k] + lead, buf + k * bd, (size_t)bd);
    }

    memcpy(hdr_out, h, 12);
    hdr_out[0] = (uint8_t)((h[0] & ~0x10) | ((npad >> 4) & 1) << 4);
    hdr_out[1] = (uint8_t)(SAP_IP << 4 | (npad & 0xF));
    hdr_out[8] = (uint8_t)((h[8] & 0x80) | nb);
    uint16_t hc = dmr_crc16_ccitt(hdr_out, 10) ^ HEADER_CRC_MASK;
    hdr_out[10] = (uint8_t)(hc >> 8); hdr_out[11] = (uint8_t)hc;
    LOGD(LOGN, "IPSC->HBP data: UDP/IP header restored  %u -> %u  port %d -> %d  %d bytes  %d -> %d blocks",
         u24(h + 5), u24(h + 2), sport, dport, plen, s->held_want, nb);
    return nb;
}

static void flush_held(datapath *dp, int ts, int is_private, const uint8_t src[3], const uint8_t dst[3])
{
    slot_state *s = &dp->s[ts];
    uint8_t hdr[12], blk[MAX_BLOCKS][18];
    int nb = decompress(s, hdr, blk);
    if (nb < 0) {
        LOGD(LOGN, "IPSC->HBP data: compressed UDP/IP header kept (not decodable)  %u -> %u", u24(src), u24(dst));
        memcpy(hdr, s->held_hdr, 12);
        memcpy(blk, s->held_blk, sizeof s->held_blk);
        nb = s->held_want;
    }
    send_burst(dp, ts, is_private, src, dst, DT_HEADER, s->held_cc, hdr);
    for (int k = 0; k < nb; k++) send_burst(dp, ts, is_private, src, dst, s->held_dt, s->held_cc, blk[k]);
    s->held = 0;
}

/* A burst the held message cannot take: what was held goes out as it came. */
static void release_held(datapath *dp, int ts, int is_private, const uint8_t src[3], const uint8_t dst[3])
{
    slot_state *s = &dp->s[ts];
    if (!s->held) return;
    LOGD(LOGN, "IPSC->HBP data ts=%d: compressed UDP/IP header kept (unexpected burst)", ts);
    send_burst(dp, ts, is_private, src, dst, DT_HEADER, s->held_cc, s->held_hdr);
    for (int k = 0; k < s->held_n; k++) send_burst(dp, ts, is_private, src, dst, s->held_dt, s->held_cc, s->held_blk[k]);
    s->held = 0;
}

static void drop_held(slot_state *s, int ts)
{
    if (s->held)
        LOGD(LOGN, "IPSC->HBP data ts=%d: incomplete message dropped (%d of %d blocks)", ts, s->held_n, s->held_want);
    s->held = 0;
}

void datapath_ipsc_received(datapath *dp, const uint8_t *pkt, int len)
{
    dp->rf_last = ev_now(dp->loop);
    if (len < IPSC_PAYLOAD_OFF + 8) return;
    const uint8_t *p = pkt + IPSC_PAYLOAD_OFF;
    int dt = p[0] & 0x3F, bits = p[6] << 8 | p[7];
    int size = (dt == DT_RATE34) ? 18 : 12;
    if ((dt != DT_CSBK && dt != DT_HEADER && dt != DT_RATE12 && dt != DT_RATE34) ||
        bits != size * 8 || len < IPSC_PAYLOAD_OFF + 8 + size) {
        LOGD(LOGN, "IPSC->HBP data: data type %d / %d bits not relayed", dt, bits);
        return;
    }
    if (!hbp_is_connected(dp->hb)) return;

    int ts = (pkt[GV_CALL_INFO_OFF] & TS_CALL_MSK) ? 2 : 1;
    int last = !!(pkt[GV_CALL_INFO_OFF] & END_MSK);
    int is_private = pkt[0] == PVT_DATA;
    const uint8_t *src = pkt + 6, *dst = pkt + 9;
    int cc = DEFAULT_CC;
    if ((p[5] & 0x08) && len >= IPSC_PAYLOAD_OFF + 8 + size + 2) cc = p[8 + size + 1] >> 4;

    uint8_t pdu[18];
    if (dt == DT_RATE34) {           /* IPSC moves DBSN + CRC-9 behind the 16 data octets */
        memcpy(pdu, p + 8 + 16, 2);
        memcpy(pdu + 2, p + 8, 16);
    } else {
        memcpy(pdu, p + 8, 12);
    }

    slot_state *s = &dp->s[ts];
    uint8_t key[7];
    memcpy(key, src, 3); memcpy(key + 3, dst, 3); key[6] = pkt[5];
    if (!s->up_active || memcmp(key, s->up_key, 7) != 0) {
        drop_held(s, ts);
        memcpy(s->up_key, key, 7);
        rand_bytes(s->up_stream, 4);
        s->up_seq = 0;
        s->up_active = 1;
    }

    if (dt == DT_HEADER) {
        drop_held(s, ts);
        int want = pdu[8] & 0x7F;
        LOGD(LOGN, "IPSC->HBP data header ts=%d  %u -> %u  dpf=%d sap=%d blocks=%d", ts, u24(src), u24(dst),
             pdu[0] & 0xF, pdu[1] >> 4, want);
        int dpf = pdu[0] & 0xF;
        if ((dpf == DPF_CONFIRMED || dpf == DPF_UNCONFIRMED) && (pdu[1] >> 4) == SAP_UDP_COMPRESSED && want > 0) {
            memcpy(s->held_hdr, pdu, 12);
            s->held = 1; s->held_want = want; s->held_n = 0; s->held_cc = cc; s->held_dt = 0;
        } else {
            send_burst(dp, ts, is_private, src, dst, dt, cc, pdu);
        }
    } else if (s->held && (s->held_n ? dt == s->held_dt : block_data(confirmed(s->held_hdr), dt) > 0)) {
        s->held_dt = dt;
        memcpy(s->held_blk[s->held_n++], pdu, (size_t)size);
        if (s->held_n == s->held_want) flush_held(dp, ts, is_private, src, dst);
    } else {
        release_held(dp, ts, is_private, src, dst);
        send_burst(dp, ts, is_private, src, dst, dt, cc, pdu);
    }

    if (last) {
        drop_held(s, ts);
        s->up_active = 0;
    }
}

/* ---------------- HBP -> IPSC ---------------- */

int datapath_is_data_frame(const uint8_t *dmrd, int len)
{
    if (len < DMRD_CORE_LEN) return 0;
    int flags = dmrd[DMRD_FLAGS_OFF], dt = flags & HBPF_DTYPE_MASK;
    return (flags & HBPF_FRAMETYPE_MASK) == HBPF_FRAMETYPE_DATASYNC &&
           (dt == DT_CSBK || dt == DT_HEADER || dt == DT_RATE12 || dt == DT_RATE34);
}

static void dn_burst(datapath *dp, const uint8_t *dmrd, const uint8_t *pdu, int cc)
{
    int flags = dmrd[DMRD_FLAGS_OFF];
    int dt = flags & HBPF_DTYPE_MASK;
    int ts = (flags & HBPF_TGID_TS2) ? 2 : 1;
    int is_private = !!(flags & HBPF_TGID_CALL_P);
    int size = dt == DT_RATE34 ? 18 : 12;
    const uint8_t *src = dmrd + DMRD_SRC_OFF, *dst = dmrd + DMRD_DST_OFF, *stream = dmrd + DMRD_STREAM_OFF;

    slot_state *s = &dp->s[ts];
    int new_stream = !s->dn_active || memcmp(stream, s->dn_stream, 4) != 0;
    if ((dt == DT_RATE12 || dt == DT_RATE34) && (new_stream || s->dn_blocks_left <= 0)) {
        LOGD(LOGN, "HBP->IPSC data ts=%d  %u -> %u: block without its header — dropped", ts, u24(src), u24(dst));
        return;
    }
    int first = 0;
    if (new_stream) {
        memcpy(s->dn_stream, stream, 4);
        s->dn_call_seq++;
        rand_bytes(s->dn_ctrl, 4);
        uint8_t r[6]; rand_bytes(r, 6);
        s->dn_rtp_seq = (uint16_t)(r[0] << 8 | r[1]);
        s->dn_rtp_ts = (uint32_t)r[2] << 24 | (uint32_t)r[3] << 16 | (uint32_t)r[4] << 8 | r[5];
        s->dn_blocks_left = 0;
        s->dn_active = 1;
        first = 1;
    }

    int last;
    if (dt == DT_HEADER) {
        s->dn_blocks_left = pdu[8] & 0x7F;
        last = s->dn_blocks_left == 0;
        LOGD(LOGN, "HBP->IPSC data header ts=%d  %u -> %u  dpf=%d sap=%d blocks=%d", ts, u24(src), u24(dst),
             pdu[0] & 0xF, pdu[1] >> 4, s->dn_blocks_left);
    } else if (dt == DT_CSBK) {
        last = (pdu[0] & 0x3F) != CSBKO_PREAMBLE;
    } else {
        last = --s->dn_blocks_left == 0;
    }

    uint8_t out[DN_PKT_MAX];
    uint32_t mid = dp->cfg->ipsc_master_id;
    int n = 0;
    out[n++] = is_private ? PVT_DATA : GROUP_DATA;
    out[n++] = (uint8_t)(mid >> 24); out[n++] = (uint8_t)(mid >> 16); out[n++] = (uint8_t)(mid >> 8); out[n++] = (uint8_t)mid;
    out[n++] = s->dn_call_seq;
    memcpy(out + n, src, 3); n += 3;
    memcpy(out + n, dst, 3); n += 3;
    out[n++] = 0x01;                                    /* call priority, as the repeater sends it */
    memcpy(out + n, s->dn_ctrl, 4); n += 4;
    out[n++] = (uint8_t)((ts == 2 ? TS_CALL_MSK : 0) | (last ? END_MSK : 0));
    out[n++] = 0x80;                                    /* RTP v2 */
    out[n++] = (uint8_t)((first ? 0x80 : 0) | (last ? 0x5E : 0x5D));
    out[n++] = (uint8_t)(s->dn_rtp_seq >> 8); out[n++] = (uint8_t)s->dn_rtp_seq;
    out[n++] = (uint8_t)(s->dn_rtp_ts >> 24); out[n++] = (uint8_t)(s->dn_rtp_ts >> 16);
    out[n++] = (uint8_t)(s->dn_rtp_ts >> 8); out[n++] = (uint8_t)s->dn_rtp_ts;
    memset(out + n, 0, 4); n += 4;                      /* SSRC */
    s->dn_rtp_seq++;
    s->dn_rtp_ts += 480;

    int words = (2 + 2 + size + 2) / 2;                 /* bytes after the length field, no RSSI */
    out[n++] = (uint8_t)dt;
    out[n++] = ts == 2 ? 0x80 : 0x00;
    out[n++] = (uint8_t)(words >> 8); out[n++] = (uint8_t)words;
    out[n++] = 0x00;                                    /* no RSSI */
    out[n++] = 0x0A;                                    /* slot type present, data sync */
    out[n++] = 0x00; out[n++] = (uint8_t)(size * 8);
    if (dt == DT_RATE34) {
        memcpy(out + n, pdu + 2, 16); memcpy(out + n + 16, pdu, 2);
    } else {
        memcpy(out + n, pdu, 12);
    }
    n += size;
    out[n++] = 0x00;
    out[n++] = (uint8_t)(cc << 4 | dt);
    if (last) s->dn_active = 0;

    if (s->q_len == DN_QUEUE) { LOGW(LOGN, "HBP->IPSC data ts=%d: queue full — burst dropped", ts); return; }
    dn_pkt *q = &s->q[(s->q_head + s->q_len++) % DN_QUEUE];
    memcpy(q->b, out, (size_t)n);
    q->n = n; q->ts = ts; q->dt = dt; q->last = last;
    q->queued = ev_now(dp->loop);
    dn_pump(s);
}

static void release_preambles(datapath *dp, slot_state *s)
{
    for (int i = 0; i < s->pre_n; i++) dn_burst(dp, s->pre[i].dmrd, s->pre[i].pdu, s->pre[i].cc);
    s->pre_n = 0;
}

void datapath_hbp_received(datapath *dp, const uint8_t *dmrd, int len)
{
    if (len < DMRD_CORE_LEN || !ipsc_has_peers(dp->ip)) return;
    int flags = dmrd[DMRD_FLAGS_OFF];
    int dt = flags & HBPF_DTYPE_MASK;
    int ts = (flags & HBPF_TGID_TS2) ? 2 : 1;

    dmr_bit fb[264], info[196];
    dmr_bytes_to_bits(dmrd + DMRD_PAYLOAD_OFF, 33, fb);
    memcpy(info, fb, 98);
    memcpy(info + 98, fb + 166, 98);
    int cc = fb[98] << 3 | fb[99] << 2 | fb[100] << 1 | fb[101];

    uint8_t pdu[18];
    if (dt == DT_RATE34) {
        if (dmr_trellis34_decode(info, pdu) != 0) {
            LOGD(LOGN, "HBP->IPSC data ts=%d: rate 3/4 block does not decode — dropped", ts);
            return;
        }
    } else {
        dmr_bptc_decode_data(info, pdu);
    }

    slot_state *s = &dp->s[ts];
    if (s->pre_n && memcmp(dmrd + DMRD_STREAM_OFF, s->pre[0].dmrd + DMRD_STREAM_OFF, 4) != 0)
        release_preambles(dp, s);
    if (dt == DT_CSBK && (pdu[0] & 0x3F) == CSBKO_PREAMBLE) {
        if (s->pre_n == MAX_HELD_PRE) release_preambles(dp, s);
        held_pre *h = &s->pre[s->pre_n++];
        memcpy(h->dmrd, dmrd, DMRD_CORE_LEN); memcpy(h->pdu, pdu, 12); h->cc = cc;
        return;
    }
    if (s->pre_n && dt == DT_HEADER && (pdu[0] & 0xF) == DPF_RESPONSE) {
        /* A MOTOTRBO radio ignores a link ACK/NACK that comes after a preamble (field-tested:
         * with one it retries the message, without it the ACK is taken), as its own ACKs carry none. */
        LOGD(LOGN, "HBP->IPSC data ts=%d: %d preamble(s) ahead of a response header dropped", ts, s->pre_n);
        s->pre_n = 0;
    }
    release_preambles(dp, s);
    dn_burst(dp, dmrd, pdu, cc);
}
