/* test_data.c — IPSC data <-> HBP data bursts, through a real IPSC master on loopback.
 *
 * Uplink vectors are a MOTOTRBO ARS registration as a repeater sent it (captures/data-ipsc.pcap,
 * radio 7140023 -> 900999, confirmed rate 3/4 with a compressed UDP/IP header).  The expected
 * IPv4 datagram is what D-APRS's confirmed_rx.py reassembled from this translator's output.
 * Downlink vectors are D-APRS's own replies (confirmed_rx / motorola_confirmed34 builders), with
 * their PDUs as ok-dmrlib decodes them.  Trellis/BPTC known answers are ok-dmrlib encodings. */
#include <stdio.h>
#include <string.h>
#include <poll.h>
#include "../src/data.h"
#include "../src/translate.h"
#include "../src/config.h"
#include "../src/eventloop.h"
#include "../src/ipsc.h"
#include "../src/ipsc_const.h"
#include "../src/hbp_const.h"
#include "../src/net.h"
#include "../src/dmr/dmr.h"

#define MAXCAP 32
static uint8_t dmrd[MAXCAP][DMRD_LEN];
static int ndmrd = 0;

int  hbp_is_connected(struct hbp *hb) { (void)hb; return 1; }
void hbp_activate(struct hbp *hb) { (void)hb; }
void hbp_deactivate(struct hbp *hb) { (void)hb; }
void hbp_send_dmrd(struct hbp *hb, const uint8_t *d, int n)
{
    (void)hb;
    if (ndmrd < MAXCAP && n == DMRD_LEN) memcpy(dmrd[ndmrd++], d, DMRD_LEN);
}

static const char *ARS_UP[4] = {   /* CSBK preamble, data header (SAP 3), 2 confirmed rate 3/4 blocks */
    "84000ae511146cf2b70dbf8701000064360080dda5d1346ea7b0000000000340000a800a0060bd0080030dbf876cf2b77bc900135e61",
    "84000ae511146cf2b70dbf87010000643600805da5d2346ea990000000000640000a800a006043390dbf876cf2b78258c91a00165e68",
    "84000ae511146cf2b70dbf87010000643600805da5d3346eab70000000000840000d800a0090a747016161000cf0200737313430303200c700185e6d",
    "84000ae511146cf2b70dbf87010000643640805ea5d4346ead50000000000840000d800a00903300000000000000000000001a28af34036d00185e65",
};
static const char *ARS_IPV4 =
    "4500002aa7470000401107c40c6cf2b70d0dbf870fa50fa5001689c4000cf02007373134303032330000";

/* Unconfirmed LRRP location report, header (SAP 3) + 4 rate 1/2 blocks; position replaced with 10 N 90 W */
static const char *LRRP_UP[5] = {
    "84000ae511ed6cf2b70dbf870100000d7b0080ddd894102cda73000000000640000a800a0060023b0dbf876cf2b78400637900164e3e",
    "84000ae511ed6cf2b70dbf870100000d7b00805dd895102cdc53000000000740000a800a0060a7b0015f5f0d1a22047fffff00174e3b",
    "84000ae511ed6cf2b70dbf870100000d7b00805dd896102cde33000000000740000a800a0060ff690e38e38ec0000000804900174e46",
    "84000ae511ed6cf2b70dbf870100000d7b00805dd897102ce013000000000740000a800a00606f6c0023560270000100000000174e52",
    "84000ae511ed6cf2b70dbf870100000d7b40805ed898102ce1f3000000000740000a800a0060000000000000000045e231e000174e57",
};
static const char *LRRP_IPV4 =
    "45000038a7b000004011074d0c6cf2b70d0dbf870fa10fa1002459250d1a22047fffffff690e38e38ec000000080496f6c00235602700001";

static const char *REPLY_DN[6] = { /* link ACK (preamble + response header), ARS response (header + 3 blocks) */
    "444d5244000dbf876cf2b7000dbf87e30728691a4b9404a9b5207b746e79de50c4cdff57d75df5dacbe21e806cc894e0a4580997f60000",
    "444d5244010dbf876cf2b7000dbf87e60728691a37517ed56e548900a15029c7458dff57d75df5d3393090d0e5b409238b938d356c0000",
    "444d5244020dbf876cf2b7000dbf87e645e4d45237297f336cfe892aafd0354f458dff57d75df5d33acc909ce0540e07828385b5e60000",
    "444d5244030dbf876cf2b7000dbf87e845e4d4522c32a22f4e35b21222722571463dff57d75df5da87e3e72f2ea00768a3227584040000",
    "444d5244040dbf876cf2b7000dbf87e845e4d45222c4b612b625e22071fc2572c63dff57d75df5da868093507d728fcbd12da2f25d0000",
    "444d5244050dbf876cf2b7000dbf87e845e4d4522e7222222205bdaf222222d2c63dff57d75df5da85732222222842ca22222223fc0000",
};
static const char *REPLY_PDU[6] = {
    "bd0080016cf2b70dbf87fcf1", "01406cf2b70dbf87000d8814", "434a6cf2b70dbf87838844fa",
    "012f45010022000100004011af110d0dbf87", "029d0c6cf2b70fa50fa5000e454c0002bf01", "05791080000000000000000000005c39a3d7",
};
static const int REPLY_LAST[6] = { 0, 1, 0, 0, 0, 1 };

static const char *TRELLIS_IN = "003aa746016161000cf02007373134303032";
static const char *TRELLIS_OUT =
    "00101100001000101000001010011010010001100000011010100010101001011000011000101110001001000111101100"
    "10001011111011011011101101111100101111011011111011000110011111000010101011111001000100001000011011";
static const char *BPTC_IN = "43390dbf876cf2b78248db2b";
static const char *BPTC_OUT =
    "00010111001011110010101110100101000001111001111110110100111000011110011111010001011010100110111111"
    "10000110011101000101000110011100100101001101101011000000111000011000001110100001101011110010111101";

static int checks, fails;
static void check(int ok, const char *what)
{
    checks++;
    if (!ok) { fails++; fprintf(stderr, "FAIL: %s\n", what); }
}

static int unhex(const char *h, uint8_t *out)
{
    int n = 0;
    while (h[0] && h[1] && sscanf(h, "%2hhx", &out[n]) == 1) { n++; h += 2; }
    return n;
}

static void stop_cb(ev_loop *loop, void *ud) { (void)ud; ev_stop(loop); }
static void pump(ev_loop *loop) { ev_timer_after(loop, 0.05, stop_cb, loop); ev_run(loop); }

static const Config *g_cfg;
static int g_fd;
static ev_loop *g_loop;

static void send_ipsc(const char *hex)
{
    uint8_t b[128]; int n = unhex(hex, b);
    udp_sendto(g_fd, b, n, g_cfg->ipsc_bind_ip, g_cfg->ipsc_bind_port);
    pump(g_loop);
}

/* Next data packet the master sent to the fake repeater within `wait` s, skipping keepalive/registration
 * replies; the loop runs meanwhile, since queued bursts go out from a timer. */
static int recv_data_within(uint8_t *buf, double wait)
{
    double end = ev_now(g_loop) + wait;
    for (;;) {
        struct pollfd pfd = { .fd = g_fd, .events = POLLIN };
        if (poll(&pfd, 1, 0) > 0) {
            char host[64]; int port;
            int n = udp_recvfrom(g_fd, buf, 256, host, &port);
            if (n > 0 && (buf[0] == PVT_DATA || buf[0] == GROUP_DATA)) return n;
            continue;
        }
        if (ev_now(g_loop) >= end) return -1;
        ev_timer_after(g_loop, 0.01, stop_cb, NULL);
        ev_run(g_loop);
    }
}

static int recv_data(uint8_t *buf) { return recv_data_within(buf, 1.0); }

static void burst_info(const uint8_t *frame, dmr_bit info[196])
{
    dmr_bit fb[264];
    dmr_bytes_to_bits(frame + DMRD_PAYLOAD_OFF, 33, fb);
    memcpy(info, fb, 98);
    memcpy(info + 98, fb + 166, 98);
}

static int bits_equal(const dmr_bit *b, const char *s, int n)
{
    for (int i = 0; i < n; i++) if (b[i] != s[i] - '0') return 0;
    return 1;
}

static void test_primitives(void)
{
    uint8_t in[18], back[18]; dmr_bit bits[196];
    unhex(TRELLIS_IN, in);
    dmr_trellis34_encode(in, bits);
    check(bits_equal(bits, TRELLIS_OUT, 196), "trellis 3/4 encode matches ok-dmrlib");
    check(dmr_trellis34_decode(bits, back) == 0 && !memcmp(back, in, 18), "trellis 3/4 decode round trip");
    bits[7] ^= 1;
    check(dmr_trellis34_decode(bits, back) != 0, "trellis 3/4 decode rejects a corrupted codeword");
    unhex(BPTC_IN, in);
    dmr_bptc_encode_data(in, bits);
    check(bits_equal(bits, BPTC_OUT, 196), "BPTC(196,96) data encode matches ok-dmrlib");
    dmr_bptc_decode_data(bits, back);
    check(!memcmp(back, in, 12), "BPTC(196,96) data decode round trip");
    check(dmr_slot_type_encode(1, 1) == 0x11B8C, "slot type CC1/voice header matches DMR_SLOT_TYPE_VHEAD");
    check((dmr_crc16_ccitt(in, 10) ^ 0xCCCC) == 0xDB2B, "data header CRC-16 of the captured header");
    uint8_t b1[16], u[28];
    unhex("a747016161000cf02007373134303032", b1);
    check((dmr_crc9(b1, 16, 0) ^ 0x1FF) == 0x0C7, "CRC-9 of the captured first block");
    memcpy(u, b1, 16); unhex("330000000000000000000000", u + 16);
    check(dmr_crc32_data(u, 28) == 0x34AF281A, "CRC-32 of the captured message");
}

/* Reassemble the confirmed rate 3/4 message in dmrd[first..] the way a receiver would. */
static int reassemble(int first, uint8_t *user, int *nblocks)
{
    dmr_bit info[196]; uint8_t hdr[12];
    burst_info(dmrd[first], info);
    dmr_bptc_decode_data(info, hdr);
    int nb = hdr[8] & 0x7F, pad = ((hdr[0] >> 4) & 1) << 4 | (hdr[1] & 0xF);
    *nblocks = nb;
    if ((dmr_crc16_ccitt(hdr, 10) ^ 0xCCCC) != (hdr[10] << 8 | hdr[11])) return -1;
    int conf = (hdr[0] & 0xF) == 3;
    if ((hdr[1] >> 4) != 4 || (conf && ((hdr[9] >> 4) & 7) != 5)) return -1;
    int n = 0;
    for (int k = 0; k < nb; k++) {
        uint8_t blk[18];
        burst_info(dmrd[first + 1 + k], info);
        if (conf) {
            if ((dmrd[first + 1 + k][DMRD_FLAGS_OFF] & 0x0F) != 8) return -1;
            if (dmr_trellis34_decode(info, blk) != 0 || blk[0] >> 1 != k) return -1;
            if ((dmr_crc9(blk + 2, 16, k) ^ 0x1FF) != ((blk[0] & 1) << 8 | blk[1])) return -1;
            memcpy(user + n, blk + 2, 16); n += 16;
        } else {
            if ((dmrd[first + 1 + k][DMRD_FLAGS_OFF] & 0x0F) != 7) return -1;
            dmr_bptc_decode_data(info, blk);
            memcpy(user + n, blk, 12); n += 12;
        }
    }
    uint32_t c = (uint32_t)(user[n - 4] | user[n - 3] << 8 | user[n - 2] << 16 | (uint32_t)user[n - 1] << 24);
    if (dmr_crc32_data(user, n - 4) != c) return -1;
    return n - 4 - pad;
}

static void test_uplink(void)
{
    uint8_t user[512], want[64]; int nb;

    ndmrd = 0;
    for (int i = 0; i < 4; i++) send_ipsc(ARS_UP[i]);
    check(ndmrd == 5, "ARS: preamble + header + 3 blocks reach HBP");
    if (ndmrd == 5) {
        int types_ok = 1;
        static const int TYPES[5] = { 3, 6, 8, 8, 8 };
        for (int i = 0; i < 5; i++)
            types_ok &= dmrd[i][DMRD_FLAGS_OFF] == (HBPF_TGID_CALL_P | HBPF_FRAMETYPE_DATASYNC | TYPES[i]) &&
                        !memcmp(dmrd[i] + DMRD_STREAM_OFF, dmrd[0] + DMRD_STREAM_OFF, 4);
        check(types_ok, "ARS: private TS1 data bursts on one HBP stream");
        dmr_bit info[196]; uint8_t csbk[12], cap[12];
        burst_info(dmrd[0], info);
        dmr_bptc_decode_data(info, csbk);
        unhex("bd0080030dbf876cf2b77bc9", cap);
        check(!memcmp(csbk, cap, 12), "ARS: CSBK preamble passed through unchanged");
        int ulen = reassemble(1, user, &nb);
        int wlen = unhex(ARS_IPV4, want);
        check(ulen == wlen && !memcmp(user, want, (size_t)wlen),
              "ARS: UDP/IP header restored to the datagram D-APRS accepts");
    }

    /* Unconfirmed rate 1/2 (an LRRP location report) gets its UDP/IP header restored too */
    ndmrd = 0;
    for (int i = 0; i < 5; i++) send_ipsc(LRRP_UP[i]);
    int types12 = ndmrd == 6;
    for (int i = 0; types12 && i < 6; i++) types12 &= (dmrd[i][DMRD_FLAGS_OFF] & 0x0F) == (i ? 7 : 6);
    check(types12, "LRRP: header + 5 rate 1/2 blocks reach HBP");
    if (types12) {
        int ulen = reassemble(0, user, &nb);
        int wlen = unhex(LRRP_IPV4, want);
        check(ulen == wlen && !memcmp(user, want, (size_t)wlen), "LRRP: unconfirmed report restored to IPv4/UDP 4001");
    }

    /* An uncompressed IP header (SAP 4) goes out as it arrives, blocks untouched */
    ndmrd = 0;
    char hdr4[128]; strcpy(hdr4, ARS_UP[1]);
    uint8_t h[12]; unhex(hdr4 + 76, h);
    h[1] = 0x49;
    uint16_t c = dmr_crc16_ccitt(h, 10) ^ 0xCCCC; h[10] = (uint8_t)(c >> 8); h[11] = (uint8_t)c;
    for (int i = 0; i < 12; i++) sprintf(hdr4 + 76 + 2 * i, "%02x", h[i]);
    hdr4[76 + 24] = '0';   /* sprintf's NUL lands on the slot type: restore it */
    send_ipsc(ARS_UP[0]);
    send_ipsc(hdr4);
    check(ndmrd == 2, "SAP 4: header sent on arrival, not held");
    send_ipsc(ARS_UP[2]); send_ipsc(ARS_UP[3]);
    check(ndmrd == 4, "SAP 4: blocks passed through, no re-blocking");

    /* A block failing its CRC-9: the original message is forwarded rather than lost */
    ndmrd = 0;
    char bad[160]; strcpy(bad, ARS_UP[2]);
    bad[90] = bad[90] == '0' ? '1' : '0';
    send_ipsc(ARS_UP[0]); send_ipsc(ARS_UP[1]); send_ipsc(bad); send_ipsc(ARS_UP[3]);
    check(ndmrd == 4, "bad CRC-9: original header + 2 blocks forwarded");
    if (ndmrd == 4) {
        dmr_bit info[196]; uint8_t hd[12];
        burst_info(dmrd[1], info);
        dmr_bptc_decode_data(info, hd);
        check((hd[1] >> 4) == 3, "bad CRC-9: header still declares the compressed header");
    }

    /* A burst the held message cannot take: the held header still goes out, then that burst */
    ndmrd = 0;
    send_ipsc(ARS_UP[1]); send_ipsc(ARS_UP[0]);
    check(ndmrd == 2 && (dmrd[0][DMRD_FLAGS_OFF] & 0x0F) == 6 && (dmrd[1][DMRD_FLAGS_OFF] & 0x0F) == 3,
          "unexpected burst: held header released ahead of it");

    /* The last block never arrives: nothing of the held message is sent */
    ndmrd = 0;
    char next[160]; strcpy(next, ARS_UP[0]);
    next[11] = '5';   /* call sequence 0x15: the radio's next attempt */
    send_ipsc(ARS_UP[0]); send_ipsc(ARS_UP[1]); send_ipsc(ARS_UP[2]); send_ipsc(next);
    check(ndmrd == 2, "lost block: incomplete message dropped, next attempt goes on");
    check(ndmrd == 2 && memcmp(dmrd[0] + DMRD_STREAM_OFF, dmrd[1] + DMRD_STREAM_OFF, 4) != 0,
          "next attempt: its own HBP stream");
}

static void test_downlink(datapath *dp)
{
    uint8_t pkt[256], f[64], pre[64];
    int ok_pdu = 1, ok_hdr = 1, ok_last = 1, ok_rtp = 1;
    double t_prev = 0, min_gap = 1;

    /* The radio's message has just ended: D-APRS answers with preamble + response header */
    double t_rf = ev_now(g_loop);
    send_ipsc(ARS_UP[0]);
    int pn = unhex(REPLY_DN[0], pre);
    datapath_hbp_received(dp, pre, pn);
    int fn = unhex(REPLY_DN[1], f);
    datapath_hbp_received(dp, f, fn);
    check(recv_data_within(pkt, 0.3) < 0, "link ACK: nothing sent while the repeater was just active");
    int n = recv_data(pkt);
    double waited = ev_now(g_loop) - t_rf;
    check(n > 0 && waited >= 0.5 && waited < 0.7, "link ACK: sent 500 ms after the repeater's last data packet");
    check(n > 0 && (pkt[30] & 0x3F) == 6, "link ACK: the response header goes out, its preamble does not");
    check(n > 0 && pkt[19] == 0xDE && (pkt[GV_CALL_INFO_OFF] & END_MSK), "link ACK: one burst, first and last, like the radio's own");
    int seq_ack = n > 0 ? pkt[5] : -1;
    check(recv_data_within(pkt, 0.2) < 0, "link ACK: nothing else sent");

    /* ARS response: a preamble ahead of a data header is sent, ahead of it, in the same call */
    memcpy(pre + DMRD_STREAM_OFF, f + DMRD_STREAM_OFF, 4);
    unhex(REPLY_DN[2], f);
    memcpy(pre + DMRD_STREAM_OFF, f + DMRD_STREAM_OFF, 4);
    static const int ORDER[5] = { -1, 2, 3, 4, 5 };   /* -1: the preamble */
    int seq_ars = -1, same_call = 1;
    for (int i = 0; i < 5; i++) {
        const uint8_t *in = pre; int in_len = pn; const char *pdu_hex = REPLY_PDU[0]; int last = 0;
        if (ORDER[i] >= 0) {
            in_len = unhex(REPLY_DN[ORDER[i]], f); in = f; pdu_hex = REPLY_PDU[ORDER[i]]; last = REPLY_LAST[ORDER[i]];
        }
        datapath_hbp_received(dp, in, in_len);
        if (i == 0) continue;            /* held until the header shows what follows */
        for (int k = i == 1 ? 0 : 1; k < 2; k++) {
            int m = recv_data(pkt);
            if (m < 0) { check(0, "ARS response: an IPSC packet per HBP burst"); return; }
            int dt = pkt[30] & 0x3F, size = dt == 8 ? 18 : 12;
            const char *want_hex = k == 0 ? REPLY_PDU[0] : pdu_hex;
            int want_last = k == 0 ? 0 : last, want_first = k == 0;
            uint8_t want[18], got[18];
            unhex(want_hex, want);
            const uint8_t *p = pkt + 30;
            if (dt == 8) { memcpy(got, p + 8 + 16, 2); memcpy(got + 2, p + 8, 16); } else memcpy(got, p + 8, 12);
            ok_pdu &= (p[6] << 8 | p[7]) == size * 8 && !memcmp(got, want, (size_t)size) && (p[2] << 8 | p[3]) * 2 == m - 34;
            ok_hdr &= pkt[0] == PVT_DATA && !memcmp(pkt + 6, in + DMRD_SRC_OFF, 3) && !memcmp(pkt + 9, in + DMRD_DST_OFF, 3) &&
                      (pkt[GV_CALL_INFO_OFF] & TS_CALL_MSK) && (p[1] & 0x80);
            ok_last &= !!(pkt[GV_CALL_INFO_OFF] & END_MSK) == want_last;
            ok_rtp &= pkt[19] == (want_first ? 0xDD : want_last ? 0x5E : 0x5D);
            if (seq_ars < 0) seq_ars = pkt[5];
            same_call &= pkt[5] == seq_ars;
            double t = ev_now(g_loop);
            if (t_prev && t - t_prev < min_gap) min_gap = t - t_prev;
            t_prev = t;
        }
    }
    check(ok_pdu, "ARS response: preamble, header and blocks arrive intact in the IPSC data layout");
    check(ok_hdr, "ARS response: private data, src/dst and TS2 kept");
    check(ok_last, "ARS response: END only on the last block");
    check(ok_rtp, "ARS response: RTP marks first and last burst");
    check(same_call && seq_ack != seq_ars, "ARS response: one IPSC call, apart from the link ACK's");
    check(min_gap >= 0.055, "ARS response: bursts of a slot go out one TDMA frame apart");

    /* Blocks whose header never arrived (the master dropped it) are of no use to the radio */
    fn = unhex(REPLY_DN[4], f);
    datapath_hbp_received(dp, f, fn);
    check(recv_data_within(pkt, 0.3) < 0, "a block without its header is not sent");
}

static void test_classification(void)
{
    uint8_t f[DMRD_LEN] = {0};
    int ok = 1;
    for (int dt = 0; dt < 16; dt++) {
        f[DMRD_FLAGS_OFF] = (uint8_t)(HBPF_FRAMETYPE_DATASYNC | dt);
        ok &= datapath_is_data_frame(f, DMRD_LEN) == (dt == 3 || dt == 6 || dt == 7 || dt == 8);
    }
    f[DMRD_FLAGS_OFF] = HBPF_FRAMETYPE_VOICESYNC; ok &= !datapath_is_data_frame(f, DMRD_LEN);
    f[DMRD_FLAGS_OFF] = HBPF_FRAMETYPE_VOICE | 3; ok &= !datapath_is_data_frame(f, DMRD_LEN);
    check(ok, "only CSBK, data header and rate 1/2, 3/4 frames are data; voice and LC header/terminator are not");
}

int main(void)
{
    static Config cfg; char err[4096];
    if (config_load("tests/data.toml", &cfg, err, sizeof err)) { fprintf(stderr, "config: %s\n", err); return 2; }
    g_cfg = &cfg;
    g_loop = ev_new();
    translator *tr = translator_new(&cfg, g_loop);
    ipsc *ip = ipsc_new(&cfg, tr, g_loop);
    translator_set_protocols(tr, ip, (struct hbp *)1);
    datapath *dp = datapath_new(&cfg, g_loop);
    datapath_set_protocols(dp, ip, (struct hbp *)1, tr);
    ipsc_set_datapath(ip, dp);
    if (ipsc_start(ip) != 0) { fprintf(stderr, "ipsc_start failed\n"); return 2; }
    g_fd = udp_socket();

    uint8_t reg[10] = { MASTER_REG_REQ, 0x00, 0x0a, 0xe5, 0x11, 0x6A, 0, 0, 0, 0 };   /* repeater 714001 */
    udp_sendto(g_fd, reg, sizeof reg, cfg.ipsc_bind_ip, cfg.ipsc_bind_port);
    pump(g_loop);

    test_primitives();
    test_uplink();
    test_downlink(dp);
    test_classification();

    printf("IPSC data <-> HBP: %d checks, %d failures\n", checks, fails);
    ipsc_free(ip); datapath_free(dp); translator_free(tr); ev_free(g_loop);
    return fails ? 1 : 0;
}
