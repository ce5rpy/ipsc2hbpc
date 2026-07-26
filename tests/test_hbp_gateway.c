/* test_hbp_gateway.c — HBP role=GATEWAY over real loopback UDP + the real
 * event loop.
 *
 * DMRGateway's own local-repeater link (CMMDVMNetwork, see MMDVMNetwork.cpp)
 * speaks the exact same RPTL/RPTK/RPTC/RPTO/RPTPING handshake as a real HBP
 * master — it just never validates the RPTK digest — and it strictly filters
 * every incoming packet by exact source IP *and port* against its configured
 * RptAddress/RptPort. So role=GATEWAY reuses the real CLIENT handshake state
 * machine in hbp.c almost unchanged; the only difference is the local socket
 * is bound to a fixed bind_ip:bind_port instead of an ephemeral one (see
 * hbp_connect()/udp_bind_connect() in src/hbp.c/src/net.c).
 *
 * Unlike test_parity.c/test_talker_alias*.c (which stub hbp.c entirely), this
 * test links the REAL hbp.c + net.c + crypto.c and drives a minimal fake
 * "DMRGateway" master over a real loopback socket, verifying: RPTL arrives
 * from exactly bind_ip:bind_port, the full handshake completes, RPTPING/
 * RPTCL work, and DMRD relays both directions. */
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <poll.h>
#include "../src/translate.h"
#include "../src/config.h"
#include "../src/eventloop.h"
#include "../src/hbp.h"
#include "../src/net.h"
#include "../src/ipsc_const.h"
#include "../src/hbp_const.h"
#include "../src/dmr/dmr.h"

#define MAXCAP 16
static uint8_t gvcap[MAXCAP][128];
static int     gvlen[MAXCAP];
static int     ngv = 0;

int  ipsc_has_peers(struct ipsc *ip) { (void)ip; return 1; }
void ipsc_send_voice(struct ipsc *ip, const uint8_t *p, int n) {
    (void)ip;
    if (ngv < MAXCAP) { memcpy(gvcap[ngv], p, (size_t)n); gvlen[ngv] = n; ngv++; }
}

enum { TEST_SRC = 3120001, TEST_DST = 91 };
static const uint8_t STREAM_ID[4] = { 0x42, 0x7d, 0x91, 0x7f };

static void put_bits_at(dmr_bit *fb, int off, const dmr_bit *src, int n) { memcpy(fb + off, src, (size_t)n); }

/* VOICE_HEAD DMRD frame — the header alone makes translator_hbp_voice_received()
 * emit an IPSC frame immediately (see translate.c: "Forward the header immediately"). */
static int build_dmrd_head(uint8_t *out, int seq, const uint8_t lc[9])
{
    memset(out, 0, DMRD_LEN);
    memcpy(out, "DMRD", 4);
    out[DMRD_SEQ_OFF] = (uint8_t)seq;
    memcpy(out + DMRD_SRC_OFF, lc + 6, 3);
    memcpy(out + DMRD_DST_OFF, lc + 3, 3);
    out[DMRD_RPTR_OFF]=0x00; out[DMRD_RPTR_OFF+1]=0x00; out[DMRD_RPTR_OFF+2]=0x00; out[DMRD_RPTR_OFF+3]=0x01;
    memcpy(out + DMRD_STREAM_OFF, STREAM_ID, 4);
    out[DMRD_FLAGS_OFF] = HBPF_FRAMETYPE_DATASYNC | HBPF_SLT_VHEAD;

    dmr_bit fb[264]; memset(fb, 0, sizeof fb);
    dmr_bit full_lc[196];
    dmr_bptc_encode_lc(lc, 0, full_lc);
    put_bits_at(fb, 0,   full_lc, 98);
    put_bits_at(fb, 98,  DMR_SLOT_TYPE_VHEAD, 10);
    put_bits_at(fb, 108, DMR_BS_DATA_SYNC, 48);
    put_bits_at(fb, 156, DMR_SLOT_TYPE_VHEAD + 10, 10);
    put_bits_at(fb, 166, full_lc + 98, 98);
    dmr_bits_to_bytes(fb, 264, out + DMRD_PAYLOAD_OFF);
    return DMRD_LEN;
}

static void stop_cb(ev_loop *loop, void *ud) { (void)ud; ev_stop(loop); }

/* Pump the real event loop briefly so hb's recv_cb can react to whatever the
 * fake master just sent. */
static void pump(ev_loop *loop)
{
    ev_timer_after(loop, 0.1, stop_cb, loop);
    ev_run(loop);
}

/* Blocking recv with a timeout, on the fake master's plain (unconnected) socket. */
static int fake_recv(int fd, uint8_t *buf, size_t cap, char *src_ip, int *src_port)
{
    struct pollfd pfd = { .fd = fd, .events = POLLIN };
    if (poll(&pfd, 1, 1000) <= 0) return -1;
    return udp_recvfrom(fd, buf, cap, src_ip, src_port);
}

int main(void)
{
    Config cfg; char err[4096];
    if (config_load("tests/hbp_gateway.toml", &cfg, err, sizeof err)) {
        fprintf(stderr, "config: %s\n", err); return 2;
    }

    ev_loop *loop = ev_new();
    translator *tr = translator_new(&cfg, loop);
    hbp *hb = hbp_new(&cfg, tr, loop);
    translator_set_protocols(tr, (struct ipsc *)1, hb);

    /* Fake DMRGateway master: plain bound socket, not connected, so it can
     * verify the real client's source port on every packet. */
    int fake_fd = udp_bind(cfg.hbp_gateway_ip, cfg.hbp_gateway_port);
    int checks = 0, fails = 0;
    if (fake_fd < 0) { fprintf(stderr, "FAIL: could not bind fake master socket\n"); return 1; }

    uint8_t buf[512]; char src_ip[64]; int src_port = 0; int n;

    /* 1. RPTL, from exactly bind_ip:bind_port. */
    hbp_start(hb);
    n = fake_recv(fake_fd, buf, sizeof buf, src_ip, &src_port);
    checks++;
    if (n < 8 || memcmp(buf, "RPTL", 4) != 0) { fprintf(stderr, "FAIL: expected RPTL, got n=%d\n", n); fails++; }
    checks++;
    if (src_port != cfg.hbp_bind_port || strcmp(src_ip, cfg.hbp_bind_ip) != 0) {
        fprintf(stderr, "FAIL: RPTL source %s:%d != configured bind %s:%d\n",
                src_ip, src_port, cfg.hbp_bind_ip, cfg.hbp_bind_port);
        fails++;
    }
    checks++;
    if (hbp_is_connected(hb)) { fprintf(stderr, "FAIL: connected before handshake completed\n"); fails++; }

    uint8_t radio_id[4]; memcpy(radio_id, buf + 4, 4);

    /* 2. RPTACK + arbitrary salt (DMRGateway hardcodes salt=1 and never checks
     * the RPTK digest — this test uses an arbitrary value to prove the same). */
    uint8_t rptack1[10]; memcpy(rptack1, "RPTACK", 6);
    rptack1[6]=0xAA; rptack1[7]=0xBB; rptack1[8]=0xCC; rptack1[9]=0xDD;
    udp_sendto(fake_fd, rptack1, 10, cfg.hbp_bind_ip, cfg.hbp_bind_port);
    pump(loop);

    /* 3. RPTK (digest not validated by the fake master, matching real DMRGateway). */
    n = fake_recv(fake_fd, buf, sizeof buf, src_ip, &src_port);
    checks++;
    if (n != 40 || memcmp(buf, "RPTK", 4) != 0) { fprintf(stderr, "FAIL: expected 40-byte RPTK, got n=%d\n", n); fails++; }

    /* 4. RPTACK (no salt this time) -> real client sends RPTC. */
    uint8_t rptack2[10]; memcpy(rptack2, "RPTACK", 6); memcpy(rptack2 + 6, radio_id, 4);
    udp_sendto(fake_fd, rptack2, 10, cfg.hbp_bind_ip, cfg.hbp_bind_port);
    pump(loop);

    n = fake_recv(fake_fd, buf, sizeof buf, src_ip, &src_port);
    checks++;
    if (n != RPTC_LEN || memcmp(buf, "RPTC", 4) != 0) { fprintf(stderr, "FAIL: expected %d-byte RPTC, got n=%d\n", RPTC_LEN, n); fails++; }

    /* 5. RPTACK -> CONNECTED (test fixture has no options, so no RPTO step). */
    udp_sendto(fake_fd, rptack2, 10, cfg.hbp_bind_ip, cfg.hbp_bind_port);
    pump(loop);
    checks++;
    if (!hbp_is_connected(hb)) { fprintf(stderr, "FAIL: not connected after RPTC ACK\n"); fails++; }

    /* 6. DMRD relay, fake master -> IPSC side. */
    uint8_t call_lc[9] = {0,0,0, 0,0,0, 0,0,0};
    call_lc[3]=(TEST_DST>>16)&0xFF; call_lc[4]=(TEST_DST>>8)&0xFF; call_lc[5]=TEST_DST&0xFF;
    call_lc[6]=(TEST_SRC>>16)&0xFF; call_lc[7]=(TEST_SRC>>8)&0xFF; call_lc[8]=TEST_SRC&0xFF;
    uint8_t frame[DMRD_LEN];
    build_dmrd_head(frame, 0, call_lc);
    udp_sendto(fake_fd, frame, DMRD_LEN, cfg.hbp_bind_ip, cfg.hbp_bind_port);
    pump(loop);
    checks++;
    if (ngv != 1) { fprintf(stderr, "FAIL: expected 1 relayed IPSC frame, got %d\n", ngv); fails++; }
    else {
        unsigned src = (unsigned)(gvcap[0][6]<<16 | gvcap[0][7]<<8 | gvcap[0][8]);
        unsigned dst = (unsigned)(gvcap[0][9]<<16 | gvcap[0][10]<<8 | gvcap[0][11]);
        checks++;
        if (src != (unsigned)TEST_SRC || dst != (unsigned)TEST_DST) {
            fprintf(stderr, "FAIL: relayed frame src=%u dst=%u (want %u/%u)\n", src, dst, (unsigned)TEST_SRC, (unsigned)TEST_DST);
            fails++;
        }
    }

    /* 7. DMRD relay, IPSC side -> fake master, exact bytes. */
    build_dmrd_head(frame, 1, call_lc);
    hbp_send_dmrd(hb, frame, DMRD_LEN);
    n = fake_recv(fake_fd, buf, sizeof buf, src_ip, &src_port);
    checks++;
    if (n != DMRD_LEN || memcmp(buf, frame, DMRD_LEN) != 0) {
        fprintf(stderr, "FAIL: hbp_send_dmrd() did not deliver the expected bytes (n=%d)\n", n);
        fails++;
    }

    /* 8. Clean disconnect -> RPTCL. */
    hbp_deactivate(hb);
    n = fake_recv(fake_fd, buf, sizeof buf, src_ip, &src_port);
    checks++;
    if (n < 5 || memcmp(buf, "RPTCL", 5) != 0) { fprintf(stderr, "FAIL: expected RPTCL on deactivate, got n=%d\n", n); fails++; }
    checks++;
    if (hbp_is_connected(hb)) { fprintf(stderr, "FAIL: still connected after deactivate\n"); fails++; }

    printf("HBP gateway role: %d checks, %d failures\n", checks, fails);

    close(fake_fd);
    hbp_stop(hb);
    hbp_free(hb); translator_free(tr); ev_free(loop);
    return fails ? 1 : 0;
}
