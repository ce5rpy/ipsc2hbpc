/* test_talker_alias_hbp.c — Talker Alias relay (HBP -> IPSC).
 *
 * There is no captured live-network HBP master traffic carrying Talker Alias
 * (no C7 capture; DMRGateway/real-master reference is still pending — see
 * FEATURE_ROADMAP.md). So, per the same round-trip-against-our-own-encoder
 * approach used for tests/test_emblc_roundtrip.c: this test builds a
 * synthetic DMR superframe using dmr_encode_emblc() — the SAME primitive a
 * real DMR transmitter uses, already validated against the Python reference
 * — to produce the 4 burst B/C/D/E embedded-LC fragments a real network
 * would send for Talker Alias blocks, feeds them through
 * translator_hbp_voice_received() at a master's pace, and checks that every
 * superframe the repeater gets carries one whole embedded LC (B-E fragments
 * and the burst-E LC repeat agree), that each alias block arrives whole and in
 * order, that the call's identity (src/dst) is unchanged, and that an alias
 * still queued when a call ends does not leak into the next one.
 *
 * All frame bytes are fabricated (arbitrary ids, silence AMBE) — no captured
 * wire data / audio of any kind. */
#include <stdio.h>
#include <string.h>
#include "../src/translate.h"
#include "../src/config.h"
#include "../src/eventloop.h"
#include "../src/ipsc_const.h"
#include "../src/hbp_const.h"
#include "../src/dmr/dmr.h"

#define MAXCAP 64
static uint8_t gvcap[MAXCAP][128];
static int     gvlen[MAXCAP];
static int     ngv = 0;

int  ipsc_has_peers(struct ipsc *ip) { (void)ip; return 1; }
uint32_t ipsc_sole_peer_id(struct ipsc *ip) { (void)ip; return 0; }
void ipsc_send_voice(struct ipsc *ip, const uint8_t *p, int n) {
    (void)ip;
    if (ngv < MAXCAP) { memcpy(gvcap[ngv], p, (size_t)n); gvlen[ngv] = n; ngv++; }
}
int  hbp_is_connected(struct hbp *hb) { (void)hb; return 1; }
void hbp_activate(struct hbp *hb) { (void)hb; }
void hbp_deactivate(struct hbp *hb) { (void)hb; }
void hbp_send_dmrd(struct hbp *hb, const uint8_t *d, int n) { (void)hb; (void)d; (void)n; }

enum { TEST_SRC = 3120001, TEST_DST = 91 };
static int test_ts = 1;
static uint8_t STREAM_ID[4] = { 0x42, 0x7d, 0x91, 0x7f };

static void put_bits_at(dmr_bit *fb, int off, const dmr_bit *src, int n)
{
    memcpy(fb + off, src, (size_t)n);
}

/* Build one synthetic HBP DMRD frame. burst_type: HBPF_SLT_VHEAD/VTERM for
 * the header/terminator, or -1/pos(0..5) for VOICESYNC/VOICE bursts. When
 * frag is non-NULL (only meaningful for pos 1-4 / burst B-D-E), its 4 bytes
 * are spliced into the burst's embedded-LC position exactly like a real
 * transmitter's build_embed() would (DMR_EMB sync nibbles around the data). */
static int build_dmrd(uint8_t *out, int seq, const uint8_t lc[9], int is_head, int is_term,
                      int pos, const uint8_t *frag)
{
    memset(out, 0, DMRD_LEN);
    memcpy(out, "DMRD", 4);
    out[DMRD_SEQ_OFF] = (uint8_t)seq;
    memcpy(out + DMRD_SRC_OFF, lc + 6, 3);
    memcpy(out + DMRD_DST_OFF, lc + 3, 3);
    out[DMRD_RPTR_OFF]=0x00; out[DMRD_RPTR_OFF+1]=0x00; out[DMRD_RPTR_OFF+2]=0x00; out[DMRD_RPTR_OFF+3]=0x01;
    memcpy(out + DMRD_STREAM_OFF, STREAM_ID, 4);

    dmr_bit fb[264]; memset(fb, 0, sizeof fb);

    if (is_head || is_term) {
        out[DMRD_FLAGS_OFF] = HBPF_FRAMETYPE_DATASYNC | (uint8_t)(is_term ? HBPF_SLT_VTERM : HBPF_SLT_VHEAD);
        dmr_bit full_lc[196];
        dmr_bptc_encode_lc(lc, is_term, full_lc);
        put_bits_at(fb, 0, full_lc, 98);
        put_bits_at(fb, 98, is_term ? DMR_SLOT_TYPE_VTERM : DMR_SLOT_TYPE_VHEAD, 10);
        put_bits_at(fb, 108, DMR_BS_DATA_SYNC, 48);
        put_bits_at(fb, 156, (is_term ? DMR_SLOT_TYPE_VTERM : DMR_SLOT_TYPE_VHEAD) + 10, 10);
        put_bits_at(fb, 166, full_lc + 98, 98);
    } else {
        int frame_type = (pos == 0) ? HBPF_FRAMETYPE_VOICESYNC : HBPF_FRAMETYPE_VOICE;
        int dtype = (pos == 0) ? 0 : pos;
        out[DMRD_FLAGS_OFF] = (uint8_t)(frame_type | dtype);
        /* a1/a2/a3 AMBE codewords: all-zero (silence-shaped, no real audio). */
        /* embed region [108:156]: sync nibbles + 32-bit fragment (or silence sync for pos 0). */
        if (pos == 0) {
            put_bits_at(fb, 108, DMR_BS_VOICE_SYNC, 48);
        } else {
            int idx = pos - 1;
            put_bits_at(fb, 108, DMR_EMB[idx], 8);
            if (frag) {
                dmr_bit fragbits[32]; dmr_bytes_to_bits(frag, 4, fragbits);
                put_bits_at(fb, 116, fragbits, 32);
            }
            put_bits_at(fb, 148, DMR_EMB[idx] + 8, 8);
        }
    }
    if (test_ts == 2) out[DMRD_FLAGS_OFF] |= HBPF_TGID_TS2;

    dmr_bits_to_bytes(fb, 264, out + DMRD_PAYLOAD_OFF);
    return DMRD_LEN;
}

static void stop_cb(ev_loop *loop, void *ud){ (void)ud; ev_stop(loop); }

/* The frames a master would send, fed at its pace (one burst every 60 ms). */
#define MAXFEED 64
static uint8_t feed[MAXFEED][DMRD_LEN];
static double feed_gap[MAXFEED];   /* extra silence on the wire before frame i */
static int nfeed, fed, feed_len = DMRD_LEN;
static translator *TR;

static void feed_cb(ev_loop *loop, void *ud)
{
    (void)ud;
    translator_hbp_voice_received(TR, feed[fed++], feed_len);
    if (fed < nfeed) ev_timer_after(loop, 0.060 + feed_gap[fed], feed_cb, NULL);
}

/* One call on TS1: `heads` headers, nsf superframes (the first `skip` bursts not sent, as
 * when a call is joined midway) with `gap` seconds of nothing on the wire before superframe
 * `gap_at`, a terminator; played out, then the GV frames sent. */
static void play_call(ev_loop *loop, const uint8_t lc[9], int heads, int skip, int nsf, int gap_at, double gap)
{
    uint8_t frag[4][4];
    int seq = 0;
    dmr_encode_emblc(lc, frag);
    ngv = 0; nfeed = fed = 0; feed_len = DMRD_LEN; test_ts = 1; STREAM_ID[1]++;
    memset(feed_gap, 0, sizeof feed_gap);
    for (int h = 0; h < heads; h++) build_dmrd(feed[nfeed++], seq++, lc, 1, 0, 0, NULL);
    for (int k = 0; k < nsf; k++) {
        if (k == gap_at) feed_gap[nfeed] = gap;
        for (int pos = k ? 0 : skip; pos < 6; pos++)
            build_dmrd(feed[nfeed++], seq++, lc, 0, 0, pos, (pos >= 1 && pos <= 4) ? frag[pos - 1] : NULL);
    }
    build_dmrd(feed[nfeed++], seq++, lc, 0, 1, 0, NULL);
    ev_timer_after(loop, 0.0, feed_cb, NULL);
    ev_timer_after(loop, 0.5 + nsf * 0.36 + gap + 2.0, stop_cb, loop);
    ev_run(loop);
}

/* One transmission cut into two streams with no terminator between them, as a bridge sends it
 * and a master forwards it: the first ends on burst C of the third superframe, the master holds
 * the second back for its stream timeout and lets it through from burst E. */
static void play_split(ev_loop *loop, const uint8_t lc[9], const uint8_t lc2[9])
{
    uint8_t frag[4][4], frag2[4][4];
    int seq = 0;
    dmr_encode_emblc(lc, frag); dmr_encode_emblc(lc2, frag2);
    ngv = 0; nfeed = fed = 0; feed_len = DMRD_LEN; test_ts = 1; STREAM_ID[1]++;
    memset(feed_gap, 0, sizeof feed_gap);
    build_dmrd(feed[nfeed++], seq++, lc, 1, 0, 0, NULL);
    for (int i = 0; i < 15; i++) {
        int pos = i % 6;
        build_dmrd(feed[nfeed++], seq++, lc, 0, 0, pos, (pos >= 1 && pos <= 4) ? frag[pos - 1] : NULL);
    }
    STREAM_ID[2]++;
    feed_gap[nfeed] = 0.30;
    for (int i = 16; i < 32; i++) {
        int pos = i % 6;
        build_dmrd(feed[nfeed++], seq++, lc2, 0, 0, pos, (pos >= 1 && pos <= 4) ? frag2[pos - 1] : NULL);
    }
    build_dmrd(feed[nfeed++], seq++, lc2, 0, 1, 0, NULL);
    ev_timer_after(loop, 0.0, feed_cb, NULL);
    ev_timer_after(loop, 4.5, stop_cb, loop);
    ev_run(loop);
}

/* VOICE_HEADs and VOICE_TERMs sent, how many headers came right before a voice burst A (a
 * call's voice starts on burst A after its headers), and whether every frame kept the first
 * one's IPSC stream id. */
static void count_calls(int *heads, int *terms, int *heads_before_voice, int *one_stream)
{
    *heads = *terms = *heads_before_voice = 0;
    *one_stream = 1;
    for (int i = 0; i < ngv; i++) {
        if (gvcap[i][30] == VOICE_HEAD) {
            (*heads)++;
            if (i + 1 < ngv && gvcap[i + 1][30] == SLOT1_VOICE && gvcap[i + 1][31] == 0x14 && gvcap[i + 1][32] == 0x40)
                (*heads_before_voice)++;
        }
        if (gvcap[i][30] == VOICE_TERM) (*terms)++;
        if (gvcap[i][5] != gvcap[0][5]) *one_stream = 0;
    }
}

/* RTP as a Motorola repeater sends a call (captures/20260720-talkeralias-group-ts1.pcap, DMRlink's
 * Retired/template.py): the marker on its first packet only, and each packet but the terminator one
 * slot (480) after the one before, header repeats included. */
static uint32_t rtp_ts(const uint8_t *p)
{
    return (uint32_t)p[GV_RTP_TS_OFF] << 24 | (uint32_t)p[GV_RTP_TS_OFF + 1] << 16 |
           (uint32_t)p[GV_RTP_TS_OFF + 2] << 8 | p[GV_RTP_TS_OFF + 3];
}

static int rtp_like_motorola(const char *what)
{
    enum { RTP_MARKER_OFF = GV_RTP_TS_OFF - 3 };
    int fails = 0;
    for (int i = 0; i < ngv; i++) {
        const uint8_t *p = gvcap[i];
        int type = p[GV_BURST_TYPE_OFF];
        int first = i == 0 || p[GV_CALL_SEQ_OFF] != gvcap[i - 1][GV_CALL_SEQ_OFF];
        int marker = (p[RTP_MARKER_OFF] & 0x80) != 0;
        if (marker != first) {
            fprintf(stderr, "FAIL: %s: packet %d (type 0x%02x) %s the call-start marker\n",
                    what, i, type, marker ? "carries" : "lacks");
            fails++;
        }
        if (first || type == VOICE_TERM) continue;
        uint32_t step = rtp_ts(p) - rtp_ts(gvcap[i - 1]);
        if (step != 480) {
            fprintf(stderr, "FAIL: %s: packet %d (type 0x%02x) is %u RTP ticks after the one before, want 480\n",
                    what, i, type, step);
            fails++;
        }
    }
    return fails;
}

static int check_lc(const char *what, const uint8_t got[9], const uint8_t want[9])
{
    if (memcmp(got, want, 9) == 0) return 0;
    fprintf(stderr, "FAIL: %s\n  got:  ", what);
    for (int i = 0; i < 9; i++) fprintf(stderr, "%02x", got[i]);
    fprintf(stderr, "\n  want: ");
    for (int i = 0; i < 9; i++) fprintf(stderr, "%02x", want[i]);
    fprintf(stderr, "\n");
    return 1;
}

int main(void)
{
    Config cfg; char err[4096];
    if (config_load("tests/parity.toml", &cfg, err, sizeof err)) {
        fprintf(stderr, "config: %s\n", err); return 2;
    }
    ev_loop *loop = ev_new();
    TR = translator_new(&cfg, loop);
    translator_set_protocols(TR, (struct ipsc *)1, (struct hbp *)1);

    uint8_t call_lc[9] = {0x00,0x00,0x00, 0,0,0, 0,0,0};
    call_lc[3]=(TEST_DST>>16)&0xFF; call_lc[4]=(TEST_DST>>8)&0xFF; call_lc[5]=TEST_DST&0xFF;
    call_lc[6]=(TEST_SRC>>16)&0xFF; call_lc[7]=(TEST_SRC>>8)&0xFF; call_lc[8]=TEST_SRC&0xFF;
    uint8_t ta_head[9] = {0x04, 0x00, (2<<6)|13, 'T','E','S', 'T','L','B'};
    uint8_t ta_blk1[9] = {0x05, 0x00, 'L','A','B',' ','V','H','F'};
    uint8_t ta_blk2[9] = {0x06, 0x00, ' ','2','m',0,0,0,0};
    uint8_t call_frag[4][4], head_frag[4][4], blk1_frag[4][4], blk2_frag[4][4];
    dmr_encode_emblc(call_lc, call_frag);
    dmr_encode_emblc(ta_head, head_frag);
    dmr_encode_emblc(ta_blk1, blk1_frag);
    dmr_encode_emblc(ta_blk2, blk2_frag);

    /* A master's superframes: the radio's alias rides in some of them, between call-LC ones.
     * The first call ends right after an alias block, still queued when its terminator comes;
     * the next call (another stream) must not start with it. */
    uint8_t (*sf[])[4] = { head_frag, call_frag, blk1_frag, call_frag, blk2_frag, call_frag };
    int ncall1 = 5, nsf = 6, seq = 0;
    build_dmrd(feed[nfeed++], seq++, call_lc, 1, 0, 0, NULL);
    for (int k = 0; k < nsf; k++) {
        if (k == ncall1) {
            build_dmrd(feed[nfeed++], seq++, call_lc, 0, 1, 0, NULL);
            STREAM_ID[3] ^= 0xFF;
            build_dmrd(feed[nfeed++], seq++, call_lc, 1, 0, 0, NULL);
        }
        for (int pos = 0; pos < 6; pos++)
            build_dmrd(feed[nfeed++], seq++, call_lc, 0, 0, pos, (pos >= 1 && pos <= 4) ? sf[k][pos - 1] : NULL);
    }
    build_dmrd(feed[nfeed++], seq++, call_lc, 0, 1, 0, NULL);

    ev_timer_after(loop, 0.0, feed_cb, NULL);
    ev_timer_after(loop, 3.0, stop_cb, loop);
    ev_run(loop);

    int fails = 0, checks = 0, nout = 0, ta_seen = 0;
    const uint8_t *ta_want[] = { ta_head, ta_blk1 };
    /* Every superframe the repeater gets must carry one whole embedded LC: B,C,D,E fragments
     * that reassemble, matching the LC repeat on burst E. A mixed one (the call's B-D with an
     * alias's E) is what left late-entry radios mute. */
    for (int i = 0; i < ngv; i++) {
        const uint8_t *a = gvcap[i];
        if (a[30] != SLOT1_VOICE || a[31] != 0x14 || a[32] != 0x40 || i + 4 >= ngv) continue;
        const uint8_t *b = gvcap[i + 1], *c = gvcap[i + 2], *d = gvcap[i + 3], *e = gvcap[i + 4];
        if (e[32] != GV_BE_FLAG) continue;
        uint8_t frag[4][4], lc[9];
        memcpy(frag[0], b + 52, 4); memcpy(frag[1], c + 52, 4);
        memcpy(frag[2], d + 52, 4); memcpy(frag[3], e + 52, 4);
        nout++;
        checks++;
        if (!dmr_decode_emblc(frag, lc)) {
            fprintf(stderr, "FAIL: outgoing superframe %d: bursts B-E do not reassemble into one LC\n", nout);
            fails++;
            continue;
        }
        checks++;
        fails += check_lc("burst-E LC repeat differs from the superframe's embedded LC", e + GV_BE_LC_FLCO_OFF, lc);
        checks++;
        unsigned src = (unsigned)(e[6]<<16 | e[7]<<8 | e[8]), dst = (unsigned)(e[9]<<16 | e[10]<<8 | e[11]);
        if (src != (unsigned)TEST_SRC || dst != (unsigned)TEST_DST) {
            fprintf(stderr, "FAIL: superframe %d changed the call identity: src=%u dst=%u\n", nout, src, dst);
            fails++;
        }
        if (lc[0] >= 0x04 && lc[0] <= 0x07) {
            checks++;
            if (ta_seen < 2) fails += check_lc("alias relayed out of order or altered", lc, ta_want[ta_seen]);
            else { fprintf(stderr, "FAIL: more alias superframes than were sent\n"); fails++; }
            ta_seen++;
        } else {
            checks++;
            fails += check_lc("a call-LC superframe lost the call's identity", lc, call_lc);
        }
    }
    checks++;
    if (nout < nsf) { fprintf(stderr, "FAIL: %d superframes delivered, want at least %d\n", nout, nsf); fails++; }
    checks++;
    if (ta_seen != 2) { fprintf(stderr, "FAIL: %d alias superframes relayed, want 2\n", ta_seen); fails++; }

    printf("Talker Alias relay (HBP->IPSC): %d checks, %d failures\n", checks, fails);

    /* The calls above were on TS1: their VOICE_HEAD / VOICE_TERM say so (byte 31 bit 7). */
    int call_fails = 0, call_checks = 0, heads = 0, terms = 0;
    for (int i = 0; i < ngv; i++)
        if (gvcap[i][30] == VOICE_HEAD || gvcap[i][30] == VOICE_TERM) {
            heads += gvcap[i][30] == VOICE_HEAD;
            terms += gvcap[i][30] == VOICE_TERM;
            call_checks++;
            if (gvcap[i][31] & GV_PDU_SLOT_MSK) {
                fprintf(stderr, "FAIL: a TS1 %s is marked TS2\n", gvcap[i][30] == VOICE_HEAD ? "VOICE_HEAD" : "VOICE_TERM");
                call_fails++;
            }
        }
    call_checks++;
    if (!heads || !terms) { fprintf(stderr, "FAIL: %d VOICE_HEAD and %d VOICE_TERM delivered\n", heads, terms); call_fails++; }

    /* A TS2 call from a master sending 53-byte DMRD (no BER/RSSI, as HBlink3 does): it is
     * delivered whole, and its VOICE_HEAD / VOICE_TERM are marked TS2. */
    ngv = 0; nfeed = fed = 0; feed_len = DMRD_CORE_LEN; test_ts = 2; STREAM_ID[2] ^= 0xFF;
    build_dmrd(feed[nfeed++], seq++, call_lc, 1, 0, 0, NULL);
    for (int pos = 0; pos < 6; pos++)
        build_dmrd(feed[nfeed++], seq++, call_lc, 0, 0, pos, (pos >= 1 && pos <= 4) ? call_frag[pos - 1] : NULL);
    build_dmrd(feed[nfeed++], seq++, call_lc, 0, 1, 0, NULL);
    ev_timer_after(loop, 0.0, feed_cb, NULL);
    ev_timer_after(loop, 1.5, stop_cb, loop);
    ev_run(loop);
    int voice = 0, ts2_heads = 0;
    for (int i = 0; i < ngv; i++) {
        if (gvcap[i][30] == SLOT2_VOICE) voice++;
        if (gvcap[i][30] == VOICE_HEAD || gvcap[i][30] == VOICE_TERM) {
            ts2_heads++;
            call_checks++;
            if (!(gvcap[i][31] & GV_PDU_SLOT_MSK)) { fprintf(stderr, "FAIL: a TS2 VOICE_HEAD/TERM is marked TS1\n"); call_fails++; }
        }
    }
    call_checks++;
    if (voice != 6 || ts2_heads != 2) {
        fprintf(stderr, "FAIL: 53-byte DMRD call: %d voice bursts and %d head/term delivered, want 6 and 2\n", voice, ts2_heads);
        call_fails++;
    }
    printf("HBP->IPSC call frames: %d checks, %d failures\n", call_checks, call_fails);

    /* Network gaps, as the master's path drops voice now and then. A repeater mutes a call it
     * is told ended and then gets voice without a VOICE_HEAD, so, as MMDVMHost does: a gap
     * under 1.5 s is silence within the call; a longer one ends it, and voice after it, like a
     * call joined without its header, waits for burst A and is preceded by 3 VOICE_HEADs. */
    int gap_fails = 0, gap_checks = 0, hv, one;
    play_call(loop, call_lc, 1, 0, 4, 2, 1.0);
    count_calls(&heads, &terms, &hv, &one);
    gap_checks++;
    if (heads != 1 || terms != 1 || !one) {
        fprintf(stderr, "FAIL: a 1 s gap cut the call: %d VOICE_HEAD, %d VOICE_TERM, %s stream id\n",
                heads, terms, one ? "one" : "more than one");
        gap_fails++;
    }
    play_call(loop, call_lc, 0, 2, 2, -1, 0.0);   /* joined on burst C */
    count_calls(&heads, &terms, &hv, &one);
    gap_checks++;
    if (heads != 3 || hv != 1 || terms != 1) {
        fprintf(stderr, "FAIL: a call joined without its header: %d VOICE_HEAD (%d before voice), %d VOICE_TERM\n",
                heads, hv, terms);
        gap_fails++;
    }
    for (int i = 0; i < ngv; i++)
        if (gvcap[i][30] == VOICE_HEAD) {
            gap_checks++;
            unsigned src = (unsigned)(gvcap[i][6]<<16 | gvcap[i][7]<<8 | gvcap[i][8]);
            unsigned dst = (unsigned)(gvcap[i][9]<<16 | gvcap[i][10]<<8 | gvcap[i][11]);
            if (src != (unsigned)TEST_SRC || dst != (unsigned)TEST_DST) {
                fprintf(stderr, "FAIL: late-entry VOICE_HEAD names src=%u dst=%u\n", src, dst);
                gap_fails++;
            }
        }
    play_call(loop, call_lc, 1, 0, 4, 2, 2.0);
    count_calls(&heads, &terms, &hv, &one);
    gap_checks++;
    if (heads != 4 || hv != 2 || terms != 2) {
        fprintf(stderr, "FAIL: a 2 s gap: %d VOICE_HEAD (%d before voice), %d VOICE_TERM, want 4, 2, 2\n",
                heads, hv, terms);
        gap_fails++;
    }
    /* A new stream id with the same caller and target is the same call; another caller is not. */
    play_split(loop, call_lc, call_lc);
    count_calls(&heads, &terms, &hv, &one);
    gap_checks++;
    if (heads != 1 || terms != 1 || !one) {
        fprintf(stderr, "FAIL: a call split into two streams: %d VOICE_HEAD, %d VOICE_TERM, %s IPSC call\n",
                heads, terms, one ? "one" : "more than one");
        gap_fails++;
    }
    uint8_t other_lc[9]; memcpy(other_lc, call_lc, 9); other_lc[8]++;
    play_split(loop, call_lc, other_lc);
    count_calls(&heads, &terms, &hv, &one);
    gap_checks++;
    if (heads != 4 || one) {
        fprintf(stderr, "FAIL: another caller on a new stream: %d VOICE_HEAD, %s IPSC call, want 4 and two\n",
                heads, one ? "one" : "more than one");
        gap_fails++;
    }
    printf("HBP->IPSC network gaps: %d checks, %d failures\n", gap_checks, gap_fails);

    /* Header repeats: a call bridged from another repeater reaches the master with two headers,
     * and a call joined midway gets three. */
    int rtp_fails = 0, rtp_checks = 0;
    play_call(loop, call_lc, 2, 0, 3, -1, 0.0);
    count_calls(&heads, &terms, &hv, &one);
    rtp_checks++;
    if (heads != 2 || terms != 1) {
        fprintf(stderr, "FAIL: a call with two headers: %d VOICE_HEAD, %d VOICE_TERM, want 2 and 1\n", heads, terms);
        rtp_fails++;
    }
    rtp_checks++;
    rtp_fails += rtp_like_motorola("a call with two headers") != 0;
    play_call(loop, call_lc, 0, 2, 2, -1, 0.0);
    rtp_checks++;
    rtp_fails += rtp_like_motorola("a call joined midway") != 0;
    play_call(loop, call_lc, 1, 0, 2, -1, 0.0);
    rtp_checks++;
    rtp_fails += rtp_like_motorola("a call with one header") != 0;
    printf("HBP->IPSC header RTP: %d checks, %d failures\n", rtp_checks, rtp_fails);

    translator_free(TR); ev_free(loop);
    return fails || call_fails || gap_fails || rtp_fails ? 1 : 0;
}
