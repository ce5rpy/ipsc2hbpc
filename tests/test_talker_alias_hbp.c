/* test_talker_alias_hbp.c — Talker Alias burst-E relay (HBP -> IPSC).
 *
 * There is no captured live-network HBP master traffic carrying Talker Alias
 * (no C7 capture; DMRGateway/real-master reference is still pending — see
 * FEATURE_ROADMAP.md). So, per the same round-trip-against-our-own-encoder
 * approach used for tests/test_emblc_roundtrip.c: this test builds a
 * synthetic DMR superframe using dmr_encode_emblc() — the SAME primitive a
 * real DMR transmitter uses, already validated against the Python reference
 * — to produce the 4 burst B/C/D/E embedded-LC fragments a real network
 * would send for a Talker Alias header block, feeds them through
 * translator_hbp_voice_received() exactly like a live master would, and
 * checks that the resulting IPSC-side burst-E frame carries the reassembled
 * TA LC while the call's own identity (src/dst) is unchanged.
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

#define MAXCAP 16
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

enum { TEST_SRC = 3120001, TEST_DST = 91, TEST_TS = 1 };
static const uint8_t STREAM_ID[4] = { 0x42, 0x7d, 0x91, 0x7f };

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
    if (TEST_TS == 2) out[DMRD_FLAGS_OFF] |= HBPF_TGID_TS2;

    dmr_bits_to_bytes(fb, 264, out + DMRD_PAYLOAD_OFF);
    return DMRD_LEN;
}

static void stop_cb(ev_loop *loop, void *ud){ (void)ud; ev_stop(loop); }

int main(void)
{
    Config cfg; char err[4096];
    if (config_load("tests/parity.toml", &cfg, err, sizeof err)) {
        fprintf(stderr, "config: %s\n", err); return 2;
    }
    ev_loop *loop = ev_new();
    translator *tr = translator_new(&cfg, loop);
    translator_set_protocols(tr, (struct ipsc *)1, (struct hbp *)1);

    uint8_t call_lc[9] = {0x00,0x00,0x00, 0,0,0, 0,0,0};   /* filled below */
    call_lc[3]=(TEST_DST>>16)&0xFF; call_lc[4]=(TEST_DST>>8)&0xFF; call_lc[5]=TEST_DST&0xFF;
    call_lc[6]=(TEST_SRC>>16)&0xFF; call_lc[7]=(TEST_SRC>>8)&0xFF; call_lc[8]=TEST_SRC&0xFF;

    uint8_t ta_lc[9] = {0x04, 0x00, (2<<6)|6, 'T','E','S', 'T','L','B'};
    uint8_t ta_frag[4][4];
    dmr_encode_emblc(ta_lc, ta_frag);

    uint8_t frame[DMRD_LEN];
    int seq = 0;

    build_dmrd(frame, seq++, call_lc, 1, 0, 0, NULL);
    translator_hbp_voice_received(tr, frame, DMRD_LEN);

    build_dmrd(frame, seq++, call_lc, 0, 0, 0, NULL);            /* burst A */
    translator_hbp_voice_received(tr, frame, DMRD_LEN);
    for (int pos = 1; pos <= 4; pos++) {                          /* bursts B,C,D,E: TA fragments */
        build_dmrd(frame, seq++, call_lc, 0, 0, pos, ta_frag[pos - 1]);
        translator_hbp_voice_received(tr, frame, DMRD_LEN);
    }

    /* Let the jitter-buffer delivery clock play out bursts A-E. */
    ev_timer_after(loop, 1.0, stop_cb, loop);
    ev_run(loop);

    int fails = 0, checks = 0;

    /* Find the delivered SLOT1_VOICE burst-E frame (data[32]==GV_BE_FLAG). */
    const uint8_t *be = NULL;
    for (int i = 0; i < ngv; i++) {
        if (gvlen[i] > GV_BE_LC_FLCO_OFF + 9 && gvcap[i][32] == GV_BE_FLAG) { be = gvcap[i]; break; }
    }
    checks++;
    if (!be) {
        fprintf(stderr, "FAIL: no burst-E GROUP_VOICE frame delivered (got %d GV frames)\n", ngv);
        fails++;
    } else {
        /* 1. Call identity unchanged: header src/dst (bytes 6:9 / 9:12) still
         * the call's own TEST_SRC/TEST_DST, not the TA text bytes. */
        checks++;
        unsigned src = (unsigned)(be[6]<<16 | be[7]<<8 | be[8]);
        unsigned dst = (unsigned)(be[9]<<16 | be[10]<<8 | be[11]);
        if (src != (unsigned)TEST_SRC || dst != (unsigned)TEST_DST) {
            fprintf(stderr, "FAIL: burst-E frame corrupted call identity: src=%u dst=%u (want %u/%u)\n",
                    src, dst, (unsigned)TEST_SRC, (unsigned)TEST_DST);
            fails++;
        }

        /* 2. Burst-E reassembled-LC repeat (data[56..64]) equals the TA LC. */
        checks++;
        if (memcmp(be + GV_BE_LC_FLCO_OFF, ta_lc, 9) != 0) {
            fprintf(stderr, "FAIL: burst-E LC repeat does not match reinjected Talker Alias LC\n  got:  ");
            for (int i=0;i<9;i++) fprintf(stderr,"%02x",be[GV_BE_LC_FLCO_OFF+i]);
            fprintf(stderr, "\n  want: ");
            for (int i=0;i<9;i++) fprintf(stderr,"%02x",ta_lc[i]);
            fprintf(stderr, "\n");
            fails++;
        }
    }

    printf("Talker Alias relay (HBP->IPSC): %d checks, %d failures\n", checks, fails);

    translator_free(tr); ev_free(loop);
    return fails ? 1 : 0;
}
