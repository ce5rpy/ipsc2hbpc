/* test_talker_alias.c — Talker Alias relay (IPSC -> HBP), all four embedded-LC
 * positions (B/C/D/E).
 *
 * Real captures (2026-07-23, non-audio protocol bytes only — no voice payload
 * used) confirmed IPSC only ever reveals the current superframe's full 9-byte
 * LC once, on the burst-E frame; B/C/D carry no LC information of their own
 * over IPSC, just AMBE — this is a fixed, deterministic pattern (52,57,57,57,
 * 66,57-byte frame cycle) across group/private calls alike, not a firmware
 * quirk. So B/C/D of a TA superframe are built and HELD (not sent) until E
 * arrives and resolves whether this superframe is TA; only then are all four
 * flushed with a self-consistent embedded LC — otherwise a real receiver
 * reconstructing bursts B/C/D/E together gets 3 fragments of the old call LC
 * plus 1 of the TA LC, fails the checksum, and silently drops it (audio is
 * unaffected since it travels on a separate channel — this is why group calls
 * "worked" while Talker Alias never displayed).
 *
 * Feeds a synthetic call (VOICE_HEAD + sync filler + B/C/D as plain audio,
 * matching real IPSC — no TA marker on those — + E carrying a fabricated TA
 * header block) through the C translator and checks:
 *
 *   1. B, C, D and E of the DMRD stream given the CALL's own identity match
 *      dmr_encode_emblc() of the 9-byte TA LC — B/C/D/E are all reflown from
 *      IPSC's single burst-E revelation, spliced the same way build_embed()
 *      does (DMR_EMB[idx] sync nibbles around the 32-bit fragment for that
 *      position).
 *   2. The call identity (src/dst in the DMRD header) is UNCHANGED — still
 *      the call's own src/dst, not the TA alias bytes. This is the property
 *      translate.c commit ae6a7b2 already protects; this test additionally
 *      proves the buffered TA relay code path does not regress it.
 *
 * All frame bytes are fabricated (arbitrary ids matching the style already
 * used by tests/parity_in.txt; no captured wire data / audio of any kind). */
#include <stdio.h>
#include <string.h>
#include "../src/translate.h"
#include "../src/config.h"
#include "../src/eventloop.h"
#include "../src/ipsc_const.h"
#include "../src/dmr/dmr.h"

#define MAXCAP 16
static uint8_t cap[MAXCAP][128];
static int     caplen[MAXCAP];
static int     ncap = 0;

int  ipsc_has_peers(struct ipsc *ip) { (void)ip; return 1; }
void ipsc_send_voice(struct ipsc *ip, const uint8_t *p, int n) { (void)ip; (void)p; (void)n; }
int  hbp_is_connected(struct hbp *hb) { (void)hb; return 1; }
void hbp_activate(struct hbp *hb) { (void)hb; }
void hbp_deactivate(struct hbp *hb) { (void)hb; }
void hbp_send_dmrd(struct hbp *hb, const uint8_t *d, int n) {
    (void)hb;
    if (ncap < MAXCAP) { memcpy(cap[ncap], d, (size_t)n); caplen[ncap] = n; ncap++; }
}

enum { TEST_SRC = 3120001, TEST_DST = 91, TEST_TS = 1 };

/* Build one fake IPSC GROUP_VOICE burst. seq is just a wire token (not a call
 * boundary, per ae6a7b2). ta_text, when non-NULL, must be exactly 6 chars —
 * writes a fabricated TA_HEADER LC (FLCO 0x04) into the burst-E position
 * (data[32]=GV_BE_FLAG, data[56..64]=LC). Returns the frame length. */
static int build_frame(uint8_t *out, int burst_type, int seq, const char *ta_text)
{
    memset(out, 0, 66);
    out[0] = GROUP_VOICE;
    out[1]=0x00; out[2]=0x04; out[3]=0xc3;      /* peer_id (arbitrary) */
    out[5] = (uint8_t)seq;
    out[6]=(TEST_SRC>>16)&0xFF; out[7]=(TEST_SRC>>8)&0xFF; out[8]=TEST_SRC&0xFF;
    out[9]=(TEST_DST>>16)&0xFF; out[10]=(TEST_DST>>8)&0xFF; out[11]=TEST_DST&0xFF;
    out[12] = 0x02;                              /* call_type: group */
    out[13]=0x00; out[14]=0x00; out[15]=0x43; out[16]=0xe2;  /* call_ctrl (arbitrary) */
    out[17] = (TEST_TS == 2) ? TS_CALL_MSK : 0x00;
    /* bytes 18-29: RTP header, left zero — not exercised by this test */
    out[30] = (uint8_t)burst_type;

    if (burst_type == VOICE_HEAD) return 54;   /* tail unused by translate.c for HEAD */

    /* SLOT1_VOICE: bytes 31-49 = AMBE (left zero, no audio content) */
    if (!ta_text) return 52;

    out[32] = GV_BE_FLAG;
    out[GV_BE_LC_FLCO_OFF]     = FLCO_TA_HEADER;
    out[GV_BE_LC_FLCO_OFF + 1] = 0x00;                                  /* FID */
    out[GV_BE_LC_FLCO_OFF + 2] = (uint8_t)((2 << 6) | (int)strlen(ta_text)); /* format=2, declen */
    memcpy(out + GV_BE_LC_FLCO_OFF + 3, ta_text, 6);
    return 66;
}

/* Bit-splice identical to translate.c's static build_embed() for a given
 * superframe position pos (1..4 = burst B..E, idx = pos-1): DMR_EMB[idx][0:8]
 * + 32-bit fragment for that position + DMR_EMB[idx][8:16]. */
static void expected_embed(const uint8_t ta_lc[9], int pos, dmr_bit out[48])
{
    uint8_t frag[4][4];
    dmr_encode_emblc(ta_lc, frag);
    int idx = pos - 1;
    memcpy(out, DMR_EMB[idx], 8);
    dmr_bytes_to_bits(frag[idx], 4, out + 8);
    memcpy(out + 40, DMR_EMB[idx] + 8, 8);
}

int main(void)
{
    Config cfg; char err[4096];
    if (config_load("tests/parity.toml", &cfg, err, sizeof err)) {
        fprintf(stderr, "config: %s\n", err); return 2;
    }
    ev_loop *loop = ev_new();
    translator *tr = translator_new(&cfg, loop);
    translator_set_protocols(tr, (struct ipsc *)1, (struct hbp *)1);

    uint8_t raw[66];
    int n, seq = 1;

    n = build_frame(raw, VOICE_HEAD, seq++, NULL);
    translator_ipsc_voice_received(tr, raw, n, TEST_TS, VOICE_HEAD);

    for (int i = 0; i < 4; i++) {
        n = build_frame(raw, SLOT1_VOICE, seq++, NULL);
        translator_ipsc_voice_received(tr, raw, n, TEST_TS, SLOT1_VOICE);
    }

    uint8_t ta_lc[9];
    n = build_frame(raw, SLOT1_VOICE, seq++, "TESTLB");
    memcpy(ta_lc, raw + GV_BE_LC_FLCO_OFF, 9);
    translator_ipsc_voice_received(tr, raw, n, TEST_TS, SLOT1_VOICE);

    int fails = 0;

    if (ncap != 6) {
        fprintf(stderr, "FAIL: expected 6 DMRD frames (HEAD + sync filler + B/C/D/E), got %d\n", ncap);
        fails++;
    } else {
        /* DMRD frame index 1 = position A (sync, no embedded-LC, no check);
         * 2..5 = B,C,D,E — cap[pos+1] for pos in 1..4. */
        for (int pos = 1; pos <= 4; pos++) {
            const uint8_t *dmrd = cap[pos + 1];
            int dlen = caplen[pos + 1];

            /* 1. Call identity unchanged: DMRD src (bytes 5:8) / dst (bytes
             * 8:11) must still be the call's own TEST_SRC/TEST_DST — NOT the
             * TA text bytes (which would decode to something else entirely). */
            unsigned dmrd_src = (unsigned)(dmrd[5]<<16 | dmrd[6]<<8 | dmrd[7]);
            unsigned dmrd_dst = (unsigned)(dmrd[8]<<16 | dmrd[9]<<8 | dmrd[10]);
            if (dmrd_src != (unsigned)TEST_SRC || dmrd_dst != (unsigned)TEST_DST) {
                fprintf(stderr, "FAIL: pos=%d corrupted call identity: src=%u dst=%u (want %u/%u)\n",
                        pos, dmrd_src, dmrd_dst, (unsigned)TEST_SRC, (unsigned)TEST_DST);
                fails++;
                continue;
            }

            /* 2. This position's EMB carries ITS OWN fragment of the TA LC —
             * proving B/C/D were reflown once E revealed the TA LC, not left
             * on the call's own GVCU LC fragment. */
            if (dlen < 20 + 33) {
                fprintf(stderr, "FAIL: pos=%d DMRD frame too short (%d bytes)\n", pos, dlen);
                fails++;
                continue;
            }
            const uint8_t *payload_33 = dmrd + 20;
            dmr_bit full_bits[264];
            dmr_bytes_to_bits(payload_33, 33, full_bits);
            dmr_bit got_embed[48];
            memcpy(got_embed, full_bits + 108, 48);   /* 72 + 36 = 108 */

            dmr_bit want_embed[48];
            expected_embed(ta_lc, pos, want_embed);

            if (memcmp(got_embed, want_embed, sizeof want_embed) != 0) {
                fprintf(stderr, "FAIL: pos=%d EMB does not match reinjected Talker Alias LC fragment\n", pos);
                fprintf(stderr, "  got:  "); for (int i=0;i<48;i++) fprintf(stderr,"%d",got_embed[i]); fprintf(stderr,"\n");
                fprintf(stderr, "  want: "); for (int i=0;i<48;i++) fprintf(stderr,"%d",want_embed[i]); fprintf(stderr,"\n");
                fails++;
            }
        }
    }

    printf("Talker Alias relay (IPSC->HBP): %d failures\n", fails);

    translator_free(tr); ev_free(loop);
    return fails ? 1 : 0;
}
