/* test_talker_alias.c — Talker Alias relay (IPSC -> HBP).
 *
 * IPSC reveals a superframe's LC only on burst E (bytes 56-64), after B-D are gone.  An
 * alias LC seen there must reach HBP whole in a later superframe: bursts B-E carrying
 * the four fragments of that one LC, which is how a receiver reassembles it.  The
 * superframe that revealed it keeps the call's LC, and the DMRD header keeps the call's
 * identity throughout.  All frame bytes are fabricated. */
#include <stdio.h>
#include <string.h>
#include "../src/translate.h"
#include "../src/config.h"
#include "../src/eventloop.h"
#include "../src/ipsc_const.h"
#include "../src/dmr/dmr.h"

#define MAXCAP 128
static uint8_t cap[MAXCAP][64];
static int     ncap = 0;

int  ipsc_has_peers(struct ipsc *ip) { (void)ip; return 1; }
uint32_t ipsc_sole_peer_id(struct ipsc *ip) { (void)ip; return 0; }
void ipsc_send_voice(struct ipsc *ip, const uint8_t *p, int n) { (void)ip; (void)p; (void)n; }
int  hbp_is_connected(struct hbp *hb) { (void)hb; return 1; }
void hbp_activate(struct hbp *hb) { (void)hb; }
void hbp_deactivate(struct hbp *hb) { (void)hb; }
void hbp_send_dmrd(struct hbp *hb, const uint8_t *d, int n) {
    (void)hb;
    if (ncap < MAXCAP && n <= 64) memcpy(cap[ncap++], d, (size_t)n);
}

enum { TEST_SRC = 3120001, TEST_DST = 91, TEST_TS = 1 };

static const uint8_t CALL_LC[9]  = { 0x00, 0x00, 0x00 };   /* burst E of a call superframe */
static const uint8_t TA_HEAD[9]  = { 0x04, 0x00, 0x94, 'T', 'E', 'S', 'T', 'L', 'B' };
static const uint8_t TA_BLOCK[9] = { 0x05, 0x00, ' ', 'V', 'H', 'F', 0, 0, 0 };

static translator *tr;
static int seq;
static int checks, fails;

static void check(int ok, const char *what)
{
    checks++;
    if (!ok) { fails++; fprintf(stderr, "FAIL: %s\n", what); }
}

/* One IPSC GROUP_VOICE frame shaped like a repeater's: burst A 52 bytes with the sync
 * flag, B-D and F 57 bytes, E 66 bytes carrying its superframe's LC. */
static void send(int burst_type, char pos, const uint8_t *lc, uint8_t e_ctrl)
{
    uint8_t out[66];
    memset(out, 0, sizeof out);
    out[0] = GROUP_VOICE;
    out[1] = 0x00; out[2] = 0x04; out[3] = 0xc3;
    out[5] = (uint8_t)seq++;
    out[6] = (TEST_SRC >> 16) & 0xFF; out[7] = (TEST_SRC >> 8) & 0xFF; out[8] = TEST_SRC & 0xFF;
    out[9] = (TEST_DST >> 16) & 0xFF; out[10] = (TEST_DST >> 8) & 0xFF; out[11] = TEST_DST & 0xFF;
    out[12] = 0x02;
    out[30] = (uint8_t)burst_type;
    int n = 57;
    if (burst_type == VOICE_HEAD || burst_type == VOICE_TERM) n = 54;
    else if (pos == 'A') { n = 52; out[GV_CTRL_OFF] = GV_CTRL_SYNC; }
    else if (pos == 'E') { n = 66; out[GV_CTRL_OFF] = e_ctrl; memcpy(out + GV_BE_LC_FLCO_OFF, lc, 9); }
    else out[GV_CTRL_OFF] = 0x06;
    translator_ipsc_voice_received(tr, out, n, TEST_TS, burst_type);
}

/* Bursts named by pos ("ABCDEF", or fewer when RF lost some); E carries lc. */
static void bursts(const char *pos, const uint8_t *lc, uint8_t e_ctrl)
{
    for (; *pos; pos++) send(SLOT1_VOICE, *pos, lc, e_ctrl);
}

/* The LCs of the superframes HBP got, in order: each one whose B, C, D and E (vseq 1-4)
 * arrived back to back, reassembled from their embedded fragments. */
static int superframe_lcs(uint8_t lcs[][9], int max)
{
    int n = 0, want = 1;
    uint8_t frag[4][4];
    for (int i = 0; i < ncap; i++) {
        int vseq = cap[i][15] & 0x0F;
        if ((cap[i][15] & 0x30) || vseq < 1 || vseq > 4) { want = 1; continue; }
        if (vseq == 1) want = 1;
        if (vseq != want) { want = 1; continue; }
        dmr_bit bits[264];
        dmr_bytes_to_bits(cap[i] + 20, 33, bits);
        dmr_bits_to_bytes(bits + 116, 32, frag[vseq - 1]);
        if (++want == 5) {
            if (n < max) dmr_decode_emblc((const uint8_t (*)[4])frag, lcs[n++]);
            want = 1;
        }
    }
    return n;
}

static int is_call_lc(const uint8_t lc[9])
{
    unsigned dst = (unsigned)(lc[3] << 16 | lc[4] << 8 | lc[5]);
    unsigned src = (unsigned)(lc[6] << 16 | lc[7] << 8 | lc[8]);
    return lc[0] == FLCO_GROUP && dst == TEST_DST && src == TEST_SRC;
}

static int identity_kept(void)
{
    for (int i = 0; i < ncap; i++) {
        unsigned src = (unsigned)(cap[i][5] << 16 | cap[i][6] << 8 | cap[i][7]);
        unsigned dst = (unsigned)(cap[i][8] << 16 | cap[i][9] << 8 | cap[i][10]);
        if (src != TEST_SRC || dst != TEST_DST) return 0;
    }
    return 1;
}

static void start(const Config *cfg, ev_loop *loop)
{
    tr = translator_new(cfg, loop);
    translator_set_protocols(tr, (struct ipsc *)1, (struct hbp *)1);
    ncap = 0; seq = 1;
    send(VOICE_HEAD, 0, NULL, 0);
}

int main(void)
{
    Config cfg; char err[4096];
    if (config_load("tests/parity.toml", &cfg, err, sizeof err)) {
        fprintf(stderr, "config: %s\n", err); return 2;
    }
    ev_loop *loop = ev_new();
    uint8_t lcs[16][9];
    int n;

    /* clean call: the alias goes out whole in the next superframe */
    start(&cfg, loop);
    bursts("ABCDEF", TA_HEAD, GV_BE_FLAG);
    bursts("ABCDEF", CALL_LC, GV_BE_FLAG);
    n = superframe_lcs(lcs, 16);
    check(n == 2, "clean call: two whole superframes");
    check(n >= 1 && is_call_lc(lcs[0]), "clean call: the revealing superframe keeps the call LC");
    check(n >= 2 && !memcmp(lcs[1], TA_HEAD, 9), "clean call: the next superframe carries the alias");
    check(identity_kept(), "clean call: DMRD src/dst stay the call's");
    translator_free(tr);

    /* RF lost A, B and C before the alias' E: it still goes out whole after it */
    start(&cfg, loop);
    bursts("ABCDEF", CALL_LC, GV_BE_FLAG);
    bursts("DEF", TA_BLOCK, GV_BE_FLAG);
    bursts("ABCDEF", CALL_LC, GV_BE_FLAG);
    n = superframe_lcs(lcs, 16);
    check(n == 2 && !memcmp(lcs[1], TA_BLOCK, 9), "RF loss: the alias block arrives whole");
    translator_free(tr);

    /* an E whose LC failed its check relays nothing */
    start(&cfg, loop);
    bursts("ABCDEF", TA_HEAD, GV_CTRL_LC_ERR | GV_BE_FLAG);
    bursts("ABCDEF", CALL_LC, GV_BE_FLAG);
    n = superframe_lcs(lcs, 16);
    check(n == 2 && is_call_lc(lcs[1]), "LC error: no alias relayed");
    translator_free(tr);

    /* bad AMBE on the E frame does not affect its LC */
    start(&cfg, loop);
    bursts("ABCDEF", TA_HEAD, GV_CTRL_EMB_LC | 0x06 | GV_CTRL_BAD1);
    bursts("ABCDEF", CALL_LC, GV_BE_FLAG);
    n = superframe_lcs(lcs, 16);
    check(n == 2 && !memcmp(lcs[1], TA_HEAD, 9), "bad AMBE: alias still relayed");
    translator_free(tr);

    /* an alias still queued when a call ends is not sent on the next call */
    start(&cfg, loop);
    bursts("ABCDEF", TA_HEAD, GV_BE_FLAG);
    send(VOICE_TERM, 0, NULL, 0);
    ncap = 0;
    send(VOICE_HEAD, 0, NULL, 0);
    bursts("ABCDEF", CALL_LC, GV_BE_FLAG);
    n = superframe_lcs(lcs, 16);
    check(n == 1 && is_call_lc(lcs[0]), "next call: starts with the call LC");
    translator_free(tr);

    printf("Talker Alias relay (IPSC->HBP): %d checks, %d failures\n", checks, fails);
    ev_free(loop);
    return fails ? 1 : 0;
}
