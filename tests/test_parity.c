/* test_parity.c — feed identical IPSC frames through the C translator and
 * compare the synthesized DMRD against the Python reference (stream-id masked).
 *
 * Stubs the ipsc and hbp layers so only translate.c + the dmr module run. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../src/translate.h"
#include "../src/config.h"
#include "../src/eventloop.h"

/* ---- captured DMRD output ---- */
#define MAXCAP 64
static uint8_t cap[MAXCAP][128];
static int     caplen[MAXCAP];
static int     ncap = 0;

/* ---- stubbed protocol layer ---- */
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
void hbp_send_dmrd(struct hbp *hb, const uint8_t *d, int n) {
    (void)hb;
    if (ncap < MAXCAP) { memcpy(cap[ncap], d, (size_t)n); caplen[ncap] = n; ncap++; }
}

static void stop_cb(ev_loop *loop, void *ud){ (void)ud; ev_stop(loop); }

static int hexv(char c){ if(c>='0'&&c<='9')return c-'0'; if(c>='a'&&c<='f')return c-'a'+10; return -1; }
static int unhex(const char *s, uint8_t *out){ int n=0; for(;s[0]&&s[1]&&hexv(s[0])>=0;s+=2) out[n++]=(uint8_t)((hexv(s[0])<<4)|hexv(s[1])); return n; }

int main(void)
{
    Config cfg; char err[4096];
    if (config_load("tests/parity.toml", &cfg, err, sizeof err)) {
        fprintf(stderr, "config: %s\n", err); return 2;
    }
    ev_loop *loop = ev_new();
    translator *tr = translator_new(&cfg, loop);
    translator_set_protocols(tr, (struct ipsc *)1, (struct hbp *)1);

    FILE *fi = fopen("tests/parity_in.txt", "r");
    if (!fi) { perror("tests/parity_in.txt"); return 2; }
    char line[512];
    while (fgets(line, sizeof line, fi)) {
        int ts, bt; char hex[400];
        if (sscanf(line, "%d %d %399s", &ts, &bt, hex) != 3) continue;
        uint8_t raw[400]; int n = unhex(hex, raw);
        translator_ipsc_voice_received(tr, raw, n, ts, bt);
    }
    fclose(fi);

    /* compare against reference (mask stream id 16:20) */
    FILE *fr = fopen("tests/parity_ref.txt", "r");
    if (!fr) { perror("tests/parity_ref.txt"); return 2; }
    int idx = 0, fails = 0;
    while (fgets(line, sizeof line, fr)) {
        uint8_t ref[128]; int rn = unhex(line, ref);
        if (idx >= ncap) { fprintf(stderr, "FAIL: ref[%d] but C produced only %d DMRD\n", idx, ncap); fails++; break; }
        uint8_t got[128]; memcpy(got, cap[idx], (size_t)caplen[idx]);
        if (caplen[idx] >= 20) memset(got + 16, 0, 4);   /* mask stream id */
        if (rn != caplen[idx] || memcmp(got, ref, (size_t)rn) != 0) {
            fails++;
            fprintf(stderr, "FAIL DMRD[%d]\n  got(%d): ", idx, caplen[idx]);
            for (int i=0;i<caplen[idx];i++) fprintf(stderr,"%02x",got[i]);
            fprintf(stderr, "\n  ref(%d): ", rn);
            for (int i=0;i<rn;i++) fprintf(stderr,"%02x",ref[i]);
            fprintf(stderr, "\n");
        }
        idx++;
    }
    fclose(fr);
    if (idx != ncap) { fprintf(stderr, "FAIL: C produced %d DMRD, ref had %d\n", ncap, idx); fails++; }

    printf("Parity (IPSC->HBP): %d DMRD compared, %d failures\n", idx, fails);

    /* ---- inbound parity (HBP -> IPSC): HEAD + TERM, compared verbatim ---- */
    fi = fopen("tests/parity_in_dmrd.txt", "r");
    fr = fopen("tests/parity_ref_gv.txt", "r");
    int in_fails = 0;
    if (fi && fr) {
        while (fgets(line, sizeof line, fi)) {
            uint8_t raw[128]; int n = unhex(line, raw);
            translator_hbp_voice_received(tr, raw, n);
        }
        /* HEAD/voice/TERM are now clocked out via the delivery timer; run the
         * loop briefly so those timers fire and emit the GROUP_VOICE frames. */
        ev_timer_after(loop, 0.4, stop_cb, loop);
        ev_run(loop);
        int gidx = 0;
        while (fgets(line, sizeof line, fr)) {
            uint8_t ref[128]; int rn = unhex(line, ref);
            if (gidx >= ngv) { fprintf(stderr, "FAIL: ref GV[%d] but C produced only %d\n", gidx, ngv); in_fails++; break; }
            if (rn != gvlen[gidx] || memcmp(gvcap[gidx], ref, (size_t)rn) != 0) {
                in_fails++;
                fprintf(stderr, "FAIL GV[%d]\n  got(%d): ", gidx, gvlen[gidx]);
                for (int i=0;i<gvlen[gidx];i++) fprintf(stderr,"%02x",gvcap[gidx][i]);
                fprintf(stderr, "\n  ref(%d): ", rn);
                for (int i=0;i<rn;i++) fprintf(stderr,"%02x",ref[i]);
                fprintf(stderr, "\n");
            }
            gidx++;
        }
        printf("Parity (HBP->IPSC): %d GROUP_VOICE compared, %d failures\n", gidx, in_fails);
    }
    if (fi) fclose(fi);
    if (fr) fclose(fr);

    /* ---- private call smoke (C1a PVT_VOICE VOICE_HEAD → DMRD with CALL_P) ---- */
    int pvt_fails = 0;
    ncap = 0;
    {
        /* First VOICE_HEAD from captures/20260720-private-ipsc2hbp-ts1.pcap */
        static const char *pvt_hex =
            "81000ae511296cf2b7000fa00200006e540080dd0f6407f911430000000001"
            "40000a800a0060030000000fa06cf2b737ee0a00114971";
        uint8_t raw[128]; int n = unhex(pvt_hex, raw);
        translator_ipsc_voice_received(tr, raw, n, 1, 0x01);
        if (ncap < 1) {
            fprintf(stderr, "FAIL private: no DMRD produced from PVT_VOICE HEAD\n");
            pvt_fails++;
        } else {
            const uint8_t *d = cap[0];
            int flags = d[15];
            unsigned dst = ((unsigned)d[8] << 16) | ((unsigned)d[9] << 8) | d[10];
            unsigned src = ((unsigned)d[5] << 16) | ((unsigned)d[6] << 8) | d[7];
            if (!(flags & 0x40)) {
                fprintf(stderr, "FAIL private: DMRD flags 0x%02x missing CALL_P (0x40)\n", flags);
                pvt_fails++;
            }
            if (dst != 4000 || src != 7140023u) {
                fprintf(stderr, "FAIL private: src=%u dst=%u (want 7140023→4000)\n", src, dst);
                pvt_fails++;
            }
        }
    }
    /* HBP private → IPSC must emit PVT_VOICE (0x81), not GROUP_VOICE */
    ngv = 0;
    {
        /* Minimal private VHEAD DMRD: CALL_P|DATASYNC|VHEAD on TS1, dst=4000 src=7140023 */
        static const char *dmrd_hex =
            "444d524400"           /* DMRD + seq */
            "6cf2b7"               /* src */
            "000fa0"               /* dst 4000 */
            "00000001"             /* rptr */
            "61"                   /* flags: CALL_P|DATASYNC|VHEAD */
            "aabbccdd"             /* stream */
            "000000000000000000000000000000000000000000000000000000000000000000" /* 33 payload */
            "0000";                /* ber/rssi — length padded below */
        uint8_t raw[128]; int n = unhex(dmrd_hex, raw);
        while (n < 55) raw[n++] = 0;
        translator_hbp_voice_received(tr, raw, 55);
        if (ngv < 1) {
            fprintf(stderr, "FAIL private HBP→IPSC: no voice frame emitted\n");
            pvt_fails++;
        } else if (gvcap[0][0] != 0x81) {
            fprintf(stderr, "FAIL private HBP→IPSC: opcode 0x%02x want PVT_VOICE 0x81\n",
                    gvcap[0][0]);
            pvt_fails++;
        }
    }
    printf("Private smoke: %d failures\n", pvt_fails);

    translator_free(tr); ev_free(loop);
    return (fails || in_fails || pvt_fails) ? 1 : 0;
}
