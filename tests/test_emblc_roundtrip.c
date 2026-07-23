/* test_emblc_roundtrip.c — dmr_decode_emblc() vs dmr_encode_emblc().
 *
 * There is no external (Python dmr_utils3) reference for the DECODE
 * direction — only the encoder was validated against it. So this checks the
 * only thing we can: decode(encode(lc)) reproduces lc exactly, for a spread
 * of LC byte patterns (all-zero, all-one, walking bit, incrementing, and the
 * real field layout used for a Talker Alias header/block) — including the
 * one bit the encoder's own table doesn't transmit directly, recovered via
 * checksum inversion (see dmr_decode_emblc's doc comment in dmr.h). Also
 * checks that decode reports invalid (checksum mismatch) on non-LC-shaped
 * garbage. */
#include <stdio.h>
#include <string.h>
#include "../src/dmr/dmr.h"

static int check_roundtrip(const char *label, const uint8_t lc[9])
{
    uint8_t frag[4][4];
    dmr_encode_emblc(lc, frag);

    uint8_t got[9];
    int ok = dmr_decode_emblc(frag, got);
    if (!ok) {
        fprintf(stderr, "FAIL %-20s: decode reported invalid checksum\n", label);
        return 1;
    }

    if (memcmp(got, lc, 9) != 0) {
        fprintf(stderr, "FAIL %-20s: roundtrip mismatch\n  lc:   ", label);
        for (int i=0;i<9;i++) fprintf(stderr,"%02x",lc[i]);
        fprintf(stderr, "\n  got:  ");
        for (int i=0;i<9;i++) fprintf(stderr,"%02x",got[i]);
        fprintf(stderr, "\n  want: ");
        for (int i=0;i<9;i++) fprintf(stderr,"%02x",lc[i]);
        fprintf(stderr, "\n");
        return 1;
    }
    return 0;
}

int main(void)
{
    int fails = 0, checks = 0;

    const uint8_t zero[9]  = {0,0,0,0,0,0,0,0,0};
    const uint8_t ones[9]  = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
    const uint8_t incr[9]  = {0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88};
    /* FLCO=0x00 group, fid=0, svc=0, dst=91, src=3120001 (mirrors real call LC) */
    const uint8_t group[9] = {0x00,0x00,0x00, 0x00,0x00,0x5b, 0x2f,0x9b,0x81};
    /* FLCO=0x04 TA_HEADER, fid=0, fmt/len=(2<<6)|6, text "TESTLB" */
    const uint8_t ta_hdr[9] = {0x04,0x00,0x86, 'T','E','S', 'T','L','B'};
    /* FLCO=0x05 TA_BLOCK1, fid=0, 7 text chars */
    const uint8_t ta_blk[9] = {0x05,0x00,' ', 'V','H','F', 0,0,0};

    struct { const char *label; const uint8_t *lc; } cases[] = {
        {"all-zero", zero}, {"all-one", ones}, {"incrementing", incr},
        {"group-call-lc", group}, {"ta-header", ta_hdr}, {"ta-block1", ta_blk},
    };
    for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++) {
        checks++;
        fails += check_roundtrip(cases[i].label, cases[i].lc);
    }

    /* Walking single bit across all 9 bytes -> 72 individual patterns. */
    for (int bitpos = 0; bitpos < 72; bitpos++) {
        uint8_t lc[9] = {0};
        lc[bitpos / 8] = (uint8_t)(1u << (7 - (bitpos % 8)));
        checks++;
        char label[32]; snprintf(label, sizeof label, "walk-bit-%d", bitpos);
        fails += check_roundtrip(label, lc);
    }

    /* Non-lc-shaped garbage: decode must report invalid, not fabricate a result. */
    uint8_t garbage[4][4] = {
        {0x11,0x22,0x33,0x44}, {0x55,0x66,0x77,0x88},
        {0x99,0xaa,0xbb,0xcc}, {0xdd,0xee,0xff,0x00},
    };
    uint8_t out[9];
    checks++;
    if (dmr_decode_emblc(garbage, out)) {
        fprintf(stderr, "FAIL garbage-checksum: decode reported valid on non-LC-shaped input\n");
        fails++;
    }

    printf("Embedded-LC decode roundtrip: %d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
