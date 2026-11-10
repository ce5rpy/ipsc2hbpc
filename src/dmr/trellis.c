/* trellis.c — rate 3/4 Trellis code for 18-byte data PDUs (ETSI TS 102 361-1 B.2).
 * Tables as in MMDVMHost's DMRTrellis.cpp; bits are the 196 info bits of a burst. */
#include "dmr.h"

static const uint8_t ENCODE[64] = {
    0, 8, 4, 12, 2, 10, 6, 14,   4, 12, 2, 10, 6, 14, 0, 8,
    1, 9, 5, 13, 3, 11, 7, 15,   5, 13, 3, 11, 7, 15, 1, 9,
    3, 11, 7, 15, 1, 9, 5, 13,   7, 15, 1, 9, 5, 13, 3, 11,
    2, 10, 6, 14, 0, 8, 4, 12,   6, 14, 0, 8, 4, 12, 2, 10
};

static const uint8_t INTERLEAVE[98] = {
    0, 1, 8, 9, 16, 17, 24, 25, 32, 33, 40, 41, 48, 49, 56, 57, 64, 65, 72, 73, 80, 81, 88, 89, 96, 97,
    2, 3, 10, 11, 18, 19, 26, 27, 34, 35, 42, 43, 50, 51, 58, 59, 66, 67, 74, 75, 82, 83, 90, 91,
    4, 5, 12, 13, 20, 21, 28, 29, 36, 37, 44, 45, 52, 53, 60, 61, 68, 69, 76, 77, 84, 85, 92, 93,
    6, 7, 14, 15, 22, 23, 30, 31, 38, 39, 46, 47, 54, 55, 62, 63, 70, 71, 78, 79, 86, 87, 94, 95
};

/* Constellation point -> its two dibits as 4 bits (dibit +1 = 00, +3 = 01, -1 = 10, -3 = 11). */
static const uint8_t POINT_BITS[16] = {
    0x2, 0xA, 0x7, 0xF, 0xE, 0x6, 0xB, 0x3, 0xD, 0x5, 0x8, 0x0, 0x1, 0x9, 0x4, 0xC
};

void dmr_trellis34_encode(const uint8_t data18[18], dmr_bit out196[196])
{
    dmr_bit in[144];
    dmr_bytes_to_bits(data18, 18, in);
    uint8_t dibits[98];
    int state = 0;
    for (int i = 0; i < 49; i++) {
        int tribit = i < 48 ? (in[3*i] << 2 | in[3*i+1] << 1 | in[3*i+2]) : 0;
        uint8_t bits = POINT_BITS[ENCODE[state * 8 + tribit]];
        dibits[2*i] = bits >> 2; dibits[2*i+1] = bits & 3;
        state = tribit;
    }
    for (int i = 0; i < 98; i++) {
        out196[2*i]   = dibits[INTERLEAVE[i]] >> 1;
        out196[2*i+1] = dibits[INTERLEAVE[i]] & 1;
    }
}

int dmr_trellis34_decode(const dmr_bit in196[196], uint8_t data18[18])
{
    uint8_t dibits[98];
    for (int i = 0; i < 98; i++)
        dibits[INTERLEAVE[i]] = (uint8_t)(in196[2*i] << 1 | in196[2*i+1]);
    dmr_bit out[144];
    int state = 0;
    for (int i = 0; i < 49; i++) {
        uint8_t bits = (uint8_t)(dibits[2*i] << 2 | dibits[2*i+1]);
        int point = 0;
        while (POINT_BITS[point] != bits) point++;
        int tribit = 0;
        while (tribit < 8 && ENCODE[state * 8 + tribit] != point) tribit++;
        if (tribit == 8) return -1;   /* not error-free: no correction attempted */
        if (i < 48) { out[3*i] = tribit >> 2; out[3*i+1] = (tribit >> 1) & 1; out[3*i+2] = tribit & 1; }
        state = tribit;
    }
    dmr_bits_to_bytes(out, 144, data18);
    return 0;
}
