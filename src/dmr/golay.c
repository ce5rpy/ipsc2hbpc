/* golay.c — Golay(23,12) codeword + parity used by AMBE conversion
 * (ambe_utils.py golay2312 / parity). */
#include "dmr_internal.h"

uint32_t dmr_golay2312(uint32_t cw)
{
    const uint32_t POLY = 0xAE3;
    cw &= 0xFFF;             /* data bits only */
    uint32_t c = cw;         /* save original codeword */
    for (int i = 1; i <= 12; i++) {
        if (cw & 1)
            cw ^= POLY;
        cw >>= 1;
    }
    return (cw << 12) | c;
}

int dmr_parity(uint32_t cw)
{
    uint32_t p = cw & 0xFF;
    p ^= (cw >> 8) & 0xFF;
    p ^= (cw >> 16) & 0xFF;
    p ^= p >> 4;
    p ^= p >> 2;
    p ^= p >> 1;
    return (int)(p & 1);
}

/* Golay(20,8) slot type (ETSI TS 102 361-1 B.3.1): the code is linear, so the 12 parity bits of
 * any byte are the XOR of those of its set bits (MMDVMHost ENCODING_TABLE_2087 at 1, 2, 4 ... 128). */
uint32_t dmr_slot_type_encode(int color_code, int data_type)
{
    static const uint16_t PARITY[8] = { 0xB08E, 0xE093, 0x70A9, 0x60DC, 0x7036, 0xD06C, 0x90D9, 0xA03D };
    unsigned value = (unsigned)((color_code & 0xF) << 4 | (data_type & 0xF));
    unsigned parity = 0;
    for (int i = 0; i < 8; i++)
        if (value & (1u << i)) parity ^= PARITY[i];
    return (uint32_t)(value << 12 | (parity & 0xFF) << 4 | parity >> 12);
}
