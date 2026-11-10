/* crc.c — 5-bit Embedded-LC checksum (crc.py csum5).
 *
 * Python: accum = sum(bytes) % 31; take that one byte as 8 bits big-endian,
 * drop the top 3 bits, leaving the low 5 bits.  We return those 5 bits in the
 * low 5 bits of a uint8_t. */
#include "dmr.h"

uint8_t dmr_csum5(const uint8_t lc[9])
{
    unsigned accum = 0;
    for (int i = 0; i < 9; i++)
        accum += lc[i];
    return (uint8_t)(accum % 31) & 0x1F;
}

/* CCITT CRC-16, inverted (ETSI B.3.7); the caller XORs the PDU's mask (data header 0xCCCC). */
uint16_t dmr_crc16_ccitt(const uint8_t *d, int n)
{
    unsigned c = 0;
    for (int i = 0; i < n; i++) {
        c ^= (unsigned)d[i] << 8;
        for (int b = 0; b < 8; b++) c = (c & 0x8000) ? ((c << 1) ^ 0x1021) : (c << 1);
    }
    return (uint16_t)(~c & 0xFFFF);
}

static unsigned crc9_step(unsigned crc, unsigned byte, int bits)
{
    unsigned v = 0x80u >> (8 - bits);
    for (int i = 0; i < 8; i++) {
        unsigned msb = crc & 0x100;
        crc = (crc << 1) & 0x1FF;
        if (byte & v) crc++;
        if (msb) crc ^= 0x59;
        v >>= 1;
    }
    return crc;
}

/* Confirmed data block CRC-9 over the block's data and its 7-bit serial, inverted; the caller XORs
 * the rate's mask (rate 3/4: 0x1FF). Bit order as node-dmr-lib CRC9.js, checked on captured blocks. */
uint16_t dmr_crc9(const uint8_t *d, int n, int serial)
{
    unsigned crc = 0;
    for (int i = 0; i < n; i++) crc = crc9_step(crc, d[i], 8);
    crc = crc9_step(crc, (unsigned)serial, 7);
    for (int i = 0; i < 8; i++) crc = (crc & 0x100) ? (((crc << 1) & 0x1FF) ^ 0x59) : ((crc << 1) & 0x1FF);
    return (uint16_t)(~crc & 0x1FF);
}

/* Packet data CRC-32 (ETSI B.3.9) over user data + pad: byte pairs swapped, MSB first, init 0.
 * Stored little-endian in the last 4 octets of the last block. */
uint32_t dmr_crc32_data(const uint8_t *d, int n)
{
    uint32_t reg = 0;
    for (int i = 0; i < n; i++) {
        uint8_t byte = (i ^ 1) < n ? d[i ^ 1] : d[i];
        for (int b = 7; b >= 0; b--) {
            uint32_t msb = reg >> 31;
            reg <<= 1;
            if (msb ^ ((byte >> b) & 1u)) reg ^= 0x04C11DB7u;
        }
    }
    return reg;
}
