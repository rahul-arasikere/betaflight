#include "crc.h"

crc_t compute_crc(const uint8_t* block, size_t block_size) {
	// MCRF4XX algorithm
    uint8_t tmp;
    crc_t crcAccum = 0xffff;
    for (int i = 0; i < block_size; i++) {
        tmp = block[i] ^ (uint8_t)(crcAccum & 0xff);
        tmp ^= (tmp << 4);
        crcAccum = (crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
    }
    return crcAccum;
}