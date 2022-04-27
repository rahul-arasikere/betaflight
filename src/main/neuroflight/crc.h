#ifndef CRC_H
#define CRC_H

#include <stdint.h>
#include <stddef.h>

#define crc_t uint16_t
#define NUM_CRC_BYTES (sizeof(crc_t))

crc_t compute_crc(const uint8_t* block, size_t block_size);

#endif