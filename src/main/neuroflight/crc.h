#ifndef CRC_H
#define CRC_H

#include "neuroflight/defines.h"

crc_t compute_crc(const uint8_t* block, size_t block_size);

#endif