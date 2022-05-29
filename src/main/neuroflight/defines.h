#pragma once

#include <stdint.h>
#include <stddef.h>

#define GRAPH_INPUT_SIZE 13
#define GRAPH_OUTPUT_SIZE 4

typedef uint16_t buffer_size_t;
typedef uint16_t crc_t ;

#define NUM_CRC_BYTES (sizeof(crc_t))
#define NUM_SIZE_BYTES (sizeof(buffer_size_t))
#define NUM_META_BYTES (NUM_SIZE_BYTES + NUM_CRC_BYTES)