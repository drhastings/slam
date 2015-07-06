#pragma once

#include <stdint.h>
#include "matrices.h"

int decode_packet(int16_t ranges[360], double readings[360 * 2], 
          struct mat *** map, char packet[22]);

