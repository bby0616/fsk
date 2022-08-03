#ifndef __CRC16__
#define __CRC16__

#include "stdlib.h"

unsigned short crc16(unsigned short crc, unsigned char const *buffer, size_t len);

#endif