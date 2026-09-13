#ifndef _CRC16_H
#define _CRC16_H

#include <stddef.h>
#include <stdint.h>

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no input/output reflection,
// xorout 0x0000. Used by binproto.c to protect each decoded frame (header +
// payload, computed before appending the CRC field itself).
uint16_t crc16_ccitt_false(const uint8_t *data, size_t len);

#endif // _CRC16_H
