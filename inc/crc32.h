#ifndef _CRC32_H
#define _CRC32_H

#include <stddef.h>
#include <stdint.h>

// CRC-32/ISO-HDLC (the common "zlib"/PKZIP CRC-32: poly 0xEDB88320
// reflected, init 0xFFFFFFFF, final XOR 0xFFFFFFFF). Internal use only
// (flash_store.c's integrity check) -- unlike crc16.h (which protects the
// binproto wire format a host must independently reimplement), nothing
// outside this firmware ever needs to reproduce this value, so any correct
// CRC-32 would do; this one was picked only because it's the most commonly
// available reference to check a from-scratch implementation against
// (check value 0xCBF43926 for the ASCII string "123456789").
uint32_t crc32_ieee(const uint8_t *data, size_t len);

#endif // _CRC32_H
