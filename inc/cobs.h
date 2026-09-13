#ifndef _COBS_H
#define _COBS_H

#include <stddef.h>
#include <stdint.h>

// Consistent Overhead Byte Stuffing. Used to frame binary protocol packets
// over the USB CDC byte stream (see binproto.c): each encoded packet is
// followed by a single 0x00 delimiter, and 0x00 never appears inside an
// encoded packet, so a receiver can always resynchronize on the next 0x00
// even if bytes were dropped or garbled.
//
// Neither function appends/expects the trailing 0x00 delimiter -- that's
// the caller's job (framing), not COBS's (byte stuffing).

// Encodes src[0..src_len) into dst. dst must be at least
// COBS_ENCODED_MAX(src_len) bytes. Returns the encoded length.
#define COBS_ENCODED_MAX(src_len) ((src_len) + (((src_len) + 253) / 254))
size_t cobs_encode(const uint8_t *src, size_t src_len, uint8_t *dst);

// Decodes an encoded buffer (src, WITHOUT the trailing 0x00) into dst.
// dst_max bounds how many bytes may be written to dst. Returns the decoded
// length, or (size_t)-1 if src is malformed or would overflow dst_max.
size_t cobs_decode(const uint8_t *src, size_t src_len, uint8_t *dst, size_t dst_max);

#endif // _COBS_H
