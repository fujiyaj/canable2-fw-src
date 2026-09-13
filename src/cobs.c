//
// cobs: Consistent Overhead Byte Stuffing. See inc/cobs.h.
//

#include "cobs.h"

size_t cobs_encode(const uint8_t *src, size_t src_len, uint8_t *dst)
{
    size_t read_idx = 0;
    size_t write_idx = 1;
    size_t code_idx = 0;
    uint8_t code = 1;

    while (read_idx < src_len) {
        if (src[read_idx] == 0) {
            dst[code_idx] = code;
            code = 1;
            code_idx = write_idx++;
            read_idx++;
        } else {
            dst[write_idx++] = src[read_idx++];
            code++;
            if (code == 0xFF) {
                dst[code_idx] = code;
                code = 1;
                code_idx = write_idx++;
            }
        }
    }
    dst[code_idx] = code;
    return write_idx;
}

size_t cobs_decode(const uint8_t *src, size_t src_len, uint8_t *dst, size_t dst_max)
{
    size_t read_idx = 0;
    size_t write_idx = 0;

    while (read_idx < src_len) {
        uint8_t code = src[read_idx];
        if (code == 0)
            return (size_t)-1; // a literal 0x00 can never appear inside an encoded packet

        read_idx++;
        size_t run = (size_t)code - 1;
        if (read_idx + run > src_len)
            return (size_t)-1; // truncated/malformed input

        for (size_t i = 0; i < run; i++) {
            if (write_idx >= dst_max)
                return (size_t)-1;
            dst[write_idx++] = src[read_idx++];
        }

        // A code of 0xFF means "254 non-zero bytes, no implied zero
        // follows" (that's how the encoder avoids ever emitting a real
        // 0x00 mid-stream for long non-zero runs) -- every other code
        // implies a zero byte next, EXCEPT when this was the very last
        // group in the packet (nothing left to separate).
        if (code != 0xFF && read_idx < src_len) {
            if (write_idx >= dst_max)
                return (size_t)-1;
            dst[write_idx++] = 0;
        }
    }

    return write_idx;
}
