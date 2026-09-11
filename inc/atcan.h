#ifndef _ATCAN_H
#define _ATCAN_H

// AT-frame protocol (matches cr09/tools atcan_bridge_node.cpp's wire format
// exactly, so this firmware is a drop-in replacement for the CH340 "AT-CAN"
// dongle on the wire):
//
//   41 54 | wire_addr(4, big-endian) | dlc(1) | data(dlc, 0..8) | 0D 0A
//   wire_addr = (29bit CAN ID << 3) | 0x04   (bit2 = extended-frame flag,
//               always set -- this firmware only speaks 29bit extended IDs)
//
// Unlike slcan there is no host-side "open port"/"set bitrate" command --
// this firmware is dedicated to one fixed bitrate (ATCAN_BITRATE, see
// atcan.c) and enables the bus itself at startup.

#define ATCAN_FRAME_MAX (2 + 4 + 1 + 8 + 2) // AT + addr(4) + dlc(1) + data(<=8) + CRLF

// Bring up the CAN bus at the fixed bitrate this firmware speaks. Call once
// from main() at startup (replaces the slcan 'O'/'S' commands).
void atcan_init(void);

// Feed one byte received over USB-CDC into the AT-frame parser state
// machine. A complete, well-formed frame is transmitted directly onto the
// CAN bus (via can_tx()). Malformed framing resyncs silently.
void atcan_feed_byte(uint8_t b);

// Build an outgoing AT-frame from a received CAN message into buf[].
// Returns the frame length, or -1 if the message can't be represented
// (FD frame, or >8 data bytes -- this firmware only relays classic frames).
int32_t atcan_build_frame(uint8_t *buf, FDCAN_RxHeaderTypeDef *frame_header, uint8_t *frame_data);

#endif // _ATCAN_H
