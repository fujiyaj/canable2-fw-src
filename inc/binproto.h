#ifndef _BINPROTO_H
#define _BINPROTO_H

#include <stdint.h>

// Binary protocol for ROS2 (or any non-human client) over the same USB CDC
// link shell.c's ASCII debug shell uses. Boots in ASCII mode; a client
// sends the ASCII line "binary\n" to switch over (gets "OK BINARY 2" back,
// still as plain ASCII text -- the switch takes effect only after that
// reply). Reverts to ASCII on a USB bus reset (cable replug/re-enumeration
// -- see usbd_conf.c's HAL_PCD_ResetCallback()) or an MCU reset, so a
// wedged/crashed ROS2 node can't strand the link in binary mode forever
// against a human who just plugged in a terminal.
//
// ---------------------------------------------------------------------
// v2: motor addressing
// ---------------------------------------------------------------------
// v1 was single-motor only. v2 adds an explicit u8 motor_index to every
// per-motor message (READ_PARAM/WRITE_PARAM/PARAM_RESPONSE/
// CONTROL_COMMAND/TELEMETRY) -- see params.h's "4-motor addressing"
// comment for which registers are MOTOR-scope (need a real 0..MOTOR_COUNT-1
// index) vs SYSTEM-scope (motor_index must be 0). This is a breaking wire
// change from v1, not an extension -- BINPROTO_VERSION bumped 1 -> 2 so a
// stale v1 client is rejected outright (BINPROTO_ERR_VERSION_MISMATCH)
// rather than silently misinterpreting the new payload layouts.
//
// ---------------------------------------------------------------------
// Framing
// ---------------------------------------------------------------------
// Each packet is COBS-encoded (see cobs.h) and followed by a single 0x00
// delimiter, so a receiver can always resynchronize on the next 0x00 even
// after dropped/garbled bytes. The COBS-decoded packet is:
//
//   u8  protocol_version   (BINPROTO_VERSION)
//   u8  message_type       (BINPROTO_MSG_*)
//   u16 sequence           little-endian
//   u16 payload_length     little-endian
//   u8  payload[payload_length]
//   u16 crc16              little-endian, CRC-16/CCITT-FALSE over every
//                          byte above (NOT including this field itself)
//
// All multi-byte fields are explicit little-endian, encoded/decoded byte
// by byte -- never a raw struct memcpy (would depend on this MCU's
// alignment/padding/endianness matching whatever writes the other end,
// which is exactly the kind of implicit coupling ROS2/Linux vs Cortex-M4
// should not rely on).
//
// A frame that fails CRC is dropped silently -- every field, including
// sequence, is inside the CRC-protected region, so nothing in a
// CRC-failed frame (not even the sequence to address a reply to) can be
// trusted. A frame that decodes correctly but has an unrecognized
// protocol_version or message_type gets a BINPROTO_MSG_ERROR reply
// instead, since the sequence field is trustworthy at that point.
//
// ---------------------------------------------------------------------
// Message types
// ---------------------------------------------------------------------
// GET_INFO/HEARTBEAT/READ_PARAM/WRITE_PARAM/SET_TELEMETRY_RATE are
// request/response, addressed by the client's sequence number.
// CONTROL_COMMAND is fire-and-forget (see below). TELEMETRY is
// server-initiated, periodic (see SET_TELEMETRY_RATE).
#define BINPROTO_VERSION 2

#define BINPROTO_MSG_GET_INFO           0x01 // req: empty. resp: same type, see handle_get_info(). SYSTEM (no motor_index)
#define BINPROTO_MSG_HEARTBEAT          0x02 // req: empty. resp: ACK. Feeds the SYSTEM-wide host_timeout_ms watchdog. SYSTEM
#define BINPROTO_MSG_READ_PARAM         0x10 // req: u8 motor_index, u16 param_id. resp: PARAM_RESPONSE or ERROR
#define BINPROTO_MSG_WRITE_PARAM        0x11 // req: u8 motor_index, u16 param_id, u8 datatype, value[size(datatype)]. resp: ACK or ERROR
#define BINPROTO_MSG_PARAM_RESPONSE     0x12 // resp only: u8 motor_index, u16 param_id, u8 datatype, value[size(datatype)]
#define BINPROTO_MSG_CONTROL_COMMAND    0x20 // req: u8 motor_index, see control_command_t / control_apply_command(). No ACK -- see below
#define BINPROTO_MSG_SET_TELEMETRY_RATE 0x30 // req: u16 period_ms (0 = disabled). resp: ACK. SYSTEM -- applies to every motor's TELEMETRY stream
#define BINPROTO_MSG_TELEMETRY          0x31 // server-initiated, periodic, one frame PER MOTOR per cycle -- see below
#define BINPROTO_MSG_ACK                0x7E // resp: empty payload
#define BINPROTO_MSG_ERROR              0x7F // resp: u8 error_code -- a param_result_t value (params.h) for
                                              // param/command-related failures, or a BINPROTO_ERR_* value below

#define BINPROTO_ERR_VERSION_MISMATCH 100
#define BINPROTO_ERR_UNKNOWN_TYPE     101
#define BINPROTO_ERR_MALFORMED        102

// READ_PARAM/WRITE_PARAM's motor_index is validated against param_scope(id)
// exactly like params_read()/params_write() (params.h) -- SYSTEM-scope IDs
// require motor_index == 0 (PARAM_ERR_RANGE otherwise), MOTOR-scope IDs
// require motor_index < MOTOR_COUNT.

// CONTROL_COMMAND is deliberately NOT ack'd per-packet: at 100-200Hz/motor
// that would double the USB CDC round-trip traffic for no real benefit.
// Instead each motor's TELEMETRY frame carries that motor's own
// last_control_sequence -- the client compares that against the sequence
// it last sent FOR THAT MOTOR to confirm receipt, the same way a video
// game client reconciles server state without waiting for a per-input ack.
// A rejected CONTROL_COMMAND (out-of-range control_source, out-of-range
// target_position, unknown/out-of-range motor_index, etc -- see
// control_apply_command()) still gets an ERROR reply, so a persistently-bad
// command doesn't fail silently forever with zero feedback.
//
// Wire payload (25 bytes): u8 motor_index, then control_command_t
// (control.h) field for field, individually little-endian encoded (not
// memcpy'd):
//   u8  motor_index
//   u8  control_source
//   u8  profile_enable
//   u8  override_source
//   u8  override_enable
//   f32 target_position
//   f32 target_velocity
//   f32 target_current
//   f32 velocity_ff
//   f32 current_ff

// TELEMETRY: rather than pack all MOTOR_COUNT motors into one frame (which
// would exceed this protocol's RAW_MAX=128-byte pre-COBS budget at
// MOTOR_COUNT==4), one period emits MOTOR_COUNT separate TELEMETRY frames,
// one per motor, back to back. Every frame in the same period shares the
// same telemetry_cycle_sequence so the client can group them back into one
// 4-motor snapshot without relying on arrival order or timing:
//   u32 timestamp_ms              HAL_GetTick() at the time this cycle started
//   u32 telemetry_cycle_sequence  same value on all MOTOR_COUNT frames of one cycle, incremented once per cycle
//   u8  motor_index
//   u32 last_control_sequence     this motor's own last-applied CONTROL_COMMAND sequence
//   u8  state                     controller_state_t
//   u8  active_source             control_source_t
//   u8  flags                     bit0 vesc_alive, bit1 velocity_feedback_valid, bit2 position_feedback_valid,
//                                 bit3 temperature_valid, bit4 voltage_valid
//   f32 actual_position
//   f32 actual_velocity
//   f32 actual_current
//   f32 actual_duty
//   f32 actual_voltage
//   f32 actual_temperature
//   f32 effective_position_ref
//   f32 effective_velocity_ref
//   f32 effective_current_cmd
//   u32 fault_active               this motor's OWN causes only -- see params.h PARAM_SYSTEM_FAULT_ACTIVE
//   u32 fault_latched
// System-wide diagnostics (system_fault_active/latched, control_overrun_count,
// control_dt_max_us, cdc_tx_*) are NOT in TELEMETRY -- poll them via
// READ_PARAM (motor_index=0) if needed; they change far less often than
// per-motor feedback.

// Feed one byte received over USB-CDC (binary mode only -- see
// binproto_active()) into the frame reassembler/parser.
void binproto_feed_byte(uint8_t b);

// Call every main loop iteration (like vesc_can_tx_service()) with the
// current HAL_GetTick() value -- drives periodic TELEMETRY sending per the
// last SET_TELEMETRY_RATE. A no-op when telemetry is disabled (period 0,
// the default) or not due yet.
void binproto_tick(uint32_t now_ms);

// True once the ASCII "binary" command has switched the link into binary
// mode. usbd_cdc_if.c's cdc_process_motion_rx() checks this to route each
// RX byte to binproto_feed_byte() instead of shell_feed_byte().
uint8_t binproto_active(void);

// Called by shell.c's "binary" command handler.
void binproto_enter_binary_mode(void);

// Called from usbd_conf.c's HAL_PCD_ResetCallback() (USB bus reset --
// cable replug/re-enumeration) so a ROS2 node dying mid-session can't stay
// permanently between a human and their terminal.
void binproto_force_ascii_mode(void);

#endif // _BINPROTO_H
