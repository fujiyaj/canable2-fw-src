#ifndef _SHELL_H
#define _SHELL_H

#include <stdint.h>

// ASCII debug shell over USB CDC. Deliberately a thin layer: every command
// resolves to a params_read()/params_write() call (or, for enable/disable/
// clear_fault/zero, the same PARAM_ENABLE/PARAM_CLEAR_FAULT/PARAM_ZERO_POSITION
// writes params_write() would dispatch anyway) -- this file never touches
// bus_config_t/motor_config_t/motor_control_t or control.c/vesc_can.c state
// directly, so every safety property already built into params.c/control.c
// (RUNNING-time write guards, range checks, PARAM_ERR_CONFIG on enable,
// atomic ZERO, etc.) applies to shell commands automatically, for free.
//
// Commands (one per line, LF or CRLF terminated). <i> is a motor index
// (0..MOTOR_COUNT-1), required for MOTOR-scope registers and omitted for
// SYSTEM-scope ones -- see params.h's "4-motor addressing" comment and
// shell.c's top-of-file comment:
//   get [<i>] <name>       read a register, print its value
//   set [<i>] <name> <val> write a register
//   enable <i> / disable <i>  PARAM_ENABLE = 1 / 0 for motor i
//   clear_fault <i>        PARAM_CLEAR_FAULT for motor i
//   zero <i>               PARAM_ZERO_POSITION for motor i
//   save / load_defaults / factory_reset  SYSTEM-scope, no index
//   status                 one-shot human-readable snapshot, all motors
//   help                   list commands and register names
//
// Not a binary/ROS2 protocol -- see the project's design discussion for why
// that should be a separate framed protocol (COBS+CRC16 or similar) layered
// on the same USB CDC link rather than replacing this shell. This file's
// job is only the human/debug side.

// Feed one byte received over USB-CDC into the line parser. Call from
// usbd_cdc_if.c's cdc_process_motion_rx() for every buffered RX byte.
void shell_feed_byte(uint8_t b);

#endif // _SHELL_H
