#ifndef _VESC_CAN_H
#define _VESC_CAN_H

#include <stdint.h>

#include "params.h" // MOTOR_COUNT

// VESC CAN bridge: encodes SET_CURRENT commands and decodes VESC status
// frames (STATUS/STATUS_4/STATUS_5) into one motor_feedback_t per motor,
// for up to MOTOR_COUNT VESCs sharing one physical CAN bus. This is
// control.c's only dependency for talking to the motor -- control.c does
// not touch can.h / FDCAN, or VESC packet IDs/units, directly. Output-axis
// unit conversion (gear_ratio, motor_direction, position_offset) happens
// here, not in control.c -- by the time control_step() reads
// vesc_can_feedback(motor_index), actual_position/actual_velocity are
// already in output-shaft rad / rad/s.
//
// ---------------------------------------------------------------------
// RX dispatch: sender_id -> motor_index
// ---------------------------------------------------------------------
// Every VESC status frame's extended CAN ID is (packet_id << 8) |
// sender_id (sender_id = the transmitting VESC's own configured CAN ID,
// the low byte). vesc_can_poll() looks up which motor_index that
// sender_id belongs to with a linear scan over the MOTOR_COUNT-entry
// motor_config[] table (motor_index_from_vesc_id() in vesc_can.c) --
// trivial at MOTOR_COUNT==4, no hash table needed. A frame whose
// sender_id doesn't match any configured motor is ignored.
//
// Duplicate vesc_can_id across motors is accepted at write time (see
// params.h's PARAM_VESC_CAN_ID comment) but makes RX dispatch ambiguous:
// the linear scan returns the FIRST matching motor_index, so only that one
// motor ever sees status frames from the shared ID -- the other motor's
// feedback goes stale and (once its own can_timeout_ms elapses)
// eventually reports vesc_alive == false, on top of control.c's
// enable-time duplicate-ID rejection. This is treated as an acceptable,
// transient, IDLE-only misconfiguration state, not something this file
// tries to further defend against.
//
// ---------------------------------------------------------------------
// Position source: STATUS_4 pid_pos + p_pid_ang_div, NOT STATUS_5 tachometer
// ---------------------------------------------------------------------
// STATUS_5's tachometer looked like the obvious multi-turn source, but it's
// a coarse, quantized value (VESC's FOC loop buckets the electrical angle
// into 6 steps of 60 degrees each and accumulates *that*, not a continuous
// angle) and depends on the same electrical-angle estimate STATUS_4 uses
// anyway -- in practice it doesn't track reliably enough to be the primary
// position feedback. It's still decoded (see vesc_raw_t) but only for
// debug/sanity-check use, never fed into actual_position.
//
// Instead, position comes from STATUS_4's pid_pos, decoded here as a
// wrapping 0-360deg angle and unwrapped in software using motor_config[i].
// angle_span_motor_rev, which MUST match that VESC's own p_pid_ang_div
// (p_pid_ang_div = pole_pairs * angle_span_motor_rev) -- see that
// register's comment in params.h. The unwrap step is assisted by the most
// recent STATUS erpm to disambiguate large jumps across dropped frames
// (predict the expected delta from erpm, then pick the unwrap count that
// makes the observed wrapped delta match the prediction) -- see
// unwrap_pid_pos() in vesc_can.c.
//
// Safety note: this firmware sends a SET_CURRENT command to every
// configured motor every control period (rate-gated to ~500Hz/motor -- see
// main.c and vesc_can_tx_service()) unconditionally, including 0A during
// FAULT/disable -- so each VESC's *own* CAN command timeout (default 0.5s
// on stock VESC firmware) should be left at its default, not disabled. It
// is the last line of defense if this board itself hangs (TIM ISR dies,
// FDCAN wedges, etc.) -- with it disabled, a hung G431 would leave a VESC
// driving its last received current forever instead of coasting to a stop.
typedef struct
{
    float actual_position;   // rad, output shaft (gear_ratio/motor_direction/position_offset already applied)
    float actual_velocity;   // rad/s, output shaft
    float actual_current;    // A
    float actual_duty;       // -1..1
    float actual_voltage;    // V
    float actual_temperature;// degC

    uint32_t vesc_last_rx_ms; // timestamp of the most recent valid VESC frame of ANY kind, for this motor

    // Feedback validity, split by what each control source actually needs
    // -- see params.h's PARAM_VESC_ONLINE/PARAM_VELOCITY_FEEDBACK_VALID/
    // PARAM_POSITION_FEEDBACK_VALID/PARAM_TEMPERATURE_VALID comments for
    // exactly what gates what. vesc_alive is diagnostic only and gates
    // nothing by itself -- CURRENT and VELOCITY both require
    // velocity_feedback_valid (actual_current and actual_velocity both
    // come only from STATUS), and POSITION additionally requires STATUS_4
    // plus a trustworthy unwrap. A slow STATUS_4 rate on this VESC (e.g.
    // temperature-only) never faults a running CURRENT/VELOCITY loop.
    uint8_t vesc_alive;              // any status frame (STATUS/STATUS_4/STATUS_5) seen recently -- diagnostic only
    uint8_t velocity_feedback_valid; // STATUS fresh
    uint8_t position_feedback_valid; // STATUS_4 fresh AND STATUS fresh AND unwrap baseline set AND last unwrap correction trustworthy
    uint8_t temperature_valid;       // STATUS_4 fresh
    uint8_t voltage_valid;           // STATUS_5 fresh -- see params.h PARAM_MIN_BUS_VOLTAGE/PARAM_MAX_BUS_VOLTAGE
} motor_feedback_t;

// Bring up the (single, shared) CAN bus at params_bus()->can_bitrate. Call
// once at startup, after params_init().
void vesc_can_init(void);

// Resets one motor's decoded state (raw wire-format cache AND the public
// motor_feedback_t) back to boot state -- in particular, drops the pid_pos
// unwrap accumulator/baseline. MUST be called whenever a config field that
// changes how that motor's frames are addressed or interpreted changes:
// vesc_can_id, pole_pairs, gear_ratio, motor_direction, angle_span_motor_rev.
// Without this, e.g. changing pole_pairs mid-session would reinterpret an
// accumulator built under the old value, silently producing a wrong
// position. params.c's params_write() calls this after each such write
// succeeds (all of them are refused while that motor's state == RUNNING,
// so this only ever runs from IDLE). can_bitrate is deliberately handled
// separately (bus-wide, not per-motor) -- changing it doesn't actually
// reconfigure the FDCAN peripheral until the next vesc_can_init() (v0.1:
// reboot required after changing it).
void vesc_can_reset_feedback(uint8_t motor_index);

// Updates motor_index's SET_CURRENT "latest command wins" mailbox -- does
// NOT touch CAN hardware and does NOT itself decide when to actually
// publish (see vesc_can_tx_service() for the round-robin TX, and main.c
// for the ~500Hz/motor rate gate that decides which motors' mailboxes get
// written on any given 1kHz tick). Called from main.c's rate-gated publish
// step, not from control_step() directly -- see control.h's control_step()
// comment. A no-op (mailbox left untouched) for a motor whose vesc_can_id
// is unconfigured (0xFF) -- there's no real VESC to send to, and main.c's
// rate gate calls this for every motor_index every tick regardless of
// which ones are actually configured, so without this guard an
// unconfigured slot would waste bus bandwidth on frames addressed to
// nobody.
//
// Deliberately does not use can.h's can_tx() (a FIFO software queue
// drained by can_process()): if main() ever stalls for a few ms, a FIFO'd
// SET_CURRENT would replay a whole backlog of stale current commands once
// it catches up, oldest first, which is never what motion control wants.
// Instead this just overwrites a single-slot mailbox per motor -- at most
// one not-yet-transmitted current value exists per motor at any time, so a
// burst of calls (5A, 3A, 1A, 0A, ...) while the bus is momentarily busy
// collapses to "send the last one." See vesc_can_tx_service() for how the
// mailboxes actually reach the bus, and vesc_can_current_overwrite_count() for
// how often a value gets overwritten before it ever does.
void vesc_can_set_current(uint8_t motor_index, float amps);

// Services the SET_CURRENT mailboxes: if the hardware Tx FIFO/Queue is
// confirmed completely empty (nothing of ours still in flight -- see
// FDCAN_TX_FIFO_DEPTH in vesc_can.c) AND at least one motor's mailbox has
// a newer value waiting, submits the NEXT one in round-robin order
// (motor0 -> motor1 -> motor2 -> motor3 -> motor0 -> ...) among whichever
// mailboxes are currently marked dirty -- this is what spreads the (up to)
// 4 motors' frames out over time instead of bursting them back-to-back,
// and is a separate concern from main.c's ~500Hz/motor rate gate that
// decides how often each mailbox gets a fresh value to begin with. Call
// every main loop iteration, NOT just once per control tick -- the FIFO
// can empty out between ticks, and servicing promptly keeps whichever
// mailbox is next in line as fresh as possible by the time it actually
// goes out. A no-op when there's nothing new to send or a frame is still
// in flight, so calling it liberally costs nothing.
void vesc_can_tx_service(void);

// Diagnostic counter: how many times vesc_can_set_current(motor_index, ...)
// overwrote a value that had not yet reached the bus -- NOT a CAN hardware
// transmit failure count, see params.h PARAM_CAN_TX_CURRENT_OVERWRITE_COUNT.
uint32_t vesc_can_current_overwrite_count(uint8_t motor_index);

// Drain pending CAN RX frames (shared bus, all motors), decode any VESC
// status frames, dispatch each by sender_id -> motor_index (see the RX
// dispatch header comment), apply that motor's gear_ratio/motor_direction/
// position_offset (and, for position, the ERPM-assisted pid_pos unwrap),
// and update that motor's motor_feedback_t. Call once per control period,
// before the per-motor control_step() calls run.
void vesc_can_poll(uint32_t now_ms);

// Read-only accessor for one motor's 0x60xx register group storage.
const motor_feedback_t *vesc_can_feedback(uint8_t motor_index);

#endif // _VESC_CAN_H
