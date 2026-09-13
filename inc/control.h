#ifndef _CONTROL_H
#define _CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "params.h"

// 1kHz control loop, one independent instance per motor (POSITION ->
// VELOCITY -> CURRENT -> VESC), control-source arbitration, bumpless source
// switching, fault handling -- plus system-level state shared across all
// MOTOR_COUNT motors (aggregate fault interlock, the 1kHz dispatcher's own
// stats). See params.h for the register map this operates on ("4-motor
// addressing" / "Two-tier host watchdog" header comments in particular) and
// control.c's top-of-file comment for the state machine this implements.

// control.c's own per-motor state machine output -- the storage behind the
// MOTOR-scope part of the 0x70xx register group (params.c's params_read()
// calls control_status(motor_index) to serve those IDs; see params.h's
// "Storage ownership" comment). NOT motor feedback -- that's vesc_can.h's
// motor_feedback_t. fault_active/fault_latched here are THIS MOTOR'S OWN
// causes only -- never set because of another motor's fault; see
// system_status_t below for the aggregate view.
typedef struct
{
    uint8_t  state;             // controller_state_t
    uint32_t fault_active;
    uint32_t fault_latched;
    uint8_t  active_source;     // control_source_t, reflects override
    uint8_t  profile_active;

    // Effective (post-sync/post-profile/post-blend) values -- see
    // PARAM_EFFECTIVE_* in params.h.
    float effective_position_ref;
    float effective_velocity_ref;
    float effective_current_cmd;
} control_status_t;

// System-wide state, one singleton -- the storage behind the SYSTEM-scope
// part of the 0x70xx group (PARAM_SYSTEM_FAULT_ACTIVE/LATCHED,
// PARAM_CONTROL_OVERRUN_COUNT/DT_MAX_US). See params.h's PARAM_SYSTEM_FAULT_ACTIVE
// comment for the escalation policy and control_recompute_system_fault().
typedef struct
{
    uint32_t system_fault_active;  // bit i = motor i's own fault_active is currently nonzero
    uint32_t system_fault_latched; // bit i = motor i's own fault_latched is currently nonzero (derived, no separate clear)

    // Diagnostics -- see PARAM_CONTROL_OVERRUN_COUNT / PARAM_CONTROL_DT_MAX_US.
    uint32_t overrun_count; // total 1ms ticks discarded by main()'s tick dispatcher, see control_notify_overrun()
    uint32_t dt_max_us;     // largest dt control_step() has ever actually been called with
} system_status_t;

void control_init(void);

// Call once per control period, once per motor, from main()'s tick
// dispatcher (not from an ISR -- see main.c's HAL_SYSTICK_Callback()/control
// loop for why). Caller must call control_recompute_system_fault() once
// after calling this for every motor_index in a given tick (see main.c) --
// the system-level interlock this motor observes (has ANY motor got a
// runtime-fatal fault right now) reflects the *previous* tick's aggregate,
// bounded to <=1ms/1 tick propagation latency; see control.c.
//   motor_index : 0..MOTOR_COUNT-1
//   dt          : actual elapsed time since the previous call for THIS
//                 motor, in seconds. NOT assumed to be a fixed 0.001f -- if
//                 main() had to discard backlogged ticks (see
//                 control_notify_overrun()), dt reflects the real elapsed
//                 time so the PI integrators/derivative terms don't
//                 silently under/over-integrate against a wrong period.
//   now_ms      : free-running millisecond tick (e.g. HAL_GetTick())
void control_step(uint8_t motor_index, float dt, uint32_t now_ms);

// Call once per control period, after control_step() has been called for
// every motor -- recomputes system_status_t's system_fault_active/latched
// from each motor's own control_status_t::fault_active/latched (level,
// live -- not edge-triggered, so it stays escalated for as long as any
// causing motor's own fault does). See control.c.
void control_recompute_system_fault(void);

// Call from main()'s tick dispatcher whenever it discards backlogged 1ms
// ticks (more than one was pending at once) -- accumulates into
// system_status_t::overrun_count. Purely a diagnostic; never affects dt
// (see control_step()'s comment) or anything else in the control loop.
// System-wide: one shared dispatcher drives every motor's control_step()
// each tick, this isn't per-motor.
void control_notify_overrun(uint32_t missed_ticks);

// Call from the USB command layer whenever a well-formed command is
// received, to feed the SYSTEM-WIDE host_timeout_ms watchdog
// (bus_config_t::host_timeout_ms -- "is the USB/ROS2 link itself alive").
// See params.h's "Two-tier host watchdog" comment: this does NOT feed any
// individual motor's own command-freshness tracking -- that's
// control_apply_command()'s job, driven only by CONTROL_COMMAND(motor_index).
void control_notify_host_rx(uint32_t now_ms);

// Marks whether the current command session requires a live host_timeout_ms
// watchdog before control_request_enable() will arm ANY motor's output
// stage -- see that function. System-wide (one USB link drives all
// motors). Transport-agnostic on purpose: control.c doesn't know what
// binproto.c or ASCII mode are, it just tracks "does whoever's talking
// right now need a watchdog". binproto.c calls this with true on entering
// binary mode and false on returning to ASCII (see
// binproto_enter_binary_mode()/binproto_force_ascii_mode()) -- a ROS2
// session that goes silent (USB unplugged, node crashed) must not leave
// any motor holding its last command forever with no way for anything to
// notice, whereas a human at an ASCII terminal deliberately holding a
// position with host_timeout_ms=0 is a normal, supervised bench-test case.
//
// Calling this with required=true also arms a "prove you're alive" gate:
// control_request_enable() additionally requires at least one
// control_notify_host_rx() call to have happened SINCE this call, not just
// ever. This makes the intended binary arm sequence explicit --
// binary -> HEARTBEAT (or CONTROL_COMMAND) -> configure -> enable -- rather
// than letting a stale host_last_rx_ms from a completely different earlier
// session (e.g. ASCII activity before switching to binary) count as
// "alive".
void control_require_host_watchdog(bool required);

// Read-only accessor for what control_require_host_watchdog() last set --
// params.c's PARAM_HOST_TIMEOUT_MS write handler uses this to refuse
// setting the timeout to 0 while any motor is RUNNING and a watchdog is
// required (see that write case's comment): control_request_enable()'s
// check alone only protects the moment of enabling, not the invariant
// afterward.
bool control_host_watchdog_required(void);

// Gate for PARAM_ENABLE writes (params.c's register dispatcher forwards the
// write here instead of poking motor_control_t directly): refuses to arm
// this motor's output stage with an incomplete config (current_limit == 0,
// vesc_can_id unset, VESC never seen online, this motor's vesc_can_id
// duplicated against another motor's, or -- only when
// control_require_host_watchdog(true) is in effect -- host_timeout_ms == 0
// or no host rx seen since that call -> PARAM_ERR_CONFIG, with
// FAULT_BIT_CONFIG_ERROR latched on this motor for visibility), and always
// refuses while this motor is latched in FAULT (PARAM_ERR_STATE).
param_result_t control_request_enable(uint8_t motor_index);

// Gate for a PARAM_ENABLE=0 write for one motor. Always succeeds.
void control_request_disable(uint8_t motor_index);

// Gate for PARAM_CLEAR_FAULT writes for one motor. Always trims this
// motor's fault_latched down to its current fault_active; additionally
// drops state FAULT -> IDLE if fault_active is 0 at the time of the call
// (see control.c header comment for why FAULT -> IDLE also requires this
// explicit trigger, not just the condition clearing on its own). Also
// triggers control_recompute_system_fault() so a resolved system-wide
// interlock clears as soon as every causing motor's own latch is clear --
// see params.h's PARAM_SYSTEM_FAULT_LATCHED comment.
void control_clear_fault(uint8_t motor_index);

// Gate for PARAM_ZERO_POSITION writes for one motor. Atomically shifts
// position_offset (motor_config_t), target_position (motor_control_t), and
// the position stage's internal reference by the same delta so
// actual_position becomes ~0 without introducing a step/impulse (see
// control.c). No-op if the motor has never reported a valid position
// (vesc_can_feedback(motor_index)->position_feedback_valid == 0) -- returns
// PARAM_ERR_CONFIG in that case.
param_result_t control_zero_position(uint8_t motor_index);

// Read-only accessor for one motor's 0x70xx (MOTOR-scope) register storage.
const control_status_t *control_status(uint8_t motor_index);

// Read-only accessor for the SYSTEM-scope part of 0x70xx.
const system_status_t *control_system_status(void);

// One high-rate command packet (binproto.c's CONTROL_COMMAND), applied
// atomically to one motor -- see control_apply_command(). Field
// types/ranges mirror the corresponding params.h registers exactly
// (control_source_t, override_source_t, motor_control_t's target_*/*_ff).
// motor_index is carried alongside this in the wire message (binproto.h),
// not inside this struct -- it's passed as a separate parameter to
// control_apply_command() instead, matching every other per-motor
// control.h entry point.
typedef struct
{
    uint8_t control_source;   // control_source_t
    uint8_t profile_enable;   // 0/1
    uint8_t override_source;  // override_source_t (NONE/CURRENT/VELOCITY only)
    uint8_t override_enable;  // 0/1
    float target_position;    // rad
    float target_velocity;    // rad/s
    float target_current;     // A
    float velocity_ff;        // rad/s
    float current_ff;         // A
} control_command_t;

// Applies an entire control_command_t to one motor in one shot -- NOT a
// sequence of individual params_write() calls. That distinction matters:
// control_step() only ever runs between main-loop iterations (this
// codebase is single-threaded/cooperative, no preemption), so as long as
// every field in motor_control_t is written within one function call with
// no intervening control_step(), the update is already atomic from
// control_step()'s point of view -- but five separate params_write() calls
// from a byte-at-a-time binary parser are NOT atomic against, say, a
// partially-received/aborted packet leaving some fields updated and others
// stale.
//
// Validates the same way the individual PARAM_CONTROL_SOURCE/
// PARAM_OVERRIDE_SOURCE/PARAM_PROFILE_ENABLE/PARAM_OVERRIDE_ENABLE/
// PARAM_TARGET_POSITION writes in params.c would, with the same
// reject-vs-clamp policy per field (position out of [position_min,
// position_max] -- see params.c's PARAM_TARGET_POSITION comment -- and
// override_source == POSITION are hard rejects; target_velocity/
// target_current are clamped, not rejected). On any hard-reject condition,
// returns PARAM_ERR_RANGE and applies NOTHING from the packet (all or
// nothing, not a partial apply).
//
// On success, also updates this motor's own command-freshness tracking
// (motor_control_t::last_command_ms = now_ms, command_seen = true) -- see
// params.h's "Two-tier host watchdog" comment. Does NOT feed the
// system-wide host_timeout_ms watchdog (control_notify_host_rx()) itself
// -- callers (binproto.c's handle_control_command()) call that separately,
// same as before.
param_result_t control_apply_command(uint8_t motor_index, const control_command_t *cmd, uint32_t now_ms);

#endif // _CONTROL_H
