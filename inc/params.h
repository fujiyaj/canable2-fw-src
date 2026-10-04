#ifndef _PARAMS_H
#define _PARAMS_H

#include <stdbool.h>
#include <stdint.h>

// Register map for the VESC motion-controller firmware built on top of this
// board's FDCAN/USB stack (see can.h / atcan.h). This is a from-scratch
// protocol -- it does not reuse the AT-frame wire format in atcan.h. It is
// deliberately modeled after VESC/RobStride's "parameter index" style, but
// fixes the one thing that made RobStride awkward to use: control mode and
// enable are decoupled here, and mode switching does not require a
// disable/enable cycle (see "control source" below).
//
// Register map version: 0x0002 (v0.2 -- 4-motor addressing; see below.
// v0.1 was single-motor only)
//
// ---------------------------------------------------------------------
// 4-motor addressing
// ---------------------------------------------------------------------
//
// This board drives up to MOTOR_COUNT VESCs on one shared physical CAN bus.
// Rather than quadruplicating every register ID (0x1000 motor0, 0x1100
// motor1, ...), every register keeps its v0.1 numeric ID and is classified
// as one of two scopes (see param_scope_t / param_scope()):
//
//   PARAM_SCOPE_SYSTEM : one instance for the whole board (bus config, the
//                        USB host link watchdog, aggregate/system fault,
//                        persistence triggers, USB diagnostics). Read/write
//                        with motor_index == 0; any other index is rejected
//                        (PARAM_ERR_RANGE) rather than silently ignored, so
//                        a client bug (wrong index) is caught immediately.
//   PARAM_SCOPE_MOTOR  : one instance per motor (control/targets/PID/
//                        profile/most limits/feedback/state/fault/motor
//                        identity). Read/write with motor_index in
//                        [0, MOTOR_COUNT).
//
// params_read()/params_write() take an explicit motor_index parameter and
// validate it against the ID's scope. The ASCII shell mirrors this as an
// optional leading index token ("get 2 actual_position", vs "get
// can_bitrate" with no index -- see shell.c). The binary protocol (v2,
// binproto.h) carries motor_index as an explicit field in every per-motor
// message.
//
// ---------------------------------------------------------------------
// Cascade model (per motor -- each of the MOTOR_COUNT motors runs its own
// independent instance of everything below)
// ---------------------------------------------------------------------
//
//   POSITION -> VELOCITY -> CURRENT -> VESC
//
// `control_source` selects which stage of the cascade receives its target
// directly from the host, instead of from the stage above it:
//
//   CONTROL_SOURCE_CURRENT  : target_current fed straight to the current
//                             limiter / VESC.
//   CONTROL_SOURCE_VELOCITY : target_velocity fed to the velocity PI, whose
//                             output becomes the current command.
//   CONTROL_SOURCE_POSITION : target_position (optionally passed through the
//                             trapezoidal profile generator first, see
//                             `profile_enable`) fed to the position
//                             controller, whose output becomes the velocity
//                             target for the velocity PI.
//
// `override_source` / `override_enable` let the host punch a temporary,
// higher-priority command into the cascade without changing
// `control_source` or touching enable. Priority, highest first:
//
//   override (CURRENT or VELOCITY) > control_source
//
// override_source only ever takes CURRENT or VELOCITY -- there is no
// "override with POSITION" because POSITION is the top of the cascade
// already; overriding *into* it doesn't mean anything. To go back to normal
// position control, just clear override_enable.
//
// Overrides and control_source reuse the same target registers
// (target_current / target_velocity / target_position) -- there is no
// separate set of "override target" registers.
//
// On any transition (control_source change, override_enable edge, or
// profile_enable edge) the firmware performs a bumpless handoff, and this
// is *not* a target ramp. Two separate things happen:
//
//   reference sync : the newly engaged stage's internal reference (and, for
//                     PI stages, its integrator) is seeded so its own output
//                     starts at (near) zero error -- see control_state_t /
//                     begin_transition() in control.c.
//   output blend   : the final current command is blended from the old
//                     path's last output to the new path's live output over
//                     `transition_blend_ms`, i.e.
//                       I_cmd = lerp(I_old, I_new(t), t / transition_blend_ms)
//                     where I_new(t) is recomputed every cycle from the
//                     (now-synced) new path, not a fixed target.
//
// This is why the register is named transition_blend_ms and not
// transition_ramp_ms: it blends two live controller outputs, it does not
// ramp a setpoint. Setpoint shaping for POSITION moves is a separate,
// orthogonal concern owned by `profile_enable` / profile_vel_max / acc /
// dec. See control.c (control_step()) for the actual state machine -- this
// header only defines the register map and config storage.
//
// ---------------------------------------------------------------------
// Two-tier host watchdog (system-wide link liveness vs per-motor command
// freshness)
// ---------------------------------------------------------------------
//
// bus_config_t's host_timeout_ms/host_timeout_action is deliberately
// system-wide, not per-motor: it answers "is the USB/ROS2 link itself still
// alive at all", fed by HEARTBEAT (or any recognized ASCII line -- see
// control_notify_host_rx()). But gating ONLY on that would let one motor's
// steady CONTROL_COMMAND stream keep the global watchdog fed forever while
// a different motor's command stream has silently stopped -- the host is
// demonstrably alive, but motor[1..3] could be replaying a stale target
// indefinitely. So each motor_control_t additionally tracks its own
// last_command_ms/command_seen, fed only by CONTROL_COMMAND(motor_index)
// for that specific motor, and checked against the *same*
// host_timeout_ms/host_timeout_action pair (see control.c) -- there is no
// separate per-motor timeout register; both checks just measure a
// different "last seen" timestamp against one shared threshold+policy.
//
// ---------------------------------------------------------------------
// Register groups
// ---------------------------------------------------------------------
//
//   0x00xx  System / identity                    SYSTEM  (RO)
//   0x10xx  Control source arbitration            MOTOR   (RAM)
//   0x20xx  Targets                               MOTOR   (RAM)
//   0x30xx  Position PID                          MOTOR   (RAM, persisted on `save`)
//   0x31xx  Velocity PID                          MOTOR   (RAM, persisted on `save`)
//   0x40xx  Motion profile (trapezoidal)          MOTOR   (RAM, persisted on `save`)
//   0x50xx  Limits / safety                       MOTOR*  (RAM, persisted on `save`)
//           * except host_timeout_ms/action, which are SYSTEM -- see above
//   0x60xx  Feedback (actual values)               MOTOR   (RO, owned by vesc_can.c)
//   0x70xx  State / fault / effective              MOTOR*  (RO except 0x7010 clear_fault; owned by control.c)
//           * except control_overrun_count/dt_max_us (the shared 1kHz
//             dispatcher's own stats, not any one motor's) and the new
//             0x7030/0x7031 system_fault_active/latched, which are SYSTEM
//   0x80xx  Motor / CAN config                     MOTOR*  (RAM, persisted on `save`)
//           * except can_bitrate (one physical bus), which is SYSTEM
//   0x90xx  Persistence triggers                   SYSTEM  (WO -- saves/resets bus config + all MOTOR_COUNT motors atomically)
//   0xA0xx  USB CDC TX diagnostics                 SYSTEM  (RO, owned by usbd_cdc_if.c)
//
// "persisted on `save`" fields are only written to flash when PARAM_SAVE
// (0x9000) is triggered; until then changes are RAM-only, so gain tuning
// doesn't wear the flash.
//
// ---------------------------------------------------------------------
// Storage ownership (who this header's structs belong to)
// ---------------------------------------------------------------------
//
// params.h only defines the register IDs and the host-writable config
// structs (bus_config_t / motor_config_t / motor_control_t), which params.c
// owns the storage for (one bus_config_t, MOTOR_COUNT of each of the other
// two). The two *read-only* per-motor structs behind the 0x60xx/0x70xx
// groups are deliberately NOT defined here:
//
//   0x60xx (actual_*, vesc_last_rx_ms, vesc_online) is motor_feedback_t,
//     defined in vesc_can.h and owned by vesc_can.c, one per motor -- it's
//     a property of whichever motor driver is plugged in (VESC today), not
//     of the register map itself. params.c's params_read() calls
//     vesc_can_feedback(motor_index) to serve these IDs.
//
//   0x70xx (state, fault_active/latched, active_source, profile_active,
//     effective_*) is control_status_t, defined in control.h and owned by
//     control.c, one per motor (plus one system_status_t singleton for
//     0x7030/0x7031/0x7023/0x7024) -- it's the control state machine's own
//     output. params.c's params_read() calls control_status(motor_index) /
//     control_system_status() to serve these IDs.
//
// This keeps params.c a thin, motor-agnostic read/write/validate layer:
// swapping VESC for RobStride/C610 later only touches vesc_can.h's
// replacement, not params.c or control.c.

#define MOTOR_COUNT 4

typedef enum
{
    // ---- 0x00 System / identity (SYSTEM, RO) ----
    PARAM_DEVICE_ID          = 0x0000, // U32
    PARAM_FW_VERSION         = 0x0001, // U16
    PARAM_PROTOCOL_VERSION   = 0x0002, // U16
    PARAM_MOTOR_COUNT        = 0x0003, // U8, compile-time MOTOR_COUNT

    // ---- 0x10 Control source arbitration (MOTOR, RAM) ----
    PARAM_ENABLE             = 0x1000, // U8   0/1, master output enable
    PARAM_CONTROL_SOURCE     = 0x1001, // U8   control_source_t
    PARAM_PROFILE_ENABLE     = 0x1002, // U8   0/1, only meaningful when control_source==POSITION
    PARAM_OVERRIDE_SOURCE    = 0x1003, // U8   override_source_t
    PARAM_OVERRIDE_ENABLE    = 0x1004, // U8   0/1
    PARAM_TRANSITION_BLEND_MS= 0x1005, // U16  ms, output-blend time (NOT a setpoint ramp -- see header comment)

    // ---- 0x20 Targets (MOTOR, RAM) ----
    PARAM_TARGET_POSITION    = 0x2000, // F32  rad
    PARAM_TARGET_VELOCITY    = 0x2001, // F32  rad/s  (also used as override target)
    PARAM_TARGET_CURRENT     = 0x2002, // F32  A      (also used as override target)
    PARAM_CURRENT_FF         = 0x2003, // F32  A,     added at the current stage
    PARAM_VELOCITY_FF        = 0x2004, // F32  rad/s, added at the velocity stage

    // ---- 0x30 Position PID (MOTOR, RAM, persisted) ----
    PARAM_POS_KP             = 0x3000, // F32, default 0
    PARAM_POS_KI             = 0x3001, // F32, default 0
    PARAM_POS_KD             = 0x3002, // F32, default 0
    PARAM_POS_INTEGRAL_LIMIT = 0x3003, // F32, default 0 (0 = integral disabled)

    // ---- 0x31 Velocity PID (MOTOR, RAM, persisted) ----
    PARAM_VEL_KP             = 0x3100, // F32, default 0
    PARAM_VEL_KI             = 0x3101, // F32, default 0
    PARAM_VEL_KD             = 0x3102, // F32, default 0
    PARAM_VEL_INTEGRAL_LIMIT = 0x3103, // F32, default 0 (0 = integral disabled)

    // ---- 0x40 Motion profile / trapezoidal PP (MOTOR, RAM, persisted) ----
    PARAM_PROFILE_VEL_MAX    = 0x4000, // F32  rad/s,  default 10.0
    PARAM_PROFILE_ACC_MAX    = 0x4001, // F32  rad/s^2, default 30.0
    PARAM_PROFILE_DEC_MAX    = 0x4002, // F32  rad/s^2, default 30.0
    // 0x4003 profile_jerk_max reserved for a future S-curve generator.

    // ---- 0x50 Limits / safety (MOTOR except host_timeout_*, RAM, persisted) ----
    PARAM_CURRENT_LIMIT      = 0x5000, // F32  A, default 0 (0 = unconfigured, enable refused)  MOTOR
    PARAM_VELOCITY_LIMIT     = 0x5001, // F32  rad/s, default 0 (0 = disabled)                  MOTOR
    PARAM_POSITION_MIN       = 0x5002, // F32  rad, default -1e6 (effectively disabled)         MOTOR
    PARAM_POSITION_MAX       = 0x5003, // F32  rad, default +1e6 (effectively disabled)         MOTOR
    PARAM_CAN_TIMEOUT_MS     = 0x5004, // U16  ms, default 100 -- this motor's own VESC feedback watchdog, always fatal (-> FAULT). MOTOR
    PARAM_HOST_TIMEOUT_MS    = 0x5005, // U16  ms, default 0 (0 = disabled). SYSTEM -- see "Two-tier host watchdog" above
    PARAM_HOST_TIMEOUT_ACTION= 0x5006, // U8   host_timeout_action_t, default HOLD. SYSTEM
    PARAM_MAX_TEMPERATURE    = 0x5007, // F32  degC, default 80.0                               MOTOR
    PARAM_MIN_BUS_VOLTAGE    = 0x5008, // F32  V, default 0 (0 = disabled)                      MOTOR
    PARAM_MAX_BUS_VOLTAGE    = 0x5009, // F32  V, default 0 (0 = disabled)                      MOTOR

    // ---- 0x60 Feedback / actual (MOTOR, RO, motor_feedback_t -- see vesc_can.h) ----
    PARAM_ACTUAL_POSITION    = 0x6000, // F32  rad
    PARAM_ACTUAL_VELOCITY    = 0x6001, // F32  rad/s
    PARAM_ACTUAL_CURRENT     = 0x6002, // F32  A
    PARAM_ACTUAL_DUTY        = 0x6003, // F32  -1..1
    PARAM_ACTUAL_VOLTAGE     = 0x6004, // F32  V
    PARAM_ACTUAL_TEMPERATURE = 0x6005, // F32  degC
    PARAM_VESC_LAST_RX_MS    = 0x6006, // U32  ms since boot
    // Feedback validity, split by what each control source actually needs
    // -- deliberately NOT a single "vesc_online" bool. STATUS and STATUS_4
    // can run at different CAN rates on the VESC, so a slow STATUS_4 rate
    // must not fault a running VELOCITY loop, and a stale temperature
    // reading (STATUS_4-only) must not block CURRENT/VELOCITY control. See
    // control.c's control_request_enable() and update_faults()
    // (FAULT_BIT_FEEDBACK_INVALID) for how these gate things.
    //
    // vesc_alive is diagnostic only ("has this VESC said anything
    // recently") and does NOT by itself gate any control source -- CURRENT
    // and VELOCITY both require PARAM_VELOCITY_FEEDBACK_VALID, because
    // actual_current (the over-current fault input) and actual_velocity
    // both come only from STATUS; POSITION additionally needs STATUS_4
    // (see below).
    PARAM_VESC_ONLINE            = 0x6007, // U8 bool -- "vesc_alive": ANY status frame seen recently. Diagnostic only, see above.
    PARAM_VELOCITY_FEEDBACK_VALID= 0x6008, // U8 bool -- STATUS fresh. Required for CONTROL_SOURCE_CURRENT and CONTROL_SOURCE_VELOCITY.
    PARAM_POSITION_FEEDBACK_VALID= 0x6009, // U8 bool -- STATUS_4 fresh AND STATUS fresh (the pid_pos unwrap uses the latest erpm) AND
                                            // unwrap baseline established AND the last unwrap correction was self-consistent (see
                                            // vesc_can.c's unwrap_pid_pos() residual check). Required for CONTROL_SOURCE_POSITION.
    PARAM_TEMPERATURE_VALID      = 0x600A, // U8 bool -- STATUS_4 fresh. Required whenever PARAM_MAX_TEMPERATURE > 0 (temp protection armed but no fresh reading -> FAULT_BIT_FEEDBACK_INVALID, not silently skipped).
    PARAM_VOLTAGE_VALID          = 0x600B, // U8 bool -- STATUS_5 fresh. Required whenever PARAM_MIN_BUS_VOLTAGE or PARAM_MAX_BUS_VOLTAGE > 0, same reasoning.

    // Diagnostic: how many times vesc_can_set_current()'s per-motor
    // single-slot "latest command wins" mailbox got overwritten before the
    // previous value ever reached the bus (see vesc_can.c /
    // vesc_can_tx_service()). NOT a CAN hardware transmit failure/drop
    // count -- deliberately named "overwrite", not "drop", to avoid that
    // misreading: this counts values that were superseded by coalescing
    // (working as designed under load), never a frame the hardware
    // actually tried and failed to send. Should read 0 in normal
    // operation; a nonzero, growing count means something (bus load,
    // bus-off recovery, a wedged FDCAN) is starving CAN TX badly enough
    // that control_step() is outrunning it for this motor.
    PARAM_CAN_TX_CURRENT_OVERWRITE_COUNT = 0x600C, // U32

    // ---- 0x70 State / fault / effective (MOTOR except where noted; control_status_t -- see control.h) ----
    PARAM_STATE              = 0x7000, // U8   controller_state_t
    PARAM_FAULT_ACTIVE       = 0x7001, // U32  bitfield, currently-true fault conditions CAUSED BY THIS MOTOR ONLY --
                                        // never contaminated by another motor's fault (see PARAM_SYSTEM_FAULT_ACTIVE below)
    PARAM_FAULT_LATCHED      = 0x7002, // U32  bitfield, sticky until clear_fault -- this motor's own causes only
    PARAM_ACTIVE_SOURCE      = 0x7003, // U8   control_source_t, actually-driving stage (reflects override)
    PARAM_PROFILE_ACTIVE     = 0x7004, // U8   bool, trajectory generator still interpolating

    PARAM_CLEAR_FAULT        = 0x7010, // WO trigger -- clears THIS motor's fault_latched (and fault_active if cause is gone).
                                        // See PARAM_SYSTEM_FAULT_LATCHED for how a system-level interlock clears.

    // Effective (post-sync/post-profile/post-blend) values, as opposed to
    // the raw host-requested 0x2000-0x2002 registers. Read these to see
    // what the cascade is actually chasing right now -- e.g. during a
    // profile-shaped move, target_position is the final destination while
    // effective_position_ref is where the trapezoidal generator currently
    // is; during a source transition, effective_current_cmd is the
    // blended output actually being sent to the VESC.
    PARAM_EFFECTIVE_POSITION_REF = 0x7020, // F32  rad
    PARAM_EFFECTIVE_VELOCITY_REF = 0x7021, // F32  rad/s
    PARAM_EFFECTIVE_CURRENT_CMD  = 0x7022, // F32  A

    // Diagnostic: main()'s 1kHz tick dispatcher -- see control_notify_overrun()
    // and control_step()'s comment in control.h. SYSTEM scope: one shared
    // dispatcher drives all MOTOR_COUNT motors' control_step() calls each
    // tick, this isn't any single motor's own statistic. Expected in normal
    // operation: overrun_count == 0, dt_max_us in the ~1000-1100us range
    // (a little over 1000 from ordinary jitter). A growing overrun_count or
    // a dt_max_us far past 1000 means something in the main loop (CAN
    // housekeeping, a future USB command parse) is taking too long.
    PARAM_CONTROL_OVERRUN_COUNT  = 0x7023, // U32  SYSTEM
    PARAM_CONTROL_DT_MAX_US      = 0x7024, // U32  SYSTEM

    // Aggregate/system-level fault view (SYSTEM scope; see control.c's
    // "system fault escalation" -- any one motor's runtime-fatal fault
    // forces ALL motors to output 0 and refuses re-enable on all of them,
    // without setting that fault's bit on any motor that wasn't itself the
    // cause). Bit i (1<<i) here means "motor i's own fault_active/latched
    // is currently the (or a) cause of the system-wide interlock" -- read
    // motor i's own PARAM_FAULT_ACTIVE/LATCHED to see exactly what tripped.
    // A motor whose own fault bits are all 0 but which is nonetheless
    // sitting in STATE_FAULT is an "innocent bystander": check this
    // register's nonzero bits (for OTHER motor indices) to see who caused
    // it. No dedicated clear trigger: system_fault_latched is always
    // derived as OR(motor[i].fault_latched-has-a-fatal-bit for all i) --
    // clearing every causing motor's own fault_latched (PARAM_CLEAR_FAULT
    // on each) clears this automatically. See control.c's
    // control_recompute_system_fault().
    PARAM_SYSTEM_FAULT_ACTIVE    = 0x7030, // U32  SYSTEM
    PARAM_SYSTEM_FAULT_LATCHED   = 0x7031, // U32  SYSTEM

    // ---- 0x80 Motor / CAN config (MOTOR except can_bitrate, RAM, persisted) ----
    PARAM_VESC_CAN_ID        = 0x8000, // U8, default 0xFF (unconfigured). Write refused while state==RUNNING. MOTOR
                                        // Duplicate IDs across motors are ACCEPTED on write (so e.g. swapping two
                                        // motors' IDs doesn't get blocked mid-transition) but every affected
                                        // motor's enable is refused (PARAM_ERR_CONFIG) while any duplicate exists
                                        // -- see control_request_enable()'s duplicate-ID check. 0xFF (unconfigured)
                                        // never counts as a duplicate against another 0xFF.
    PARAM_CAN_BITRATE        = 0x8001, // U8, enum can_bitrate (can.h), default CAN_BITRATE_1000K. SYSTEM -- one
                                        // physical bus shared by all motors. Write refused while ANY motor state==RUNNING.
    PARAM_GEAR_RATIO         = 0x8002, // F32, motor_rotation / output_rotation, default 1.0. Write refused while state==RUNNING. MOTOR
    PARAM_MOTOR_DIRECTION    = 0x8003, // I8, +1 or -1, default +1. Write refused while state==RUNNING. MOTOR
    PARAM_POSITION_OFFSET    = 0x8004, // F32  rad (output axis), default 0.0 -- see PARAM_ZERO_POSITION. MOTOR
    PARAM_POLE_PAIRS         = 0x8005, // U8, default 7 (M2006/M3508 are both 7). Write refused while state==RUNNING. MOTOR
                                        // Needed to convert VESC's electrical-domain feedback (STATUS erpm,
                                        // STATUS_4 pid_pos) to mechanical motor revolutions -- see vesc_can.c.

    // How many motor mechanical revolutions STATUS_4's pid_pos (0-360deg)
    // spans before wrapping. This MUST match the VESC's own p_pid_ang_div
    // setting (configured out-of-band via VESC Tool -- this firmware
    // cannot read or write it over CAN), via:
    //   p_pid_ang_div (on the VESC) = pole_pairs * angle_span_motor_rev
    // A mismatch here silently corrupts vesc_can.c's ERPM-assisted unwrap
    // -- there is currently no automatic cross-check. Default 6 (with
    // pole_pairs=7 that's p_pid_ang_div=42) -- chosen for a 4-VESC/1Mbps
    // bus budget where STATUS_4 only needs to run at ~250Hz (not 500Hz):
    // at M2006's ~18000rpm/36:1 worst case, angle_div=42 wraps at 50Hz, so
    // 250Hz sampling still sees only ~72deg/sample (vs. ~108deg/sample for
    // the previous angle_div=28 default at the same rate) -- see the CAN
    // bus-load design discussion for the full 4-VESC frame-rate budget
    // this pairs with (STATUS1+STATUS4 @ 250Hz, STATUS5 @ 20Hz,
    // SET_CURRENT @ 500Hz/motor, ~50-60% bus load at 1Mbps). MOTOR
    PARAM_ANGLE_SPAN_MOTOR_REV = 0x8006, // U8, default 6. Write refused while state==RUNNING. MOTOR

    // Coarse motor-identity selector. Currently just storage ("the box") --
    // does NOT yet lock/derive pole_pairs/gear_ratio/angle_span_motor_rev
    // or current limits, and does NOT yet cross-check against VESC-side
    // identity (temperature sensor presence, Information Storage). That's
    // future work; see the design discussion. motor_profile_t values below. MOTOR
    PARAM_MOTOR_PROFILE      = 0x8007, // U8 motor_profile_t, default MOTOR_PROFILE_UNSET. Write refused while state==RUNNING.

    // Atomically re-zeroes this motor's output-axis coordinate system:
    // shifts position_offset so actual_position becomes ~0, and shifts
    // target_position and the position stage's internal reference by the
    // same delta so no step/impulse is introduced (see control_zero_position()
    // in control.c). WO trigger; value written is ignored. MOTOR
    PARAM_ZERO_POSITION      = 0x8010, // WO trigger

    // ---- 0x90 Persistence (SYSTEM, WO triggers) -- bus_config + all MOTOR_COUNT motor_config in one flash write ----
    PARAM_SAVE               = 0x9000, // WO -- write bus_config_t + motor_config_t[MOTOR_COUNT] to flash
    PARAM_LOAD_DEFAULTS      = 0x9001, // WO -- reset RAM (not flash) to factory defaults, all motors + bus config
    PARAM_FACTORY_RESET      = 0x9002, // WO -- erase flash config, reload defaults, save

    // ---- 0xA0 USB CDC TX diagnostics (SYSTEM, RO, owned by usbd_cdc_if.c) ----
    // Added to chase down intermittent multi-command USB CDC hangs seen
    // during ASCII shell bring-up: single commands always succeed, but
    // tight command sequences would occasionally stop producing any host
    // -side output while the firmware kept processing internally (state
    // changes from later single-command checks proved it wasn't wedged).
    // Root cause (fixed): cdc_process_tx() advanced txbuf.tail BEFORE
    // confirming USBD_CDC_TransmitPacket() actually accepted the packet --
    // see usbd_cdc_if.c. Verified via 10000-iteration ASCII and mixed-message
    // binary stress tests (0 timeouts/drops/CRC errors) post-fix.
    PARAM_CDC_TX_ENQUEUE_COUNT   = 0xA000, // U32  successful cdc_transmit() calls (data admitted to txbuf)
    PARAM_CDC_TX_DROP_COUNT      = 0xA001, // U32  cdc_transmit() calls refused because txbuf was full
    PARAM_CDC_TX_BUSY_COUNT      = 0xA002, // U32  cdc_process_tx() calls that found hcdc->TxState != 0 (host/USB core not ready for more)
    PARAM_CDC_TX_BYTES_PENDING   = 0xA003, // U32  bytes currently queued in txbuf, not yet handed to USBD_CDC_TransmitPacket()
    PARAM_CDC_TX_HEAD            = 0xA004, // U32  txbuf.head (live ring buffer index)
    PARAM_CDC_TX_TAIL            = 0xA005, // U32  txbuf.tail (live ring buffer index)
} reg_id_t;


// Whether a register is a single board-wide instance (motor_index must be
// 0) or one-per-motor (motor_index must be < MOTOR_COUNT). See the
// "4-motor addressing" header comment. Implemented in params.c as an
// explicit list of the (deliberately few) SYSTEM-scope IDs; everything
// else defaults to MOTOR scope.
typedef enum
{
    PARAM_SCOPE_SYSTEM,
    PARAM_SCOPE_MOTOR,
} param_scope_t;

param_scope_t param_scope(reg_id_t id);


typedef enum
{
    CONTROL_SOURCE_NONE     = 0, // no target stage engaged (equivalent to output 0 when enabled)
    CONTROL_SOURCE_CURRENT  = 1,
    CONTROL_SOURCE_VELOCITY = 2,
    CONTROL_SOURCE_POSITION = 3,
} control_source_t;

// override_source uses the same numbering as control_source, but only
// NONE/CURRENT/VELOCITY are valid values -- POSITION is rejected.
typedef control_source_t override_source_t;

// Coarse motor identity (PARAM_MOTOR_PROFILE). v0.1: storage only -- see
// that register's comment. UNSET is not CUSTOM: UNSET means "nobody has
// asserted what's plugged in yet", CUSTOM means "deliberately hand-tuned,
// not M2006/M3508". Neither currently changes firmware behavior.
typedef enum
{
    MOTOR_PROFILE_UNSET  = 0,
    MOTOR_PROFILE_M2006  = 1,
    MOTOR_PROFILE_M3508  = 2,
    MOTOR_PROFILE_CUSTOM = 3,
} motor_profile_t;

typedef enum
{
    HOST_TIMEOUT_HOLD         = 0, // keep last accepted target (position hold friendly)
    HOST_TIMEOUT_CURRENT_ZERO = 1, // drop to target_current = 0 but stay enabled
    HOST_TIMEOUT_FAULT        = 2, // -> FAULT, current = 0
    HOST_TIMEOUT_DISABLE      = 3, // -> enable=0, state=IDLE, current = 0. Unlike
                                    // CURRENT_ZERO, this clears `enable` itself (not
                                    // just the live output), so a comms blip can never
                                    // silently self-heal into motion the instant
                                    // CONTROL_COMMAND resumes -- PARAM_ENABLE=1 is
                                    // required again afterward. Self-sustaining: once
                                    // set, control_step()'s `!pr->enable` branch keeps
                                    // reporting IDLE/NONE/0 on every subsequent tick
                                    // even after host_timeout_active itself clears.
} host_timeout_action_t;

typedef enum
{
    STATE_BOOT    = 0,
    STATE_IDLE    = 1,
    STATE_RUNNING = 2,
    STATE_FAULT   = 3,
} controller_state_t;

// fault_active / fault_latched bit positions (per-motor -- see
// motor_status_t in control.h; PARAM_SYSTEM_FAULT_ACTIVE/LATCHED are a
// separate, derived, per-motor-indexed bitmask, not built from these bits)
#define FAULT_BIT_CAN_TIMEOUT     (1u << 0) // no VESC status frame within can_timeout_ms
#define FAULT_BIT_COMMAND_TIMEOUT (1u << 1) // reserved (superseded by host_timeout_action, kept for bit compat)
#define FAULT_BIT_OVER_CURRENT    (1u << 2)
#define FAULT_BIT_OVER_SPEED      (1u << 3)
#define FAULT_BIT_POS_LIMIT       (1u << 4)
#define FAULT_BIT_VESC_FAULT      (1u << 5) // VESC reported its own fault code over CAN
#define FAULT_BIT_OVER_TEMP       (1u << 6)
#define FAULT_BIT_UNDER_VOLTAGE   (1u << 7)
#define FAULT_BIT_OVER_VOLTAGE   (1u << 8)
// Enable-time reject ONLY -- set exclusively by control_request_enable()
// when a PARAM_ENABLE=1 write is refused (current_limit == 0, vesc_can_id
// unset, VESC never seen online, a duplicate vesc_can_id against another
// motor, or a required host watchdog isn't configured/proven live yet --
// see control_require_host_watchdog()). Latched for visibility but NEVER
// forces RUNNING -> FAULT: state stays IDLE, because the enable that would
// have caused a problem never happened. If you're looking at whether
// something already RUNNING can set this bit, the answer is no -- that's
// FAULT_BIT_RUNTIME_CONFIG below.
#define FAULT_BIT_CONFIG_ERROR   (1u << 9)

// active control_source's required feedback (see PARAM_VELOCITY_FEEDBACK_VALID
// / PARAM_POSITION_FEEDBACK_VALID) isn't valid right now. Runtime-fatal --
// see control.c's update_faults() -- only evaluated while pr->enable != 0.
#define FAULT_BIT_FEEDBACK_INVALID (1u << 10)

// A safety invariant that control_request_enable() verified at enable time
// (currently: a required host watchdog is actually configured -- see
// control_require_host_watchdog()) turned out to be false WHILE RUNNING.
// This should be unreachable in normal operation (the write path that
// could break the invariant is supposed to refuse to -- see
// PARAM_HOST_TIMEOUT_MS's write handler in params.c) -- this bit exists as
// the runtime-checked backstop for that guard, not as a routine fault.
// Deliberately a separate bit from FAULT_BIT_CONFIG_ERROR: that one is
// enable-reject-only and never forces RUNNING -> FAULT (see its comment);
// this one is the opposite -- it only ever fires from within RUNNING.
#define FAULT_BIT_RUNTIME_CONFIG (1u << 11)

// This motor's own fault_active/latched picked up a bit from the set above
// (CAN_TIMEOUT, OVER_CURRENT, OVER_SPEED, POS_LIMIT, VESC_FAULT, OVER_TEMP,
// UNDER/OVER_VOLTAGE, FEEDBACK_INVALID, RUNTIME_CONFIG, or
// COMMAND_TIMEOUT-as-FAULT) is what "runtime-fatal" means for the
// system-fault escalation described at PARAM_SYSTEM_FAULT_ACTIVE -- see
// control.c's control_recompute_system_fault(). There is deliberately no
// separate "SYSTEM_FAULT_BIT_*" namespace: PARAM_SYSTEM_FAULT_ACTIVE's bit
// i is just "motor i's own fault_active is nonzero", not a new kind of
// fault condition.


// ---------------------------------------------------------------------
// Storage
// ---------------------------------------------------------------------

// Bus-wide config: one instance for the whole board. Persisted (see
// motor_config_t below for why gains/limits are RAM-until-`save`).
typedef struct
{
    uint8_t  can_bitrate;         // enum can_bitrate (can.h) -- one physical bus for all motors
    uint16_t host_timeout_ms;     // USB/ROS2 link watchdog (HEARTBEAT-driven), 0 = disabled -- see header comment
    uint8_t  host_timeout_action; // host_timeout_action_t
} bus_config_t;

// Per-motor flash-backed config: only touched on PARAM_SAVE. Gains and
// limits live here so tuning over serial doesn't wear the flash -- changes
// are RAM-only (this struct, in RAM) until explicitly saved. One instance
// per motor (motor_config[MOTOR_COUNT] in params.c).
typedef struct
{
    // Position PID
    float pos_kp;
    float pos_ki;
    float pos_kd;
    float pos_integral_limit;

    // Velocity PID
    float vel_kp;
    float vel_ki;
    float vel_kd;
    float vel_integral_limit;

    // Motion profile
    float profile_vel_max;
    float profile_acc_max;
    float profile_dec_max;

    // Limits / safety (host_timeout_* live in bus_config_t instead -- see
    // the "Two-tier host watchdog" header comment)
    float current_limit;
    float velocity_limit;
    float position_min;
    float position_max;
    uint16_t can_timeout_ms;
    float max_temperature;
    float min_bus_voltage;
    float max_bus_voltage;

    // Motor / CAN identity (can_bitrate lives in bus_config_t instead)
    uint8_t vesc_can_id;
    float gear_ratio;
    int8_t motor_direction;
    float position_offset;
    uint8_t pole_pairs;
    uint8_t angle_span_motor_rev;
    uint8_t motor_profile; // motor_profile_t
} motor_config_t;

// RAM-only per-motor command/arbitration state. Never persisted; always
// boots to enable=0 / control_source=NONE regardless of what the host was
// doing before the last reset. One instance per motor (motor_control[MOTOR_COUNT]
// in params.c).
typedef struct
{
    uint8_t enable;
    uint8_t control_source;    // control_source_t
    uint8_t profile_enable;
    uint8_t override_source;   // override_source_t
    uint8_t override_enable;
    uint16_t transition_blend_ms;

    float target_position;
    float target_velocity;
    float target_current;
    float current_ff;
    float velocity_ff;

    // Per-motor CONTROL_COMMAND freshness -- fed only by
    // CONTROL_COMMAND(motor_index) for this specific motor, checked against
    // bus_config_t's shared host_timeout_ms/host_timeout_action. See the
    // "Two-tier host watchdog" header comment and control.c.
    uint32_t last_command_ms;
    uint8_t  command_seen; // false until the first CONTROL_COMMAND for this motor ever arrives
} motor_control_t;

// NOTE: there is no feedback_t here. 0x60xx is served by vesc_can.h's
// motor_feedback_t (vesc_can_feedback(motor_index)); 0x70xx is served by
// control.h's control_status_t / system_status_t (control_status(motor_index) /
// control_system_status()). See the "Storage ownership" comment near the
// top of this file.


// ---------------------------------------------------------------------
// Factory defaults
// ---------------------------------------------------------------------
// All gains default to 0 so that enabling a freshly-flashed board never
// applies unexpected torque -- the host must explicitly tune and `save`
// before POSITION/VELOCITY sources do anything. Every motor gets the same
// defaults (motor_config[i] = {...these macros...} for all i); vesc_can_id
// defaulting to 0xFF (unconfigured) on all MOTOR_COUNT motors simultaneously
// is fine -- 0xFF never counts as a duplicate (see PARAM_VESC_CAN_ID).

#define DEFAULT_POS_KP              0.0f
#define DEFAULT_POS_KI              0.0f
#define DEFAULT_POS_KD              0.0f
#define DEFAULT_POS_INTEGRAL_LIMIT  0.0f

#define DEFAULT_VEL_KP              0.0f
#define DEFAULT_VEL_KI              0.0f
#define DEFAULT_VEL_KD              0.0f
#define DEFAULT_VEL_INTEGRAL_LIMIT  0.0f

#define DEFAULT_PROFILE_VEL_MAX     10.0f
#define DEFAULT_PROFILE_ACC_MAX     30.0f
#define DEFAULT_PROFILE_DEC_MAX     30.0f

#define DEFAULT_CURRENT_LIMIT       0.0f   // 0 = unconfigured; enable is refused until set
#define DEFAULT_VELOCITY_LIMIT      0.0f   // 0 = disabled
#define DEFAULT_POSITION_MIN        (-1.0e6f)
#define DEFAULT_POSITION_MAX        (1.0e6f)
#define DEFAULT_CAN_TIMEOUT_MS      100
#define DEFAULT_MAX_TEMPERATURE     80.0f
#define DEFAULT_MIN_BUS_VOLTAGE     0.0f   // 0 = disabled
#define DEFAULT_MAX_BUS_VOLTAGE     0.0f   // 0 = disabled

#define DEFAULT_VESC_CAN_ID         0xFF   // 0xFF = unconfigured
#define DEFAULT_GEAR_RATIO          1.0f
#define DEFAULT_MOTOR_DIRECTION     1
#define DEFAULT_POSITION_OFFSET     0.0f
#define DEFAULT_POLE_PAIRS          7
#define DEFAULT_ANGLE_SPAN_MOTOR_REV 6
#define DEFAULT_MOTOR_PROFILE       MOTOR_PROFILE_UNSET

#define DEFAULT_TRANSITION_BLEND_MS 100

// Bus-wide defaults (bus_config_t -- one instance, not per-motor)
//
// 1Mbps -- matches the VESC's bus speed (bench-confirmed back to 1Mbps
// after earlier instability; see the 4-VESC bus-load design discussion for
// the Rate1/Rate2 + SET_CURRENT frame budget this targets). Since
// can.c's can_set_bitrate() refuses to change bitrate once the bus is
// already ON, a `set can_bitrate` + `save` doesn't take effect until the
// next boot (vesc_can_init() only reads it at startup) -- either reflash
// this default, or write+save+power-cycle. The G431 and every VESC on the
// bus must always be changed together, or none of them can decode each
// other's frames at all (this is exactly what happened switching one VESC
// to 1Mbps while this was still 500Kbps: total silence, not a partial/
// degraded link).
#define BUS_DEFAULT_CAN_BITRATE         CAN_BITRATE_1000K
// 0 (disabled) by default -- ASCII bring-up needs to `set target_current`
// and `enable` without the motor going quiet ~200ms later for no visible
// reason. Binary/ROS2 sessions don't rely on this default: binary-mode
// enable refuses to arm at all while host_timeout_ms == 0 (see
// control_require_host_watchdog()), so a ROS2 node is expected to
// explicitly `WRITE_PARAM host_timeout_ms` (200ms is a reasonable start)
// and `host_timeout_action` (CURRENT_ZERO is a reasonable start; HOLD may
// be safer for a gravity-loaded axis) itself right after switching to
// binary, before enabling -- not inherit a value tuned for the other mode.
#define BUS_DEFAULT_HOST_TIMEOUT_MS     0
#define BUS_DEFAULT_HOST_TIMEOUT_ACTION HOST_TIMEOUT_CURRENT_ZERO


// ---------------------------------------------------------------------
// params_write() result codes
// ---------------------------------------------------------------------
//
// Validation happens in up to three stages, checked in this order:
//   1. PARAM_ERR_TYPE / PARAM_ERR_UNKNOWN_ID / PARAM_ERR_READ_ONLY /
//      PARAM_ERR_WRITE_ONLY / PARAM_ERR_RANGE (motor_index out of range for
//      the ID's scope -- see param_scope()) -- structural: wrong size,
//      unknown register, wrong direction, wrong addressing.
//   2. PARAM_ERR_RANGE -- the value itself is invalid regardless of state
//      (e.g. gear_ratio <= 0, motor_direction not +-1, override_source ==
//      POSITION, current_limit < 0).
//   3. PARAM_ERR_STATE -- value would be fine, but not while state ==
//      RUNNING (e.g. changing vesc_can_id / can_bitrate / gear_ratio /
//      motor_direction out from under a running control loop).
// PARAM_ERR_CONFIG is distinct from all of the above: it is only ever
// returned by the PARAM_ENABLE write path (control_request_enable()), when
// the config is structurally fine but incomplete for arming the output
// stage (current_limit == 0, vesc_can_id unset, VESC never seen online,
// vesc_can_id duplicated against another motor).
//
// PARAM_ERR_IO is returned by PARAM_SAVE/PARAM_FACTORY_RESET specifically,
// and only if the underlying flash_store_save() (see flash_store.h) itself
// fails -- a real HAL_FLASH erase/program error, not a policy refusal (that's
// still PARAM_ERR_STATE, checked first, before flash is ever touched).
// PARAM_LOAD_DEFAULTS never returns this -- it's genuinely RAM-only.
typedef enum
{
    PARAM_OK = 0,
    PARAM_ERR_UNKNOWN_ID,
    PARAM_ERR_READ_ONLY,
    PARAM_ERR_WRITE_ONLY,
    PARAM_ERR_TYPE,
    PARAM_ERR_RANGE,
    PARAM_ERR_STATE,
    PARAM_ERR_CONFIG,
    PARAM_ERR_NOT_IMPLEMENTED, // no longer returned by anything as of flash_store.c landing; kept (not
                                // renumbered) so old binary-protocol clients that hardcoded its value stay correct
    PARAM_ERR_IO,              // PARAM_SAVE/PARAM_FACTORY_RESET: the flash_store_save() write itself failed
                                // (HAL_FLASH erase/program error) -- rare, but distinct from "refused by policy"
} param_result_t;


// ---------------------------------------------------------------------
// Prototypes (implemented in params.c)
// ---------------------------------------------------------------------

// Config-only init: tries flash_store_load() first (see flash_store.h) and
// falls back to compiled-in defaults (params_load_defaults()) if no valid
// image is found there (blank flash, corrupt image, or a version stamp
// from an older/differently-shaped layout). Every motor_control_t is
// always zeroed (enable=0, control_source=NONE) regardless -- RAM-only
// state never persists across a reset by design. Does NOT touch
// motor_feedback_t or control_status_t -- those are vesc_can_init()'s and
// control_init()'s jobs respectively; call params_init() first, then
// control_init(), then vesc_can_init().
void params_init(void);
void params_load_defaults(void);        // bus_config_t + motor_config_t[MOTOR_COUNT] = factory defaults (RAM only)

// Writes the current RAM bus_config/motor_config to flash (flash_store_save())
// -- see PARAM_SAVE's write handler in params.c for the "no motor RUNNING"
// gating. Returns true on success; false only on an actual HAL_FLASH error
// (see PARAM_ERR_IO).
bool params_save(void);

// Erases the flash image, reloads compiled-in defaults into RAM, then
// immediately re-saves those defaults to flash -- so a factory-reset
// device boots with a known-valid flash image, not one left erased until
// the next explicit SAVE. Returns true on success.
bool params_factory_reset(void);

// Generic register access for the USB command layer. motor_index is
// validated against param_scope(id) -- see the "4-motor addressing" header
// comment; pass 0 for SYSTEM-scope IDs.
param_result_t params_read(reg_id_t id, uint8_t motor_index, void *out, uint32_t out_size);
param_result_t params_write(reg_id_t id, uint8_t motor_index, const void *in, uint32_t in_size);

bus_config_t    *params_bus(void);
motor_config_t  *params_motor(uint8_t motor_index);     // flash-backed, persisted on save
motor_control_t *params_motor_ram(uint8_t motor_index); // RAM-only

#endif // _PARAMS_H
