//
// control: 1kHz cascade control loop (POSITION -> VELOCITY -> CURRENT ->
// VESC), one independent instance per motor (MOTOR_COUNT of them), with
// source arbitration, bumpless source switching, and fault handling -- plus
// system-level state shared across all motors (aggregate fault interlock,
// the 1kHz dispatcher's own stats). See inc/params.h for the register map,
// inc/control.h for control_status_t/system_status_t (this module's own RO
// output), and inc/vesc_can.h for motor_feedback_t (the motor driver's RO
// output -- NOT owned here).
//
// ---------------------------------------------------------------------
// Design recap (see the project's design discussion for the full derivation)
// ---------------------------------------------------------------------
//
// Cascade (independently, per motor):
//
//   POSITION -> VELOCITY -> CURRENT -> VESC
//
// `control_source` (params.h) picks which stage receives its target
// directly from the host; `override_source`/`override_enable` let CURRENT
// or VELOCITY punch in on top of that without touching control_source.
// resolve_active_source() folds both into a single "requested" source each
// cycle.
//
// Switching sources (or toggling profile_enable) never snaps the output.
// Two independent mechanisms handle it:
//
//   reference sync : begin_transition() seeds the *newly engaged* stage's
//                     internal reference at the current measurement (error
//                     = 0 at the instant of the switch), and seeds its PI
//                     integrator so its own output already equals whatever
//                     "no-op" should mean for that stage (actual_velocity
//                     for the position stage's output, output_current for
//                     the velocity stage's output). Stages that stay
//                     engaged across the switch (e.g. the velocity stage
//                     when going VELOCITY -> POSITION) are left untouched.
//
//   output blend   : the final current command is lerp'd from the old
//                     path's last output to the new path's live output
//                     over transition_blend_ms:
//                       I_cmd = lerp(I_old, I_new(t), alpha), alpha: 0->1
//                     I_new(t) is recomputed every cycle from the
//                     (already-synced) new path -- it is not a fixed
//                     target, so this is not a setpoint ramp.
//
// FAULT recovery is treated as a full re-entry, not "resume from where we
// left off": while state == FAULT (or enable == 0), active_source and
// last_requested_source are forced to CONTROL_SOURCE_NONE every cycle.
// This makes the normal key_changed check in control_step() fire
// begin_transition(NONE, requested) automatically the next time the host
// re-enables -- i.e. recovery always re-syncs from the current physical
// state, exactly like a fresh enable. No special-case code needed.
//
// PI integrators are stored in the *output units* of their own stage
// (velocity stage's integral is a current in amps; position stage's
// integral is a velocity in rad/s). That is what makes the seeding formula
// a one-liner: seed = desired_output - kp * error. If ki == 0 or
// integral_limit == 0 for a stage, its integral is force-held at 0 instead
// of being seeded -- bumplessness for that stage then relies entirely on
// the output blend, not on a hidden persistent feedforward term.
//
// Faults are grouped into three buckets with different handling, PER
// MOTOR:
//
//   config reject  (FAULT_BIT_CONFIG_ERROR): checked only when the host
//                   writes PARAM_ENABLE=1 for that motor
//                   (control_request_enable()). Never forces RUNNING ->
//                   FAULT. Latched for visibility, but state stays IDLE.
//   runtime fatal  (CAN_TIMEOUT, VESC_FAULT, OVER_TEMP, OVER_SPEED,
//                   POS_LIMIT, OVER_CURRENT, UNDER/OVER_VOLTAGE,
//                   FEEDBACK_INVALID, RUNTIME_CONFIG, and COMMAND_TIMEOUT
//                   when host_timeout_action == FAULT): computed by
//                   update_faults(), but ONLY while that motor's enable !=
//                   0 -- an IDLE motor (nothing enabled yet, e.g. right
//                   after boot before its VESC has sent a single STATUS
//                   frame) must never be pushed into FAULT by conditions
//                   that control_request_enable() would have refused to
//                   arm against in the first place. Any bit set on a
//                   motor's OWN fault_active forces that motor's
//                   RUNNING -> FAULT AND escalates to a system-wide
//                   interlock (see "System fault escalation" below) --
//                   VESCs still get 0A commands at the (rate-gated) full
//                   rate, not just once.
//   host policy    (host_timeout_ms elapsed, action == HOLD or
//                   CURRENT_ZERO): not a fault at all -- no bit is set, no
//                   state change. HOLD substitutes a safe effective target
//                   per source (position: unchanged; velocity/current: 0)
//                   and keeps running the normal cascade. CURRENT_ZERO
//                   bypasses the cascade like a fault (output 0) but
//                   without latching or leaving RUNNING, and resumes the
//                   instant the host is heard from again. See params.h's
//                   "Two-tier host watchdog" -- this now fires from EITHER
//                   the system-wide HEARTBEAT-driven watchdog OR this
//                   motor's own CONTROL_COMMAND-driven freshness check,
//                   whichever is stale, using the same host_timeout_ms/
//                   host_timeout_action pair.
//
// ---------------------------------------------------------------------
// System fault escalation
// ---------------------------------------------------------------------
// Any one motor's own runtime-fatal fault_active forces ALL motors to
// output 0 and refuses re-enable on ALL of them -- but WITHOUT setting
// that fault's bit (or any bit) on a motor that wasn't itself the cause:
// update_faults() only ever writes a motor's OWN fault_active/latched from
// that motor's OWN conditions. The interlock is a separate, derived
// system_status_t::system_fault_active/latched bitmask (bit i = "motor i's
// own fault is/was the cause"), recomputed each tick by
// control_recompute_system_fault() from every motor's current
// fault_active/latched (level-triggered: stays escalated for as long as
// any causing motor's own fault persists, not just on the edge it first
// appeared). control_step() checks
// `motor_status.fault_active != 0 || system_status.system_fault_active != 0`
// to decide whether THIS motor is interlocked -- an "innocent bystander"
// motor shows state==STATE_FAULT with its own fault_active/latched still
// 0x00000000, distinguishable from the actual cause by checking
// PARAM_SYSTEM_FAULT_ACTIVE's bits against each OTHER motor's own
// fault_active. There is no separate system-level clear trigger:
// PARAM_CLEAR_FAULT on each causing motor clears that motor's own latch,
// and system_fault_latched is always re-derived from whatever remains.
//
// Because control_step() is called once per motor per tick (see main.c)
// and control_recompute_system_fault() runs once after all MOTOR_COUNT
// calls, the interlock a motor observes reflects the aggregate as of the
// END of the PREVIOUS tick, not instantaneously within the same tick --
// bounded to <=1ms/1 tick propagation latency at 1kHz, which is far faster
// than anything mechanical or a human could react to. control_clear_fault()
// additionally calls control_recompute_system_fault() immediately (not
// tick-gated) so a manual clear takes effect without waiting for the next
// tick, and checks the freshly-recomputed system_fault_active (not just
// this motor's own fault_active) before allowing FAULT -> IDLE, so a motor
// whose own cause just cleared but which is still interlocked by another
// motor's ongoing fault doesn't even momentarily report IDLE.
//
// Ownership: this file owns motor_config_t/motor_control_t only through
// the params_motor()/params_motor_ram() accessors -- params.c still holds
// the actual storage. It owns control_status_t[MOTOR_COUNT]/system_status_t
// (`status[]`/`sys_status`, below) outright. It only *reads* motor_feedback_t
// through vesc_can_feedback(motor_index) -- it never writes through it,
// even vesc_alive; that's vesc_can_poll()'s job. It does NOT call
// vesc_can_set_current() anymore -- that's main.c's job now (rate-gated
// publish step, see control_step()'s comment in control.h and main.c),
// reading control_status(motor_index)->effective_current_cmd after each
// control_step() call.
//

#include <math.h>
#include <stdbool.h>

#include "params.h"
#include "control.h"
#include "vesc_can.h"

#define CONTROL_PERIOD_MS 1

// Over-current fault trip margin / debounce. v0.1: fixed constants, not
// register-configurable.
#define OVER_CURRENT_TRIP_RATIO 1.25f
#define OVER_CURRENT_DEBOUNCE_MS 20

static inline float clampf(float v, float lo, float hi)
{
    if (hi < lo) return v; // limit pair disabled/inverted -> no clamp
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline float lerpf(float a, float b, float alpha)
{
    return a + (b - a) * alpha;
}

// ---------------------------------------------------------------------
// Internal state (NOT persisted, NOT directly host-visible -- host sees
// derived values through control_status(motor_index) / PARAM_ACTIVE_SOURCE
// etc.). One instance per motor.
// ---------------------------------------------------------------------

typedef struct
{
    control_source_t active_source;     // holds old value until alpha >= 1
    control_source_t previous_source;   // diagnostic only
    control_source_t last_requested_source;
    uint8_t last_profile_enable;

    bool  transitioning;
    float transition_alpha;             // 0..1
    float transition_start_current;     // output_current at switch time
    float velocity_ref_sync_start;      // actual_velocity at switch time, for velocity_ref lerp

    // Independent of transitioning/transition_alpha: true from the moment
    // the position stage is newly engaged until position_ref actually
    // reaches its target. profile_vel_max/acc_max govern how long that
    // takes, which is normally much longer than transition_blend_ms --
    // gating this off `transitioning` instead would let position_ref jump
    // to the target the instant the (short) output blend finishes, even
    // though the physical move hasn't caught up yet.
    bool pos_ref_catching_up;

    float position_ref;                 // rad
    float velocity_ref;                 // rad/s
    float output_current;               // A, the continuous quantity everything anchors to

    float pos_integral;                 // rad/s (position stage's own output units)
    float pos_prev_error;
    float vel_integral;                 // A (velocity stage's own output units)
    float vel_prev_error;

    // Two independent freshness checks feeding the SAME host_timeout_ms/
    // host_timeout_action policy -- see params.h's "Two-tier host
    // watchdog". host_timeout_active is this motor's own OR of both:
    // system-wide HEARTBEAT liveness, and this motor's own CONTROL_COMMAND
    // freshness (only checked once this motor has ever received one --
    // see motor_control_t::command_seen).
    bool host_timeout_active;
    float over_current_debounce_ms;
} control_state_t;

static control_state_t ctrl[MOTOR_COUNT];
static control_status_t status[MOTOR_COUNT];
static system_status_t sys_status;
static uint32_t host_last_rx_ms;
static bool host_watchdog_required;          // see control_require_host_watchdog()
static bool host_rx_seen_since_armed;         // see control_require_host_watchdog()'s "prove you're alive" comment

static control_source_t resolve_active_source(const motor_control_t *pr); // defined below; needed by control_request_enable()

void control_init(void)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        ctrl[i] = (control_state_t){0};
        ctrl[i].active_source = CONTROL_SOURCE_NONE;
        ctrl[i].last_requested_source = CONTROL_SOURCE_NONE;

        status[i] = (control_status_t){0};
        status[i].state = STATE_IDLE;
        status[i].active_source = CONTROL_SOURCE_NONE;
    }

    sys_status = (system_status_t){0};
    host_last_rx_ms = 0;
}

void control_notify_host_rx(uint32_t now_ms)
{
    host_last_rx_ms = now_ms;
    host_rx_seen_since_armed = true;
}

void control_notify_overrun(uint32_t missed_ticks)
{
    sys_status.overrun_count += missed_ticks;
}

void control_require_host_watchdog(bool required)
{
    host_watchdog_required = required;
    if (required)
        host_rx_seen_since_armed = false; // must prove liveness again -- see header comment
}

bool control_host_watchdog_required(void)
{
    return host_watchdog_required;
}

param_result_t control_request_enable(uint8_t motor_index)
{
    motor_config_t *pf = params_motor(motor_index);
    motor_control_t *pr = params_motor_ram(motor_index);
    const motor_feedback_t *mf = vesc_can_feedback(motor_index);
    const bus_config_t *bus = params_bus();

    if (status[motor_index].state == STATE_FAULT)
        return PARAM_ERR_STATE; // must clear_fault first

    // Duplicate vesc_can_id against another motor -- see params.h's
    // PARAM_VESC_CAN_ID comment. 0xFF (unconfigured) never counts.
    bool duplicate_id = false;
    if (pf->vesc_can_id != 0xFF) {
        for (uint8_t j = 0; j < MOTOR_COUNT; j++) {
            if (j != motor_index && params_motor(j)->vesc_can_id == pf->vesc_can_id) {
                duplicate_id = true;
                break;
            }
        }
    }

    if (pf->current_limit <= 0.0f ||
        pf->vesc_can_id == 0xFF ||
        !mf->vesc_alive ||
        duplicate_id ||
        (host_watchdog_required && (bus->host_timeout_ms == 0 || !host_rx_seen_since_armed)))
    {
        status[motor_index].fault_latched |= FAULT_BIT_CONFIG_ERROR;
        return PARAM_ERR_CONFIG;
    }

    // The currently-configured source (before enable takes effect) must
    // already have the feedback it needs. vesc_alive (checked above) is
    // NOT sufficient for CURRENT -- actual_current, which the over-current
    // fault check reads, comes only from STATUS, so CURRENT needs
    // velocity_feedback_valid too (name is about STATUS's contents, not
    // "you must be in VELOCITY mode"). If the host changes control_source
    // to something that needs more while already RUNNING, update_faults()'s
    // FAULT_BIT_FEEDBACK_INVALID check below catches that continuously;
    // this is just the enable-time gate so we don't even start in a
    // broken state.
    control_source_t requested = resolve_active_source(pr);
    bool feedback_ok = true;
    switch (requested) {
    case CONTROL_SOURCE_CURRENT:  feedback_ok = mf->velocity_feedback_valid; break;
    case CONTROL_SOURCE_VELOCITY: feedback_ok = mf->velocity_feedback_valid; break;
    case CONTROL_SOURCE_POSITION: feedback_ok = mf->position_feedback_valid; break;
    default: break;
    }
    if (!feedback_ok) {
        status[motor_index].fault_latched |= FAULT_BIT_CONFIG_ERROR;
        return PARAM_ERR_CONFIG;
    }

    pr->enable = 1;
    return PARAM_OK;
}

void control_request_disable(uint8_t motor_index)
{
    params_motor_ram(motor_index)->enable = 0;
}

void control_recompute_system_fault(void)
{
    uint32_t active = 0, latched = 0;
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if (status[i].fault_active)
            active |= (1u << i);
        if (status[i].fault_latched)
            latched |= (1u << i);
    }
    sys_status.system_fault_active = active;
    sys_status.system_fault_latched = latched;
}

void control_clear_fault(uint8_t motor_index)
{
    status[motor_index].fault_latched = status[motor_index].fault_active; // drops CONFIG_ERROR and any resolved bit
    control_recompute_system_fault(); // immediate, not tick-gated -- see file header comment
    if (status[motor_index].fault_active == 0 && sys_status.system_fault_active == 0 &&
        status[motor_index].state == STATE_FAULT)
        status[motor_index].state = STATE_IDLE;
    // active_source/last_requested_source are already CONTROL_SOURCE_NONE
    // (forced every cycle while state == FAULT) -- the next enable will
    // naturally trigger a full begin_transition(NONE, requested) re-entry.
}

param_result_t control_zero_position(uint8_t motor_index)
{
    const motor_feedback_t *mf = vesc_can_feedback(motor_index);
    if (!mf->position_feedback_valid)
        return PARAM_ERR_CONFIG; // no valid position to zero against

    motor_config_t *pf = params_motor(motor_index);
    motor_control_t *pr = params_motor_ram(motor_index);

    // Shift the output-axis coordinate system so actual_position becomes
    // ~0, without introducing a step: position_offset absorbs the shift,
    // and every absolute reference expressed in the old frame
    // (target_position, and the position stage's own internal ref) is
    // shifted by the same delta so it still names the same physical point.
    float delta = mf->actual_position;
    pf->position_offset += delta;
    pr->target_position -= delta;
    ctrl[motor_index].position_ref -= delta;
    return PARAM_OK;
}

const control_status_t *control_status(uint8_t motor_index)
{
    return &status[motor_index];
}

const system_status_t *control_system_status(void)
{
    return &sys_status;
}

param_result_t control_apply_command(uint8_t motor_index, const control_command_t *cmd, uint32_t now_ms)
{
    motor_config_t *pf = params_motor(motor_index);
    motor_control_t *pr = params_motor_ram(motor_index);

    // Hard-reject checks first, before writing anything -- see this
    // function's header comment for why partial application is not
    // acceptable. Mirrors params.c's individual write handlers for these
    // same fields exactly (PARAM_CONTROL_SOURCE/PARAM_OVERRIDE_SOURCE/
    // PARAM_PROFILE_ENABLE/PARAM_OVERRIDE_ENABLE/PARAM_TARGET_POSITION).
    if (cmd->control_source > CONTROL_SOURCE_POSITION)
        return PARAM_ERR_RANGE;
    if (cmd->override_source > CONTROL_SOURCE_VELOCITY) // excludes POSITION -- see params.h
        return PARAM_ERR_RANGE;
    if (cmd->profile_enable > 1 || cmd->override_enable > 1)
        return PARAM_ERR_RANGE;
    if (cmd->target_position < pf->position_min || cmd->target_position > pf->position_max)
        return PARAM_ERR_RANGE;

    // Clamp-not-reject fields, same policy as the individual
    // PARAM_TARGET_VELOCITY/PARAM_TARGET_CURRENT writes.
    float velocity = cmd->target_velocity;
    if (pf->velocity_limit > 0.0f)
        velocity = clampf(velocity, -pf->velocity_limit, pf->velocity_limit);

    float current = (pf->current_limit > 0.0f)
        ? clampf(cmd->target_current, -pf->current_limit, pf->current_limit)
        : 0.0f;

    // All validated -- apply in one shot. No control_step() call can occur
    // between these writes (single-threaded/cooperative main loop), so
    // this is already atomic from the control loop's point of view.
    pr->control_source = cmd->control_source;
    pr->profile_enable = cmd->profile_enable;
    pr->override_source = cmd->override_source;
    pr->override_enable = cmd->override_enable;
    pr->target_position = cmd->target_position;
    pr->target_velocity = velocity;
    pr->target_current = current;
    pr->velocity_ff = cmd->velocity_ff;
    pr->current_ff = cmd->current_ff;

    // This motor's own command-freshness tracking -- see params.h's
    // "Two-tier host watchdog" comment. Deliberately does NOT call
    // control_notify_host_rx() (the system-wide watchdog) -- callers
    // handle that separately, same as before this function existed.
    pr->last_command_ms = now_ms;
    pr->command_seen = 1;

    return PARAM_OK;
}


// ---------------------------------------------------------------------
// resolve_active_source
// ---------------------------------------------------------------------

static control_source_t resolve_active_source(const motor_control_t *pr)
{
    if (pr->override_enable && pr->override_source != CONTROL_SOURCE_NONE) {
        if (pr->override_source == CONTROL_SOURCE_POSITION)
            return (control_source_t)pr->control_source; // defensive; params_write should reject this write
        return (control_source_t)pr->override_source;
    }
    return (control_source_t)pr->control_source;
}


// ---------------------------------------------------------------------
// begin_transition
// ---------------------------------------------------------------------

static void begin_transition(control_state_t *c, const motor_config_t *pf, const motor_feedback_t *mf,
                              control_source_t old_src, control_source_t new_src)
{
    c->previous_source = old_src;
    c->transitioning = true;
    c->transition_alpha = 0.0f;
    c->transition_start_current = c->output_current;

    bool vel_was_engaged = (old_src == CONTROL_SOURCE_VELOCITY || old_src == CONTROL_SOURCE_POSITION);
    bool vel_now_engaged = (new_src == CONTROL_SOURCE_VELOCITY || new_src == CONTROL_SOURCE_POSITION);
    bool pos_was_engaged = (old_src == CONTROL_SOURCE_POSITION);
    bool pos_now_engaged = (new_src == CONTROL_SOURCE_POSITION);

    if (vel_now_engaged && !vel_was_engaged) {
        c->velocity_ref = mf->actual_velocity;
        c->velocity_ref_sync_start = mf->actual_velocity;
        c->vel_prev_error = 0.0f;
        if (pf->vel_ki > 0.0f && pf->vel_integral_limit > 0.0f) {
            float seed = c->output_current; // error == 0, so kp term drops out
            c->vel_integral = clampf(seed, -pf->vel_integral_limit, pf->vel_integral_limit);
        } else {
            c->vel_integral = 0.0f;
        }
    }

    if (pos_now_engaged && !pos_was_engaged) {
        c->position_ref = mf->actual_position;
        c->pos_prev_error = 0.0f;
        if (pf->pos_ki > 0.0f && pf->pos_integral_limit > 0.0f) {
            float seed = mf->actual_velocity; // error == 0
            c->pos_integral = clampf(seed, -pf->pos_integral_limit, pf->pos_integral_limit);
        } else {
            c->pos_integral = 0.0f;
        }
        c->pos_ref_catching_up = true;
    }

    // CURRENT has no internal state -- nothing to sync when it's the newly
    // engaged source; the output blend alone makes it bumpless.
}


// ---------------------------------------------------------------------
// update_internal_reference
// ---------------------------------------------------------------------

// TODO(before real POSITION testing): profile_acc_max (acc_max here) is not
// implemented yet -- step_vel jumps straight to vel_max (or the
// decel-limited speed near the target) with no ramp-up, so this is only a
// decel-limited trapezoid, not a true trapezoidal profile. Low-current/
// no-load bring-up (verifying CAN/angle decode) doesn't need this, but
// anything with real inertia will feel the missing accel limit as a step
// in commanded velocity at move start. Needs a persistent `profile_velocity`
// state (ramped by +-acc_max/dec_max per step, clamped to vel_max) instead
// of recomputing step_vel from scratch every call.
static float trapezoidal_step(float ref, float target, float vel_max, float acc_max, float dec_max, float dt)
{
    float err = target - ref;
    if (err == 0.0f)
        return ref;

    float dir = (err > 0.0f) ? 1.0f : -1.0f;
    float dist = fabsf(err);

    // Distance needed to decelerate to 0 from vel_max at dec_max.
    float stop_dist = (vel_max * vel_max) / (2.0f * dec_max);
    float step_vel = (dist > stop_dist) ? vel_max : sqrtf(2.0f * dec_max * dist);
    (void)acc_max; // see TODO above

    float step = dir * step_vel * dt;
    if (fabsf(step) >= dist)
        return target;
    return ref + step;
}

static void update_internal_reference(control_state_t *c, const motor_control_t *pr, const motor_config_t *pf,
                                       control_status_t *st, float dt, float eff_target_velocity,
                                       control_source_t requested)
{
    // position ref: shaped while (a) profile_enable is on in steady state,
    // or (b) the position stage is still catching up from a reference sync
    // (see control_state_t::pos_ref_catching_up -- this runs on its own
    // clock, governed by profile_vel_max/acc/dec, independent of the much
    // shorter transition_blend_ms output blend).
    bool pos_shaping = pr->profile_enable || c->pos_ref_catching_up;

    float pos_target = clampf(pr->target_position, pf->position_min, pf->position_max);

    if (pos_shaping) {
        c->position_ref = trapezoidal_step(c->position_ref, pos_target,
                                            pf->profile_vel_max, pf->profile_acc_max, pf->profile_dec_max, dt);
        bool settled = (c->position_ref == pos_target);
        st->profile_active = !settled;
        if (settled)
            c->pos_ref_catching_up = false;
    } else {
        c->position_ref = pos_target;
        st->profile_active = 0;
    }

    // velocity ref: only meaningful when VELOCITY is the entry point
    // (under POSITION, compute_source_output() overwrites it every cycle
    // with the position stage's output instead).
    if (requested == CONTROL_SOURCE_VELOCITY) {
        if (c->transitioning)
            c->velocity_ref = lerpf(c->velocity_ref_sync_start, eff_target_velocity, c->transition_alpha);
        else
            c->velocity_ref = eff_target_velocity;
    }
}


// ---------------------------------------------------------------------
// compute_source_output
// ---------------------------------------------------------------------

static float pi_stage_step(float ref, float actual, float kp, float ki, float kd,
                            float integral_limit, float *integral, float *prev_error, float dt)
{
    float error = ref - actual;
    float d = (dt > 0.0f) ? (error - *prev_error) / dt : 0.0f;
    *prev_error = error;

    if (ki > 0.0f && integral_limit > 0.0f) {
        *integral += ki * error * dt;
        *integral = clampf(*integral, -integral_limit, integral_limit);
    } else {
        *integral = 0.0f; // no hidden persistent feedforward when ki/limit == 0
    }

    return kp * error + *integral + kd * d;
}

static float compute_source_output(control_state_t *c, const motor_control_t *pr, const motor_config_t *pf,
                                    const motor_feedback_t *mf, control_source_t src, float dt,
                                    float eff_target_current)
{
    switch (src) {
    case CONTROL_SOURCE_CURRENT:
        return eff_target_current + pr->current_ff;

    case CONTROL_SOURCE_VELOCITY: {
        float i = pi_stage_step(c->velocity_ref, mf->actual_velocity,
                                 pf->vel_kp, pf->vel_ki, pf->vel_kd, pf->vel_integral_limit,
                                 &c->vel_integral, &c->vel_prev_error, dt);
        return i + pr->current_ff;
    }

    case CONTROL_SOURCE_POSITION: {
        float v_ref = pi_stage_step(c->position_ref, mf->actual_position,
                                     pf->pos_kp, pf->pos_ki, pf->pos_kd, pf->pos_integral_limit,
                                     &c->pos_integral, &c->pos_prev_error, dt);
        v_ref += pr->velocity_ff;
        if (pf->velocity_limit > 0.0f)
            v_ref = clampf(v_ref, -pf->velocity_limit, pf->velocity_limit);
        c->velocity_ref = v_ref; // position stage's output feeds the velocity stage every cycle

        float i = pi_stage_step(c->velocity_ref, mf->actual_velocity,
                                 pf->vel_kp, pf->vel_ki, pf->vel_kd, pf->vel_integral_limit,
                                 &c->vel_integral, &c->vel_prev_error, dt);
        return i + pr->current_ff;
    }

    case CONTROL_SOURCE_NONE:
    default:
        return 0.0f;
    }
}


// ---------------------------------------------------------------------
// update_faults
// ---------------------------------------------------------------------
//
// Runtime-fatal bits are only ever evaluated while this motor's enable !=
// 0 ("armed" below) -- see the file header's "Faults are grouped into
// three buckets" comment for why. Only ever writes THIS motor's OWN
// fault_active/latched -- see the "System fault escalation" file header
// comment for how a per-motor fault becomes a system-wide interlock
// without contaminating another motor's own fault bits.
//
// FAULT_BIT_CONFIG_ERROR is set only by control_request_enable(); never
// touched here (see that bit's comment in params.h) -- it's an
// enable-reject bit, disjoint from this function's runtime-fatal bucket.
// FAULT_BIT_RUNTIME_CONFIG is the runtime-checked counterpart for the one
// enable-time invariant (a required host watchdog actually being
// configured) that could theoretically be violated after the fact -- see
// its comment in params.h.
//
// Never *writes* motor_feedback_t -- CAN_TIMEOUT is derived from
// vesc_can.c's own vesc_alive determination (not re-computed from raw
// timestamps here), and per-source feedback validity
// (FAULT_BIT_FEEDBACK_INVALID) likewise just reads vesc_can.c's
// velocity_feedback_valid/position_feedback_valid. `requested` is passed
// in (from resolve_active_source(), computed once at the top of
// control_step()) so a live control_source switch to a source whose
// feedback isn't ready faults immediately -- not just at enable time (see
// control_request_enable()'s comment).

static void update_faults(uint8_t motor_index, uint32_t now_ms, float dt, control_source_t requested)
{
    const motor_config_t *pf = params_motor(motor_index);
    motor_control_t *pr = params_motor_ram(motor_index);
    const motor_feedback_t *mf = vesc_can_feedback(motor_index);
    const bus_config_t *bus = params_bus();
    control_state_t *c = &ctrl[motor_index];
    control_status_t *st = &status[motor_index];

    uint32_t active = 0;
    const bool armed = (pr->enable != 0);

    // Host/command liveness tracking happens regardless of `armed`: a
    // binary/ROS2 session's HEARTBEAT (or CONTROL_COMMAND) is expected to
    // arrive DURING the arm sequence, before enable -- see
    // control_require_host_watchdog()'s "prove you're alive" comment --
    // so this needs to already be correct by the time enable happens, not
    // start being tracked only after. Two independent checks against the
    // SAME threshold (see params.h's "Two-tier host watchdog"): the
    // system-wide HEARTBEAT age, and (only once this motor has ever
    // received one) this motor's own CONTROL_COMMAND age -- either being
    // stale marks this motor's host_timeout_active.
    c->host_timeout_active = false;
    if (bus->host_timeout_ms > 0) {
        uint32_t host_age = now_ms - host_last_rx_ms;
        if (host_age > bus->host_timeout_ms)
            c->host_timeout_active = true;

        if (pr->command_seen) {
            uint32_t cmd_age = now_ms - pr->last_command_ms;
            if (cmd_age > bus->host_timeout_ms)
                c->host_timeout_active = true;
        }
    }

    if (!armed) {
        c->over_current_debounce_ms = 0.0f; // don't let idle-time noise pre-load the debounce for the next enable
        st->fault_active = 0;
        return; // fault_latched deliberately left untouched -- it's sticky history, not a live reading
    }

    if (!mf->vesc_alive)
        active |= FAULT_BIT_CAN_TIMEOUT;

    // Defense in depth: params.c's PARAM_HOST_TIMEOUT_MS write handler
    // already refuses to set this to 0 while any motor is RUNNING and a
    // watchdog is required (see control_host_watchdog_required()), so this
    // should be unreachable -- but don't rely on that guard alone for
    // something this safety-critical.
    if (host_watchdog_required && bus->host_timeout_ms == 0)
        active |= FAULT_BIT_RUNTIME_CONFIG;

    bool feedback_ok = true;
    switch (requested) {
    case CONTROL_SOURCE_CURRENT:  feedback_ok = mf->velocity_feedback_valid; break; // actual_current comes only from STATUS
    case CONTROL_SOURCE_VELOCITY: feedback_ok = mf->velocity_feedback_valid; break;
    case CONTROL_SOURCE_POSITION: feedback_ok = mf->position_feedback_valid; break;
    default: break;
    }
    // A safety limit being armed (>0) but its input feedback never having
    // arrived must not silently read as "no fault" just because the
    // underlying actual_* defaults to 0 -- that would let e.g. CURRENT run
    // indefinitely with temperature protection armed but no temperature
    // ever received. Fold these into the same feedback_ok gate the active
    // source's own requirement uses above.
    if (pf->max_temperature > 0.0f && !mf->temperature_valid)
        feedback_ok = false;
    if ((pf->min_bus_voltage > 0.0f || pf->max_bus_voltage > 0.0f) && !mf->voltage_valid)
        feedback_ok = false;
    if (!feedback_ok)
        active |= FAULT_BIT_FEEDBACK_INVALID;

    // Gated on *_valid so a stale-but-remembered reading (message stopped
    // arriving after a real trip) doesn't stay silently exonerated forever,
    // and so a stale-but-remembered *low* reading doesn't mask a real trip
    // happening after the message stopped -- either way, an invalid
    // reading is already covered by FAULT_BIT_FEEDBACK_INVALID above, not
    // by pretending the old number is still current.
    if (pf->max_temperature > 0.0f && mf->temperature_valid && mf->actual_temperature > pf->max_temperature)
        active |= FAULT_BIT_OVER_TEMP;

    if (pf->min_bus_voltage > 0.0f && mf->voltage_valid && mf->actual_voltage < pf->min_bus_voltage)
        active |= FAULT_BIT_UNDER_VOLTAGE;
    if (pf->max_bus_voltage > 0.0f && mf->voltage_valid && mf->actual_voltage > pf->max_bus_voltage)
        active |= FAULT_BIT_OVER_VOLTAGE;

    if (pf->velocity_limit > 0.0f && fabsf(mf->actual_velocity) > pf->velocity_limit)
        active |= FAULT_BIT_OVER_SPEED;

    if (mf->actual_position < pf->position_min || mf->actual_position > pf->position_max)
        active |= FAULT_BIT_POS_LIMIT;

    if (pf->current_limit > 0.0f) {
        if (fabsf(mf->actual_current) > pf->current_limit * OVER_CURRENT_TRIP_RATIO)
            c->over_current_debounce_ms += dt * 1000.0f;
        else
            c->over_current_debounce_ms = 0.0f;

        if (c->over_current_debounce_ms >= OVER_CURRENT_DEBOUNCE_MS)
            active |= FAULT_BIT_OVER_CURRENT;
    } else {
        c->over_current_debounce_ms = 0.0f;
    }

    if (c->host_timeout_active && bus->host_timeout_action == HOST_TIMEOUT_FAULT)
        active |= FAULT_BIT_COMMAND_TIMEOUT;

    st->fault_active = active;
    st->fault_latched |= active;
}


// ---------------------------------------------------------------------
// control_step
// ---------------------------------------------------------------------

void control_step(uint8_t motor_index, float dt, uint32_t now_ms)
{
    const motor_config_t *pf = params_motor(motor_index);
    motor_control_t *pr = params_motor_ram(motor_index);
    const motor_feedback_t *mf = vesc_can_feedback(motor_index);
    const bus_config_t *bus = params_bus();
    control_state_t *c = &ctrl[motor_index];
    control_status_t *st = &status[motor_index];

    uint32_t dt_us = (uint32_t)(dt * 1.0e6f);
    if (dt_us > sys_status.dt_max_us)
        sys_status.dt_max_us = dt_us;

    // Computed once up front (pure function of pr's state) so update_faults()
    // can check the *actually requested* source's feedback validity, not
    // just what was true when the host last called enable.
    control_source_t requested = resolve_active_source(pr);

    update_faults(motor_index, now_ms, dt, requested);

    // System-wide interlock: see the file header's "System fault
    // escalation" comment. sys_status here reflects the aggregate as of
    // the end of the PREVIOUS tick (control_recompute_system_fault() runs
    // once in main.c after every motor's control_step() for THIS tick) --
    // bounded <=1 tick (<=1ms @ 1kHz) propagation latency.
    bool interlocked = (st->fault_active != 0) || (sys_status.system_fault_active != 0);

    if (interlocked) {
        c->active_source = c->last_requested_source = CONTROL_SOURCE_NONE;
        c->transitioning = false;
        st->active_source = CONTROL_SOURCE_NONE;
        st->state = STATE_FAULT;
        c->output_current = 0.0f;
        st->effective_current_cmd = 0.0f;
        return;
    }

    // Gate on THIS MOTOR'S OWN latched runtime-fatal bits -- not
    // st->state == STATE_FAULT -- so that "stuck until acknowledged"
    // applies only to a motor that actually had its own runtime-fatal
    // cause (fault_latched only ever gets set from THIS motor's own
    // conditions, see update_faults()). A pure bystander (interlocked
    // only by another motor's system fault, own fault_latched always
    // 0x0) falls straight through here the instant `interlocked` above
    // goes false -- i.e. it resumes automatically once the causing
    // motor's condition clears, WITHOUT needing its own PARAM_CLEAR_FAULT
    // call, matching "clear_fault on the causing motor alone is enough."
    // Using st->state here instead would incorrectly leave every bystander
    // latched in FAULT forever, since nothing else ever resets state back
    // out of STATE_FAULT for a motor that never got its own clear_fault.
    //
    // FAULT_BIT_CONFIG_ERROR is deliberately EXCLUDED from this check: per
    // its own contract in params.h ("enable-reject ONLY ... NEVER forces
    // RUNNING -> FAULT ... state stays IDLE"), a rejected PARAM_ENABLE=1
    // (e.g. current_limit still 0) latches that bit for visibility but
    // must never, by itself, flip this motor into STATE_FAULT -- doing so
    // would wedge every future enable attempt behind PARAM_ERR_STATE
    // ("must clear_fault first") for a motor that was simply never armed.
    // Masking it out here is what makes that contract hold even with the
    // fault_latched-based gating above.
    if ((st->fault_latched & ~(uint32_t)FAULT_BIT_CONFIG_ERROR) != 0) {
        c->output_current = 0.0f;
        st->state = STATE_FAULT; // still displayed FAULT while awaiting this motor's own clear_fault
        st->effective_current_cmd = 0.0f;
        return;
    }

    if (!pr->enable) {
        c->active_source = c->last_requested_source = CONTROL_SOURCE_NONE;
        c->transitioning = false;
        st->active_source = CONTROL_SOURCE_NONE;
        st->state = STATE_IDLE;
        c->output_current = 0.0f;
        st->effective_current_cmd = 0.0f;
        return;
    }

    st->state = STATE_RUNNING;

    // host/command timeout policy (HOLD substitutes safe effective
    // targets; CURRENT_ZERO bypasses the cascade outright; DISABLE also
    // clears `enable` itself; FAULT already handled above via
    // update_faults()/FAULT_BIT_COMMAND_TIMEOUT).
    float eff_target_velocity = pr->target_velocity;
    float eff_target_current  = pr->target_current;

    if (c->host_timeout_active) {
        if (bus->host_timeout_action == HOST_TIMEOUT_DISABLE) {
            // Unlike CURRENT_ZERO below, this clears `enable` itself, not
            // just the live output -- see host_timeout_action_t's comment:
            // a comms blip must never silently self-heal into motion the
            // instant CONTROL_COMMAND resumes. Self-sustaining from here:
            // next tick's `if (!pr->enable)` branch above takes over and
            // keeps reporting IDLE/NONE/0 even after host_timeout_active
            // itself clears, until an explicit PARAM_ENABLE=1.
            pr->enable = 0;
            c->active_source = c->last_requested_source = CONTROL_SOURCE_NONE;
            c->transitioning = false;
            st->active_source = CONTROL_SOURCE_NONE;
            st->state = STATE_IDLE;
            c->output_current = 0.0f;
            st->effective_current_cmd = 0.0f;
            return;
        }
        if (bus->host_timeout_action == HOST_TIMEOUT_CURRENT_ZERO) {
            c->active_source = c->last_requested_source = CONTROL_SOURCE_NONE;
            c->transitioning = false;
            st->active_source = CONTROL_SOURCE_NONE;
            c->output_current = 0.0f;
            st->effective_current_cmd = 0.0f;
            return;
        }
        // HOST_TIMEOUT_HOLD: velocity/current sources fall back to zero;
        // target_position is left as-is (position hold needs no change).
        eff_target_velocity = 0.0f;
        eff_target_current  = 0.0f;
    }

    bool key_changed = (requested != c->last_requested_source) ||
                        (pr->profile_enable != c->last_profile_enable);
    if (key_changed) {
        begin_transition(c, pf, mf, c->active_source, requested);
        c->last_requested_source = requested;
        c->last_profile_enable   = pr->profile_enable;
    }

    update_internal_reference(c, pr, pf, st, dt, eff_target_velocity, requested);
    float new_output = compute_source_output(c, pr, pf, mf, requested, dt, eff_target_current);

    if (c->transitioning) {
        if (pr->transition_blend_ms == 0) {
            c->transition_alpha = 1.0f; // 0ms == instant switching, debug-friendly
        } else {
            c->transition_alpha += dt / ((float)pr->transition_blend_ms * 0.001f);
        }
        if (c->transition_alpha >= 1.0f) {
            c->transition_alpha = 1.0f;
            c->transitioning = false;
            c->active_source = requested;
        }
        c->output_current = lerpf(c->transition_start_current, new_output, c->transition_alpha);
    } else {
        c->output_current = new_output;
    }

    c->output_current = clampf(c->output_current, -pf->current_limit, pf->current_limit);
    st->active_source = c->active_source;
    st->effective_position_ref = c->position_ref;
    st->effective_velocity_ref = c->velocity_ref;
    st->effective_current_cmd  = c->output_current;

    // NOT vesc_can_set_current() here -- see file header comment and
    // control.h's control_step() comment. main.c reads
    // control_status(motor_index)->effective_current_cmd and publishes it
    // to the per-motor mailbox on its own ~500Hz/motor rate-gated schedule.
}
