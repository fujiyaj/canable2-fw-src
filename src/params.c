//
// params: register storage (bus_config_t / motor_config_t[MOTOR_COUNT] /
// motor_control_t[MOTOR_COUNT]) and the generic params_read()/params_write()
// dispatcher for the USB command layer. Deliberately thin and motor-agnostic
// -- see params.h's "Storage ownership" comment. PARAM_ENABLE,
// PARAM_CLEAR_FAULT and PARAM_ZERO_POSITION are forwarded to control.c
// rather than handled here, so control.c's state machine invariants can't
// be bypassed by a direct register poke.
//

#include <stdbool.h>
#include <string.h>

#include "stm32g4xx_hal.h" // can.h's FDCAN_* types need this pulled in first (matches atcan.c)
#include "can.h" // enum can_bitrate, CAN_BITRATE_INVALID
#include "params.h"
#include "control.h"
#include "vesc_can.h"
#include "usbd_cdc_if.h"
#include "flash_store.h"

static bus_config_t    bus_flash;
static motor_config_t  motor_flash[MOTOR_COUNT];
static motor_control_t motor_ram[MOTOR_COUNT];

bus_config_t    *params_bus(void)                      { return &bus_flash; }
motor_config_t  *params_motor(uint8_t motor_index)     { return &motor_flash[motor_index]; }
motor_control_t *params_motor_ram(uint8_t motor_index) { return &motor_ram[motor_index]; }


// ---------------------------------------------------------------------
// scope classification -- see params.h's "4-motor addressing" comment.
// Deliberately an explicit list of the (few) SYSTEM-scope IDs; everything
// else defaults to MOTOR scope, so adding a new per-motor register needs no
// change here.
// ---------------------------------------------------------------------

param_scope_t param_scope(reg_id_t id)
{
    switch (id) {
    case PARAM_DEVICE_ID:
    case PARAM_FW_VERSION:
    case PARAM_PROTOCOL_VERSION:
    case PARAM_MOTOR_COUNT:
    case PARAM_HOST_TIMEOUT_MS:
    case PARAM_HOST_TIMEOUT_ACTION:
    case PARAM_CONTROL_OVERRUN_COUNT:
    case PARAM_CONTROL_DT_MAX_US:
    case PARAM_SYSTEM_FAULT_ACTIVE:
    case PARAM_SYSTEM_FAULT_LATCHED:
    case PARAM_CAN_BITRATE:
    case PARAM_SAVE:
    case PARAM_LOAD_DEFAULTS:
    case PARAM_FACTORY_RESET:
    case PARAM_CDC_TX_ENQUEUE_COUNT:
    case PARAM_CDC_TX_DROP_COUNT:
    case PARAM_CDC_TX_BUSY_COUNT:
    case PARAM_CDC_TX_BYTES_PENDING:
    case PARAM_CDC_TX_HEAD:
    case PARAM_CDC_TX_TAIL:
        return PARAM_SCOPE_SYSTEM;
    default:
        return PARAM_SCOPE_MOTOR;
    }
}

static bool motor_index_ok(reg_id_t id, uint8_t motor_index)
{
    if (param_scope(id) == PARAM_SCOPE_SYSTEM)
        return motor_index == 0;
    return motor_index < MOTOR_COUNT;
}


// ---------------------------------------------------------------------
// init / defaults / persistence
// ---------------------------------------------------------------------

void params_load_defaults(void)
{
    bus_flash = (bus_config_t){
        .can_bitrate = BUS_DEFAULT_CAN_BITRATE,
        .host_timeout_ms = BUS_DEFAULT_HOST_TIMEOUT_MS,
        .host_timeout_action = BUS_DEFAULT_HOST_TIMEOUT_ACTION,
    };

    const motor_config_t defaults = {
        .pos_kp = DEFAULT_POS_KP,
        .pos_ki = DEFAULT_POS_KI,
        .pos_kd = DEFAULT_POS_KD,
        .pos_integral_limit = DEFAULT_POS_INTEGRAL_LIMIT,

        .vel_kp = DEFAULT_VEL_KP,
        .vel_ki = DEFAULT_VEL_KI,
        .vel_kd = DEFAULT_VEL_KD,
        .vel_integral_limit = DEFAULT_VEL_INTEGRAL_LIMIT,

        .profile_vel_max = DEFAULT_PROFILE_VEL_MAX,
        .profile_acc_max = DEFAULT_PROFILE_ACC_MAX,
        .profile_dec_max = DEFAULT_PROFILE_DEC_MAX,

        .current_limit = DEFAULT_CURRENT_LIMIT,
        .velocity_limit = DEFAULT_VELOCITY_LIMIT,
        .position_min = DEFAULT_POSITION_MIN,
        .position_max = DEFAULT_POSITION_MAX,
        .can_timeout_ms = DEFAULT_CAN_TIMEOUT_MS,
        .max_temperature = DEFAULT_MAX_TEMPERATURE,
        .min_bus_voltage = DEFAULT_MIN_BUS_VOLTAGE,
        .max_bus_voltage = DEFAULT_MAX_BUS_VOLTAGE,

        .vesc_can_id = DEFAULT_VESC_CAN_ID,
        .gear_ratio = DEFAULT_GEAR_RATIO,
        .motor_direction = DEFAULT_MOTOR_DIRECTION,
        .position_offset = DEFAULT_POSITION_OFFSET,
        .pole_pairs = DEFAULT_POLE_PAIRS,
        .angle_span_motor_rev = DEFAULT_ANGLE_SPAN_MOTOR_REV,
        .motor_profile = DEFAULT_MOTOR_PROFILE,
    };

    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
        motor_flash[i] = defaults;
}

static void reset_ram(void)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        motor_ram[i] = (motor_control_t){0}; // enable=0, control_source=NONE(0), targets=0, ff=0, command_seen=0
        motor_ram[i].transition_blend_ms = DEFAULT_TRANSITION_BLEND_MS;
    }
}

void params_init(void)
{
    if (!flash_store_load(&bus_flash, motor_flash))
        params_load_defaults(); // blank/corrupt/wrong-version flash image -- fall back to compiled-in defaults
    reset_ram();
}

bool params_save(void)
{
    return flash_store_save(&bus_flash, motor_flash);
}

bool params_factory_reset(void)
{
    params_load_defaults();
    return params_save();
}


// ---------------------------------------------------------------------
// small helpers
// ---------------------------------------------------------------------

static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline bool is_running(uint8_t motor_index)
{
    return control_status(motor_index)->state == STATE_RUNNING;
}

static bool any_motor_running(void)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
        if (is_running(i))
            return true;
    return false;
}

static param_result_t read_raw(void *out, uint32_t out_size, const void *src, uint32_t src_size)
{
    if (out_size != src_size)
        return PARAM_ERR_TYPE;
    memcpy(out, src, src_size);
    return PARAM_OK;
}

#define READ(type, val) do { type _v = (type)(val); return read_raw(out, out_size, &_v, sizeof(_v)); } while (0)

// Generic bounded-write helpers. All return PARAM_ERR_TYPE on a size
// mismatch, PARAM_ERR_RANGE if the decoded value falls outside [lo, hi].
static param_result_t write_f32(float *dst, const void *in, uint32_t in_size, float lo, float hi)
{
    if (in_size != sizeof(float)) return PARAM_ERR_TYPE;
    float v; memcpy(&v, in, sizeof(v));
    if (v < lo || v > hi) return PARAM_ERR_RANGE;
    *dst = v;
    return PARAM_OK;
}

static param_result_t write_u8(uint8_t *dst, const void *in, uint32_t in_size, uint8_t lo, uint8_t hi)
{
    if (in_size != sizeof(uint8_t)) return PARAM_ERR_TYPE;
    uint8_t v; memcpy(&v, in, sizeof(v));
    if (v < lo || v > hi) return PARAM_ERR_RANGE;
    *dst = v;
    return PARAM_OK;
}

static param_result_t write_u16(uint16_t *dst, const void *in, uint32_t in_size, uint16_t lo, uint16_t hi)
{
    if (in_size != sizeof(uint16_t)) return PARAM_ERR_TYPE;
    uint16_t v; memcpy(&v, in, sizeof(v));
    if (v < lo || v > hi) return PARAM_ERR_RANGE;
    *dst = v;
    return PARAM_OK;
}


// ---------------------------------------------------------------------
// params_read
// ---------------------------------------------------------------------

param_result_t params_read(reg_id_t id, uint8_t motor_index, void *out, uint32_t out_size)
{
    if (!motor_index_ok(id, motor_index))
        return PARAM_ERR_RANGE;

    motor_config_t  *mc = &motor_flash[motor_index];
    motor_control_t *mr = &motor_ram[motor_index];

    switch (id) {
    // ---- 0x00 system (SYSTEM) ----
    case PARAM_DEVICE_ID:        READ(uint32_t, 0); // TODO: source from STM32 UID
    case PARAM_FW_VERSION:       READ(uint16_t, 0x0001);
    case PARAM_PROTOCOL_VERSION: READ(uint16_t, 0x0002);
    case PARAM_MOTOR_COUNT:      READ(uint8_t, MOTOR_COUNT);

    // ---- 0x10 control source arbitration (MOTOR) ----
    case PARAM_ENABLE:              READ(uint8_t, mr->enable);
    case PARAM_CONTROL_SOURCE:      READ(uint8_t, mr->control_source);
    case PARAM_PROFILE_ENABLE:      READ(uint8_t, mr->profile_enable);
    case PARAM_OVERRIDE_SOURCE:     READ(uint8_t, mr->override_source);
    case PARAM_OVERRIDE_ENABLE:     READ(uint8_t, mr->override_enable);
    case PARAM_TRANSITION_BLEND_MS: READ(uint16_t, mr->transition_blend_ms);

    // ---- 0x20 targets (MOTOR) ----
    case PARAM_TARGET_POSITION: READ(float, mr->target_position);
    case PARAM_TARGET_VELOCITY: READ(float, mr->target_velocity);
    case PARAM_TARGET_CURRENT:  READ(float, mr->target_current);
    case PARAM_CURRENT_FF:      READ(float, mr->current_ff);
    case PARAM_VELOCITY_FF:     READ(float, mr->velocity_ff);

    // ---- 0x30 / 0x31 PID (MOTOR) ----
    case PARAM_POS_KP:             READ(float, mc->pos_kp);
    case PARAM_POS_KI:             READ(float, mc->pos_ki);
    case PARAM_POS_KD:             READ(float, mc->pos_kd);
    case PARAM_POS_INTEGRAL_LIMIT: READ(float, mc->pos_integral_limit);
    case PARAM_VEL_KP:             READ(float, mc->vel_kp);
    case PARAM_VEL_KI:             READ(float, mc->vel_ki);
    case PARAM_VEL_KD:             READ(float, mc->vel_kd);
    case PARAM_VEL_INTEGRAL_LIMIT: READ(float, mc->vel_integral_limit);

    // ---- 0x40 motion profile (MOTOR) ----
    case PARAM_PROFILE_VEL_MAX: READ(float, mc->profile_vel_max);
    case PARAM_PROFILE_ACC_MAX: READ(float, mc->profile_acc_max);
    case PARAM_PROFILE_DEC_MAX: READ(float, mc->profile_dec_max);

    // ---- 0x50 limits / safety (MOTOR except host_timeout_*) ----
    case PARAM_CURRENT_LIMIT:       READ(float, mc->current_limit);
    case PARAM_VELOCITY_LIMIT:      READ(float, mc->velocity_limit);
    case PARAM_POSITION_MIN:        READ(float, mc->position_min);
    case PARAM_POSITION_MAX:        READ(float, mc->position_max);
    case PARAM_CAN_TIMEOUT_MS:      READ(uint16_t, mc->can_timeout_ms);
    case PARAM_HOST_TIMEOUT_MS:     READ(uint16_t, bus_flash.host_timeout_ms);
    case PARAM_HOST_TIMEOUT_ACTION: READ(uint8_t, bus_flash.host_timeout_action);
    case PARAM_MAX_TEMPERATURE:     READ(float, mc->max_temperature);
    case PARAM_MIN_BUS_VOLTAGE:     READ(float, mc->min_bus_voltage);
    case PARAM_MAX_BUS_VOLTAGE:     READ(float, mc->max_bus_voltage);

    // ---- 0x60 feedback (MOTOR, owned by vesc_can.c) ----
    case PARAM_ACTUAL_POSITION:    READ(float, vesc_can_feedback(motor_index)->actual_position);
    case PARAM_ACTUAL_VELOCITY:    READ(float, vesc_can_feedback(motor_index)->actual_velocity);
    case PARAM_ACTUAL_CURRENT:     READ(float, vesc_can_feedback(motor_index)->actual_current);
    case PARAM_ACTUAL_DUTY:        READ(float, vesc_can_feedback(motor_index)->actual_duty);
    case PARAM_ACTUAL_VOLTAGE:     READ(float, vesc_can_feedback(motor_index)->actual_voltage);
    case PARAM_ACTUAL_TEMPERATURE: READ(float, vesc_can_feedback(motor_index)->actual_temperature);
    case PARAM_VESC_LAST_RX_MS:    READ(uint32_t, vesc_can_feedback(motor_index)->vesc_last_rx_ms);
    case PARAM_VESC_ONLINE:             READ(uint8_t, vesc_can_feedback(motor_index)->vesc_alive);
    case PARAM_VELOCITY_FEEDBACK_VALID: READ(uint8_t, vesc_can_feedback(motor_index)->velocity_feedback_valid);
    case PARAM_POSITION_FEEDBACK_VALID: READ(uint8_t, vesc_can_feedback(motor_index)->position_feedback_valid);
    case PARAM_TEMPERATURE_VALID:       READ(uint8_t, vesc_can_feedback(motor_index)->temperature_valid);
    case PARAM_VOLTAGE_VALID:           READ(uint8_t, vesc_can_feedback(motor_index)->voltage_valid);
    case PARAM_CAN_TX_CURRENT_OVERWRITE_COUNT: READ(uint32_t, vesc_can_current_overwrite_count(motor_index));

    // ---- 0x70 state / fault / effective (MOTOR except overrun/dt_max/system_fault, owned by control.c) ----
    case PARAM_STATE:                 READ(uint8_t, control_status(motor_index)->state);
    case PARAM_FAULT_ACTIVE:          READ(uint32_t, control_status(motor_index)->fault_active);
    case PARAM_FAULT_LATCHED:         READ(uint32_t, control_status(motor_index)->fault_latched);
    case PARAM_ACTIVE_SOURCE:         READ(uint8_t, control_status(motor_index)->active_source);
    case PARAM_PROFILE_ACTIVE:        READ(uint8_t, control_status(motor_index)->profile_active);
    case PARAM_EFFECTIVE_POSITION_REF:READ(float, control_status(motor_index)->effective_position_ref);
    case PARAM_EFFECTIVE_VELOCITY_REF:READ(float, control_status(motor_index)->effective_velocity_ref);
    case PARAM_EFFECTIVE_CURRENT_CMD: READ(float, control_status(motor_index)->effective_current_cmd);
    case PARAM_CONTROL_OVERRUN_COUNT: READ(uint32_t, control_system_status()->overrun_count);
    case PARAM_CONTROL_DT_MAX_US:     READ(uint32_t, control_system_status()->dt_max_us);
    case PARAM_SYSTEM_FAULT_ACTIVE:   READ(uint32_t, control_system_status()->system_fault_active);
    case PARAM_SYSTEM_FAULT_LATCHED:  READ(uint32_t, control_system_status()->system_fault_latched);
    case PARAM_CLEAR_FAULT:           return PARAM_ERR_WRITE_ONLY;

    // ---- 0x80 motor / CAN config (MOTOR except can_bitrate) ----
    case PARAM_VESC_CAN_ID:     READ(uint8_t, mc->vesc_can_id);
    case PARAM_CAN_BITRATE:     READ(uint8_t, bus_flash.can_bitrate);
    case PARAM_GEAR_RATIO:      READ(float, mc->gear_ratio);
    case PARAM_MOTOR_DIRECTION: READ(int8_t, mc->motor_direction);
    case PARAM_POSITION_OFFSET: READ(float, mc->position_offset);
    case PARAM_POLE_PAIRS:      READ(uint8_t, mc->pole_pairs);
    case PARAM_ANGLE_SPAN_MOTOR_REV: READ(uint8_t, mc->angle_span_motor_rev);
    case PARAM_MOTOR_PROFILE:   READ(uint8_t, mc->motor_profile);
    case PARAM_ZERO_POSITION:   return PARAM_ERR_WRITE_ONLY;

    // ---- 0x90 persistence (SYSTEM) ----
    case PARAM_SAVE:
    case PARAM_LOAD_DEFAULTS:
    case PARAM_FACTORY_RESET:
        return PARAM_ERR_WRITE_ONLY;

    // ---- 0xA0 USB CDC TX diagnostics (SYSTEM, owned by usbd_cdc_if.c) ----
    case PARAM_CDC_TX_ENQUEUE_COUNT: READ(uint32_t, cdc_tx_get_enqueue_count());
    case PARAM_CDC_TX_DROP_COUNT:    READ(uint32_t, cdc_tx_get_drop_count());
    case PARAM_CDC_TX_BUSY_COUNT:    READ(uint32_t, cdc_tx_get_busy_count());
    case PARAM_CDC_TX_BYTES_PENDING: READ(uint32_t, cdc_tx_get_bytes_pending());
    case PARAM_CDC_TX_HEAD:          READ(uint32_t, cdc_tx_get_head());
    case PARAM_CDC_TX_TAIL:          READ(uint32_t, cdc_tx_get_tail());

    default:
        return PARAM_ERR_UNKNOWN_ID;
    }
}

#undef READ


// ---------------------------------------------------------------------
// params_write
// ---------------------------------------------------------------------

param_result_t params_write(reg_id_t id, uint8_t motor_index, const void *in, uint32_t in_size)
{
    if (!motor_index_ok(id, motor_index))
        return PARAM_ERR_RANGE;

    motor_config_t  *mc = &motor_flash[motor_index];
    motor_control_t *mr = &motor_ram[motor_index];

    switch (id) {
    // ---- 0x00 system: all RO ----
    case PARAM_DEVICE_ID:
    case PARAM_FW_VERSION:
    case PARAM_PROTOCOL_VERSION:
    case PARAM_MOTOR_COUNT:
        return PARAM_ERR_READ_ONLY;

    // ---- 0x10 control source arbitration ----
    case PARAM_ENABLE: {
        if (in_size != sizeof(uint8_t)) return PARAM_ERR_TYPE;
        uint8_t v; memcpy(&v, in, sizeof(v));
        if (v > 1) return PARAM_ERR_RANGE;
        if (v)
            return control_request_enable(motor_index);
        control_request_disable(motor_index);
        return PARAM_OK;
    }

    case PARAM_CONTROL_SOURCE:
        return write_u8(&mr->control_source, in, in_size, CONTROL_SOURCE_NONE, CONTROL_SOURCE_POSITION);

    case PARAM_PROFILE_ENABLE:
        return write_u8(&mr->profile_enable, in, in_size, 0, 1);

    case PARAM_OVERRIDE_SOURCE: {
        // override_source excludes POSITION -- overriding *into* the top of
        // the cascade doesn't mean anything (see params.h).
        if (in_size != sizeof(uint8_t)) return PARAM_ERR_TYPE;
        uint8_t v; memcpy(&v, in, sizeof(v));
        if (v > CONTROL_SOURCE_VELOCITY) return PARAM_ERR_RANGE;
        mr->override_source = v;
        return PARAM_OK;
    }

    case PARAM_OVERRIDE_ENABLE:
        return write_u8(&mr->override_enable, in, in_size, 0, 1);

    case PARAM_TRANSITION_BLEND_MS:
        return write_u16(&mr->transition_blend_ms, in, in_size, 0, 65535);

    // ---- 0x20 targets ----
    case PARAM_TARGET_POSITION: {
        // Reject out-of-range requests outright (rather than silently
        // clamping) so the host finds out immediately -- unlike
        // current/velocity, a stale/wrong position target is not a "safe
        // to clamp and continue" situation. control.c additionally clamps
        // defensively in case position_min/max are narrowed after this
        // write (see update_internal_reference()).
        if (in_size != sizeof(float)) return PARAM_ERR_TYPE;
        float v; memcpy(&v, in, sizeof(v));
        if (v < mc->position_min || v > mc->position_max) return PARAM_ERR_RANGE;
        mr->target_position = v;
        return PARAM_OK;
    }

    case PARAM_TARGET_VELOCITY: {
        if (in_size != sizeof(float)) return PARAM_ERR_TYPE;
        float v; memcpy(&v, in, sizeof(v));
        if (mc->velocity_limit > 0.0f)
            v = clampf(v, -mc->velocity_limit, mc->velocity_limit);
        mr->target_velocity = v;
        return PARAM_OK;
    }

    case PARAM_TARGET_CURRENT: {
        if (in_size != sizeof(float)) return PARAM_ERR_TYPE;
        float v; memcpy(&v, in, sizeof(v));
        v = (mc->current_limit > 0.0f) ? clampf(v, -mc->current_limit, mc->current_limit) : 0.0f;
        mr->target_current = v;
        return PARAM_OK;
    }

    case PARAM_CURRENT_FF:
        return write_f32(&mr->current_ff, in, in_size, -1.0e6f, 1.0e6f);
    case PARAM_VELOCITY_FF:
        return write_f32(&mr->velocity_ff, in, in_size, -1.0e6f, 1.0e6f);

    // ---- 0x30 position PID ----
    case PARAM_POS_KP: return write_f32(&mc->pos_kp, in, in_size, 0.0f, 1.0e6f);
    case PARAM_POS_KI: return write_f32(&mc->pos_ki, in, in_size, 0.0f, 1.0e6f);
    case PARAM_POS_KD: return write_f32(&mc->pos_kd, in, in_size, 0.0f, 1.0e6f);
    case PARAM_POS_INTEGRAL_LIMIT: return write_f32(&mc->pos_integral_limit, in, in_size, 0.0f, 1.0e6f);

    // ---- 0x31 velocity PID ----
    case PARAM_VEL_KP: return write_f32(&mc->vel_kp, in, in_size, 0.0f, 1.0e6f);
    case PARAM_VEL_KI: return write_f32(&mc->vel_ki, in, in_size, 0.0f, 1.0e6f);
    case PARAM_VEL_KD: return write_f32(&mc->vel_kd, in, in_size, 0.0f, 1.0e6f);
    case PARAM_VEL_INTEGRAL_LIMIT: return write_f32(&mc->vel_integral_limit, in, in_size, 0.0f, 1.0e6f);

    // ---- 0x40 motion profile: all strictly positive (profile_dec_max in
    // particular is a divisor in control.c's trapezoidal_step()) ----
    case PARAM_PROFILE_VEL_MAX: return write_f32(&mc->profile_vel_max, in, in_size, 1.0e-3f, 1.0e6f);
    case PARAM_PROFILE_ACC_MAX: return write_f32(&mc->profile_acc_max, in, in_size, 1.0e-3f, 1.0e6f);
    case PARAM_PROFILE_DEC_MAX: return write_f32(&mc->profile_dec_max, in, in_size, 1.0e-3f, 1.0e6f);

    // ---- 0x50 limits / safety ----
    case PARAM_CURRENT_LIMIT:  return write_f32(&mc->current_limit, in, in_size, 0.0f, 1.0e6f);
    case PARAM_VELOCITY_LIMIT: return write_f32(&mc->velocity_limit, in, in_size, 0.0f, 1.0e6f);

    case PARAM_POSITION_MIN: {
        if (in_size != sizeof(float)) return PARAM_ERR_TYPE;
        float v; memcpy(&v, in, sizeof(v));
        if (v > mc->position_max) return PARAM_ERR_RANGE;
        mc->position_min = v;
        return PARAM_OK;
    }
    case PARAM_POSITION_MAX: {
        if (in_size != sizeof(float)) return PARAM_ERR_TYPE;
        float v; memcpy(&v, in, sizeof(v));
        if (v < mc->position_min) return PARAM_ERR_RANGE;
        mc->position_max = v;
        return PARAM_OK;
    }

    case PARAM_CAN_TIMEOUT_MS:  return write_u16(&mc->can_timeout_ms, in, in_size, 1, 65535);

    case PARAM_HOST_TIMEOUT_MS: {
        // A watchdog-required session (binary/ROS2 -- see
        // control_require_host_watchdog()) must not be able to disarm its
        // own watchdog while ANY motor is RUNNING: control_request_enable()
        // only checks this at the moment of enabling, so without this
        // check a session with one motor already armed could WRITE_PARAM
        // its way to host_timeout_ms=0 and keep driving that motor on a
        // stale command with nothing watching. ASCII sessions
        // (host_watchdog_required == false) are unaffected -- ASCII
        // bring-up is allowed timeout=0 by design. System-wide: this one
        // threshold gates every motor's watchdog (see params.h's "Two-tier
        // host watchdog" comment), so the check is against ANY motor
        // running, not just motor_index (which is always 0 here anyway --
        // SYSTEM scope).
        if (in_size != sizeof(uint16_t)) return PARAM_ERR_TYPE;
        uint16_t v; memcpy(&v, in, sizeof(v));
        if (control_host_watchdog_required() && any_motor_running() && v == 0)
            return PARAM_ERR_STATE;
        bus_flash.host_timeout_ms = v;
        return PARAM_OK;
    }
    case PARAM_HOST_TIMEOUT_ACTION: return write_u8(&bus_flash.host_timeout_action, in, in_size, HOST_TIMEOUT_HOLD, HOST_TIMEOUT_FAULT);
    case PARAM_MAX_TEMPERATURE: return write_f32(&mc->max_temperature, in, in_size, 0.0f, 300.0f);
    case PARAM_MIN_BUS_VOLTAGE: return write_f32(&mc->min_bus_voltage, in, in_size, 0.0f, 1000.0f);
    case PARAM_MAX_BUS_VOLTAGE: return write_f32(&mc->max_bus_voltage, in, in_size, 0.0f, 1000.0f);

    // ---- 0x60 / 0x70: all RO ----
    case PARAM_ACTUAL_POSITION:
    case PARAM_ACTUAL_VELOCITY:
    case PARAM_ACTUAL_CURRENT:
    case PARAM_ACTUAL_DUTY:
    case PARAM_ACTUAL_VOLTAGE:
    case PARAM_ACTUAL_TEMPERATURE:
    case PARAM_VESC_LAST_RX_MS:
    case PARAM_VESC_ONLINE:
    case PARAM_VELOCITY_FEEDBACK_VALID:
    case PARAM_POSITION_FEEDBACK_VALID:
    case PARAM_TEMPERATURE_VALID:
    case PARAM_VOLTAGE_VALID:
    case PARAM_CAN_TX_CURRENT_OVERWRITE_COUNT:
    case PARAM_STATE:
    case PARAM_FAULT_ACTIVE:
    case PARAM_FAULT_LATCHED:
    case PARAM_ACTIVE_SOURCE:
    case PARAM_PROFILE_ACTIVE:
    case PARAM_EFFECTIVE_POSITION_REF:
    case PARAM_EFFECTIVE_VELOCITY_REF:
    case PARAM_EFFECTIVE_CURRENT_CMD:
    case PARAM_CONTROL_OVERRUN_COUNT:
    case PARAM_CONTROL_DT_MAX_US:
    case PARAM_SYSTEM_FAULT_ACTIVE:
    case PARAM_SYSTEM_FAULT_LATCHED:
        return PARAM_ERR_READ_ONLY;

    case PARAM_CLEAR_FAULT:
        control_clear_fault(motor_index);
        return PARAM_OK;

    // ---- 0x80 motor / CAN config: refused while that motor is RUNNING
    // (VESC/ODrive both refuse to let you change wiring/bus config out from
    // under an armed control loop). vesc_can_id/gear_ratio/motor_direction/
    // pole_pairs/angle_span_motor_rev all change how received frames are
    // addressed or interpreted, so a successful write also resets
    // vesc_can.c's decoded state for THIS motor
    // (vesc_can_reset_feedback(motor_index)) -- otherwise the pid_pos
    // unwrap accumulator (or, for vesc_can_id, frames from the *old* VESC)
    // would carry over and be reinterpreted under the new config. Since all
    // of these are refused while that motor is RUNNING, this only ever
    // runs from IDLE. ----
    case PARAM_VESC_CAN_ID: {
        // Duplicate IDs across motors are intentionally NOT rejected here
        // -- see this register's comment in params.h and
        // control_request_enable()'s duplicate check.
        if (is_running(motor_index)) return PARAM_ERR_STATE;
        param_result_t r = write_u8(&mc->vesc_can_id, in, in_size, 0, 0xFF);
        if (r == PARAM_OK) vesc_can_reset_feedback(motor_index);
        return r;
    }

    case PARAM_CAN_BITRATE:
        // Bus-wide: refused while ANY motor is RUNNING, not just motor_index
        // (which is always 0 here -- SYSTEM scope, one physical bus).
        if (any_motor_running()) return PARAM_ERR_STATE;
        // NOT followed by vesc_can_reset_feedback() -- unlike the others,
        // this doesn't actually reconfigure the FDCAN peripheral until the
        // next vesc_can_init(). v0.1: takes effect after reboot.
        return write_u8(&bus_flash.can_bitrate, in, in_size, 0, CAN_BITRATE_INVALID - 1);

    case PARAM_GEAR_RATIO: {
        if (is_running(motor_index)) return PARAM_ERR_STATE;
        param_result_t r = write_f32(&mc->gear_ratio, in, in_size, 1.0e-6f, 1.0e6f);
        if (r == PARAM_OK) vesc_can_reset_feedback(motor_index);
        return r;
    }

    case PARAM_MOTOR_DIRECTION: {
        if (is_running(motor_index)) return PARAM_ERR_STATE;
        if (in_size != sizeof(int8_t)) return PARAM_ERR_TYPE;
        int8_t v; memcpy(&v, in, sizeof(v));
        if (v != 1 && v != -1) return PARAM_ERR_RANGE;
        mc->motor_direction = v;
        vesc_can_reset_feedback(motor_index);
        return PARAM_OK;
    }

    case PARAM_POSITION_OFFSET:
        // Direct writes don't get the atomic target_position/position_ref
        // shift that PARAM_ZERO_POSITION performs, so unlike ZERO (which
        // IS allowed while RUNNING, being atomic/impulse-free) this would
        // introduce a step in actual_position relative to whatever the
        // cascade is currently chasing -- refused while RUNNING for the
        // same reason as gear_ratio/motor_direction above.
        if (is_running(motor_index)) return PARAM_ERR_STATE;
        return write_f32(&mc->position_offset, in, in_size, -1.0e6f, 1.0e6f);

    case PARAM_POLE_PAIRS: {
        if (is_running(motor_index)) return PARAM_ERR_STATE;
        param_result_t r = write_u8(&mc->pole_pairs, in, in_size, 1, 100);
        if (r == PARAM_OK) vesc_can_reset_feedback(motor_index);
        return r;
    }

    case PARAM_ANGLE_SPAN_MOTOR_REV: {
        if (is_running(motor_index)) return PARAM_ERR_STATE;
        param_result_t r = write_u8(&mc->angle_span_motor_rev, in, in_size, 1, 100);
        if (r == PARAM_OK) vesc_can_reset_feedback(motor_index);
        return r;
    }

    case PARAM_MOTOR_PROFILE:
        if (is_running(motor_index)) return PARAM_ERR_STATE;
        return write_u8(&mc->motor_profile, in, in_size, MOTOR_PROFILE_UNSET, MOTOR_PROFILE_CUSTOM);

    case PARAM_ZERO_POSITION:
        return control_zero_position(motor_index);

    // ---- 0x90 persistence: refused while ANY motor is RUNNING -- e.g.
    // LOAD_DEFAULTS rewrites current_limit/gear_ratio/PID gains/etc for
    // every motor out from under an armed control loop otherwise. ----
    case PARAM_SAVE:
        if (any_motor_running()) return PARAM_ERR_STATE;
        return params_save() ? PARAM_OK : PARAM_ERR_IO;

    case PARAM_LOAD_DEFAULTS:
        // Genuinely RAM-only by design (not a stub) -- real success.
        // Resets vesc_can_id/gear_ratio/motor_direction/pole_pairs/
        // angle_span_motor_rev for every motor at once, so this needs the
        // same vesc_can_reset_feedback() the individual per-field writes
        // get (see PARAM_VESC_CAN_ID etc above) -- otherwise each motor's
        // pid_pos unwrap accumulator would carry over and be reinterpreted
        // under the now-default scale factors. can_bitrate is reset too
        // but (as with the individual write) doesn't take effect until
        // reboot.
        if (any_motor_running()) return PARAM_ERR_STATE;
        params_load_defaults();
        for (uint8_t i = 0; i < MOTOR_COUNT; i++)
            vesc_can_reset_feedback(i);
        return PARAM_OK;

    case PARAM_FACTORY_RESET:
        // Same reasoning as PARAM_SAVE -- refused before flash is ever
        // touched, no side effect at all if refused.
        if (any_motor_running()) return PARAM_ERR_STATE;
        return params_factory_reset() ? PARAM_OK : PARAM_ERR_IO;

    // ---- 0xA0 USB CDC TX diagnostics: all RO ----
    case PARAM_CDC_TX_ENQUEUE_COUNT:
    case PARAM_CDC_TX_DROP_COUNT:
    case PARAM_CDC_TX_BUSY_COUNT:
    case PARAM_CDC_TX_BYTES_PENDING:
    case PARAM_CDC_TX_HEAD:
    case PARAM_CDC_TX_TAIL:
        return PARAM_ERR_READ_ONLY;

    default:
        return PARAM_ERR_UNKNOWN_ID;
    }
}
