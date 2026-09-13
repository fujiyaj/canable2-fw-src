//
// shell: ASCII debug shell over USB CDC. See inc/shell.h for the command
// list and the "thin layer over params_read()/params_write()" design note
// -- this file must never touch bus_config_t/motor_config_t/motor_control_t
// or control.c/vesc_can.c state directly.
//
// ---------------------------------------------------------------------
// Motor addressing
// ---------------------------------------------------------------------
// Every register is either SYSTEM-scope (one board-wide instance) or
// MOTOR-scope (one per motor, index 0..MOTOR_COUNT-1) -- see params.h's
// "4-motor addressing" comment and param_scope(). get/set take an OPTIONAL
// leading index token:
//
//   get can_bitrate            (SYSTEM -- no index)
//   get 2 actual_position      (MOTOR -- index required)
//   set 1 gear_ratio 36        (MOTOR -- index required)
//
// A MOTOR-scope field used without an index, or a SYSTEM-scope field used
// with an index, is a syntax error (ERR SYNTAX / ERR RANGE) -- there is no
// silent "defaults to motor 0" fallback, since that would be easy to
// mistake for a board-wide read. enable/disable/clear_fault/zero are
// per-motor verbs and take a mandatory trailing index the same way
// ("enable 0", "clear_fault 2").
//

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "stm32g4xx_hal.h" // HAL_GetTick()
#include "printf.h"        // snprintf_ (PRINTF_SUPPORT_FLOAT) -- see printf.h; NOT printf()/_putchar, unimplemented in this project
#include "usbd_cdc_if.h"    // cdc_transmit()
#include "params.h"
#include "control.h"
#include "binproto.h"
#include "shell.h"

#define LINE_MAX 96
#define OUT_MAX  96

static char line_buf[LINE_MAX];
static uint16_t line_len;


// ---------------------------------------------------------------------
// Register name table -- one entry per get/set-able reg_id_t. WO triggers
// (PARAM_CLEAR_FAULT/PARAM_ZERO_POSITION/PARAM_SAVE/PARAM_LOAD_DEFAULTS/
// PARAM_FACTORY_RESET) are deliberately NOT here -- they're bare verbs
// instead (clear_fault/zero/save/load_defaults/factory_reset), since
// "set clear_fault 1" is a strange way to spell a trigger. Scope (SYSTEM
// vs MOTOR) is NOT duplicated here -- param_scope(id) (params.c) is the
// single source of truth; this table calls it at parse time.
// ---------------------------------------------------------------------

typedef enum
{
    T_F32,
    T_U8,
    T_U16,
    T_U32,
    T_I8,
    T_U32_HEX,      // fault_active/fault_latched -- printed as 0x%08lX
    T_ENUM_SOURCE,  // control_source_t -- printed/parsed as none/current/velocity/position
    T_ENUM_STATE,   // controller_state_t -- printed only, RO (no field uses this as settable)
} field_type_t;

typedef struct
{
    const char *name;
    reg_id_t id;
    field_type_t type;
} field_t;

static const field_t FIELDS[] =
{
    {"device_id",               PARAM_DEVICE_ID,               T_U32},
    {"fw_version",               PARAM_FW_VERSION,              T_U16},
    {"protocol_version",         PARAM_PROTOCOL_VERSION,        T_U16},
    {"motor_count",              PARAM_MOTOR_COUNT,             T_U8},

    {"enable",                   PARAM_ENABLE,                  T_U8}, // also reachable via the bare "enable <i>"/"disable <i>" verbs
    {"control_source",           PARAM_CONTROL_SOURCE,          T_ENUM_SOURCE},
    {"profile_enable",           PARAM_PROFILE_ENABLE,          T_U8},
    {"override_source",          PARAM_OVERRIDE_SOURCE,         T_ENUM_SOURCE},
    {"override_enable",          PARAM_OVERRIDE_ENABLE,         T_U8},
    {"transition_blend_ms",      PARAM_TRANSITION_BLEND_MS,     T_U16},

    {"target_position",          PARAM_TARGET_POSITION,         T_F32},
    {"target_velocity",          PARAM_TARGET_VELOCITY,         T_F32},
    {"target_current",           PARAM_TARGET_CURRENT,          T_F32},
    {"current_ff",               PARAM_CURRENT_FF,              T_F32},
    {"velocity_ff",              PARAM_VELOCITY_FF,             T_F32},

    {"pos_kp",                   PARAM_POS_KP,                  T_F32},
    {"pos_ki",                   PARAM_POS_KI,                  T_F32},
    {"pos_kd",                   PARAM_POS_KD,                  T_F32},
    {"pos_integral_limit",       PARAM_POS_INTEGRAL_LIMIT,      T_F32},

    {"vel_kp",                   PARAM_VEL_KP,                  T_F32},
    {"vel_ki",                   PARAM_VEL_KI,                  T_F32},
    {"vel_kd",                   PARAM_VEL_KD,                  T_F32},
    {"vel_integral_limit",       PARAM_VEL_INTEGRAL_LIMIT,      T_F32},

    {"profile_vel_max",          PARAM_PROFILE_VEL_MAX,         T_F32},
    {"profile_acc_max",          PARAM_PROFILE_ACC_MAX,         T_F32},
    {"profile_dec_max",          PARAM_PROFILE_DEC_MAX,         T_F32},

    {"current_limit",            PARAM_CURRENT_LIMIT,           T_F32},
    {"velocity_limit",           PARAM_VELOCITY_LIMIT,          T_F32},
    {"position_min",             PARAM_POSITION_MIN,            T_F32},
    {"position_max",             PARAM_POSITION_MAX,            T_F32},
    {"can_timeout_ms",           PARAM_CAN_TIMEOUT_MS,          T_U16},
    {"host_timeout_ms",          PARAM_HOST_TIMEOUT_MS,         T_U16}, // SYSTEM -- see param_scope()
    {"host_timeout_action",      PARAM_HOST_TIMEOUT_ACTION,     T_U8},  // SYSTEM -- 0 HOLD / 1 CURRENT_ZERO / 2 FAULT, raw numeric (v0.1)
    {"max_temperature",          PARAM_MAX_TEMPERATURE,         T_F32},
    {"min_bus_voltage",          PARAM_MIN_BUS_VOLTAGE,         T_F32},
    {"max_bus_voltage",          PARAM_MAX_BUS_VOLTAGE,         T_F32},

    {"actual_position",          PARAM_ACTUAL_POSITION,         T_F32},
    {"actual_velocity",          PARAM_ACTUAL_VELOCITY,         T_F32},
    {"actual_current",           PARAM_ACTUAL_CURRENT,          T_F32},
    {"actual_duty",               PARAM_ACTUAL_DUTY,             T_F32},
    {"actual_voltage",           PARAM_ACTUAL_VOLTAGE,          T_F32},
    {"actual_temperature",       PARAM_ACTUAL_TEMPERATURE,      T_F32},
    {"vesc_last_rx_ms",          PARAM_VESC_LAST_RX_MS,         T_U32},
    {"vesc_online",              PARAM_VESC_ONLINE,             T_U8},
    {"velocity_feedback_valid",  PARAM_VELOCITY_FEEDBACK_VALID, T_U8},
    {"position_feedback_valid",  PARAM_POSITION_FEEDBACK_VALID, T_U8},
    {"temperature_valid",        PARAM_TEMPERATURE_VALID,       T_U8},
    {"voltage_valid",            PARAM_VOLTAGE_VALID,           T_U8},
    {"can_tx_current_overwrite_count",PARAM_CAN_TX_CURRENT_OVERWRITE_COUNT,T_U32},

    {"state",                    PARAM_STATE,                   T_ENUM_STATE},
    {"fault_active",             PARAM_FAULT_ACTIVE,            T_U32_HEX},
    {"fault_latched",            PARAM_FAULT_LATCHED,           T_U32_HEX},
    {"active_source",            PARAM_ACTIVE_SOURCE,           T_ENUM_SOURCE},
    {"profile_active",           PARAM_PROFILE_ACTIVE,          T_U8},
    {"effective_position_ref",   PARAM_EFFECTIVE_POSITION_REF,  T_F32},
    {"effective_velocity_ref",   PARAM_EFFECTIVE_VELOCITY_REF,  T_F32},
    {"effective_current_cmd",    PARAM_EFFECTIVE_CURRENT_CMD,   T_F32},
    {"control_overrun_count",    PARAM_CONTROL_OVERRUN_COUNT,   T_U32}, // SYSTEM
    {"control_dt_max_us",        PARAM_CONTROL_DT_MAX_US,       T_U32}, // SYSTEM
    {"system_fault_active",      PARAM_SYSTEM_FAULT_ACTIVE,     T_U32_HEX}, // SYSTEM
    {"system_fault_latched",     PARAM_SYSTEM_FAULT_LATCHED,    T_U32_HEX}, // SYSTEM

    {"vesc_can_id",              PARAM_VESC_CAN_ID,             T_U8},
    {"can_bitrate",               PARAM_CAN_BITRATE,             T_U8}, // SYSTEM -- enum can_bitrate (can.h), raw numeric (v0.1)
    {"gear_ratio",                PARAM_GEAR_RATIO,              T_F32},
    {"motor_direction",          PARAM_MOTOR_DIRECTION,         T_I8},
    {"position_offset",          PARAM_POSITION_OFFSET,         T_F32},
    {"pole_pairs",                PARAM_POLE_PAIRS,              T_U8},
    {"angle_span_motor_rev",     PARAM_ANGLE_SPAN_MOTOR_REV,    T_U8},
    {"motor_profile",            PARAM_MOTOR_PROFILE,           T_U8}, // 0 UNSET / 1 M2006 / 2 M3508 / 3 CUSTOM -- raw numeric (v0.1, "the box")

    {"cdc_tx_enqueue_count",     PARAM_CDC_TX_ENQUEUE_COUNT,    T_U32}, // SYSTEM
    {"cdc_tx_drop_count",        PARAM_CDC_TX_DROP_COUNT,       T_U32}, // SYSTEM
    {"cdc_tx_busy_count",        PARAM_CDC_TX_BUSY_COUNT,       T_U32}, // SYSTEM
    {"cdc_tx_bytes_pending",     PARAM_CDC_TX_BYTES_PENDING,    T_U32}, // SYSTEM
    {"cdc_tx_head",              PARAM_CDC_TX_HEAD,             T_U32}, // SYSTEM
    {"cdc_tx_tail",              PARAM_CDC_TX_TAIL,             T_U32}, // SYSTEM
};

#define NUM_FIELDS (sizeof(FIELDS) / sizeof(FIELDS[0]))


static const char *SOURCE_NAMES[4] = {"none", "current", "velocity", "position"};
static const char *STATE_NAMES[4] = {"boot", "idle", "running", "fault"};

static const char *source_name(uint8_t v) { return (v < 4) ? SOURCE_NAMES[v] : "?"; }
static const char *state_name(uint8_t v)  { return (v < 4) ? STATE_NAMES[v]  : "?"; }

static int32_t source_from_name(const char *s)
{
    for (int32_t i = 0; i < 4; i++)
        if (strcmp(SOURCE_NAMES[i], s) == 0)
            return i;
    return -1;
}

static uint32_t field_size(field_type_t t)
{
    switch (t) {
    case T_F32: case T_U32: case T_U32_HEX: return 4;
    case T_U16: return 2;
    case T_U8: case T_I8: case T_ENUM_SOURCE: case T_ENUM_STATE: return 1;
    default: return 0;
    }
}

static int32_t find_field(const char *name)
{
    for (uint32_t i = 0; i < NUM_FIELDS; i++)
        if (strcmp(FIELDS[i].name, name) == 0)
            return (int32_t)i;
    return -1;
}

// Renders raw[0..field_size(f->type)) into a human string.
static void format_field(char *out, size_t outsz, const field_t *f, const uint8_t *raw)
{
    switch (f->type) {
    case T_F32: { float v; memcpy(&v, raw, 4); snprintf(out, outsz, "%f", (double)v); break; }
    case T_U8:  { uint8_t v; memcpy(&v, raw, 1); snprintf(out, outsz, "%u", (unsigned)v); break; }
    case T_U16: { uint16_t v; memcpy(&v, raw, 2); snprintf(out, outsz, "%u", (unsigned)v); break; }
    case T_U32: { uint32_t v; memcpy(&v, raw, 4); snprintf(out, outsz, "%lu", (unsigned long)v); break; }
    case T_I8:  { int8_t v; memcpy(&v, raw, 1); snprintf(out, outsz, "%d", (int)v); break; }
    case T_U32_HEX: { uint32_t v; memcpy(&v, raw, 4); snprintf(out, outsz, "0x%08lX", (unsigned long)v); break; }
    case T_ENUM_SOURCE: { uint8_t v; memcpy(&v, raw, 1); snprintf(out, outsz, "%s", source_name(v)); break; }
    case T_ENUM_STATE:  { uint8_t v; memcpy(&v, raw, 1); snprintf(out, outsz, "%s", state_name(v)); break; }
    default: out[0] = 0; break;
    }
}

// Parses text into raw[0..field_size(f->type)); returns false on a type
// this field doesn't accept writes for (currently just T_ENUM_STATE, which
// no field actually uses -- kept for completeness).
static bool parse_field(const field_t *f, const char *text, uint8_t *raw)
{
    switch (f->type) {
    case T_F32: { float v = strtof(text, NULL); memcpy(raw, &v, 4); return true; }
    case T_U32: case T_U32_HEX: { uint32_t v = (uint32_t)strtoul(text, NULL, 0); memcpy(raw, &v, 4); return true; }
    case T_U16: { uint16_t v = (uint16_t)strtoul(text, NULL, 0); memcpy(raw, &v, 2); return true; }
    case T_U8:  { uint8_t v = (uint8_t)strtoul(text, NULL, 0); memcpy(raw, &v, 1); return true; }
    case T_I8:  { int8_t v = (int8_t)strtol(text, NULL, 0); memcpy(raw, &v, 1); return true; }
    case T_ENUM_SOURCE: {
        int32_t idx = source_from_name(text);
        uint8_t v = (idx >= 0) ? (uint8_t)idx : (uint8_t)strtoul(text, NULL, 0); // numeric fallback
        memcpy(raw, &v, 1);
        return true;
    }
    default: return false;
    }
}


// ---------------------------------------------------------------------
// output helpers
// ---------------------------------------------------------------------

static void reply(const char *s)
{
    cdc_transmit((uint8_t *)s, (uint16_t)strlen(s));
}

static void replyln(const char *s)
{
    reply(s);
    reply("\r\n");
}

static const char *err_name(param_result_t r)
{
    switch (r) {
    case PARAM_OK:                   return "OK";
    case PARAM_ERR_UNKNOWN_ID:       return "ERR UNKNOWN_ID";
    case PARAM_ERR_READ_ONLY:        return "ERR READ_ONLY";
    case PARAM_ERR_WRITE_ONLY:       return "ERR WRITE_ONLY";
    case PARAM_ERR_TYPE:             return "ERR TYPE";
    case PARAM_ERR_RANGE:            return "ERR RANGE";
    case PARAM_ERR_STATE:            return "ERR STATE";
    case PARAM_ERR_CONFIG:           return "ERR CONFIG";
    case PARAM_ERR_NOT_IMPLEMENTED:  return "ERR NOT_IMPLEMENTED";
    case PARAM_ERR_IO:               return "ERR IO";
    default:                         return "ERR UNKNOWN";
    }
}

// Parses a decimal token as a motor index; returns false if it's not a
// plain non-negative integer or is out of [0, MOTOR_COUNT).
static bool parse_motor_index(const char *tok, uint8_t *out)
{
    if (!tok || tok[0] == '\0') return false;
    for (const char *p = tok; *p; p++)
        if (*p < '0' || *p > '9') return false;
    unsigned long v = strtoul(tok, NULL, 10);
    if (v >= MOTOR_COUNT) return false;
    *out = (uint8_t)v;
    return true;
}


// ---------------------------------------------------------------------
// commands
// ---------------------------------------------------------------------

// get [<index>] <name> -- tok_b non-NULL means "tok_a index, tok_b name";
// tok_b NULL means "tok_a name, no index" (only valid for SYSTEM scope).
static void cmd_get(char *tok_a, char *tok_b)
{
    const char *name;
    uint8_t index = 0;
    bool has_index = false;

    if (tok_b) {
        if (!parse_motor_index(tok_a, &index)) { replyln("ERR SYNTAX"); return; }
        has_index = true;
        name = tok_b;
    } else if (tok_a) {
        name = tok_a;
    } else {
        replyln("ERR SYNTAX"); return;
    }

    int32_t idx = find_field(name);
    if (idx < 0) { replyln("ERR UNKNOWN_ID"); return; }

    if (param_scope(FIELDS[idx].id) == PARAM_SCOPE_MOTOR && !has_index) { replyln("ERR SYNTAX"); return; }

    uint8_t raw[4] = {0};
    param_result_t r = params_read(FIELDS[idx].id, index, raw, field_size(FIELDS[idx].type));
    if (r != PARAM_OK) { replyln(err_name(r)); return; }

    char valbuf[OUT_MAX];
    format_field(valbuf, sizeof(valbuf), &FIELDS[idx], raw);
    replyln(valbuf);
}

// set [<index>] <name> <value> -- tok_c non-NULL means "tok_a index, tok_b
// name, tok_c value"; tok_c NULL means "tok_a name, tok_b value, no index"
// (only valid for SYSTEM scope).
static void cmd_set(char *tok_a, char *tok_b, char *tok_c)
{
    const char *name, *value;
    uint8_t index = 0;
    bool has_index = false;

    if (tok_c) {
        if (!parse_motor_index(tok_a, &index)) { replyln("ERR SYNTAX"); return; }
        has_index = true;
        name = tok_b;
        value = tok_c;
    } else if (tok_a && tok_b) {
        name = tok_a;
        value = tok_b;
    } else {
        replyln("ERR SYNTAX"); return;
    }

    int32_t idx = find_field(name);
    if (idx < 0) { replyln("ERR UNKNOWN_ID"); return; }

    if (param_scope(FIELDS[idx].id) == PARAM_SCOPE_MOTOR && !has_index) { replyln("ERR SYNTAX"); return; }

    uint8_t raw[4] = {0};
    if (!parse_field(&FIELDS[idx], value, raw)) { replyln("ERR TYPE"); return; }

    param_result_t r = params_write(FIELDS[idx].id, index, raw, field_size(FIELDS[idx].type));
    replyln(err_name(r));
}

// One params_write() trigger call for a MOTOR-scope WO trigger (clear_fault/zero).
static void cmd_trigger_motor(reg_id_t id, char *index_tok)
{
    uint8_t index;
    if (!parse_motor_index(index_tok, &index)) { replyln("ERR SYNTAX"); return; }
    param_result_t r = params_write(id, index, NULL, 0);
    replyln(err_name(r));
}

// One params_write() trigger call for a SYSTEM-scope WO trigger (save/load_defaults/factory_reset).
static void cmd_trigger_system(reg_id_t id)
{
    param_result_t r = params_write(id, 0, NULL, 0);
    replyln(err_name(r));
}

static void cmd_status(void)
{
    char buf[OUT_MAX];
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        uint8_t state_v = 0, source_v = 0;
        float pos = 0, vel = 0, cur = 0;
        uint32_t fault = 0;
        params_read(PARAM_STATE, i, &state_v, 1);
        params_read(PARAM_ACTIVE_SOURCE, i, &source_v, 1);
        params_read(PARAM_ACTUAL_POSITION, i, &pos, 4);
        params_read(PARAM_ACTUAL_VELOCITY, i, &vel, 4);
        params_read(PARAM_ACTUAL_CURRENT, i, &cur, 4);
        params_read(PARAM_FAULT_ACTIVE, i, &fault, 4);
        snprintf(buf, sizeof(buf), "M%u STATE %s SOURCE %s POS %f VEL %f CUR %f FAULT 0x%08lX",
                 (unsigned)i, state_name(state_v), source_name(source_v),
                 (double)pos, (double)vel, (double)cur, (unsigned long)fault);
        replyln(buf);
    }
    uint32_t sys_active = 0, sys_latched = 0;
    params_read(PARAM_SYSTEM_FAULT_ACTIVE, 0, &sys_active, 4);
    params_read(PARAM_SYSTEM_FAULT_LATCHED, 0, &sys_latched, 4);
    snprintf(buf, sizeof(buf), "SYS FAULT_ACTIVE 0x%08lX FAULT_LATCHED 0x%08lX",
             (unsigned long)sys_active, (unsigned long)sys_latched);
    replyln(buf);
}

static void cmd_help(void)
{
    replyln("commands: get [<i>] <name> | set [<i>] <name> <value> | enable <i> | disable <i> |");
    replyln("          clear_fault <i> | zero <i> | save | load_defaults | factory_reset |");
    replyln("          status | help | binary  (switches to the ROS2 binary protocol,");
    replyln("          see the project docs -- USB replug or MCU reset to come back)");
    replyln("<i> is a motor index 0..MOTOR_COUNT-1, required for per-motor registers and");
    replyln("omitted entirely for board-wide (SYSTEM-scope) ones -- see 'get'/'set' below.");
    replyln("registers:");
    for (uint32_t i = 0; i < NUM_FIELDS; i++)
        replyln(FIELDS[i].name);
}


// ---------------------------------------------------------------------
// line parsing / dispatch
// ---------------------------------------------------------------------

static void process_line(char *s)
{
    char *tok = strtok(s, " \t");
    if (!tok)
        return; // blank line, ignore silently

    control_notify_host_rx(HAL_GetTick()); // any recognized-or-not command line counts as "host is alive"

    if (strcmp(tok, "get") == 0) {
        char *a = strtok(NULL, " \t");
        char *b = strtok(NULL, " \t");
        cmd_get(a, b);
    } else if (strcmp(tok, "set") == 0) {
        char *a = strtok(NULL, " \t");
        char *b = strtok(NULL, " \t");
        char *c = strtok(NULL, " \t");
        cmd_set(a, b, c);
    } else if (strcmp(tok, "enable") == 0) {
        uint8_t index;
        if (!parse_motor_index(strtok(NULL, " \t"), &index)) { replyln("ERR SYNTAX"); return; }
        uint8_t v = 1;
        replyln(err_name(params_write(PARAM_ENABLE, index, &v, 1)));
    } else if (strcmp(tok, "disable") == 0) {
        uint8_t index;
        if (!parse_motor_index(strtok(NULL, " \t"), &index)) { replyln("ERR SYNTAX"); return; }
        uint8_t v = 0;
        replyln(err_name(params_write(PARAM_ENABLE, index, &v, 1)));
    } else if (strcmp(tok, "clear_fault") == 0) {
        cmd_trigger_motor(PARAM_CLEAR_FAULT, strtok(NULL, " \t"));
    } else if (strcmp(tok, "zero") == 0) {
        cmd_trigger_motor(PARAM_ZERO_POSITION, strtok(NULL, " \t"));
    } else if (strcmp(tok, "save") == 0) {
        cmd_trigger_system(PARAM_SAVE);
    } else if (strcmp(tok, "load_defaults") == 0) {
        cmd_trigger_system(PARAM_LOAD_DEFAULTS);
    } else if (strcmp(tok, "factory_reset") == 0) {
        cmd_trigger_system(PARAM_FACTORY_RESET);
    } else if (strcmp(tok, "status") == 0) {
        cmd_status();
    } else if (strcmp(tok, "help") == 0) {
        cmd_help();
    } else if (strcmp(tok, "binary") == 0) {
        // Only if every motor is IDLE: switching while any motor is RUNNING
        // would let an ASCII session that's already armed (possibly with
        // host_timeout_ms==0, which ASCII allows) hand control to binary
        // mode without ever passing through control_request_enable()'s
        // watchdog check for that motor -- "binary but no watchdog and
        // already spinning" is exactly the gap that check exists to
        // prevent. Disable from ASCII first.
        bool all_idle = true;
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            uint8_t state_v = 0;
            params_read(PARAM_STATE, i, &state_v, 1);
            if (state_v != STATE_IDLE) { all_idle = false; break; }
        }
        if (!all_idle) {
            replyln("ERR STATE");
            return;
        }
        // Reply BEFORE switching -- the client is still speaking ASCII at
        // this point and needs to see this line before any COBS-framed
        // bytes could show up. See binproto.h for the reverse switch
        // (USB bus reset / MCU reset only -- there's no ASCII command to
        // go back, ROS2 shouldn't accidentally trigger it mid-flight).
        replyln("OK BINARY 2");
        binproto_enter_binary_mode();
    } else {
        replyln("ERR UNKNOWN_COMMAND");
    }
}

void shell_feed_byte(uint8_t b)
{
    if (b == '\r')
        return; // wait for the LF

    if (b == '\n') {
        line_buf[line_len] = '\0';
        process_line(line_buf);
        line_len = 0;
        return;
    }

    if (b == 0x08 || b == 0x7F) { // backspace / DEL
        if (line_len > 0)
            line_len--;
        return;
    }

    if (line_len + 1 < LINE_MAX)
        line_buf[line_len++] = (char)b;
    // else: silently drop until the next LF resyncs the line
}
