//
// binproto: binary protocol (v2, 4-motor addressing) for ROS2/non-human
// clients over USB CDC. See inc/binproto.h for the wire format and message
// list.
//

#include <string.h>

#include "stm32g4xx_hal.h" // HAL_GetTick()
#include "cobs.h"
#include "crc16.h"
#include "usbd_cdc_if.h" // cdc_transmit()
#include "params.h"
#include "control.h"
#include "vesc_can.h"
#include "binproto.h"

#define RAW_MAX 128                           // pre-COBS frame bytes (header+payload+crc), see binproto.h
#define ENC_MAX (COBS_ENCODED_MAX(RAW_MAX) + 1) // +1 for the trailing 0x00 delimiter
#define HEADER_LEN 6                           // version(1) + type(1) + sequence(2) + payload_length(2)

static uint8_t mode_binary;

static uint8_t rx_enc[ENC_MAX];
static uint16_t rx_enc_len;
static uint8_t rx_overflow; // true while dropping bytes of an over-length frame, until the next 0x00 resyncs

// Per-motor: last binary sequence number whose CONTROL_COMMAND was
// successfully applied to that motor -- reported in that motor's own
// TELEMETRY frame (last_control_sequence) so the client can confirm
// receipt per motor, not just for the link as a whole.
static uint32_t last_control_sequence[MOTOR_COUNT];

static uint32_t telemetry_cycle_sequence; // shared across all MOTOR_COUNT frames of one TELEMETRY cycle
static uint16_t telemetry_period_ms; // 0 = disabled (default)
static uint32_t telemetry_last_sent_ms;


// ---------------------------------------------------------------------
// little-endian pack/unpack -- never a raw struct memcpy, see binproto.h
// ---------------------------------------------------------------------

static void put_u8(uint8_t **p, uint8_t v)   { *(*p)++ = v; }
static void put_u16(uint8_t **p, uint16_t v) { *(*p)++ = (uint8_t)v; *(*p)++ = (uint8_t)(v >> 8); }
static void put_u32(uint8_t **p, uint32_t v)
{
    *(*p)++ = (uint8_t)v;
    *(*p)++ = (uint8_t)(v >> 8);
    *(*p)++ = (uint8_t)(v >> 16);
    *(*p)++ = (uint8_t)(v >> 24);
}
static void put_f32(uint8_t **p, float v) { uint32_t u; memcpy(&u, &v, 4); put_u32(p, u); }

static uint8_t get_u8(const uint8_t **p) { return *(*p)++; }
static uint16_t get_u16(const uint8_t **p)
{
    uint16_t v = (uint16_t)((*p)[0] | ((uint16_t)(*p)[1] << 8));
    *p += 2;
    return v;
}
static uint32_t get_u32(const uint8_t **p)
{
    uint32_t v = (uint32_t)(*p)[0] | ((uint32_t)(*p)[1] << 8) | ((uint32_t)(*p)[2] << 16) | ((uint32_t)(*p)[3] << 24);
    *p += 4;
    return v;
}
static float get_f32(const uint8_t **p) { uint32_t u = get_u32(p); float v; memcpy(&v, &u, 4); return v; }


// ---------------------------------------------------------------------
// mode
// ---------------------------------------------------------------------

uint8_t binproto_active(void) { return mode_binary; }

void binproto_enter_binary_mode(void)
{
    mode_binary = 1;
    rx_enc_len = 0;
    rx_overflow = 0;
    // A silent binary session (USB unplugged, ROS2 node crashed) must not
    // be able to arm any motor's output stage with host_timeout_ms == 0
    // and then hold its last command forever with nothing watching -- see
    // control_require_host_watchdog()'s comment. ASCII bench testing is
    // still allowed timeout=0 (see binproto_force_ascii_mode() below).
    control_require_host_watchdog(true);
}

void binproto_force_ascii_mode(void)
{
    mode_binary = 0;
    rx_enc_len = 0;
    rx_overflow = 0;
    telemetry_period_ms = 0; // don't keep streaming into what may now be a human's terminal
    control_require_host_watchdog(false);
}


// ---------------------------------------------------------------------
// param_id -> wire datatype table
// ---------------------------------------------------------------------
// Deliberately separate from shell.c's FIELDS[] table (which maps
// human-readable names to a richer display-oriented type, e.g. enum name
// lookup) -- this one only needs numeric param_id -> wire size for
// READ_PARAM/WRITE_PARAM, keyed by ID rather than name. Both ultimately
// derive from params.h's own field comments; if they ever drift, params.h
// is the source of truth to reconcile against.

typedef enum
{
    DT_U8 = 0,
    DT_U16 = 1,
    DT_U32 = 2,
    DT_F32 = 3,
    DT_I8 = 4,
    DT_TRIGGER = 5, // WO trigger register (PARAM_CLEAR_FAULT etc) -- 0-byte value
    DT_UNKNOWN = 0xFF,
} datatype_t;

typedef struct
{
    reg_id_t id;
    datatype_t type;
} param_type_entry_t;

static const param_type_entry_t PARAM_TYPES[] =
{
    {PARAM_DEVICE_ID, DT_U32},
    {PARAM_FW_VERSION, DT_U16},
    {PARAM_PROTOCOL_VERSION, DT_U16},
    {PARAM_MOTOR_COUNT, DT_U8},

    {PARAM_ENABLE, DT_U8},
    {PARAM_CONTROL_SOURCE, DT_U8},
    {PARAM_PROFILE_ENABLE, DT_U8},
    {PARAM_OVERRIDE_SOURCE, DT_U8},
    {PARAM_OVERRIDE_ENABLE, DT_U8},
    {PARAM_TRANSITION_BLEND_MS, DT_U16},

    {PARAM_TARGET_POSITION, DT_F32},
    {PARAM_TARGET_VELOCITY, DT_F32},
    {PARAM_TARGET_CURRENT, DT_F32},
    {PARAM_CURRENT_FF, DT_F32},
    {PARAM_VELOCITY_FF, DT_F32},

    {PARAM_POS_KP, DT_F32},
    {PARAM_POS_KI, DT_F32},
    {PARAM_POS_KD, DT_F32},
    {PARAM_POS_INTEGRAL_LIMIT, DT_F32},

    {PARAM_VEL_KP, DT_F32},
    {PARAM_VEL_KI, DT_F32},
    {PARAM_VEL_KD, DT_F32},
    {PARAM_VEL_INTEGRAL_LIMIT, DT_F32},

    {PARAM_PROFILE_VEL_MAX, DT_F32},
    {PARAM_PROFILE_ACC_MAX, DT_F32},
    {PARAM_PROFILE_DEC_MAX, DT_F32},

    {PARAM_CURRENT_LIMIT, DT_F32},
    {PARAM_VELOCITY_LIMIT, DT_F32},
    {PARAM_POSITION_MIN, DT_F32},
    {PARAM_POSITION_MAX, DT_F32},
    {PARAM_CAN_TIMEOUT_MS, DT_U16},
    {PARAM_HOST_TIMEOUT_MS, DT_U16},
    {PARAM_HOST_TIMEOUT_ACTION, DT_U8},
    {PARAM_MAX_TEMPERATURE, DT_F32},
    {PARAM_MIN_BUS_VOLTAGE, DT_F32},
    {PARAM_MAX_BUS_VOLTAGE, DT_F32},

    {PARAM_ACTUAL_POSITION, DT_F32},
    {PARAM_ACTUAL_VELOCITY, DT_F32},
    {PARAM_ACTUAL_CURRENT, DT_F32},
    {PARAM_ACTUAL_DUTY, DT_F32},
    {PARAM_ACTUAL_VOLTAGE, DT_F32},
    {PARAM_ACTUAL_TEMPERATURE, DT_F32},
    {PARAM_VESC_LAST_RX_MS, DT_U32},
    {PARAM_VESC_ONLINE, DT_U8},
    {PARAM_VELOCITY_FEEDBACK_VALID, DT_U8},
    {PARAM_POSITION_FEEDBACK_VALID, DT_U8},
    {PARAM_TEMPERATURE_VALID, DT_U8},
    {PARAM_VOLTAGE_VALID, DT_U8},
    {PARAM_CAN_TX_CURRENT_OVERWRITE_COUNT, DT_U32},

    {PARAM_STATE, DT_U8},
    {PARAM_FAULT_ACTIVE, DT_U32},
    {PARAM_FAULT_LATCHED, DT_U32},
    {PARAM_ACTIVE_SOURCE, DT_U8},
    {PARAM_PROFILE_ACTIVE, DT_U8},
    {PARAM_EFFECTIVE_POSITION_REF, DT_F32},
    {PARAM_EFFECTIVE_VELOCITY_REF, DT_F32},
    {PARAM_EFFECTIVE_CURRENT_CMD, DT_F32},
    {PARAM_CONTROL_OVERRUN_COUNT, DT_U32},
    {PARAM_CONTROL_DT_MAX_US, DT_U32},
    {PARAM_SYSTEM_FAULT_ACTIVE, DT_U32},
    {PARAM_SYSTEM_FAULT_LATCHED, DT_U32},
    {PARAM_CLEAR_FAULT, DT_TRIGGER},

    {PARAM_VESC_CAN_ID, DT_U8},
    {PARAM_CAN_BITRATE, DT_U8},
    {PARAM_GEAR_RATIO, DT_F32},
    {PARAM_MOTOR_DIRECTION, DT_I8},
    {PARAM_POSITION_OFFSET, DT_F32},
    {PARAM_POLE_PAIRS, DT_U8},
    {PARAM_ANGLE_SPAN_MOTOR_REV, DT_U8},
    {PARAM_MOTOR_PROFILE, DT_U8},
    {PARAM_ZERO_POSITION, DT_TRIGGER},

    {PARAM_SAVE, DT_TRIGGER},
    {PARAM_LOAD_DEFAULTS, DT_TRIGGER},
    {PARAM_FACTORY_RESET, DT_TRIGGER},

    {PARAM_CDC_TX_ENQUEUE_COUNT, DT_U32},
    {PARAM_CDC_TX_DROP_COUNT, DT_U32},
    {PARAM_CDC_TX_BUSY_COUNT, DT_U32},
    {PARAM_CDC_TX_BYTES_PENDING, DT_U32},
    {PARAM_CDC_TX_HEAD, DT_U32},
    {PARAM_CDC_TX_TAIL, DT_U32},
};
#define NUM_PARAM_TYPES (sizeof(PARAM_TYPES) / sizeof(PARAM_TYPES[0]))

static datatype_t datatype_of(reg_id_t id)
{
    for (uint32_t i = 0; i < NUM_PARAM_TYPES; i++)
        if (PARAM_TYPES[i].id == id)
            return PARAM_TYPES[i].type;
    return DT_UNKNOWN;
}

static uint32_t datatype_size(datatype_t t)
{
    switch (t) {
    case DT_U8: case DT_I8: return 1;
    case DT_U16: return 2;
    case DT_U32: case DT_F32: return 4;
    case DT_TRIGGER: default: return 0;
    }
}


// ---------------------------------------------------------------------
// TX
// ---------------------------------------------------------------------

static void send_frame(uint8_t type, uint16_t seq, const uint8_t *payload, uint16_t paylen)
{
    uint8_t raw[RAW_MAX];
    uint8_t *p = raw;
    put_u8(&p, BINPROTO_VERSION);
    put_u8(&p, type);
    put_u16(&p, seq);
    put_u16(&p, paylen);
    if (paylen > 0)
        memcpy(p, payload, paylen);
    p += paylen;

    uint16_t crc = crc16_ccitt_false(raw, (size_t)(HEADER_LEN + paylen));
    put_u16(&p, crc);
    uint16_t raw_len = (uint16_t)(HEADER_LEN + paylen + 2);

    uint8_t enc[ENC_MAX];
    size_t enc_len = cobs_encode(raw, raw_len, enc);
    enc[enc_len] = 0x00;
    cdc_transmit(enc, (uint16_t)(enc_len + 1));
}

static void send_ack(uint16_t seq)
{
    send_frame(BINPROTO_MSG_ACK, seq, NULL, 0);
}

static void send_error(uint16_t seq, uint8_t code)
{
    uint8_t buf[1] = {code};
    send_frame(BINPROTO_MSG_ERROR, seq, buf, 1);
}


// ---------------------------------------------------------------------
// request handlers
// ---------------------------------------------------------------------

static void handle_get_info(uint16_t seq)
{
    uint16_t fw_version = 0, reg_ver = 0;
    uint32_t device_id = 0;
    params_read(PARAM_FW_VERSION, 0, &fw_version, sizeof(fw_version));
    params_read(PARAM_PROTOCOL_VERSION, 0, &reg_ver, sizeof(reg_ver));
    params_read(PARAM_DEVICE_ID, 0, &device_id, sizeof(device_id));

    uint8_t buf[1 + 2 + 2 + 4];
    uint8_t *p = buf;
    put_u8(&p, BINPROTO_VERSION);
    put_u16(&p, fw_version);
    put_u16(&p, reg_ver);
    put_u32(&p, device_id);
    send_frame(BINPROTO_MSG_GET_INFO, seq, buf, (uint16_t)sizeof(buf));
}

static void handle_read_param(uint16_t seq, const uint8_t *payload, uint16_t paylen)
{
    if (paylen != 3) { send_error(seq, BINPROTO_ERR_MALFORMED); return; }
    const uint8_t *p = payload;
    uint8_t motor_index = get_u8(&p);
    uint16_t id = get_u16(&p);

    datatype_t dt = datatype_of((reg_id_t)id);
    if (dt == DT_UNKNOWN) { send_error(seq, PARAM_ERR_UNKNOWN_ID); return; }

    uint8_t raw[4] = {0};
    param_result_t r = params_read((reg_id_t)id, motor_index, raw, datatype_size(dt));
    if (r != PARAM_OK) { send_error(seq, (uint8_t)r); return; }

    uint8_t buf[1 + 2 + 1 + 4];
    uint8_t *out = buf;
    put_u8(&out, motor_index);
    put_u16(&out, id);
    put_u8(&out, (uint8_t)dt);
    memcpy(out, raw, datatype_size(dt));
    send_frame(BINPROTO_MSG_PARAM_RESPONSE, seq, buf, (uint16_t)(1 + 2 + 1 + datatype_size(dt)));
}

static void handle_write_param(uint16_t seq, const uint8_t *payload, uint16_t paylen)
{
    if (paylen < 4) { send_error(seq, BINPROTO_ERR_MALFORMED); return; }
    const uint8_t *p = payload;
    uint8_t motor_index = get_u8(&p);
    uint16_t id = get_u16(&p);
    uint8_t client_dt = get_u8(&p);
    uint16_t value_len = (uint16_t)(paylen - 4);

    datatype_t dt = datatype_of((reg_id_t)id);
    if (dt == DT_UNKNOWN) { send_error(seq, PARAM_ERR_UNKNOWN_ID); return; }
    if (client_dt != (uint8_t)dt || value_len != datatype_size(dt)) { send_error(seq, PARAM_ERR_TYPE); return; }

    param_result_t r = params_write((reg_id_t)id, motor_index, (value_len > 0) ? p : NULL, value_len);
    if (r != PARAM_OK) { send_error(seq, (uint8_t)r); return; }
    send_ack(seq);
}

static void handle_control_command(uint16_t seq, const uint8_t *payload, uint16_t paylen)
{
    if (paylen != 25) { send_error(seq, BINPROTO_ERR_MALFORMED); return; }

    const uint8_t *p = payload;
    uint8_t motor_index = get_u8(&p);
    if (motor_index >= MOTOR_COUNT) { send_error(seq, PARAM_ERR_RANGE); return; }

    control_command_t cmd;
    cmd.control_source = get_u8(&p);
    cmd.profile_enable = get_u8(&p);
    cmd.override_source = get_u8(&p);
    cmd.override_enable = get_u8(&p);
    cmd.target_position = get_f32(&p);
    cmd.target_velocity = get_f32(&p);
    cmd.target_current = get_f32(&p);
    cmd.velocity_ff = get_f32(&p);
    cmd.current_ff = get_f32(&p);

    uint32_t now_ms = HAL_GetTick();
    param_result_t r = control_apply_command(motor_index, &cmd, now_ms);
    if (r != PARAM_OK) { send_error(seq, (uint8_t)r); return; }

    last_control_sequence[motor_index] = seq;
    control_notify_host_rx(now_ms); // system-wide link liveness -- see params.h "Two-tier host watchdog"
    // No ACK on success -- see binproto.h's CONTROL_COMMAND comment.
    // TELEMETRY's per-motor last_control_sequence is the confirmation channel.
}

static void handle_set_telemetry_rate(uint16_t seq, const uint8_t *payload, uint16_t paylen)
{
    if (paylen != 2) { send_error(seq, BINPROTO_ERR_MALFORMED); return; }
    const uint8_t *p = payload;
    telemetry_period_ms = get_u16(&p);
    telemetry_last_sent_ms = HAL_GetTick(); // don't immediately burst-send on the next tick
    send_ack(seq);
}

static void dispatch(uint8_t type, uint16_t seq, const uint8_t *payload, uint16_t paylen)
{
    switch (type) {
    case BINPROTO_MSG_GET_INFO:
        handle_get_info(seq);
        break;
    case BINPROTO_MSG_HEARTBEAT:
        control_notify_host_rx(HAL_GetTick());
        send_ack(seq);
        break;
    case BINPROTO_MSG_READ_PARAM:
        handle_read_param(seq, payload, paylen);
        break;
    case BINPROTO_MSG_WRITE_PARAM:
        handle_write_param(seq, payload, paylen);
        break;
    case BINPROTO_MSG_CONTROL_COMMAND:
        handle_control_command(seq, payload, paylen);
        break;
    case BINPROTO_MSG_SET_TELEMETRY_RATE:
        handle_set_telemetry_rate(seq, payload, paylen);
        break;
    default:
        send_error(seq, BINPROTO_ERR_UNKNOWN_TYPE);
        break;
    }
}


// ---------------------------------------------------------------------
// RX / framing
// ---------------------------------------------------------------------

static void handle_encoded_frame(const uint8_t *enc, size_t enc_len)
{
    uint8_t dec[RAW_MAX];
    size_t n = cobs_decode(enc, enc_len, dec, sizeof(dec));
    if (n == (size_t)-1 || n < (size_t)(HEADER_LEN + 2))
        return; // malformed/truncated -- nothing here can be trusted, drop silently

    const uint8_t *p = dec;
    uint8_t version = get_u8(&p);
    uint8_t type = get_u8(&p);
    uint16_t seq = get_u16(&p);
    uint16_t paylen = get_u16(&p);

    if (n != (size_t)(HEADER_LEN + paylen + 2))
        return; // declared payload_length doesn't match actual frame size -- drop silently

    uint16_t crc_calc = crc16_ccitt_false(dec, (size_t)(HEADER_LEN + paylen));
    uint16_t crc_recv = (uint16_t)(dec[HEADER_LEN + paylen] | ((uint16_t)dec[HEADER_LEN + paylen + 1] << 8));
    if (crc_calc != crc_recv)
        return; // CRC covers the sequence field too -- an untrusted sequence can't be replied to, drop silently

    if (version != BINPROTO_VERSION) { send_error(seq, BINPROTO_ERR_VERSION_MISMATCH); return; }

    dispatch(type, seq, dec + HEADER_LEN, paylen);
}

void binproto_feed_byte(uint8_t b)
{
    if (b == 0x00) {
        if (!rx_overflow && rx_enc_len > 0)
            handle_encoded_frame(rx_enc, rx_enc_len);
        rx_enc_len = 0;
        rx_overflow = 0;
        return;
    }

    if (rx_enc_len < sizeof(rx_enc)) {
        rx_enc[rx_enc_len++] = b;
    } else {
        rx_overflow = 1; // drop the rest of this over-length frame until the next 0x00 resyncs
    }
}

void binproto_tick(uint32_t now_ms)
{
    if (!mode_binary)
        return;
    if (telemetry_period_ms == 0)
        return;
    if ((uint32_t)(now_ms - telemetry_last_sent_ms) < telemetry_period_ms)
        return;

    telemetry_last_sent_ms = now_ms;
    telemetry_cycle_sequence++;

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        const motor_feedback_t *mf = vesc_can_feedback(i);
        const control_status_t *cs = control_status(i);

        uint8_t flags = (uint8_t)(
            (mf->vesc_alive              ? 0x01 : 0) |
            (mf->velocity_feedback_valid ? 0x02 : 0) |
            (mf->position_feedback_valid ? 0x04 : 0) |
            (mf->temperature_valid       ? 0x08 : 0) |
            (mf->voltage_valid           ? 0x10 : 0));

        uint8_t buf[4+4+1+4 + 1+1+1 + 4*9 + 4+4];
        uint8_t *p = buf;
        put_u32(&p, now_ms);
        put_u32(&p, telemetry_cycle_sequence);
        put_u8(&p, i);
        put_u32(&p, last_control_sequence[i]);
        put_u8(&p, cs->state);
        put_u8(&p, cs->active_source);
        put_u8(&p, flags);
        put_f32(&p, mf->actual_position);
        put_f32(&p, mf->actual_velocity);
        put_f32(&p, mf->actual_current);
        put_f32(&p, mf->actual_duty);
        put_f32(&p, mf->actual_voltage);
        put_f32(&p, mf->actual_temperature);
        put_f32(&p, cs->effective_position_ref);
        put_f32(&p, cs->effective_velocity_ref);
        put_f32(&p, cs->effective_current_cmd);
        put_u32(&p, cs->fault_active);
        put_u32(&p, cs->fault_latched);

        // Sequence each motor's own frame independently so a dropped frame
        // for one motor doesn't desync another's -- telemetry_cycle_sequence
        // (in the payload, not here) is what ties a cycle's frames together.
        send_frame(BINPROTO_MSG_TELEMETRY, (uint16_t)((telemetry_cycle_sequence * MOTOR_COUNT + i) & 0xFFFFu),
                   buf, (uint16_t)(p - buf));
    }
}
