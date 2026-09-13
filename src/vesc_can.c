//
// vesc_can: VESC CAN protocol encode/decode for up to MOTOR_COUNT VESCs on
// one shared bus. Owns motor_feedback_t[MOTOR_COUNT] -- see inc/vesc_can.h.
// This is the only file in the firmware that knows about VESC CAN_PACKET_*
// IDs or their byte layouts; control.c and params.c never touch a raw CAN
// frame.
//
// ---------------------------------------------------------------------
// Scope for this pass (minimal, per team decision): SET_CURRENT TX, decode
// of STATUS (erpm/current/duty) and STATUS_4 (pid_pos for position,
// temperatures). STATUS_5 is decoded only for its input-voltage field plus
// a debug-only tachometer snapshot. motor_profile_t is currently just
// storage (params.h PARAM_MOTOR_PROFILE) -- it doesn't derive/lock
// pole_pairs/gear_ratio/angle_span_motor_rev or current limits yet, and
// there's no VESC-side identity cross-check (temperature sensor presence,
// Information Storage). That's future work, parked until this minimal
// G431<->VESC pipeline is proven end to end on real hardware.
//
// ---------------------------------------------------------------------
// Why STATUS_4 pid_pos + p_pid_ang_div instead of STATUS_5's tachometer
// ---------------------------------------------------------------------
// STATUS_5's tachometer looked like the obvious multi-turn position source
// (an accumulating counter, no 360deg wrap), but VESC's FOC loop derives it
// by quantizing the electrical angle into 6 steps of 60deg each and
// counting steps -- it's not a continuous angle, and in practice doesn't
// track reliably enough to be the primary position feedback (confirmed by
// hands-on testing on this project). It's still decoded below but only
// folded into actual_voltage's sibling field (debug), never into
// actual_position.
//
// VESC firmware already has a mechanism for exactly what this project
// wanted from a custom "angle_div": p_pid_ang_div (a real, existing VESC
// config parameter, default 1, set via VESC Tool -- out of band, this
// firmware cannot read or write it over CAN). When p_pid_ang_div != 1, the
// PID position reported in STATUS_4 (pid_pos, still 0-360deg, still wraps
// every cycle -- that part of prior testing is unchanged) represents
// p_pid_ang_div electrical revolutions per wrap instead of 1. In mechanical
// motor revolutions that's p_pid_ang_div / pole_pairs per wrap, which this
// firmware calls angle_span_motor_rev (params.h PARAM_ANGLE_SPAN_MOTOR_REV
// -- it MUST be kept equal to p_pid_ang_div/pole_pairs; nothing here can
// verify that automatically).
//
// unwrap_pid_pos() below reconstructs multi-turn position from that
// wrapping angle: a plain +-180deg unwrap breaks if more than half a wrap
// happens between two STATUS_4 frames (easy at 1kHz with M2006/M3508's
// electrical speeds -- see the angle_span_motor_rev sizing discussion), so
// the unwrap is assisted by the most recently known STATUS erpm: predict
// the expected delta from erpm*dt, then pick whichever +-360deg candidate
// matches that prediction. This tolerates several dropped STATUS_4 frames
// in a row, not just <=1.
//
// ---------------------------------------------------------------------
// RX dispatch: sender_id -> motor_index
// ---------------------------------------------------------------------
// See vesc_can.h's header comment. motor_index_from_vesc_id() below is a
// plain linear scan over motor_config[]::vesc_can_id -- trivial at
// MOTOR_COUNT==4.
//
// ---------------------------------------------------------------------
// !!! VERIFY BEFORE TRUSTING ANY MOTION FROM THIS FILE !!!
// ---------------------------------------------------------------------
// The byte layouts and scale factors below (SET_CURRENT id=1 x1000 int32
// BE, STATUS id=9, STATUS_4 id=16, STATUS_5 id=27, pid_pos scale 50) match
// public VESC CAN protocol documentation. p_pid_ang_div's existence and
// formula are as documented/discussed for this project; NEITHER has been
// cross-checked against this project's own captured CAN traffic yet.
// Before relying on actual_position for real PID gains: (1) set
// p_pid_ang_div on the VESC via VESC Tool to pole_pairs*angle_span_motor_rev
// and set PARAM_ANGLE_SPAN_MOTOR_REV to match, (2) verify against captured
// traffic (tools/record_can.sh, tools/bag_analyze.py, or the
// atcan_2026-09-11_*/ logs already in this repo).
//

#include <math.h>
#include <stdbool.h>

#include "stm32g4xx_hal.h"
#include "can.h"
#include "slcan.h" // hal_dlc_code_to_bytes()
#include "params.h"
#include "vesc_can.h"

#define VESC_TWO_PI 6.283185307179586f

// If the ERPM-assisted unwrap correction's residual (how far the chosen
// +-360deg*n candidate ends up from the erpm-predicted delta) exceeds this,
// the correction is not trusted: rather than silently accept a possibly
// wrong multi-turn count, that frame's decoded angle is folded into
// unwrapped_deg anyway (keeps the accumulator moving so a future good
// sample doesn't have to bridge an even bigger gap) but
// position_feedback_valid is held false until a subsequent sample is
// trustworthy again. Not yet empirically tuned -- see the file header's
// "VERIFY BEFORE TRUSTING" note.
#define UNWRAP_RESIDUAL_LIMIT_DEG 90.0f

// Subset of VESC's CAN_PACKET_ID enum actually used here. Prefixed to avoid
// any future collision with can.h's own CAN_* names.
enum
{
    VESC_CAN_PACKET_SET_CURRENT = 1,
    VESC_CAN_PACKET_STATUS      = 9,
    VESC_CAN_PACKET_STATUS_4    = 16,
    VESC_CAN_PACKET_STATUS_5    = 27,
};

// Raw, motor/electrical-domain values decoded straight off the wire, before
// gear_ratio/motor_direction/position_offset conversion. STATUS/STATUS_4/
// STATUS_5 arrive as separate, unsynchronized CAN messages (and may run at
// different rates on the VESC -- see vesc_can.h) -- each group tracks its
// own "seen at least once" flag and its own last-rx timestamp so
// rebuild_feedback() can judge freshness per group, not globally. One
// instance per motor (raw[MOTOR_COUNT]).
typedef struct
{
    float erpm;          // electrical RPM, from STATUS
    float current;       // A, from STATUS
    float duty;          // -1..1, from STATUS
    bool  has_status;
    uint32_t status_rx_ms;

    float temp_fet;      // degC, from STATUS_4
    float temp_motor;    // degC, from STATUS_4
    bool  has_status_4;
    uint32_t status4_rx_ms; // also doubles as unwrap_pid_pos()'s previous-sample timestamp

    int32_t tachometer;  // raw counts, from STATUS_5 -- debug/sanity only, NOT used for actual_position (see file header)
    float   voltage;     // V, from STATUS_5
    bool    has_status_5;
    uint32_t status5_rx_ms;

    // pid_pos unwrap state (see unwrap_pid_pos())
    float pid_pos_deg_prev;   // last decoded wrapped angle, degrees [0,360)
    float unwrapped_deg;      // accumulated unwrapped angle, degrees, monotonic in the direction of travel
    bool  unwrap_initialized;
    bool  unwrap_trustworthy; // false when the last correction's residual exceeded UNWRAP_RESIDUAL_LIMIT_DEG
} vesc_raw_t;

static vesc_raw_t raw[MOTOR_COUNT];
static motor_feedback_t feedback[MOTOR_COUNT];

// Hardware Tx FIFO/Queue depth. This board's HAL_FDCAN driver is a trimmed
// variant that doesn't expose TxFifoQueueElmtsNbr as a configurable
// FDCAN_InitTypeDef field the way upstream ST HAL does -- the depth is
// hardcoded inside stm32g4xx_hal_fdcan.c as SRAMCAN_TFQ_NBR (currently 3),
// not visible from outside that file. Mirrored here so vesc_can_tx_service()
// can tell "the Tx FIFO/Queue is completely empty" (HAL_FDCAN_GetTxFifoFreeLevel()
// back to full) apart from "just has some room" -- see that function.
#define FDCAN_TX_FIFO_DEPTH 3

// Per-motor single-slot "latest command wins" mailboxes for SET_CURRENT --
// see vesc_can_set_current() and vesc_can_tx_service(). Deliberately NOT
// queues: at most one not-yet-confirmed-sent current command may exist per
// motor at any time. round_robin_next is where vesc_can_tx_service() will
// look first for a dirty mailbox to send, and always advances past
// whichever motor it actually sends (or would have sent) -- see that
// function for why this, not main.c's separate ~500Hz/motor rate gate, is
// what spreads the motors' frames out in time.
static float current_mailbox_amps[MOTOR_COUNT];
static bool  current_mailbox_dirty[MOTOR_COUNT];
static bool  current_frame_pending;  // at most one of our frames may be in the HW Tx FIFO/Queue right now
static uint8_t round_robin_next;

// Lifetime counters (not reset by vesc_can_reset_feedback() -- unlike raw/
// feedback, this isn't decode state that goes stale when e.g. pole_pairs
// changes; only vesc_can_init() clears them). Count mailbox overwrites,
// i.e. a computed current value that was superseded before it ever reached
// the bus -- NOT a CAN hardware transmit failure count, see
// PARAM_CAN_TX_CURRENT_OVERWRITE_COUNT.
static uint32_t current_overwrite_count[MOTOR_COUNT];


static inline int32_t get_i32_be(const uint8_t *b)
{
    return (int32_t)(((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | (uint32_t)b[3]);
}

static inline int16_t get_i16_be(const uint8_t *b)
{
    return (int16_t)(((uint16_t)b[0] << 8) | (uint16_t)b[1]);
}

static inline void put_i32_be(uint8_t *b, int32_t v)
{
    b[0] = (uint8_t)((uint32_t)v >> 24);
    b[1] = (uint8_t)((uint32_t)v >> 16);
    b[2] = (uint8_t)((uint32_t)v >> 8);
    b[3] = (uint8_t)((uint32_t)v);
}

// Linear scan, sender_id -> motor_index. Trivial at MOTOR_COUNT==4. Returns
// -1 if no configured motor claims this ID. If more than one motor shares
// the same (non-0xFF) vesc_can_id -- an accepted-at-write-time, enable-time
// -rejected misconfiguration, see params.h's PARAM_VESC_CAN_ID comment --
// this returns the FIRST (lowest-index) match, so only that motor ever
// sees frames from the shared ID.
static int8_t motor_index_from_vesc_id(uint8_t id)
{
    if (id == 0xFF)
        return -1; // 0xFF is "unconfigured", never a real sender
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
        if (params_motor(i)->vesc_can_id == id)
            return (int8_t)i;
    return -1;
}


void vesc_can_init(void)
{
    can_set_bitrate((enum can_bitrate)params_bus()->can_bitrate);
    can_enable();

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        raw[i] = (vesc_raw_t){0};
        feedback[i] = (motor_feedback_t){0};
        current_overwrite_count[i] = 0;
        current_mailbox_dirty[i] = false;
    }
    current_frame_pending = false;
    round_robin_next = 0;
}

void vesc_can_reset_feedback(uint8_t motor_index)
{
    raw[motor_index] = (vesc_raw_t){0};
    feedback[motor_index] = (motor_feedback_t){0};
}

void vesc_can_set_current(uint8_t motor_index, float amps)
{
    // An unconfigured motor (vesc_can_id == 0xFF) has no real VESC to send
    // to -- main.c's rate gate calls this for every motor_index every
    // tick regardless of which ones are actually configured, so without
    // this guard an unconfigured slot would have this firmware
    // continuously transmit SET_CURRENT frames to CAN ID 0xFF for nothing,
    // wasting bus arbitration/bandwidth that matters once more VESCs share
    // this bus (see params.h's 4-VESC bus-load budget discussion). No-op,
    // not even a mailbox write -- there's nothing to send later either.
    if (params_motor(motor_index)->vesc_can_id == 0xFF)
        return;

    // Does NOT touch CAN hardware -- only updates this motor's mailbox.
    // Actual TX happens in vesc_can_tx_service(), called every main loop
    // iteration (not just once per control tick) so a freshly-emptied Tx
    // FIFO/Queue gets serviced promptly. If main.c's rate gate calls this
    // for the same motor again before the previous value went out, only
    // the last value written here is what vesc_can_tx_service() will ever
    // find waiting for that motor -- never a replay of intermediate values.
    current_mailbox_amps[motor_index] = amps;
    if (current_mailbox_dirty[motor_index])
        current_overwrite_count[motor_index]++; // a previous not-yet-sent value for this motor just got superseded
    current_mailbox_dirty[motor_index] = true;
}

static void submit_current_frame(uint8_t motor_index)
{
    const motor_config_t *mc = params_motor(motor_index);

    // motor_direction is applied on RX (actual_velocity/current/position)
    // to map the motor's electrical convention onto the output-axis sign
    // convention -- it must be applied here too, symmetrically, or a
    // motor_direction == -1 motor would see actual_velocity/position move
    // the "wrong" way relative to the commanded current, which a running
    // velocity/position PID would read as negative feedback and fight by
    // increasing output -- i.e. positive feedback into the real motor.
    float motor_amps = current_mailbox_amps[motor_index] * (float)mc->motor_direction;

    uint8_t data[4];
    put_i32_be(data, (int32_t)(motor_amps * 1000.0f)); // mA, matches VESC's CAN_PACKET_SET_CURRENT scale

    FDCAN_TxHeaderTypeDef h =
    {
        .Identifier = ((uint32_t)VESC_CAN_PACKET_SET_CURRENT << 8) | mc->vesc_can_id,
        .IdType = FDCAN_EXTENDED_ID,
        .TxFrameType = FDCAN_DATA_FRAME,
        .FDFormat = FDCAN_CLASSIC_CAN,
        .BitRateSwitch = FDCAN_BRS_OFF,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
        .MessageMarker = 0,
        .DataLength = (uint32_t)4 << 16, // 4-byte classic CAN DLC (see atcan.c's dlc_to_hal())
    };

    // NOT can_tx() -- deliberately bypasses that software FIFO queue; see
    // vesc_can_set_current()'s header-comment for why.
    if (HAL_FDCAN_AddMessageToTxFifoQ(can_gethandle(), &h, data) == HAL_OK) {
        current_frame_pending = true;
        current_mailbox_dirty[motor_index] = false;
    }
    // else: HW unexpectedly rejected it (shouldn't happen -- we only get
    // here when the FIFO/Queue was just confirmed empty) -- mailbox stays
    // dirty, retried next call.
}

void vesc_can_tx_service(void)
{
    if (current_frame_pending) {
        // At most one of our frames may be in flight at a time (see
        // FDCAN_TX_FIFO_DEPTH) -- has it left the FIFO/Queue yet?
        if (HAL_FDCAN_GetTxFifoFreeLevel(can_gethandle()) < FDCAN_TX_FIFO_DEPTH)
            return; // still in flight -- try again next call
        current_frame_pending = false;
    }

    // Round-robin: starting from round_robin_next, find the next motor
    // with a dirty mailbox and send it, so 4 motors all being updated
    // doesn't burst all 4 frames back-to-back -- see vesc_can.h. Always
    // advance round_robin_next past whatever we just checked, even if
    // nothing was dirty, so the starting point keeps rotating instead of
    // favoring motor0 whenever mailboxes go idle and dirty again later.
    for (uint8_t n = 0; n < MOTOR_COUNT; n++) {
        uint8_t i = (uint8_t)((round_robin_next + n) % MOTOR_COUNT);
        if (current_mailbox_dirty[i]) {
            round_robin_next = (uint8_t)((i + 1) % MOTOR_COUNT);
            submit_current_frame(i);
            return;
        }
    }
    round_robin_next = (uint8_t)((round_robin_next + 1) % MOTOR_COUNT);
}

uint32_t vesc_can_current_overwrite_count(uint8_t motor_index)
{
    return current_overwrite_count[motor_index];
}

// Reconstructs multi-turn position from STATUS_4's wrapping pid_pos
// (new_deg, degrees [0,360)) for one motor. Called once per STATUS_4
// frame, in CAN-RX order -- NOT once per control tick, since pid_pos only
// changes when a new frame arrives. See the file header comment for the
// algorithm.
static void unwrap_pid_pos(uint8_t motor_index, float new_deg, uint32_t now_ms)
{
    vesc_raw_t *r = &raw[motor_index];
    const motor_config_t *mc = params_motor(motor_index);

    if (!r->unwrap_initialized) {
        // First sample: nothing to unwrap yet, just establish the
        // baseline. angle_span_motor_rev/pole_pairs conversion in
        // rebuild_feedback() already gives a correct (if not yet
        // multi-turn-tested) position from this alone.
        r->pid_pos_deg_prev = new_deg;
        r->unwrapped_deg = new_deg;
        r->unwrap_initialized = true;
        r->unwrap_trustworthy = true;
        r->status4_rx_ms = now_ms;
        return;
    }

    float dt = (float)(now_ms - r->status4_rx_ms) * 0.001f;
    if (dt <= 0.0f)
        dt = 1.0e-3f; // guard against a 0ms or (wrapped-counter) negative gap

    // The plain +-180deg wrapped delta (n=0 candidate) is what a normal
    // sample resolves to on its own -- angle_span_motor_rev is sized so
    // that STATUS_4's usual rate keeps per-sample motion well under a half
    // wrap (see params.h PARAM_ANGLE_SPAN_MOTOR_REV). ERPM-assisted
    // correction below only matters as a rescue for outliers (dropped
    // frames, a burst of latency, etc).
    float raw_delta = new_deg - r->pid_pos_deg_prev;
    while (raw_delta > 180.0f)  raw_delta -= 360.0f;
    while (raw_delta <= -180.0f) raw_delta += 360.0f;

    // Predict the expected delta from the most recently known electrical
    // RPM (STATUS and STATUS_4 are separate, unsynchronized messages, so
    // this erpm may be a sample or two stale -- that's fine for picking
    // between +-360deg candidates, which only needs the right order of
    // magnitude and sign, not precision), then pick the +-360deg*n
    // candidate closest to that prediction.
    float motor_rev_per_s = r->erpm / 60.0f / (float)mc->pole_pairs;
    float predicted_deg = motor_rev_per_s * dt / (float)mc->angle_span_motor_rev * 360.0f;

    float n = roundf((predicted_deg - raw_delta) / 360.0f);
    float corrected_delta = raw_delta + 360.0f * n;

    // If even the best candidate is far from the prediction, the
    // prediction itself (erpm, or dt across a long gap) isn't trustworthy
    // enough to bet a +-360deg correction on -- apply it anyway (keeps the
    // accumulator from falling further behind) but don't let the cascade
    // act on the result until a subsequent sample looks sane again.
    float residual = fabsf(corrected_delta - predicted_deg);
    r->unwrap_trustworthy = residual <= UNWRAP_RESIDUAL_LIMIT_DEG;

    r->unwrapped_deg += corrected_delta;
    r->pid_pos_deg_prev = new_deg;
    r->status4_rx_ms = now_ms;
}

// Recompute one motor's public, output-axis motor_feedback_t from whatever
// raw_* fields have been decoded so far, and the per-group freshness flags.
static void rebuild_feedback(uint8_t motor_index, uint32_t now_ms)
{
    const motor_config_t *mc = params_motor(motor_index);
    vesc_raw_t *r = &raw[motor_index];
    motor_feedback_t *fb = &feedback[motor_index];
    float pole_pairs = (float)mc->pole_pairs;

    if (r->has_status) {
        float motor_rpm = r->erpm / pole_pairs;
        float motor_rad_s = motor_rpm * (VESC_TWO_PI / 60.0f);
        fb->actual_velocity = motor_rad_s / mc->gear_ratio * (float)mc->motor_direction;
        fb->actual_current = r->current * (float)mc->motor_direction;
        fb->actual_duty = r->duty * (float)mc->motor_direction;
    }

    if (r->has_status_4) {
        float motor_rev = r->unwrapped_deg / 360.0f * (float)mc->angle_span_motor_rev;
        float output_rev = motor_rev / mc->gear_ratio;
        fb->actual_position = output_rev * VESC_TWO_PI * (float)mc->motor_direction - mc->position_offset;
        // Report whichever of FET/motor is hotter -- either one crossing
        // max_temperature should trip FAULT_BIT_OVER_TEMP in control.c.
        fb->actual_temperature = fmaxf(r->temp_fet, r->temp_motor);
    }

    if (r->has_status_5) {
        fb->actual_voltage = r->voltage; // tachometer intentionally not used here -- see file header
    }

    uint16_t timeout_ms = mc->can_timeout_ms;
    bool status_fresh  = r->has_status   && (now_ms - r->status_rx_ms)  <= timeout_ms;
    bool status4_fresh = r->has_status_4 && (now_ms - r->status4_rx_ms) <= timeout_ms;
    bool status5_fresh = r->has_status_5 && (now_ms - r->status5_rx_ms) <= timeout_ms;

    // CURRENT and VELOCITY both need STATUS -- actual_current AND
    // actual_velocity both come only from that message, so vesc_alive
    // (below) alone isn't enough to know either is fresh (e.g. only
    // STATUS_4/STATUS_5 arriving would leave actual_current stale while
    // still reporting "alive").
    fb->velocity_feedback_valid = status_fresh;

    // POSITION needs STATUS_4 (for pid_pos itself) AND STATUS (the unwrap
    // uses the latest erpm every sample -- a stale erpm risks picking the
    // wrong +-360deg candidate), AND the unwrap must have an established
    // baseline and not be flagging its last correction as untrustworthy
    // (see unwrap_pid_pos()'s residual check).
    fb->position_feedback_valid =
        status4_fresh && status_fresh && r->unwrap_initialized && r->unwrap_trustworthy;

    fb->temperature_valid = status4_fresh;
    fb->voltage_valid = status5_fresh;

    // Diagnostic only -- "has this VESC said anything recently" -- never
    // used to gate control by itself; see PARAM_VESC_ONLINE's comment.
    fb->vesc_alive = status_fresh || status4_fresh || status5_fresh;
}

void vesc_can_poll(uint32_t now_ms)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    bool got_frame[MOTOR_COUNT] = {0};

    while (is_can_msg_pending(FDCAN_RX_FIFO0)) {
        if (can_rx(&rx_header, rx_data) != HAL_OK)
            break;

        if (rx_header.IdType != FDCAN_EXTENDED_ID || rx_header.FDFormat != FDCAN_CLASSIC_CAN)
            continue;

        uint32_t id = rx_header.Identifier;
        uint8_t sender_id = (uint8_t)(id & 0xFF);
        uint8_t packet_id  = (uint8_t)((id >> 8) & 0xFF);

        int8_t mi = motor_index_from_vesc_id(sender_id);
        if (mi < 0)
            continue; // not addressed to any configured motor
        uint8_t motor_index = (uint8_t)mi;
        vesc_raw_t *r = &raw[motor_index];

        // Reject frames too short for the packet type before touching
        // rx_data beyond what was actually received -- a corrupt/unexpected
        // short frame on the same ID would otherwise read uninitialized
        // buffer contents as real values.
        int8_t dlc_bytes = hal_dlc_code_to_bytes(rx_header.DataLength);

        switch (packet_id) {
        case VESC_CAN_PACKET_STATUS:
            if (dlc_bytes < 8) break;
            r->erpm = (float)get_i32_be(&rx_data[0]);
            r->current = (float)get_i16_be(&rx_data[4]) / 10.0f;
            r->duty = (float)get_i16_be(&rx_data[6]) / 1000.0f;
            r->has_status = true;
            r->status_rx_ms = now_ms;
            got_frame[motor_index] = true;
            break;

        case VESC_CAN_PACKET_STATUS_4: {
            if (dlc_bytes < 8) break;
            r->temp_fet = (float)get_i16_be(&rx_data[0]) / 10.0f;
            r->temp_motor = (float)get_i16_be(&rx_data[2]) / 10.0f;
            // bytes 4..5 (current_in) intentionally not decoded -- not
            // needed by the cascade (actual_current comes from STATUS).
            float pid_pos_deg = (float)get_i16_be(&rx_data[6]) / 50.0f;
            if (pid_pos_deg < 0.0f)
                pid_pos_deg += 360.0f; // defensive normalize; VESC documents this as 0..360 already
            unwrap_pid_pos(motor_index, pid_pos_deg, now_ms); // also updates r->status4_rx_ms
            r->has_status_4 = true;
            got_frame[motor_index] = true;
            break;
        }

        case VESC_CAN_PACKET_STATUS_5:
            if (dlc_bytes < 6) break;
            r->tachometer = get_i32_be(&rx_data[0]); // debug/sanity only -- see file header
            r->voltage = (float)get_i16_be(&rx_data[4]) / 10.0f;
            r->has_status_5 = true;
            r->status5_rx_ms = now_ms;
            got_frame[motor_index] = true;
            break;

        default:
            break;
        }
    }

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if (got_frame[i])
            feedback[i].vesc_last_rx_ms = now_ms;
        rebuild_feedback(i, now_ms);
    }
}

const motor_feedback_t *vesc_can_feedback(uint8_t motor_index)
{
    return &feedback[motor_index];
}
