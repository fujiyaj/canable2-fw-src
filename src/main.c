//
// CANable2 firmware -- VESC motion controller
//
// This board started as a plain AT-CAN-bridge fork (see atcan.c); it's
// being turned into a dedicated motion controller (params.c/control.c/
// vesc_can.c). The two modes are mutually exclusive CAN-RX-FIFO consumers
// -- atcan_feed_byte() can call can_tx() and the old main loop below drained
// FDCAN_RX_FIFO0 itself via can_rx(), while vesc_can_poll() now does both
// of those for the motion controller. Running both in the same build would
// have them race for CAN frames and TX collide, so FIRMWARE_MODE picks one
// at compile time. #define, not an enum, specifically so it can gate an
// #if below -- v0.1 has no runtime switch yet.
#define FW_MODE_MOTION 0
#define FW_MODE_ATCAN  1
#define FIRMWARE_MODE FW_MODE_MOTION

#include "stm32g4xx.h"
#include "printf.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "can.h"
#include "atcan.h"
#include "system.h"
#include "led.h"
#include "params.h"
#include "control.h"
#include "vesc_can.h"
#include "binproto.h"


// 1kHz control tick, generated from the HAL SysTick that's already running
// at 1kHz for HAL_GetTick()/HAL_Delay() (see interrupts.c's SysTick_Handler()
// -> HAL_SYSTICK_IRQHandler() -> this weak callback) -- no separate TIM
// peripheral needed. Deliberately trivial: only a counter increment happens
// in interrupt context. control_step() itself (CAN TX/RX, the full cascade)
// runs from main()'s loop, not here -- see the loop below for why.
static volatile uint32_t control_ticks_pending;

void HAL_SYSTICK_Callback(void)
{
    control_ticks_pending++;
}


int main(void)
{
    // Initialize peripherals
    system_init();
    can_init();
    led_init();
    usb_init();

#if FIRMWARE_MODE == FW_MODE_MOTION
    params_init();
    control_init();
    vesc_can_init(); // also brings up the CAN bus at params_bus()->can_bitrate and enables it
#else
    // AT-frameプロトコルはホスト側の open/bitrate コマンドが無い専用FWなので、
    // ここで固定ビットレートでバスを起動する（atcan.c の ATCAN_BITRATE）。
    atcan_init();
#endif

    // Power-on blink sequence
    led_blue_blink(2);

#if FIRMWARE_MODE == FW_MODE_ATCAN
    // Storage for status and received message buffer
    FDCAN_RxHeaderTypeDef rx_msg_header;
    uint8_t rx_msg_data[64] = {0};
    uint8_t msg_buf[ATCAN_FRAME_MAX];
#endif

    uint32_t last_recover = 0;
#if FIRMWARE_MODE == FW_MODE_MOTION
    uint32_t last_control_ms = HAL_GetTick(); // seeds the first dt as ~0 rather than "since boot"
    uint32_t control_tick_counter = 0; // even/odd drives the ~500Hz/motor SET_CURRENT rate gate -- see below
#endif

    while(1)
    {
        led_process();
        can_process();
        cdc_process_tx(); // always -- flushes any queued cdc_transmit() output regardless of mode

        // bus-off ウォッチドッグ（~20ms毎）: 詰まったら FDCAN を自動再初期化
        if ((uint32_t)(HAL_GetTick() - last_recover) >= 20u)
        {
            last_recover = HAL_GetTick();
            can_check_recover();
        }

#if FIRMWARE_MODE == FW_MODE_MOTION
        // Every iteration, not just once per control tick -- the hardware
        // Tx FIFO/Queue can empty out between ticks, and servicing
        // promptly keeps the SET_CURRENT mailbox's value as fresh as
        // possible by the time it actually reaches the bus. No-op when
        // there's nothing new to send or a frame is still in flight.
        vesc_can_tx_service();

        // Drives periodic TELEMETRY sends per the last SET_TELEMETRY_RATE
        // -- a no-op outside binary mode or when telemetry is disabled.
        binproto_tick(HAL_GetTick());

        // control_ticks_pending is written only from HAL_SYSTICK_Callback()
        // (interrupt context) and read/cleared only here -- the
        // disable/enable pair makes that non-atomic read-modify-write safe
        // against a tick landing mid-read.
        __disable_irq();
        uint32_t run_step = control_ticks_pending;
        control_ticks_pending = 0;
        __enable_irq();

        if (run_step)
        {
            if (run_step > 1)
                control_notify_overrun(run_step - 1);

            // dt reflects actual elapsed time, NOT a fixed 0.001f: if a
            // tick (or several) got discarded above, the PI integrators'
            // dt*error term and the derivative term's /dt both need the
            // real elapsed period or they silently under/over-react
            // relative to what actually happened physically. Clamped to
            // [1,10]ms -- the lower bound guards a 0ms read (two ticks
            // landing in the same HAL_GetTick() millisecond), the upper
            // bound keeps one truly pathological stall (main() wedged for
            // 100ms) from producing a single wild-dt control_step() call;
            // beyond 10ms the position/velocity feedback is stale enough
            // that FAULT_BIT_CAN_TIMEOUT should already be tripping on its
            // own via vesc_can_poll() -> update_faults() regardless.
            uint32_t now_ms = HAL_GetTick();
            uint32_t elapsed_ms = now_ms - last_control_ms;
            last_control_ms = now_ms;

            float dt = (float)elapsed_ms * 0.001f;
            if (dt < 0.001f) dt = 0.001f;
            if (dt > 0.010f) dt = 0.010f;

            // vesc_can_poll() drains the shared RX FIFO ONCE per tick (not
            // once per motor -- it dispatches each frame to whichever
            // motor_index its sender_id belongs to internally) -- the sole
            // FDCAN_RX_FIFO0 consumer in this mode. Must run before any
            // motor's control_step() so this tick's cascade sees fresh
            // feedback.
            vesc_can_poll(now_ms);

            // 1kHz control calculation for every motor, then one
            // system-wide fault aggregation pass -- see control.c's
            // "System fault escalation" comment for why this order (each
            // motor's control_step() observes the aggregate as of the end
            // of the PREVIOUS tick; recomputing here makes THIS tick's
            // result available starting next tick, <=1ms propagation).
            for (uint8_t i = 0; i < MOTOR_COUNT; i++)
                control_step(i, dt, now_ms);
            control_recompute_system_fault();

            // SET_CURRENT rate gate: control_step() computes a fresh
            // effective_current_cmd for every motor every 1kHz tick, but
            // publishing all 4 to their mailboxes every tick would let
            // vesc_can_tx_service()'s round-robin send as fast as the bus
            // allows -- not the ~500Hz/motor budget this board's 4-VESC/
            // 1Mbps bus design targets (see params.h's
            // PARAM_ANGLE_SPAN_MOTOR_REV comment). Splitting the 4 motors
            // into two alternating pairs, published on alternating 1ms
            // ticks, gives each motor exactly 1/(2*1ms) = 500Hz of fresh
            // mailbox values, spread evenly rather than bursting all 4 on
            // the same tick. vesc_can_tx_service()'s round-robin then
            // still spreads even a same-tick pair's two frames apart in
            // time rather than back-to-back. This applies uniformly
            // regardless of fault/idle state -- a FAULT/disabled motor's
            // 0A is republished at the same ~500Hz rate, not a one-shot
            // (still far faster than any mechanical or VESC-side command
            // timeout needs). Hardcoded to two pairs of 2 -- this assumes
            // MOTOR_COUNT==4 exactly; a different MOTOR_COUNT would need
            // this loop restructured (e.g. N alternating groups of
            // MOTOR_COUNT/N), not just a bigger unrolled if/else.
            if ((control_tick_counter & 1u) == 0u) {
                vesc_can_set_current(0, control_status(0)->effective_current_cmd);
                vesc_can_set_current(2, control_status(2)->effective_current_cmd);
            } else {
                vesc_can_set_current(1, control_status(1)->effective_current_cmd);
                vesc_can_set_current(3, control_status(3)->effective_current_cmd);
            }
            control_tick_counter++;
        }

        cdc_process_motion_rx(); // drains rxbuf through shell.c's line parser (get/set/enable/status/... -- see shell.h)
#else
        // Message has been received, pull it from the buffer
        if(is_can_msg_pending(FDCAN_RX_FIFO0))
        {
            // If message received from bus, parse the frame
            if (can_rx(&rx_msg_header, rx_msg_data) == HAL_OK)
            {
                int32_t msg_len = atcan_build_frame((uint8_t *)&msg_buf, &rx_msg_header, rx_msg_data);

                // Transmit message via USB-CDC
                if(msg_len > 0)
                {
                    cdc_transmit(msg_buf, msg_len);
                }
            }
        }

        cdc_process_atcan_rx();
#endif
    }
}
