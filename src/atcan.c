//
// atcan: AT-frame protocol (CH340 AT-CAN dongle-compatible) parse/generate.
// See inc/atcan.h for the wire format.
//

#include "stm32g4xx_hal.h"
#include "can.h"
#include "atcan.h"
#include "slcan.h" // hal_dlc_code_to_bytes()

// Fixed bus bitrate this firmware speaks. cr09 の VESC バスは 500k 前提
// （tools/can0_up.sh / atcan.yaml と揃える）。変える場合はここだけ直す。
#define ATCAN_BITRATE CAN_BITRATE_500K

// ---- RX (USB -> CAN) parser state machine ----
typedef enum
{
    ST_WAIT_A,
    ST_WAIT_T,
    ST_ADDR,
    ST_DLC,
    ST_DATA,
    ST_CR,
    ST_LF,
} atcan_state_t;

static atcan_state_t state = ST_WAIT_A;
static uint8_t addr_buf[4];
static uint8_t addr_idx;
static uint8_t dlc;
static uint8_t data_buf[8];
static uint8_t data_idx;


void atcan_init(void)
{
    can_set_bitrate(ATCAN_BITRATE);
    can_enable();
}


// Classic-CAN DLC code (0..8, byte count) -> FDCAN HAL DataLength code.
// Matches slcan.c's __std_dlc_code_to_hal_dlc_code() (not exported, so
// duplicated here -- it's a one-line shift).
static inline uint32_t dlc_to_hal(uint8_t dlc_bytes)
{
    return (uint32_t)dlc_bytes << 16;
}


void atcan_feed_byte(uint8_t b)
{
    switch (state)
    {
    case ST_WAIT_A:
        if (b == 0x41) // 'A'
            state = ST_WAIT_T;
        break;

    case ST_WAIT_T:
        if (b == 0x54) // 'T'
        {
            state = ST_ADDR;
            addr_idx = 0;
        }
        else if (b != 0x41)
        {
            state = ST_WAIT_A;
        }
        // else: stay on ST_WAIT_T (repeated 'A', handles "AAT..." resync)
        break;

    case ST_ADDR:
        addr_buf[addr_idx++] = b;
        if (addr_idx >= 4)
            state = ST_DLC;
        break;

    case ST_DLC:
        dlc = b;
        data_idx = 0;
        if (dlc > 8)
        {
            // 分類できないDLC。フレーミング崩れとみなし再同期。
            state = ST_WAIT_A;
            break;
        }
        state = (dlc == 0) ? ST_CR : ST_DATA;
        break;

    case ST_DATA:
        data_buf[data_idx++] = b;
        if (data_idx >= dlc)
            state = ST_CR;
        break;

    case ST_CR:
        state = (b == 0x0D) ? ST_LF : ST_WAIT_A;
        break;

    case ST_LF:
        if (b == 0x0A)
        {
            const uint32_t wire_addr = ((uint32_t)addr_buf[0] << 24) |
                                        ((uint32_t)addr_buf[1] << 16) |
                                        ((uint32_t)addr_buf[2] << 8) |
                                        (uint32_t)addr_buf[3];
            FDCAN_TxHeaderTypeDef h =
            {
                .Identifier = wire_addr >> 3,
                .IdType = FDCAN_EXTENDED_ID,
                .TxFrameType = FDCAN_DATA_FRAME,
                .FDFormat = FDCAN_CLASSIC_CAN,
                .BitRateSwitch = FDCAN_BRS_OFF,
                .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
                .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
                .MessageMarker = 0,
                .DataLength = dlc_to_hal(dlc),
            };
            can_tx(&h, data_buf);
        }
        state = ST_WAIT_A;
        break;
    }
}


int32_t atcan_build_frame(uint8_t *buf, FDCAN_RxHeaderTypeDef *frame_header, uint8_t *frame_data)
{
    // このFWはclassic CANのみ中継する（FDフレームはRobStride/VESCどちらも
    // 使わない。atcan_bridge_node側もclassic前提）。
    if (frame_header->FDFormat != FDCAN_CLASSIC_CAN)
        return -1;

    const int8_t bytes = hal_dlc_code_to_bytes(frame_header->DataLength);
    if (bytes < 0 || bytes > 8)
        return -1;

    // 29bit拡張ID前提（atcan_bridge_node / VESC / RobStride ともこれのみ使用）。
    if (frame_header->IdType != FDCAN_EXTENDED_ID)
        return -1;

    const uint32_t wire_addr = (frame_header->Identifier << 3) | 0x04u;

    uint8_t idx = 0;
    buf[idx++] = 0x41; // 'A'
    buf[idx++] = 0x54; // 'T'
    buf[idx++] = (uint8_t)(wire_addr >> 24);
    buf[idx++] = (uint8_t)(wire_addr >> 16);
    buf[idx++] = (uint8_t)(wire_addr >> 8);
    buf[idx++] = (uint8_t)(wire_addr);
    buf[idx++] = (uint8_t)bytes;
    for (int8_t i = 0; i < bytes; ++i)
        buf[idx++] = frame_data[i];
    buf[idx++] = 0x0D;
    buf[idx++] = 0x0A;

    return (int32_t)idx;
}
