//
// CANable2 firmware
//

#include "stm32g4xx.h"
#include "printf.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "can.h"
#include "atcan.h"
#include "system.h"
#include "led.h"


int main(void)
{
    // Initialize peripherals
    system_init();
    can_init();
    led_init();
    usb_init();

    // AT-frameプロトコルはホスト側の open/bitrate コマンドが無い専用FWなので、
    // ここで固定ビットレートでバスを起動する（atcan.c の ATCAN_BITRATE）。
    atcan_init();

    // Power-on blink sequence
    led_blue_blink(2);

    // Storage for status and received message buffer
    FDCAN_RxHeaderTypeDef rx_msg_header;
    uint8_t rx_msg_data[64] = {0};
    uint8_t msg_buf[ATCAN_FRAME_MAX];


    while(1)
    {
        led_process();
        can_process();
        cdc_process();

        // bus-off ウォッチドッグ（~20ms毎）: 詰まったら FDCAN を自動再初期化
        static uint32_t last_recover = 0;
        if ((uint32_t)(HAL_GetTick() - last_recover) >= 20u)
        {
            last_recover = HAL_GetTick();
            can_check_recover();
        }
        
        

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
    }
}

