#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#include "usbd_cdc.h"


// This takes 4k of RAM.
#define NUM_RX_BUFS 8
#define RX_BUF_SIZE CDC_DATA_FS_MAX_PACKET_SIZE // Size of RX buffer item


// CDC transmit buffering
#define TX_LINBUF_SIZE 64 // Set to 64 for max single packet size
#define USBTXQUEUE_LEN 2048 // Number of bytes allocated


// Transmit buffering: circular buffer FIFO
typedef struct usbtxbuf_
{
	uint8_t data[USBTXQUEUE_LEN]; // Data buffer
	uint32_t head; // Head pointer
	uint32_t tail; // Tail pointer
} usbtx_buf_t;


// Receive buffering: circular buffer FIFO
typedef struct _usbrx_buf_
{
	// Receive buffering: circular buffer FIFO
	uint8_t buf[NUM_RX_BUFS][RX_BUF_SIZE];
	uint32_t msglen[NUM_RX_BUFS];
	uint32_t head;
	uint32_t tail;

} usbrx_buf_t;


extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;


// Prototypes
void cdc_transmit(uint8_t* buf, uint16_t len);

// Split so callers can drive the TX flush unconditionally while choosing
// whether the AT-frame RX parser (atcan_feed_byte()) runs -- see main.c's
// firmware_mode_t. cdc_process_tx() must still be called every main loop
// iteration in ANY mode (including a future ASCII command shell) or queued
// cdc_transmit() output never actually goes out over USB.
void cdc_process_tx(void);
void cdc_process_atcan_rx(void);

// FW_MODE_MOTION's RX path: drains rxbuf, routing each byte to either
// shell_feed_byte() (ASCII debug shell) or binproto_feed_byte() (ROS2
// binary protocol) depending on binproto_active() -- see binproto.h for
// how/when that switches.
void cdc_process_motion_rx(void);

// TX diagnostics (PARAM_CDC_TX_* in params.h) -- see the register map
// comment there for why these exist. Free-running counters, never reset
// except by CDC_Init_FS() (i.e. on USB reset/reconfig).
uint32_t cdc_tx_get_enqueue_count(void);
uint32_t cdc_tx_get_drop_count(void);
uint32_t cdc_tx_get_busy_count(void);
uint32_t cdc_tx_get_bytes_pending(void);
uint32_t cdc_tx_get_head(void);
uint32_t cdc_tx_get_tail(void);


#endif
