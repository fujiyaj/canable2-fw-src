#include <stdbool.h>

#include "usbd_cdc_if.h"
#include "atcan.h"
#include "shell.h"
#include "binproto.h"
#include "system.h"
#include "error.h"

// Private variables
static usbrx_buf_t rxbuf = {0};
static usbtx_buf_t txbuf = {0};
static uint8_t tx_linbuf[TX_LINBUF_SIZE] = {0};

// TX diagnostics -- see PARAM_CDC_TX_* in params.h for what these mean and
// why they were added (chasing the intermittent multi-command USB hang).
static volatile uint32_t cdc_tx_enqueue_count = 0;
static volatile uint32_t cdc_tx_drop_count = 0;
static volatile uint32_t cdc_tx_busy_count = 0;


// Externs
extern USBD_HandleTypeDef hUsbDeviceFS;


// Private prototypes
static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
void cdc_process(void);


USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};


// Initializes the CDC media low layer over the FS USB IP
static int8_t CDC_Init_FS(void)
{
	rxbuf.head = 0;
	rxbuf.tail = 0;
	txbuf.head = 0;
	txbuf.tail = 0;
	cdc_tx_enqueue_count = 0;
	cdc_tx_drop_count = 0;
	cdc_tx_busy_count = 0;

	USBD_CDC_SetTxBuffer(&hUsbDeviceFS, tx_linbuf, 0);
	USBD_CDC_SetRxBuffer(&hUsbDeviceFS, rxbuf.buf[rxbuf.head]);
	return (USBD_OK);
}


// DeInitializes the CDC media low layer
static int8_t CDC_DeInit_FS(void)
{
	return (USBD_OK);
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:

    break;

    case CDC_GET_LINE_CODING:
        pbuf[0] = (uint8_t)(115200);
	pbuf[1] = (uint8_t)(115200 >> 8);
	pbuf[2] = (uint8_t)(115200 >> 16);
	pbuf[3] = (uint8_t)(115200 >> 24);
	pbuf[4] = 0; // stop bits (1)
	pbuf[5] = 0; // parity (none)
	pbuf[6] = 8; // number of bits (8)
        break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will block any OUT packet reception on USB endpoint
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result
  *         in receiving more data while previous ones are still not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */


static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
	// Check for overflow!
	// If when we increment the head we're going to hit the tail
	// (if we're filling the last spot in the queue)
	// FIXME: Use a "full" variable instead of wasting one
	// spot in the cirbuf as we are doing now
	if( ((rxbuf.head + 1) % NUM_RX_BUFS) == rxbuf.tail)
	{
		error_assert(ERR_FULLBUF_USBRX);

		// Listen again on the same buffer. Old data will be overwritten.
	    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, rxbuf.buf[rxbuf.head]);
	    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
		return HAL_ERROR;
	}
	else
	{
		// Save off length
		rxbuf.msglen[rxbuf.head] = *Len;
		rxbuf.head = (rxbuf.head + 1) % NUM_RX_BUFS;

		// Start listening on next buffer. Previous buffer will be processed in main loop.
	    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, rxbuf.buf[rxbuf.head]);
	    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
	    return (USBD_OK);
	}
}


// Flush queued USB-CDC transmit data (cdc_transmit()'s output). Must be
// called every main loop iteration regardless of firmware mode -- this is
// the only thing that actually pushes bytes out over USB.
void cdc_process_tx(void)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

    if(hcdc->TxState != 0)
    {
    	// Hardware/USB core isn't ready for another packet yet. Normally
    	// this just means the previous packet's IN-token completion (or,
    	// for an exact-maxpacket-multiple transfer, its follow-up
    	// Zero-Length-Packet's completion -- see USBD_CDC_DataIn() in
    	// usbd_cdc.c) hasn't fired yet, which clears back to 0 quickly. If
    	// this counter climbs steadily while cdc_tx_bytes_pending stays
    	// nonzero, TxState is stuck busy and nothing will ever go out --
    	// that's the leading hypothesis for the multi-command USB hang.
    	cdc_tx_busy_count++;
    	return;
    }

    if(txbuf.tail == txbuf.head)
    	return; // nothing queued

    // Peek (don't yet commit) up to TX_LINBUF_SIZE bytes into tx_linbuf.
    // txbuf.tail is only advanced below, after USBD_CDC_TransmitPacket()
    // confirms it actually accepted the packet -- previously tail was
    // advanced *before* that call and the call's return value was never
    // checked, so any non-OK result would have silently dropped the
    // already-dequeued bytes with no record of it ever happening.
    uint16_t linbuf_ctr = 0;
    uint32_t peek = txbuf.tail;
    while(peek != txbuf.head)
    {
    	tx_linbuf[linbuf_ctr++] = txbuf.data[peek];
    	peek = (peek + 1UL) % USBTXQUEUE_LEN;

    	// Take up to the number of bytes to fill the linbuf
    	if(linbuf_ctr >= TX_LINBUF_SIZE)
    		break;
    }

	// Set transmit buffer and start TX
	USBD_CDC_SetTxBuffer(&hUsbDeviceFS, tx_linbuf, linbuf_ctr);
	uint8_t ret = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
	if(ret == USBD_OK)
	{
		txbuf.tail = peek; // commit: these bytes are now the hardware's problem
	}
	else
	{
		// USBD_BUSY/USBD_FAIL despite TxState==0 above -- shouldn't happen
		// in normal single-threaded main-loop use (nothing else calls
		// USBD_CDC_TransmitPacket()), but if it does, leave txbuf
		// untouched so the same bytes are retried next call instead of
		// being silently lost.
		cdc_tx_busy_count++;
	}
}

// Drain the USB-CDC receive buffer through the AT-frame parser (state kept
// in atcan.c). Only call this in FW_MODE_ATCAN (see main.c) -- atcan_feed_byte()
// can call can_tx(), which in FW_MODE_MOTION must only ever be driven by
// vesc_can_set_current(). If this isn't called, incoming rxbuf entries are
// simply left unconsumed (CDC_Receive_FS()'s overflow handling already
// tolerates that -- see its "Listen again on the same buffer" path) until a
// future ASCII command shell replaces this call with its own parser.
void cdc_process_atcan_rx(void)
{
    system_irq_disable();
	if(rxbuf.tail != rxbuf.head)
	{
		//  Process one whole buffer
		for (uint32_t i = 0; i < rxbuf.msglen[rxbuf.tail]; i++)
		{
			atcan_feed_byte(rxbuf.buf[rxbuf.tail][i]);
		}

		// Move on to next buffer
		rxbuf.tail = (rxbuf.tail + 1) % NUM_RX_BUFS;
	}
    system_irq_enable();
}

void cdc_process_motion_rx(void)
{
    // Unlike cdc_process_atcan_rx(), shell_feed_byte() can call back into
    // cdc_transmit() (shell replies) -- which itself takes/releases this
    // same system_irq_disable()/enable() pair. That pair isn't nesting-safe
    // (plain __disable_irq()/__enable_irq(), no PRIMASK save/restore), so
    // calling shell_feed_byte() while still holding the lock here would
    // have a nested cdc_transmit() re-enable interrupts out from under us
    // mid-loop. Fix: grab just this one buffer's length and advance the
    // tail under the lock, then process the (now already-consumed, so safe
    // to read lock-free) bytes after releasing it.
    system_irq_disable();
    bool has_data = (rxbuf.tail != rxbuf.head);
    uint32_t idx = rxbuf.tail;
    uint32_t len = has_data ? rxbuf.msglen[idx] : 0;
    if (has_data)
        rxbuf.tail = (rxbuf.tail + 1) % NUM_RX_BUFS;
    system_irq_enable();

    for (uint32_t i = 0; i < len; i++) {
        if (binproto_active())
            binproto_feed_byte(rxbuf.buf[idx][i]);
        else
            shell_feed_byte(rxbuf.buf[idx][i]);
    }
}


// Enqueue data for transmission over USB CDC to host
void cdc_transmit(uint8_t* buf, uint16_t len)
{
	system_irq_disable();

	// Pre-existing bug fixed in passing: `(head + len) % N == tail` only
	// catches the exact case where the write would land head precisely on
	// tail -- it misses every case where the write would wrap PAST tail
	// (e.g. N=2048, head=2000, tail=100, len=200: (2000+200)%2048=152,
	// which != 100, so the old check waved this through even though only
	// ~147 bytes were actually free -- it would silently overwrite 53
	// bytes of not-yet-transmitted data). Compute actual used/free space
	// instead and reject anything that wouldn't fit, full stop. This
	// matters more now than it used to: TELEMETRY at 100-200Hz can burst
	// enough bytes that a momentary stall on the host side (or just a
	// slow enough USB round-trip) makes this reachable, where the old
	// AT-frame-only traffic rarely would.
	uint32_t used = (txbuf.head >= txbuf.tail)
		? (txbuf.head - txbuf.tail)
		: (USBTXQUEUE_LEN - txbuf.tail + txbuf.head);
	uint32_t free = USBTXQUEUE_LEN - used - 1; // -1: reserve one slot so head==tail stays unambiguously "empty"

	if (len > free)
	{
		error_assert(ERR_FULLBUF_USBTX);
		cdc_tx_drop_count++;
		// Pre-existing bug fixed in passing: this used to `return` here
		// without re-enabling interrupts, permanently disabling ALL
		// interrupts (including CAN TX/RX and the 1kHz SysTick control
		// tick) the first time the TX queue ever overflowed. shell.c's
		// status/help output makes overflow much more likely to actually
		// happen than the old AT-frame-only traffic did, so this needed
		// fixing now rather than staying a latent landmine.
		system_irq_enable();
    	return;
    }
	else
	{
		// Copy data
	    for (uint32_t i=0; i < len; i++)
	    {
	    	txbuf.data[txbuf.head] = buf[i];

		    // Increment the head
			txbuf.head = (txbuf.head + 1UL) % USBTXQUEUE_LEN;
	    }
	    cdc_tx_enqueue_count++;

	}
    system_irq_enable();
}


// TX diagnostics getters -- see PARAM_CDC_TX_* in params.h. head/tail/
// bytes_pending are read without system_irq_disable(): these are informal
// diagnostics, not something a torn 32-bit read (which this Cortex-M4 core
// doesn't produce anyway for a naturally-aligned uint32_t) would corrupt.
uint32_t cdc_tx_get_enqueue_count(void) { return cdc_tx_enqueue_count; }
uint32_t cdc_tx_get_drop_count(void)    { return cdc_tx_drop_count; }
uint32_t cdc_tx_get_busy_count(void)    { return cdc_tx_busy_count; }
uint32_t cdc_tx_get_head(void)          { return txbuf.head; }
uint32_t cdc_tx_get_tail(void)          { return txbuf.tail; }

uint32_t cdc_tx_get_bytes_pending(void)
{
	uint32_t head = txbuf.head;
	uint32_t tail = txbuf.tail;
	return (head >= tail) ? (head - tail) : (USBTXQUEUE_LEN - tail + head);
}

