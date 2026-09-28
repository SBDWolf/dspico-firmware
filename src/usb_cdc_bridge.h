// DSpico firmware side of the PMDSky uplink.
//
// Runs a local TinyUSB CDC-ACM device stack on the RP2040 so USB
// enumeration and data transfer happen on-chip (no SDIO round trips per
// SETUP packet). The NDS ROM talks to this module over the card bus:
//   - WRITE_DATA (0xE9)  -> TX ring  -> tud_cdc_write()  (device -> host)
//   - READ_DATA  (0xEA)  <- RX ring  <- tud_cdc_n_read() (host -> device)
//   - READ_DATA  (0xEA)  <- status block (endpoint field selects which)
//
// Legacy mode (NDS runs the TinyUSB stack, firmware only forwards USB
// events over the event queue) remains available for old ROMs; the two
// modes are selected per boot by the LOCAL_STACK sub-command.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// True while the local (on-chip) TinyUSB stack owns the USB controller.
// The DCD IRQ handler polls this to decide between dcd_event_*() and
// forwarding the event to the NDS over the card bus.
bool usb_cdc_local_stack_active(void);

// Bring up the local stack: tusb_init() + tud_connect(). Safe to call
// again after a stop. Must be called from the card command context.
void usb_cdc_local_stack_start(void);

// Disconnect and hand the USB controller back to legacy event forwarding.
void usb_cdc_local_stack_stop(void);

// Pump the local stack: process queued USB events (tud_task) and drain
// the TX ring into the CDC class driver. No-op while local mode is off.
// Called from the firmware main loop.
void usb_cdc_task(void);

// Append up to 512 bytes of host-bound data to the TX ring. Returns the
// number of bytes accepted; the rest is dropped (counted). May be called
// from the card PIO context.
uint32_t usb_cdc_bridge_tx_append(const uint8_t* data, uint32_t len);

// Fill a 512-byte status block (rest zero-filled):
//   word0: bit0 local stack active
//          bit1 configured (SET_CONFIGURATION accepted)
//          bit2 CDC data interface open (tud_cdc_connected)
//          bit3 DTR, bit4 RTS
//   word1: total bytes handed to the CDC TX path
//   word2: bytes dropped (TX ring overflow)
//   word3: RX bytes pending
void usb_cdc_bridge_read_status(uint8_t* out512);

// Copy up to 512 bytes of pending host data into out512 (rest zero-filled)
// and return the number of bytes copied.
uint32_t usb_cdc_bridge_read_rx(uint8_t* out512);

#ifdef __cplusplus
}
#endif