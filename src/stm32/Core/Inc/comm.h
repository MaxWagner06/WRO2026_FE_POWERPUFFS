#ifndef COMM_H
#define COMM_H

#include <stdint.h>
#include "protocol.h"

/* ── Public API ─────────────────────────────────────────────────────────────── */
void comm_init(void);

/* Call from USART1 RX interrupt — feeds one byte into the ring buffer */
void comm_rx_byte(uint8_t byte);

/* Process pending bytes in ring buffer; updates g_target_rpm / g_target_steer.
   Call from main loop or low-priority task. */
void comm_process_rx(void);

/* Build and transmit one telemetry_packet_t over USART1. Call at 100 Hz. */
void comm_send_telemetry(void);

/* Globals written by comm, read by motor/servo tasks */
extern volatile int32_t  g_target_rpm;
extern volatile int16_t  g_target_steer;
extern volatile uint32_t g_last_cmd_tick;  /* HAL_GetTick() at last valid command */

#endif /* COMM_H */
