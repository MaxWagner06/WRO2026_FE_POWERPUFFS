#include "comm.h"
#include "sensors.h"
#include "motor_control.h"
#include "servo.h"
#include "safety.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* USART1 (PA9/PA10) is the Pi link */
extern UART_HandleTypeDef huart1;

/* ── Globals read by other modules ─────────────────────────────────────────── */
volatile int32_t  g_target_rpm    = 0;
volatile int16_t  g_target_steer  = 0;
volatile uint32_t g_last_cmd_tick = 0;

/* ── Ring buffer for incoming bytes (DMA / interrupt feed) ───────────────────  */
#define RX_BUF_SIZE 128
static uint8_t  s_rx_buf[RX_BUF_SIZE];
static uint16_t s_rx_head = 0;   /* written by ISR/DMA      */
static uint16_t s_rx_tail = 0;   /* consumed by comm_process_rx */

/* ── Telemetry TX buffer ────────────────────────────────────────────────────── */
static telemetry_packet_t s_telem_pkt;

/* ── CRC-8/MAXIM (same algorithm as Python side) ────────────────────────────── */
uint8_t crc8_maxim(const uint8_t *data, uint16_t len) {
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x01)
                crc = (crc >> 1) ^ 0x8C;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/* ── Public API ─────────────────────────────────────────────────────────────── */

void comm_init(void) {
    g_target_rpm   = 0;
    g_target_steer = 0;

    /* Enable USART1 RX interrupt — one byte at a time into the ring buffer */
    HAL_UART_Receive_IT(&huart1, &s_rx_buf[s_rx_head], 1);
}

/* Called from USART1 RX complete interrupt via HAL callback */
void comm_rx_byte(uint8_t byte) {
    uint16_t next = (s_rx_head + 1) % RX_BUF_SIZE;
    if (next != s_rx_tail) {
        s_rx_buf[s_rx_head] = byte;
        s_rx_head = next;
    }
    /* Re-arm the interrupt for the next byte */
    HAL_UART_Receive_IT(&huart1, &s_rx_buf[s_rx_head], 1);
}

void comm_process_rx(void) {
    /* Consume bytes until we find a complete, valid command packet */
    while (1) {
        /* Calculate how many bytes are available */
        uint16_t avail = (s_rx_head - s_rx_tail + RX_BUF_SIZE) % RX_BUF_SIZE;
        if (avail < sizeof(command_packet_t)) break;

        /* Peek at header */
        if (s_rx_buf[s_rx_tail] != CMD_HEADER_0) {
            s_rx_tail = (s_rx_tail + 1) % RX_BUF_SIZE;   /* skip garbage byte */
            continue;
        }
        if (s_rx_buf[(s_rx_tail + 1) % RX_BUF_SIZE] != CMD_HEADER_1) {
            s_rx_tail = (s_rx_tail + 1) % RX_BUF_SIZE;
            continue;
        }

        /* Copy the full packet into a contiguous local buffer */
        uint8_t pkt_buf[sizeof(command_packet_t)];
        for (uint8_t i = 0; i < sizeof(command_packet_t); i++)
            pkt_buf[i] = s_rx_buf[(s_rx_tail + i) % RX_BUF_SIZE];

        /* Validate CRC (covers all bytes except the last one) */
        uint8_t expected = crc8_maxim(pkt_buf, sizeof(command_packet_t) - 1);
        if (expected != pkt_buf[sizeof(command_packet_t) - 1]) {
            s_rx_tail = (s_rx_tail + 1) % RX_BUF_SIZE;   /* bad CRC, skip one byte */
            continue;
        }

        /* Parse the packet */
        command_packet_t *cmd = (command_packet_t *)pkt_buf;
        switch (cmd->msg_type) {
            case CMD_SPEED:
                g_target_rpm = cmd->value;
                motor_set_target_rpm(cmd->value);
                break;
            case CMD_STEER:
                g_target_steer = cmd->value;
                servo_set_angle(cmd->value);
                break;
            case CMD_ESTOP:
                motor_estop();
                g_target_rpm   = 0;
                g_target_steer = 0;
                break;
            default:
                break;
        }

        g_last_cmd_tick = HAL_GetTick();   /* refresh watchdog timestamp */
        s_rx_tail = (s_rx_tail + sizeof(command_packet_t)) % RX_BUF_SIZE;
    }
}

void comm_send_telemetry(void) {
    /* Fill the telemetry struct from the global sensor snapshot */
    s_telem_pkt.header[0]    = TELEM_HEADER_0;
    s_telem_pkt.header[1]    = TELEM_HEADER_1;
    s_telem_pkt.msg_type     = TELEM_SENSORS;
    s_telem_pkt.length       = 20;   /* payload bytes */

    s_telem_pkt.tof_front    = g_sensors.tof_front_mm;
    s_telem_pkt.tof_left     = g_sensors.tof_left_mm;
    s_telem_pkt.tof_right    = g_sensors.tof_right_mm;
    s_telem_pkt.imu_yaw      = (int16_t)(g_sensors.imu_yaw_deg   * 100.0f);
    s_telem_pkt.imu_pitch    = (int16_t)(g_sensors.imu_pitch_deg * 100.0f);
    s_telem_pkt.imu_roll     = (int16_t)(g_sensors.imu_roll_deg  * 100.0f);
    s_telem_pkt.encoder_ticks = g_sensors.encoder_ticks;
    s_telem_pkt.battery_mv   = 0;   /* filled by safety.c at 1 Hz; copied here */

    /* CRC covers everything except the CRC byte itself */
    s_telem_pkt.crc8 = crc8_maxim((uint8_t *)&s_telem_pkt,
                                   sizeof(s_telem_pkt) - 1);

    HAL_UART_Transmit(&huart1, (uint8_t *)&s_telem_pkt, sizeof(s_telem_pkt), 10);
}
