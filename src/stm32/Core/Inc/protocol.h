#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

/* ── Packet headers ──────────────────────────────────────────────────────────── */
#define CMD_HEADER_0    0xAA
#define CMD_HEADER_1    0x55
#define TELEM_HEADER_0  0xBB
#define TELEM_HEADER_1  0x66

/* ── Command message types (Pi → STM32) ─────────────────────────────────────── */
#define CMD_SPEED   0x01   /* int16 payload: RPM (-500..500)      */
#define CMD_STEER   0x02   /* int16 payload: degrees (-30..30)    */
#define CMD_MODE    0x03   /* int16 payload: mode enum            */
#define CMD_ESTOP   0xFF   /* emergency stop, ignore payload      */

/* ── Telemetry message types (STM32 → Pi) ───────────────────────────────────── */
#define TELEM_SENSORS 0x10

/* ── Packet structs ─────────────────────────────────────────────────────────── */

/* Pi → STM32 command packet (8 bytes total) */
typedef struct __attribute__((packed)) {
    uint8_t  header[2];   /* CMD_HEADER_0, CMD_HEADER_1          */
    uint8_t  msg_type;    /* CMD_SPEED / CMD_STEER / CMD_MODE / CMD_ESTOP */
    uint8_t  length;      /* payload byte count = 2              */
    int16_t  value;       /* speed RPM or steering degrees       */
    uint8_t  crc8;        /* CRC-8/MAXIM over all preceding bytes */
} command_packet_t;

/* STM32 → Pi telemetry packet (25 bytes total) */
typedef struct __attribute__((packed)) {
    uint8_t  header[2];       /* TELEM_HEADER_0, TELEM_HEADER_1  */
    uint8_t  msg_type;        /* TELEM_SENSORS                   */
    uint8_t  length;          /* payload byte count = 20         */
    uint16_t tof_front;       /* mm                              */
    uint16_t tof_left;        /* mm                              */
    uint16_t tof_right;       /* mm                              */
    int16_t  imu_yaw;         /* degrees * 100                   */
    int16_t  imu_pitch;       /* degrees * 100                   */
    int16_t  imu_roll;        /* degrees * 100                   */
    int32_t  encoder_ticks;   /* cumulative encoder ticks        */
    uint16_t battery_mv;      /* millivolts                      */
    uint8_t  crc8;
} telemetry_packet_t;

/* ── CRC-8/MAXIM (polynomial 0x31, reflected) ───────────────────────────────── */
uint8_t crc8_maxim(const uint8_t *data, uint16_t len);

#endif /* PROTOCOL_H */
