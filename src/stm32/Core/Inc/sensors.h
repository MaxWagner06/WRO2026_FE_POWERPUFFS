#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <stdbool.h>

/* ── Sensor data exposed to the rest of the firmware ───────────────────────── */
typedef struct {
    uint16_t tof_front_mm;    /* filtered TFMini-S #1 reading   */
    uint16_t tof_left_mm;     /* filtered TFMini-S #2 reading   */
    uint16_t tof_right_mm;    /* filtered TFMini-S #3 reading   */

    float    imu_yaw_deg;     /* complementary-filtered heading  */
    float    imu_pitch_deg;
    float    imu_roll_deg;
    float    gyro_z_deg_s;    /* raw gyro Z rate (for corner det)*/

    int32_t  encoder_ticks;   /* cumulative TIM3 encoder count   */
    int32_t  encoder_rpm;     /* computed RPM from tick delta    */
} SensorData;

/* Global sensor snapshot — written by sensors task, read by comm/motor tasks */
extern volatile SensorData g_sensors;

/* ── Public API ─────────────────────────────────────────────────────────────── */
void sensors_init(void);

/* Call at 200 Hz from TIM interrupt or high-priority task */
void sensors_poll(void);

/* Parse a 9-byte TFMini-S frame; returns true on valid checksum */
bool tfmini_parse(const uint8_t *buf, uint16_t *dist_cm);

#endif /* SENSORS_H */
