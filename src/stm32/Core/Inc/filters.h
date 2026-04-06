#ifndef FILTERS_H
#define FILTERS_H

#include <stdint.h>

/* ── Moving Average (for ToF readings) ──────────────────────────────────────── */
#define MOVING_AVG_WINDOW 5

typedef struct {
    uint16_t buf[MOVING_AVG_WINDOW];
    uint8_t  head;      /* next write index                  */
    uint8_t  count;     /* how many samples stored so far    */
    uint32_t sum;       /* running sum for O(1) average      */
} MovingAvg;

void     moving_avg_init(MovingAvg *f);
uint16_t moving_avg_update(MovingAvg *f, uint16_t new_sample);

/* ── Complementary Filter (for IMU heading) ─────────────────────────────────── */
/* ALPHA = 0.98: trust gyro short-term, magnetometer long-term                  */
#define COMP_FILTER_ALPHA 0.98f

typedef struct {
    float heading;       /* current fused heading in degrees  */
} CompFilter;

void  comp_filter_init(CompFilter *f, float initial_heading);
float comp_filter_update(CompFilter *f, float gyro_z_deg_s, float mag_heading_deg, float dt);

#endif /* FILTERS_H */
