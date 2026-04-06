#include "filters.h"
#include <string.h>

/* ── Moving Average ─────────────────────────────────────────────────────────── */

void moving_avg_init(MovingAvg *f) {
    memset(f->buf, 0, sizeof(f->buf));
    f->head  = 0;
    f->count = 0;
    f->sum   = 0;
}

uint16_t moving_avg_update(MovingAvg *f, uint16_t new_sample) {
    /* Subtract the oldest sample from the running sum */
    f->sum -= f->buf[f->head];

    /* Write the new sample at the current head position */
    f->buf[f->head] = new_sample;
    f->sum += new_sample;

    /* Advance head (circular) */
    f->head = (f->head + 1) % MOVING_AVG_WINDOW;

    if (f->count < MOVING_AVG_WINDOW)
        f->count++;

    return (uint16_t)(f->sum / f->count);
}

/* ── Complementary Filter ───────────────────────────────────────────────────── */

void comp_filter_init(CompFilter *f, float initial_heading) {
    f->heading = initial_heading;
}

float comp_filter_update(CompFilter *f, float gyro_z_deg_s, float mag_heading_deg, float dt) {
    /* Integrate gyro for short-term accuracy */
    float gyro_heading = f->heading + gyro_z_deg_s * dt;

    /* Blend with magnetometer to prevent long-term drift */
    f->heading = COMP_FILTER_ALPHA * gyro_heading
               + (1.0f - COMP_FILTER_ALPHA) * mag_heading_deg;

    /* Keep heading in [0, 360) */
    if (f->heading < 0.0f)    f->heading += 360.0f;
    if (f->heading >= 360.0f) f->heading -= 360.0f;

    return f->heading;
}
