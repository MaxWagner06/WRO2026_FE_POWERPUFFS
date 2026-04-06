#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

/* ── Servo PWM parameters ───────────────────────────────────────────────────── */
#define SERVO_CENTER_US    1500   /* neutral / straight-ahead pulse width */
#define SERVO_US_PER_DEG   18    /* microseconds per degree of steering  */
#define MAX_STEER_DEG      30    /* Ackermann mechanical limit           */

/* TIM1 CH1 (PA8) drives the servo */

/* ── Public API ─────────────────────────────────────────────────────────────── */
void servo_init(void);

/* angle_deg: -30 (full left) to +30 (full right), clamped internally */
void servo_set_angle(int16_t angle_deg);

#endif /* SERVO_H */
