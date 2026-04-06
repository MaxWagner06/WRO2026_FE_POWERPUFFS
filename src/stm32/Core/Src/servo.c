#include "servo.h"
#include "stm32f4xx_hal.h"

/* TIM1 CH1 (PA8) — 50 Hz PWM, ARR = 19999 at 1 MHz timer clock */
extern TIM_HandleTypeDef htim1;

#define CLAMP(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

void servo_init(void) {
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    /* Centre the servo at init */
    servo_set_angle(0);
}

void servo_set_angle(int16_t angle_deg) {
    angle_deg = CLAMP(angle_deg, -MAX_STEER_DEG, MAX_STEER_DEG);

    /* Convert angle to PWM pulse width in microseconds, then to timer counts.
       Timer is configured at 1 MHz so 1 count = 1 µs. */
    int32_t pulse_us = SERVO_CENTER_US + (angle_deg * SERVO_US_PER_DEG);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)pulse_us);
}
