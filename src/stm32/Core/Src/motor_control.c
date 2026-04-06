#include "motor_control.h"
#include "sensors.h"
#include "stm32f4xx_hal.h"

/* TIM2 CH1 drives the motor driver PWM (PA0) */
extern TIM_HandleTypeDef htim2;

/* ── PID state ──────────────────────────────────────────────────────────────── */
static volatile int32_t s_target_rpm  = 0;
static int32_t s_integral  = 0;
static int32_t s_prev_error = 0;

/* ── Helpers ────────────────────────────────────────────────────────────────── */
#define CLAMP(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

static void set_direction(int32_t rpm) {
    /* DIR pin determines forward vs reverse */
    if (rpm >= 0)
        HAL_GPIO_WritePin(MOTOR_DIR_PORT, MOTOR_DIR_PIN, MOTOR_FWD);
    else
        HAL_GPIO_WritePin(MOTOR_DIR_PORT, MOTOR_DIR_PIN, MOTOR_REV);
}

static void set_pwm(int32_t duty) {
    /* duty is 0..MOTOR_PWM_MAX */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)duty);
}

/* ── Public API ─────────────────────────────────────────────────────────────── */

void motor_init(void) {
    s_target_rpm  = 0;
    s_integral    = 0;
    s_prev_error  = 0;

    /* Configure direction pin as output (should already be in CubeMX init) */
    HAL_GPIO_WritePin(MOTOR_DIR_PORT, MOTOR_DIR_PIN, MOTOR_FWD);

    /* Start PWM */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    set_pwm(0);
}

void motor_set_target_rpm(int32_t rpm) {
    /* Clamp to safe operating range */
    s_target_rpm = CLAMP(rpm, -500, 500);
}

void motor_pid_update(void) {
    /* Read encoder RPM from sensor module */
    int32_t current_rpm = g_sensors.encoder_rpm;
    int32_t error = s_target_rpm - current_rpm;

    /* Accumulate integral with anti-windup clamp */
    s_integral = CLAMP(s_integral + error, -MOTOR_INTEGRAL_MAX, MOTOR_INTEGRAL_MAX);

    int32_t derivative = error - s_prev_error;
    s_prev_error = error;

    int32_t output = (int32_t)(MOTOR_KP * error
                             + MOTOR_KI * s_integral
                             + MOTOR_KD * derivative);

    /* Set direction based on sign of target, then apply absolute PWM */
    set_direction(s_target_rpm);
    set_pwm(CLAMP(output < 0 ? -output : output, 0, MOTOR_PWM_MAX));
}

void motor_estop(void) {
    s_target_rpm = 0;
    s_integral   = 0;
    s_prev_error = 0;
    set_pwm(0);
}
