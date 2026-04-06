#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

/* ── PID tuning constants (change here, not in .c) ─────────────────────────── */
#define MOTOR_KP           2.0f
#define MOTOR_KI           0.5f
#define MOTOR_KD           0.1f
#define MOTOR_INTEGRAL_MAX 500    /* anti-windup clamp             */
#define MOTOR_PWM_MAX      999    /* TIM2 ARR-1 for 100 % duty     */

/* ── Direction pin (PB0) ────────────────────────────────────────────────────── */
#define MOTOR_DIR_PORT  GPIOB
#define MOTOR_DIR_PIN   GPIO_PIN_0
#define MOTOR_FWD       GPIO_PIN_SET
#define MOTOR_REV       GPIO_PIN_RESET

/* ── Public API ─────────────────────────────────────────────────────────────── */
void  motor_init(void);

/* Set desired speed; negative = reverse */
void  motor_set_target_rpm(int32_t rpm);

/* PID update — call at 200 Hz */
void  motor_pid_update(void);

/* Force stop and reset integrator */
void  motor_estop(void);

#endif /* MOTOR_CONTROL_H */
