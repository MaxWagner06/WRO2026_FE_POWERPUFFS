#ifndef SAFETY_H
#define SAFETY_H

#include <stdint.h>
#include <stdbool.h>

/* ── Thresholds ─────────────────────────────────────────────────────────────── */
#define WATCHDOG_TIMEOUT_MS  500    /* estop if no command for this long  */
#define BATTERY_LOW_MV       6800   /* 2S LiPo cutoff in millivolts       */
#define BATTERY_ADC_DIVIDER  2.0f   /* resistor divider ratio on PA4      */
#define VREF_MV              3300   /* ADC reference voltage               */
#define ADC_RESOLUTION       4096   /* 12-bit ADC                          */

/* ── Status LED (PA5 = LD2 on Nucleo) ──────────────────────────────────────── */
#define STATUS_LED_PORT  GPIOA
#define STATUS_LED_PIN   GPIO_PIN_5

/* Start button on PC13 */
#define START_BTN_PORT   GPIOC
#define START_BTN_PIN    GPIO_PIN_13

/* ── Public API ─────────────────────────────────────────────────────────────── */
void safety_init(void);

/* Call at 1 Hz: checks watchdog timer and battery voltage */
void safety_check(void);

/* Returns true once start button has been pressed */
bool safety_is_started(void);

/* Call from EXTI13 interrupt */
void safety_btn_irq(void);

#endif /* SAFETY_H */
