#include "safety.h"
#include "motor_control.h"
#include "comm.h"
#include "stm32f4xx_hal.h"

extern ADC_HandleTypeDef hadc1;   /* ADC1 CH4 (PA4) for battery voltage */

static volatile bool s_started    = false;
static volatile bool s_battery_ok = true;

/* Battery millivolts updated at 1 Hz (written here, read by comm.c) */
volatile uint16_t g_battery_mv = 0;

void safety_init(void) {
    /* Status LED off at start */
    HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, GPIO_PIN_RESET);

    /* Enable EXTI interrupt for start button on PC13 */
    /* (GPIO_EXTI13 init done in CubeMX/HAL_GPIO_Init) */
}

void safety_check(void) {
    /* ── Watchdog: stop motors if Pi silent for WATCHDOG_TIMEOUT_MS ── */
    uint32_t now = HAL_GetTick();
    if ((now - g_last_cmd_tick) > WATCHDOG_TIMEOUT_MS) {
        motor_estop();
        /* Blink LED to signal watchdog trip */
        HAL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED_PIN);
    }

    /* ── Battery voltage via ADC ── */
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) {
        uint32_t raw = HAL_ADC_GetValue(&hadc1);
        /* Convert ADC counts to millivolts, accounting for resistor divider */
        g_battery_mv = (uint16_t)((raw * VREF_MV / ADC_RESOLUTION) * BATTERY_ADC_DIVIDER);
    }
    HAL_ADC_Stop(&hadc1);

    if (g_battery_mv > 0 && g_battery_mv < BATTERY_LOW_MV) {
        /* Battery too low — cut drive power and light LED solid */
        motor_estop();
        HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, GPIO_PIN_SET);
        s_battery_ok = false;
    }
}

bool safety_is_started(void) {
    return s_started;
}

void safety_btn_irq(void) {
    /* Debounce by checking pin state; EXTI fires on falling edge (button pulled-up) */
    if (HAL_GPIO_ReadPin(START_BTN_PORT, START_BTN_PIN) == GPIO_PIN_RESET) {
        s_started = true;
        /* Confirm with LED on */
        HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, GPIO_PIN_SET);
    }
}
