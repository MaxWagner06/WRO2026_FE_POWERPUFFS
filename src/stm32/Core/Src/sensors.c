#include "sensors.h"
#include "filters.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* ── Globals ────────────────────────────────────────────────────────────────── */
volatile SensorData g_sensors = {0};

/* External HAL handles — defined in main.c / CubeMX-generated code */
extern UART_HandleTypeDef huart2;   /* TFMini Front  */
extern UART_HandleTypeDef huart3;   /* TFMini Left   */
extern UART_HandleTypeDef huart4;   /* TFMini Right  */
extern I2C_HandleTypeDef  hi2c1;    /* IMU           */
extern TIM_HandleTypeDef  htim3;    /* Encoder       */

/* ── I2C addresses for Adafruit ISM330DHCX + LIS3MDL ───────────────────────── */
#define LSM6_ADDR    (0x6A << 1)   /* SA0 tied low                  */
#define LIS3MDL_ADDR (0x1C << 1)   /* SA1 tied low                  */

/* LSM6 register addresses */
#define LSM6_CTRL1_XL  0x10   /* accelerometer control             */
#define LSM6_CTRL2_G   0x11   /* gyroscope control                 */
#define LSM6_OUTX_L_G  0x22   /* gyro output start register        */

/* LIS3MDL register addresses */
#define LIS3_CTRL_REG1 0x20
#define LIS3_OUT_X_L   0x28

/* ── Internal state ─────────────────────────────────────────────────────────── */
static MovingAvg  s_avg_front, s_avg_left, s_avg_right;
static CompFilter s_heading;

/* 9-byte receive buffers for each TFMini */
#define TFMINI_FRAME_LEN 9
static uint8_t s_tf_front_buf[TFMINI_FRAME_LEN];
static uint8_t s_tf_left_buf[TFMINI_FRAME_LEN];
static uint8_t s_tf_right_buf[TFMINI_FRAME_LEN];

/* Encoder tick tracking */
static int32_t s_prev_ticks = 0;
static uint32_t s_prev_tick_ms = 0;

/* Gyro sensitivity: ±250 dps full-scale → 8.75 mdps/LSB */
#define GYRO_SENSITIVITY_MDPS  8.75f

/* ── Public API ─────────────────────────────────────────────────────────────── */

void sensors_init(void) {
    moving_avg_init(&s_avg_front);
    moving_avg_init(&s_avg_left);
    moving_avg_init(&s_avg_right);
    comp_filter_init(&s_heading, 0.0f);

    /* Configure LSM6 accelerometer + gyro: 208 Hz ODR, ±2g / ±250 dps */
    uint8_t val;
    val = 0x50; HAL_I2C_Mem_Write(&hi2c1, LSM6_ADDR, LSM6_CTRL1_XL, 1, &val, 1, 10);
    val = 0x50; HAL_I2C_Mem_Write(&hi2c1, LSM6_ADDR, LSM6_CTRL2_G,  1, &val, 1, 10);

    /* Configure LIS3MDL: continuous measurement, 80 Hz */
    val = 0x7C; HAL_I2C_Mem_Write(&hi2c1, LIS3MDL_ADDR, LIS3_CTRL_REG1, 1, &val, 1, 10);

    /* Kick off DMA reception on all three TFMini UARTs */
    HAL_UART_Receive_DMA(&huart2, s_tf_front_buf, TFMINI_FRAME_LEN);
    HAL_UART_Receive_DMA(&huart3, s_tf_left_buf,  TFMINI_FRAME_LEN);
    HAL_UART_Receive_DMA(&huart4, s_tf_right_buf, TFMINI_FRAME_LEN);

    /* Start encoder timer */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

void sensors_poll(void) {
    /* ── ToF: parse last DMA-filled frame, feed moving average ── */
    uint16_t dist_cm;

    if (tfmini_parse(s_tf_front_buf, &dist_cm))
        g_sensors.tof_front_mm = moving_avg_update(&s_avg_front, dist_cm * 10);

    if (tfmini_parse(s_tf_left_buf, &dist_cm))
        g_sensors.tof_left_mm  = moving_avg_update(&s_avg_left,  dist_cm * 10);

    if (tfmini_parse(s_tf_right_buf, &dist_cm))
        g_sensors.tof_right_mm = moving_avg_update(&s_avg_right, dist_cm * 10);

    /* ── IMU: read 6 gyro bytes from LSM6 ── */
    uint8_t gyro_raw[6];
    if (HAL_I2C_Mem_Read(&hi2c1, LSM6_ADDR, LSM6_OUTX_L_G | 0x80, 1, gyro_raw, 6, 5) == HAL_OK) {
        int16_t gz = (int16_t)((gyro_raw[5] << 8) | gyro_raw[4]);
        g_sensors.gyro_z_deg_s = gz * GYRO_SENSITIVITY_MDPS / 1000.0f;
    }

    /* ── IMU: read magnetometer for heading ── */
    uint8_t mag_raw[6];
    float mag_heading = g_sensors.imu_yaw_deg;   /* keep last if read fails */
    if (HAL_I2C_Mem_Read(&hi2c1, LIS3MDL_ADDR, LIS3_OUT_X_L | 0x80, 1, mag_raw, 6, 5) == HAL_OK) {
        int16_t mx = (int16_t)((mag_raw[1] << 8) | mag_raw[0]);
        int16_t my = (int16_t)((mag_raw[3] << 8) | mag_raw[2]);
        /* atan2 gives heading in radians; convert to degrees in [0,360) */
        float rad = -__builtin_atan2f((float)my, (float)mx);
        mag_heading = rad * (180.0f / 3.14159265f);
        if (mag_heading < 0.0f) mag_heading += 360.0f;
    }

    /* Complementary filter: dt = 5 ms at 200 Hz */
    float heading = comp_filter_update(&s_heading, g_sensors.gyro_z_deg_s, mag_heading, 0.005f);
    g_sensors.imu_yaw_deg = heading;

    /* ── Encoder: compute RPM from tick delta ── */
    int32_t current_ticks = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    uint32_t now_ms = HAL_GetTick();
    uint32_t dt_ms  = now_ms - s_prev_tick_ms;

    if (dt_ms > 0) {
        int32_t delta = current_ticks - s_prev_ticks;
        /* Encoder PPR = 1000 (250 CPR × 4 quadrature) — adjust to your encoder */
        g_sensors.encoder_rpm = (int32_t)((delta * 60000L) / ((int32_t)dt_ms * 1000L));
        g_sensors.encoder_ticks += delta;
    }

    s_prev_ticks   = current_ticks;
    s_prev_tick_ms = now_ms;
}

bool tfmini_parse(const uint8_t *buf, uint16_t *dist_cm) {
    /* Verify start bytes */
    if (buf[0] != 0x59 || buf[1] != 0x59) return false;

    /* Verify checksum: sum of bytes 0-7 == byte 8 (low 8 bits) */
    uint8_t chk = 0;
    for (int i = 0; i < 8; i++) chk += buf[i];
    if (chk != buf[8]) return false;

    /* Distance in bytes 2-3, little-endian, unit = cm */
    *dist_cm = (uint16_t)(buf[2] | (buf[3] << 8));
    return true;
}
