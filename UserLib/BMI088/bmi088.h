/**
 * @file bmi088.h
 * @brief BMI088 driver (C), adapted from QGimbal UserLib/BMI088 (polling SPI).
 */
#ifndef BMI088_H
#define BMI088_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  BMI088_OK = 0x00,
  BMI088_ERR_ACC_PWR_CTRL = 0x01,
  BMI088_ERR_ACC_PWR_CONF = 0x02,
  BMI088_ERR_ACC_CONF = 0x03,
  BMI088_ERR_ACC_RANGE = 0x05,
  BMI088_ERR_GYRO_RANGE = 0x08,
  BMI088_ERR_GYRO_BANDWIDTH = 0x09,
  BMI088_ERR_GYRO_LPM1 = 0x0A,
  BMI088_ERR_GYRO_CTRL = 0x0B,
  /** Still-board check failed during BMI088_CalibrateGyro (motion / outliers). */
  BMI088_ERR_GYRO_MOTION = 0x0C,
  BMI088_ERR_NO_SENSOR = 0xFF,
} bmi088_err_t;

typedef struct {
  bool initialized;
  float accel[3]; /* m/s^2-ish scale from QGimbal (raw * sen) */
  float gyro[3];  /* rad/s, bias-subtracted after cal */
  float gyro_bias[3]; /* rad/s, from BMI088_CalibrateGyro */
  bool gyro_calibrated;
  float temperature_c;
} bmi088_t;

void BMI088_StructInit(bmi088_t *dev);
bmi088_err_t BMI088_Init(bmi088_t *dev);
void BMI088_Read(bmi088_t *dev);
/**
 * Average N still samples into gyro_bias; keep board still. delay_ms between samples.
 * Discards warmup samples, then two-pass trimmed mean; fails with BMI088_ERR_GYRO_MOTION
 * if too many samples look like motion.
 */
bmi088_err_t BMI088_CalibrateGyro(bmi088_t *dev, uint16_t samples, uint16_t delay_ms);

#ifdef __cplusplus
}
#endif

#endif /* BMI088_H */
