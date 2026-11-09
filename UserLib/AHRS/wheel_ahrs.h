/**
 * @file wheel_ahrs.h
 * @brief Chassis attitude interface: BMI088 + IST8310 -> Mahony Euler.
 */
#ifndef WHEEL_AHRS_H
#define WHEEL_AHRS_H

#include <stdint.h>
#include <stdbool.h>
#include "bmi088.h"
#include "ist8310.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float roll_deg;
  float pitch_deg;
  float yaw_deg;
  float q[4];
  bool mag_used;
  uint32_t update_count;
} wheel_ahrs_state_t;

void WheelAhrs_Init(float sample_hz);
/** Fuse one IMU (+ optional mag) sample. Gyro rad/s, accel m/s^2-ish, mag uT. */
void WheelAhrs_Update(const bmi088_t *imu, const ist8310_t *mag, bool use_mag);
void WheelAhrs_Get(wheel_ahrs_state_t *out);

#ifdef __cplusplus
}
#endif

#endif /* WHEEL_AHRS_H */
