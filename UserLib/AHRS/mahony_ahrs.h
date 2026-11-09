/**
 * @file mahony_ahrs.h
 * @brief Compact C Mahony AHRS (QGimbal-style), 6-DOF IMU + optional 9-DOF mag.
 */
#ifndef MAHONY_AHRS_H
#define MAHONY_AHRS_H

#include <stdint.h>

#ifdef __cplusplus
extern C {
#endif

typedef struct {
  float sample_hz;
  float twoKp;
  float twoKi;
  float q0, q1, q2, q3;
  float integralFBx, integralFBy, integralFBz;
} mahony_ahrs_t;

void Mahony_Init(mahony_ahrs_t *a, float sample_hz);
void Mahony_Reset(mahony_ahrs_t *a);
/** Gyro rad/s, accel any consistent unit. */
void Mahony_UpdateIMU(mahony_ahrs_t *a, float gx, float gy, float gz,
                      float ax, float ay, float az);
/** Mag in uT (or any linear unit); falls back to IMU-only if mag is zero. */
void Mahony_Update(mahony_ahrs_t *a, float gx, float gy, float gz,
                   float ax, float ay, float az, float mx, float my, float mz);
/** Euler deg: roll(X), pitch(Y), yaw(Z). */
void Mahony_GetEulerDeg(const mahony_ahrs_t *a, float *roll_deg, float *pitch_deg,
                        float *yaw_deg);

#ifdef __cplusplus
}
#endif

#endif /* MAHONY_AHRS_H */
