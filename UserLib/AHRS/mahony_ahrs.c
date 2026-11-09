/**
 * @file mahony_ahrs.c
 * @brief Mahony AHRS (public-domain algorithm), C port for wheel chassis.
 */
#include "mahony_ahrs.h"
#include <math.h>

static float inv_sqrt(float x)
{
  return 1.0f / sqrtf(x);
}

void Mahony_Init(mahony_ahrs_t *a, float sample_hz)
{
  if (a == 0) {
    return;
  }
  a->sample_hz = (sample_hz > 1.0f) ? sample_hz : 100.0f;
  a->twoKp = 1.0f;
  a->twoKi = 0.0f;
  Mahony_Reset(a);
}

void Mahony_Reset(mahony_ahrs_t *a)
{
  if (a == 0) {
    return;
  }
  a->q0 = 1.0f;
  a->q1 = 0.0f;
  a->q2 = 0.0f;
  a->q3 = 0.0f;
  a->integralFBx = 0.0f;
  a->integralFBy = 0.0f;
  a->integralFBz = 0.0f;
}

void Mahony_UpdateIMU(mahony_ahrs_t *a, float gx, float gy, float gz,
                      float ax, float ay, float az)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;
  float invSample;

  if (a == 0) {
    return;
  }
  invSample = 1.0f / a->sample_hz;

  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    halfvx = a->q1 * a->q3 - a->q0 * a->q2;
    halfvy = a->q0 * a->q1 + a->q2 * a->q3;
    halfvz = a->q0 * a->q0 - 0.5f + a->q3 * a->q3;

    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    if (a->twoKi > 0.0f) {
      a->integralFBx += a->twoKi * halfex * invSample;
      a->integralFBy += a->twoKi * halfey * invSample;
      a->integralFBz += a->twoKi * halfez * invSample;
      gx += a->integralFBx;
      gy += a->integralFBy;
      gz += a->integralFBz;
    } else {
      a->integralFBx = 0.0f;
      a->integralFBy = 0.0f;
      a->integralFBz = 0.0f;
    }

    gx += a->twoKp * halfex;
    gy += a->twoKp * halfey;
    gz += a->twoKp * halfez;
  }

  gx *= (0.5f * invSample);
  gy *= (0.5f * invSample);
  gz *= (0.5f * invSample);
  qa = a->q0;
  qb = a->q1;
  qc = a->q2;
  a->q0 += (-qb * gx - qc * gy - a->q3 * gz);
  a->q1 += (qa * gx + qc * gz - a->q3 * gy);
  a->q2 += (qa * gy - qb * gz + a->q3 * gx);
  a->q3 += (qa * gz + qb * gy - qc * gx);

  recipNorm = inv_sqrt(a->q0 * a->q0 + a->q1 * a->q1 + a->q2 * a->q2 + a->q3 * a->q3);
  a->q0 *= recipNorm;
  a->q1 *= recipNorm;
  a->q2 *= recipNorm;
  a->q3 *= recipNorm;
}

void Mahony_Update(mahony_ahrs_t *a, float gx, float gy, float gz,
                   float ax, float ay, float az, float mx, float my, float mz)
{
  float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float hx, hy, bx, bz;
  float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
  float halfex, halfey, halfez;
  float qa, qb, qc;
  float invSample;

  if (a == 0) {
    return;
  }
  if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    Mahony_UpdateIMU(a, gx, gy, gz, ax, ay, az);
    return;
  }

  invSample = 1.0f / a->sample_hz;

  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    recipNorm = inv_sqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    q0q0 = a->q0 * a->q0;
    q0q1 = a->q0 * a->q1;
    q0q2 = a->q0 * a->q2;
    q0q3 = a->q0 * a->q3;
    q1q1 = a->q1 * a->q1;
    q1q2 = a->q1 * a->q2;
    q1q3 = a->q1 * a->q3;
    q2q2 = a->q2 * a->q2;
    q2q3 = a->q2 * a->q3;
    q3q3 = a->q3 * a->q3;

    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrtf(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

    halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
    halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
    halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

    if (a->twoKi > 0.0f) {
      a->integralFBx += a->twoKi * halfex * invSample;
      a->integralFBy += a->twoKi * halfey * invSample;
      a->integralFBz += a->twoKi * halfez * invSample;
      gx += a->integralFBx;
      gy += a->integralFBy;
      gz += a->integralFBz;
    } else {
      a->integralFBx = 0.0f;
      a->integralFBy = 0.0f;
      a->integralFBz = 0.0f;
    }

    gx += a->twoKp * halfex;
    gy += a->twoKp * halfey;
    gz += a->twoKp * halfez;
  }

  gx *= (0.5f * invSample);
  gy *= (0.5f * invSample);
  gz *= (0.5f * invSample);
  qa = a->q0;
  qb = a->q1;
  qc = a->q2;
  a->q0 += (-qb * gx - qc * gy - a->q3 * gz);
  a->q1 += (qa * gx + qc * gz - a->q3 * gy);
  a->q2 += (qa * gy - qb * gz + a->q3 * gx);
  a->q3 += (qa * gz + qb * gy - qc * gx);

  recipNorm = inv_sqrt(a->q0 * a->q0 + a->q1 * a->q1 + a->q2 * a->q2 + a->q3 * a->q3);
  a->q0 *= recipNorm;
  a->q1 *= recipNorm;
  a->q2 *= recipNorm;
  a->q3 *= recipNorm;
}

void Mahony_GetEulerDeg(const mahony_ahrs_t *a, float *roll_deg, float *pitch_deg,
                        float *yaw_deg)
{
  float roll, pitch, yaw;
  if (a == 0) {
    return;
  }
  roll = atan2f(a->q0 * a->q1 + a->q2 * a->q3, 0.5f - a->q1 * a->q1 - a->q2 * a->q2);
  pitch = asinf(-2.0f * (a->q1 * a->q3 - a->q0 * a->q2));
  yaw = atan2f(a->q1 * a->q2 + a->q0 * a->q3, 0.5f - a->q2 * a->q2 - a->q3 * a->q3);
  if (roll_deg) {
    *roll_deg = roll * 57.2957795f;
  }
  if (pitch_deg) {
    *pitch_deg = pitch * 57.2957795f;
  }
  if (yaw_deg) {
    *yaw_deg = yaw * 57.2957795f;
  }
}
