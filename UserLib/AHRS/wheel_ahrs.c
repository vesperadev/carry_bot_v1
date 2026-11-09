/**
 * @file wheel_ahrs.c
 */
#include "wheel_ahrs.h"
#include "mahony_ahrs.h"

static mahony_ahrs_t s_mahony;
static wheel_ahrs_state_t s_state;

static bool mag_vector_ok(float mx, float my, float mz)
{
  const float b2 = mx * mx + my * my + mz * mz;
  if (b2 <= 0.0f) {
    return false;
  }
  if (b2 < (20.0f * 20.0f) || b2 > (120.0f * 120.0f)) {
    return false;
  }
  return true;
}

void WheelAhrs_Init(float sample_hz)
{
  Mahony_Init(&s_mahony, sample_hz);
  s_state.roll_deg = 0.0f;
  s_state.pitch_deg = 0.0f;
  s_state.yaw_deg = 0.0f;
  s_state.q[0] = 1.0f;
  s_state.q[1] = s_state.q[2] = s_state.q[3] = 0.0f;
  s_state.mag_used = false;
  s_state.update_count = 0U;
}

void WheelAhrs_Update(const bmi088_t *imu, const ist8310_t *mag, bool use_mag)
{
  float mx = 0.0f, my = 0.0f, mz = 0.0f;
  bool mag_ok = false;

  if (imu == 0 || !imu->initialized) {
    return;
  }

  if (use_mag && mag != 0 && mag->initialized && mag->mag_ready && imu->gyro_calibrated) {
    mx = mag->mag_ut[0];
    my = mag->mag_ut[1];
    mz = mag->mag_ut[2];
    mag_ok = mag_vector_ok(mx, my, mz);
  }

  if (mag_ok) {
    Mahony_Update(&s_mahony, imu->gyro[0], imu->gyro[1], imu->gyro[2],
                  imu->accel[0], imu->accel[1], imu->accel[2], mx, my, mz);
  } else {
    Mahony_UpdateIMU(&s_mahony, imu->gyro[0], imu->gyro[1], imu->gyro[2],
                     imu->accel[0], imu->accel[1], imu->accel[2]);
  }

  Mahony_GetEulerDeg(&s_mahony, &s_state.roll_deg, &s_state.pitch_deg, &s_state.yaw_deg);
  s_state.q[0] = s_mahony.q0;
  s_state.q[1] = s_mahony.q1;
  s_state.q[2] = s_mahony.q2;
  s_state.q[3] = s_mahony.q3;
  s_state.mag_used = mag_ok;
  s_state.update_count++;
}

void WheelAhrs_Get(wheel_ahrs_state_t *out)
{
  if (out == 0) {
    return;
  }
  *out = s_state;
}
