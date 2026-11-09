/**
 * @file imu_task.c
 * @brief BMI088 + IST8310 + Mahony wheel AHRS; ~1 Hz lines on 串口1 (USART6@9600).
 *
 * SMOKE: raw IMU/mag. AHRS: roll/pitch/yaw deg (+ mag flag).
 */
#include "imu_task.h"
#include "bmi088.h"
#include "ist8310.h"
#include "wheel_ahrs.h"
#include "board_led.h"
#include "board_task.h"
#include "bt_public.h"
#include "cmsis_os.h"

#include <stdio.h>

bmi088_t g_bmi088;
volatile uint8_t g_bmi088_last_err;
volatile uint32_t g_bmi088_ok_count;

ist8310_t g_ist8310;
volatile uint8_t g_ist8310_last_err;
volatile uint32_t g_ist8310_ok_count;

wheel_ahrs_state_t g_wheel_ahrs;
volatile uint8_t g_bmi088_cal_req;

static const char *led_hint(uint8_t key, uint8_t mode)
{
  if (key != 0u) {
    return "RED";
  }
  switch (mode) {
  case 1u: return "R";
  case 2u: return "G";
  case 3u: return "B";
  default: return "HB";
  }
}

static void gyrocal_fail_msg(const char *prefix, bmi088_err_t err)
{
  const char *tail;
  if (err == BMI088_ERR_GYRO_MOTION) {
    tail = "MOTION — keep still and retry\r\n";
  } else {
    tail = "FAIL\r\n";
  }
  char line[64];
  int n = snprintf(line, sizeof(line), "%s %s", prefix, tail);
  if (n > 0 && n < (int)sizeof(line)) {
    (void)BT_Send((const uint8_t *)line, (uint16_t)n);
  }
}

static void smoke_send_line(void)
{
  char line[280];
  int n;
  int ax = (int)(g_bmi088.accel[0] * 1000.0f);
  int ay = (int)(g_bmi088.accel[1] * 1000.0f);
  int az = (int)(g_bmi088.accel[2] * 1000.0f);
  int gx = (int)(g_bmi088.gyro[0] * 1000.0f);
  int gy = (int)(g_bmi088.gyro[1] * 1000.0f);
  int gz = (int)(g_bmi088.gyro[2] * 1000.0f);
  int tc = (int)(g_bmi088.temperature_c * 10.0f);

  n = snprintf(
      line, sizeof(line),
      "SMOKE key=%u mode=%u led=%s err=%u ok=%lu "
      "ax=%d ay=%d az=%d gx=%d gy=%d gz=%d t=%d "
      "merr=%u mok=%lu mrdy=%u mx=%d my=%d mz=%d\r\n",
      (unsigned)g_board_key, (unsigned)g_board_led_mode,
      led_hint(g_board_key, g_board_led_mode),
      (unsigned)g_bmi088_last_err, (unsigned long)g_bmi088_ok_count, ax, ay, az, gx,
      gy, gz, tc, (unsigned)g_ist8310_last_err, (unsigned long)g_ist8310_ok_count,
      (unsigned)(g_ist8310.mag_ready ? 1u : 0u),
      (int)g_ist8310.raw_mx, (int)g_ist8310.raw_my, (int)g_ist8310.raw_mz);

  if (n > 0 && n < (int)sizeof(line)) {
    (void)BT_Send((const uint8_t *)line, (uint16_t)n);
  }
}

static void ahrs_send_line(void)
{
  char line[160];
  int n;
  int r = (int)(g_wheel_ahrs.roll_deg * 10.0f);
  int p = (int)(g_wheel_ahrs.pitch_deg * 10.0f);
  int y = (int)(g_wheel_ahrs.yaw_deg * 10.0f);

  n = snprintf(
      line, sizeof(line),
      "AHRS r=%d p=%d y=%d mag=%u n=%lu\r\n",
      r, p, y, (unsigned)(g_wheel_ahrs.mag_used ? 1u : 0u),
      (unsigned long)g_wheel_ahrs.update_count);
  if (n > 0 && n < (int)sizeof(line)) {
    (void)BT_Send((const uint8_t *)line, (uint16_t)n);
  }
}

void StartImuTask(void *argument)
{
  bmi088_err_t err;
  ist8310_err_t merr;
  uint32_t sample = 0;
  (void)argument;

  BMI088_StructInit(&g_bmi088);
  IST8310_StructInit(&g_ist8310);
  WheelAhrs_Init(100.0f);

  for (;;) {
    err = BMI088_Init(&g_bmi088);
    g_bmi088_last_err = (uint8_t)err;
    if (err == BMI088_OK) {
      break;
    }
    BoardLed_Toggle(BOARD_LED_B);
    osDelay(200);
  }

  merr = IST8310_Init(&g_ist8310);
  g_ist8310_last_err = (uint8_t)merr;

  {
    static const char ready[] =
        "AHRS ready 串口1@9600 (SMOKE+AHRS; send gcal for gyro bias)\r\n";
    (void)BT_Send((const uint8_t *)ready, (uint16_t)(sizeof(ready) - 1u));
  }

  /* Boot gyro bias: keep board still ~1 s (200 x 5 ms). */
  {
    static const char cal_start[] = "GYROCAL boot start — keep still\r\n";
    char cal_done[96];
    int n;
    bmi088_err_t cal_err;
    (void)BT_Send((const uint8_t *)cal_start, (uint16_t)(sizeof(cal_start) - 1u));
    cal_err = BMI088_CalibrateGyro(&g_bmi088, 200u, 5u);
    if (cal_err != BMI088_OK) {
      gyrocal_fail_msg("GYROCAL boot", cal_err);
    } else {
      n = snprintf(cal_done, sizeof(cal_done),
                   "GYROCAL boot OK bx=%d by=%d bz=%d (mrad/s)\r\n",
                   (int)(g_bmi088.gyro_bias[0] * 1000.0f),
                   (int)(g_bmi088.gyro_bias[1] * 1000.0f),
                   (int)(g_bmi088.gyro_bias[2] * 1000.0f));
      if (n > 0) {
        (void)BT_Send((const uint8_t *)cal_done, (uint16_t)n);
      }
      WheelAhrs_Init(100.0f);
    }
  }

  for (;;) {
    if (g_bmi088_cal_req != 0u) {
      static const char cal_start[] = "GYROCAL cmd start — keep still\r\n";
      char cal_done[96];
      int n;
      bmi088_err_t cal_err;
      g_bmi088_cal_req = 0u;
      (void)BT_Send((const uint8_t *)cal_start, (uint16_t)(sizeof(cal_start) - 1u));
      cal_err = BMI088_CalibrateGyro(&g_bmi088, 200u, 5u);
      if (cal_err != BMI088_OK) {
        gyrocal_fail_msg("GYROCAL cmd", cal_err);
      } else {
        n = snprintf(cal_done, sizeof(cal_done),
                     "GYROCAL cmd OK bx=%d by=%d bz=%d (mrad/s)\r\n",
                     (int)(g_bmi088.gyro_bias[0] * 1000.0f),
                     (int)(g_bmi088.gyro_bias[1] * 1000.0f),
                     (int)(g_bmi088.gyro_bias[2] * 1000.0f));
        if (n > 0) {
          (void)BT_Send((const uint8_t *)cal_done, (uint16_t)n);
        }
        WheelAhrs_Init(100.0f);
      }
    }

    BMI088_Read(&g_bmi088);
    g_bmi088_ok_count++;

    if (g_ist8310.initialized) {
      merr = IST8310_Read(&g_ist8310);
      g_ist8310_last_err = (uint8_t)merr;
      if (merr == IST8310_OK) {
        g_ist8310_ok_count++;
      }
    } else if ((sample % 100u) == 0u) {
      merr = IST8310_Init(&g_ist8310);
      g_ist8310_last_err = (uint8_t)merr;
    }

    WheelAhrs_Update(&g_bmi088, &g_ist8310, true);
    WheelAhrs_Get(&g_wheel_ahrs);

    sample++;
    if ((sample % 100u) == 0u) {
      smoke_send_line();
      ahrs_send_line();
    }
    osDelay(10);
  }
}
