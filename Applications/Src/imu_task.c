/**
 * @file imu_task.c
 * @brief BMI088 polling bring-up + ~1 Hz USART6 smoke line (BT path).
 *
 * On init failure: blink blue LED rapidly via BoardLed (best-effort).
 * Samples land in g_bmi088; every ~1 s a SMOKE line goes out on USART6 @ 9600
 * (app/BT DEBUG) so SerialTest can eyeball KEY / LED mode / IMU health.
 */
#include "imu_task.h"
#include "bmi088.h"
#include "board_led.h"
#include "board_task.h"
#include "bt_public.h"
#include "cmsis_os.h"

#include <stdio.h>

bmi088_t g_bmi088;
volatile uint8_t g_bmi088_last_err;
volatile uint32_t g_bmi088_ok_count;

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

static void smoke_send_line(void)
{
  char line[192];
  int n;
  int ax = (int)(g_bmi088.accel[0] * 1000.0f);
  int ay = (int)(g_bmi088.accel[1] * 1000.0f);
  int az = (int)(g_bmi088.accel[2] * 1000.0f);
  int gx = (int)(g_bmi088.gyro[0] * 1000.0f);
  int gy = (int)(g_bmi088.gyro[1] * 1000.0f);
  int gz = (int)(g_bmi088.gyro[2] * 1000.0f);
  int tc = (int)(g_bmi088.temperature_c * 10.0f); /* decidegrees C */

  n = snprintf(
      line, sizeof(line),
      "SMOKE key=%u mode=%u led=%s err=%u ok=%lu "
      "ax=%d ay=%d az=%d gx=%d gy=%d gz=%d t=%d\r\n",
      (unsigned)g_board_key, (unsigned)g_board_led_mode,
      led_hint(g_board_key, g_board_led_mode),
      (unsigned)g_bmi088_last_err, (unsigned long)g_bmi088_ok_count, ax, ay, az, gx,
      gy, gz, tc);

  if (n > 0 && n < (int)sizeof(line)) {
    (void)BT_Send((const uint8_t *)line, (uint16_t)n);
  }
}

void StartImuTask(void *argument)
{
  bmi088_err_t err;
  uint32_t sample = 0;
  (void)argument;

  BMI088_StructInit(&g_bmi088);

  for (;;) {
    err = BMI088_Init(&g_bmi088);
    g_bmi088_last_err = (uint8_t)err;
    if (err == BMI088_OK) {
      break;
    }
    /* Fail pulse: blue toggle while retrying */
    BoardLed_Toggle(BOARD_LED_B);
    osDelay(200);
  }

  {
    static const char ready[] =
        "SMOKE ready USART6 9600 8N1 (key/mode/err/ok + imu milli)\r\n";
    (void)BT_Send((const uint8_t *)ready, (uint16_t)(sizeof(ready) - 1u));
  }

  for (;;) {
    BMI088_Read(&g_bmi088);
    g_bmi088_ok_count++;
    sample++;
    /* ~100 Hz poll; print once per second */
    if ((sample % 100u) == 0u) {
      smoke_send_line();
    }
    osDelay(10);
  }
}