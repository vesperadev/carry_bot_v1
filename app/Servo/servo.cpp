#include "servo_public.h"
#include "servo_config.h"
#include "pca9685.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include <cstring>

namespace {

struct ServoCmd
{
  uint8_t  ch;
  uint16_t us;
  uint8_t  kind; /* 0=pulse us, 1=raw pwm uses on/off in us/on packed - keep simple: pulse only */
};

uint16_t s_last_us[16];
volatile uint8_t s_ready = 0U;
QueueHandle_t s_cmd_q = nullptr;

HAL_StatusTypeDef apply_pulse(uint8_t ch, uint16_t us)
{
  if (ch > SERVO_CH_MAX) return HAL_ERROR;
  HAL_StatusTypeDef st = Pca9685_SetServoUs(ch, us);
  if (st == HAL_OK)
  {
    if (us < SERVO_PULSE_MIN_US) us = SERVO_PULSE_MIN_US;
    if (us > SERVO_PULSE_MAX_US) us = SERVO_PULSE_MAX_US;
    s_last_us[ch] = us;
  }
  return st;
}

}  // namespace

extern "C" uint8_t Servo_IsReady(void)
{
  return s_ready;
}

extern "C" uint16_t Servo_GetPulseUs(uint8_t ch)
{
  if (ch > SERVO_CH_MAX) return 0U;
  return s_last_us[ch];
}

extern "C" HAL_StatusTypeDef Servo_SetPulseUs(uint8_t ch, uint16_t us)
{
  if (!s_ready) return HAL_ERROR;
  if (ch > SERVO_CH_MAX) return HAL_ERROR;

  /* Prefer queue so ISR/Bridge never block on I2C; fall back to direct if queue full path from same prio. */
  if (s_cmd_q != nullptr && xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
    ServoCmd cmd{};
    cmd.ch = ch;
    cmd.us = us;
    cmd.kind = 0U;
    if (xQueueSend(s_cmd_q, &cmd, pdMS_TO_TICKS(SERVO_I2C_TIMEOUT_MS)) == pdTRUE)
      return HAL_OK;
    return HAL_BUSY;
  }
  return apply_pulse(ch, us);
}

extern "C" HAL_StatusTypeDef Servo_SetClampUs(uint16_t us)
{
  return Servo_SetPulseUs(SERVO_CH_CLAMP, us);
}

extern "C" HAL_StatusTypeDef Servo_SetTurntableUs(uint16_t us)
{
  return Servo_SetPulseUs(SERVO_CH_TURNTABLE, us);
}

extern "C" HAL_StatusTypeDef Servo_ClampClose(void)
{
  return Servo_SetClampUs(SERVO_CLAMP_CLOSE_US);
}

extern "C" HAL_StatusTypeDef Servo_ClampOpen(void)
{
  return Servo_SetClampUs(SERVO_CLAMP_OPEN_US);
}

extern "C" HAL_StatusTypeDef Servo_TurntableMid(void)
{
  return Servo_SetTurntableUs(SERVO_TURNTABLE_MID_US);
}

extern "C" HAL_StatusTypeDef Servo_SetPWM(uint8_t ch, uint16_t on, uint16_t off)
{
  if (!s_ready) return HAL_ERROR;
  return Pca9685_SetPWM(ch, on, off);
}

extern "C" void StartServoTask(void *argument)
{
  (void)argument;
  std::memset(s_last_us, 0, sizeof(s_last_us));
  s_cmd_q = xQueueCreate(SERVO_CMD_QUEUE_LEN, sizeof(ServoCmd));

  if (Pca9685_Init(SERVO_PWM_HZ) != HAL_OK)
  {
    s_ready = 0U;
    for (;;)
    {
      /* Soft-fail: keep task alive so Bridge can poll Servo_IsReady()==0 */
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  s_ready = 1U;

#if SERVO_BOOT_APPLY
  (void)apply_pulse(SERVO_CH_CLAMP, SERVO_CLAMP_OPEN_US);
  (void)apply_pulse(SERVO_CH_TURNTABLE, SERVO_TURNTABLE_MID_US);
#endif

  for (;;)
  {
    ServoCmd cmd{};
    if (xQueueReceive(s_cmd_q, &cmd, portMAX_DELAY) == pdTRUE)
    {
      if (cmd.kind == 0U)
        (void)apply_pulse(cmd.ch, cmd.us);
    }
  }
}