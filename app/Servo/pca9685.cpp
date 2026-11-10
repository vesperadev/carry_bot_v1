#include "pca9685.h"
#include "servo_config.h"
#include "i2c.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

namespace {

constexpr uint8_t kMode1     = 0x00U;
constexpr uint8_t kPrescale  = 0xFEU;
constexpr uint8_t kLed0OnL   = 0x06U;

float s_freq_hz = SERVO_PWM_HZ;
SemaphoreHandle_t s_bus_mtx = nullptr;
SemaphoreHandle_t s_xfer_done = nullptr;
volatile HAL_StatusTypeDef s_xfer_st = HAL_OK;

bool lock_bus(TickType_t ticks)
{
  if (s_bus_mtx == nullptr) return false;
  return xSemaphoreTake(s_bus_mtx, ticks) == pdTRUE;
}

void unlock_bus()
{
  if (s_bus_mtx != nullptr) (void)xSemaphoreGive(s_bus_mtx);
}

HAL_StatusTypeDef wait_xfer(TickType_t ticks)
{
  if (s_xfer_done == nullptr) return HAL_ERROR;
  if (xSemaphoreTake(s_xfer_done, ticks) != pdTRUE) return HAL_TIMEOUT;
  return s_xfer_st;
}

HAL_StatusTypeDef mem_write(uint8_t reg, const uint8_t *data, uint16_t len)
{
  if (data == nullptr || len == 0U) return HAL_ERROR;
  if (!lock_bus(pdMS_TO_TICKS(SERVO_I2C_TIMEOUT_MS))) return HAL_BUSY;

  while (xSemaphoreTake(s_xfer_done, 0) == pdTRUE) { /* drain */ }
  s_xfer_st = HAL_OK;

  HAL_StatusTypeDef st = HAL_I2C_Mem_Write_DMA(
      &hi2c2, SERVO_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
      const_cast<uint8_t *>(data), len);
  if (st != HAL_OK)
  {
    unlock_bus();
    return st;
  }

  st = wait_xfer(pdMS_TO_TICKS(SERVO_I2C_TIMEOUT_MS));
  if (st == HAL_TIMEOUT)
  {
    (void)HAL_I2C_Master_Abort_IT(&hi2c2, SERVO_I2C_ADDR);
    unlock_bus();
    return HAL_TIMEOUT;
  }
  unlock_bus();
  return st;
}

HAL_StatusTypeDef mem_read(uint8_t reg, uint8_t *data, uint16_t len)
{
  if (data == nullptr || len == 0U) return HAL_ERROR;
  if (!lock_bus(pdMS_TO_TICKS(SERVO_I2C_TIMEOUT_MS))) return HAL_BUSY;

  while (xSemaphoreTake(s_xfer_done, 0) == pdTRUE) { /* drain */ }
  s_xfer_st = HAL_OK;

  HAL_StatusTypeDef st = HAL_I2C_Mem_Read_DMA(
      &hi2c2, SERVO_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len);
  if (st != HAL_OK)
  {
    unlock_bus();
    return st;
  }

  st = wait_xfer(pdMS_TO_TICKS(SERVO_I2C_TIMEOUT_MS));
  if (st == HAL_TIMEOUT)
  {
    (void)HAL_I2C_Master_Abort_IT(&hi2c2, SERVO_I2C_ADDR);
    unlock_bus();
    return HAL_TIMEOUT;
  }
  unlock_bus();
  return st;
}

HAL_StatusTypeDef reg_write(uint8_t reg, uint8_t val)
{
  return mem_write(reg, &val, 1U);
}

HAL_StatusTypeDef reg_read(uint8_t reg, uint8_t *val)
{
  return mem_read(reg, val, 1U);
}

}  // namespace

extern "C" void Pca9685_BusInit(void)
{
  if (s_bus_mtx == nullptr)
    s_bus_mtx = xSemaphoreCreateMutex();
  if (s_xfer_done == nullptr)
    s_xfer_done = xSemaphoreCreateBinary();
}

float Pca9685_GetFreq(void)
{
  return s_freq_hz;
}

HAL_StatusTypeDef Pca9685_SetFreq(float freq_hz)
{
  if (freq_hz <= 0.0f) return HAL_ERROR;

  float p = SERVO_OSC_HZ / 4096.0f / (freq_hz * SERVO_FREQ_CAL) - 1.0f;
  if (p < 3.0f) p = 3.0f;
  if (p > 255.0f) p = 255.0f;
  const uint8_t prescale = static_cast<uint8_t>(p + 0.5f);

  uint8_t oldmode = 0U;
  HAL_StatusTypeDef st = reg_read(kMode1, &oldmode);
  if (st != HAL_OK) return st;

  const uint8_t sleepmode = static_cast<uint8_t>((oldmode & 0x7FU) | 0x10U);
  if ((st = reg_write(kMode1, sleepmode)) != HAL_OK) return st;
  if ((st = reg_write(kPrescale, prescale)) != HAL_OK) return st;
  if ((st = reg_write(kMode1, oldmode)) != HAL_OK) return st;
  vTaskDelay(pdMS_TO_TICKS(5));
  st = reg_write(kMode1, static_cast<uint8_t>(oldmode | 0xA1U)); /* RESTART|AI|ALLCALL */
  if (st == HAL_OK) s_freq_hz = freq_hz;
  return st;
}

HAL_StatusTypeDef Pca9685_SetPWM(uint8_t ch, uint16_t on, uint16_t off)
{
  if (ch > SERVO_CH_MAX) return HAL_ERROR;
  uint8_t buf[4] = {
      static_cast<uint8_t>(on),
      static_cast<uint8_t>(on >> 8),
      static_cast<uint8_t>(off),
      static_cast<uint8_t>(off >> 8),
  };
  return mem_write(static_cast<uint8_t>(kLed0OnL + 4U * ch), buf, 4U);
}

HAL_StatusTypeDef Pca9685_SetServoUs(uint8_t ch, uint16_t us)
{
  if (us < SERVO_PULSE_MIN_US) us = SERVO_PULSE_MIN_US;
  if (us > SERVO_PULSE_MAX_US) us = SERVO_PULSE_MAX_US;
  uint32_t counts = static_cast<uint32_t>(
      static_cast<float>(us) * 4096.0f * s_freq_hz / 1000000.0f + 0.5f);
  if (counts > 4095U) counts = 4095U;
  return Pca9685_SetPWM(ch, 0U, static_cast<uint16_t>(counts));
}

HAL_StatusTypeDef Pca9685_Init(float freq_hz)
{
  Pca9685_BusInit();
  if (s_bus_mtx == nullptr || s_xfer_done == nullptr) return HAL_ERROR;

#if SERVO_REQUIRE_DEVICE
  if (HAL_I2C_IsDeviceReady(&hi2c2, SERVO_I2C_ADDR, SERVO_READY_RETRIES,
                            SERVO_I2C_TIMEOUT_MS) != HAL_OK)
    return HAL_ERROR;
#endif

  HAL_StatusTypeDef st = reg_write(kMode1, 0x00U);
  if (st != HAL_OK) return st;
  vTaskDelay(pdMS_TO_TICKS(1));
  return Pca9685_SetFreq(freq_hz);
}

extern "C" void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == nullptr || hi2c->Instance != I2C2) return;
  s_xfer_st = HAL_OK;
  if (s_xfer_done != nullptr)
  {
    BaseType_t woken = pdFALSE;
    (void)xSemaphoreGiveFromISR(s_xfer_done, &woken);
    portYIELD_FROM_ISR(woken);
  }
}

extern "C" void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == nullptr || hi2c->Instance != I2C2) return;
  s_xfer_st = HAL_OK;
  if (s_xfer_done != nullptr)
  {
    BaseType_t woken = pdFALSE;
    (void)xSemaphoreGiveFromISR(s_xfer_done, &woken);
    portYIELD_FROM_ISR(woken);
  }
}

extern "C" void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == nullptr || hi2c->Instance != I2C2) return;
  s_xfer_st = HAL_ERROR;
  if (s_xfer_done != nullptr)
  {
    BaseType_t woken = pdFALSE;
    (void)xSemaphoreGiveFromISR(s_xfer_done, &woken);
    portYIELD_FROM_ISR(woken);
  }
}