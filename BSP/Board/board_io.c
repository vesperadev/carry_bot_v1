/**
 * @file board_io.c
 */
#include "board_io.h"
#include "board_led.h"
#include "board_key.h"
#include "main.h"

void BoardIO_Init(void)
{
  GPIO_InitTypeDef cfg = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* CS idle high (deselected) */
  HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);

  cfg.Mode = GPIO_MODE_OUTPUT_PP;
  cfg.Pull = GPIO_NOPULL;
  cfg.Speed = GPIO_SPEED_FREQ_HIGH;

  cfg.Pin = CS1_ACCEL_Pin;
  HAL_GPIO_Init(CS1_ACCEL_GPIO_Port, &cfg);

  cfg.Pin = CS1_GYRO_Pin;
  HAL_GPIO_Init(CS1_GYRO_GPIO_Port, &cfg);

  BoardLed_Init();
  BoardKey_Init();
}
