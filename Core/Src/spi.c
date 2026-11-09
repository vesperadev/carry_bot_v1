/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   SPI1 master for BMI088 — pins match RoboMaster Type-C / QGimbal:
  *          PB3=SCK, PB4=MISO, PA7=MOSI, Mode3, soft NSS, presc=8.
  *          Polling only on this branch (no SPI DMA) so USART1/6 + I2C1 DMA
  *          streams stay untouched. Free streams for later: DMA2 S0/S3.
  ******************************************************************************
  */
/* USER CODE END Header */
#include "spi.h"

SPI_HandleTypeDef hspi1;

void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *spiHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (spiHandle->Instance != SPI1) {
    return;
  }

  __HAL_RCC_SPI1_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /**SPI1 GPIO Configuration
  PB3     ------> SPI1_SCK
  PB4     ------> SPI1_MISO
  PA7     ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *spiHandle)
{
  if (spiHandle->Instance != SPI1) {
    return;
  }
  __HAL_RCC_SPI1_CLK_DISABLE();
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3 | GPIO_PIN_4);
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_7);
}
