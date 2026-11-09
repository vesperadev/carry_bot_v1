/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   SPI1 for onboard BMI088 (DJI Type-C). Hand-maintained to match QGimbal.
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern SPI_HandleTypeDef hspi1;

void MX_SPI1_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */
