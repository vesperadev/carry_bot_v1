#ifndef __PCA9685_H
#define __PCA9685_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

HAL_StatusTypeDef Pca9685_Init(float freq_hz);
HAL_StatusTypeDef Pca9685_SetFreq(float freq_hz);
HAL_StatusTypeDef Pca9685_SetPWM(uint8_t ch, uint16_t on, uint16_t off);
HAL_StatusTypeDef Pca9685_SetServoUs(uint8_t ch, uint16_t us);
float Pca9685_GetFreq(void);

#ifdef __cplusplus
}
#endif

#endif /* __PCA9685_H */
