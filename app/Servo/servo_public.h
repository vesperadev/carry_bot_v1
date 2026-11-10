#ifndef __SERVO_PUBLIC_H
#define __SERVO_PUBLIC_H

#include "main.h"
#include <stdint.h>

/* I2C2 -> PCA9685: CH0 clamp / CH1 turntable. Bridge parses S* later. */

#ifdef __cplusplus
extern "C" {
#endif

void StartServoTask(void *argument);

uint8_t Servo_IsReady(void);

/* Absolute pulse on channel 0..15; clamped to SERVO_PULSE_*_US. */
HAL_StatusTypeDef Servo_SetPulseUs(uint8_t ch, uint16_t us);

uint16_t Servo_GetPulseUs(uint8_t ch);

HAL_StatusTypeDef Servo_SetClampUs(uint16_t us);
HAL_StatusTypeDef Servo_SetTurntableUs(uint16_t us);

HAL_StatusTypeDef Servo_ClampClose(void);
HAL_StatusTypeDef Servo_ClampOpen(void);
HAL_StatusTypeDef Servo_TurntableMid(void);

/* Low-level PWM counts (on/off 0..4095 / full-on/off 4096). */
HAL_StatusTypeDef Servo_SetPWM(uint8_t ch, uint16_t on, uint16_t off);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_PUBLIC_H */
