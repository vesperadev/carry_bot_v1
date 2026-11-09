#ifndef __ZDT_PUBLIC_H
#define __ZDT_PUBLIC_H

#include "main.h"
#include <stdint.h>

/* USART1 -> Emm V5 motor bus: encode, address slots, DMA TX/RX */

#ifdef __cplusplus
extern "C" {
#endif

void StartZdtTask(void *argument);

HAL_StatusTypeDef Zdt_SendRaw(const uint8_t *data, uint16_t len);

/* ME1 / ME0: enable / disable. addr=0 broadcast. Does not wait for ACK. */
HAL_StatusTypeDef Zdt_Enable(uint8_t addr, uint8_t on);

/* MG+/-deg: position mode by angle (pulses = |deg| * 3200 / 360). */
HAL_StatusTypeDef Zdt_GoDeg(uint8_t addr, int32_t deg);

/* MX: immediate stop */
HAL_StatusTypeDef Zdt_Stop(uint8_t addr);

/* Velocity mode (Emm 0xF6). dir: 0/1; vel_rpm uses Emm velocity field. */
HAL_StatusTypeDef Zdt_Vel(uint8_t addr, uint8_t dir, uint16_t vel_rpm);

/* Wait for Addr+Cmd ACK with status 0x02 (OK). */
HAL_StatusTypeDef Zdt_WaitAck(uint8_t addr, uint8_t cmd, uint32_t timeout_ms);

HAL_StatusTypeDef Zdt_EnableAndWait(uint8_t addr, uint8_t on, uint32_t timeout_ms);
HAL_StatusTypeDef Zdt_GoDegAndWait(uint8_t addr, int32_t deg, uint32_t timeout_ms);
HAL_StatusTypeDef Zdt_VelAndWait(uint8_t addr, uint8_t dir, uint16_t vel_rpm, uint32_t timeout_ms);
HAL_StatusTypeDef Zdt_StopAndWait(uint8_t addr, uint32_t timeout_ms);

/* Debug: last USART1 IDLE frame as "aa bb cc" (empty string if none). */
void Zdt_ClearLastRx(void);
uint16_t Zdt_FormatLastRx(char *out, uint16_t capacity);

/* Unified UART dispatch hooks (Instance == USART1) */
void Zdt_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void Zdt_UART_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __ZDT_PUBLIC_H */
