#ifndef __LINK_PUBLIC_H
#define __LINK_PUBLIC_H

#include "main.h"

/* USART6 上位机串口链路：DMA+IDLE 收、DMA 发；协议层可插拔，对端可为蓝牙或树莓派 */

#ifdef __cplusplus
extern "C" {
#endif

/* FreeRTOS 任务入口，由 freertos.c 创建 */
void StartLinkTask(void *argument);

/* 发送一帧；数据会拷贝进模块内发送缓冲，调用者无需保持有效 */
HAL_StatusTypeDef Link_Send(const uint8_t *data, uint16_t len);

/* 运行时修改串口6波特率并重启接收 */
HAL_StatusTypeDef Link_SetBaudRate(uint32_t baudrate);

#ifdef __cplusplus
}
#endif

#endif /* __LINK_PUBLIC_H */
