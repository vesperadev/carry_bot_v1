#ifndef __ZDT_CONFIG_H
#define __ZDT_CONFIG_H

/* USART1 Emm V5 motor bus: 115200 8N1, DMA+IDLE RX, DMA TX */

#define ZDT_BAUDRATE            115200U
#define ZDT_RX_BUF_SIZE         256U
#define ZDT_FRAME_MAX           64U
#define ZDT_TX_BUF_SIZE         64U
#define ZDT_QUEUE_LEN           8U
#define ZDT_SLOT_MAX            8U
#define ZDT_PULSE_PER_REV       3200U
#define ZDT_ACK_TIMEOUT_MS      500U
#define ZDT_DEFAULT_VEL_RPM     300U
#define ZDT_DEFAULT_ACC         100U

/* Address map (from USART reference) */
#define ZDT_ADDR_BROADCAST      0U
#define ZDT_ADDR_LIFT           1U
#define ZDT_ADDR_TURN           2U
#define ZDT_ADDR_WHEEL_FL       3U
#define ZDT_ADDR_WHEEL_RL       4U
#define ZDT_ADDR_WHEEL_FR       5U
#define ZDT_ADDR_WHEEL_RR       6U

/* Optional boot smoke: ME1 + MG+1 and wait 02 ACK on ZDT_SMOKE_ADDR */
#ifndef ZDT_SMOKE_ENABLE
#define ZDT_SMOKE_ENABLE        0
#endif
#ifndef ZDT_SMOKE_ADDR
#define ZDT_SMOKE_ADDR          ZDT_ADDR_LIFT
#endif

/* For Core/Src/freertos.c (include cmsis_os.h first) */
#define ZDT_TASK_STACK_SIZE     (256U * 4U)
#define ZDT_TASK_PRIORITY       osPriorityNormal

#endif /* __ZDT_CONFIG_H */
