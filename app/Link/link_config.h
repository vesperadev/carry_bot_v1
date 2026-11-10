#ifndef __LINK_CONFIG_H
#define __LINK_CONFIG_H

/* LINK_PEER_BT / LINK_PEER_RPI 由 app/Link/CMakeLists.txt 的 LINK_PEER 定义 */
#if defined(LINK_PEER_RPI)
#define LINK_DEFAULT_BAUDRATE   115200U
#else
#define LINK_DEFAULT_BAUDRATE   9600U
#endif

#define LINK_RX_BUF_SIZE        1024U             /* DMA 接收缓冲区 */
#define LINK_FRAME_MAX          256U              /* 单帧最大字节数 */
#define LINK_QUEUE_LEN          4U                /* 接收队列深度 */

/* 以下两项仅供 Core/Src/freertos.c 使用（需先包含 cmsis_os.h） */
#define LINK_TASK_STACK_SIZE    (512U * 4U)
#define LINK_TASK_PRIORITY      osPriorityNormal

#endif /* __LINK_CONFIG_H */
