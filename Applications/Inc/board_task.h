/**
 * @file board_task.h
 */
#ifndef BOARD_TASK_H
#define BOARD_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** LED mode: 0=heartbeat green, 1=R, 2=G, 3=B (KEY click cycles). */
extern volatile uint8_t g_board_led_mode;
/** 1 while KEY held (active-low PA0). */
extern volatile uint8_t g_board_key;

void StartBoardTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif