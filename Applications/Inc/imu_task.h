/**
 * @file imu_task.h
 */
#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <stdint.h>
#include "bmi088.h"

#ifdef __cplusplus
extern "C" {
#endif

extern bmi088_t g_bmi088;
extern volatile uint8_t g_bmi088_last_err;
extern volatile uint32_t g_bmi088_ok_count;

void StartImuTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif