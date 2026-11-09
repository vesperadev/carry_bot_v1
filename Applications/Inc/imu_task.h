/**
 * @file imu_task.h
 */
#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <stdint.h>
#include "bmi088.h"
#include "ist8310.h"
#include "wheel_ahrs.h"

#ifdef __cplusplus
extern "C" {
#endif

extern bmi088_t g_bmi088;
extern volatile uint8_t g_bmi088_last_err;
extern volatile uint32_t g_bmi088_ok_count;

extern ist8310_t g_ist8310;
extern volatile uint8_t g_ist8310_last_err;
extern volatile uint32_t g_ist8310_ok_count;

extern wheel_ahrs_state_t g_wheel_ahrs;
/** Set to 1 to request gyro bias cal from BT (gcal). Cleared by imu task. */
extern volatile uint8_t g_bmi088_cal_req;

void StartImuTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif
