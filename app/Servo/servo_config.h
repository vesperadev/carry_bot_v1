#ifndef __SERVO_CONFIG_H
#define __SERVO_CONFIG_H

/* I2C1 PCA9685: PB8=SCL PB9=SDA @ 400 kHz, 7-bit addr 0x40 (HAL 0x80) */

#define SERVO_I2C_ADDR_7BIT       0x40U
#define SERVO_I2C_ADDR            (SERVO_I2C_ADDR_7BIT << 1)
#define SERVO_PWM_HZ              50.0f
#define SERVO_OSC_HZ              25000000.0f
#define SERVO_FREQ_CAL            1.0f
#define SERVO_PULSE_MIN_US        200U
#define SERVO_PULSE_MAX_US        2800U
#define SERVO_I2C_TIMEOUT_MS      50U
#define SERVO_READY_RETRIES       3U

/* Channel map (from CubeMx/I2C calibration notes) */
#define SERVO_CH_CLAMP            0U
#define SERVO_CH_TURNTABLE        1U
#define SERVO_CH_MAX              15U

/* Calibrated CH0 clamp endpoints (2026-11-08): close=1000 open=1500 */
#define SERVO_CLAMP_CLOSE_US      1000U
#define SERVO_CLAMP_OPEN_US       1500U
#define SERVO_TURNTABLE_MID_US    1500U

/* Boot: park clamp open; turntable mid. Set 0 to skip first output. */
#ifndef SERVO_BOOT_APPLY
#define SERVO_BOOT_APPLY          1
#endif

/* Optional IsDeviceReady smoke; keep on so missing wiring fails soft. */
#ifndef SERVO_REQUIRE_DEVICE
#define SERVO_REQUIRE_DEVICE      1
#endif

#define SERVO_TASK_STACK_SIZE     (256U * 4U)
#define SERVO_TASK_PRIORITY       osPriorityNormal
#define SERVO_CMD_QUEUE_LEN       8U

#endif /* __SERVO_CONFIG_H */
