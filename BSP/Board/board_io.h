/**
 * @file board_io.h
 * @brief Aggregate board GPIO bring-up (LED + KEY + BMI088 CS).
 */
#ifndef BOARD_IO_H
#define BOARD_IO_H

#ifdef __cplusplus
extern "C" {
#endif

/** Configure LED / KEY / CS1_ACCEL / CS1_GYRO pins. Call after MX_GPIO_Init clocks. */
void BoardIO_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_IO_H */
