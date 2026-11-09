/**
 * @file board_led.h
 * @brief DJI RoboMaster Type-C onboard RGB LED (PH12/PH11/PH10 = R/G/B).
 *        Manual: IO high = LED on (board has drive circuitry on common-anode RGB).
 */
#ifndef BOARD_LED_H
#define BOARD_LED_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  BOARD_LED_R = 0,
  BOARD_LED_G = 1,
  BOARD_LED_B = 2,
} board_led_t;

void BoardLed_Init(void);
void BoardLed_Set(board_led_t led, bool on);
void BoardLed_Toggle(board_led_t led);
void BoardLed_WriteRgb(bool r, bool g, bool b);
void BoardLed_AllOff(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_LED_H */
