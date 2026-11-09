/**
 * @file board_task.c
 * @brief LED heartbeat + KEY scan on DJI Type-C board.
 *
 * - Idle: green LED toggles ~1 Hz
 * - KEY held: red LED on (overrides green blink)
 * - KEY click: cycle R -> G -> B solid briefly via mode counter
 */
#include "board_task.h"
#include "board_led.h"
#include "board_key.h"
#include "cmsis_os.h"

volatile uint8_t g_board_led_mode;
volatile uint8_t g_board_key;

void StartBoardTask(void *argument)
{
  uint32_t tick = 0;
  uint8_t mode = 0; /* 0=heartbeat green, 1=R, 2=G, 3=B */

  (void)argument;

  for (;;) {
    BoardKey_Scan();
    if (BoardKey_WasClicked()) {
      mode = (uint8_t)((mode + 1u) % 4u);
    }

    g_board_led_mode = mode;
    g_board_key = BoardKey_IsPressed() ? 1u : 0u;

    if (g_board_key != 0u) {
      BoardLed_WriteRgb(true, false, false);
    } else if (mode == 0u) {
      BoardLed_WriteRgb(false, ((tick / 50u) & 1u) != 0u, false); /* 10ms * 50 = 500ms */
    } else if (mode == 1u) {
      BoardLed_WriteRgb(true, false, false);
    } else if (mode == 2u) {
      BoardLed_WriteRgb(false, true, false);
    } else {
      BoardLed_WriteRgb(false, false, true);
    }

    tick++;
    osDelay(10);
  }
}