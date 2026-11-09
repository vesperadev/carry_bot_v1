/**
 * @file board_led.c
 */
#include "board_led.h"
#include "main.h"

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
} board_led_pin_t;

static const board_led_pin_t kLeds[3] = {
  {LED_R_GPIO_Port, LED_R_Pin},
  {LED_G_GPIO_Port, LED_G_Pin},
  {LED_B_GPIO_Port, LED_B_Pin},
};

void BoardLed_Init(void)
{
  GPIO_InitTypeDef cfg = {0};

  __HAL_RCC_GPIOH_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOH, LED_R_Pin | LED_G_Pin | LED_B_Pin, GPIO_PIN_RESET);

  cfg.Pin = LED_R_Pin | LED_G_Pin | LED_B_Pin;
  cfg.Mode = GPIO_MODE_OUTPUT_PP;
  cfg.Pull = GPIO_NOPULL;
  cfg.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &cfg);

  BoardLed_AllOff();
}

void BoardLed_Set(board_led_t led, bool on)
{
  if ((unsigned)led > BOARD_LED_B) {
    return;
  }
  HAL_GPIO_WritePin(kLeds[led].port, kLeds[led].pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void BoardLed_Toggle(board_led_t led)
{
  if ((unsigned)led > BOARD_LED_B) {
    return;
  }
  HAL_GPIO_TogglePin(kLeds[led].port, kLeds[led].pin);
}

void BoardLed_WriteRgb(bool r, bool g, bool b)
{
  BoardLed_Set(BOARD_LED_R, r);
  BoardLed_Set(BOARD_LED_G, g);
  BoardLed_Set(BOARD_LED_B, b);
}

void BoardLed_AllOff(void)
{
  BoardLed_WriteRgb(false, false, false);
}
