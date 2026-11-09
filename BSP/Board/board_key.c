/**
 * @file board_key.c
 */
#include "board_key.h"
#include "main.h"

static uint8_t s_stable;
static uint8_t s_prev;
static uint8_t s_clicked;
static uint8_t s_debounce;

void BoardKey_Init(void)
{
  GPIO_InitTypeDef cfg = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  cfg.Pin = KEY_Pin;
  cfg.Mode = GPIO_MODE_INPUT;
  cfg.Pull = GPIO_PULLUP; /* board also has 10k; keep internal as backup */
  HAL_GPIO_Init(KEY_GPIO_Port, &cfg);

  s_stable = 0;
  s_prev = 0;
  s_clicked = 0;
  s_debounce = 0;
}

bool BoardKey_IsPressed(void)
{
  return HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET;
}

void BoardKey_Scan(void)
{
  const uint8_t raw = BoardKey_IsPressed() ? 1u : 0u;

  if (raw != s_stable) {
    s_debounce++;
    if (s_debounce >= 2u) { /* ~20 ms at 10 ms scan */
      s_stable = raw;
      s_debounce = 0;
      if (s_stable && !s_prev) {
        s_clicked = 1u;
      }
      s_prev = s_stable;
    }
  } else {
    s_debounce = 0;
  }
}

bool BoardKey_WasClicked(void)
{
  if (s_clicked) {
    s_clicked = 0;
    return true;
  }
  return false;
}
