/**
 * @file board_key.h
 * @brief DJI Type-C user KEY on PA0 (active low when pressed, external 10k pull-up).
 */
#ifndef BOARD_KEY_H
#define BOARD_KEY_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void BoardKey_Init(void);
bool BoardKey_IsPressed(void);   /* raw level, pressed = true */
bool BoardKey_WasClicked(void);  /* edge after BoardKey_Scan */
void BoardKey_Scan(void);        /* call periodically (~10 ms) */

#ifdef __cplusplus
}
#endif

#endif /* BOARD_KEY_H */
