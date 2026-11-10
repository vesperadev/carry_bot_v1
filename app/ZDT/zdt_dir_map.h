#ifndef ZDT_DIR_MAP_H
#define ZDT_DIR_MAP_H

#include <stdint.h>
#include "zdt_config.h"

/*
 * Board motor direction map (field cal Nov 2026):
 *   MxG+ / test-forward should move the vehicle forward.
 *   M3 FL G+ was backward -> invert
 *   M4 RL G+ was backward -> invert
 *   M5 FR G+ was forward  -> keep
 *   M6 RR G+ was forward  -> keep
 * Sign: +1 keep wiring dir, -1 flip before Emm encode.
 */
#ifndef ZDT_DIR_SIGN_LIFT
#define ZDT_DIR_SIGN_LIFT   (+1)
#endif
#ifndef ZDT_DIR_SIGN_TURN
#define ZDT_DIR_SIGN_TURN   (+1)
#endif
#ifndef ZDT_DIR_SIGN_FL
#define ZDT_DIR_SIGN_FL     (-1)
#endif
#ifndef ZDT_DIR_SIGN_RL
#define ZDT_DIR_SIGN_RL     (-1)
#endif
#ifndef ZDT_DIR_SIGN_FR
#define ZDT_DIR_SIGN_FR     (+1)
#endif
#ifndef ZDT_DIR_SIGN_RR
#define ZDT_DIR_SIGN_RR     (+1)
#endif

static inline int8_t Zdt_DirSign(uint8_t addr)
{
  switch (addr)
  {
    case ZDT_ADDR_LIFT:     return (int8_t)ZDT_DIR_SIGN_LIFT;
    case ZDT_ADDR_TURN:     return (int8_t)ZDT_DIR_SIGN_TURN;
    case ZDT_ADDR_WHEEL_FL: return (int8_t)ZDT_DIR_SIGN_FL;
    case ZDT_ADDR_WHEEL_RL: return (int8_t)ZDT_DIR_SIGN_RL;
    case ZDT_ADDR_WHEEL_FR: return (int8_t)ZDT_DIR_SIGN_FR;
    case ZDT_ADDR_WHEEL_RR: return (int8_t)ZDT_DIR_SIGN_RR;
    default:                return (int8_t)(+1);
  }
}

static inline uint8_t Zdt_ApplyDirBit(uint8_t addr, uint8_t dir)
{
  const uint8_t d = dir ? 1U : 0U;
  if (Zdt_DirSign(addr) < 0) return (uint8_t)(d ^ 1U);
  return d;
}

static inline int32_t Zdt_ApplyDirDeg(uint8_t addr, int32_t deg)
{
  if (Zdt_DirSign(addr) < 0) return -deg;
  return deg;
}

/* Emm 0xFD wire delta: dir=0 increases 0x36 counts on this bus (lift MA verified). */
static inline uint8_t Zdt_PosWireDirBit(uint8_t addr, int32_t wire_delta)
{
  if (wire_delta == 0) return 0U;
  uint8_t up = (wire_delta > 0) ? 1U : 0U;
  up = Zdt_ApplyDirBit(addr, up);
  return (uint8_t)(up ^ 1U);
}

#endif /* ZDT_DIR_MAP_H */
