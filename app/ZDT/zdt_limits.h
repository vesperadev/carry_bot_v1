#ifndef ZDT_LIMITS_H
#define ZDT_LIMITS_H

#include <stdint.h>
#include "zdt_config.h"

/*
 * Soft travel limits (logical deg, same space as M<n>A / M<n>G on shell).
 * Default OFF — no clamp until enabled (future: BT-tunable limits).
 * Placeholder min/max only apply when ENABLE is 1.
 */
#ifndef ZDT_LIFT_SOFT_LIMIT_ENABLE
#define ZDT_LIFT_SOFT_LIMIT_ENABLE  0
#endif
#ifndef ZDT_LIFT_SOFT_LIMIT_MIN_DEG
#define ZDT_LIFT_SOFT_LIMIT_MIN_DEG (-120)
#endif
#ifndef ZDT_LIFT_SOFT_LIMIT_MAX_DEG
#define ZDT_LIFT_SOFT_LIMIT_MAX_DEG (120)
#endif

/* Other axes: widen or disable (0 = no clamp). */
#ifndef ZDT_TURN_SOFT_LIMIT_ENABLE
#define ZDT_TURN_SOFT_LIMIT_ENABLE  0
#endif
#ifndef ZDT_TURN_SOFT_LIMIT_MIN_DEG
#define ZDT_TURN_SOFT_LIMIT_MIN_DEG (-360)
#endif
#ifndef ZDT_TURN_SOFT_LIMIT_MAX_DEG
#define ZDT_TURN_SOFT_LIMIT_MAX_DEG (360)
#endif

typedef struct
{
  uint8_t  enable;
  int32_t  min_deg;
  int32_t  max_deg;
} ZdtSoftLimit;

static inline ZdtSoftLimit Zdt_SoftLimitForAddr(uint8_t addr)
{
  ZdtSoftLimit lim{};
  lim.enable = 0U;
  lim.min_deg = -2147483647L;
  lim.max_deg = 2147483647L;
  switch (addr)
  {
    case ZDT_ADDR_LIFT:
      lim.enable = (uint8_t)(ZDT_LIFT_SOFT_LIMIT_ENABLE != 0);
      lim.min_deg = ZDT_LIFT_SOFT_LIMIT_MIN_DEG;
      lim.max_deg = ZDT_LIFT_SOFT_LIMIT_MAX_DEG;
      break;
    case ZDT_ADDR_TURN:
      lim.enable = (uint8_t)(ZDT_TURN_SOFT_LIMIT_ENABLE != 0);
      lim.min_deg = ZDT_TURN_SOFT_LIMIT_MIN_DEG;
      lim.max_deg = ZDT_TURN_SOFT_LIMIT_MAX_DEG;
      break;
    default:
      break;
  }
  return lim;
}

/* Returns clamped target; *clamped=1 if value was reduced to the limit band. */
static inline int32_t Zdt_ClampLogicalTarget(uint8_t addr, int32_t deg, uint8_t *clamped)
{
  if (clamped != nullptr) *clamped = 0U;
  const ZdtSoftLimit lim = Zdt_SoftLimitForAddr(addr);
  if (lim.enable == 0U) return deg;
  if (deg < lim.min_deg)
  {
    if (clamped != nullptr) *clamped = 1U;
    return lim.min_deg;
  }
  if (deg > lim.max_deg)
  {
    if (clamped != nullptr) *clamped = 1U;
    return lim.max_deg;
  }
  return deg;
}

#endif /* ZDT_LIMITS_H */
