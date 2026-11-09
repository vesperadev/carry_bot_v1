#ifndef __BRIDGE_CONFIG_H
#define __BRIDGE_CONFIG_H

/* BT text bridge over Link (USART6). A-tier: M?/ME/MG/MX/MS/ML/MJ + S?/SU + aliases + test. */

#define BRIDGE_LINE_MAX           96U
#define BRIDGE_REPLY_MAX          256U
#define BRIDGE_DEFAULT_ADDR       2U
#define BRIDGE_SLOT_MAX           8U

#define BRIDGE_CLAMP_MIN_US       1000U
#define BRIDGE_CLAMP_MAX_US       1500U


/* Smoke: serial line "test" -> wheels 3-6 velocity forward then stop */
#define BRIDGE_TEST_MS            2000U
#define BRIDGE_TEST_VEL_RPM       60U
#define BRIDGE_TEST_DIR           1U
#define BRIDGE_TEST_GAP_MS        20U
/* test streams debug lines via Link_Send; final OK/ERR stays in reply buf */

/* test1m: forward ~1 m via position mode (dir map applies in Zdt_GoDeg).
 * Integer geometry only (newlib nano: no float in vsnprintf).
 * Override at build: -DBRIDGE_WHEEL_DIAMETER_MM=96U etc. or edit below.
 */
#ifndef BRIDGE_WHEEL_DIAMETER_MM
#define BRIDGE_WHEEL_DIAMETER_MM    100U   /* mm */
#endif
#ifndef BRIDGE_WHEEL_GEAR_X100
#define BRIDGE_WHEEL_GEAR_X100      100U   /* motor shaft revs : wheel rev = x/100 */
#endif
#ifndef BRIDGE_TEST1M_DIST_MM
#define BRIDGE_TEST1M_DIST_MM       1000U  /* mm */
#endif

/* Safer motion: reject oversized single MG (per addr, not broadcast). */
#ifndef BRIDGE_MG_MAX_DEG
#define BRIDGE_MG_MAX_DEG           3600L
#endif


#endif /* __BRIDGE_CONFIG_H */
