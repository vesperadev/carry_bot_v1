#ifndef __BRIDGE_CONFIG_H
#define __BRIDGE_CONFIG_H

/* BT text bridge over Link (USART6). A-tier + test + telem. */

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

/* Continuous wheel telemetry (USART1 read -> USART6 report) */
#define BRIDGE_TELEM_PERIOD_MS    1000U
#define BRIDGE_TELEM_READ_MS      80U
#define BRIDGE_TELEM_GAP_MS       20U
/* Default off; "telem" enables, "telem0" disables */

#endif /* __BRIDGE_CONFIG_H */
