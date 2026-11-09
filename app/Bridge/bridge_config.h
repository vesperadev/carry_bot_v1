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

#endif /* __BRIDGE_CONFIG_H */