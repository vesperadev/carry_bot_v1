#ifndef __BRIDGE_CONFIG_H
#define __BRIDGE_CONFIG_H

/* BT text bridge over Link (USART6). A-tier: M?/ME/MG/MX/MS/ML/MJ + S?/SU + aliases. */

#define BRIDGE_LINE_MAX           96U
#define BRIDGE_REPLY_MAX          256U
#define BRIDGE_DEFAULT_ADDR       2U
#define BRIDGE_SLOT_MAX           8U

#define BRIDGE_CLAMP_MIN_US       1000U
#define BRIDGE_CLAMP_MAX_US       1500U

#endif /* __BRIDGE_CONFIG_H */