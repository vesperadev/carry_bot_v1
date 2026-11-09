#ifndef __BRIDGE_CONFIG_H
#define __BRIDGE_CONFIG_H

/* USART1@115200 RX sniff -> USART6@9600 Link TX (pure passthrough).
 * This branch replaces Bridge motor/servo text cmds; ZDT TX is unused.
 */

#define BRIDGE_LINE_MAX           64U
#define BRIDGE_REPLY_MAX          192U
#define BRIDGE_SNIFF_QUEUE_LEN    16U
#define BRIDGE_SNIFF_FRAME_MAX    64U   /* match ZDT_FRAME_MAX */

#endif /* __BRIDGE_CONFIG_H */