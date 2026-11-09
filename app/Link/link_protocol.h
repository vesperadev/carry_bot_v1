#ifndef __LINK_PROTOCOL_H
#define __LINK_PROTOCOL_H

#include <stdint.h>
#include "link_config.h"

/* 队列元素：中断里从 DMA 缓冲拷贝出来，避免被下一次接收覆写 */
struct LinkFrame
{
  uint16_t len;
  uint8_t  data[LINK_FRAME_MAX];
};

/* 协议层接口：对端为蓝牙调试(BT) 或 树莓派(RPI) 各提供一个实现，由 CMake 选编 */
class LinkProtocol
{
public:
  virtual bool parse(const LinkFrame &frame) = 0;
  virtual uint16_t make_reply(uint8_t *out, uint16_t capacity) = 0;
};

/* 由 link_proto_debug.cpp / link_proto_rpi.cpp 之一实现 */
LinkProtocol &Link_GetProtocol();

#endif /* __LINK_PROTOCOL_H */
