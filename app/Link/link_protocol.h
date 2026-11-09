#ifndef __LINK_PROTOCOL_H
#define __LINK_PROTOCOL_H

#include <stdint.h>
#include "link_config.h"

/* Queue element: copy from DMA buffer in ISR so next RX cannot overwrite it. */
struct LinkFrame
{
  uint16_t len;
  uint8_t  data[LINK_FRAME_MAX];
};

/* Protocol plug-in: BT (Bridge) or RPI implementation selected by CMake. */
class LinkProtocol
{
public:
  virtual bool parse(const LinkFrame &frame) = 0;
  virtual uint16_t make_reply(uint8_t *out, uint16_t capacity) = 0;
  /* Optional: when true, Link uses a short queue wait and calls on_idle(). */
  virtual bool wants_idle_tick() { return false; }
  virtual void on_idle() {}
};

LinkProtocol &Link_GetProtocol();

#endif /* __LINK_PROTOCOL_H */
