#include "link_protocol.h"
#include <cstring>

/* BT 对端（蓝牙调试）：收到的原样回显，便于 AT 命令与串口助手调试。
 * 需要解析具体 AT 命令时，只改本文件即可，驱动层不受影响。 */
namespace {

class DebugProtocol final : public LinkProtocol
{
public:
  bool parse(const LinkFrame &frame) override
  {
    reply_len_ = 0U;
    if (frame.len == 0U || frame.len > LINK_FRAME_MAX) return false;
    std::memcpy(reply_, frame.data, frame.len);
    reply_len_ = frame.len;
    return true;
  }

  uint16_t make_reply(uint8_t *out, uint16_t capacity) override
  {
    if (reply_len_ == 0U || reply_len_ > capacity) return 0U;
    std::memcpy(out, reply_, reply_len_);
    const uint16_t n = reply_len_;
    reply_len_ = 0U;
    return n;
  }

private:
  uint8_t  reply_[LINK_FRAME_MAX];
  uint16_t reply_len_ = 0U;
};

DebugProtocol protocol;

}

LinkProtocol &Link_GetProtocol()
{
  return protocol;
}
