#include "link_protocol.h"

/* RPI 对端（树莓派）协议骨架，协议定稿后只改本文件：
 *   请求 | 0xA5 | cmd(1) | payload_len(1) | payload | crc8 |
 *   回复 | 0x5A | cmd(1) | 0x01           | result  | crc8 |
 * 目前仅实现 PING(0x00)，其余命令返回 result=0xFF，
 * 后续按 cmd 分发业务（速度/模式/状态查询等）在此扩展。 */
namespace {

constexpr uint8_t FRAME_HEAD = 0xA5U;
constexpr uint8_t REPLY_HEAD = 0x5AU;
constexpr uint8_t CMD_PING   = 0x00U;
constexpr uint8_t FRAME_MIN_LEN = 4U;   /* head + cmd + len + crc8 */

uint8_t crc8(const uint8_t *data, uint16_t len)
{
  uint8_t crc = 0x00U;
  for (uint16_t i = 0U; i < len; ++i)
  {
    crc ^= data[i];
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
      crc = (crc & 0x80U) ? static_cast<uint8_t>((crc << 1U) ^ 0x07U)
                          : static_cast<uint8_t>(crc << 1U);
    }
  }
  return crc;
}

class RpiProtocol final : public LinkProtocol
{
public:
  bool parse(const LinkFrame &frame) override
  {
    pending_ = false;
    if (frame.len < FRAME_MIN_LEN || frame.data[0] != FRAME_HEAD) return false;

    const uint8_t payload_len = frame.data[2];
    if (frame.len != static_cast<uint16_t>(FRAME_MIN_LEN + payload_len)) return false;
    if (crc8(frame.data, static_cast<uint16_t>(frame.len - 1U)) != frame.data[frame.len - 1U]) return false;

    cmd_ = frame.data[1];
    result_ = (cmd_ == CMD_PING) ? 0x00U : 0xFFU;
    pending_ = true;
    return true;
  }

  uint16_t make_reply(uint8_t *out, uint16_t capacity) override
  {
    if (!pending_ || capacity < 5U)
    {
      pending_ = false;
      return 0U;
    }
    out[0] = REPLY_HEAD;
    out[1] = cmd_;
    out[2] = 0x01U;
    out[3] = result_;
    out[4] = crc8(out, 4U);
    pending_ = false;
    return 5U;
  }

private:
  uint8_t cmd_ = 0U;
  uint8_t result_ = 0xFFU;
  bool    pending_ = false;
};

RpiProtocol protocol;

}

LinkProtocol &Link_GetProtocol()
{
  return protocol;
}
