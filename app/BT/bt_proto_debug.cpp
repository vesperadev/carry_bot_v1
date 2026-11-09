#include "bt_protocol.h"
#include "imu_task.h"
#include "fw_version.h"
#include <cstdio>
#include <cstring>
#include <cctype>

/* 蓝牙调试模式：收到的原样回显，便于 AT 命令与串口助手调试。
 * 需要解析具体 AT 命令时，只改本文件即可，驱动层不受影响。 */
namespace {

class DebugProtocol final : public BtProtocol
{
public:
  bool parse(const BtFrame &frame) override
  {
    reply_len_ = 0U;
    if (frame.len == 0U || frame.len > BT_FRAME_MAX) return false;

    /* Trim CR/LF and accept "gcal" / "GYROCAL" to request gyro bias cal. */
    char cmd[32];
    uint16_t n = frame.len;
    if (n >= sizeof(cmd)) n = static_cast<uint16_t>(sizeof(cmd) - 1U);
    std::memcpy(cmd, frame.data, n);
    cmd[n] = '\0';
    while (n > 0U && (cmd[n - 1U] == '\r' || cmd[n - 1U] == '\n' || cmd[n - 1U] == ' '))
    {
      cmd[--n] = '\0';
    }
    for (uint16_t i = 0U; i < n; ++i)
      cmd[i] = static_cast<char>(std::tolower(static_cast<unsigned char>(cmd[i])));


    if ((n == 3U && std::memcmp(cmd, "ver", 3) == 0) ||
        (n == 7U && std::memcmp(cmd, "version", 7) == 0))
    {
      /* nano.specs: no float printf; integers/strings only */
      const int nwr = std::snprintf(
          reinterpret_cast<char *>(reply_), sizeof(reply_),
          "OK VER %s %s %s\r\n",
          FW_VERSION_BRANCH, FW_VERSION_HASH, FW_VERSION_DESC);
      if (nwr > 0 && static_cast<size_t>(nwr) < sizeof(reply_))
        reply_len_ = static_cast<uint16_t>(nwr);
      else
        reply_len_ = 0U;
      return true;
    }

    if ((n == 4U && std::memcmp(cmd, "gcal", 4) == 0) ||
        (n == 7U && std::memcmp(cmd, "gyrocal", 7) == 0))
    {
      g_bmi088_cal_req = 1U;
      static const char ack[] = "GYROCAL queued\r\n";
      std::memcpy(reply_, ack, sizeof(ack) - 1U);
      reply_len_ = static_cast<uint16_t>(sizeof(ack) - 1U);
      return true;
    }

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
  uint8_t  reply_[BT_FRAME_MAX];
  uint16_t reply_len_ = 0U;
};

DebugProtocol protocol;

}

BtProtocol &BT_GetProtocol()
{
  return protocol;
}
