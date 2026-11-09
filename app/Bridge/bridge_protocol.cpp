#include "bridge_config.h"
#include "link_public.h"
#include "link_protocol.h"
#include "zdt_public.h"

#include <cstdio>
#include <cstring>
#include <cctype>
#include <cstdint>
#include <cstddef>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

namespace {

class SniffProtocol;
SniffProtocol *g_sniff_self = nullptr;

struct SniffFrame
{
  uint16_t len;
  uint8_t  data[BRIDGE_SNIFF_FRAME_MAX];
};

char to_upper(char c)
{
  return static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
}

bool eq_ci(const char *a, const char *b)
{
  if (a == nullptr || b == nullptr) return false;
  while (*a != '\0' && *b != '\0')
  {
    if (to_upper(*a) != to_upper(*b)) return false;
    ++a;
    ++b;
  }
  return *a == '\0' && *b == '\0';
}

const char *trim_line(char *s)
{
  while (*s == ' ' || *s == '\t') ++s;
  char *end = s + std::strlen(s);
  while (end > s && (end[-1] == ' ' || end[-1] == '\t'))
  {
    --end;
    *end = '\0';
  }
  return s;
}

class SniffProtocol final : public LinkProtocol
{
public:
  SniffProtocol()
  {
    line_len_ = 0U;
    reply_len_ = 0U;
    fwd_on_ = true;
    banner_sent_ = false;
    hooked_ = false;
    q_ = nullptr;
    g_sniff_self = this;
  }

  void ensure_ready()
  {
    if (q_ == nullptr)
      q_ = xQueueCreate(BRIDGE_SNIFF_QUEUE_LEN, sizeof(SniffFrame));
    if (!hooked_)
    {
      Zdt_SetRxHook(&SniffProtocol::rx_hook_thunk);
      hooked_ = true;
    }
  }

  bool wants_idle_tick() override
  {
    return !banner_sent_ || fwd_on_;
  }

  void on_idle() override
  {
    ensure_ready();
    if (!banner_sent_)
    {
      static const char kBanner[] = "SNIFF USART1@115200 -> USART6@9600\r\n";
      if (Link_Send(reinterpret_cast<const uint8_t *>(kBanner),
                    static_cast<uint16_t>(sizeof(kBanner) - 1U)) == HAL_OK)
      {
        banner_sent_ = true;
      }
      return;
    }

    if (!fwd_on_ || q_ == nullptr) return;

    SniffFrame frame{};
    while (xQueueReceive(q_, &frame, 0) == pdTRUE)
    {
      if (frame.len == 0U) continue;
      for (int i = 0; i < 20; ++i)
      {
        const HAL_StatusTypeDef st = Link_Send(frame.data, frame.len);
        if (st == HAL_OK) break;
        if (st != HAL_BUSY) break;
        vTaskDelay(1);
      }
    }
  }

  bool parse(const LinkFrame &frame) override
  {
    ensure_ready();
    reply_len_ = 0U;
    if (frame.len == 0U) return false;

    bool any = false;
    for (uint16_t i = 0U; i < frame.len; ++i)
    {
      const char c = static_cast<char>(frame.data[i]);
      if (c == '\r' || c == '\n')
      {
        if (line_len_ > 0U)
        {
          line_buf_[line_len_] = '\0';
          dispatch_line(line_buf_);
          line_len_ = 0U;
          any = true;
        }
        continue;
      }
      if (line_len_ + 1U < BRIDGE_LINE_MAX)
        line_buf_[line_len_++] = c;
    }
    return any && reply_len_ > 0U;
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
  static void rx_hook_thunk(const uint8_t *data, uint16_t len)
  {
    if (g_sniff_self != nullptr) g_sniff_self->on_u1_rx(data, len);
  }

  void on_u1_rx(const uint8_t *data, uint16_t len)
  {
    if (!fwd_on_ || q_ == nullptr || data == nullptr || len == 0U) return;
    if (len > BRIDGE_SNIFF_FRAME_MAX) len = BRIDGE_SNIFF_FRAME_MAX;
    SniffFrame frame{};
    frame.len = len;
    std::memcpy(frame.data, data, len);
    (void)xQueueSendToBack(q_, &frame, 0);
  }

  void append_line(const char *s)
  {
    if (s == nullptr) return;
    const size_t n = std::strlen(s);
    if (reply_len_ + n + 2U > BRIDGE_REPLY_MAX) return;
    std::memcpy(reply_ + reply_len_, s, n);
    reply_len_ = static_cast<uint16_t>(reply_len_ + n);
    reply_[reply_len_++] = '\r';
    reply_[reply_len_++] = '\n';
  }

  void dispatch_line(char *raw)
  {
    const char *p = trim_line(raw);
    if (*p == '\0') return;

    if (eq_ci(p, "fwd?") || eq_ci(p, "sniff?"))
    {
      append_line(fwd_on_
                      ? "OK FWD on USART1@115200 -> USART6@9600"
                      : "OK FWD off");
      return;
    }
    if (eq_ci(p, "fwd") || eq_ci(p, "fwd1") || eq_ci(p, "sniff"))
    {
      fwd_on_ = true;
      append_line("OK FWD on");
      return;
    }
    if (eq_ci(p, "fwd0") || eq_ci(p, "sniff0"))
    {
      fwd_on_ = false;
      append_line("OK FWD off");
      return;
    }

    append_line("ERR sniff cmds: fwd? fwd fwd0");
  }

  char line_buf_[BRIDGE_LINE_MAX];
  uint16_t line_len_;
  uint8_t reply_[BRIDGE_REPLY_MAX];
  uint16_t reply_len_;
  bool fwd_on_;
  bool banner_sent_;
  bool hooked_;
  QueueHandle_t q_;
};

SniffProtocol protocol;

}  // namespace

LinkProtocol &Link_GetProtocol()
{
  return protocol;
}