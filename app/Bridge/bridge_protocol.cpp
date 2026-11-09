#include "bridge_config.h"
#include "link_protocol.h"
#include "zdt_public.h"
#include "zdt_config.h"
#include "servo_public.h"
#include "servo_config.h"

#include <cstdio>
#include <cstring>
#include <cctype>
#include <cstdarg>

namespace {

struct BridgeSlot
{
  uint8_t  used;
  uint8_t  addr;
  uint8_t  dir;
  uint8_t  acc;
  uint16_t vel;
  int32_t  last_deg;
};

char to_upper(char c)
{
  return static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
}

bool is_digit(char c)
{
  return c >= '0' && c <= '9';
}

const char *skip_ws(const char *p)
{
  while (*p == ' ' || *p == '\t') ++p;
  return p;
}

bool parse_u32(const char *&p, uint32_t *out)
{
  if (!is_digit(*p)) return false;
  uint32_t v = 0U;
  while (is_digit(*p))
  {
    v = v * 10U + static_cast<uint32_t>(*p - '0');
    ++p;
  }
  *out = v;
  return true;
}

bool parse_i32(const char *&p, int32_t *out)
{
  int sign = 1;
  if (*p == '+') { ++p; }
  else if (*p == '-') { sign = -1; ++p; }
  uint32_t v = 0U;
  if (!parse_u32(p, &v)) return false;
  *out = sign < 0 ? -static_cast<int32_t>(v) : static_cast<int32_t>(v);
  return true;
}

const char *hal_err(HAL_StatusTypeDef st)
{
  switch (st)
  {
    case HAL_OK:      return "OK";
    case HAL_BUSY:    return "ERR busy";
    case HAL_TIMEOUT: return "ERR timeout";
    default:          return "ERR hal";
  }
}

class BridgeProtocol final : public LinkProtocol
{
public:
  BridgeProtocol()
  {
    reset_slots();
    default_addr_ = BRIDGE_DEFAULT_ADDR;
    line_len_ = 0U;
    reply_len_ = 0U;
  }

  bool parse(const LinkFrame &frame) override
  {
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
      else
        line_len_ = 0U; /* drop overrun line */
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
  void reset_slots()
  {
    std::memset(slots_, 0, sizeof(slots_));
    static const uint8_t kAddrs[] = {
      ZDT_ADDR_LIFT, ZDT_ADDR_TURN,
      ZDT_ADDR_WHEEL_FL, ZDT_ADDR_WHEEL_RL,
      ZDT_ADDR_WHEEL_FR, ZDT_ADDR_WHEEL_RR
    };
    for (uint8_t i = 0U; i < sizeof(kAddrs); ++i)
    {
      slots_[i].used = 1U;
      slots_[i].addr = kAddrs[i];
      slots_[i].dir = 0U;
      slots_[i].acc = static_cast<uint8_t>(ZDT_DEFAULT_ACC);
      slots_[i].vel = static_cast<uint16_t>(ZDT_DEFAULT_VEL_RPM);
      slots_[i].last_deg = 0;
    }
  }

  BridgeSlot *slot_find(uint8_t addr)
  {
    if (addr == ZDT_ADDR_BROADCAST) return nullptr;
    for (uint8_t i = 0U; i < BRIDGE_SLOT_MAX; ++i)
    {
      if (slots_[i].used && slots_[i].addr == addr) return &slots_[i];
    }
    for (uint8_t i = 0U; i < BRIDGE_SLOT_MAX; ++i)
    {
      if (!slots_[i].used)
      {
        slots_[i].used = 1U;
        slots_[i].addr = addr;
        slots_[i].dir = 0U;
        slots_[i].acc = static_cast<uint8_t>(ZDT_DEFAULT_ACC);
        slots_[i].vel = static_cast<uint16_t>(ZDT_DEFAULT_VEL_RPM);
        slots_[i].last_deg = 0;
        return &slots_[i];
      }
    }
    return &slots_[0];
  }

  void append_raw(const char *s)
  {
    if (s == nullptr) return;
    while (*s && reply_len_ + 1U < BRIDGE_REPLY_MAX)
      reply_[reply_len_++] = static_cast<uint8_t>(*s++);
  }

  void append_line(const char *s)
  {
    if (reply_len_ > 0U && reply_len_ + 2U < BRIDGE_REPLY_MAX)
    {
      reply_[reply_len_++] = '\r';
      reply_[reply_len_++] = '\n';
    }
    append_raw(s);
  }

  void reply_fmt(const char *fmt, ...)
  {
    char tmp[160];
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    if (n > 0) append_line(tmp);
  }

  void dispatch_line(char *line)
  {
    const char *p = skip_ws(line);
    if (*p == '\0') return;

    const char head = to_upper(*p);
    if (head == 'M')
    {
      handle_motor(p + 1);
      return;
    }
    if (head == 'S')
    {
      handle_servo(p + 1);
      return;
    }
    append_line("ERR unknown, send M? or SH");
  }

  void handle_motor(const char *p)
  {
    p = skip_ws(p);
    if (*p == '\0')
    {
      append_line("ERR missing cmd, send M?");
      return;
    }

    if (*p == '?' || to_upper(*p) == 'H')
    {
      append_line("OK A-tier: M? ME0/1 MG[+|-]deg MX MS[n] ML MJ S? S0U/S1U S0C/S0O/S1M");
      append_line("addr 1=lift 2=turn 3-6=wheels *=bcast default=2");
      return;
    }

    /* Two-letter B-tier (mecanum / etc.): MFW / MSW / MWS / ... */
    if (std::isalpha(static_cast<unsigned char>(p[0])) &&
        std::isalpha(static_cast<unsigned char>(p[1])))
    {
      append_line("ERR not implemented (B-tier), send M?");
      return;
    }

    bool have_addr = false;
    uint8_t addr = default_addr_;

    if (*p == '*' || *p == '0')
    {
      /* lone 0 or * => broadcast; M0E1 / M*E1 */
      if (*p == '*' || (*(p + 1) != '\0' && !is_digit(*(p + 1))))
      {
        addr = ZDT_ADDR_BROADCAST;
        have_addr = true;
        ++p;
      }
    }
    if (!have_addr && is_digit(*p))
    {
      uint32_t v = 0U;
      if (!parse_u32(p, &v) || v > 255U)
      {
        append_line("ERR bad addr");
        return;
      }
      addr = static_cast<uint8_t>(v);
      have_addr = true;
    }

    p = skip_ws(p);
    if (*p == '\0')
    {
      append_line("ERR missing subcmd, send M?");
      return;
    }

    const char cmd = to_upper(*p++);
    switch (cmd)
    {
      case 'E':
      {
        uint32_t on = 0U;
        if (!parse_u32(p, &on) || (on != 0U && on != 1U))
        {
          append_line("ERR ME needs 0 or 1");
          return;
        }
        const HAL_StatusTypeDef st = Zdt_Enable(addr, static_cast<uint8_t>(on));
        reply_fmt("%s ME addr=%u on=%u", hal_err(st), static_cast<unsigned>(addr),
                  static_cast<unsigned>(on));
        break;
      }
      case 'G':
      {
        int32_t deg = 0;
        if (!parse_i32(p, &deg))
        {
          append_line("ERR MG needs deg");
          return;
        }
        const HAL_StatusTypeDef st = Zdt_GoDeg(addr, deg);
        BridgeSlot *s = slot_find(addr);
        if (s != nullptr)
        {
          s->last_deg = deg;
          if (deg > 0) s->dir = 1U;
          else if (deg < 0) s->dir = 0U;
        }
        reply_fmt("%s MG addr=%u deg=%ld", hal_err(st), static_cast<unsigned>(addr),
                  static_cast<long>(deg));
        break;
      }
      case 'X':
      {
        const HAL_StatusTypeDef st = Zdt_Stop(addr);
        reply_fmt("%s MX addr=%u", hal_err(st), static_cast<unsigned>(addr));
        break;
      }
      case 'S':
      {
        uint8_t new_addr = addr;
        if (!have_addr)
        {
          uint32_t v = 0U;
          if (!parse_u32(p, &v) || v == 0U || v > 255U)
          {
            append_line("ERR default addr must be 1-255");
            return;
          }
          new_addr = static_cast<uint8_t>(v);
        }
        else if (addr == ZDT_ADDR_BROADCAST)
        {
          append_line("ERR default addr must be 1-255");
          return;
        }
        default_addr_ = new_addr;
        (void)slot_find(new_addr);
        reply_fmt("OK MS default=%u", static_cast<unsigned>(default_addr_));
        break;
      }
      case 'L':
      {
        append_line("OK ML");
        for (uint8_t i = 0U; i < BRIDGE_SLOT_MAX; ++i)
        {
          if (!slots_[i].used) continue;
          reply_fmt("SLOT %u addr=%u dir=%u vel=%u acc=%u last_deg=%ld",
                    static_cast<unsigned>(i),
                    static_cast<unsigned>(slots_[i].addr),
                    static_cast<unsigned>(slots_[i].dir),
                    static_cast<unsigned>(slots_[i].vel),
                    static_cast<unsigned>(slots_[i].acc),
                    static_cast<long>(slots_[i].last_deg));
        }
        reply_fmt("default=%u", static_cast<unsigned>(default_addr_));
        break;
      }
      case 'J':
      {
        if (addr == ZDT_ADDR_BROADCAST)
        {
          append_line("ERR MJ no broadcast");
          return;
        }
        BridgeSlot *s = slot_find(addr);
        if (s == nullptr)
        {
          append_line("ERR no slot");
          return;
        }
        reply_fmt("OK MJ addr=%u dir=%u vel=%u acc=%u last_deg=%ld",
                  static_cast<unsigned>(s->addr),
                  static_cast<unsigned>(s->dir),
                  static_cast<unsigned>(s->vel),
                  static_cast<unsigned>(s->acc),
                  static_cast<long>(s->last_deg));
        break;
      }
      case '?':
        append_line("OK A-tier: M? ME0/1 MG[+|-]deg MX MS[n] ML MJ");
        break;
      default:
        append_line("ERR unknown M cmd, send M?");
        break;
    }
  }

  void handle_servo(const char *p)
  {
    p = skip_ws(p);
    if (*p == '\0')
    {
      append_line("ERR missing cmd, send SH");
      return;
    }

    if (*p == '?' || to_upper(*p) == 'H')
    {
      append_line("OK S?: S0U1000 close S0U1500 open S1U<us> mid=1500; S0C/S0O/S1M");
      return;
    }

    /* Optional channel digit(s) */
    bool have_ch = false;
    uint8_t ch = 0U;
    if (is_digit(*p))
    {
      uint32_t v = 0U;
      if (!parse_u32(p, &v) || v > SERVO_CH_MAX)
      {
        append_line("ERR ch 0-15");
        return;
      }
      ch = static_cast<uint8_t>(v);
      have_ch = true;
    }

    p = skip_ws(p);
    if (*p == '\0')
    {
      append_line("ERR missing cmd, send SH");
      return;
    }

    const char cmd = to_upper(*p++);
    switch (cmd)
    {
      case 'U':
      {
        uint32_t us = 0U;
        if (!parse_u32(p, &us))
        {
          append_line("ERR SU needs us");
          return;
        }
        if (!have_ch) ch = 0U;
        if (!Servo_IsReady())
        {
          append_line("ERR servo not ready");
          return;
        }
        HAL_StatusTypeDef st;
        if (ch == SERVO_CH_CLAMP)
        {
          if (us < BRIDGE_CLAMP_MIN_US) us = BRIDGE_CLAMP_MIN_US;
          if (us > BRIDGE_CLAMP_MAX_US) us = BRIDGE_CLAMP_MAX_US;
          st = Servo_SetClampUs(static_cast<uint16_t>(us));
        }
        else if (ch == SERVO_CH_TURNTABLE)
        {
          st = Servo_SetTurntableUs(static_cast<uint16_t>(us));
        }
        else
        {
          st = Servo_SetPulseUs(ch, static_cast<uint16_t>(us));
        }
        reply_fmt("%s S%uU%u", hal_err(st), static_cast<unsigned>(ch),
                  static_cast<unsigned>(us));
        break;
      }
      case 'C':
      {
        if (have_ch && ch != SERVO_CH_CLAMP)
        {
          append_line("ERR S0C only");
          return;
        }
        if (!Servo_IsReady()) { append_line("ERR servo not ready"); return; }
        const HAL_StatusTypeDef st = Servo_ClampClose();
        reply_fmt("%s S0C", hal_err(st));
        break;
      }
      case 'O':
      {
        if (have_ch && ch != SERVO_CH_CLAMP)
        {
          append_line("ERR S0O only");
          return;
        }
        if (!Servo_IsReady()) { append_line("ERR servo not ready"); return; }
        const HAL_StatusTypeDef st = Servo_ClampOpen();
        reply_fmt("%s S0O", hal_err(st));
        break;
      }
      case 'M':
      {
        if (have_ch && ch != SERVO_CH_TURNTABLE)
        {
          append_line("ERR S1M only");
          return;
        }
        if (!Servo_IsReady()) { append_line("ERR servo not ready"); return; }
        const HAL_StatusTypeDef st = Servo_TurntableMid();
        reply_fmt("%s S1M", hal_err(st));
        break;
      }
      default:
        append_line("ERR unknown S cmd, send SH");
        break;
    }
  }

  BridgeSlot slots_[BRIDGE_SLOT_MAX];
  uint8_t default_addr_;
  char line_buf_[BRIDGE_LINE_MAX];
  uint16_t line_len_;
  uint8_t reply_[BRIDGE_REPLY_MAX];
  uint16_t reply_len_;
};

BridgeProtocol protocol;

}  // namespace

LinkProtocol &Link_GetProtocol()
{
  return protocol;
}