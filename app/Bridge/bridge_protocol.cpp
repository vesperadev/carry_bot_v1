#include "bridge_config.h"
#include "fw_version.h"
#include "link_public.h"
#include "link_protocol.h"
#include "zdt_public.h"
#include "zdt_config.h"
#include "zdt_limits.h"
#include "servo_public.h"
#include "servo_config.h"

#include <cstdio>
#include <cstring>
#include <cctype>
#include <cstdarg>

#include "FreeRTOS.h"
#include "task.h"

namespace {

struct BridgeSlot
{
  uint8_t  used;
  uint8_t  addr;
  uint8_t  dir;
  uint8_t  acc;
  uint16_t vel;
  int32_t  last_deg;     /* last MG relative command (deg) */
  int32_t  drv_deg;      /* cached Emm 0x36 angle (logical); not encoder if stale */
  uint8_t  drv_valid;    /* 1 if drv_deg from successful USART1 read */
  int32_t  fallback_deg; /* soft estimate when 0x36 read fails; not truth */
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
      slots_[i].drv_deg = 0;
      slots_[i].drv_valid = 0U;
      slots_[i].fallback_deg = 0;
    }
  }

  void slot_refresh_driver_pos(BridgeSlot *s)
  {
    if (s == nullptr) return;
    int32_t deg = 0;
    const HAL_StatusTypeDef st =
        Zdt_ReadRealtimePosDeg(s->addr, &deg, ZDT_ACK_TIMEOUT_MS);
    if (st == HAL_OK)
    {
      s->drv_deg = deg;
      s->drv_valid = 1U;
    }
  }

  int32_t slot_current_for_ma(BridgeSlot *s, const char **src_out)
  {
    if (s == nullptr)
    {
      if (src_out != nullptr) *src_out = "none";
      return 0;
    }
    int32_t deg = 0;
    if (Zdt_ReadRealtimePosDeg(s->addr, &deg, ZDT_ACK_TIMEOUT_MS) == HAL_OK)
    {
      s->drv_deg = deg;
      s->drv_valid = 1U;
      if (src_out != nullptr) *src_out = "drv";
      return deg;
    }
    s->drv_valid = 0U;
    if (src_out != nullptr) *src_out = "fallback";
    return s->fallback_deg;
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
        slots_[i].drv_deg = 0;
        slots_[i].drv_valid = 0U;
        slots_[i].fallback_deg = 0;
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

    if (eq_ci(p, "ver") || eq_ci(p, "version"))
    {
      handle_ver();
      return;
    }
    if (eq_ci(p, "test1m"))
    {
      handle_test_1m();
      return;
    }
    if (eq_ci(p, "test"))
    {
      handle_test_fwd();
      return;
    }

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
    append_line("ERR unknown, send M? or SH or test/test1m/ver");
  }

  void flush_reply()
  {
    if (reply_len_ == 0U) return;
    (void)Link_Send(reply_, reply_len_);
    /* USART6 @9600: ~1ms/byte; short lines + margin before next TX */
    vTaskDelay(pdMS_TO_TICKS(40U + reply_len_ / 2U));
    reply_len_ = 0U;
  }

  const char *ack_tag(HAL_StatusTypeDef st)
  {
    switch (st)
    {
      case HAL_OK:      return "ACK";
      case HAL_TIMEOUT: return "TIMEOUT";
      case HAL_BUSY:    return "BUSY";
      default:          return "FAIL";
    }
  }



  void handle_ver()
  {
    /* 串口1: ver / version — branch + short hash + subject (from fw_version.h). */
    reply_fmt("OK VER %s %s %s",
              FW_VERSION_BRANCH, FW_VERSION_HASH, FW_VERSION_DESC);
  }

  void handle_test_1m()
  {
    /* Forward ~1 m: enable -> GoDeg(+deg) on FL/RL/FR/RR -> wait travel -> stop/disable.
     * Wheel dir polarity from zdt_dir_map.h so +deg is vehicle-forward. */
    static const uint8_t kWheels[] = {
      ZDT_ADDR_WHEEL_FL, ZDT_ADDR_WHEEL_RL,
      ZDT_ADDR_WHEEL_FR, ZDT_ADDR_WHEEL_RR
    };
    const uint32_t tmo = ZDT_ACK_TIMEOUT_MS;
    const TickType_t gap = pdMS_TO_TICKS(BRIDGE_TEST_GAP_MS);
    const float diam = BRIDGE_WHEEL_DIAMETER_MM;
    const float gear = BRIDGE_WHEEL_GEAR_RATIO;
    const float dist = BRIDGE_TEST1M_DIST_MM;
    const float circ = 3.14159265f * diam;
    int32_t deg = 0;
    uint32_t travel_ms = 3000U;
    char rxhex[48];
    uint8_t fail_en = 0U;
    uint8_t fail_go = 0U;
    uint8_t fail_stop = 0U;
    uint8_t fail_dis = 0U;

    if (circ > 1.0f)
    {
      deg = static_cast<int32_t>((360.0f * dist * gear) / circ + 0.5f);
    }
    if (deg < 1) deg = 1;

    {
      const float rpm = static_cast<float>(ZDT_DEFAULT_VEL_RPM);
      const float revs = static_cast<float>(deg) / 360.0f;
      float sec = (rpm > 1.0f) ? (revs * 60.0f / rpm) : 3.0f;
      sec = sec * 2.5f + 1.0f;
      if (sec < 2.0f) sec = 2.0f;
      if (sec > 30.0f) sec = 30.0f;
      travel_ms = static_cast<uint32_t>(sec * 1000.0f + 0.5f);
    }

    /* nano.specs: no float printf — print assumed geometry as integers */
    reply_fmt("TEST1M dist=%ldmm diam=%ldmm gear=%ld/100 deg=%ld wait=%lums (ASSUME; edit bridge_config)",
              static_cast<long>(dist + 0.5f), static_cast<long>(diam + 0.5f),
              static_cast<long>(gear * 100.0f + 0.5f), static_cast<long>(deg),
              static_cast<unsigned long>(travel_ms));
    flush_reply();

    for (uint8_t i = 0U; i < 4U; ++i)
    {
      const uint8_t addr = kWheels[i];
      const HAL_StatusTypeDef st = Zdt_EnableAndWait(addr, 1U, tmo);
      (void)Zdt_FormatLastRx(rxhex, sizeof(rxhex));
      reply_fmt("E a=%u %s rx=%s", static_cast<unsigned>(addr), ack_tag(st), rxhex);
      flush_reply();
      if (st != HAL_OK) fail_en = static_cast<uint8_t>(fail_en | (1U << i));
      vTaskDelay(gap);
    }

    for (uint8_t i = 0U; i < 4U; ++i)
    {
      const uint8_t addr = kWheels[i];
      const HAL_StatusTypeDef st = Zdt_GoDegAndWait(addr, deg, tmo);
      (void)Zdt_FormatLastRx(rxhex, sizeof(rxhex));
      reply_fmt("G a=%u deg=%ld %s rx=%s", static_cast<unsigned>(addr),
                static_cast<long>(deg), ack_tag(st), rxhex);
      flush_reply();
      if (st != HAL_OK) fail_go = static_cast<uint8_t>(fail_go | (1U << i));
      vTaskDelay(gap);
    }

    if (fail_en != 0U || fail_go != 0U)
    {
      for (uint8_t i = 0U; i < 4U; ++i)
      {
        (void)Zdt_Stop(kWheels[i]);
        vTaskDelay(gap);
      }
      reply_fmt("ERR TEST1M en=0x%02X go=0x%02X",
                static_cast<unsigned>(fail_en), static_cast<unsigned>(fail_go));
      return;
    }

    reply_fmt("TEST1M moving wait=%lums", static_cast<unsigned long>(travel_ms));
    flush_reply();
    vTaskDelay(pdMS_TO_TICKS(travel_ms));

    for (uint8_t i = 0U; i < 4U; ++i)
    {
      const uint8_t addr = kWheels[i];
      const HAL_StatusTypeDef st = Zdt_StopAndWait(addr, tmo);
      (void)Zdt_FormatLastRx(rxhex, sizeof(rxhex));
      reply_fmt("X a=%u %s rx=%s", static_cast<unsigned>(addr), ack_tag(st), rxhex);
      flush_reply();
      if (st != HAL_OK) fail_stop = static_cast<uint8_t>(fail_stop | (1U << i));
      vTaskDelay(gap);
    }

    for (uint8_t i = 0U; i < 4U; ++i)
    {
      const uint8_t addr = kWheels[i];
      const HAL_StatusTypeDef st = Zdt_EnableAndWait(addr, 0U, tmo);
      (void)Zdt_FormatLastRx(rxhex, sizeof(rxhex));
      reply_fmt("D a=%u %s rx=%s", static_cast<unsigned>(addr), ack_tag(st), rxhex);
      flush_reply();
      if (st != HAL_OK) fail_dis = static_cast<uint8_t>(fail_dis | (1U << i));
      vTaskDelay(gap);
    }

    if (fail_stop != 0U || fail_dis != 0U)
    {
      reply_fmt("ERR TEST1M ran stop=0x%02X dis=0x%02X",
                static_cast<unsigned>(fail_stop), static_cast<unsigned>(fail_dis));
      return;
    }
    append_line("OK TEST1M fwd~1m ack=ok");
  }

  void handle_test_fwd()
  {
    /* Four mecanum wheels, same dir/vel for ~2s then stop.
     * Streams per-step USART1 TX/ACK/RX debug on USART6; final OK/ERR in reply. */
    static const uint8_t kWheels[] = {
      ZDT_ADDR_WHEEL_FL, ZDT_ADDR_WHEEL_RL,
      ZDT_ADDR_WHEEL_FR, ZDT_ADDR_WHEEL_RR
    };
    const uint8_t dir = static_cast<uint8_t>(BRIDGE_TEST_DIR);
    const uint16_t vel = static_cast<uint16_t>(BRIDGE_TEST_VEL_RPM);
    const uint32_t tmo = ZDT_ACK_TIMEOUT_MS;
    const TickType_t gap = pdMS_TO_TICKS(BRIDGE_TEST_GAP_MS);
    char rxhex[48];
    uint8_t fail_en = 0U;
    uint8_t fail_vel = 0U;
    uint8_t fail_stop = 0U;
    uint8_t fail_dis = 0U;

    reply_fmt("TEST start addr=3-6 dir=%u vel=%u tmo=%lu",
              static_cast<unsigned>(dir), static_cast<unsigned>(vel),
              static_cast<unsigned long>(tmo));
    flush_reply();

    for (uint8_t i = 0U; i < 4U; ++i)
    {
      const uint8_t addr = kWheels[i];
      const HAL_StatusTypeDef st = Zdt_EnableAndWait(addr, 1U, tmo);
      (void)Zdt_FormatLastRx(rxhex, sizeof(rxhex));
      reply_fmt("E a=%u %s rx=%s", static_cast<unsigned>(addr), ack_tag(st), rxhex);
      flush_reply();
      if (st != HAL_OK) fail_en = static_cast<uint8_t>(fail_en | (1U << i));
      vTaskDelay(gap);
    }

    for (uint8_t i = 0U; i < 4U; ++i)
    {
      const uint8_t addr = kWheels[i];
      const HAL_StatusTypeDef st = Zdt_VelAndWait(addr, dir, vel, tmo);
      (void)Zdt_FormatLastRx(rxhex, sizeof(rxhex));
      reply_fmt("V a=%u %s rx=%s", static_cast<unsigned>(addr), ack_tag(st), rxhex);
      flush_reply();
      if (st != HAL_OK) fail_vel = static_cast<uint8_t>(fail_vel | (1U << i));
      vTaskDelay(gap);
    }

    if (fail_en != 0U || fail_vel != 0U)
    {
      /* Still attempt stop so a half-started wheel does not free-run. */
      for (uint8_t i = 0U; i < 4U; ++i)
      {
        (void)Zdt_Stop(kWheels[i]);
        vTaskDelay(gap);
      }
      reply_fmt("ERR TEST en=0x%02X vel=0x%02X (bit0=FL..bit3=RR; TIMEOUT=鏃燯SART1 ACK)",
                static_cast<unsigned>(fail_en), static_cast<unsigned>(fail_vel));
      return;
    }

    vTaskDelay(pdMS_TO_TICKS(BRIDGE_TEST_MS));

    for (uint8_t i = 0U; i < 4U; ++i)
    {
      const uint8_t addr = kWheels[i];
      const HAL_StatusTypeDef st = Zdt_StopAndWait(addr, tmo);
      (void)Zdt_FormatLastRx(rxhex, sizeof(rxhex));
      reply_fmt("X a=%u %s rx=%s", static_cast<unsigned>(addr), ack_tag(st), rxhex);
      flush_reply();
      if (st != HAL_OK) fail_stop = static_cast<uint8_t>(fail_stop | (1U << i));
      vTaskDelay(gap);
    }

    for (uint8_t i = 0U; i < 4U; ++i)
    {
      const uint8_t addr = kWheels[i];
      const HAL_StatusTypeDef st = Zdt_EnableAndWait(addr, 0U, tmo);
      (void)Zdt_FormatLastRx(rxhex, sizeof(rxhex));
      reply_fmt("D a=%u %s rx=%s", static_cast<unsigned>(addr), ack_tag(st), rxhex);
      flush_reply();
      if (st != HAL_OK) fail_dis = static_cast<uint8_t>(fail_dis | (1U << i));
      vTaskDelay(gap);
    }

    if (fail_stop != 0U || fail_dis != 0U)
    {
      reply_fmt("ERR TEST ran but stop=0x%02X dis=0x%02X",
                static_cast<unsigned>(fail_stop), static_cast<unsigned>(fail_dis));
      return;
    }

    append_line("OK TEST fwd 2s ack=ok");
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
      append_line("OK A-tier: M? ME MG MA MX MZ MR MS ML MJ ... test test1m ver");
      append_line("MR=USART1 smoke: query 36/1F/3A dump hex on shell (lift M1R turn M2R)");
      append_line("MA=0x36 goto+0x3A wait; tol=ZDT_MA_TOL_DEG; soft lim OFF (zdt_limits.h)");
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
        const int32_t deg_req = deg;
        deg = Zdt_ClampRelativeDeg(addr, deg);
        const HAL_StatusTypeDef st = Zdt_GoDeg(addr, deg);
        BridgeSlot *s = slot_find(addr);
        if (s != nullptr)
        {
          s->last_deg = deg;
          if (st == HAL_OK)
          {
            if (deg > 0) s->dir = 1U;
            else if (deg < 0) s->dir = 0U;
            slot_refresh_driver_pos(s);
            if (s->drv_valid == 0U) s->fallback_deg += deg;
          }
        }
        if (deg != deg_req)
        {
          reply_fmt("%s MG addr=%u deg=%ld lim=%ld", hal_err(st),
                    static_cast<unsigned>(addr), static_cast<long>(deg),
                    static_cast<long>(deg_req));
        }
        else
        {
          reply_fmt("%s MG addr=%u deg=%ld", hal_err(st), static_cast<unsigned>(addr),
                    static_cast<long>(deg));
        }
        break;
      }
      case 'A':
      {
        if (addr == ZDT_ADDR_BROADCAST)
        {
          append_line("ERR MA no broadcast");
          return;
        }
        int32_t target = 0;
        if (!parse_i32(p, &target))
        {
          append_line("ERR MA needs abs deg e.g. M1A0 M2A+90");
          return;
        }
        BridgeSlot *s = slot_find(addr);
        if (s == nullptr)
        {
          append_line("ERR no slot");
          return;
        }
        ZdtAbsGotoResult ma{};
        const HAL_StatusTypeDef st = Zdt_AbsGotoLogical(addr, target, &ma);
        if (s != nullptr && st == HAL_OK)
        {
          s->drv_deg = ma.final_logical;
          s->drv_valid = 1U;
          s->last_deg = ma.final_logical - ma.start_logical;
          if (s->last_deg > 0) s->dir = 1U;
          else if (s->last_deg < 0) s->dir = 0U;
        }
        const char *mw =
            (ma.motion_wait == HAL_OK) ? "OK" : "TIMEOUT";
        const char *lim =
            (ma.target_clamped != 0U) ? "clamp" : "none";
        reply_fmt("%s MA addr=%u tgt=%ld lim=%s start=%ld passes=%u mwt=%s "
                  "err_u=%ld pos=%ld(drv) tol<=%ddeg",
                  hal_err(st), static_cast<unsigned>(addr),
                  static_cast<long>(ma.target_logical), lim,
                  static_cast<long>(ma.start_logical),
                  static_cast<unsigned>(ma.passes), mw,
                  static_cast<long>(ma.err_units),
                  static_cast<long>(ma.final_logical),
                  ZDT_MA_TOL_DEG);
        break;
      }
      case 'R':
      {
        if (addr == ZDT_ADDR_BROADCAST)
        {
          append_line("ERR MR no broadcast");
          return;
        }
        static const uint8_t kQueryCmds[] = {
          ZDT_CMD_READ_POS, ZDT_CMD_READ_VER, ZDT_CMD_READ_FLG
        };
        char rxhex[72];
        reply_fmt("OK MR start a=%u", static_cast<unsigned>(addr));
        flush_reply();
        for (uint8_t qi = 0U; qi < sizeof(kQueryCmds); ++qi)
        {
          const uint8_t qcmd = kQueryCmds[qi];
          Zdt_ClearLastRx();
          const HAL_StatusTypeDef st = Zdt_QueryRead(addr, qcmd, ZDT_ACK_TIMEOUT_MS);
          (void)Zdt_FormatLastRx(rxhex, sizeof(rxhex));
          const char *rxst = "FAIL";
          if (st == HAL_OK) rxst = "OK";
          else if (st == HAL_TIMEOUT) rxst = "TIMEOUT";
          else if (st == HAL_BUSY) rxst = "BUSY";
          if (qcmd == ZDT_CMD_READ_POS)
          {
            ZdtPosRead pos{};
            if (st == HAL_OK && Zdt_ParseLastRealtimePos(&pos) != 0)
            {
              const int32_t emm_deg = ZdtProto_UnitsToEmmDeg(pos.units);
              reply_fmt("MR a=%u cmd=%02X %s rx=%s units=%ld deg=%ld parse=OK",
                        static_cast<unsigned>(addr),
                        static_cast<unsigned>(qcmd), rxst, rxhex,
                        static_cast<long>(pos.units),
                        static_cast<long>(emm_deg));
            }
            else
            {
              reply_fmt("MR a=%u cmd=%02X %s rx=%s parse=FAIL",
                        static_cast<unsigned>(addr),
                        static_cast<unsigned>(qcmd), rxst, rxhex);
            }
          }
          else
          {
            reply_fmt("MR a=%u cmd=%02X %s rx=%s parse=NA",
                      static_cast<unsigned>(addr),
                      static_cast<unsigned>(qcmd), rxst, rxhex);
          }
          flush_reply();
          vTaskDelay(pdMS_TO_TICKS(BRIDGE_TEST_GAP_MS));
        }
        append_line("OK MR done");
        break;
      }
      case 'Z':
      {
        if (addr == ZDT_ADDR_BROADCAST)
        {
          append_line("ERR MZ no broadcast");
          return;
        }
        const HAL_StatusTypeDef st = Zdt_Zero(addr);
        BridgeSlot *s = slot_find(addr);
        if (s != nullptr && st == HAL_OK)
        {
          s->fallback_deg = 0;
          slot_refresh_driver_pos(s);
        }
        int32_t pos = 0;
        const char *src = "none";
        if (s != nullptr)
        {
          if (s->drv_valid != 0U)
          {
            pos = s->drv_deg;
            src = "drv";
          }
          else
          {
            pos = s->fallback_deg;
            src = "fallback";
          }
        }
        reply_fmt("%s MZ addr=%u drv_deg=%ld(%s)",
                  hal_err(st), static_cast<unsigned>(addr),
                  static_cast<long>(pos), src);
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
          slot_refresh_driver_pos(&slots_[i]);
          const char *src = (slots_[i].drv_valid != 0U) ? "drv" : "fallback";
          const int32_t pos =
              (slots_[i].drv_valid != 0U) ? slots_[i].drv_deg : slots_[i].fallback_deg;
          reply_fmt("SLOT %u addr=%u dir=%u vel=%u acc=%u pos=%ld(%s) fb=%ld lastMG=%ld",
                    static_cast<unsigned>(i),
                    static_cast<unsigned>(slots_[i].addr),
                    static_cast<unsigned>(slots_[i].dir),
                    static_cast<unsigned>(slots_[i].vel),
                    static_cast<unsigned>(slots_[i].acc),
                    static_cast<long>(pos), src,
                    static_cast<long>(slots_[i].fallback_deg),
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
        slot_refresh_driver_pos(s);
        const char *src = (s->drv_valid != 0U) ? "drv" : "fallback";
        const int32_t pos = (s->drv_valid != 0U) ? s->drv_deg : s->fallback_deg;
        reply_fmt("OK MJ addr=%u dir=%u vel=%u acc=%u pos=%ld(%s) fb=%ld lastMG=%ld",
                  static_cast<unsigned>(s->addr),
                  static_cast<unsigned>(s->dir),
                  static_cast<unsigned>(s->vel),
                  static_cast<unsigned>(s->acc),
                  static_cast<long>(pos), src,
                  static_cast<long>(s->fallback_deg),
                  static_cast<long>(s->last_deg));
        break;
      }
      case '?':
        append_line("OK A-tier: M? ME0/1 MG MA MZ MX MS[n] ML MJ test");
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
