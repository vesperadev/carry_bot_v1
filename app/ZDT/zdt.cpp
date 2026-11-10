#include "zdt_public.h"
#include "zdt_config.h"
#include "zdt_protocol.h"
#include "zdt_dir_map.h"
#include "zdt_limits.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include <cstring>

namespace {

struct ZdtFrame
{
  uint16_t len;
  uint8_t  data[ZDT_FRAME_MAX];
};

uint8_t zdt_rx_buf[ZDT_RX_BUF_SIZE];
uint8_t zdt_tx_buf[ZDT_TX_BUF_SIZE];
QueueHandle_t zdt_rx_queue = nullptr;
SemaphoreHandle_t zdt_ack_sem = nullptr;

ZdtSlot slots[ZDT_SLOT_MAX];
volatile uint8_t wait_addr = 0xFFU;
volatile uint8_t wait_cmd  = 0x00U;
volatile uint8_t wait_kind = 0U; /* 0=idle 1=cmd ack 2=read pos (0x36) */
volatile uint8_t last_ack_ok = 0U;
volatile int32_t wait_read_units = 0;

uint8_t last_rx_copy[ZDT_FRAME_MAX];
volatile uint16_t last_rx_len = 0U;

const uint8_t kPresetAddrs[] = {
  ZDT_ADDR_LIFT,
  ZDT_ADDR_TURN,
  ZDT_ADDR_WHEEL_FL,
  ZDT_ADDR_WHEEL_RL,
  ZDT_ADDR_WHEEL_FR,
  ZDT_ADDR_WHEEL_RR,
};

void start_receive()
{
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, zdt_rx_buf, sizeof(zdt_rx_buf));
  __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

void slot_reset(ZdtSlot *s, uint8_t addr)
{
  s->used    = 1U;
  s->addr    = addr;
  s->dir     = 0U;
  s->acc     = static_cast<uint8_t>(ZDT_DEFAULT_ACC);
  s->mode    = 0U;
  s->snF     = 0U;
  s->vel     = static_cast<uint16_t>(ZDT_DEFAULT_VEL_RPM);
  s->pos_clk = 0;
}

void slots_init()
{
  std::memset(slots, 0, sizeof(slots));
  for (uint8_t i = 0U; i < sizeof(kPresetAddrs); ++i)
    slot_reset(&slots[i], kPresetAddrs[i]);
}

ZdtSlot *slot_find(uint8_t addr)
{
  if (addr == ZDT_ADDR_BROADCAST)
  {
    static ZdtSlot view{};
    slot_reset(&view, ZDT_ADDR_BROADCAST);
    return &view;
  }
  for (uint8_t i = 0U; i < ZDT_SLOT_MAX; ++i)
  {
    if (slots[i].used && slots[i].addr == addr) return &slots[i];
  }
  for (uint8_t i = 0U; i < ZDT_SLOT_MAX; ++i)
  {
    if (!slots[i].used)
    {
      slot_reset(&slots[i], addr);
      return &slots[i];
    }
  }
  return &slots[0];
}

HAL_StatusTypeDef send_frame(const uint8_t *data, uint16_t len)
{
  if (data == nullptr || len == 0U || len > static_cast<uint16_t>(sizeof(zdt_tx_buf))) return HAL_ERROR;
  if (huart1.gState != HAL_UART_STATE_READY) return HAL_BUSY;
  if (huart1.hdmatx == nullptr) return HAL_ERROR;
  if ((huart1.hdmatx->Instance->CR & DMA_SxCR_EN) != 0U) return HAL_BUSY;
  std::memcpy(zdt_tx_buf, data, len);
  return HAL_UART_Transmit_DMA(&huart1, zdt_tx_buf, len);
}

void give_wait_sem(BaseType_t *woken)
{
  if (zdt_ack_sem == nullptr) return;
  last_ack_ok = 1U;
  if (xPortIsInsideInterrupt() != pdFALSE)
  {
    BaseType_t w = pdFALSE;
    xSemaphoreGiveFromISR(zdt_ack_sem, &w);
    if (woken != nullptr)
    {
      if (w) *woken = pdTRUE;
    }
    else
    {
      portYIELD_FROM_ISR(w);
    }
  }
  else
  {
    (void)xSemaphoreGive(zdt_ack_sem);
  }
}

int32_t units_to_logical_deg(uint8_t addr, int32_t units)
{
  const int64_t deg64 =
      (static_cast<int64_t>(units) * 360LL) / static_cast<int64_t>(ZDT_POS_UNITS_PER_REV);
  return Zdt_ApplyDirDeg(addr, static_cast<int32_t>(deg64));
}

extern "C" int32_t Zdt_WireUnitsToLogicalDeg(uint8_t addr, int32_t units)
{
  return units_to_logical_deg(addr, units);
}

extern "C" int32_t Zdt_LogicalDegToWireUnits(uint8_t addr, int32_t logical_deg)
{
  const int32_t wire_deg = Zdt_ApplyDirDeg(addr, logical_deg);
  const int64_t u =
      (static_cast<int64_t>(wire_deg) * static_cast<int64_t>(ZDT_POS_UNITS_PER_REV)) / 360LL;
  return static_cast<int32_t>(u);
}

extern "C" HAL_StatusTypeDef Zdt_GoPosUnits(uint8_t addr, int32_t delta_units)
{
  if (delta_units == 0) return HAL_OK;
  ZdtSlot *s = slot_find(addr);
  const uint8_t up = Zdt_PosWireDirBit(addr, delta_units);
  const uint32_t clk = static_cast<uint32_t>(delta_units < 0 ? -delta_units : delta_units);
  s->dir = up ? 1U : 0U;
  if (addr != ZDT_ADDR_BROADCAST) s->addr = addr;
  ZdtSlot wire = *s;
  wire.addr = addr;
  uint8_t f[13];
  const uint8_t n = ZdtProto_EncodeGoPulses(f, sizeof(f), &wire, clk);
  if (n == 0U) return HAL_ERROR;
  if (addr != ZDT_ADDR_BROADCAST)
  {
    if (up) s->pos_clk += static_cast<int32_t>(clk);
    else    s->pos_clk -= static_cast<int32_t>(clk);
  }
  return send_frame(f, n);
}

bool frame_matches_query(const ZdtFrame &frame)
{
  if (frame.len < 2U) return false;
  if (frame.data[0] != wait_addr) return false;
  if (frame.data[1] == wait_cmd) return true;
  /* Emm err: addr 00 EE 6B — still count as response to our addr at [0]. */
  if (frame.len >= 3U && frame.data[0] == wait_addr) return true;
  return false;
}

bool last_rx_matches_query(void)
{
  ZdtFrame frame{};
  uint16_t n = 0U;
  taskENTER_CRITICAL();
  n = last_rx_len;
  if (n > ZDT_FRAME_MAX) n = ZDT_FRAME_MAX;
  frame.len = n;
  if (n > 0U) std::memcpy(frame.data, last_rx_copy, n);
  taskEXIT_CRITICAL();
  return frame_matches_query(frame);
}

HAL_StatusTypeDef send_frame_with_retry(const uint8_t *data, uint16_t len)
{
  for (uint8_t attempt = 0U; attempt < 8U; ++attempt)
  {
    const HAL_StatusTypeDef st = send_frame(data, len);
    if (st != HAL_BUSY) return st;
    vTaskDelay(pdMS_TO_TICKS(2U));
  }
  return HAL_BUSY;
}

void note_wait_from_frame(const ZdtFrame &frame, BaseType_t *woken)
{
  /* wait_kind==3: QueryRead polls last_rx_copy; do not sem-post (ISR+task dup). */
  if (wait_kind == 3U) return;
  if (wait_kind == 2U)
  {
    ZdtPosRead pos{};
    if (ZdtProto_ParseRealtimePos(frame.data, frame.len, &pos) &&
        pos.addr == wait_addr)
    {
      wait_read_units = pos.units;
      give_wait_sem(woken);
      return;
    }
  }
  if (wait_kind != 1U) return;
  ZdtAck ack{};
  if (!ZdtProto_FindAck(frame.data, frame.len, &ack)) return;
  if (ack.addr == wait_addr && ack.cmd == wait_cmd && ack.status == ZDT_ACK_OK)
    give_wait_sem(woken);
}

#if ZDT_SMOKE_ENABLE
void smoke_me_mg()
{
  const uint8_t addr = static_cast<uint8_t>(ZDT_SMOKE_ADDR);
  (void)Zdt_EnableAndWait(addr, 1U, ZDT_ACK_TIMEOUT_MS);
  (void)Zdt_GoDegAndWait(addr, 1, ZDT_ACK_TIMEOUT_MS);
}
#endif

} // namespace

extern "C" HAL_StatusTypeDef Zdt_SendRaw(const uint8_t *data, uint16_t len)
{
  return send_frame(data, len);
}

extern "C" HAL_StatusTypeDef Zdt_Enable(uint8_t addr, uint8_t on)
{
  uint8_t f[6];
  ZdtSlot *s = slot_find(addr);
  const uint8_t n = ZdtProto_EncodeEnable(f, sizeof(f), addr, on, s->snF);
  if (n == 0U) return HAL_ERROR;
  return send_frame(f, n);
}

extern "C" HAL_StatusTypeDef Zdt_Stop(uint8_t addr)
{
  uint8_t f[5];
  ZdtSlot *s = slot_find(addr);
  const uint8_t n = ZdtProto_EncodeStop(f, sizeof(f), addr, s->snF);
  if (n == 0U) return HAL_ERROR;
  return send_frame(f, n);
}

extern "C" HAL_StatusTypeDef Zdt_Zero(uint8_t addr)
{
  if (addr == ZDT_ADDR_BROADCAST) return HAL_ERROR;
  uint8_t f[4];
  const uint8_t n = ZdtProto_EncodeZero(f, sizeof(f), addr);
  if (n == 0U) return HAL_ERROR;
  ZdtSlot *s = slot_find(addr);
  if (s != nullptr) s->pos_clk = 0;
  return send_frame(f, n);
}

extern "C" HAL_StatusTypeDef Zdt_Vel(uint8_t addr, uint8_t dir, uint16_t vel_rpm)
{
  ZdtSlot *s = slot_find(addr);
  dir = Zdt_ApplyDirBit(addr, dir);
  s->dir = dir ? 1U : 0U;
  s->vel = vel_rpm;
  if (addr != ZDT_ADDR_BROADCAST) s->addr = addr;

  ZdtSlot wire = *s;
  wire.addr = addr;

  uint8_t f[8];
  const uint8_t n = ZdtProto_EncodeVel(f, sizeof(f), &wire);
  if (n == 0U) return HAL_ERROR;
  return send_frame(f, n);
}

extern "C" HAL_StatusTypeDef Zdt_GoDeg(uint8_t addr, int32_t deg)
{
  ZdtSlot *s = slot_find(addr);
  uint8_t up;
  uint32_t abs_deg;

  deg = Zdt_ApplyDirDeg(addr, deg);

  if (deg == 0) return HAL_OK;

  up = Zdt_PosWireDirBit(addr, deg);
  abs_deg = static_cast<uint32_t>(deg < 0 ? -deg : deg);

  s->dir = up ? 1U : 0U;
  if (addr != ZDT_ADDR_BROADCAST) s->addr = addr;

  const uint32_t clk = ZdtProto_DegToPulses(abs_deg);
  if (clk == 0U) return HAL_OK;

  ZdtSlot wire = *s;
  wire.addr = addr;

  uint8_t f[13];
  const uint8_t n = ZdtProto_EncodeGoPulses(f, sizeof(f), &wire, clk);
  if (n == 0U) return HAL_ERROR;

  if (addr != ZDT_ADDR_BROADCAST)
  {
    if (up) s->pos_clk += static_cast<int32_t>(clk);
    else    s->pos_clk -= static_cast<int32_t>(clk);
  }

  return send_frame(f, n);
}

extern "C" HAL_StatusTypeDef Zdt_WaitAck(uint8_t addr, uint8_t cmd, uint32_t timeout_ms)
{
  if (zdt_ack_sem == nullptr) return HAL_ERROR;
  wait_kind = 1U;
  wait_addr = addr;
  wait_cmd = cmd;
  last_ack_ok = 0U;
  (void)xSemaphoreTake(zdt_ack_sem, 0);
  if (xSemaphoreTake(zdt_ack_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
  {
    wait_kind = 0U;
    wait_addr = 0xFFU;
    return HAL_TIMEOUT;
  }
  wait_kind = 0U;
  wait_addr = 0xFFU;
  return (last_ack_ok != 0U) ? HAL_OK : HAL_ERROR;
}

extern "C" HAL_StatusTypeDef Zdt_QueryRead(uint8_t addr, uint8_t cmd, uint32_t timeout_ms)
{
  if (addr == ZDT_ADDR_BROADCAST) return HAL_ERROR;

  if (zdt_rx_queue != nullptr)
  {
    ZdtFrame junk{};
    while (xQueueReceive(zdt_rx_queue, &junk, 0) == pdTRUE) { /* drop stale */ }
  }

  Zdt_ClearLastRx();
  wait_kind = 3U;
  wait_addr = addr;
  wait_cmd = cmd;
  last_ack_ok = 0U;

  uint8_t f[3];
  const uint8_t n = ZdtProto_EncodeRead(f, sizeof(f), addr, cmd);
  if (n == 0U)
  {
    wait_kind = 0U;
    wait_addr = 0xFFU;
    return HAL_ERROR;
  }
  const HAL_StatusTypeDef tx = send_frame_with_retry(f, n);
  if (tx != HAL_OK)
  {
    wait_kind = 0U;
    wait_addr = 0xFFU;
    return tx;
  }

  const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
  while (xTaskGetTickCount() <= deadline)
  {
    if (last_rx_matches_query())
    {
      last_ack_ok = 1U;
      wait_kind = 0U;
      wait_addr = 0xFFU;
      return HAL_OK;
    }
    vTaskDelay(pdMS_TO_TICKS(2U));
  }
  wait_kind = 0U;
  wait_addr = 0xFFU;
  return HAL_TIMEOUT;
}

extern "C" int Zdt_ParseLastRealtimePos(ZdtPosRead *out)
{
  if (out == nullptr) return 0;
  return ZdtProto_ParseRealtimePos(last_rx_copy, last_rx_len, out);
}

static HAL_StatusTypeDef read_wire_pos(uint8_t addr, ZdtPosRead *pos, uint32_t query_tmo)
{
  if (pos == nullptr) return HAL_ERROR;
  if (Zdt_QueryRead(addr, ZDT_CMD_READ_POS, query_tmo) != HAL_OK) return HAL_TIMEOUT;
  if (!ZdtProto_ParseRealtimePos(last_rx_copy, last_rx_len, pos) || pos->addr != addr)
    return HAL_ERROR;
  return HAL_OK;
}

extern "C" HAL_StatusTypeDef Zdt_ReadRealtimePosDeg(uint8_t addr, int32_t *deg_out,
                                                    uint32_t timeout_ms)
{
  if (deg_out == nullptr) return HAL_ERROR;
  ZdtPosRead pos{};
  const HAL_StatusTypeDef st = read_wire_pos(addr, &pos, timeout_ms);
  if (st != HAL_OK) return st;
  *deg_out = units_to_logical_deg(addr, pos.units);
  return HAL_OK;
}

extern "C" int32_t Zdt_ClampRelativeDeg(uint8_t addr, int32_t rel_deg)
{
  const ZdtSoftLimit lim = Zdt_SoftLimitForAddr(addr);
  if (lim.enable == 0U) return rel_deg;
  int32_t cur = 0;
  if (Zdt_ReadRealtimePosDeg(addr, &cur, ZDT_ACK_TIMEOUT_MS) != HAL_OK) return rel_deg;
  uint8_t clamped = 0U;
  const int32_t end =
      Zdt_ClampLogicalTarget(addr, cur + rel_deg, &clamped);
  (void)clamped;
  return end - cur;
}

extern "C" HAL_StatusTypeDef Zdt_WaitMotionSettled(uint8_t addr, uint32_t timeout_ms)
{
  const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
  uint8_t stable = 0U;
  while (xTaskGetTickCount() <= deadline)
  {
    bool settled = false;
    if (Zdt_QueryRead(addr, ZDT_CMD_READ_FLG, ZDT_ACK_TIMEOUT_MS) == HAL_OK)
    {
      uint8_t flags = 0U;
      if (ZdtProto_ParseStatusFlags(last_rx_copy, last_rx_len, nullptr, &flags) != 0 &&
          (flags & ZDT_ST_POS_REACHED) != 0U)
        settled = true;
    }
    if (!settled &&
        Zdt_QueryRead(addr, ZDT_CMD_READ_VEL, ZDT_ACK_TIMEOUT_MS) == HAL_OK)
    {
      int32_t rpm = 0;
      if (ZdtProto_ParseRealtimeVel(last_rx_copy, last_rx_len, &rpm) != 0 && rpm == 0)
        settled = true;
    }
    if (settled)
    {
      stable++;
      if (stable >= ZDT_MA_ARRIVE_STABLE_CNT) return HAL_OK;
    }
    else
    {
      stable = 0U;
    }
    vTaskDelay(pdMS_TO_TICKS(ZDT_MA_POLL_MS));
  }
  return HAL_TIMEOUT;
}

extern "C" HAL_StatusTypeDef Zdt_AbsGotoLogical(uint8_t addr, int32_t target_logical,
                                                  ZdtAbsGotoResult *out)
{
  if (out == nullptr || addr == ZDT_ADDR_BROADCAST) return HAL_ERROR;
  std::memset(out, 0, sizeof(*out));
  out->target_logical = target_logical;
  out->motion_wait = HAL_OK;
  out->move_st = HAL_OK;

  uint8_t clamped = 0U;
  target_logical = Zdt_ClampLogicalTarget(addr, target_logical, &clamped);
  out->target_logical = target_logical;
  out->target_clamped = clamped;

  const int32_t tgt_u = Zdt_LogicalDegToWireUnits(addr, target_logical);
  ZdtPosRead pos{};
  if (read_wire_pos(addr, &pos, ZDT_ACK_TIMEOUT_MS) != HAL_OK)
    return HAL_TIMEOUT;
  out->start_logical = units_to_logical_deg(addr, pos.units);

  for (uint8_t pass = 0U; pass < ZDT_MA_MAX_PASSES; ++pass)
  {
    const int32_t err_u = tgt_u - pos.units;
    const int32_t aerr = (err_u < 0) ? -err_u : err_u;
    if (aerr <= ZDT_MA_TOL_UNITS) break;

    out->passes = static_cast<uint8_t>(out->passes + 1U);
    out->move_st = Zdt_GoPosUnits(addr, err_u);
    if (out->move_st != HAL_OK) break;

    const HAL_StatusTypeDef mw =
        Zdt_WaitMotionSettled(addr, ZDT_MA_MOTION_WAIT_MS);
    if (mw != HAL_OK) out->motion_wait = mw;

    if (read_wire_pos(addr, &pos, ZDT_ACK_TIMEOUT_MS) != HAL_OK)
    {
      out->move_st = HAL_TIMEOUT;
      break;
    }
  }

  out->final_logical = units_to_logical_deg(addr, pos.units);
  out->err_units = tgt_u - pos.units;
  if (out->move_st != HAL_OK) return out->move_st;
  return HAL_OK;
}

extern "C" HAL_StatusTypeDef Zdt_EnableAndWait(uint8_t addr, uint8_t on, uint32_t timeout_ms)
{
  if (zdt_ack_sem == nullptr) return HAL_ERROR;
  Zdt_ClearLastRx();
  wait_kind = 1U;
  wait_addr = addr;
  wait_cmd = ZDT_CMD_ENABLE;
  last_ack_ok = 0U;
  (void)xSemaphoreTake(zdt_ack_sem, 0);

  const HAL_StatusTypeDef st = Zdt_Enable(addr, on);
  if (st != HAL_OK)
  {
    wait_kind = 0U;
    wait_addr = 0xFFU;
    return st;
  }
  if (addr == ZDT_ADDR_BROADCAST)
  {
    wait_kind = 0U;
    wait_addr = 0xFFU;
    return HAL_OK;
  }
  if (xSemaphoreTake(zdt_ack_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
  {
    wait_kind = 0U;
    wait_addr = 0xFFU;
    return HAL_TIMEOUT;
  }
  wait_kind = 0U;
  wait_addr = 0xFFU;
  return (last_ack_ok != 0U) ? HAL_OK : HAL_ERROR;
}

extern "C" HAL_StatusTypeDef Zdt_GoDegAndWait(uint8_t addr, int32_t deg, uint32_t timeout_ms)
{
  if (zdt_ack_sem == nullptr) return HAL_ERROR;
  wait_kind = 1U;
  wait_addr = addr;
  wait_cmd = ZDT_CMD_POS;
  last_ack_ok = 0U;
  (void)xSemaphoreTake(zdt_ack_sem, 0);

  const HAL_StatusTypeDef st = Zdt_GoDeg(addr, deg);
  if (st != HAL_OK)
  {
    wait_kind = 0U;
    wait_addr = 0xFFU;
    return st;
  }
  if (addr == ZDT_ADDR_BROADCAST || deg == 0)
  {
    wait_kind = 0U;
    wait_addr = 0xFFU;
    return HAL_OK;
  }
  if (xSemaphoreTake(zdt_ack_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
  {
    wait_kind = 0U;
    wait_addr = 0xFFU;
    return HAL_TIMEOUT;
  }
  wait_kind = 0U;
  wait_addr = 0xFFU;
  return (last_ack_ok != 0U) ? HAL_OK : HAL_ERROR;
}

extern "C" void Zdt_ClearLastRx(void)
{
  last_rx_len = 0U;
}

extern "C" uint16_t Zdt_FormatLastRx(char *out, uint16_t capacity)
{
  if (out == nullptr || capacity == 0U) return 0U;
  out[0] = '\0';
  const uint16_t n = last_rx_len;
  if (n == 0U)
  {
    if (capacity < 2U) return 0U;
    out[0] = '-';
    out[1] = '\0';
    return 1U;
  }
  uint16_t pos = 0U;
  for (uint16_t i = 0U; i < n; ++i)
  {
    if (pos + 3U >= capacity) break;
    static const char kHex[] = "0123456789ABCDEF";
    if (i > 0U) out[pos++] = ' ';
    out[pos++] = kHex[(last_rx_copy[i] >> 4) & 0x0FU];
    out[pos++] = kHex[last_rx_copy[i] & 0x0FU];
  }
  out[pos] = '\0';
  return pos;
}

extern "C" HAL_StatusTypeDef Zdt_VelAndWait(uint8_t addr, uint8_t dir, uint16_t vel_rpm, uint32_t timeout_ms)
{
  if (zdt_ack_sem == nullptr) return HAL_ERROR;
  Zdt_ClearLastRx();
  wait_kind = 1U;
  wait_addr = addr;
  wait_cmd = ZDT_CMD_VEL;
  last_ack_ok = 0U;
  (void)xSemaphoreTake(zdt_ack_sem, 0);

  const HAL_StatusTypeDef st = Zdt_Vel(addr, dir, vel_rpm);
  if (st != HAL_OK)
  {
    wait_kind = 0U;
    wait_addr = 0xFFU;
    return st;
  }
  if (addr == ZDT_ADDR_BROADCAST)
  {
    wait_kind = 0U;
    wait_addr = 0xFFU;
    return HAL_OK;
  }
  if (xSemaphoreTake(zdt_ack_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
  {
    wait_kind = 0U;
    wait_addr = 0xFFU;
    return HAL_TIMEOUT;
  }
  wait_kind = 0U;
  wait_addr = 0xFFU;
  return (last_ack_ok != 0U) ? HAL_OK : HAL_ERROR;
}

extern "C" HAL_StatusTypeDef Zdt_ZeroAndWait(uint8_t addr, uint32_t timeout_ms)
{
  if (zdt_ack_sem == nullptr) return HAL_ERROR;
  if (addr == ZDT_ADDR_BROADCAST) return HAL_ERROR;
  Zdt_ClearLastRx();
  wait_kind = 1U;
  wait_addr = addr;
  wait_cmd = ZDT_CMD_ZERO;
  last_ack_ok = 0U;
  (void)xSemaphoreTake(zdt_ack_sem, 0);

  const HAL_StatusTypeDef st = Zdt_Zero(addr);
  if (st != HAL_OK)
  {
    wait_kind = 0U;
    wait_addr = 0xFFU;
    return st;
  }
  if (xSemaphoreTake(zdt_ack_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
  {
    wait_kind = 0U;
    wait_addr = 0xFFU;
    return HAL_TIMEOUT;
  }
  wait_kind = 0U;
  wait_addr = 0xFFU;
  return (last_ack_ok != 0U) ? HAL_OK : HAL_ERROR;
}

extern "C" HAL_StatusTypeDef Zdt_StopAndWait(uint8_t addr, uint32_t timeout_ms)
{
  if (zdt_ack_sem == nullptr) return HAL_ERROR;
  Zdt_ClearLastRx();
  wait_kind = 1U;
  wait_addr = addr;
  wait_cmd = ZDT_CMD_STOP;
  last_ack_ok = 0U;
  (void)xSemaphoreTake(zdt_ack_sem, 0);

  const HAL_StatusTypeDef st = Zdt_Stop(addr);
  if (st != HAL_OK)
  {
    wait_kind = 0U;
    wait_addr = 0xFFU;
    return st;
  }
  if (addr == ZDT_ADDR_BROADCAST)
  {
    wait_kind = 0U;
    wait_addr = 0xFFU;
    return HAL_OK;
  }
  if (xSemaphoreTake(zdt_ack_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
  {
    wait_kind = 0U;
    wait_addr = 0xFFU;
    return HAL_TIMEOUT;
  }
  wait_kind = 0U;
  wait_addr = 0xFFU;
  return (last_ack_ok != 0U) ? HAL_OK : HAL_ERROR;
}

extern "C" void StartZdtTask(void *argument)
{
  (void)argument;
  slots_init();
  zdt_rx_queue = xQueueCreate(ZDT_QUEUE_LEN, sizeof(ZdtFrame));
  zdt_ack_sem = xSemaphoreCreateBinary();

  if (huart1.Init.BaudRate != ZDT_BAUDRATE)
  {
    HAL_UART_DeInit(&huart1);
    huart1.Init.BaudRate = ZDT_BAUDRATE;
    (void)HAL_UART_Init(&huart1);
  }
  start_receive();

#if ZDT_SMOKE_ENABLE
  vTaskDelay(pdMS_TO_TICKS(50));
  smoke_me_mg();
#endif

  ZdtFrame frame{};
  for (;;)
  {
    if (xQueueReceive(zdt_rx_queue, &frame, portMAX_DELAY) == pdTRUE)
      note_wait_from_frame(frame, nullptr);
  }
}

extern "C" void Zdt_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == nullptr || huart->Instance != USART1) return;

  BaseType_t woken = pdFALSE;
  if (Size > 0U && Size <= ZDT_FRAME_MAX)
  {
    std::memcpy(last_rx_copy, zdt_rx_buf, Size);
    last_rx_len = Size;
  }
  if (zdt_rx_queue != nullptr && Size > 0U && Size <= ZDT_FRAME_MAX)
  {
    ZdtFrame frame{};
    frame.len = Size;
    std::memcpy(frame.data, zdt_rx_buf, Size);
    note_wait_from_frame(frame, &woken);
    xQueueSendToBackFromISR(zdt_rx_queue, &frame, &woken);
  }

  std::memset(zdt_rx_buf, 0, sizeof(zdt_rx_buf));
  start_receive();
  portYIELD_FROM_ISR(woken);
}

extern "C" void Zdt_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == nullptr || huart->Instance != USART1) return;
  __HAL_UNLOCK(huart);
  start_receive();
}

extern "C" void Link_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
extern "C" void Link_UART_ErrorCallback(UART_HandleTypeDef *huart);

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == nullptr) return;
  if (huart->Instance == USART1)
    Zdt_UART_RxEventCallback(huart, Size);
  else if (huart->Instance == USART6)
    Link_UART_RxEventCallback(huart, Size);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == nullptr) return;
  if (huart->Instance == USART1)
    Zdt_UART_ErrorCallback(huart);
  else if (huart->Instance == USART6)
    Link_UART_ErrorCallback(huart);
}
