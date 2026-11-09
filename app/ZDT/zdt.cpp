#include "zdt_public.h"
#include "zdt_config.h"
#include "zdt_protocol.h"
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
volatile uint8_t last_ack_ok = 0U;
volatile uint8_t wait_any_rx = 0U;

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

void note_ack_from_frame(const ZdtFrame &frame)
{
  if (wait_any_rx != 0U)
  {
    last_ack_ok = 1U;
    if (zdt_ack_sem != nullptr)
    {
      BaseType_t woken = pdFALSE;
      xSemaphoreGiveFromISR(zdt_ack_sem, &woken);
      portYIELD_FROM_ISR(woken);
    }
    return;
  }
  ZdtAck ack{};
  if (!ZdtProto_FindAck(frame.data, frame.len, &ack)) return;
  if (ack.addr == wait_addr && ack.cmd == wait_cmd && ack.status == ZDT_ACK_OK)
  {
    last_ack_ok = 1U;
    if (zdt_ack_sem != nullptr)
    {
      BaseType_t woken = pdFALSE;
      xSemaphoreGiveFromISR(zdt_ack_sem, &woken);
      portYIELD_FROM_ISR(woken);
    }
  }
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

extern "C" HAL_StatusTypeDef Zdt_Vel(uint8_t addr, uint8_t dir, uint16_t vel_rpm)
{
  ZdtSlot *s = slot_find(addr);
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

  if (deg < 0)
  {
    up = 0U;
    abs_deg = static_cast<uint32_t>(-deg);
  }
  else if (deg > 0)
  {
    up = 1U;
    abs_deg = static_cast<uint32_t>(deg);
  }
  else
  {
    return HAL_OK;
  }

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
  wait_addr = addr;
  wait_cmd = cmd;
  last_ack_ok = 0U;
  (void)xSemaphoreTake(zdt_ack_sem, 0);
  if (xSemaphoreTake(zdt_ack_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
  {
    wait_addr = 0xFFU;
    return HAL_TIMEOUT;
  }
  wait_addr = 0xFFU;
  return (last_ack_ok != 0U) ? HAL_OK : HAL_ERROR;
}

extern "C" HAL_StatusTypeDef Zdt_EnableAndWait(uint8_t addr, uint8_t on, uint32_t timeout_ms)
{
  if (zdt_ack_sem == nullptr) return HAL_ERROR;
  Zdt_ClearLastRx();
  wait_addr = addr;
  wait_cmd = ZDT_CMD_ENABLE;
  last_ack_ok = 0U;
  (void)xSemaphoreTake(zdt_ack_sem, 0);

  const HAL_StatusTypeDef st = Zdt_Enable(addr, on);
  if (st != HAL_OK)
  {
    wait_addr = 0xFFU;
    return st;
  }
  if (addr == ZDT_ADDR_BROADCAST)
  {
    wait_addr = 0xFFU;
    return HAL_OK;
  }
  if (xSemaphoreTake(zdt_ack_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
  {
    wait_addr = 0xFFU;
    return HAL_TIMEOUT;
  }
  wait_addr = 0xFFU;
  return (last_ack_ok != 0U) ? HAL_OK : HAL_ERROR;
}

extern "C" HAL_StatusTypeDef Zdt_GoDegAndWait(uint8_t addr, int32_t deg, uint32_t timeout_ms)
{
  if (zdt_ack_sem == nullptr) return HAL_ERROR;
  wait_addr = addr;
  wait_cmd = ZDT_CMD_POS;
  last_ack_ok = 0U;
  (void)xSemaphoreTake(zdt_ack_sem, 0);

  const HAL_StatusTypeDef st = Zdt_GoDeg(addr, deg);
  if (st != HAL_OK)
  {
    wait_addr = 0xFFU;
    return st;
  }
  if (addr == ZDT_ADDR_BROADCAST || deg == 0)
  {
    wait_addr = 0xFFU;
    return HAL_OK;
  }
  if (xSemaphoreTake(zdt_ack_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
  {
    wait_addr = 0xFFU;
    return HAL_TIMEOUT;
  }
  wait_addr = 0xFFU;
  return (last_ack_ok != 0U) ? HAL_OK : HAL_ERROR;
}

extern "C" HAL_StatusTypeDef Zdt_Read(uint8_t addr, uint8_t cmd)
{
  uint8_t f[3];
  const uint8_t n = ZdtProto_EncodeRead(f, sizeof(f), addr, cmd);
  if (n == 0U) return HAL_ERROR;
  return send_frame(f, n);
}

extern "C" HAL_StatusTypeDef Zdt_ReadAndWait(uint8_t addr, uint8_t cmd, uint32_t timeout_ms)
{
  if (zdt_ack_sem == nullptr) return HAL_ERROR;
  if (addr == ZDT_ADDR_BROADCAST) return HAL_ERROR;
  Zdt_ClearLastRx();
  wait_addr = addr;
  wait_cmd = cmd;
  wait_any_rx = 1U;
  last_ack_ok = 0U;
  (void)xSemaphoreTake(zdt_ack_sem, 0);

  const HAL_StatusTypeDef st = Zdt_Read(addr, cmd);
  if (st != HAL_OK)
  {
    wait_any_rx = 0U;
    wait_addr = 0xFFU;
    return st;
  }
  if (xSemaphoreTake(zdt_ack_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
  {
    wait_any_rx = 0U;
    wait_addr = 0xFFU;
    return HAL_TIMEOUT;
  }
  wait_any_rx = 0U;
  wait_addr = 0xFFU;
  return (last_rx_len > 0U) ? HAL_OK : HAL_ERROR;
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

extern "C" uint16_t Zdt_CopyLastRx(uint8_t *out, uint16_t capacity)
{
  if (out == nullptr || capacity == 0U) return 0U;
  const uint16_t n = last_rx_len;
  if (n == 0U) return 0U;
  const uint16_t copy = (n < capacity) ? n : capacity;
  std::memcpy(out, last_rx_copy, copy);
  return copy;
}


extern "C" HAL_StatusTypeDef Zdt_VelAndWait(uint8_t addr, uint8_t dir, uint16_t vel_rpm, uint32_t timeout_ms)
{
  if (zdt_ack_sem == nullptr) return HAL_ERROR;
  Zdt_ClearLastRx();
  wait_addr = addr;
  wait_cmd = ZDT_CMD_VEL;
  last_ack_ok = 0U;
  (void)xSemaphoreTake(zdt_ack_sem, 0);

  const HAL_StatusTypeDef st = Zdt_Vel(addr, dir, vel_rpm);
  if (st != HAL_OK)
  {
    wait_addr = 0xFFU;
    return st;
  }
  if (addr == ZDT_ADDR_BROADCAST)
  {
    wait_addr = 0xFFU;
    return HAL_OK;
  }
  if (xSemaphoreTake(zdt_ack_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
  {
    wait_addr = 0xFFU;
    return HAL_TIMEOUT;
  }
  wait_addr = 0xFFU;
  return (last_ack_ok != 0U) ? HAL_OK : HAL_ERROR;
}

extern "C" HAL_StatusTypeDef Zdt_StopAndWait(uint8_t addr, uint32_t timeout_ms)
{
  if (zdt_ack_sem == nullptr) return HAL_ERROR;
  Zdt_ClearLastRx();
  wait_addr = addr;
  wait_cmd = ZDT_CMD_STOP;
  last_ack_ok = 0U;
  (void)xSemaphoreTake(zdt_ack_sem, 0);

  const HAL_StatusTypeDef st = Zdt_Stop(addr);
  if (st != HAL_OK)
  {
    wait_addr = 0xFFU;
    return st;
  }
  if (addr == ZDT_ADDR_BROADCAST)
  {
    wait_addr = 0xFFU;
    return HAL_OK;
  }
  if (xSemaphoreTake(zdt_ack_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
  {
    wait_addr = 0xFFU;
    return HAL_TIMEOUT;
  }
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
    {
      ZdtAck ack{};
      if (wait_any_rx != 0U)
      {
        last_ack_ok = 1U;
        (void)xSemaphoreGive(zdt_ack_sem);
      }
      else if (ZdtProto_FindAck(frame.data, frame.len, &ack))
      {
        if (ack.addr == wait_addr && ack.cmd == wait_cmd && ack.status == ZDT_ACK_OK)
        {
          last_ack_ok = 1U;
          (void)xSemaphoreGive(zdt_ack_sem);
        }
      }
    }
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
    note_ack_from_frame(frame);
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
