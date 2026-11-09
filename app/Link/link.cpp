#include "link_public.h"
#include "link_config.h"
#include "link_protocol.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include <cstring>

namespace {

uint8_t link_rx_buf[LINK_RX_BUF_SIZE];
uint8_t link_tx_buf[LINK_FRAME_MAX];
QueueHandle_t link_queue = nullptr;

void start_receive()
{
  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, link_rx_buf, sizeof(link_rx_buf));
  /* HAL 启动时会重新使能 HT 中断，每次重启后都要关掉 */
  __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
}

}

extern "C" HAL_StatusTypeDef Link_Send(const uint8_t *data, uint16_t len)
{
  if (data == nullptr || len == 0U || len > static_cast<uint16_t>(sizeof(link_tx_buf))) return HAL_ERROR;
  if (huart6.gState != HAL_UART_STATE_READY) return HAL_BUSY;
  if ((huart6.hdmatx->Instance->CR & DMA_SxCR_EN) != 0U) return HAL_BUSY;
  /* 确认可发送后才写缓冲，避免覆盖上一帧尚未发完的数据 */
  std::memcpy(link_tx_buf, data, len);
  return HAL_UART_Transmit_DMA(&huart6, link_tx_buf, len);
}

extern "C" HAL_StatusTypeDef Link_SetBaudRate(uint32_t baudrate)
{
  if (baudrate < 1200U || baudrate > 5000000U) return HAL_ERROR;
  if (huart6.Init.BaudRate == baudrate) return HAL_OK;

  HAL_UART_DeInit(&huart6);
  huart6.Init.BaudRate = baudrate;
  if (HAL_UART_Init(&huart6) != HAL_OK) return HAL_ERROR;
  start_receive();
  return HAL_OK;
}

extern "C" void StartLinkTask(void *argument)
{
  (void)argument;
  link_queue = xQueueCreate(LINK_QUEUE_LEN, sizeof(LinkFrame));
  LinkProtocol &protocol = Link_GetProtocol();
  LinkFrame frame{};

  if (huart6.Init.BaudRate != LINK_DEFAULT_BAUDRATE)
    Link_SetBaudRate(LINK_DEFAULT_BAUDRATE);
  else
    start_receive();

  for (;;)
  {
    if (xQueueReceive(link_queue, &frame, portMAX_DELAY) == pdTRUE)
    {
      if (protocol.parse(frame))
      {
        const uint16_t n = protocol.make_reply(link_tx_buf, sizeof(link_tx_buf));
        if (n > 0U) Link_Send(link_tx_buf, n);
      }
    }
  }
}

extern "C" void Link_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance != USART6) return;

  BaseType_t woken = pdFALSE;
  if (link_queue != nullptr && Size > 0U && Size <= LINK_FRAME_MAX)
  {
    LinkFrame frame{};
    frame.len = Size;
    std::memcpy(frame.data, link_rx_buf, Size);
    xQueueSendToBackFromISR(link_queue, &frame, &woken);
  }

  std::memset(link_rx_buf, 0, sizeof(link_rx_buf));
  start_receive();
  portYIELD_FROM_ISR(woken);
}

extern "C" void Link_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART6) return;
  __HAL_UNLOCK(huart);
  start_receive();
}
