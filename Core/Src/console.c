/**
  ******************************************************************************
  * @file    console.c
  * @brief   Non-blocking UART console (USART2 VCP) with RX/TX ring buffers.
  ******************************************************************************
  */
#include "main.h"
#include "console.h"
#include <string.h>

/* USART2 + its RX DMA (defined in main.c). */
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef  hdma_usart2_rx;

#define TX_RING_SIZE     (1024u)        /* power of two */
#define TX_RING_MASK     (TX_RING_SIZE - 1u)
#define RX_DMA_SIZE      (256u)         /* power of two */
#define RX_DMA_MASK      (RX_DMA_SIZE - 1u)
#define STREAM_KEEP_FREE (96u)          /* don't queue stream below this free space */

/* TX ring: producer = caller (main loop), consumer = USART2 IRQ. */
static volatile uint8_t  tx_buf[TX_RING_SIZE];
static volatile uint16_t tx_head = 0;
static volatile uint16_t tx_tail = 0;

/* RX ring: producer = DMA, consumer = Console_ReadByte (main loop). */
static volatile uint8_t  rx_dma[RX_DMA_SIZE];
static uint16_t          rx_rd = 0;

static void tx_push(const char *s, bool is_stream)
{
  uint16_t n    = (uint16_t)strlen(s);
  uint16_t used = (uint16_t)((tx_head - tx_tail) & TX_RING_MASK);
  uint16_t freeSpace = (uint16_t)(TX_RING_MASK - used);

  if (is_stream && freeSpace < (uint16_t)(n + STREAM_KEEP_FREE)) return;
  if (freeSpace < n) return;

  for (uint16_t i = 0; i < n; i++)
  {
    tx_buf[tx_head] = (uint8_t)s[i];
    tx_head = (uint16_t)((tx_head + 1) & TX_RING_MASK);
  }
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_TXE);   /* kick the TX interrupt */
}

void Console_Print(const char *s)  { tx_push(s, false); }
void Console_Stream(const char *s) { tx_push(s, true); }

void Console_TxIrq(void)
{
  if (__HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_TXE) &&
      __HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE))
  {
    if (tx_tail != tx_head)
    {
      huart2.Instance->TDR = tx_buf[tx_tail];
      tx_tail = (uint16_t)((tx_tail + 1) & TX_RING_MASK);
    }
    if (tx_tail == tx_head)
    {
      __HAL_UART_DISABLE_IT(&huart2, UART_IT_TXE);
    }
  }
}

bool Console_ReadByte(uint8_t *c)
{
  uint16_t wr = (uint16_t)(RX_DMA_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx));
  if (rx_rd == wr) return false;
  *c = rx_dma[rx_rd];
  rx_rd = (uint16_t)((rx_rd + 1) & RX_DMA_MASK);
  return true;
}

void Console_Init(void)
{
  /* RX: circular DMA ring; write index polled in Console_ReadByte(). */
  HAL_UART_Receive_DMA(&huart2, (uint8_t *)rx_dma, RX_DMA_SIZE);
  /* DMA always empties RDR -> no overrun; keep USART2 IRQ for TX only. */
  __HAL_UART_DISABLE_IT(&huart2, UART_IT_ERR);
  __HAL_UART_DISABLE_IT(&huart2, UART_IT_PE);
  CLEAR_BIT(huart2.Instance->CR3, USART_CR3_EIE);

  HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);      /* lowest prio: below control loop */
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  rx_rd = (uint16_t)(RX_DMA_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx));
}
