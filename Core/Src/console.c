/**
  ******************************************************************************
  * @file    console.c
  * @brief   Non-blocking dual-UART console with per-port RX/TX ring buffers.
  *
  * Two consoles run in parallel and behave as one shared session:
  *   - USART2 = ST-Link VCP (debug console on the PC).
  *   - USART1 = PC4/PC5 header (e.g. Raspberry Pi controlling the Nucleo).
  * Output (responses, logs, telemetry) is mirrored to BOTH ports; input is
  * accepted from BOTH ports and fed into the single command parser.
  ******************************************************************************
  */
#include "main.h"
#include "console.h"
#include <string.h>

/* UARTs + their RX DMA streams (defined in main.c). */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef  hdma_usart1_rx;
extern DMA_HandleTypeDef  hdma_usart2_rx;

#define TX_RING_SIZE     (1024u)        /* power of two */
#define TX_RING_MASK     (TX_RING_SIZE - 1u)
#define RX_DMA_SIZE      (256u)         /* power of two */
#define RX_DMA_MASK      (RX_DMA_SIZE - 1u)
#define STREAM_KEEP_FREE (96u)          /* don't queue stream below this free space */

/* One independent console port: TX ring (drained by its USART TXE IRQ) +
   circular RX DMA ring (drained by polling the DMA counter). */
typedef struct {
  UART_HandleTypeDef *huart;
  DMA_HandleTypeDef  *hdma_rx;
  IRQn_Type           irqn;
  volatile uint8_t    tx_buf[TX_RING_SIZE];
  volatile uint16_t   tx_head;          /* producer = main loop */
  volatile uint16_t   tx_tail;          /* consumer = USART TXE IRQ */
  volatile uint8_t    rx_dma[RX_DMA_SIZE];
  uint16_t            rx_rd;            /* consumer = Console_ReadByte */
} console_t;

#define N_CONSOLE 2
static console_t cons[N_CONSOLE];

/* Output routing: CONSOLE_BOTH = mirror to all ports, else a single port index.
   The command layer sets this per console so each terminal is independent. */
static int g_out = CONSOLE_BOTH;
void Console_Route(int port) { g_out = port; }

/* Enqueue a string to one port's TX ring. Stream traffic is dropped early if
   the ring is nearly full, so telemetry never starves command responses. */
static void port_push(console_t *p, const char *s, uint16_t n, bool is_stream)
{
  uint16_t used      = (uint16_t)((p->tx_head - p->tx_tail) & TX_RING_MASK);
  uint16_t freeSpace = (uint16_t)(TX_RING_MASK - used);

  if (is_stream && freeSpace < (uint16_t)(n + STREAM_KEEP_FREE)) return;
  if (freeSpace < n) return;

  for (uint16_t i = 0; i < n; i++)
  {
    p->tx_buf[p->tx_head] = (uint8_t)s[i];
    p->tx_head = (uint16_t)((p->tx_head + 1) & TX_RING_MASK);
  }
  __HAL_UART_ENABLE_IT(p->huart, UART_IT_TXE);    /* kick the TX interrupt */
}

/* Send output to the currently routed port (or every port if CONSOLE_BOTH). */
static void broadcast(const char *s, bool is_stream)
{
  uint16_t n = (uint16_t)strlen(s);
  if (g_out == CONSOLE_BOTH)
    for (int i = 0; i < N_CONSOLE; i++) port_push(&cons[i], s, n, is_stream);
  else if (g_out >= 0 && g_out < N_CONSOLE)
    port_push(&cons[g_out], s, n, is_stream);
}

void Console_Print(const char *s)  { broadcast(s, false); }
void Console_Stream(const char *s) { broadcast(s, true); }

/* Drain one port's TX ring from its USART TXE interrupt. */
static void port_tx_irq(console_t *p)
{
  if (__HAL_UART_GET_IT_SOURCE(p->huart, UART_IT_TXE) &&
      __HAL_UART_GET_FLAG(p->huart, UART_FLAG_TXE))
  {
    if (p->tx_tail != p->tx_head)
    {
      p->huart->Instance->TDR = p->tx_buf[p->tx_tail];
      p->tx_tail = (uint16_t)((p->tx_tail + 1) & TX_RING_MASK);
    }
    if (p->tx_tail == p->tx_head)
    {
      __HAL_UART_DISABLE_IT(p->huart, UART_IT_TXE);
    }
  }
}

/* Called from USART1_IRQHandler / USART2_IRQHandler (huart is the handle ptr). */
void Console_UartIrq(void *huart)
{
  for (int i = 0; i < N_CONSOLE; i++)
    if (cons[i].huart == (UART_HandleTypeDef *)huart) { port_tx_irq(&cons[i]); return; }
}

/* Pop one received byte from any port (scanned in order) and report which port
   it came from, so the command layer can keep the two consoles independent. */
bool Console_ReadByte(uint8_t *c, int *port)
{
  for (int i = 0; i < N_CONSOLE; i++)
  {
    console_t *p = &cons[i];
    uint16_t wr = (uint16_t)(RX_DMA_SIZE - __HAL_DMA_GET_COUNTER(p->hdma_rx));
    if (p->rx_rd != wr)
    {
      *c = p->rx_dma[p->rx_rd];
      p->rx_rd = (uint16_t)((p->rx_rd + 1) & RX_DMA_MASK);
      *port = i;
      return true;
    }
  }
  return false;
}

static void port_init(console_t *p, UART_HandleTypeDef *huart,
                      DMA_HandleTypeDef *hdma_rx, IRQn_Type irqn)
{
  p->huart   = huart;
  p->hdma_rx = hdma_rx;
  p->irqn    = irqn;
  p->tx_head = 0;
  p->tx_tail = 0;

  /* RX: circular DMA ring; write index polled in Console_ReadByte(). */
  HAL_UART_Receive_DMA(p->huart, (uint8_t *)p->rx_dma, RX_DMA_SIZE);
  /* DMA always empties RDR -> no overrun; keep the USART IRQ for TX only. */
  __HAL_UART_DISABLE_IT(p->huart, UART_IT_ERR);
  __HAL_UART_DISABLE_IT(p->huart, UART_IT_PE);
  CLEAR_BIT(p->huart->Instance->CR3, USART_CR3_EIE);

  HAL_NVIC_SetPriority(p->irqn, 3, 0);          /* lowest prio: below control loop */
  HAL_NVIC_EnableIRQ(p->irqn);

  p->rx_rd = (uint16_t)(RX_DMA_SIZE - __HAL_DMA_GET_COUNTER(p->hdma_rx));
}

void Console_Init(void)
{
  port_init(&cons[0], &huart2, &hdma_usart2_rx, USART2_IRQn);   /* ST-Link VCP */
  port_init(&cons[1], &huart1, &hdma_usart1_rx, USART1_IRQn);   /* Raspberry Pi */
}
