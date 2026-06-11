/**
  ******************************************************************************
  * @file    console.h
  * @brief   Dual-UART console, 921600 8N1: USART2 (ST-Link VCP) + USART1
  *          (PC4/PC5 header, e.g. Raspberry Pi). Both run in parallel.
  *
  * RX: each USART + circular DMA ring (drained by polling); bytes from either
  *     port feed the same parser.
  * TX: per-port software ring drained by that USART's TXE interrupt; output is
  *     mirrored to both ports. No blocking HAL_UART_Transmit / HAL_Delay.
  ******************************************************************************
  */
#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdint.h>
#include <stdbool.h>

#define CONSOLE_NPORT   2             /* port 0 = USART2 (VCP), port 1 = USART1   */
#define CONSOLE_BOTH   (-1)           /* route output to every port               */

void Console_Init(void);              /* start RX DMA + TX IRQ on both ports      */
void Console_Route(int port);         /* set output target: 0,1 or CONSOLE_BOTH   */
void Console_Print(const char *s);    /* enqueue a response to the routed port(s) */
void Console_Stream(const char *s);   /* enqueue telemetry, dropped if near full  */
bool Console_ReadByte(uint8_t *c, int *port);  /* pop a byte + its source port     */
void Console_UartIrq(void *huart);    /* call from USART1/USART2 IRQHandler        */

#endif /* CONSOLE_H */
