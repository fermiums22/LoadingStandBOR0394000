/**
  ******************************************************************************
  * @file    console.h
  * @brief   Non-blocking UART console over USART2 (ST-Link VCP), 921600 8N1.
  *
  * RX: USART2 + circular DMA ring (drained by polling).
  * TX: software ring buffer drained by the USART2 TXE interrupt.
  * No blocking HAL_UART_Transmit / HAL_Delay is used.
  ******************************************************************************
  */
#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdint.h>
#include <stdbool.h>

void Console_Init(void);              /* start RX DMA, enable TX interrupt        */
void Console_Print(const char *s);    /* enqueue a response (priority over stream)*/
void Console_Stream(const char *s);   /* enqueue telemetry, dropped if near full  */
bool Console_ReadByte(uint8_t *c);    /* pop one received byte; true if available */
void Console_TxIrq(void);             /* call from USART2_IRQHandler               */

#endif /* CONSOLE_H */
