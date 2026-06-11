/**
  ******************************************************************************
  * @file    cmd.h
  * @brief   Line-based command parser + telemetry stream scheduler.
  ******************************************************************************
  */
#ifndef CMD_H
#define CMD_H

void Cmd_Init(void);
void Cmd_Banner(void);            /* print the BORK welcome logo + hint */
void Cmd_FeedByte(int port, char c);  /* feed a byte from a console; echo/parse per port */
void Cmd_StreamTask(void);       /* emit each console's stream to its own port */

#endif /* CMD_H */
