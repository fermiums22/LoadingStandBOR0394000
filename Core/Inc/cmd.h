/**
  ******************************************************************************
  * @file    cmd.h
  * @brief   Line-based command parser + telemetry stream scheduler.
  ******************************************************************************
  */
#ifndef CMD_H
#define CMD_H

void Cmd_Init(void);
void Cmd_Banner(void);         /* print the BORK welcome logo + hint */
void Cmd_FeedByte(char c);     /* feed received bytes; echoes + parses on CR/LF */
void Cmd_StreamTask(void);     /* emit periodic DATA, call from main loop */

#endif /* CMD_H */
