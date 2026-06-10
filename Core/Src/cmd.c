/**
  ******************************************************************************
  * @file    cmd.c
  * @brief   Command-line parser for the loading-stand PI current controller.
  *
  * Commands:
  *   help                  set <mA>            kp [v] / ki [v]
  *   led on|off|toggle     reg on|off          imax <mA> / dmax <0..100>
  *   pwm <0..100>          stream on|off       rate <ms>
  *   cal                   status
  ******************************************************************************
  */
#include "main.h"
#include "cmd.h"
#include "console.h"
#include "control.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define CMD_BUF_SIZE     (96u)
#define LED_MANUAL_PCT   (50.0f)        /* duty applied by "led on" */

static char     cmd_buf[CMD_BUF_SIZE];
static uint16_t cmd_len = 0;
static bool     led_state = false;

static bool     stream_on   = false;
static uint32_t stream_rate = 100;      /* [ms] */
static uint32_t stream_last = 0;

/* Format a float with 3 decimals WITHOUT pulling in newlib float-printf. */
static void f3(char *b, size_t n, float v)
{
  int neg = (v < 0.0f);
  if (neg) v = -v;
  long ip = (long)v;
  long fp = (long)((v - (float)ip) * 1000.0f + 0.5f);
  if (fp >= 1000) { ip++; fp -= 1000; }
  snprintf(b, n, "%s%ld.%03ld", neg ? "-" : "", ip, fp);
}

void Cmd_Banner(void)
{
  Console_Print("\r\n");
  Console_Print("  ####    ###    ####   #   #\r\n");
  Console_Print("  #   #  #   #   #   #   #  #\r\n");
  Console_Print("  ####   #   #   ####   ###\r\n");
  Console_Print("  #   #  #   #   #  #    #  #\r\n");
  Console_Print("  ####    ###    #   #   #   #\r\n");
  Console_Print("===================================\r\n");
  Console_Print("  BORK  Loading Stand  BOR0394000\r\n");
  Console_Print("  PI current control @ 10 kHz\r\n");
  Console_Print("===================================\r\n");
  Console_Print("Type 'help'.\r\n");
}

static void cmd_help(void)
{
  Console_Print("Commands:\r\n");
  Console_Print("  help                 - this help\r\n");
  Console_Print("  led on|off|toggle    - manual PWM 50%/0 on PA5 (LED+transistor)\r\n");
  Console_Print("  pwm <0..100>         - manual duty %, stops regulator\r\n");
  Console_Print("  set <mA>             - current setpoint, starts PI regulator\r\n");
  Console_Print("  vt on|off            - transistor output on/off (vt off = stop)\r\n");
  Console_Print("  kp [value]           - get/set proportional gain\r\n");
  Console_Print("  ki [value]           - get/set integral gain\r\n");
  Console_Print("  imax <mA>            - max current setpoint (safety)\r\n");
  Console_Print("  dmax <0..100>        - max duty clamp % (safety)\r\n");
  Console_Print("  cal                  - re-run current-sensor zero calibration\r\n");
  Console_Print("  stream on|off        - DATA,<tick_ms>,<set_mA>,<I_mA>,<duty%>\r\n");
  Console_Print("  rate <ms>            - stream period\r\n");
  Console_Print("  status               - print current state\r\n");
  Console_Print("  logo                 - show the BORK banner\r\n");
}

static void cmd_status(void)
{
  char b[96], fp[24], fi[24];
  out_mode_t mode = Reg_GetMode();
  const char *m = (mode == OUT_REG) ? "VT" : (mode == OUT_MANUAL) ? "MANUAL" : "OFF";
  f3(fp, sizeof fp, Reg_GetKp());
  f3(fi, sizeof fi, Reg_GetKi());
  snprintf(b, sizeof b, "STATUS mode=%s set=%ldmA I=%ldmA duty=%lu%% cal=%d\r\n",
           m, (long)Reg_GetSetpoint_mA(), (long)Reg_GetCurrent_mA(),
           (unsigned long)Reg_GetDutyPct(), Reg_IsCalDone() ? 1 : 0);
  Console_Print(b);
  snprintf(b, sizeof b, "       kp=%s ki=%s imax=%ldmA dmax=%ld%%\r\n",
           fp, fi, (long)Reg_GetImax_mA(), (long)Reg_GetDmaxPct());
  Console_Print(b);
}

static void cmd_dispatch(char *line)
{
  char *cmd = strtok(line, " \t");
  if (!cmd) return;
  char *arg = strtok(NULL, " \t");
  char b[64];

  if (!strcmp(cmd, "help"))   { cmd_help();    return; }
  if (!strcmp(cmd, "logo"))   { Cmd_Banner();  return; }
  if (!strcmp(cmd, "status")) { cmd_status();  return; }

  if (!strcmp(cmd, "led"))
  {
    if (arg && !strcmp(arg, "on"))       { Reg_SetManualPct(LED_MANUAL_PCT); led_state = true;  Console_Print("OK led on\r\n"); }
    else if (arg && !strcmp(arg, "off")) { Reg_SetMode(OUT_OFF);             led_state = false; Console_Print("OK led off\r\n"); }
    else if (arg && !strcmp(arg, "toggle"))
    {
      if (led_state) { Reg_SetMode(OUT_OFF);             led_state = false; Console_Print("OK led off\r\n"); }
      else           { Reg_SetManualPct(LED_MANUAL_PCT); led_state = true;  Console_Print("OK led on\r\n"); }
    }
    else Console_Print("ERR led on|off|toggle\r\n");
    return;
  }

  if (!strcmp(cmd, "pwm"))
  {
    if (!arg) { Console_Print("ERR pwm <0..100>\r\n"); return; }
    float p = strtof(arg, NULL);
    Reg_SetManualPct(p);
    led_state = (p > 0.0f);
    snprintf(b, sizeof b, "OK pwm %ld%%\r\n", (long)p); Console_Print(b);
    return;
  }

  if (!strcmp(cmd, "set"))
  {
    if (!arg) { Console_Print("ERR set <mA>\r\n"); return; }
    Reg_SetSetpoint_mA(strtof(arg, NULL));
    snprintf(b, sizeof b, "OK set %ld mA\r\n", (long)Reg_GetSetpoint_mA()); Console_Print(b);
    return;
  }

  if (!strcmp(cmd, "vt"))
  {
    if (arg && !strcmp(arg, "on"))       { Reg_SetMode(OUT_REG); Console_Print("OK vt on\r\n"); }
    else if (arg && !strcmp(arg, "off")) { Reg_SetMode(OUT_OFF); Console_Print("OK vt off\r\n"); }
    else Console_Print("ERR vt on|off\r\n");
    return;
  }

  if (!strcmp(cmd, "kp"))
  {
    if (arg) Reg_SetKp(strtof(arg, NULL));
    char f[24]; f3(f, sizeof f, Reg_GetKp());
    snprintf(b, sizeof b, "OK kp=%s\r\n", f); Console_Print(b);
    return;
  }

  if (!strcmp(cmd, "ki"))
  {
    if (arg) Reg_SetKi(strtof(arg, NULL));
    char f[24]; f3(f, sizeof f, Reg_GetKi());
    snprintf(b, sizeof b, "OK ki=%s\r\n", f); Console_Print(b);
    return;
  }

  if (!strcmp(cmd, "imax"))
  {
    if (arg) Reg_SetImax_mA(strtof(arg, NULL));
    snprintf(b, sizeof b, "OK imax=%ld mA\r\n", (long)Reg_GetImax_mA()); Console_Print(b);
    return;
  }

  if (!strcmp(cmd, "dmax"))
  {
    if (arg) Reg_SetDmaxPct(strtof(arg, NULL));
    snprintf(b, sizeof b, "OK dmax=%ld%%\r\n", (long)Reg_GetDmaxPct()); Console_Print(b);
    return;
  }

  if (!strcmp(cmd, "cal"))
  {
    Reg_Recalibrate();
    Console_Print("OK calibrating (keep load at 0 A)...\r\n");
    return;
  }

  if (!strcmp(cmd, "stream"))
  {
    if (arg && !strcmp(arg, "on"))       { stream_on = true; stream_last = HAL_GetTick(); Console_Print("OK stream on\r\n"); }
    else if (arg && !strcmp(arg, "off")) { stream_on = false; Console_Print("OK stream off\r\n"); }
    else Console_Print("ERR stream on|off\r\n");
    return;
  }

  if (!strcmp(cmd, "rate"))
  {
    if (!arg) { Console_Print("ERR rate <ms>\r\n"); return; }
    long r = strtol(arg, NULL, 10);
    if (r < 1) r = 1;
    stream_rate = (uint32_t)r;
    snprintf(b, sizeof b, "OK rate %ld ms\r\n", r); Console_Print(b);
    return;
  }

  Console_Print("ERR unknown cmd (try help)\r\n");
}

void Cmd_Init(void)
{
  cmd_len = 0;
  stream_on = false;
  stream_rate = 100;
}

void Cmd_FeedByte(char c)
{
  if (c == '\r' || c == '\n')
  {
    Console_Print("\r\n");                          /* echo newline */
    if (cmd_len) { cmd_buf[cmd_len] = '\0'; cmd_dispatch(cmd_buf); cmd_len = 0; }
  }
  else if (c == '\b' || c == 0x7F)                  /* backspace / DEL */
  {
    if (cmd_len) { cmd_len--; Console_Print("\b \b"); }
  }
  else if ((unsigned char)c >= 0x20 && (unsigned char)c < 0x7F)
  {
    if (cmd_len < (CMD_BUF_SIZE - 1))
    {
      cmd_buf[cmd_len++] = c;
      char e[2] = { c, '\0' };
      Console_Print(e);                             /* echo typed char */
    }
  }
}

void Cmd_StreamTask(void)
{
  if (!stream_on) return;
  uint32_t now = HAL_GetTick();
  if ((now - stream_last) < stream_rate) return;
  stream_last = now;

  char line[64];
  snprintf(line, sizeof line, "DATA,%lu,%ld,%ld,%lu\r\n",
           (unsigned long)now, (long)Reg_GetSetpoint_mA(),
           (long)Reg_GetCurrent_mA(), (unsigned long)Reg_GetDutyPct());
  Console_Stream(line);
}
