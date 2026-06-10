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
#define HIST_N           (8u)           /* command history depth */

static char     cmd_buf[CMD_BUF_SIZE];
static uint16_t cmd_len = 0;
static bool     led_state = false;

/* command history (newest at hist[hist_count-1]) + line-editor state */
static char     hist[HIST_N][CMD_BUF_SIZE];
static int      hist_count = 0;
static int      hist_nav   = 0;         /* == hist_count means "fresh line" */
static uint8_t  esc_state  = 0;         /* 0:normal 1:got ESC 2:got '[' */

static bool     stream_on   = false;    /* DATA,... telemetry stream     */
static bool     streamA_on  = false;    /* current-only monitor (Amps)   */
static uint32_t stream_rate = 100;      /* [ms], shared by both streams  */
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

/* Fixed-width current in Amps: " 0.5000 A" / "-0.5000 A" (sign slot + 4 dec,
   so columns/zeros/sign never jump). Input is milliamps. */
static void f_amps(char *b, size_t n, float mA)
{
  long u = (long)(mA * 10.0f + (mA >= 0.0f ? 0.5f : -0.5f));  /* 0.1 mA units */
  char sign = (u < 0) ? '-' : ' ';
  if (u < 0) u = -u;
  snprintf(b, n, "%c%ld.%04ld A", sign, u / 10000, u % 10000);
}

/* Set the shared stream period from a rate in Hz (1..1000). false if invalid. */
static bool set_rate_hz(const char *s)
{
  long hz = strtol(s, NULL, 10);
  if (hz < 1 || hz > 1000) return false;
  uint32_t ms = (uint32_t)(1000 / hz);
  if (ms < 1) ms = 1;
  stream_rate = ms;
  return true;
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
  Console_Print("  kd [value]           - get/set derivative gain\r\n");
  Console_Print("  imax <mA>            - max current setpoint (safety)\r\n");
  Console_Print("  dmax <0..100>        - max duty clamp % (safety)\r\n");
  Console_Print("  cal                  - re-run current-sensor zero calibration\r\n");
  Console_Print("  stream on|off [Hz]   - DATA,<tick_ms>,<set_mA>,<I_mA>,<duty%>\r\n");
  Console_Print("  streamA on|off [Hz]  - current only, e.g.  0.5000 A\r\n");
  Console_Print("  rate <ms>            - stream period (alt to [Hz])\r\n");
  Console_Print("  status               - print current state\r\n");
  Console_Print("  logo                 - show the BORK banner\r\n");
  Console_Print("  reset | reboot       - restart the controller\r\n");
  Console_Print("  (Up/Down arrows recall command history)\r\n");
}

static void cmd_status(void)
{
  char b[160], fp[24], fi[24], fd[24];
  out_mode_t mode = Reg_GetMode();
  const char *m = (mode == OUT_REG) ? "VT" : (mode == OUT_MANUAL) ? "MANUAL" : "OFF";
  f3(fp, sizeof fp, Reg_GetKp());
  f3(fi, sizeof fi, Reg_GetKi());
  f3(fd, sizeof fd, Reg_GetKd());
  snprintf(b, sizeof b, "STATUS mode=%s set=%ldmA I=%ldmA duty=%lu%% cal=%d\r\n",
           m, (long)Reg_GetSetpoint_mA(), (long)Reg_GetCurrent_mA(),
           (unsigned long)Reg_GetDutyPct(), Reg_IsCalDone() ? 1 : 0);
  Console_Print(b);
  snprintf(b, sizeof b, "       kp=%s ki=%s kd=%s imax=%ldmA dmax=%ld%%\r\n",
           fp, fi, fd, (long)Reg_GetImax_mA(), (long)Reg_GetDmaxPct());
  Console_Print(b);
}

static void cmd_dispatch(char *line)
{
  /* case-insensitive: lower-case the whole line in place */
  for (char *p = line; *p; ++p)
    if (*p >= 'A' && *p <= 'Z') *p += 32;

  char *tok = strtok(line, " \t");
  if (!tok) return;

  /* split a leading alphabetic command from a glued argument:
     "pwm10" -> cmd="pwm", arg="10"; "led on" -> cmd="led", arg="on" */
  char cmd[16];
  int ci = 0;
  while (tok[ci] >= 'a' && tok[ci] <= 'z' && ci < 15) { cmd[ci] = tok[ci]; ci++; }
  cmd[ci] = '\0';
  char *arg = (tok[ci] != '\0') ? &tok[ci] : strtok(NULL, " \t");
  char b[64];

  if (!strcmp(cmd, "help"))   { cmd_help();    return; }
  if (!strcmp(cmd, "logo"))   { Cmd_Banner();  return; }
  if (!strcmp(cmd, "status")) { cmd_status();  return; }

  if (!strcmp(cmd, "reset") || !strcmp(cmd, "reboot"))
  {
    Console_Print("OK reboot\r\n");
    HAL_Delay(20);            /* let the TX ring flush before the reset */
    NVIC_SystemReset();
  }

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

  if (!strcmp(cmd, "kd"))
  {
    if (arg) Reg_SetKd(strtof(arg, NULL));
    char f[24]; f3(f, sizeof f, Reg_GetKd());
    snprintf(b, sizeof b, "OK kd=%s\r\n", f); Console_Print(b);
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
    if (arg && !strcmp(arg, "on"))
    {
      char *hz = strtok(NULL, " \t");
      if (hz && !set_rate_hz(hz)) { Console_Print("ERR rate must be 1..1000 Hz\r\n"); return; }
      stream_on = true; stream_last = HAL_GetTick();
      Console_Print("OK stream on\r\n");
    }
    else if (arg && !strcmp(arg, "off")) { stream_on = false; Console_Print("OK stream off\r\n"); }
    else Console_Print("ERR stream on|off [Hz]\r\n");
    return;
  }

  if (!strcmp(cmd, "streama"))   /* command line is lower-cased; user types streamA */
  {
    if (arg && !strcmp(arg, "on"))
    {
      char *hz = strtok(NULL, " \t");
      if (hz && !set_rate_hz(hz)) { Console_Print("ERR rate must be 1..1000 Hz\r\n"); return; }
      streamA_on = true; stream_last = HAL_GetTick();
      Console_Print("OK streamA on\r\n");
    }
    else if (arg && !strcmp(arg, "off")) { streamA_on = false; Console_Print("OK streamA off\r\n"); }
    else Console_Print("ERR streamA on|off [Hz]\r\n");
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
  streamA_on = false;
  stream_rate = 100;
  hist_count = 0;
  hist_nav = 0;
  esc_state = 0;
}

/* Redraw the edit line: CR, erase to end of line, reprint the buffer. */
static void line_redraw(void)
{
  cmd_buf[cmd_len] = '\0';
  Console_Print("\r\033[K");
  if (cmd_len) Console_Print(cmd_buf);
}

static void hist_store(const char *s)
{
  if (s[0] == '\0') return;
  if (hist_count > 0 && !strcmp(hist[hist_count - 1], s)) return;  /* skip duplicate */
  if (hist_count < (int)HIST_N)
  {
    strncpy(hist[hist_count], s, CMD_BUF_SIZE - 1);
    hist[hist_count][CMD_BUF_SIZE - 1] = '\0';
    hist_count++;
  }
  else
  {
    for (int k = 1; k < (int)HIST_N; k++) strcpy(hist[k - 1], hist[k]);
    strncpy(hist[HIST_N - 1], s, CMD_BUF_SIZE - 1);
    hist[HIST_N - 1][CMD_BUF_SIZE - 1] = '\0';
  }
}

static void hist_recall(int idx)
{
  strcpy(cmd_buf, hist[idx]);
  cmd_len = (uint16_t)strlen(cmd_buf);
  line_redraw();
}

void Cmd_FeedByte(char c)
{
  /* arrow keys arrive as ESC '[' 'A'/'B'/'C'/'D' */
  if (esc_state == 1) { esc_state = (c == '[') ? 2 : 0; return; }
  if (esc_state == 2)
  {
    if (c == 'A') { if (hist_nav > 0)          { hist_nav--; hist_recall(hist_nav); } }      /* up   */
    else if (c == 'B') { if (hist_nav < hist_count) { hist_nav++;
                          if (hist_nav == hist_count) { cmd_len = 0; line_redraw(); }
                          else hist_recall(hist_nav); } }                                     /* down */
    /* 'C'/'D' (left/right) ignored */
    esc_state = 0;
    return;
  }
  if (c == 0x1B) { esc_state = 1; return; }

  if (c == '\r' || c == '\n')
  {
    Console_Print("\r\n");                          /* echo newline */
    if (cmd_len)
    {
      cmd_buf[cmd_len] = '\0';
      hist_store(cmd_buf);                          /* store before dispatch tokenizes it */
      cmd_dispatch(cmd_buf);
      cmd_len = 0;
    }
    hist_nav = hist_count;
  }
  else if (c == '\b' || c == 0x7F)                  /* backspace / DEL */
  {
    if (cmd_len) { cmd_len--; Console_Print("\b \b"); }
  }
  else if ((unsigned char)c >= 0x20)                /* printable ASCII or UTF-8 (Cyrillic) */
  {
    if (cmd_len < (CMD_BUF_SIZE - 1))
    {
      cmd_buf[cmd_len++] = c;
      char e[2] = { c, '\0' };
      Console_Print(e);                             /* echo typed byte */
    }
  }
}

void Cmd_StreamTask(void)
{
  if (!stream_on && !streamA_on) return;
  uint32_t now = HAL_GetTick();
  if ((now - stream_last) < stream_rate) return;
  stream_last = now;

  char line[64];
  if (stream_on)
  {
    snprintf(line, sizeof line, "DATA,%lu,%ld,%ld,%lu\r\n",
             (unsigned long)now, (long)Reg_GetSetpoint_mA(),
             (long)Reg_GetCurrent_mA(), (unsigned long)Reg_GetDutyPct());
    Console_Stream(line);
  }
  if (streamA_on)
  {
    char a[24];
    f_amps(a, sizeof a, Reg_GetCurrent_mA());
    snprintf(line, sizeof line, "%s\r\n", a);
    Console_Stream(line);
  }
}
