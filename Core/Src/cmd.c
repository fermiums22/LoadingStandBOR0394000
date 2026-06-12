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
#define NCON             CONSOLE_NPORT  /* one independent console per UART port */

/* Per-console state: each terminal has its own line editor, history and stream,
   so USART2 (ST-Link VCP) and USART1 (Raspberry Pi) work independently. The
   underlying hardware/controller is shared - only the console I/O is split. */
typedef struct {
  char     cmd_buf[CMD_BUF_SIZE];
  uint16_t cmd_len;
  char     hist[HIST_N][CMD_BUF_SIZE];  /* newest at hist[hist_count-1] */
  int      hist_count;
  int      hist_nav;                    /* == hist_count means "fresh line" */
  uint8_t  esc_state;                   /* 0:normal 1:got ESC 2:got '[' */
  bool     stream_on;                   /* DATA,... telemetry stream   */
  bool     streamA_on;                  /* current-only monitor (Amps) */
  bool     streamM_on;                  /* torque-only monitor (Nm)    */
  uint32_t stream_rate;                 /* [ms] */
  uint32_t stream_last;
} cmd_ctx_t;

static cmd_ctx_t g_ctx[NCON];
static cmd_ctx_t *g_cur = &g_ctx[0];    /* console currently being serviced */
static bool       led_state = false;    /* shared HW state (LED/transistor)  */

static bool set_rate_hz(const char *s);

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

static void f_nm(char *b, size_t n, float nm)
{
  int neg = (nm < 0.0f);
  if (neg) nm = -nm;
  long ip = (long)nm;
  long fp = (long)((nm - (float)ip) * 1000.0f + 0.5f);
  if (fp >= 1000) { ip++; fp -= 1000; }
  snprintf(b, n, "%s%ld.%03ld Nm", neg ? "-" : " ", ip, fp);
}

static void start_m_stream(const char *hz)
{
  if (hz && !set_rate_hz(hz)) { Console_Print("ERR rate must be 1..1000 Hz\r\n"); return; }
  g_cur->stream_on = false;
  g_cur->streamA_on = false;
  g_cur->streamM_on = true;
  g_cur->stream_last = HAL_GetTick();
  Console_Print("OK streamM on (Ctrl+C/Ctrl+Z to stop)\r\n");
}

/* Set the active console's stream period from a rate in Hz (1..1000). */
static bool set_rate_hz(const char *s)
{
  long hz = strtol(s, NULL, 10);
  if (hz < 1 || hz > 1000) return false;
  uint32_t ms = (uint32_t)(1000 / hz);
  if (ms < 1) ms = 1;
  g_cur->stream_rate = ms;
  return true;
}

static void cmd_help(void)
{
  Console_Print("Commands:\r\n");
  Console_Print("  help                 - this help\r\n");
  Console_Print("  led on|off|toggle    - manual PWM 50%/0 on PA5 (LED+transistor)\r\n");
  Console_Print("  pwm <0..100>         - manual duty %, stops regulator\r\n");
  Console_Print("  set_i <mA> | set <mA>- current setpoint, starts current PI\r\n");
  Console_Print("  set_m <Nm>           - torque setpoint, starts torque+current loops\r\n");
  Console_Print("  vt on|off            - transistor output on/off (vt off = stop)\r\n");
  Console_Print("  kp [value]           - get/set proportional gain\r\n");
  Console_Print("  ki [value]           - get/set integral gain\r\n");
  Console_Print("  imax <mA>            - max current setpoint (safety)\r\n");
  Console_Print("  mmax <Nm>            - max torque setpoint (safety)\r\n");
  Console_Print("  mkp [value]          - get/set torque-loop P gain [mA/Nm]\r\n");
  Console_Print("  mki [value]          - get/set torque-loop I gain [mA/(Nm*s)]\r\n");
  Console_Print("  mkd [value]          - get/set torque-loop D damping [mA/Nm/sample]\r\n");
  Console_Print("  dmax <0..100>        - max duty clamp % (safety)\r\n");
  Console_Print("  cal                  - re-run current-sensor zero calibration\r\n");
  Console_Print("  vref [N]             - ADC self-check vs internal 1.21V ref (N samples)\r\n");
  Console_Print("  ain [N]              - measure A3=PB1 (IN9) vs known voltage, w/ VDDA\r\n");
  Console_Print("  chan pa0|pb1         - live measure channel: PA0 sensor / PB1 A3 test\r\n");
  Console_Print("  trig [cnt]           - ADC sample point: counts before ON-pulse center\r\n");
  Console_Print("  flt [raw] [mA]       - current filter windows (avg samples), e.g. flt 16 64\r\n");
  Console_Print("  cpu                  - ADC-ISR exec time / CPU load (last,max), resets max\r\n");
  Console_Print("  stream on|off [Hz]   - DATA,<tick_ms>,<set_mA>,<I_mA>,<set_Nm>,<M_Nm>,<duty%>\r\n");
  Console_Print("  streamA on|off [Hz]  - current only, e.g.  0.5000 A (this console)\r\n");
  Console_Print("  streamM on|off [Hz]  - torque only, e.g.  12.345 Nm (this console)\r\n");
  Console_Print("  M [Hz]               - print torque once, or stream torque at Hz\r\n");
  Console_Print("  rate <ms>            - stream period (alt to [Hz])\r\n");
  Console_Print("  status               - print current state\r\n");
  Console_Print("  logo                 - show the BORK banner\r\n");
  Console_Print("  reset | reboot       - restart the controller\r\n");
  Console_Print("  (Up/Down = history; Ctrl+C/Ctrl+Z = stop this console's stream)\r\n");
  Console_Print("  (the 2 consoles are independent; only one stream type at a time)\r\n");
}

static void cmd_status(void)
{
  char b[160], fp[24], fi[24], fd[24];
  out_mode_t mode = Reg_GetMode();
  const char *m = (mode == OUT_TORQUE) ? "TORQUE" : (mode == OUT_REG) ? "CURRENT" : (mode == OUT_MANUAL) ? "PWM" : "OFF";
  f3(fp, sizeof fp, Reg_GetKp());
  f3(fi, sizeof fi, Reg_GetKi());
  snprintf(b, sizeof b, "STATUS mode=%s set=%ldmA I=%ldmA setM=%ldNm M=%ldNm |M|=%ldNm duty=%lu%% cal=%d\r\n",
           m, (long)Reg_GetSetpoint_mA(), (long)Reg_GetCurrent_mA(),
           (long)Reg_GetTorqueSetpoint_Nm(), (long)Reg_GetTorque_Nm(),
           (long)Reg_GetTorqueAbs_Nm(), (unsigned long)Reg_GetDutyPct(), Reg_IsCalDone() ? 1 : 0);
  Console_Print(b);
  snprintf(b, sizeof b, "       kp=%s ki=%s imax=%ldmA mmax=%ldNm dmax=%ld%% trig=%lucnt meas=%s\r\n",
           fp, fi, (long)Reg_GetImax_mA(), (long)Reg_GetMmax_Nm(),
           (long)Reg_GetDmaxPct(), (unsigned long)Reg_GetTrigAdv(), Reg_GetMeasInOff() ? "OFF-ctr" : "ON-ctr");
  Console_Print(b);
  f3(fp, sizeof fp, Reg_GetMkP());
  f3(fi, sizeof fi, Reg_GetMkI());
  f3(fd, sizeof fd, Reg_GetMkD());
  snprintf(b, sizeof b, "       mkp=%s mki=%s mkd=%s\r\n", fp, fi, fd);
  Console_Print(b);
  char fvp[24]; f3(fvp, sizeof fvp, Reg_GetPinVoltage_mV() / 1000.0f);
  snprintf(b, sizeof b, "       Iraw=%u Vpa0=%s V flt=%u/%u\r\n",
           Reg_GetRawAvg(), fvp, Reg_GetRawWin(), Reg_GetMaWin());
  Console_Print(b);
  f3(fvp, sizeof fvp, Reg_GetTorquePinVoltage_mV() / 1000.0f);
  snprintf(b, sizeof b, "       Mraw=%u Vpb1=%s V\r\n",
           Reg_GetTorqueRawAvg(), fvp);
  Console_Print(b);
  snprintf(b, sizeof b, "       chan=%s (display only; live ADC scans PA0+PB1)\r\n",
           (Reg_GetAdcChannel() == ADC_CHANNEL_9) ? "PB1" : "PA0");
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
  while (((tok[ci] >= 'a' && tok[ci] <= 'z') || tok[ci] == '_') && ci < 15) { cmd[ci] = tok[ci]; ci++; }
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

  if (!strcmp(cmd, "set") || !strcmp(cmd, "set_i"))
  {
    if (!arg) { Console_Print("ERR set_i <mA>\r\n"); return; }
    Reg_SetSetpoint_mA(strtof(arg, NULL));
    snprintf(b, sizeof b, "OK set_i %ld mA\r\n", (long)Reg_GetSetpoint_mA()); Console_Print(b);
    return;
  }

  if (!strcmp(cmd, "set_m"))
  {
    if (!arg) { Console_Print("ERR set_m <Nm>\r\n"); return; }
    Reg_SetTorqueSetpoint_Nm(strtof(arg, NULL));
    snprintf(b, sizeof b, "OK set_m %ld Nm\r\n", (long)Reg_GetTorqueSetpoint_Nm()); Console_Print(b);
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

  if (!strcmp(cmd, "mkp"))
  {
    if (arg) Reg_SetMkP(strtof(arg, NULL));
    char f[24]; f3(f, sizeof f, Reg_GetMkP());
    snprintf(b, sizeof b, "OK mkp=%s\r\n", f); Console_Print(b);
    return;
  }

  if (!strcmp(cmd, "mki"))
  {
    if (arg) Reg_SetMkI(strtof(arg, NULL));
    char f[24]; f3(f, sizeof f, Reg_GetMkI());
    snprintf(b, sizeof b, "OK mki=%s\r\n", f); Console_Print(b);
    return;
  }

  if (!strcmp(cmd, "mkd"))
  {
    if (arg) Reg_SetMkD(strtof(arg, NULL));
    char f[24]; f3(f, sizeof f, Reg_GetMkD());
    snprintf(b, sizeof b, "OK mkd=%s\r\n", f); Console_Print(b);
    return;
  }

  if (!strcmp(cmd, "imax"))
  {
    if (arg) Reg_SetImax_mA(strtof(arg, NULL));
    snprintf(b, sizeof b, "OK imax=%ld mA\r\n", (long)Reg_GetImax_mA()); Console_Print(b);
    return;
  }

  if (!strcmp(cmd, "mmax"))
  {
    if (arg) Reg_SetMmax_Nm(strtof(arg, NULL));
    snprintf(b, sizeof b, "OK mmax=%ld Nm\r\n", (long)Reg_GetMmax_Nm()); Console_Print(b);
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

  if (!strcmp(cmd, "trig"))
  {
    if (arg) Reg_SetTrigAdv((uint32_t)strtol(arg, NULL, 10));
    snprintf(b, sizeof b, "OK trig=%lu cnt (~%lu ns before center)\r\n",
             (unsigned long)Reg_GetTrigAdv(),
             (unsigned long)(Reg_GetTrigAdv() * 1000u / 64u));
    Console_Print(b);
    return;
  }

  if (!strcmp(cmd, "cpu"))
  {
    /* ADC-ISR load: HCLK=64MHz, ISR rate 10kHz -> period 6400 cyc; us=cyc/64, %=cyc/64. */
    uint32_t last = Reg_GetIsrLastCyc(), mx = Reg_GetIsrMaxCyc();
    snprintf(b, sizeof b, "CPU isr last=%lu.%lu us  max=%lu.%lu us  (%lu%% peak)\r\n",
             (unsigned long)(last / 64u), (unsigned long)((last % 64u) * 10u / 64u),
             (unsigned long)(mx / 64u),   (unsigned long)((mx % 64u) * 10u / 64u),
             (unsigned long)(mx / 64u));
    Console_Print(b);
    Reg_ResetIsrMax();
    return;
  }

  if (!strcmp(cmd, "flt"))
  {
    if (arg)
    {
      long rw = strtol(arg, NULL, 10);
      char *p2 = strtok(NULL, " \t");
      long mw = p2 ? strtol(p2, NULL, 10) : (long)Reg_GetMaWin();
      Reg_SetFilter((uint8_t)rw, (uint8_t)mw);
    }
    unsigned lag = ((unsigned)Reg_GetRawWin() - 1u + (unsigned)Reg_GetMaWin() - 1u) * 100u / 2u; /* us */
    snprintf(b, sizeof b, "OK flt raw=%u mA=%u samples (lag ~%u.%u ms)\r\n",
             Reg_GetRawWin(), Reg_GetMaWin(), lag / 1000u, (lag % 1000u) / 100u);
    Console_Print(b);
    return;
  }

  if (!strcmp(cmd, "chan"))
  {
    if (arg && !strcmp(arg, "pa0"))      { Reg_SetAdcChannel(ADC_CHANNEL_0); Console_Print("OK chan PA0 (current sensor)\r\n"); }
    else if (arg && !strcmp(arg, "pb1")) { Reg_SetAdcChannel(ADC_CHANNEL_9); Console_Print("OK chan PB1 (A3 bench test)\r\n"); }
    else { snprintf(b, sizeof b, "chan=%s\r\n", (Reg_GetAdcChannel() == ADC_CHANNEL_9) ? "PB1" : "PA0"); Console_Print(b); }
    return;
  }

  if (!strcmp(cmd, "ain"))
  {
    /* Absolute voltage check on A3=PB1 (ADC1_IN9) vs a known reference.
       (named 'ain', not 'a3' - the parser splits a digit off the command word) */
    uint16_t n = 256;
    if (arg) { long a = strtol(arg, NULL, 10); if (a >= 1 && a <= 4096) n = (uint16_t)a; }
    vref_diag_t d;
    Reg_A3Diag(&d, n);
    char line[96], fv[24], fd[24];
    f3(fv, sizeof fv, d.vref_mV / 1000.0f);
    f3(fd, sizeof fd, d.vdda_mV / 1000.0f);
    snprintf(line, sizeof line, "A3(PB1) n=%u raw avg=%ld min=%u max=%u spread=%u\r\n",
             d.n, (long)(d.raw_avg + 0.5f), d.raw_min, d.raw_max,
             (unsigned)(d.raw_max - d.raw_min));
    Console_Print(line);
    snprintf(line, sizeof line, "     V_A3=%s V (%ld mV)  VDDA=%s V  calfact=%lu\r\n",
             fv, (long)(d.vref_mV + 0.5f), fd, (unsigned long)Reg_GetCalFact());
    Console_Print(line);
    return;
  }

  if (!strcmp(cmd, "vref"))
  {
    /* ADC sanity check against the internal 1.21 V reference. */
    uint16_t n = 256;
    if (arg) { long a = strtol(arg, NULL, 10); if (a >= 1 && a <= 4096) n = (uint16_t)a; }
    vref_diag_t d;
    Reg_VrefDiag(&d, n);
    char fv[24], fd[24], line[96];
    f3(fv, sizeof fv, d.vref_mV / 1000.0f);
    f3(fd, sizeof fd, d.vdda_mV / 1000.0f);
    snprintf(line, sizeof line, "VREF n=%u raw avg=%ld min=%u max=%u spread=%u\r\n",
             d.n, (long)(d.raw_avg + 0.5f), d.raw_min, d.raw_max,
             (unsigned)(d.raw_max - d.raw_min));
    Console_Print(line);
    snprintf(line, sizeof line, "     Vref=%s V  VDDA=%s V\r\n", fv, fd);
    Console_Print(line);
    return;
  }

  if (!strcmp(cmd, "stream"))
  {
    if (arg && (!strcmp(arg, "m") || !strcmp(arg, "moment") || !strcmp(arg, "torque")))
    {
      char *op = strtok(NULL, " \t");
      if (!op || !strcmp(op, "on"))
      {
        start_m_stream(strtok(NULL, " \t"));
      }
      else if (!strcmp(op, "off"))
      {
        g_cur->streamM_on = false;
        Console_Print("OK streamM off\r\n");
      }
      else
      {
        start_m_stream(op);
      }
    }
    else if (arg && !strcmp(arg, "on"))
    {
      char *hz = strtok(NULL, " \t");
      if (hz && !set_rate_hz(hz)) { Console_Print("ERR rate must be 1..1000 Hz\r\n"); return; }
      g_cur->streamA_on = false;            /* only one stream type per console */
      g_cur->streamM_on = false;
      g_cur->stream_on = true; g_cur->stream_last = HAL_GetTick();
      Console_Print("OK stream on (Ctrl+C/Ctrl+Z to stop)\r\n");
    }
    else if (arg && !strcmp(arg, "off")) { g_cur->stream_on = false; Console_Print("OK stream off\r\n"); }
    else Console_Print("ERR stream on|off [Hz]\r\n");
    return;
  }

  if (!strcmp(cmd, "streama"))   /* command line is lower-cased; user types streamA */
  {
    if (arg && !strcmp(arg, "on"))
    {
      char *hz = strtok(NULL, " \t");
      if (hz && !set_rate_hz(hz)) { Console_Print("ERR rate must be 1..1000 Hz\r\n"); return; }
      g_cur->stream_on = false;             /* only one stream type per console */
      g_cur->streamM_on = false;
      g_cur->streamA_on = true; g_cur->stream_last = HAL_GetTick();
      Console_Print("OK streamA on (Ctrl+C/Ctrl+Z to stop)\r\n");
    }
    else if (arg && !strcmp(arg, "off")) { g_cur->streamA_on = false; Console_Print("OK streamA off\r\n"); }
    else Console_Print("ERR streamA on|off [Hz]\r\n");
    return;
  }

  if (!strcmp(cmd, "m") || !strcmp(cmd, "streamm") || !strcmp(cmd, "stream_m") ||
      !strcmp(cmd, "moment") || !strcmp(cmd, "torque"))
  {
    if (!arg)
    {
      char mtxt[24];
      f_nm(mtxt, sizeof mtxt, Reg_GetTorque_Nm());
      snprintf(b, sizeof b, "%s\r\n", mtxt);
      Console_Print(b);
    }
    else if (!strcmp(arg, "on"))
    {
      start_m_stream(strtok(NULL, " \t"));
    }
    else if (!strcmp(arg, "off")) { g_cur->streamM_on = false; Console_Print("OK streamM off\r\n"); }
    else { start_m_stream(arg); }
    return;
  }

  if (!strcmp(cmd, "rate"))
  {
    if (!arg) { Console_Print("ERR rate <ms>\r\n"); return; }
    long r = strtol(arg, NULL, 10);
    if (r < 1) r = 1;
    g_cur->stream_rate = (uint32_t)r;
    snprintf(b, sizeof b, "OK rate %ld ms\r\n", r); Console_Print(b);
    return;
  }

  Console_Print("ERR unknown cmd (try help)\r\n");
}

void Cmd_Init(void)
{
  for (int i = 0; i < NCON; i++)
  {
    g_ctx[i].cmd_len     = 0;
    g_ctx[i].stream_on   = false;
    g_ctx[i].streamA_on  = false;
    g_ctx[i].streamM_on  = false;
    g_ctx[i].stream_rate = 100;
    g_ctx[i].stream_last = 0;
    g_ctx[i].hist_count  = 0;
    g_ctx[i].hist_nav    = 0;
    g_ctx[i].esc_state   = 0;
  }
  g_cur = &g_ctx[0];
}

/* Redraw the edit line: CR, erase to end of line, reprint the buffer. */
static void line_redraw(void)
{
  g_cur->cmd_buf[g_cur->cmd_len] = '\0';
  Console_Print("\r\033[K");
  if (g_cur->cmd_len) Console_Print(g_cur->cmd_buf);
}

static void hist_store(const char *s)
{
  cmd_ctx_t *cx = g_cur;
  if (s[0] == '\0') return;
  if (cx->hist_count > 0 && !strcmp(cx->hist[cx->hist_count - 1], s)) return;  /* skip dup */
  if (cx->hist_count < (int)HIST_N)
  {
    strncpy(cx->hist[cx->hist_count], s, CMD_BUF_SIZE - 1);
    cx->hist[cx->hist_count][CMD_BUF_SIZE - 1] = '\0';
    cx->hist_count++;
  }
  else
  {
    for (int k = 1; k < (int)HIST_N; k++) strcpy(cx->hist[k - 1], cx->hist[k]);
    strncpy(cx->hist[HIST_N - 1], s, CMD_BUF_SIZE - 1);
    cx->hist[HIST_N - 1][CMD_BUF_SIZE - 1] = '\0';
  }
}

static void hist_recall(int idx)
{
  strcpy(g_cur->cmd_buf, g_cur->hist[idx]);
  g_cur->cmd_len = (uint16_t)strlen(g_cur->cmd_buf);
  line_redraw();
}

/* Process one received byte for the active console (g_cur already set). */
static void feed_one(char c)
{
  cmd_ctx_t *cx = g_cur;

  /* Ctrl+C / Ctrl+Z: stop this console's stream (lets one terminal stream while
     the other shows status). Also clears any half-typed line. */
  if (c == 0x03 || c == 0x1A)
  {
    if (cx->stream_on || cx->streamA_on || cx->streamM_on)
    {
      cx->stream_on = false; cx->streamA_on = false; cx->streamM_on = false;
      Console_Print("\r\n[stream stopped]\r\n");
    }
    cx->cmd_len = 0; cx->esc_state = 0;
    return;
  }

  /* arrow keys arrive as ESC '[' 'A'/'B'/'C'/'D' */
  if (cx->esc_state == 1) { cx->esc_state = (c == '[') ? 2 : 0; return; }
  if (cx->esc_state == 2)
  {
    if (c == 'A') { if (cx->hist_nav > 0)          { cx->hist_nav--; hist_recall(cx->hist_nav); } }   /* up */
    else if (c == 'B') { if (cx->hist_nav < cx->hist_count) { cx->hist_nav++;
                          if (cx->hist_nav == cx->hist_count) { cx->cmd_len = 0; line_redraw(); }
                          else hist_recall(cx->hist_nav); } }                                          /* down */
    cx->esc_state = 0;
    return;
  }
  if (c == 0x1B) { cx->esc_state = 1; return; }

  if (c == '\r' || c == '\n')
  {
    Console_Print("\r\n");                          /* echo newline */
    if (cx->cmd_len)
    {
      cx->cmd_buf[cx->cmd_len] = '\0';
      hist_store(cx->cmd_buf);                       /* store before dispatch tokenizes it */
      cmd_dispatch(cx->cmd_buf);
      cx->cmd_len = 0;
    }
    cx->hist_nav = cx->hist_count;
  }
  else if (c == '\b' || c == 0x7F)                  /* backspace / DEL */
  {
    if (cx->cmd_len) { cx->cmd_len--; Console_Print("\b \b"); }
  }
  else if ((unsigned char)c >= 0x20)                /* printable ASCII or UTF-8 (Cyrillic) */
  {
    if (cx->cmd_len < (CMD_BUF_SIZE - 1))
    {
      cx->cmd_buf[cx->cmd_len++] = c;
      char e[2] = { c, '\0' };
      Console_Print(e);                             /* echo typed byte */
    }
  }
}

/* Feed a byte tagged with its source console; output is routed back to it only. */
void Cmd_FeedByte(int port, char c)
{
  if (port < 0 || port >= NCON) return;
  g_cur = &g_ctx[port];
  Console_Route(port);
  feed_one(c);
  Console_Route(CONSOLE_BOTH);                      /* async/global prints broadcast */
}

void Cmd_StreamTask(void)
{
  uint32_t now = HAL_GetTick();
  char line[64];
  for (int p = 0; p < NCON; p++)
  {
    cmd_ctx_t *cx = &g_ctx[p];
    if (!cx->stream_on && !cx->streamA_on && !cx->streamM_on) continue;
    if ((now - cx->stream_last) < cx->stream_rate) continue;
    cx->stream_last = now;

    Console_Route(p);                               /* stream only to its own port */
    if (cx->stream_on)
    {
      snprintf(line, sizeof line, "DATA,%lu,%ld,%ld,%ld,%ld,%lu\r\n",
               (unsigned long)now, (long)Reg_GetSetpoint_mA(),
               (long)Reg_GetCurrent_mA(), (long)Reg_GetTorqueSetpoint_Nm(),
               (long)Reg_GetTorque_Nm(), (unsigned long)Reg_GetDutyPct());
      Console_Stream(line);
    }
    if (cx->streamA_on)
    {
      char a[24];
      f_amps(a, sizeof a, Reg_GetCurrent_mA());
      snprintf(line, sizeof line, "%s\r\n", a);
      Console_Stream(line);
    }
    if (cx->streamM_on)
    {
      char m[24];
      f_nm(m, sizeof m, Reg_GetTorque_Nm());
      snprintf(line, sizeof line, "%s\r\n", m);
      Console_Stream(line);
    }
  }
  Console_Route(CONSOLE_BOTH);
}
