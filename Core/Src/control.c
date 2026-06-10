/**
  ******************************************************************************
  * @file    control.c
  * @brief   ACS724 current measurement + PI current regulator.
  ******************************************************************************
  */
#include "main.h"
#include "control.h"
#include <math.h>

/* Peripherals owned by the regulator (defined in main.c). */
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;

/* ---- control timing -------------------------------------------------------*/
#define CTRL_FS_HZ           (10000.0f)            /* PI runs once per PWM period          */
#define CTRL_TS              (1.0f / CTRL_FS_HZ)   /* control sample period [s]            */

/* ---- ACS724 current sensor (taken from meatgrinder project) ---------------*/
#define ADC_VREF_MV          (3300.0f)
#define ADC_FULL_SCALE       (4096.0f)             /* 12-bit                               */
#define VOLTS_LSB            (ADC_VREF_MV / ADC_FULL_SCALE)  /* [mV/LSB]                    */
#define ADC_RES_DIV          (2.0f)                /* sensor output divided /2 before ADC  */
#define DT_OFFSET_ZERO_MV    (2500.0f)             /* [mV] sensor output @ 0 A (5V/2)      */
#define VOLT_2_CURRENT       (6.0240963855f)       /* [mA/mV] = 1000mA / 166mV (ACS724)    */
#define CURRENT_CAL_SAMPLES  (64u)                 /* samples averaged for sensor zero     */
#define CURRENT_LPF_K        (0.2f)                /* current low-pass filter coefficient  */

/* ---- PI / safety defaults (tunable over UART) -----------------------------*/
#define DEFAULT_KP           (0.20f)               /* [PWM counts / mA]                    */
#define DEFAULT_KI           (20.0f)               /* [PWM counts / (mA*s)]                */
#define DEFAULT_DMAX_PCT     (95.0f)               /* max duty clamp [%]  (safety)         */
#define DEFAULT_IMAX_MA      (5000.0f)             /* max current setpoint clamp [mA]      */

/* ---- state (ADC ISR is the only writer of *_meas/integ; CLI writes params) */
static volatile out_mode_t g_mode        = OUT_OFF;
static volatile uint32_t   g_manual_ccr  = 0;
static volatile float      g_setpoint_mA = 0.0f;
static volatile float      g_kp          = DEFAULT_KP;
static volatile float      g_ki          = DEFAULT_KI;
static volatile float      g_integ       = 0.0f;
static volatile float      g_dmax_pct    = DEFAULT_DMAX_PCT;
static volatile float      g_imax_mA     = DEFAULT_IMAX_MA;

static volatile float      g_I_mA        = 0.0f;   /* filtered measured current [mA]       */
static volatile float      g_zero_mA     = 0.0f;   /* sensor zero offset [mA]              */
static volatile uint32_t   g_ccr         = 0;      /* duty currently applied [counts]      */
static volatile float      g_cal_sum     = 0.0f;
static volatile uint16_t   g_cal_cnt     = 0;
static volatile bool       g_cal_done    = false;

/* ADC DMA target: one sample, circular. Written by DMA, read in ADC ISR. */
static uint16_t adc_dma[1];

/* Clamp duty to the configured safety maximum and drive both PWM outputs:
   CH1 = PA5 (transistor gate + on-board LD4), CH2 = PA1 (scope/mirror). */
static void apply_output(uint32_t ccr)
{
  uint32_t maxc = (uint32_t)(g_dmax_pct * 0.01f * (float)PWM_ARR);
  if (ccr > maxc) ccr = maxc;
  TIM2->CCR1 = ccr;            /* PA5 = transistor gate + LD4 */
  TIM2->CCR2 = ccr;            /* PA1 = mirror for the scope   */
  g_ccr = ccr;
}

void Reg_Init(void)
{
  /* ADC self-calibration is mandatory on STM32G0 before conversions. */
  HAL_ADCEx_Calibration_Start(&hadc1);

  g_kp = DEFAULT_KP; g_ki = DEFAULT_KI;
  g_dmax_pct = DEFAULT_DMAX_PCT; g_imax_mA = DEFAULT_IMAX_MA;
  g_mode = OUT_OFF; g_manual_ccr = 0; g_setpoint_mA = 0.0f; g_integ = 0.0f;
  g_cal_done = false; g_cal_cnt = 0; g_cal_sum = 0.0f; g_I_mA = 0.0f;

  TIM2->CCR1 = 0;
  TIM2->CCR2 = 0;
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);             /* 10 kHz PWM, PA5 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);             /* 10 kHz PWM, PA1 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma, 1);    /* timer-triggered ADC */
}

/*
 * ADC conversion complete (DMA1_Channel2_3 IRQ, prio 1).
 * Triggered by TIM2 update -> fires at the underflow (center of the ON pulse)
 * and the overflow (center of the OFF pulse). We keep only the ON-pulse-center
 * sample: right after the underflow the timer counts UP, so DIR == 0. This
 * yields a clean 10 kHz feedback aligned to the middle of the control pulse.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance != ADC1) return;
  if ((TIM2->CR1 & TIM_CR1_DIR) != 0u) return;          /* counting down -> skip */

  /* ACS724 current [mA] (signed about the zero point). */
  float v_mV     = ((float)adc_dma[0] * VOLTS_LSB) * ADC_RES_DIV;
  float i_signed = (v_mV - DT_OFFSET_ZERO_MV) * VOLT_2_CURRENT;

  if (!g_cal_done)
  {
    g_cal_sum += i_signed;
    if (++g_cal_cnt >= CURRENT_CAL_SAMPLES)
    {
      g_zero_mA  = g_cal_sum / (float)CURRENT_CAL_SAMPLES;
      g_cal_done = true;
    }
    g_I_mA = 0.0f;
    apply_output(0);                                    /* force OFF while calibrating */
    return;
  }

  float i = fabsf(i_signed - g_zero_mA);
  g_I_mA = g_I_mA * (1.0f - CURRENT_LPF_K) + i * CURRENT_LPF_K;

  uint32_t ccr;
  switch (g_mode)
  {
    case OUT_MANUAL:
      ccr = g_manual_ccr;
      break;

    case OUT_REG:
    {
      /* PI current regulator with back-calculation anti-windup. */
      float err    = g_setpoint_mA - g_I_mA;
      float p      = g_kp * err;
      float outmax = g_dmax_pct * 0.01f * (float)PWM_ARR;
      float out;
      if (g_ki > 0.0f)
      {
        g_integ += err * CTRL_TS;
        out = p + g_ki * g_integ;
        if (out > outmax)    { out = outmax; g_integ = (outmax - p) / g_ki; }
        else if (out < 0.0f) { out = 0.0f;   g_integ = (0.0f   - p) / g_ki; }
      }
      else
      {
        g_integ = 0.0f;
        out = p;
        if (out > outmax) out = outmax; else if (out < 0.0f) out = 0.0f;
      }
      ccr = (uint32_t)out;
      break;
    }

    case OUT_OFF:
    default:
      ccr = 0;
      g_integ = 0.0f;
      break;
  }
  apply_output(ccr);
}

/* ---- setters --------------------------------------------------------------*/
void Reg_SetMode(out_mode_t m)
{
  if (m == OUT_OFF) g_integ = 0.0f;
  g_mode = m;
}

void Reg_SetSetpoint_mA(float v)
{
  if (v < 0.0f) v = 0.0f;
  if (v > g_imax_mA) v = g_imax_mA;
  g_setpoint_mA = v;
  g_mode = OUT_REG;
}

void Reg_SetManualPct(float pct)
{
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  g_manual_ccr = (uint32_t)(pct * 0.01f * (float)PWM_ARR);
  g_mode = OUT_MANUAL;
}

void Reg_SetKp(float v)       { g_kp = v; }
void Reg_SetKi(float v)       { g_ki = v; g_integ = 0.0f; }
void Reg_SetImax_mA(float v)  { if (v < 0.0f) v = 0.0f; g_imax_mA = v; }
void Reg_SetDmaxPct(float v)  { if (v < 0.0f) v = 0.0f; if (v > 100.0f) v = 100.0f; g_dmax_pct = v; }

void Reg_Recalibrate(void)
{
  g_mode = OUT_OFF;
  g_cal_done = false; g_cal_cnt = 0; g_cal_sum = 0.0f; g_I_mA = 0.0f;
}

/* ---- getters --------------------------------------------------------------*/
out_mode_t Reg_GetMode(void)        { return g_mode; }
float      Reg_GetCurrent_mA(void)  { return g_I_mA; }
float      Reg_GetSetpoint_mA(void) { return g_setpoint_mA; }
uint32_t   Reg_GetDutyPct(void)     { return (g_ccr * 100u) / PWM_ARR; }
float      Reg_GetKp(void)          { return g_kp; }
float      Reg_GetKi(void)          { return g_ki; }
float      Reg_GetImax_mA(void)     { return g_imax_mA; }
float      Reg_GetDmaxPct(void)     { return g_dmax_pct; }
bool       Reg_IsCalDone(void)      { return g_cal_done; }
