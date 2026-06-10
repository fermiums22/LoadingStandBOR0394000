/**
  ******************************************************************************
  * @file    control.h
  * @brief   Current-sensor reading (ACS724) + PI current regulator.
  *
  * The regulator drives TIM2_CH1 (PA5 = transistor gate, in parallel with the
  * on-board LD4) at 10 kHz center-aligned PWM. The current feedback is taken
  * from ADC1_IN0 (PA0), sampled at the center of the control pulse via the
  * TIM2 update trigger. Sensor scaling / zero-calibration is reused from the
  * meatgrinder project (ACS724, 166 mV/A, 2.5 V zero, /2 divider).
  ******************************************************************************
  */
#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include <stdbool.h>

/* TIM2 ARR -> Fpwm = 64MHz / (2*PWM_ARR) = 10 kHz (center-aligned). */
#define PWM_ARR              (3200u)

typedef enum { OUT_OFF = 0, OUT_MANUAL, OUT_REG } out_mode_t;

/* Init: ADC self-calibration, start PWM (0% duty) and timer-triggered ADC. */
void       Reg_Init(void);

/* Setters (called from the command parser) ---------------------------------*/
void       Reg_SetMode(out_mode_t m);
void       Reg_SetSetpoint_mA(float v);   /* clamps to imax, switches to OUT_REG  */
void       Reg_SetManualPct(float pct);   /* sets manual duty, switches to MANUAL  */
void       Reg_SetKp(float v);
void       Reg_SetKi(float v);
void       Reg_SetImax_mA(float v);
void       Reg_SetDmaxPct(float v);
void       Reg_Recalibrate(void);

/* Getters (telemetry / status) ---------------------------------------------*/
out_mode_t Reg_GetMode(void);
float      Reg_GetCurrent_mA(void);
float      Reg_GetSetpoint_mA(void);
uint32_t   Reg_GetDutyPct(void);
float      Reg_GetKp(void);
float      Reg_GetKi(void);
float      Reg_GetImax_mA(void);
float      Reg_GetDmaxPct(void);
bool       Reg_IsCalDone(void);

#endif /* CONTROL_H */
