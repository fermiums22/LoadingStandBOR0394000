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

/* Result of the internal-reference ADC sanity check (Reg_VrefDiag). */
typedef struct {
  uint16_t n;          /* samples captured                                   */
  uint16_t raw_min;    /* min raw ADC code over the burst                    */
  uint16_t raw_max;    /* max raw ADC code over the burst                    */
  float    raw_avg;    /* mean raw ADC code                                  */
  float    vref_mV;    /* measured VREFINT voltage [mV] (~1212 mV nominal)   */
  float    vdda_mV;    /* supply VDDA back-computed from the factory cal [mV]*/
} vref_diag_t;

/* Init: ADC self-calibration, start PWM (0% duty) and timer-triggered ADC. */
void       Reg_Init(void);

/* Diagnostic: temporarily retarget the ADC to the internal VREFINT bandgap
   with a long (proper) sampling time, capture a polled burst, report
   min/max/avg + computed VDDA, then restore the ACS724 timer-triggered DMA.
   Lets you confirm the ADC core/clock are stable independent of the sensor. */
void       Reg_VrefDiag(vref_diag_t *out, uint16_t nsamples);

/* Absolute check against a known voltage on A3 = PB1 = ADC1_IN9: out->vref_mV is
   the measured PB1 voltage, out->vdda_mV the VDDA recovered from VREFINT. */
void       Reg_A3Diag(vref_diag_t *out, uint16_t nsamples);
uint32_t   Reg_GetCalFact(void);          /* ADC calibration factor (0 => not calibrated)  */

/* Live measurement channel select (PA0 current sensor / PB1 A3 bench test) + readout. */
void       Reg_SetAdcChannel(uint32_t chan);  /* pass ADC_CHANNEL_0 (PA0) or ADC_CHANNEL_9 (PB1) */
uint32_t   Reg_GetAdcChannel(void);
uint16_t   Reg_GetRawAvg(void);           /* moving-average raw ADC value                   */
float      Reg_GetPinVoltage_mV(void);    /* voltage at the ADC pin (raw_avg * VDDA/4096)   */

/* Setters (called from the command parser) ---------------------------------*/
void       Reg_SetMode(out_mode_t m);
void       Reg_SetSetpoint_mA(float v);   /* clamps to imax, switches to OUT_REG  */
void       Reg_SetManualPct(float pct);   /* sets manual duty, switches to MANUAL  */
void       Reg_SetKp(float v);
void       Reg_SetKi(float v);
void       Reg_SetKd(float v);
void       Reg_SetImax_mA(float v);
void       Reg_SetDmaxPct(float v);
void       Reg_SetTrigAdv(uint32_t cnt);  /* ADC trigger advance before center [timer cnt] */
uint32_t   Reg_GetTrigAdv(void);
bool       Reg_GetMeasInOff(void);        /* true: sampling OFF-pulse center (short duty)   */
void       Reg_SetFilter(uint8_t raw_win, uint8_t ma_win);  /* current filter windows       */
uint8_t    Reg_GetRawWin(void);
uint8_t    Reg_GetMaWin(void);

/* ADC-ISR load (10 kHz). Duration in HCLK (64 MHz) cycles: us = cyc/64, %cpu = cyc/64. */
uint32_t   Reg_GetIsrLastCyc(void);
uint32_t   Reg_GetIsrMaxCyc(void);
void       Reg_ResetIsrMax(void);
void       Reg_Recalibrate(void);
void       Reg_ToggleOutput(void);        /* button: OFF <-> regulator (brake)    */

/* getters */
float      Reg_GetKd(void);
float      Reg_GetZero_mA(void);          /* calibrated sensor zero offset        */

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
