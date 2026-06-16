/**
  ******************************************************************************
  * @file    control.c
  * @brief   ACS724 current measurement + PI current regulator.
  ******************************************************************************
  */
#include "main.h"
#include "control.h"

/* Peripherals owned by the regulator (defined in main.c). */
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;

/* ---- control timing -------------------------------------------------------*/
#define CTRL_FS_HZ           (10000u)              /* PI runs once per PWM period          */

/* ---- ACS724 current sensor (taken from meatgrinder project) ---------------*/
#define ADC_VREF_MV          (3300.0f)
#define ADC_FULL_SCALE       (4096.0f)             /* 12-bit                               */
#define VOLTS_LSB            (ADC_VREF_MV / ADC_FULL_SCALE)  /* [mV/LSB]                    */
#define ADC_RES_DIV          (2.0f)                /* sensor output divided /2 before ADC  */
#define DT_OFFSET_ZERO_MV    (2500.0f)             /* [mV] sensor output @ 0 A (5V/2)      */
#define VOLT_2_CURRENT       (6.0240963855f)       /* [mA/mV] = 1000mA / 166mV (ACS724)    */
#define CURRENT_CAL_SAMPLES  (64u)                 /* samples averaged for sensor zero     */
#define ZERO_PLAUSIBLE_MA    (1500.0f)             /* reject zero offset above this:       */
                                                   /* means sensor was unpowered at cal    */

/* ---- PID / safety defaults (tunable over UART, persisted in RTC backup) ----*/
/* Tuned for the 24 V brake coil (R~11 ohm, L~57 mH @LF, tau~5 ms) with the
   flt 16/64 measurement delay (~3.9 ms): target loop bandwidth ~10 Hz.
   Ki = 2*pi*fc/Kdc (~94), Kp = Ki*tau (~0.47). See design notes. */
#define DEFAULT_KP           (0.5f)                /* [PWM counts / mA]                    */
#define DEFAULT_KI           (90.0f)               /* [PWM counts / (mA*s)]                */
#define DEFAULT_SETPOINT_MA  (500.0f)              /* default target current (button start)*/
#define DEFAULT_DMAX_PCT     (95.0f)               /* max duty clamp [%]  (safety)         */
#define DEFAULT_IMAX_MA      (2000.0f)             /* max current setpoint clamp [mA] (0-2A)*/

/* ---- HBM/HBK T22/50 Nm torque sensor on PB1 (Arduino A3 / ADC1_IN9) -------*/
/* T22 voltage output: +/-5 V at +/-Mnom. For the 50 Nm sensor this is
   10 Nm/V at the sensor output. The MCP6002 input network is 100k from signal
   and 54.9k to the 1.65 V reference (built as 51k + 3.9k in series),
   so PB1 sees:
     Vadc = (54.9/(100+54.9))*Vsig + (100/(100+54.9))*1.65 V.
   This is the largest one-resistor-change gain that keeps -3 V barely above
   ADC ground. ADC sensitivity is about 35.443 mV/Nm. */
#define T22_NOMINAL_NM       (50.0f)
#define T22_SENSOR_MV_PER_NM (5000.0f / T22_NOMINAL_NM)
#define T22_ADC_GAIN_NUM     (549L)
#define T22_ADC_GAIN_DEN     (1549L)
#define T22_ZERO_MV          (1650.0f * 1000.0f / 1549.0f)
#define DEFAULT_MMAX_NM      (50.0f)
#define DEFAULT_MKP          (35.0f)               /* [mA / Nm] outer loop P gain          */
#define DEFAULT_MKI          (120.0f)              /* [mA / (Nm*s)] outer loop I gain      */
#define DEFAULT_MKD          (0.0f)                /* [mA / Nm / sample] abs-torque damping*/
#define TORQUE_CAL_SAMPLES   (64u)

/* ---- torque-direction detector + feedforward (dissipative-brake handling) --*/
/* The brake can only ABSORB torque: with no shaft drive/speed there may be no
   torque even if coil current is high. Therefore the outer loop is supervised:
     - |T| below the noise/ripple floor  -> ARMED: current request is forced to
       zero and the torque integrator is cleared/frozen. This keeps the brake
       released until the motor really pushes.
     - |T| above the floor (debounced)    -> HOLD: ramp current smoothly toward
       feedforward + slow PID trim.
   A reversal must cross the |T|<Toff dead-band, so it always re-arms through
   DIR_ZERO and the integrator/ramp are reset on the zero-crossing. */
#define DEFAULT_TON_NM       (0.5f)    /* enter threshold: |T| above this = load present   */
#define DEFAULT_TOFF_NM      (0.25f)   /* exit threshold (hysteresis, < Ton)               */
#define DEFAULT_TDEB_SAMP    (500u)    /* debounce: consecutive samples > Ton (50 ms @10k) */
#define TORQUE_RAMP_MA_S     (3000L)   /* outer current slew rate in mA/s                  */
#define TORQUE_MEDIAN_N      (5u)      /* spike rejection for raw torque samples           */
#define FF_MAX_PTS           (8u)      /* feedforward I(T) table breakpoints              */

/* torque-direction states (kept as int8 so the getter is trivial) */
#define TDIR_NEG  (-1)
#define TDIR_ZERO (0)
#define TDIR_POS  (1)

/* Fast 10 kHz control loop uses fixed-point Q8 (value * 256). STM32G071 is
   Cortex-M0+ without FPU; keeping float and divisions out of the ADC ISR is
   the difference between a controller and a very small heater. */
#define Q8_SHIFT             (8)
#define Q8_ONE               (1L << Q8_SHIFT)
#define MA_TO_Q8(v)          ((int32_t)((v) * (float)Q8_ONE + ((v) >= 0.0f ? 0.5f : -0.5f)))
#define Q8_TO_FLOAT(v)       ((float)(v) / (float)Q8_ONE)
#define ADC_TO_MA_Q8_GAIN    (2485L)       /* 3300mV*2/4096 * 1000/166 * 256 */
#define ADC_TO_MA_Q8_ZERO    (3855422L)    /* 2500mV * 1000/166 * 256        */
#define ADC_TO_NM_Q8_GAIN_Q10 (5975L)      /* 3300/4096 / 35.443mV/Nm * 256 * 1024 */
#define ADC_TO_NM_Q8_ZERO    (7689L)       /* nominal PB1 zero = 1.065V = 30.0Nm Q8 */
#define KP_TO_Q8(v)          ((int32_t)((v) * 256.0f + ((v) >= 0.0f ? 0.5f : -0.5f)))
#define KI_TO_TICK_Q16(v)    ((int32_t)(((v) * 65536.0f / (float)CTRL_FS_HZ) + ((v) >= 0.0f ? 0.5f : -0.5f)))
#define NM_TO_Q8(v)          MA_TO_Q8(v)
#define MKP_TO_Q8(v)         KP_TO_Q8(v)
#define MKI_TO_TICK_Q16(v)   KI_TO_TICK_Q16(v)
#define MKD_TO_Q8(v)         KP_TO_Q8(v)
#define P_TERM_Q8(kp_q8, err_q8) \
  ((int32_t)(((kp_q8) * (err_q8)) >> 8))
#define I_INC_Q8(ki_tick_q16, err_q8) \
  ((int32_t)(((ki_tick_q16) * (err_q8)) >> 16))

/* RTC/TAMP backup registers: persist PID + setpoint across reset/power-cycle.
   Bump the magic whenever the defaults change so a stale saved config (e.g. the
   old kp=2/ki=0.01) is discarded and the new defaults load on next boot. */
#define CFG_MAGIC16          (0xB05Eu)
#define CFG_MAGIC_MASK       (0xFFFF0000u)
#define CFG_MAGIC_WORD       (CFG_MAGIC16 << 16)

/* ---- state (ADC ISR is the only writer of *_meas/integ; CLI writes params) */
static volatile out_mode_t g_mode        = OUT_OFF;
static volatile uint32_t   g_manual_ccr  = 0;
static volatile float      g_setpoint_mA = 0.0f;
static volatile float      g_kp          = DEFAULT_KP;
static volatile float      g_ki          = DEFAULT_KI;
static volatile float      g_dmax_pct    = DEFAULT_DMAX_PCT;
static volatile float      g_imax_mA     = DEFAULT_IMAX_MA;
static volatile float      g_m_setpoint_Nm = 0.0f;
static volatile float      g_mmax_Nm       = DEFAULT_MMAX_NM;
static volatile float      g_mkp           = DEFAULT_MKP;
static volatile float      g_mki           = DEFAULT_MKI;
static volatile float      g_mkd           = DEFAULT_MKD;

static volatile int32_t    g_setpoint_q8 = 0;
static volatile int32_t    g_kp_q8       = KP_TO_Q8(DEFAULT_KP);
static volatile int32_t    g_ki_tick_q16 = KI_TO_TICK_Q16(DEFAULT_KI);
static volatile int32_t    g_integ_q8    = 0;      /* integral contribution [PWM cnt Q8]   */
static volatile int32_t    g_err_z_q8    = 0;      /* previous error, for optional D term  */
static volatile int32_t    g_I_q8        = 0;      /* filtered measured current [mA Q8]    */
static volatile int32_t    g_zero_q8     = 0;      /* calibrated sensor zero [mA Q8]       */
static volatile int32_t    g_M_q8        = 0;      /* filtered measured torque [Nm Q8]     */
static volatile int32_t    g_M_zero_q8   = 0;      /* calibrated torque zero [Nm Q8]       */
static volatile int32_t    g_M_set_q8    = 0;      /* torque setpoint [Nm Q8]              */
static volatile int32_t    g_mkp_q8      = MKP_TO_Q8(DEFAULT_MKP);
static volatile int32_t    g_mki_tick_q16 = MKI_TO_TICK_Q16(DEFAULT_MKI);
static volatile int32_t    g_mkd_q8      = MKD_TO_Q8(DEFAULT_MKD);
static volatile int32_t    g_m_integ_q8  = 0;      /* outer integral contribution [mA Q8]  */
static volatile int32_t    g_m_abs_z_q8  = 0;      /* previous |torque| for D damping      */
static volatile int32_t    g_m_req_q8    = 0;      /* ramped outer current request [mA Q8] */

/* torque-direction detector + feedforward table (see notes above) */
static volatile int32_t    g_ton_q8   = NM_TO_Q8(DEFAULT_TON_NM);  /* enter threshold [Nm Q8] */
static volatile int32_t    g_toff_q8  = NM_TO_Q8(DEFAULT_TOFF_NM); /* exit threshold  [Nm Q8] */
static volatile uint16_t   g_tdeb     = DEFAULT_TDEB_SAMP;         /* debounce sample count   */
static volatile uint16_t   g_dir_cnt  = 0;         /* debounce counter (ISR-owned)         */
static volatile int8_t     g_tdir     = TDIR_ZERO; /* detected drive direction (-1/0/+1)   */
static volatile int32_t    g_iff_q8   = 0;         /* feedforward current at setpoint [mA Q8]*/
static volatile int32_t    g_ff_t_q8[FF_MAX_PTS];  /* table torque breakpoints [Nm Q8] asc */
static volatile int32_t    g_ff_i_q8[FF_MAX_PTS];  /* table current values     [mA Q8]     */
static volatile uint8_t    g_ff_n     = 0;         /* number of table points (0 = no FF)   */
static volatile uint32_t   g_dmax_ccr    = (95u * PWM_ARR) / 100u;
static volatile uint32_t   g_ccr         = 0;      /* duty currently applied [counts]      */
static volatile uint32_t   g_trig_adv    = 160u;   /* ADC trigger advance before center [cnt]*/
static volatile bool       g_meas_in_off = false;  /* true: sample OFF-pulse center (short)  */
static volatile int32_t    g_cal_sum_q8  = 0;
static volatile int32_t    g_m_cal_sum_q8 = 0;
static volatile uint16_t   g_cal_cnt     = 0;
static volatile bool       g_cal_done    = false;

/* ---- ADC trigger ----------------------------------------------------------*/
#define ADC_TRIG_ADVANCE_CNT  (160u)   /* default advance: timer counts BEFORE the chosen   */
                                       /* sample center (1 count = 15.6 ns @ 64 MHz), to    */
                                       /* compensate the sampling+conversion delay.         */
/* Where to take the single sample depends on duty, with hysteresis around 20%:
   - long  ON pulse (>=21%): center of the ON plateau  (CNT=0,   PWM1 trigger)
   - short ON pulse (<=19%): center of the OFF interval (CNT=ARR, PWM2 trigger)
   For a linear (triangular) current both centers equal the average current, but
   the OFF center is far from the commutation edges when the ON pulse is short. */
#define DUTY_HYST_LO_CCR      ((19u * PWM_ARR) / 100u)  /* below -> measure in OFF center */
#define DUTY_HYST_HI_CCR      ((21u * PWM_ARR) / 100u)  /* above -> measure in ON  center */

/* ADC DMA target: current PA0 and torque PB1, both sampled on the same trigger. */
static uint16_t adc_dma[2];

static uint8_t pow2_shift_u8(uint8_t v)
{
  if (v == 0u || (v & (uint8_t)(v - 1u)) != 0u) return 0xFFu;
  uint8_t s = 0u;
  while (v > 1u) { v >>= 1; s++; }
  return s;
}

static int32_t median5_i32(int32_t a, int32_t b, int32_t c, int32_t d, int32_t e)
{
  int32_t v[5] = { a, b, c, d, e };
  for (uint8_t i = 1u; i < 5u; i++)
  {
    int32_t x = v[i];
    uint8_t j = i;
    while (j > 0u && v[j - 1u] > x)
    {
      v[j] = v[j - 1u];
      j--;
    }
    v[j] = x;
  }
  return v[2];
}

static int32_t slew_q8(int32_t cur_q8, int32_t target_q8, int32_t step_q8)
{
  if (target_q8 > cur_q8 + step_q8) return cur_q8 + step_q8;
  if (target_q8 < cur_q8 - step_q8) return cur_q8 - step_q8;
  return target_q8;
}

static void sync_fixed_params(void)
{
  g_setpoint_q8 = MA_TO_Q8(g_setpoint_mA);
  g_kp_q8       = KP_TO_Q8(g_kp);
  g_ki_tick_q16 = KI_TO_TICK_Q16(g_ki);
  g_M_set_q8    = NM_TO_Q8(g_m_setpoint_Nm);
  g_mkp_q8      = MKP_TO_Q8(g_mkp);
  g_mki_tick_q16 = MKI_TO_TICK_Q16(g_mki);
  g_mkd_q8      = MKD_TO_Q8(g_mkd);
  g_dmax_ccr    = (uint32_t)(g_dmax_pct * 0.01f * (float)PWM_ARR);
  if (g_dmax_ccr > PWM_ARR) g_dmax_ccr = PWM_ARR;
}

/* ---- feedforward I(T) table -----------------------------------------------*/
/* Linear interpolation of the calibrated current for a target torque. Called
   only when the setpoint/table changes (NOT from the ISR), so 64-bit math is
   fine here. Outside the breakpoints the endpoints are held (no extrapolation).*/
static int32_t ff_lookup_q8(int32_t t_q8)
{
  uint8_t n = g_ff_n;
  if (n == 0u)                  return 0;            /* no table -> pure PID    */
  if (t_q8 <= g_ff_t_q8[0])     return g_ff_i_q8[0];
  if (t_q8 >= g_ff_t_q8[n - 1]) return g_ff_i_q8[n - 1];
  for (uint8_t k = 1u; k < n; k++)
  {
    if (t_q8 <= g_ff_t_q8[k])
    {
      int32_t t0 = g_ff_t_q8[k - 1], t1 = g_ff_t_q8[k];
      int32_t i0 = g_ff_i_q8[k - 1], i1 = g_ff_i_q8[k];
      int32_t dt = t1 - t0;
      if (dt <= 0) return i0;
      return i0 + (int32_t)(((int64_t)(i1 - i0) * (int64_t)(t_q8 - t0)) / (int64_t)dt);
    }
  }
  return g_ff_i_q8[n - 1];
}

/* Seed a 2-point default line (0 Nm -> 0 mA) .. (mmax -> imax). Rough, meant to
   be replaced by bench calibration via the 'ff' command. */
static void ff_set_default(void)
{
  g_ff_t_q8[0] = 0;                       g_ff_i_q8[0] = 0;
  g_ff_t_q8[1] = NM_TO_Q8(g_mmax_Nm);     g_ff_i_q8[1] = MA_TO_Q8(g_imax_mA);
  g_ff_n = 2u;
}

/* ---- moving average + live channel ----------------------------------------*/
/* Two cascaded moving-average filters, windows runtime-tunable (CLI 'flt') up
   to the buffer maxima:  filter 1 on the raw ADC, filter 2 on the current [mA]. */
#define MA_MAX   (32u)
#define IMA_MAX  (128u)
static volatile uint16_t g_ma_buf[MA_MAX];
static volatile uint8_t  g_ma_win  = 16u;        /* filter-1 window (raw ADC samples)     */
static volatile uint8_t  g_ma_idx  = 0;
static volatile uint32_t g_ma_sum  = 0;
static volatile uint8_t  g_ma_cnt  = 0;          /* filled count during warm-up           */
static volatile uint16_t g_raw_avg = 0;          /* filter-1 output: averaged raw ADC      */
static volatile uint8_t  g_ma_shift = 4u;        /* log2(window), or 0xFF if not pow2      */

static volatile int32_t  g_ima_buf[IMA_MAX];
static volatile uint8_t  g_ima_win = 64u;        /* filter-2 window (current mA samples)  */
static volatile uint8_t  g_ima_idx = 0;
static volatile uint8_t  g_ima_cnt = 0;
static volatile int32_t  g_ima_sum_q8 = 0;
static volatile uint8_t  g_ima_shift = 6u;       /* log2(window), or 0xFF if not pow2      */

static volatile int32_t  g_mma_buf[IMA_MAX];
static volatile uint8_t  g_mma_idx = 0;
static volatile uint8_t  g_mma_cnt = 0;
static volatile int32_t  g_mma_sum_q8 = 0;
static volatile uint16_t g_torque_raw_avg = 0;
static volatile int32_t  g_m_med_buf[TORQUE_MEDIAN_N];
static volatile uint8_t  g_m_med_idx = 0;
static volatile uint8_t  g_m_med_cnt = 0;

/* Live ADC channel: ADC_CHANNEL_0 = PA0 (current sensor, production) or
   ADC_CHANNEL_9 = PB1 (Arduino A3, for bench-testing the chain on a battery). */
static volatile uint32_t g_adc_chan = ADC_CHANNEL_0;
static volatile float    g_vdda_mV  = ADC_VREF_MV;   /* updated by vref/ain diagnostics  */

/* Configure the ADC for one channel, OC4REF-triggered + circular DMA, and start
   it. Also resets the moving-average and the sensor-zero calibration. */
static void adc_start_live(uint32_t chan)
{
  (void)chan;  /* live mode always scans PA0 current + PB1 torque */
  /* Per-channel analog pin config. NB: on STM32G0 the internal pull-up/down
     resistors are disabled by hardware in analog mode (verified: enabling a
     pull-down on PA0 did not move raw_avg), so a defined path to GND / an input
     low-pass must be EXTERNAL (resistor or RC). Keep the pins NOPULL. */
  GPIO_InitTypeDef g = {0};
  g.Mode = GPIO_MODE_ANALOG; g.Pull = GPIO_NOPULL;
  g.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOA, &g);
  __HAL_RCC_GPIOB_CLK_ENABLE();
  g.Pin = GPIO_PIN_1;
  HAL_GPIO_Init(GPIOB, &g);

  HAL_ADC_Stop_DMA(&hadc1);
  hadc1.Init.ScanConvMode          = ADC_SCAN_ENABLE;
  hadc1.Init.NbrOfConversion       = 2;
  hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIG_T2_TRGO;
  hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.SamplingTimeCommon1   = ADC_SAMPLETIME_39CYCLES_5;
  HAL_ADC_Init(&hadc1);

  ADC_ChannelConfTypeDef c = {0};
  c.Channel      = ADC_CHANNEL_0;
  c.Rank         = ADC_REGULAR_RANK_1;
  c.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  HAL_ADC_ConfigChannel(&hadc1, &c);
  c.Channel      = ADC_CHANNEL_9;
  c.Rank         = ADC_REGULAR_RANK_2;
  HAL_ADC_ConfigChannel(&hadc1, &c);

  /* clear both MA buffers too: the running sums subtract buf[idx], so stale
     entries would corrupt them after a reset. */
  for (uint16_t k = 0; k < MA_MAX; k++)  g_ma_buf[k]  = 0;
  for (uint16_t k = 0; k < IMA_MAX; k++) g_ima_buf[k] = 0;
  for (uint16_t k = 0; k < IMA_MAX; k++) g_mma_buf[k] = 0;
  for (uint16_t k = 0; k < TORQUE_MEDIAN_N; k++) g_m_med_buf[k] = 0;
  g_ma_idx = 0; g_ma_sum = 0; g_ma_cnt = 0; g_raw_avg = 0;
  g_ima_idx = 0; g_ima_sum_q8 = 0; g_ima_cnt = 0;
  g_mma_idx = 0; g_mma_sum_q8 = 0; g_mma_cnt = 0; g_torque_raw_avg = 0;
  g_m_med_idx = 0; g_m_med_cnt = 0;
  g_cal_done = false; g_cal_cnt = 0; g_cal_sum_q8 = 0; g_m_cal_sum_q8 = 0; g_I_q8 = 0; g_M_q8 = 0;
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma, 2);
}

/* Clamp duty to the configured safety maximum and drive both PWM outputs:
   CH1 = PA5 (transistor gate + on-board LD4), CH2 = PA1 (scope/mirror). */
static void apply_output(uint32_t ccr)
{
  uint32_t maxc = g_dmax_ccr;
  if (ccr > maxc) ccr = maxc;
  TIM2->CCR1 = ccr;            /* PA5 = transistor gate + LD4 */
  TIM2->CCR2 = ccr;            /* PA1 = mirror for the scope   */
  g_ccr = ccr;

  /* Choose the sample location from duty, with 2% hysteresis around 20%. The
     current is a linear (triangular) ramp, so both the ON-pulse center (CNT=0)
     and the OFF-interval center (CNT=ARR) equal the average current; we pick the
     one farthest from the switching edges. OC4REF triggers the ADC g_trig_adv
     counts BEFORE that center (compensates the sampling+conversion delay). */
  if (ccr <= DUTY_HYST_LO_CCR)      g_meas_in_off = true;   /* short pulse -> OFF center */
  else if (ccr >= DUTY_HYST_HI_CCR) g_meas_in_off = false;  /* long  pulse -> ON  center */

  uint32_t adv = g_trig_adv;
  if (g_meas_in_off)
  {
    /* OFF center (CNT=ARR): CH4 = PWM2, OC4REF rises at CNT=CCR4 on the up-count
       => (ARR-CCR4) counts before the peak. */
    TIM2->CCMR2 |= TIM_CCMR2_OC4M_0;                        /* CH4 -> PWM mode 2 */
    uint32_t c = (adv < PWM_ARR) ? (PWM_ARR - adv) : 1u;
    if (c < 1u) c = 1u;
    TIM2->CCR4 = c;
  }
  else
  {
    /* ON center (CNT=0): CH4 = PWM1, OC4REF rises at CNT=CCR4 on the down-count
       => CCR4 counts before center. Keep the trigger inside the ON pulse. */
    TIM2->CCMR2 &= ~TIM_CCMR2_OC4M_0;                       /* CH4 -> PWM mode 1 */
    uint32_t c = adv;
    if (c >= ccr && ccr > 1u) c = ccr - 1u;
    if (c < 1u) c = 1u;
    TIM2->CCR4 = c;
  }
}

/* ---- config persistence in RTC/TAMP backup registers ---------------------*/
static uint16_t f_to_u16(float v, float scale, uint16_t maxv)
{
  if (v < 0.0f) v = 0.0f;
  float x = v * scale + 0.5f;
  if (x > (float)maxv) return maxv;
  return (uint16_t)x;
}

static float u16_to_f(uint16_t v, float scale)
{
  return (float)v / scale;
}

static uint32_t pack2(uint16_t hi, uint16_t lo)
{
  return ((uint32_t)hi << 16) | (uint32_t)lo;
}

static uint16_t hi16(uint32_t v) { return (uint16_t)(v >> 16); }
static uint16_t lo16(uint32_t v) { return (uint16_t)v; }

static uint16_t pack_limits(float imax_mA, float dmax_pct)
{
  uint16_t imax10 = f_to_u16(imax_mA, 0.1f, 511u);  /* 0..5110 mA, 10 mA/LSB */
  uint16_t dmax1  = f_to_u16(dmax_pct, 1.0f, 100u); /* 0..100%, 1%/LSB       */
  return (uint16_t)((dmax1 << 9) | imax10);
}

static void unpack_limits(uint16_t p)
{
  g_imax_mA  = (float)(p & 0x01FFu) * 10.0f;
  g_dmax_pct = (float)((p >> 9) & 0x007Fu);
}

static void cfg_save(void)
{
  TAMP->BKP1R = pack2(f_to_u16(g_kp, 256.0f, 0xFFFFu),
                      f_to_u16(g_ki, 10.0f, 0xFFFFu));
  TAMP->BKP2R = pack2(0u,
                      f_to_u16(g_setpoint_mA, 1.0f, 0xFFFFu));
  TAMP->BKP3R = pack2(f_to_u16(g_mkp, 256.0f, 0xFFFFu),
                      f_to_u16(g_mki, 10.0f, 0xFFFFu));
  TAMP->BKP4R = pack2(f_to_u16(g_mkd, 256.0f, 0xFFFFu),
                      f_to_u16(g_mmax_Nm, 10.0f, 0xFFFFu));
  TAMP->BKP0R = CFG_MAGIC_WORD | pack_limits(g_imax_mA, g_dmax_pct);  /* marker last */
}

static void cfg_load(void)
{
  if ((TAMP->BKP0R & CFG_MAGIC_MASK) == CFG_MAGIC_WORD)
  {
    unpack_limits(lo16(TAMP->BKP0R));
    g_kp            = u16_to_f(hi16(TAMP->BKP1R), 256.0f);
    g_ki            = u16_to_f(lo16(TAMP->BKP1R), 10.0f);
    g_setpoint_mA   = u16_to_f(lo16(TAMP->BKP2R), 1.0f);
    g_mkp           = u16_to_f(hi16(TAMP->BKP3R), 256.0f);
    g_mki           = u16_to_f(lo16(TAMP->BKP3R), 10.0f);
    g_mkd           = u16_to_f(hi16(TAMP->BKP4R), 256.0f);
    g_mmax_Nm       = u16_to_f(lo16(TAMP->BKP4R), 10.0f);
  }
  sync_fixed_params();
}

void Reg_Init(void)
{
  /* ADC self-calibration is mandatory on STM32G0 before conversions. */
  HAL_ADCEx_Calibration_Start(&hadc1);

  /* enable access to the backup domain registers (TAMP_BKPxR) */
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_RTCAPB_CLK_ENABLE();

  g_kp = DEFAULT_KP; g_ki = DEFAULT_KI;
  g_mkp = DEFAULT_MKP; g_mki = DEFAULT_MKI; g_mkd = DEFAULT_MKD;
  g_dmax_pct = DEFAULT_DMAX_PCT; g_imax_mA = DEFAULT_IMAX_MA; g_mmax_Nm = DEFAULT_MMAX_NM;
  g_mode = OUT_OFF; g_manual_ccr = 0; g_setpoint_mA = DEFAULT_SETPOINT_MA;
  g_m_setpoint_Nm = 0.0f;
  sync_fixed_params();
  g_integ_q8 = 0; g_err_z_q8 = 0; g_m_integ_q8 = 0; g_m_abs_z_q8 = 0; g_m_req_q8 = 0;
  g_tdir = TDIR_ZERO; g_dir_cnt = 0; g_iff_q8 = 0;
  g_cal_done = false; g_cal_cnt = 0; g_cal_sum_q8 = 0; g_m_cal_sum_q8 = 0; g_I_q8 = 0; g_M_q8 = 0;

  cfg_load();                         /* restore saved PID + setpoint (output stays OFF) */
  ff_set_default();                   /* seed rough I(T) line from imax/mmax (calibrate!) */

  TIM2->CCR1 = 0;
  TIM2->CCR2 = 0;
  TIM2->CCR4 = ADC_TRIG_ADVANCE_CNT;                    /* ADC trigger advance before center */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);             /* 10 kHz PWM, PA5 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);             /* 10 kHz PWM, PA1 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);             /* CH4: OC4REF = ADC trigger (no pin) */
  /* Timer + CH4 trigger run forever (never stopped), so the ADC keeps sampling
     even when the regulator/PWM output is OFF (duty 0): current stays measured. */
  adc_start_live(g_adc_chan);                          /* single live channel, OC4REF + DMA */
}

/* ---- ISR load instrumentation (this callback runs at 10 kHz on a no-FPU M0+) */
static volatile uint32_t g_isr_last = 0;   /* last ISR duration [HCLK cycles]  */
static volatile uint32_t g_isr_max  = 0;   /* peak ISR duration [HCLK cycles]  */

/* Record the elapsed HCLK cycles since t0 (SysTick down-counter, handles wrap). */
static inline void isr_done(uint32_t t0)
{
  uint32_t t1   = SysTick->VAL;
  uint32_t load = SysTick->LOAD + 1u;
  uint32_t dt   = (t0 >= t1) ? (t0 - t1) : (t0 + load - t1);
  g_isr_last = dt;
  if (dt > g_isr_max) g_isr_max = dt;
}

uint32_t Reg_GetIsrLastCyc(void) { return g_isr_last; }
uint32_t Reg_GetIsrMaxCyc(void)  { return g_isr_max;  }
void     Reg_ResetIsrMax(void)   { g_isr_max = 0; }

/*
 * ADC conversion complete (DMA1_Channel2_3 IRQ, prio 1).
 * Triggered by TIM2 OC4REF (CH4, Pulse=1): a single trigger per PWM period at
 * CNT=0 = the center of the ON pulse. (Previously TRGO=UPDATE fired twice per
 * period - ON- and OFF-centers - and we discarded the down-count one via DIR;
 * now the hardware delivers exactly one centered conversion at 10 kHz.)
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance != ADC1) return;
  uint32_t t0 = SysTick->VAL;

  /* PA8 scope marker disabled (ADC verified in sync). Re-enable for debug:
     GPIOA->ODR ^= GPIO_PIN_8;  // toggles once per period at the sample instant */

  /* Moving average of the raw ADC over MA_WIN samples (running sum): one sample
     per period, each at the chosen center, then averaged -> smooth feedback. */
  g_ma_sum -= g_ma_buf[g_ma_idx];
  g_ma_buf[g_ma_idx] = adc_dma[0];
  g_ma_sum += adc_dma[0];
  if (++g_ma_idx >= g_ma_win) g_ma_idx = 0;
  if (g_ma_cnt < g_ma_win) g_ma_cnt++;
  if (g_ma_cnt == g_ma_win && g_ma_shift != 0xFFu) g_raw_avg = (uint16_t)(g_ma_sum >> g_ma_shift);
  else                                             g_raw_avg = (uint16_t)(g_ma_sum / g_ma_cnt);

  /* ACS724 current [mA Q8] (signed about the zero point), from averaged raw. */
  int32_t i_signed_q8 = (int32_t)g_raw_avg * ADC_TO_MA_Q8_GAIN - ADC_TO_MA_Q8_ZERO;
  uint16_t torque_raw = adc_dma[1];
  g_torque_raw_avg = torque_raw;
  int32_t m_signed_q8 = (((int32_t)torque_raw * ADC_TO_NM_Q8_GAIN_Q10) >> 10) - ADC_TO_NM_Q8_ZERO;

  if (!g_cal_done)
  {
    g_cal_sum_q8 += i_signed_q8;
    g_m_cal_sum_q8 += m_signed_q8;
    if (++g_cal_cnt >= CURRENT_CAL_SAMPLES)
    {
      int32_t z_q8 = g_cal_sum_q8 >> 6;              /* CURRENT_CAL_SAMPLES = 64 */
      /* Implausible offset -> sensor was unpowered during calibration.
         Fall back to the nominal 2.5 V zero instead of a bogus offset. */
      if (z_q8 > MA_TO_Q8(ZERO_PLAUSIBLE_MA) || z_q8 < -MA_TO_Q8(ZERO_PLAUSIBLE_MA)) z_q8 = 0;
      g_zero_q8  = z_q8;
      g_M_zero_q8 = g_m_cal_sum_q8 >> 6;
      g_cal_done = true;
    }
    g_I_q8 = 0;
    g_M_q8 = 0;
    apply_output(0);                                    /* force OFF while calibrating */
    isr_done(t0);
    return;
  }

  /* Filter 2: moving average over IMA_WIN current [mA] samples (running sum).
     The current is kept SIGNED here - do NOT rectify each sample with fabsf():
     the noise around zero is symmetric, so averaging signed values cancels it to
     ~0, whereas averaging |sample| leaves a positive bias (~0.8*sigma ~ tens of
     mA) that no amount of averaging or zero-calibration can remove. */
  int32_t i_q8 = i_signed_q8 - g_zero_q8;
  g_ima_sum_q8 -= g_ima_buf[g_ima_idx];
  g_ima_buf[g_ima_idx] = i_q8;
  g_ima_sum_q8 += i_q8;
  if (++g_ima_idx >= g_ima_win) g_ima_idx = 0;
  if (g_ima_cnt < g_ima_win) g_ima_cnt++;
  if (g_ima_cnt == g_ima_win && g_ima_shift != 0xFFu) g_I_q8 = g_ima_sum_q8 >> g_ima_shift;
  else                                               g_I_q8 = g_ima_sum_q8 / (int32_t)g_ima_cnt;

  int32_t m_q8 = m_signed_q8 - g_M_zero_q8;
  g_m_med_buf[g_m_med_idx] = m_q8;
  if (++g_m_med_idx >= TORQUE_MEDIAN_N) g_m_med_idx = 0;
  if (g_m_med_cnt < TORQUE_MEDIAN_N) g_m_med_cnt++;
  if (g_m_med_cnt >= TORQUE_MEDIAN_N)
  {
    m_q8 = median5_i32(g_m_med_buf[0], g_m_med_buf[1], g_m_med_buf[2],
                       g_m_med_buf[3], g_m_med_buf[4]);
  }

  g_mma_sum_q8 -= g_mma_buf[g_mma_idx];
  g_mma_buf[g_mma_idx] = m_q8;
  g_mma_sum_q8 += m_q8;
  if (++g_mma_idx >= g_ima_win) g_mma_idx = 0;
  if (g_mma_cnt < g_ima_win) g_mma_cnt++;
  if (g_mma_cnt == g_ima_win && g_ima_shift != 0xFFu) g_M_q8 = g_mma_sum_q8 >> g_ima_shift;
  else                                               g_M_q8 = g_mma_sum_q8 / (int32_t)g_mma_cnt;

  uint32_t ccr;
  switch (g_mode)
  {
    case OUT_MANUAL:
      ccr = g_manual_ccr;
      break;

    case OUT_REG:
    case OUT_TORQUE:
    {
      if (g_mode == OUT_TORQUE)
      {
        int32_t m_signed = g_M_q8;     /* signed filtered torque [Nm Q8] = drive dir   */
        int32_t m_abs_q8 = (m_signed < 0) ? -m_signed : m_signed;

        /* --- direction / load detector (the only way to tell the motor is
           actually pushing, since there is no speed sensor). Hysteresis +
           debounce reject torque ripple around zero. A reversal must pass the
           |T|<Toff dead-band, so it re-arms through TDIR_ZERO. */
        switch (g_tdir)
        {
          case TDIR_POS:
            if (m_signed < g_toff_q8) { g_tdir = TDIR_ZERO; g_dir_cnt = 0; g_m_integ_q8 = 0; }
            break;
          case TDIR_NEG:
            if (m_signed > -g_toff_q8) { g_tdir = TDIR_ZERO; g_dir_cnt = 0; g_m_integ_q8 = 0; }
            break;
          default: /* TDIR_ZERO (ARMED) */
            if (m_signed > g_ton_q8)
            { if (++g_dir_cnt >= g_tdeb) { g_tdir = TDIR_POS; g_dir_cnt = 0; g_m_integ_q8 = 0; } }
            else if (m_signed < -g_ton_q8)
            { if (++g_dir_cnt >= g_tdeb) { g_tdir = TDIR_NEG; g_dir_cnt = 0; g_m_integ_q8 = 0; } }
            else g_dir_cnt = 0;
            break;
        }

        int32_t imax_q8 = MA_TO_Q8(g_imax_mA);
        int32_t mout_q8;
        int32_t ramp_step_q8 = (int32_t)(((int64_t)TORQUE_RAMP_MA_S * Q8_ONE) / CTRL_FS_HZ);
        if (ramp_step_q8 < 1) ramp_step_q8 = 1;

        if (g_tdir == TDIR_ZERO)
        {
          /* ARMED: motor not pushing -> release the brake and freeze/clear the
             outer loop. Do not hold feedforward here: a current without torque
             can lock the drive before it starts. */
          g_m_integ_q8 = 0;
          g_m_abs_z_q8 = m_abs_q8;
          g_m_req_q8 = slew_q8(g_m_req_q8, 0, ramp_step_q8);
          mout_q8 = g_m_req_q8;
        }
        else
        {
          /* HOLD: load present -> PID trims around the feedforward current to
             hold |T| at the setpoint. The request is slew-limited below, so
             ARMED->HOLD cannot slam the brake even if I_ff is large. */
          int32_t merr_q8 = g_M_set_q8 - m_abs_q8;
          int32_t md_q8 = -P_TERM_Q8(g_mkd_q8, (m_abs_q8 - g_m_abs_z_q8));
          g_m_abs_z_q8 = m_abs_q8;
          mout_q8 = g_iff_q8 + P_TERM_Q8(g_mkp_q8, merr_q8) + g_m_integ_q8 + md_q8;
          bool msat_hi = (mout_q8 >= imax_q8);
          bool msat_lo = (mout_q8 <= 0);

          if (g_mki_tick_q16 > 0 && !((msat_hi && merr_q8 > 0) || (msat_lo && merr_q8 < 0)))
          {
            g_m_integ_q8 += I_INC_Q8(g_mki_tick_q16, merr_q8);
            if (g_m_integ_q8 > imax_q8) g_m_integ_q8 = imax_q8;
            else if (g_m_integ_q8 < -imax_q8) g_m_integ_q8 = -imax_q8;
            mout_q8 = g_iff_q8 + P_TERM_Q8(g_mkp_q8, merr_q8) + g_m_integ_q8 + md_q8;
          }
          if (mout_q8 > imax_q8) mout_q8 = imax_q8;
          else if (mout_q8 < 0)  mout_q8 = 0;
          g_m_req_q8 = slew_q8(g_m_req_q8, mout_q8, ramp_step_q8);
          mout_q8 = g_m_req_q8;
        }

        if (mout_q8 > imax_q8) mout_q8 = imax_q8;
        else if (mout_q8 < 0)  mout_q8 = 0;
        g_setpoint_q8 = mout_q8;       /* outer torque loop drives inner current loop */
      }

      /* Fast fixed-point PI. Anti-windup is conditional integration: when the
         duty is already clamped, keep integrating only if the error pulls it
         back toward the linear region. */
      int32_t err_q8 = g_setpoint_q8 - g_I_q8;
      int32_t p_q8   = P_TERM_Q8(g_kp_q8, err_q8);
      int32_t d_q8   = 0;          /* D is intentionally disabled in the fast loop. */
      g_err_z_q8 = err_q8;

      int32_t out_q8 = p_q8 + g_integ_q8 + d_q8;
      int32_t max_q8 = (int32_t)(g_dmax_ccr << Q8_SHIFT);
      bool sat_hi = (out_q8 >= max_q8);
      bool sat_lo = (out_q8 <= 0);

      if (g_ki_tick_q16 > 0 && !((sat_hi && err_q8 > 0) || (sat_lo && err_q8 < 0)))
      {
        g_integ_q8 += I_INC_Q8(g_ki_tick_q16, err_q8);
        if (g_integ_q8 > max_q8) g_integ_q8 = max_q8;
        else if (g_integ_q8 < -max_q8) g_integ_q8 = -max_q8;
        out_q8 = p_q8 + g_integ_q8 + d_q8;
      }

      if (out_q8 > max_q8) out_q8 = max_q8;
      else if (out_q8 < 0) out_q8 = 0;
      ccr = (uint32_t)(out_q8 >> Q8_SHIFT);
      break;
    }

    case OUT_OFF:
    default:
      ccr = 0;
      g_integ_q8 = 0;
      g_err_z_q8 = 0;
      g_m_integ_q8 = 0;
      g_m_abs_z_q8 = 0;
      g_m_req_q8 = 0;
      g_tdir = TDIR_ZERO;
      g_dir_cnt = 0;
      break;
  }
  apply_output(ccr);
  isr_done(t0);
}

/*
 * Internal-reference ADC sanity check.
 *
 * The ACS724 reading is suspected of jumping, so this routine isolates the ADC
 * core+clock from the noisy sensor signal path: it pauses the timer-triggered
 * DMA, retargets the ADC to the internal VREFINT bandgap (a rock-steady ~1.21 V
 * source) with a LONG sampling time, captures a polled burst, and reports
 * raw min/max/avg plus the VDDA back-computed from the factory calibration.
 *
 * Sampling time matters: VREFINT has a high source impedance and needs ~5 us of
 * sampling. The control loop samples the ACS724 at only 39.5 ADC cycles
 * (~1.23 us @ 32 MHz) which is fine for the low-impedance /2 divider but FAR too
 * short for VREFINT; using it here would itself produce a "floating" reading.
 * So the diagnostic forces 160.5 cycles (~5 us). If VREFINT is steady here, the
 * ADC/clock are healthy and the jumpiness is in the sensor path, not the ADC.
 */
void Reg_VrefDiag(vref_diag_t *out, uint16_t nsamples)
{
  if (nsamples == 0u) nsamples = 1u;

  out_mode_t saved_mode = g_mode;
  g_mode = OUT_OFF;
  apply_output(0);                          /* transistor off: quiet supply during test */

  HAL_ADC_Stop_DMA(&hadc1);                 /* pause timer-triggered conversions */

  /* Reconfigure: software-triggered single conversions, long sampling time. */
  hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
  hadc1.Init.NbrOfConversion       = 1;
  hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.SamplingTimeCommon1   = ADC_SAMPLETIME_160CYCLES_5;
  HAL_ADC_Init(&hadc1);

  ADC_ChannelConfTypeDef s = {0};
  s.Channel      = ADC_CHANNEL_VREFINT;     /* also enables the VREFINT buffer */
  s.Rank         = ADC_REGULAR_RANK_1;
  s.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  HAL_ADC_ConfigChannel(&hadc1, &s);

  uint32_t sum = 0u;
  uint16_t mn  = 0xFFFFu, mx = 0u;
  for (uint16_t i = 0; i < nsamples; i++)
  {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t v = (uint16_t)HAL_ADC_GetValue(&hadc1);
    sum += v;
    if (v < mn) mn = v;
    if (v > mx) mx = v;
  }
  HAL_ADC_Stop(&hadc1);

  float avg = (float)sum / (float)nsamples;
  out->n       = nsamples;
  out->raw_min = mn;
  out->raw_max = mx;
  out->raw_avg = avg;

  /* VDDA = 3000 mV * VREFINT_CAL / VREFINT_meas  (cal taken at VDDA = 3.0 V). */
  uint16_t cal = *VREFINT_CAL_ADDR;
  out->vdda_mV = (avg > 0.0f) ? (3000.0f * (float)cal / avg) : 0.0f;
  out->vref_mV = avg * (out->vdda_mV / ADC_FULL_SCALE);
  if (out->vdda_mV > 0.0f) g_vdda_mV = out->vdda_mV;   /* keep the measured supply */

  /* restore the live single-channel OC4REF + DMA acquisition */
  adc_start_live(g_adc_chan);

  g_mode = saved_mode;
}

/*
 * Absolute-voltage check of the whole ADC chain against a known reference on
 * A3 = PB1 = ADC1_IN9. Measures VREFINT to recover the true VDDA, then PB1, and
 * reports PB1 volts = raw * VDDA / 4096 (independent of the VDDA tolerance).
 * Use with a known voltage (e.g. a 1.6153 V cell) to confirm scaling + that the
 * ADC self-calibration is in effect.
 */
void Reg_A3Diag(vref_diag_t *out, uint16_t nsamples)
{
  if (nsamples == 0u) nsamples = 1u;

  out_mode_t saved_mode = g_mode;
  g_mode = OUT_OFF;
  apply_output(0);

  /* PB1 as analog input (G0 resets pins to analog, but make it explicit). */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  { GPIO_InitTypeDef g = {0}; g.Pin = GPIO_PIN_1; g.Mode = GPIO_MODE_ANALOG; g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &g); }

  HAL_ADC_Stop_DMA(&hadc1);

  /* software single conversions, long sampling time (accurate). */
  hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
  hadc1.Init.NbrOfConversion       = 1;
  hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.SamplingTimeCommon1   = ADC_SAMPLETIME_160CYCLES_5;
  HAL_ADC_Init(&hadc1);

  ADC_ChannelConfTypeDef s = {0};
  s.Rank         = ADC_REGULAR_RANK_1;
  s.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;

  /* 1) VREFINT -> true VDDA */
  s.Channel = ADC_CHANNEL_VREFINT;
  HAL_ADC_ConfigChannel(&hadc1, &s);
  uint32_t sv = 0u;
  for (uint16_t i = 0; i < nsamples; i++)
  { HAL_ADC_Start(&hadc1); HAL_ADC_PollForConversion(&hadc1, 10); sv += HAL_ADC_GetValue(&hadc1); }
  HAL_ADC_Stop(&hadc1);
  float avgv = (float)sv / (float)nsamples;
  float vdda = (avgv > 0.0f) ? (3000.0f * (float)(*VREFINT_CAL_ADDR) / avgv) : 0.0f;

  /* 2) PB1 = ADC1_IN9 (Arduino A3) */
  s.Channel = ADC_CHANNEL_9;
  HAL_ADC_ConfigChannel(&hadc1, &s);
  uint32_t sp = 0u; uint16_t mn = 0xFFFFu, mx = 0u;
  for (uint16_t i = 0; i < nsamples; i++)
  { HAL_ADC_Start(&hadc1); HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t v = (uint16_t)HAL_ADC_GetValue(&hadc1); sp += v; if (v < mn) mn = v; if (v > mx) mx = v; }
  HAL_ADC_Stop(&hadc1);
  float avg = (float)sp / (float)nsamples;

  out->n       = nsamples;
  out->raw_min = mn;
  out->raw_max = mx;
  out->raw_avg = avg;
  out->vdda_mV = vdda;
  out->vref_mV = avg * (vdda / ADC_FULL_SCALE);   /* PB1 voltage [mV] */
  if (vdda > 0.0f) g_vdda_mV = vdda;              /* keep the measured supply */

  /* restore the live single-channel OC4REF + DMA acquisition */
  adc_start_live(g_adc_chan);

  g_mode = saved_mode;
}

/* ADC self-calibration factor (CALFACT[6:0]); non-zero confirms calibration ran. */
uint32_t Reg_GetCalFact(void) { return (hadc1.Instance->CALFACT & 0x7Fu); }

/* ---- live ADC channel select + averaged readout --------------------------*/
/* Switch the live measurement channel. PA0 = current sensor (production),
   PB1 = Arduino A3 (bench test). Re-arms acquisition and re-zeroes. */
void Reg_SetAdcChannel(uint32_t chan)
{
  out_mode_t saved = g_mode;
  g_mode = OUT_OFF; apply_output(0);
  g_adc_chan = chan;
  adc_start_live(chan);            /* configures the analog pin for this channel */
  g_mode = saved;
}

uint32_t Reg_GetAdcChannel(void) { return g_adc_chan; }
uint16_t Reg_GetRawAvg(void)     { return g_raw_avg; }
/* Voltage at the ADC pin from the moving-average raw and the measured VDDA [mV]. */
float    Reg_GetPinVoltage_mV(void) { return (float)g_raw_avg * (g_vdda_mV / ADC_FULL_SCALE); }
uint16_t Reg_GetTorqueRawAvg(void)  { return g_torque_raw_avg; }
float    Reg_GetTorquePinVoltage_mV(void) { return (float)g_torque_raw_avg * (g_vdda_mV / ADC_FULL_SCALE); }

/* ---- setters --------------------------------------------------------------*/
void Reg_SetMode(out_mode_t m)
{
  if (m == OUT_OFF)
  {
    g_integ_q8 = 0;
    g_err_z_q8 = 0;
    g_m_integ_q8 = 0;
    g_m_abs_z_q8 = 0;
    g_m_req_q8 = 0;
    g_tdir = TDIR_ZERO;
    g_dir_cnt = 0;
  }
  g_mode = m;
}

void Reg_SetSetpoint_mA(float v)
{
  if (v < 0.0f) v = 0.0f;
  if (v > g_imax_mA) v = g_imax_mA;
  g_setpoint_mA = v;
  g_setpoint_q8 = MA_TO_Q8(v);
  g_m_integ_q8 = 0;
  g_m_req_q8 = 0;
  g_mode = OUT_REG;
  cfg_save();
}

void Reg_SetTorqueSetpoint_Nm(float v)
{
  if (v < 0.0f) v = 0.0f;
  if (v > g_mmax_Nm) v = g_mmax_Nm;
  g_m_setpoint_Nm = v;
  g_M_set_q8 = NM_TO_Q8(v);
  g_iff_q8   = ff_lookup_q8(g_M_set_q8);  /* table feedforward for this setpoint */
  g_integ_q8 = 0;
  g_err_z_q8 = 0;
  g_m_integ_q8 = 0;                       /* trim starts at 0 (FF is the base)   */
  g_m_abs_z_q8 = (g_M_q8 < 0) ? -g_M_q8 : g_M_q8;
  g_m_req_q8 = 0;
  g_tdir = TDIR_ZERO; g_dir_cnt = 0;      /* re-arm: wait for the motor to push  */
  g_mode = OUT_TORQUE;
}

void Reg_SetManualPct(float pct)
{
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  g_manual_ccr = (uint32_t)(pct * 0.01f * (float)PWM_ARR);
  if (g_manual_ccr > g_dmax_ccr) g_manual_ccr = g_dmax_ccr;
  g_integ_q8 = 0;
  g_err_z_q8 = 0;
  g_m_integ_q8 = 0;
  g_m_abs_z_q8 = 0;
  g_m_req_q8 = 0;
  g_mode = OUT_MANUAL;
}

void Reg_SetKp(float v)       { g_kp = v; sync_fixed_params(); cfg_save(); }
void Reg_SetKi(float v)       { g_ki = v; sync_fixed_params(); g_integ_q8 = 0; cfg_save(); }
void Reg_SetMkP(float v)      { g_mkp = v; sync_fixed_params(); cfg_save(); }
void Reg_SetMkI(float v)      { g_mki = v; sync_fixed_params(); g_m_integ_q8 = 0; g_m_req_q8 = 0; cfg_save(); }
void Reg_SetMkD(float v)      { g_mkd = v; sync_fixed_params(); g_m_abs_z_q8 = (g_M_q8 < 0) ? -g_M_q8 : g_M_q8; cfg_save(); }
void Reg_SetImax_mA(float v)  { if (v < 0.0f) v = 0.0f; g_imax_mA = v; cfg_save(); }
void Reg_SetMmax_Nm(float v)  { if (v < 0.0f) v = 0.0f; g_mmax_Nm = v; if (g_m_setpoint_Nm > v) Reg_SetTorqueSetpoint_Nm(v); cfg_save(); }
void Reg_SetDmaxPct(float v)  { if (v < 0.0f) v = 0.0f; if (v > 100.0f) v = 100.0f; g_dmax_pct = v; sync_fixed_params(); cfg_save(); }

/* ---- torque-direction detector params (not persisted; reset to defaults) --*/
void Reg_SetTon_Nm(float v)
{
  if (v < 0.0f) v = 0.0f;
  g_ton_q8 = NM_TO_Q8(v);
  if (g_toff_q8 > g_ton_q8) g_toff_q8 = g_ton_q8;   /* keep Toff <= Ton (hysteresis) */
}
void Reg_SetToff_Nm(float v)
{
  if (v < 0.0f) v = 0.0f;
  int32_t t = NM_TO_Q8(v);
  if (t > g_ton_q8) t = g_ton_q8;
  g_toff_q8 = t;
}
void Reg_SetTdeb(uint16_t n) { if (n < 1u) n = 1u; g_tdeb = n; }

float    Reg_GetTon_Nm(void)  { return Q8_TO_FLOAT(g_ton_q8);  }
float    Reg_GetToff_Nm(void) { return Q8_TO_FLOAT(g_toff_q8); }
uint16_t Reg_GetTdeb(void)    { return g_tdeb; }
int8_t   Reg_GetTorqueDir(void)      { return g_tdir; }
float    Reg_GetFeedforward_mA(void) { return Q8_TO_FLOAT(g_iff_q8); }

/* ---- feedforward I(T) table API (CLI 'ff') --------------------------------*/
/* Insert/replace a point (kept sorted by torque). Recomputes the active
   feedforward. Returns false if the table is full. Modifies the table from the
   CLI context only; the ISR never reads the arrays (only the precomputed
   g_iff_q8), so no locking is needed. */
bool Reg_FfAddPoint(float nm, float ma)
{
  if (nm < 0.0f) nm = 0.0f;
  if (ma < 0.0f) ma = 0.0f;
  int32_t t = NM_TO_Q8(nm), i = MA_TO_Q8(ma);
  uint8_t k;
  for (k = 0u; k < g_ff_n; k++)
  {
    if (g_ff_t_q8[k] == t) { g_ff_i_q8[k] = i; g_iff_q8 = ff_lookup_q8(g_M_set_q8); return true; }
    if (g_ff_t_q8[k] >  t) break;
  }
  if (g_ff_n >= FF_MAX_PTS) return false;
  for (uint8_t j = g_ff_n; j > k; j--) { g_ff_t_q8[j] = g_ff_t_q8[j-1]; g_ff_i_q8[j] = g_ff_i_q8[j-1]; }
  g_ff_t_q8[k] = t; g_ff_i_q8[k] = i; g_ff_n++;
  g_iff_q8 = ff_lookup_q8(g_M_set_q8);
  return true;
}
void    Reg_FfClear(void)   { g_ff_n = 0u; g_iff_q8 = ff_lookup_q8(g_M_set_q8); }
void    Reg_FfDefault(void) { ff_set_default(); g_iff_q8 = ff_lookup_q8(g_M_set_q8); }
uint8_t Reg_FfCount(void)   { return g_ff_n; }
bool    Reg_FfGetPoint(uint8_t idx, float *nm, float *ma)
{
  if (idx >= g_ff_n) return false;
  if (nm) *nm = Q8_TO_FLOAT(g_ff_t_q8[idx]);
  if (ma) *ma = Q8_TO_FLOAT(g_ff_i_q8[idx]);
  return true;
}

/* ADC trigger advance before the ON-pulse center, in timer counts (1 = 15.6 ns).
   Used for long pulses; short pulses always sample at the center. */
void     Reg_SetTrigAdv(uint32_t cnt) { if (cnt < 1u) cnt = 1u; if (cnt > PWM_ARR - 1u) cnt = PWM_ARR - 1u; g_trig_adv = cnt; }
uint32_t Reg_GetTrigAdv(void)         { return g_trig_adv; }
bool     Reg_GetMeasInOff(void)       { return g_meas_in_off; }  /* true: sampling OFF center */

/* Set the two filter windows (raw ADC samples / current mA samples). Clamped to
   the buffer maxima; resets the running averages. The ADC ISR is masked during
   the reset so the running sums can't desync from their buffers. */
void Reg_SetFilter(uint8_t raw_win, uint8_t ma_win)
{
  if (raw_win < 1u) raw_win = 1u; else if (raw_win > MA_MAX)  raw_win = MA_MAX;
  if (ma_win  < 1u) ma_win  = 1u; else if (ma_win  > IMA_MAX) ma_win  = IMA_MAX;

  HAL_NVIC_DisableIRQ(DMA1_Channel2_3_IRQn);          /* pause the ADC conv-cplt callback */
  g_ma_win = raw_win; g_ima_win = ma_win;
  g_ma_shift = pow2_shift_u8(raw_win);
  g_ima_shift = pow2_shift_u8(ma_win);
  for (uint16_t k = 0; k < MA_MAX;  k++) g_ma_buf[k]  = 0;
  for (uint16_t k = 0; k < IMA_MAX; k++) g_ima_buf[k] = 0;
  g_ma_idx = 0; g_ma_sum = 0; g_ma_cnt = 0; g_raw_avg = 0;
  g_ima_idx = 0; g_ima_sum_q8 = 0; g_ima_cnt = 0;
  g_mma_idx = 0; g_mma_sum_q8 = 0; g_mma_cnt = 0; g_torque_raw_avg = 0;
  for (uint16_t k = 0; k < TORQUE_MEDIAN_N; k++) g_m_med_buf[k] = 0;
  g_m_med_idx = 0; g_m_med_cnt = 0;
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
}
uint8_t Reg_GetRawWin(void) { return g_ma_win; }
uint8_t Reg_GetMaWin(void)  { return g_ima_win; }

/* Button (brake) toggle: OFF -> recalibrate sensor zero, then run regulator;
   ON -> stop. The zero calibration runs with the output forced to 0 (no
   current), so the sensor is always re-zeroed right before each start. */
void Reg_ToggleOutput(void)
{
  if (g_mode == OUT_REG)
  {
    g_mode = OUT_OFF;
    g_integ_q8 = 0; g_err_z_q8 = 0;
    g_m_req_q8 = 0;
  }
  else
  {
    g_integ_q8 = 0; g_err_z_q8 = 0;
    g_m_integ_q8 = 0;
    g_m_abs_z_q8 = 0;
    g_m_req_q8 = 0;
    g_cal_sum_q8 = 0; g_m_cal_sum_q8 = 0; g_cal_cnt = 0; g_cal_done = false; g_I_q8 = 0; g_M_q8 = 0;  /* re-zero first */
    g_tdir = TDIR_ZERO; g_dir_cnt = 0;
    g_mode = OUT_REG;
  }
}

void Reg_Recalibrate(void)
{
  g_mode = OUT_OFF;
  g_cal_done = false; g_cal_cnt = 0; g_cal_sum_q8 = 0; g_m_cal_sum_q8 = 0; g_I_q8 = 0; g_M_q8 = 0;
  g_integ_q8 = 0; g_err_z_q8 = 0; g_m_integ_q8 = 0; g_m_abs_z_q8 = 0; g_m_req_q8 = 0;
  g_tdir = TDIR_ZERO; g_dir_cnt = 0;
}

/* ---- getters --------------------------------------------------------------*/
out_mode_t Reg_GetMode(void)        { return g_mode; }
float      Reg_GetCurrent_mA(void)  { return Q8_TO_FLOAT(g_I_q8); }
float      Reg_GetSetpoint_mA(void) { return (g_mode == OUT_TORQUE) ? Q8_TO_FLOAT(g_setpoint_q8) : g_setpoint_mA; }
float      Reg_GetTorque_Nm(void)   { return Q8_TO_FLOAT(g_M_q8); }
float      Reg_GetTorqueAbs_Nm(void) { int32_t m = g_M_q8; return Q8_TO_FLOAT((m < 0) ? -m : m); }
float      Reg_GetTorqueSetpoint_Nm(void) { return g_m_setpoint_Nm; }
uint32_t   Reg_GetDutyPct(void)     { return (g_ccr * 100u) / PWM_ARR; }
float      Reg_GetKp(void)          { return g_kp; }
float      Reg_GetKi(void)          { return g_ki; }
float      Reg_GetMkP(void)         { return g_mkp; }
float      Reg_GetMkI(void)         { return g_mki; }
float      Reg_GetMkD(void)         { return g_mkd; }
float      Reg_GetZero_mA(void)     { return Q8_TO_FLOAT(g_zero_q8); }
float      Reg_GetImax_mA(void)     { return g_imax_mA; }
float      Reg_GetMmax_Nm(void)     { return g_mmax_Nm; }
float      Reg_GetDmaxPct(void)     { return g_dmax_pct; }
bool       Reg_IsCalDone(void)      { return g_cal_done; }
