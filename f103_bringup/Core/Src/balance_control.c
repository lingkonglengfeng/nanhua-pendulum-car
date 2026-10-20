#include "balance_control.h"
#include <math.h>

#define DEG_PER_RAD 57.2957795131f
#define BAL_MAX_GAP_MS 15U
#define BAL_GUARD_WHEELS 4U
#define BAL_SPEED_TAU_S 0.100f

volatile uint8_t g_bal_enable=0U, g_bal_state=BAL_OFF, g_bal_reason=BAL_REASON_NONE;
/* Latest commissioning baseline, duty/rad and duty/(rad/s).
 * Rod changed to solid iron: these are starting values, not validated tuning. */
volatile float g_bal_kp=8.4f, g_bal_ki=0.0f, g_bal_kd=0.17f, g_bal_integral=0.0f;
volatile float g_bal_theta=0.0f, g_bal_theta_dot=0.0f, g_bal_output=0.0f;
volatile float g_bal_entry_deg=8.0f, g_bal_exit_deg=30.0f, g_bal_limit=0.80f;
/* User observed the cart driving away from a forward-leaning rod; invert PID. */
volatile int8_t g_bal_sign=-1;
volatile uint32_t g_bal_hold_ms=0U, g_bal_dwell_ms=400U, g_bal_fault=0U;
/* max_rpm=0 explicitly disables the overspeed cutoff; boot remains 200 RPM. */
volatile float g_bal_max_rpm=200.0f, g_bal_guard_rpm=0.0f;
/* 0 disables only the commissioning timer; angle/sensor/speed checks remain. */
volatile uint32_t g_bal_max_run_ms=0U, g_bal_run_ms=0U, g_bal_overspeed_ms=0U;
/* Configured on boot; motors remain OFF until ZERO and top capture succeed. */
volatile uint8_t g_bal_vel_enable=1U;
/* Saved UART tuning trial (2026-10-19), not a validated stationary hold. */
volatile float g_bal_vel_kp=0.040f, g_bal_vel_ki=0.0015f, g_bal_vel_limit_deg=3.0f;
volatile float g_bal_vel_rpm=0.0f, g_bal_vel_integral_deg=0.0f, g_bal_ref_deg=0.0f;
volatile float g_bal_vel_slew_deg_s=10.0f;
/* Degrees relative to runtime ZERO; verify again if the reference changes. */
volatile float g_bal_trim_deg=-0.81f;
volatile float g_bal_robust_rpm=0.0f;
static float s_mean_rpm;
static uint32_t s_hold_started, s_previous_ms;
static uint32_t s_active_started, s_overspeed_started;
static uint8_t s_overspeed_tracking;
static uint8_t s_has_previous;
static volatile BalanceExitEvent s_exit_event;
static void motion_reset(void);
static uint8_t configuration_valid(void);
static uint8_t swing_configuration_valid(void);

uint8_t Balance_PeekExit(BalanceExitEvent *event)
{
  if(!s_exit_event.pending) return 0U;
  *event=s_exit_event;
  return 1U;
}

void Balance_AckExit(uint32_t ms)
{
  if(s_exit_event.pending && s_exit_event.ms==ms) s_exit_event.pending=0U;
}

static void reset_velocity(void)
{
  s_mean_rpm=0.0f;
  g_bal_vel_rpm=0.0f; g_bal_vel_integral_deg=0.0f; g_bal_ref_deg=0.0f;
  g_bal_robust_rpm=0.0f;
  motion_reset();
}

static float clamp_symmetric(float value, float limit)
{
  if (value>limit) return limit;
  if (value<-limit) return -limit;
  return value;
}

#include "balance_motion.inc"
#include "balance_swing.inc"

void Balance_Stop(uint8_t reason)
{
  if(g_bal_state==BAL_ACTIVE || g_bal_state==BAL_SWING)
  {
    if(!s_exit_event.pending)
    {
      s_exit_event.ms=s_previous_ms;
      s_exit_event.run_ms=s_previous_ms-s_active_started;
      s_exit_event.overspeed_ms=g_bal_overspeed_ms;
      s_exit_event.peak_rpm=g_bal_guard_rpm; s_exit_event.max_rpm=g_bal_max_rpm;
      s_exit_event.reason=reason; s_exit_event.coalesced=0U;
      /* angle_deg is the latest raw guard angle, frozen while pending. */
      s_exit_event.pending=1U;
    }
    else if(s_exit_event.coalesced<65535U) s_exit_event.coalesced++;
  }
  g_bal_enable=0U;
  if(g_bal_state==BAL_SWING) g_swing_phase=4U;
  g_bal_state=BAL_OFF;
  g_bal_reason=reason;
  g_bal_output=0.0f;
  g_bal_integral=0.0f;
  g_bal_hold_ms=0U;
  s_has_previous=0U;
  s_overspeed_tracking=0U;
  reset_velocity();
  /* Retain run/guard observations for Watch after a cutoff. */
}

static void trip(uint8_t reason)
{
  Balance_Stop(reason);
  g_bal_state=BAL_TRIPPED;
  g_bal_fault++;
}

void Balance_BeginZero(void)
{
  Balance_Stop(BAL_REASON_NONE);
  g_swing_phase=0U;
  g_bal_run_ms=0U; g_bal_guard_rpm=0.0f; g_bal_overspeed_ms=0U;
  g_bal_state=BAL_ZEROING;
}

void Balance_ZeroFailed(void) { trip(BAL_REASON_ZERO); }

uint8_t Balance_Arm(uint8_t calibrated)
{
  if (calibrated == 0U) return 0U;
  Balance_Stop(BAL_REASON_NONE);
  g_swing_phase=0U;
  g_bal_run_ms=0U; g_bal_guard_rpm=0.0f; g_bal_overspeed_ms=0U;
  g_bal_position_rev=0.0f;
  g_bal_state=BAL_WAIT_TOP;
  return 1U;
}

static uint8_t configuration_valid(void)
{
  /* Ordered comparisons reject NaN/infinity as well as unreasonable Watch edits. */
  return (g_bal_kp>=0.0f && g_bal_kp<=100.0f &&
          g_bal_ki>=0.0f && g_bal_ki<=100.0f &&
          g_bal_kd>=0.0f && g_bal_kd<=100.0f &&
          g_bal_entry_deg>=0.1f && g_bal_entry_deg<=45.0f &&
          g_bal_exit_deg>=g_bal_entry_deg && g_bal_exit_deg<=45.0f &&
          g_bal_limit>0.0f && g_bal_limit<=1.0f &&
          g_bal_integral>=-1.0f && g_bal_integral<=1.0f &&
          g_bal_dwell_ms>=400U && g_bal_dwell_ms<=5000U &&
          (g_bal_max_rpm==0.0f || (g_bal_max_rpm>=10.0f && g_bal_max_rpm<=280.0f)) &&
          (g_bal_max_run_ms==0U || (g_bal_max_run_ms>=500U && g_bal_max_run_ms<=60000U)) &&
          g_bal_vel_enable<=1U &&
          g_bal_vel_kp>=0.0f && g_bal_vel_kp<=0.05f &&
          g_bal_vel_ki>=0.0f && g_bal_vel_ki<=0.01f &&
          g_bal_vel_limit_deg>=0.1f && g_bal_vel_limit_deg<=3.0f &&
          g_bal_vel_integral_deg>=-3.0f && g_bal_vel_integral_deg<=3.0f &&
          g_bal_ref_deg>=-3.0f && g_bal_ref_deg<=3.0f &&
          g_bal_vel_rpm>=-100000.0f && g_bal_vel_rpm<=100000.0f &&
          g_bal_vel_slew_deg_s>=0.1f && g_bal_vel_slew_deg_s<=10.0f &&
          g_bal_trim_deg>=-1.0f && g_bal_trim_deg<=1.0f &&
          (g_bal_vel_enable ? g_bal_vel_limit_deg : 0.0f) +
            ((g_bal_trim_deg<0.0f) ? -g_bal_trim_deg : g_bal_trim_deg)<=g_bal_exit_deg*0.5f &&
          (g_bal_sign==1 || g_bal_sign==-1) && motion_configuration_valid() && swing_configuration_valid());
}

static void enter_balance(uint32_t now_ms)
{
  g_bal_state=BAL_ACTIVE; g_bal_enable=1U;g_bal_integral=0.0f;
  s_active_started=now_ms;g_bal_run_ms=0U;
  g_bal_overspeed_ms=0U;s_overspeed_tracking=0U;
  reset_velocity();g_bal_position_rev=0.0f;
  if(g_bal_vel_enable) (void)Balance_HoldPosition(1U);
}

static void update_velocity(float dt, uint32_t now)
{
  float increment, integral, demand, reference, step, speed_error;
  if (!g_bal_vel_enable)
  {
    reset_velocity();
    return;
  }
  g_bal_vel_rpm+=dt/(BAL_SPEED_TAU_S+dt)*(s_mean_rpm-g_bal_vel_rpm);
  motion_update(dt,now);
  speed_error=g_bal_vel_rpm-g_bal_vel_command_rpm;
  if (g_bal_vel_ki==0.0f) g_bal_vel_integral_deg=0.0f;
  increment=g_bal_vel_ki*speed_error*dt;
  integral=clamp_symmetric(g_bal_vel_integral_deg+increment,g_bal_vel_limit_deg);
  demand=g_bal_vel_kp*speed_error+integral;
  /* The reference contributes +Kp*(Kv*v+Iv) to logical motor output for
   * either angle sign. Freeze outward integration at reference/drive limits. */
  if (!((demand>g_bal_vel_limit_deg && increment>0.0f) ||
        (demand<-g_bal_vel_limit_deg && increment<0.0f) ||
        (g_bal_output>=g_bal_limit && increment>0.0f) ||
        (g_bal_output<=-g_bal_limit && increment<0.0f)))
    g_bal_vel_integral_deg=integral;
  demand=clamp_symmetric(g_bal_vel_kp*speed_error+g_bal_vel_integral_deg,
                         g_bal_vel_limit_deg);
  /* Cart-pole speed feedback must first tilt the pole opposite the travel.
   * Directly subtracting speed from motor duty gives the wrong outer polarity.
   * Physical speed/angle polarity still requires an on-cart commissioning run. */
  reference=-(float)g_bal_sign*demand;
  step=g_bal_vel_slew_deg_s*dt;
  g_bal_ref_deg+=clamp_symmetric(reference-g_bal_ref_deg,step);
  g_bal_ref_deg=clamp_symmetric(g_bal_ref_deg,g_bal_vel_limit_deg);
}

void Balance_Update(float theta, float rate, float guard_theta, uint8_t valid,
                    uint8_t calibrated, uint8_t conflict, uint32_t now_ms)
{
  uint32_t elapsed=now_ms-s_previous_ms;
  float abs_deg=guard_theta*DEG_PER_RAD;
  float integral, u, error;
  /* Set the observation timestamp before any early-exit check. elapsed above
   * still measures from the prior cycle, preserving timing/dwell semantics. */
  s_previous_ms=now_ms;
  if(!s_exit_event.pending) s_exit_event.angle_deg=guard_theta*DEG_PER_RAD;
  g_bal_theta=theta;
  g_bal_theta_dot=rate;
  if (abs_deg<0.0f) abs_deg=-abs_deg;
  /* Writing enable=1 cannot bypass arming/dwell, including from Watch. */
  g_bal_enable=(g_bal_state==BAL_ACTIVE || g_bal_state==BAL_SWING) ? 1U : 0U;
  if (g_bal_state==BAL_OFF || g_bal_state==BAL_ZEROING || g_bal_state==BAL_TRIPPED)
  {
    g_bal_enable=0U; g_bal_output=0.0f; return;
  }
  if (!configuration_valid()) { trip(BAL_REASON_CONFIG); return; }
  if (conflict) { trip(BAL_REASON_CONFLICT); return; }
  if (!calibrated) { trip(BAL_REASON_ZERO); return; }
  valid=valid && theta>=-3.142f && theta<=3.142f &&
        guard_theta>=-3.142f && guard_theta<=3.142f && rate>=-200.0f && rate<=200.0f;
  if(g_bal_state==BAL_SWING)
  {
    float caught_speed=g_bal_robust_rpm;
    float capture_deg=g_swing_capture_deg;
    if(capture_deg>g_bal_entry_deg) capture_deg=g_bal_entry_deg;
    if(g_swing_phase!=1U && g_swing_phase!=2U) {trip(BAL_REASON_CONFIG);return;}
    if(!valid) {trip(BAL_REASON_SENSOR);return;}
    if(!s_has_previous || !elapsed || elapsed>BAL_MAX_GAP_MS) {trip(BAL_REASON_TIMING);return;}
    g_bal_run_ms=now_ms-s_active_started;
    if(g_bal_run_ms>=g_swing_max_ms) {trip(BAL_REASON_RUN_TIME);return;}
    if(motion_abs(g_bal_position_rev)>=g_swing_travel_rev) {trip(BAL_REASON_TRAVEL);return;}
    /* Manual capture may be wide for hand support. Automatic transfer must
     * also bound the filtered PID angle to avoid an 8-degree/full-duty jump. */
    if(abs_deg<=capture_deg && motion_abs(theta)*DEG_PER_RAD<=capture_deg &&
       motion_abs(rate)<=g_swing_capture_rate &&
       motion_abs(guard_theta+rate*0.040f)*DEG_PER_RAD<=capture_deg &&
       g_bal_guard_rpm<=90.0f && (g_bal_max_rpm==0.0f || g_bal_guard_rpm<=g_bal_max_rpm))
    {
      enter_balance(now_ms);g_swing_phase=3U;
      /* Do not pretend a moving cart was stationary at automatic capture. */
      g_bal_vel_rpm=caught_speed;g_bal_robust_rpm=caught_speed;s_mean_rpm=caught_speed;
    }
    else {swing_update(theta,rate,(float)elapsed*0.001f);return;}
  }
  if (g_bal_state==BAL_ACTIVE)
  {
    if (!valid) { trip(BAL_REASON_SENSOR); return; }
    if (!s_has_previous || elapsed==0U || elapsed>BAL_MAX_GAP_MS)
      { trip(BAL_REASON_TIMING); return; }
    if (abs_deg>g_bal_exit_deg) { trip(BAL_REASON_ANGLE); return; }
    g_bal_run_ms=now_ms-s_active_started;
    if (g_bal_max_run_ms && g_bal_run_ms>=g_bal_max_run_ms)
      { trip(BAL_REASON_RUN_TIME); return; }
  }
  else
  {
    if (!valid || abs_deg>g_bal_entry_deg ||
        (s_has_previous && (elapsed==0U || elapsed>BAL_MAX_GAP_MS)))
    {
      g_bal_state=BAL_WAIT_TOP; g_bal_hold_ms=0U;
      s_previous_ms=now_ms; s_has_previous=1U; return;
    }
    if (g_bal_state==BAL_WAIT_TOP)
    {
      g_bal_state=BAL_HOLD_TOP; s_hold_started=now_ms;
    }
    g_bal_hold_ms=now_ms-s_hold_started;
    if (g_bal_hold_ms>=g_bal_dwell_ms)
    {
      enter_balance(now_ms);
    }
  }
  s_previous_ms=now_ms; s_has_previous=1U;
  if (g_bal_state!=BAL_ACTIVE) { g_bal_output=0.0f; return; }
  update_velocity((float)elapsed*0.001f,now_ms);
  error=theta-(g_bal_ref_deg+g_bal_trim_deg)/DEG_PER_RAD;
  if (g_bal_ki==0.0f) g_bal_integral=0.0f;
  integral=g_bal_integral+g_bal_ki*error*((float)elapsed*0.001f);
  if (integral>g_bal_limit) integral=g_bal_limit;
  if (integral<-g_bal_limit) integral=-g_bal_limit;
  u=g_bal_kp*error+g_bal_kd*rate+integral;
  if (!((u>g_bal_limit && g_bal_ki*error>0.0f) ||
        (u<-g_bal_limit && g_bal_ki*error<0.0f))) g_bal_integral=integral;
  u=g_bal_kp*error+g_bal_kd*rate+g_bal_integral;
  if (u>g_bal_limit) u=g_bal_limit;
  if (u<-g_bal_limit) u=-g_bal_limit;
  g_bal_output=(float)g_bal_sign*u;
}

void Balance_CheckMotion(const volatile float *wheel_rpm, uint32_t now_ms)
{
  Balance_CheckMotionFeedback(wheel_rpm,wheel_rpm,now_ms);
}

void Balance_CheckMotionFeedback(const volatile float *wheel_rpm,
                                const volatile float *feedback_rpm, uint32_t now_ms)
{
  uint8_t wheel;
  float peak=0.0f, mean=0.0f, sorted[BAL_GUARD_WHEELS];
  if ((g_bal_state!=BAL_ACTIVE && g_bal_state!=BAL_SWING) || !g_bal_enable) return;
  for (wheel=0U; wheel<BAL_GUARD_WHEELS; wheel++)
  {
    float rpm=wheel_rpm[wheel];
    /* Ordered bounds reject NaN/infinity, including invalid Watch edits. */
    float feedback=feedback_rpm[wheel];
    if (!(rpm>=-100000.0f && rpm<=100000.0f &&
          feedback>=-100000.0f && feedback<=100000.0f))
      { trip(BAL_REASON_SENSOR); return; }
    sorted[wheel]=feedback;
    mean+=feedback*0.25f;
    if (rpm<0.0f) rpm=-rpm;
    if (rpm>peak) peak=rpm;
  }
  /* In straight balance, a lone encoder
   * burst must not masquerade as chassis translation. Preserve all raw RPMs
   * and their absolute peak for logging and the independent speed cutoff.
   * Two bad/slipping wheels can still bias this estimator; diagnose raw data. */
  for (wheel=1U; wheel<BAL_GUARD_WHEELS; wheel++)
  {
    uint8_t j=wheel;
    float value=sorted[wheel];
    while (j>0U && sorted[j-1U]>value)
    {
      sorted[j]=sorted[j-1U]; j--;
    }
    sorted[j]=value;
  }
  /* Previous-cycle speed is used by the angle calculation: at most 5ms old
   * at the nominal cadence. Speed cutoff still uses this cycle's fresh peak. */
  /* During spin the two sides intentionally differ. A middle-pair estimate
   * changes wheel weights and can hide real mean travel when front/rear
   * responses differ. Match the four-wheel arithmetic odometry weights. */
  g_bal_robust_rpm=(g_bal_seq_state==BAL_SEQ_SPIN || g_bal_state==BAL_SWING) ? mean : (sorted[1]+sorted[2])*0.5f;
  s_mean_rpm=g_bal_robust_rpm;
  g_bal_guard_rpm=peak;
  if (g_bal_max_rpm>0.0f && peak>g_bal_max_rpm)
  {
    if (!s_overspeed_tracking)
    {
      s_overspeed_tracking=1U; s_overspeed_started=now_ms;
    }
    g_bal_overspeed_ms=now_ms-s_overspeed_started;
    if (g_bal_overspeed_ms>=BAL_OVERSPEED_DWELL_MS) trip(BAL_REASON_SPEED);
  }
  else
  {
    s_overspeed_tracking=0U; g_bal_overspeed_ms=0U;
  }
  if (g_bal_state==BAL_ACTIVE && g_bal_enable) turn_update(feedback_rpm,now_ms);
}
