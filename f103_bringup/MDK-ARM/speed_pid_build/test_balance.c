/* Host tests compile the production angle and balance modules unchanged. */
#include "balance_control.h"
#include "pendulum_angle.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#define RAD 0.017453292519943295f

static void sample(float deg, uint8_t valid, uint32_t ms)
{ Balance_Update(deg*RAD,0.0f,deg*RAD,valid,1U,0U,ms); }
static void arm_at(uint32_t now)
{
  assert(Balance_Arm(1U)); sample(0.0f,1U,now);
  assert(g_bal_state==BAL_HOLD_TOP && !g_bal_enable);
}
static void hold_until(uint32_t start, uint32_t duration)
{
  uint32_t t;
  for(t=5;t<=duration;t+=5) sample(0.0f,1U,start+t);
}

int main(void)
{
  PendulumAngle p;
  uint32_t t;
  uint8_t wheel;
  float rpm[4]={0.0f,0.0f,0.0f,0.0f};
  assert(g_bal_sign == -1); /* Current cart needs inverted upright PID output. */
  /* Boot baseline survives power cycling, but never bypasses ZERO/arming. */
  assert(g_bal_state==BAL_OFF && !g_bal_enable && g_bal_output==0.0f);
  assert(!Balance_Arm(0U) && g_bal_state==BAL_OFF);
  assert(g_bal_kp==8.4f && g_bal_ki==0.0f && g_bal_kd==0.17f);
  assert(g_bal_entry_deg==8.0f && g_bal_exit_deg==30.0f);
  assert(g_bal_limit==0.80f && g_bal_dwell_ms==400U);
  assert(g_bal_max_rpm==200.0f && g_bal_max_run_ms==0U);
  assert(g_bal_vel_enable==1U && g_bal_vel_kp==0.040f && g_bal_vel_ki==0.0015f);
  assert(g_bal_vel_limit_deg==3.0f && g_bal_vel_slew_deg_s==10.0f && g_bal_trim_deg==-0.81f);
  assert(PENDULUM_ZERO_STORED==0U);
  arm_at(100U); hold_until(100U,395U); assert(!g_bal_enable);
  sample(0.0f,1U,500U);
  assert(g_bal_enable && fabsf(g_bal_output+8.4f*0.81f*RAD)<1e-6f);
  Balance_Stop(BAL_REASON_STOP);
  /* Explicit fixtures below retain the original algorithm/guard regressions. */
  g_bal_trim_deg=0.0f;
  g_bal_kp=1.2f; g_bal_ki=0.0f; g_bal_kd=0.08f;
  g_bal_entry_deg=5.0f; g_bal_exit_deg=20.0f; g_bal_limit=0.25f; g_bal_vel_enable=0U;
  g_bal_max_rpm=40.0f; g_bal_max_run_ms=1500U;
  /* Verify production derivative response: dt=5ms, tau=10ms -> alpha=1/3. */
  assert(fabsf(PENDULUM_RATE_TAU_S-0.010f)<1e-7f);
  PendulumAngle_Init(&p,2000U,1,1U);
  PendulumAngle_Update(&p,2000U,0x27U,1U,5U);
  PendulumAngle_Update(&p,2001U,0x27U,1U,10U);
  assert(fabsf(p.rate_rad_s-(6.28318530718f/4096.0f/0.005f/3.0f))<1e-6f);
  PendulumAngle_Init(&p,0U,1,0U);
  assert(!p.calibrated && !Balance_Arm(p.calibrated));
  PendulumAngle_BeginZero(&p);
  /* Cluster crosses raw zero. Top must be 2048, never the integer mean 2048. */
  for(t=1;t<=200;t++) PendulumAngle_Update(&p,(t&1U)?4095U:1U,0x27U,1U,t*5U);
  assert(p.zero_result==PENDULUM_ZERO_SUCCESS && p.calibrated);
  assert(p.bottom_raw==0U && p.upright_raw==2048U && !p.valid);
  PendulumAngle_Update(&p,2048U,0x27U,1U,1005U);
  assert(p.valid && fabsf(p.angle_rad)<1e-6f);
  PendulumAngle_Update(&p,2105U,0x27U,1U,1010U);
  assert(p.angle_rad>0.0f && p.rate_rad_s>0.0f);
  PendulumAngle_SetSign(&p,-1);
  PendulumAngle_Update(&p,2105U,0x27U,1U,1015U); assert(p.angle_rad<0.0f);
  PendulumAngle_BeginZero(&p); assert(!p.calibrated);
  PendulumAngle_Update(&p,100U,0x27U,1U,1020U);
  PendulumAngle_Update(&p,140U,0x27U,1U,1025U);
  assert(p.zero_result==PENDULUM_ZERO_MOVED && !p.calibrated);
  PendulumAngle_BeginZero(&p);
  PendulumAngle_Update(&p,100U,0x37U,1U,1030U);
  assert(p.zero_result==PENDULUM_ZERO_BAD_SAMPLE && !p.calibrated);
  PendulumAngle_BeginZero(&p);
  for(t=1;t<=200;t++) PendulumAngle_Update(&p,3500U,0x27U,1U,1030U+t*5U);
  assert(p.bottom_raw==3500U && p.upright_raw==1452U);
  PendulumAngle_Update(&p,1452U,0x27U,1U,2035U); assert(p.valid && p.angle_rad==0.0f);
  PendulumAngle_CheckFreshness(&p,2070U); assert(!p.valid);

  Balance_Stop(BAL_REASON_STOP); g_bal_enable=1U; sample(0,1,0);
  assert(!g_bal_enable && g_bal_output==0 && g_bal_state==BAL_OFF);
  arm_at(100U); hold_until(100U,395U); assert(!g_bal_enable && g_bal_hold_ms==395U);
  sample(0,1,500U); assert(g_bal_state==BAL_ACTIVE && g_bal_enable);
  sample(19.95f,1,505U); assert(g_bal_enable && g_bal_output<0.0f && g_bal_output>=-0.25f);
  sample(20.04f,1,510U); assert(!g_bal_enable && g_bal_state==BAL_TRIPPED);
  assert(g_bal_reason==BAL_REASON_ANGLE && g_bal_output==0.0f);
  for(t=515U;t<=1200U;t+=5U) sample(0,1,t);
  assert(g_bal_state==BAL_TRIPPED && !g_bal_enable); /* no automatic recapture */

  arm_at(2000U); hold_until(2000U,300U); sample(5.01f,1,2305U);
  assert(g_bal_state==BAL_WAIT_TOP && !g_bal_hold_ms);
  sample(0,1,2310U); hold_until(2310U,395U); assert(!g_bal_enable);
  sample(0,0,2710U); assert(!g_bal_enable && !g_bal_hold_ms);
  sample(0,1,2715U); hold_until(2715U,395U); assert(!g_bal_enable);
  sample(0,1,3115U); assert(g_bal_enable);
  sample(0,0,3120U); assert(g_bal_state==BAL_TRIPPED && g_bal_reason==BAL_REASON_SENSOR);

  arm_at(4000U); hold_until(4000U,300U); sample(0,1,4320U);
  assert(g_bal_state==BAL_WAIT_TOP && !g_bal_hold_ms);
  sample(0,1,4325U); hold_until(4325U,400U); assert(g_bal_enable);
  sample(0,1,4745U); assert(!g_bal_enable && g_bal_reason==BAL_REASON_TIMING);

  arm_at(UINT32_MAX-200U); hold_until(UINT32_MAX-200U,400U);
  assert(g_bal_enable && g_bal_hold_ms==400U); /* wrap-safe milliseconds */
  Balance_Update(0.01f,0.1f,21*RAD,1,1,0,204U);
  assert(g_bal_reason==BAL_REASON_ANGLE); /* raw guard trips despite filtered lag */
  arm_at(6000U); hold_until(6000U,400U);
  Balance_Update(0.01f,0.1f,0.01f,1,1,1,6405U);
  assert(g_bal_reason==BAL_REASON_CONFLICT && !g_bal_enable);
  arm_at(7000U); hold_until(7000U,400U);
  Balance_Update(0.01f,0.1f,0.01f,1,0,0,7405U);
  assert(g_bal_reason==BAL_REASON_ZERO);

  /* Integral antiwindup, live gains, signed output and NaN Watch edits. */
  g_bal_ki=10.0f;
  arm_at(8000U); hold_until(8000U,400U);
  for(t=8405;t<9000;t+=5) sample(19,1,t);
  assert(fabsf(g_bal_integral)<0.00001f && fabsf(g_bal_output+0.25f)<0.00001f);
  g_bal_kp=0; g_bal_kd=0; g_bal_sign=-1;
  sample(1,1,9000U); assert(g_bal_integral>0 && g_bal_output<0);
  g_bal_ki=0; sample(1,1,9005U); assert(g_bal_integral==0 && g_bal_output==0);
  g_bal_kp=NAN; sample(0,1,9010U);
  assert(!g_bal_enable && g_bal_state==BAL_TRIPPED && g_bal_reason==BAL_REASON_CONFIG);
  g_bal_kp=1.2f; g_bal_sign=1;
  arm_at(10000U); hold_until(10000U,400U);
  sample(NAN,1,10405U); assert(g_bal_reason==BAL_REASON_SENSOR && !g_bal_enable);
  Balance_BeginZero(); Balance_ZeroFailed(); assert(g_bal_reason==BAL_REASON_ZERO);

  /* Each wheel, either direction, must cut drive after continuous 200ms.
   * The control cycle calls this guard before submitting motor waveforms. */
  assert(g_bal_max_rpm==40.0f && g_bal_max_run_ms==1500U);
  for(wheel=0U;wheel<4U;wheel++)
  {
    uint32_t start=12000U+(uint32_t)wheel*1000U;
    rpm[wheel]=(wheel&1U)?-41.0f:41.0f;
    arm_at(start); hold_until(start,400U);
    for(t=405U;t<=600U;t+=5U)
    {
      sample(0,1,start+t); Balance_CheckMotion(rpm,start+t);
      assert(g_bal_enable);
    }
    sample(0,1,start+605U); Balance_CheckMotion(rpm,start+605U);
    assert(!g_bal_enable && g_bal_state==BAL_TRIPPED);
    assert(g_bal_reason==BAL_REASON_SPEED && g_bal_output==0.0f);
    assert(g_bal_guard_rpm==41.0f && g_bal_overspeed_ms==200U);
    rpm[wheel]=0.0f;
    sample(0,1,start+610U); Balance_CheckMotion(rpm,start+610U);
    assert(g_bal_state==BAL_TRIPPED); /* no automatic restart */
  }
  arm_at(17000U); hold_until(17000U,400U);
  rpm[0]=80.0f;
  for(t=17405U;t<=17445U;t+=5U)
    { sample(0,1,t); Balance_CheckMotion(rpm,t); assert(g_bal_enable); }
  rpm[0]=40.0f; sample(0,1,17450U); Balance_CheckMotion(rpm,17450U);
  assert(g_bal_enable && !g_bal_overspeed_ms); /* equal limit resets dwell */
  rpm[0]=41.0f;
  for(t=17455U;t<=17650U;t+=5U)
    { sample(0,1,t); Balance_CheckMotion(rpm,t); assert(g_bal_enable); }
  sample(0,1,17655U); Balance_CheckMotion(rpm,17655U);
  assert(g_bal_reason==BAL_REASON_SPEED);
  rpm[0]=0.0f;

  g_bal_max_run_ms=500U;
  arm_at(20000U); hold_until(20000U,400U);
  for(t=20405U;t<=20895U;t+=5U)
    { sample(0,1,t); Balance_CheckMotion(rpm,t); assert(g_bal_enable); }
  sample(0,1,20900U); Balance_CheckMotion(rpm,20900U);
  assert(g_bal_state==BAL_TRIPPED && !g_bal_enable && g_bal_output==0.0f);
  assert(g_bal_reason==BAL_REASON_RUN_TIME && g_bal_run_ms==500U);
  sample(0,1,20905U); assert(g_bal_reason==BAL_REASON_RUN_TIME);

  /* Both guard timestamps remain correct across uint32 tick wrap. */
  arm_at(UINT32_MAX-420U); hold_until(UINT32_MAX-420U,400U);
  rpm[3]=-60.0f;
  for(t=5U;t<=200U;t+=5U)
    { uint32_t now=(UINT32_MAX-20U)+t; sample(0,1,now); Balance_CheckMotion(rpm,now); assert(g_bal_enable); }
  sample(0,1,184U); Balance_CheckMotion(rpm,184U);
  assert(g_bal_reason==BAL_REASON_SPEED && g_bal_overspeed_ms==200U);
  rpm[3]=0.0f;
  arm_at(UINT32_MAX-420U); hold_until(UINT32_MAX-420U,400U);
  for(t=5U;t<500U;t+=5U)
    { uint32_t now=(UINT32_MAX-20U)+t; sample(0,1,now); Balance_CheckMotion(rpm,now); assert(g_bal_enable); }
  sample(0,1,479U); assert(g_bal_reason==BAL_REASON_RUN_TIME && g_bal_run_ms==500U);

  arm_at(30000U); hold_until(30000U,400U);
  rpm[2]=NAN; sample(0,1,30405U); Balance_CheckMotion(rpm,30405U);
  assert(g_bal_reason==BAL_REASON_SENSOR && !g_bal_enable);
  rpm[2]=0.0f;
  g_bal_max_rpm=NAN; assert(Balance_Arm(1U)); sample(0,1,31000U);
  assert(g_bal_reason==BAL_REASON_CONFIG && !g_bal_enable);
  g_bal_max_rpm=40.0f; g_bal_max_run_ms=1U;
  assert(Balance_Arm(1U)); sample(0,1,32000U);
  assert(g_bal_reason==BAL_REASON_CONFIG && !g_bal_enable);
  g_bal_max_run_ms=1500U;
  printf("PASS: solid-rod starting defaults, boot OFF, calibration required and 400ms capture retained.\n");
  printf("PASS: bottom calibration/wrap/stationarity/weak magnet/stale/sign; top=bottom+180 degrees.\n");
  printf("PASS: 395/400 ms, uninterrupted window, invalid/gap reset, raw-angle exit, no recapture, uint32 wrap.\n");
  printf("PASS: sensor/timing/conflict/calibration/config faults, Watch enable bypass blocked, bounded PID/antiwindup.\n");
  printf("PASS: all-wheel bidirectional overspeed cutoff, continuous 200ms/reset, timed cutoff, fault latch and tick wrap.\n");
  return 0;
}
