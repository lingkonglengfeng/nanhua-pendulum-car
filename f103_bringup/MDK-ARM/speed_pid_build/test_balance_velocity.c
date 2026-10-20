/* Production controller tests. The illustrative plant checks feedback polarity,
 * not the actual cart's masses, motor dynamics, friction or achievable gains. */
#include "balance_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#define RAD 0.017453292519943295f

static void angle_sample(float theta, float rate, uint32_t ms)
{ Balance_Update(theta,rate,theta,1U,1U,0U,ms); }

static uint32_t activate(uint32_t start)
{
  uint32_t t;
  float zero[4]={0,0,0,0};
  assert(Balance_Arm(1U));
  for(t=0U;t<=400U;t+=5U) angle_sample(0,0,start+t);
  assert(g_bal_state==BAL_ACTIVE && g_bal_enable);
  Balance_CheckMotion(zero,start+400U);
  return start+400U;
}

static void speed_sample(float rpm, float theta, float rate, uint32_t ms)
{
  float wheels[4]={rpm,rpm,rpm,rpm};
  angle_sample(theta,rate,ms);
  Balance_CheckMotion(wheels,ms);
}

static float illustrative_plant(int8_t sign, uint8_t enabled, uint32_t start)
{
  float theta=0.0f, rate=0.0f, measured_rate=0.0f, velocity=20.0f;
  uint32_t t, active;
  g_bal_sign=sign; g_bal_vel_enable=enabled;
  g_bal_kp=7.05f; g_bal_ki=0.0f; g_bal_kd=0.205f;
  g_bal_vel_kp=0.008f; g_bal_vel_ki=0.002f; g_bal_vel_limit_deg=2.0f;
  active=activate(start);
  for(t=5U;t<=40000U;t+=5U)
  {
    float u, acceleration;
    measured_rate+=0.005f/0.015f*(rate-measured_rate);
    speed_sample(velocity,theta,measured_rate,active+t);
    assert(g_bal_enable);
    assert(fabsf(g_bal_ref_deg)<=2.00001f);
    u=g_bal_output;
    acceleration=25.0f*theta-(float)sign*20.0f*u;
    rate+=acceleration*0.005f;
    theta+=rate*0.005f;
    velocity+=1000.0f*u*0.005f;
    assert(fabsf(theta)<10.0f*RAD);
  }
  return velocity;
}

/* Sensitivity check with a speed-dependent DC motor response, not an identified
 * vehicle. Includes 20ms encoder LP, 10ms angle-rate LP and 10ms angle delay. */
static float motor_lag_plant(int8_t sign, float motor_tau, uint32_t start)
{
  float theta=(float)sign*0.5f*RAD, rate=0, velocity=0, measured_rate=0, measured_v=0;
  float angle_history[3]={theta,theta,theta};
  float peak=0;
  uint32_t t, active=0U;
  g_bal_sign=sign; g_bal_vel_enable=1U;
  g_bal_kp=8.0f; g_bal_ki=0; g_bal_kd=0.205f; g_bal_limit=0.5f;
  g_bal_vel_kp=0.035f; g_bal_vel_ki=0; g_bal_vel_limit_deg=3.0f;
  g_bal_vel_slew_deg_s=10.0f;
  active=activate(start);
  for(t=5U;t<=40000U;t+=5U)
  {
    uint32_t index=(t/5U-1U)%3U;
    float acceleration, angle_used=angle_history[index];
    angle_history[index]=theta;
    measured_v+=0.005f/0.025f*(velocity-measured_v);
    measured_rate+=0.005f/0.015f*(rate-measured_rate);
    speed_sample(measured_v,angle_used,measured_rate,active+t);
    assert(g_bal_enable && fabsf(g_bal_ref_deg)<=3.00001f);
    acceleration=(260.0f*g_bal_output-velocity)/motor_tau;
    rate+=(25.0f*theta-(float)sign*0.0085f*acceleration)*0.005f;
    theta+=rate*0.005f;
    velocity+=acceleration*0.005f;
    if(fabsf(velocity)>peak) peak=fabsf(velocity);
    assert(fabsf(theta)<20*RAD && fabsf(velocity)<120.0f);
  }
  printf("Illustrative motor tau %.2f sign %d: peak %.3f, final %.6f RPM\n",
         motor_tau,(int)sign,peak,velocity);
  return velocity;
}

int main(void)
{
  uint32_t t, active;
  float previous, expected, uncontrolled, controlled, reversed;
  float wheels[4]={20.0f,40.0f,-20.0f,0.0f};
  assert(g_bal_vel_enable==1U && g_bal_vel_kp==0.040f && g_bal_vel_ki==0.0015f);
  assert(g_bal_trim_deg==-0.81f);
  assert(g_bal_vel_slew_deg_s==10.0f && g_bal_vel_limit_deg==3.0f);
  /* These scenarios deliberately select their own gains and disable baseline. */
  g_bal_trim_deg=0.0f;
  g_bal_vel_enable=0U; g_bal_vel_slew_deg_s=1.0f; g_bal_vel_limit_deg=2.0f;
  g_bal_exit_deg=20.0f; /* Explicit legacy raw-angle guard fixture. */
  g_bal_max_rpm=280.0f; g_bal_max_run_ms=60000U;
  g_bal_kp=7.05f; g_bal_kd=0.205f; g_bal_limit=0.5f; g_bal_sign=-1;

  active=activate(1000U);
  speed_sample(100,2*RAD,0.1f,active+5U);
  expected=-(7.05f*2*RAD+0.205f*0.1f);
  assert(fabsf(g_bal_output-expected)<1e-6f && g_bal_ref_deg==0.0f);

  g_bal_vel_enable=1U; g_bal_vel_kp=0.008f; g_bal_vel_ki=0.002f;
  active=activate(2000U); previous=0.0f;
  for(t=5U;t<=1000U;t+=5U)
  {
    angle_sample(0,0,active+t); Balance_CheckMotion(wheels,active+t);
    assert(g_bal_enable && fabsf(g_bal_ref_deg-previous)<=0.00501f);
    previous=g_bal_ref_deg;
  }
  assert(fabsf(g_bal_vel_rpm-10.0f)<0.01f); /* Signed mean, not abs mean. */
  assert(g_bal_ref_deg>0.0f && g_bal_vel_integral_deg>0.0f);
  Balance_Stop(BAL_REASON_STOP);
  assert(!g_bal_enable && !g_bal_output && !g_bal_ref_deg && !g_bal_vel_integral_deg);

  active=activate(4000U);
  for(t=5U;t<=1000U;t+=5U) speed_sample(-20,0,0,active+t);
  assert(g_bal_ref_deg<0.0f && g_bal_vel_integral_deg<0.0f);
  g_bal_sign=1; active=activate(6000U);
  for(t=5U;t<=1000U;t+=5U) speed_sample(20,0,0,active+t);
  assert(g_bal_ref_deg<0.0f && g_bal_vel_integral_deg>0.0f);

  /* Reference saturation rejects outward integration and respects 1deg/s. */
  g_bal_sign=-1; g_bal_vel_kp=0.05f; g_bal_vel_ki=0.01f;
  active=activate(8000U); previous=0.0f;
  for(t=5U;t<=3000U;t+=5U)
  {
    speed_sample(100,0,0,active+t);
    assert(g_bal_enable && fabsf(g_bal_ref_deg)<=2.00001f);
    assert(fabsf(g_bal_ref_deg-previous)<=0.00501f);
    previous=g_bal_ref_deg;
  }
  assert(fabsf(g_bal_ref_deg-2.0f)<0.001f && fabsf(g_bal_vel_integral_deg)<0.15f);
  angle_sample(21*RAD,0,active+3005U);
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_ANGLE && !g_bal_ref_deg);

  g_bal_vel_kp=0.008f; g_bal_vel_ki=0.002f;
  active=activate(13000U);
  speed_sample(10,0,0,active+5U);
  g_bal_vel_integral_deg=0.1f; g_bal_output=g_bal_limit;
  angle_sample(0,0,active+10U);
  assert(fabsf(g_bal_vel_integral_deg-0.1f)<1e-6f); /* Motor saturation freezes I. */
  g_bal_vel_ki=0.0f; angle_sample(0,0,active+15U);
  assert(g_bal_vel_integral_deg==0.0f);
  g_bal_vel_rpm=NAN; angle_sample(0,0,active+20U);
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_CONFIG);
  g_bal_vel_kp=NAN; assert(Balance_Arm(1U)); angle_sample(0,0,15000U);
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_CONFIG);
  g_bal_vel_kp=0.008f; g_bal_vel_enable=2U;
  assert(Balance_Arm(1U)); angle_sample(0,0,16000U);
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_CONFIG);

  /* Outer feedback cannot bypass either commissioning cutoff. */
  g_bal_vel_enable=1U; g_bal_max_rpm=40.0f;
  active=activate(17000U);
  for(t=5U;t<=200U;t+=5U) { speed_sample(41,0,0,active+t); assert(g_bal_enable); }
  speed_sample(41,0,0,active+205U);
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_SPEED && !g_bal_ref_deg);
  g_bal_max_run_ms=500U;
  active=activate(18000U);
  for(t=5U;t<500U;t+=5U) { speed_sample(20,0,0,active+t); assert(g_bal_enable); }
  speed_sample(20,0,0,active+500U);
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_RUN_TIME && !g_bal_ref_deg);
  g_bal_max_rpm=280.0f; g_bal_max_run_ms=60000U;

  uncontrolled=illustrative_plant(-1,0U,20000U);
  controlled=illustrative_plant(-1,1U,70000U);
  reversed=illustrative_plant(1,1U,120000U);
  printf("Illustrative plant final RPM: angle only %.3f, outer PI %.3f, reversed angle %.3f\n",
         uncontrolled,controlled,reversed);
  assert(fabsf(uncontrolled-20.0f)<0.01f);
  assert(fabsf(controlled)<fabsf(uncontrolled)*0.25f);
  assert(fabsf(reversed-controlled)<0.01f);
  assert(g_bal_vel_slew_deg_s==1.0f);
  g_bal_vel_slew_deg_s=10.0f; g_bal_vel_kp=0.05f;
  g_bal_vel_ki=0; g_bal_vel_enable=1U;
  active=activate(170000U); previous=0.0f;
  for(t=5U;t<=200U;t+=5U)
  {
    speed_sample(100,0,0,active+t);
    assert(g_bal_enable && fabsf(g_bal_ref_deg-previous)<=0.05001f);
    previous=g_bal_ref_deg;
  }
  assert(fabsf(g_bal_ref_deg)>1.0f); /* New slew setting actually acts sooner. */
  g_bal_vel_slew_deg_s=NAN;
  angle_sample(0,0,active+205U);
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_CONFIG);
  g_bal_vel_slew_deg_s=0;
  assert(Balance_Arm(1)); angle_sample(0,0,171000U);
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_CONFIG);
  g_bal_vel_slew_deg_s=1;
  assert(fabsf(motor_lag_plant(-1,0.10f,180000U))<1.0f);
  assert(fabsf(motor_lag_plant(-1,0.18f,230000U))<1.0f);
  assert(fabsf(motor_lag_plant(-1,0.25f,280000U))<1.0f);
  assert(fabsf(motor_lag_plant(1,0.18f,330000U))<1.0f);
  /* Trim is independent of sensor calibration, speed reference, and raw guard. */
  assert(g_bal_trim_deg==0.0f);
  g_bal_vel_enable=0U; g_bal_sign=-1; g_bal_kp=8.4f; g_bal_kd=0.15f;
  g_bal_trim_deg=0.1f; active=activate(380000U);
  angle_sample(0,0,active+5U);
  assert(fabsf(g_bal_output-8.4f*0.1f*RAD)<1e-6f && g_bal_theta==0.0f);
  assert(g_bal_ref_deg==0.0f); /* Outer reference semantics unchanged. */
  g_bal_trim_deg=-0.1f; angle_sample(0,0,active+10U);
  assert(fabsf(g_bal_output+8.4f*0.1f*RAD)<1e-6f);
  g_bal_sign=1; angle_sample(0,0,active+15U);
  assert(fabsf(g_bal_output-8.4f*0.1f*RAD)<1e-6f);
  g_bal_trim_deg=1.0f; angle_sample(21*RAD,0,active+20U);
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_ANGLE);
  Balance_Stop(BAL_REASON_STOP); assert(g_bal_trim_deg==1.0f);
  g_bal_trim_deg=NAN; assert(Balance_Arm(1)); angle_sample(0,0,381000U);
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_CONFIG);
  g_bal_trim_deg=1.01f; assert(Balance_Arm(1)); angle_sample(0,0,382000U);
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_CONFIG);
  g_bal_trim_deg=1; g_bal_entry_deg=0.1f; g_bal_exit_deg=1.0f;
  assert(Balance_Arm(1)); angle_sample(0,0,383000U);
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_CONFIG);
  g_bal_trim_deg=0; g_bal_entry_deg=0.5f; g_bal_exit_deg=20;
  g_bal_sign=-1; active=activate(384000U);
  angle_sample(0.01f,0.1f,active+5U);
  assert(fabsf(g_bal_output-(-8.4f*0.01f-0.15f*0.1f))<1e-6f);

  /* Four-wheel middle pair rejects one isolated channel without erasing the
   * raw peak or weakening the independent continuous overspeed cutoff. */
  {
    uint8_t channel;
    float burst[4]={0,0,0,0};
    g_bal_vel_enable=1U; g_bal_vel_kp=0.035f; g_bal_vel_ki=0.0005f;
    g_bal_vel_limit_deg=3.0f; g_bal_vel_slew_deg_s=10.0f;
    g_bal_max_rpm=280.0f; g_bal_max_run_ms=60000U;
    for(channel=0U;channel<4U;channel++)
    {
      uint32_t start=386000U+1000U*channel;
      active=activate(start);
      burst[channel]=319.0f;
      for(t=5U;t<=40U;t+=5U)
      {
        angle_sample(0,0,active+t); Balance_CheckMotion(burst,active+t);
        assert(g_bal_enable && g_bal_guard_rpm==319.0f && g_bal_robust_rpm==0.0f);
        assert(g_bal_vel_rpm==0.0f && g_bal_ref_deg==0.0f && g_bal_output==0.0f);
      }
      burst[channel]=0.0f;
      angle_sample(0,0,active+45U); Balance_CheckMotion(burst,active+45U);
      assert(g_bal_enable && !g_bal_overspeed_ms);
      burst[channel]=-319.0f;
      for(t=50U;t<=245U;t+=5U)
      {
        angle_sample(0,0,active+t); Balance_CheckMotion(burst,active+t);
        assert(g_bal_enable && g_bal_robust_rpm==0.0f && g_bal_output==0.0f);
      }
      angle_sample(0,0,active+250U); Balance_CheckMotion(burst,active+250U);
      assert(!g_bal_enable && g_bal_reason==BAL_REASON_SPEED);
      burst[channel]=0.0f;
    }
    active=activate(391000U);
    for(t=5U;t<=1000U;t+=5U) speed_sample(-20,0,0,active+t);
    assert(g_bal_enable && g_bal_robust_rpm==-20.0f && g_bal_vel_rpm<0 && g_bal_ref_deg<0);
    burst[0]=-21; burst[1]=-19; burst[2]=-20; burst[3]=-1000;
    angle_sample(0,0,active+1005U); Balance_CheckMotion(burst,active+1005U);
    assert(g_bal_robust_rpm==-20.5f && g_bal_guard_rpm==1000.0f);
    assert(g_bal_enable);
    burst[3]=NAN; angle_sample(0,0,active+1010U); Balance_CheckMotion(burst,active+1010U);
    assert(!g_bal_enable && g_bal_reason==BAL_REASON_SENSOR);
  }
  printf("PASS: each channel's signed burst rejected from outer speed, signed DC preserved, raw peak cutoff and invalid sensor trip retained.\n");
  printf("PASS: signed trim output, zero-trim equivalence, persistence, physical angle guard and invalid/config bounds.\n");
  printf("PASS: disabled equivalence, signed mean, both polarities, reset, reference slew/limit, antiwindup, raw guard and invalid Watch edits.\n");
  printf("PASS: outer velocity feedback reduces drift in an illustrative plant; actual-cart stability remains unverified.\n");
  return 0;
}
