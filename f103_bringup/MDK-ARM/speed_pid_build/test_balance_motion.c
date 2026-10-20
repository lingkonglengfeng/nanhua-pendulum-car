/* Production motion/state-machine tests, not a physical cart simulation. */
#include "balance_control.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#define RAD 0.017453292519943295f
static uint32_t now;
static void sample(float deg, float guard)
{
  float wheels[4]={0,0,0,0};
  Balance_Update(deg*RAD,0,guard*RAD,1,1,0,now);
  Balance_CheckMotion(wheels,now);
}
static void tick(void) { now+=5U; sample(0,0); }
static void activate(uint32_t start)
{
  uint32_t i;
  assert(Balance_Arm(1));now=start;sample(0,0);
  for(i=0;i<80;i++) tick();
  assert(g_bal_state==BAL_ACTIVE && g_bal_enable);
}
static void await_stage(uint8_t state, uint32_t ticks)
{
  while(g_bal_seq_state!=state && ticks--) tick();
  assert(g_bal_seq_state==state && g_bal_enable);
}
int main(void)
{
  uint32_t i,active;
  float previous;
  assert(g_bal_entry_deg==8.0f && g_bal_exit_deg==30.0f && g_bal_limit==0.80f);
  assert(g_bal_turn_kp==0.008f);
  assert(g_bal_turn_ki==0.010f && !g_bal_turn_integral);
  assert(!Balance_SetSpeed(5) && !Balance_StartSequence(8,0.25f,2000));
  Balance_AddPosition(0.1f);assert(g_bal_position_rev==0);
  /* Capture boundary and dwell stay separate from the wider exit angle. */
  assert(Balance_Arm(1));now=100;sample(8.01f,8.01f);
  assert(g_bal_state==BAL_WAIT_TOP && !g_bal_enable);
  now+=5;sample(7.99f,7.99f);
  for(i=0;i<79;i++) {now+=5;sample(7.99f,7.99f);}
  assert(!g_bal_enable && g_bal_hold_ms==395U);
  now+=5;sample(7.99f,7.99f);assert(g_bal_enable);
  assert(g_bal_seq_state==BAL_SEQ_HOLD && !g_bal_position_rev && !g_bal_seq_goal_rev);
  assert(!g_bal_vel_target_rpm && !g_bal_turn_target_rpm);
  now+=5;sample(0,29.99f);assert(g_bal_enable);
  now+=5;sample(0,30.01f);assert(g_bal_reason==BAL_REASON_ANGLE && !g_bal_enable);

  activate(1000U);
  assert(!Balance_SetSpeed(NAN) && !Balance_SetSpeed(31));
  assert(!Balance_StartSequence(0,0.25f,1000));
  assert(!Balance_StartSequence(8,0.6f,1000));
  assert(!Balance_StartSequence(8,NAN,1000));
  assert(!Balance_StartSequence(8,0.25f,399));
  g_bal_max_rpm=20; assert(!Balance_SetSpeed(11));g_bal_max_rpm=60;
  assert(Balance_SetSpeed(10));tick();
  assert(fabsf(g_bal_vel_command_rpm-0.05f)<1e-6f);
  assert(g_bal_ref_deg<0); /* v-target retains the zero-loop polarity. */
  for(i=0;i<30;i++) {
    previous=g_bal_vel_command_rpm;tick();
    assert(fabsf(g_bal_vel_command_rpm-previous)<=0.05001f);
  }
  g_bal_vel_integral_deg=0.04f;
  assert(Balance_SetSpeed(-5) && g_bal_vel_integral_deg==0.04f);
  assert(g_bal_vel_command_rpm>0); /* Reversal is ramped, not instantaneous. */
  assert(Balance_StartSequence(8,0.25f,500));
  assert(g_bal_seq_goal_rev==0.25f && g_bal_seq_state==BAL_SEQ_OUT);
  assert(g_bal_vel_integral_deg==0.04f);
  Balance_Stop(BAL_REASON_STOP);
  assert(!g_bal_enable && !g_bal_vel_target_rpm && !g_bal_vel_command_rpm && !g_bal_seq_state);

  activate(5000U);
  assert(Balance_StartSequence(8,0.25f,500));tick();
  assert(g_bal_vel_target_rpm>0 && g_bal_seq_state==BAL_SEQ_OUT);
  Balance_AddPosition(0.25f);tick();
  for(i=0;i<79;i++) tick();
  assert(g_bal_seq_state==BAL_SEQ_OUT); /* 395ms is not enough. */
  tick();assert(g_bal_seq_state==BAL_SEQ_PAUSE);
  /* Drift during the pause restarts the required settled hold interval. */
  for(i=0;i<40;i++) tick();
  Balance_AddPosition(0.03f);tick();
  Balance_AddPosition(-0.03f);tick();
  for(i=0;i<99;i++) tick();
  assert(g_bal_seq_state==BAL_SEQ_PAUSE);
  tick();assert(g_bal_seq_state==BAL_SEQ_RETURN && g_bal_seq_goal_rev==0);
  assert(g_bal_vel_target_rpm<0);
  Balance_AddPosition(-0.25f);
  await_stage(BAL_SEQ_HOME,100U);
  assert(fabsf(g_bal_position_rev)<0.00001f && g_bal_enable);
  Balance_AddPosition(0.03f);tick();
  assert(g_bal_seq_state==BAL_SEQ_HOME && g_bal_vel_target_rpm<0);
  assert(Balance_SetSpeed(0) && g_bal_seq_state==BAL_SEQ_NONE && g_bal_enable);

  activate(12000U);
  assert(Balance_StartSequence(-8,0.25f,500));tick();
  assert(g_bal_seq_goal_rev==-0.25f && g_bal_vel_target_rpm<0);
  assert(!Balance_StartSequence(8,0.7f,500));
  assert(g_bal_seq_goal_rev==-0.25f); /* Atomic invalid-command rejection. */
  Balance_AddPosition(-0.25f);await_stage(BAL_SEQ_PAUSE,100U);
  await_stage(BAL_SEQ_RETURN,120U);
  assert(g_bal_seq_goal_rev==0 && g_bal_vel_target_rpm>0);
  Balance_AddPosition(0.25f);await_stage(BAL_SEQ_HOME,100U);
  assert(g_bal_enable && fabsf(g_bal_position_rev)<1e-6f);
  g_bal_vel_target_rpm=NAN;tick();
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_CONFIG);
  activate(14000U);Balance_AddPosition(NAN);tick();
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_CONFIG);

  /* A sequence never renews the independent ACTIVE time budget. */
  g_bal_max_run_ms=500;activate(16000U);active=now;
  for(i=0;i<60;i++) tick();
  assert(Balance_StartSequence(8,0.25f,500));
  while((uint32_t)(now-active)<495U) tick();
  assert(g_bal_enable);tick();
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_RUN_TIME && !g_bal_seq_state);
  assert(!g_bal_vel_target_rpm && !g_bal_vel_command_rpm);

  g_bal_max_run_ms=24000;activate(UINT32_MAX-600U);
  assert(Balance_StartSequence(8,0.25f,400));
  Balance_AddPosition(0.25f);await_stage(BAL_SEQ_PAUSE,100U);
  await_stage(BAL_SEQ_RETURN,100U);
  assert(g_bal_enable); /* Capture/dwell/phase timestamps all crossed wrap. */
  /* Continuous mode does not expire at old 24s/60s limits; finite mode above
   * still expires. No plant assumptions: sensor remains upright here. */
  g_bal_max_run_ms=0;activate(UINT32_MAX-1000U);
  for(i=0;i<24001;i++) tick();
  assert(g_bal_enable && g_bal_run_ms==120005U);

  assert(!Balance_SetTurn(NAN) && !Balance_SetTurn(10.01f));
  assert(Balance_SetTurn(3));tick();
  assert(fabsf(g_bal_turn_command_rpm-0.05f)<1e-6f && g_bal_turn_output>0);
  assert(Balance_MotorOutput(2)>Balance_MotorOutput(0));
  assert(Balance_MotorOutput(3)==Balance_MotorOutput(2));
  assert(Balance_MotorOutput(0)==Balance_MotorOutput(1));
  for(i=0;i<60;i++) tick();
  assert(g_bal_turn_command_rpm>2.99f);
  assert(!Balance_StartSequence(8,0.25f,500));
  assert(!Balance_SetSpeed(28)); /* Sum of translation+turn target <=30. */
  assert(Balance_SetTurn(-3));previous=g_bal_turn_command_rpm;tick();
  assert(g_bal_turn_command_rpm>0 && previous-g_bal_turn_command_rpm<=0.05001f);
  for(i=0;i<120;i++) tick();
  assert(g_bal_turn_output<0 && Balance_MotorOutput(2)<Balance_MotorOutput(0));
  /* Balance saturation cancels yaw without altering common-mode duty. */
  g_bal_output=0.79f;g_bal_turn_output=0.12f;
  assert(fabsf((Balance_MotorOutput(0)+Balance_MotorOutput(2))*0.5f-0.79f)<1e-6f);
  assert(Balance_MotorOutput(2)<=0.80001f);
  g_bal_output=-g_bal_limit;
  assert(Balance_MotorOutput(0)==-g_bal_limit && Balance_MotorOutput(3)==-g_bal_limit);
  tick();
  assert(Balance_SetTurn(0));for(i=0;i<70;i++) tick();
  assert(fabsf(g_bal_turn_command_rpm)<1e-6f);
  assert(Balance_StartSequence(8,0.25f,500));
  assert(Balance_SetTurn(0) && g_bal_seq_state==BAL_SEQ_OUT);
  assert(Balance_SetTurn(3) && g_bal_seq_state==BAL_SEQ_NONE && g_bal_vel_target_rpm==0);
  Balance_Stop(BAL_REASON_STOP);
  assert(!g_bal_turn_target_rpm && !g_bal_turn_command_rpm && !g_bal_turn_output);
  assert(Balance_MotorOutput(0)==0 && !Balance_SetTurn(3));
  g_bal_turn_ki=0;activate(1000U);
  {
    float wheels[4]={-3,-3,3,3};
    assert(Balance_SetTurn(3));
    for(i=0;i<150;i++) { now+=5;Balance_Update(0,0,0,1,1,0,now);Balance_CheckMotion(wheels,now); }
    assert(fabsf(g_bal_turn_rpm-3)<0.001f && fabsf(g_bal_turn_output)<0.0001f);
    assert(fabsf(g_bal_robust_rpm)<1e-6f); /* No fictitious forward travel. */
  }
  activate(5000U);
  {
    float raw[4]={70,0,0,0}, feedback[4]={1,1,1,1};
    for(i=0;i<=40;i++) {
      now+=5;Balance_Update(0,0,0,1,1,0,now);
      Balance_CheckMotionFeedback(raw,feedback,now);
      if(i<40) assert(g_bal_enable && g_bal_guard_rpm==70 && g_bal_robust_rpm==1);
    }
    assert(!g_bal_enable && g_bal_reason==BAL_REASON_SPEED);
  }
  activate(6000U);
  {
    float raw[4]={0,0,0,0}, feedback[4]={-3,-3,3,3};
    assert(Balance_SetTurn(3));
    for(i=0;i<150;i++) {
      now+=5;Balance_Update(0,0,0,1,1,0,now);
      Balance_CheckMotionFeedback(raw,feedback,now);
    }
    assert(fabsf(g_bal_turn_rpm-3)<0.001f && !g_bal_guard_rpm && !g_bal_robust_rpm);
    feedback[2]=NAN;now+=5;Balance_Update(0,0,0,1,1,0,now);
    Balance_CheckMotionFeedback(raw,feedback,now);
    assert(!g_bal_enable && g_bal_reason==BAL_REASON_SENSOR);
  }
  /* Persistent tracking error must raise torque above the old P-only value,
   * without an unlimited integral at yaw or balance saturation. */
  g_bal_turn_ki=0.010f;activate(8000U);
  assert(Balance_SetTurn(5));
  for(i=0;i<100;i++) tick();
  assert(g_bal_turn_output>0.04f && g_bal_turn_integral>0);
  for(i=0;i<1000;i++) tick();
  assert(g_bal_turn_output>0.119f && g_bal_turn_output<=BAL_TURN_DUTY_LIMIT);
  assert(g_bal_turn_integral<=0.081f); /* Outward integration frozen. */
  now+=5;sample(10,10); /* Common output saturates; yaw has no room. */
  assert(g_bal_enable && fabsf(g_bal_output)==g_bal_limit);
  assert(!g_bal_turn_output && !g_bal_turn_integral);
  tick();for(i=0;i<100;i++) tick();assert(g_bal_turn_integral>0);
  assert(Balance_SetTurn(-5) && !g_bal_turn_integral);
  tick();assert(!g_bal_turn_integral); /* Old ramp still positive: no old I. */
  for(i=0;i<600;i++) tick();
  assert(g_bal_turn_output<-0.1f && g_bal_turn_integral<0);
  assert(Balance_SetTurn(0) && !g_bal_turn_integral);
  for(i=0;i<120;i++) tick();
  assert(!g_bal_turn_command_rpm && !g_bal_turn_output && !g_bal_turn_integral);
  assert(Balance_SetTurn(5));for(i=0;i<150;i++) tick();
  assert(g_bal_turn_integral>0);Balance_Stop(BAL_REASON_STOP);
  assert(!g_bal_turn_integral && !g_bal_turn_output);
  activate(15000U);g_bal_turn_ki=NAN;tick();
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_CONFIG);
  g_bal_turn_ki=0.010f;
  activate(16000U);g_bal_turn_kp=NAN;tick();
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_CONFIG && !g_bal_turn_output);
  puts("PASS: continuous 120s across tick wrap; differential turn ramp/feedback/motor map, balance-priority mixer, sequence conflicts, STOP and invalid configuration");
  puts("PASS: 8deg/400ms capture auto-holds location zero; 30deg exit/80pct common/12pct yaw retained; turn PI and speed/sequence regressions");
  return 0;
}
