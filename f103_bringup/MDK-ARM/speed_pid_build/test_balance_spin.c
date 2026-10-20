/* Exercise production spin/position coupling with injected measurements. */
#include "balance_control.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
static uint32_t now;
static void tick(void)
{
  float raw[4]={0,0,0,0};
  now+=5;Balance_Update(0,0,0,1,1,0,now);Balance_CheckMotion(raw,now);
}
static void activate(uint32_t start)
{
  unsigned i;assert(Balance_Arm(1));now=start;Balance_Update(0,0,0,1,1,0,now);
  for(i=0;i<80;i++)tick();
  assert(g_bal_enable && g_bal_seq_state==BAL_SEQ_HOLD);
}
static void near(float a,float b){assert(fabsf(a-b)<0.00001f);}
int main(void)
{
  unsigned i;float goal,ramp;uint32_t active;
  assert(!Balance_StartSpin(3));activate(1000);
  /* Replay observed asymmetric spin wheel speeds: median hides translation.
   * All-wheel mean must agree with the odometry channel while spinning. */
  assert(Balance_StartSpin(10));
  {
    float wheels[4]={0,0,33.12f,14.19f};
    now+=5;Balance_Update(0,0,0,1,1,0,now);
    Balance_CheckMotionFeedback(wheels,wheels,now);
    near(g_bal_robust_rpm,11.8275f); /* Old middle pair was only 7.095. */
    near(g_bal_guard_rpm,33.12f);
    wheels[0]=5;wheels[1]=5;wheels[2]=25;wheels[3]=25;
    now+=5;Balance_Update(0,0,0,1,1,0,now);Balance_CheckMotionFeedback(wheels,wheels,now);
    near(g_bal_robust_rpm,15);
    wheels[0]=-10;wheels[1]=-10;wheels[2]=10;wheels[3]=10;
    now+=5;Balance_Update(0,0,0,1,1,0,now);Balance_CheckMotionFeedback(wheels,wheels,now);
    near(g_bal_robust_rpm,0); /* Pure differential motion is not translation. */
    wheels[0]=0;wheels[1]=0;wheels[2]=-33.12f;wheels[3]=-14.19f;
    now+=5;Balance_Update(0,0,0,1,1,0,now);Balance_CheckMotionFeedback(wheels,wheels,now);
    near(g_bal_robust_rpm,-11.8275f);
    assert(Balance_SetSpeed(0)); /* Legacy straight/manual rejection preserved. */
    wheels[2]=33.12f;wheels[3]=14.19f;
    now+=5;Balance_Update(0,0,0,1,1,0,now);Balance_CheckMotionFeedback(wheels,wheels,now);
    near(g_bal_robust_rpm,7.095f);
  }
  activate(2000);
  Balance_AddPosition(0.2f);g_bal_vel_integral_deg=0.04f;
  assert(Balance_StartSpin(3));goal=g_bal_seq_goal_rev;
  near(goal,0.2f);near(g_bal_position_rev,goal);near(g_bal_vel_integral_deg,0.04f);
  assert(g_bal_seq_state==BAL_SEQ_SPIN && !g_bal_vel_target_rpm);
  tick();near(g_bal_turn_command_rpm,0.05f);assert(g_bal_turn_output>0);
  assert(Balance_MotorOutput(2)>Balance_MotorOutput(0));
  Balance_AddPosition(0.1f);tick();near(g_bal_vel_target_rpm,-2);
  assert(g_bal_seq_state==BAL_SEQ_SPIN && g_bal_turn_target_rpm==3);
  ramp=g_bal_vel_command_rpm;
  assert(Balance_StartSpin(4));near(g_bal_seq_goal_rev,goal);near(g_bal_vel_command_rpm,ramp);
  assert(!Balance_StartSpin(NAN) && !Balance_StartSpin(10.01f) && !Balance_StartSpin(-10.01f));
  near(g_bal_turn_target_rpm,4);near(g_bal_seq_goal_rev,goal);
  for(i=0;i<100;i++)tick();
  assert(g_bal_turn_command_rpm>3.99f);
  g_bal_turn_integral=0.03f;
  assert(Balance_StartSpin(-3));assert(!g_bal_turn_integral && g_bal_turn_command_rpm>0);
  for(i=0;i<200;i++)tick();
  assert(g_bal_turn_command_rpm<0 && g_bal_turn_output<0);
  assert(g_bal_seq_state==BAL_SEQ_SPIN && g_bal_vel_target_rpm<0);
  assert(!Balance_SetPosition(0));
  assert(Balance_StartSpin(0));near(g_bal_seq_goal_rev,goal);
  for(i=0;i<100;i++)tick();
  assert(!g_bal_turn_command_rpm && g_bal_seq_state==BAL_SEQ_SPIN);
  assert(Balance_StartSpin(0));near(g_bal_seq_goal_rev,goal); /* no moving anchor */
  Balance_AddPosition(-0.1f);tick();near(g_bal_vel_target_rpm,0);
  g_bal_max_rpm=10;assert(Balance_StartSpin(4));Balance_AddPosition(1);tick();
  near(g_bal_vel_target_rpm,-1); /* mean correction + yaw <= half speed guard */
  assert(Balance_StartSpin(5));tick();near(g_bal_vel_target_rpm,0);
  assert(!Balance_StartSpin(6));near(g_bal_turn_target_rpm,5);
  g_bal_max_rpm=200;
  assert(Balance_SetSpeed(0));assert(!g_bal_seq_state && g_bal_turn_target_rpm==5);
  assert(Balance_StartSpin(0));near(g_bal_seq_goal_rev,g_bal_position_rev);
  for(i=0;i<120;i++)tick();
  assert(Balance_SetTurn(3));assert(!g_bal_seq_state); /* legacy TURN semantics */
  assert(Balance_SetTurn(0));for(i=0;i<30;i++)tick();
  assert(Balance_StartSequence(8,0.25f,500));Balance_AddPosition(0.1f);
  assert(Balance_StartSpin(3));near(g_bal_seq_goal_rev,0.1f);near(g_bal_position_rev,0.1f);
  Balance_Stop(BAL_REASON_STOP);
  assert(!g_bal_seq_state && !g_bal_turn_target_rpm && !g_bal_turn_output && !g_bal_vel_target_rpm);
  assert(!Balance_StartSpin(3));
  activate(UINT32_MAX-600U);assert(Balance_StartSpin(-3));
  for(i=0;i<200;i++)tick();
  assert(g_bal_enable && g_bal_seq_state==BAL_SEQ_SPIN && g_bal_seq_ms==1000);
  g_bal_seq_goal_rev=NAN;tick();assert(!g_bal_enable && g_bal_reason==BAL_REASON_CONFIG);
  activate(5000);assert(Balance_StartSpin(3));
  now+=5;Balance_Update(0,0,0,0,1,0,now);
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_SENSOR && !g_bal_turn_target_rpm);
  g_bal_max_run_ms=500;activate(6000);active=now;
  for(i=0;i<60;i++)tick();assert(Balance_StartSpin(3));
  while((uint32_t)(now-active)<500)tick();
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_RUN_TIME && !g_bal_seq_state);
  puts("PASS: spin retains entry anchor plus signed mean-position feedback and yaw; repeat/neutral/reverse, target headroom, invalid atomic rejection, manual/sequence switching, STOP/fault/timeout and tick wrap");
  return 0;
}
