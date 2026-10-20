/* Inject position/speed to verify production control logic, not cart physics. */
#include "balance_control.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
static uint32_t now;
static void tick(void)
{
  float wheels[4]={0,0,0,0};
  now+=5U;Balance_Update(0,0,0,1,1,0,now);Balance_CheckMotion(wheels,now);
}
static void activate(uint32_t start)
{
  uint32_t i;
  assert(Balance_Arm(1));now=start;
  Balance_Update(0,0,0,1,1,0,now);
  for(i=0;i<80;i++) tick();
  assert(g_bal_state==BAL_ACTIVE && g_bal_enable);
}
static void near(float a,float b) {assert(fabsf(a-b)<0.00001f);}
int main(void)
{
  uint32_t i,active;
  float command,position;
  assert(g_bal_pos_kp==20 && g_bal_pos_max_rpm==6);
  assert(!Balance_SetPosition(0) && !Balance_HoldPosition(0) && !Balance_HoldPosition(1));
  activate(100);
  assert(g_bal_seq_state==BAL_SEQ_HOLD && !g_bal_seq_goal_rev && !g_bal_position_rev);
  Balance_AddPosition(0.1f);tick();near(g_bal_vel_target_rpm,-2);
  tick();near(g_bal_position_rev,0.1f); /* Capture origin is not redefined each loop. */
  Balance_AddPosition(-0.2f);tick();near(g_bal_vel_target_rpm,2);
  Balance_Stop(BAL_REASON_STOP);g_bal_vel_enable=0;
  activate(600);assert(g_bal_seq_state==BAL_SEQ_NONE); /* Explicit angle-only opt-out. */
  Balance_Stop(BAL_REASON_STOP);g_bal_vel_enable=1;
  activate(1000);
  Balance_AddPosition(0.1f);assert(Balance_SetSpeed(2));
  for(i=0;i<10;i++) tick();
  command=g_bal_vel_command_rpm;g_bal_vel_integral_deg=0.04f;
  assert(Balance_SetPosition(0));
  near(g_bal_position_rev,0.1f);near(g_bal_seq_goal_rev,0);
  assert(g_bal_seq_state==BAL_SEQ_HOLD && g_bal_enable);
  near(g_bal_vel_command_rpm,command);near(g_bal_vel_integral_deg,0.04f);
  tick();near(g_bal_vel_target_rpm,-2);near(g_bal_vel_command_rpm,command-0.05f);
  assert(Balance_SetPosition(0));near(g_bal_position_rev,0.1f);
  Balance_AddPosition(-0.2f);tick();near(g_bal_vel_target_rpm,2);
  Balance_AddPosition(-0.5f);tick();near(g_bal_vel_target_rpm,6);
  assert(Balance_SetPosition(-0.6f));tick();near(g_bal_vel_target_rpm,0);
  Balance_AddPosition(0.001f);tick();near(g_bal_vel_target_rpm,-0.02f); /* no deadband */
  assert(Balance_HoldPosition(0));
  position=g_bal_position_rev;near(g_bal_seq_goal_rev,position);
  Balance_AddPosition(0.05f);tick();near(g_bal_vel_target_rpm,-1);
  assert(Balance_HoldPosition(1));
  near(g_bal_position_rev,0);near(g_bal_seq_goal_rev,0);tick();near(g_bal_vel_target_rpm,0);
  assert(Balance_SetPosition(5));tick();near(g_bal_vel_target_rpm,6);
  assert(!Balance_SetPosition(5.01f) && !Balance_SetPosition(-5.01f) && !Balance_SetPosition(NAN));
  near(g_bal_seq_goal_rev,5);assert(!Balance_HoldPosition(2));
  g_bal_pos_max_rpm=3;tick();near(g_bal_vel_target_rpm,3);
  g_bal_max_rpm=10;g_bal_pos_max_rpm=15;tick();near(g_bal_vel_target_rpm,5);
  g_bal_max_rpm=60;g_bal_pos_max_rpm=6;
  assert(Balance_SetTurn(3));
  assert(!g_bal_seq_state && !g_bal_seq_goal_rev && !g_bal_vel_target_rpm);
  assert(!Balance_SetPosition(0) && !Balance_HoldPosition(0));tick();
  assert(Balance_SetTurn(0));assert(!Balance_SetPosition(0)); /* ramp still nonzero */
  for(i=0;i<20;i++) tick();
  assert(Balance_SetPosition(0));
  assert(Balance_SetSpeed(0) && !g_bal_seq_state); /* keep balance; cancel hold */
  assert(Balance_StartSequence(8,0.25f,500));Balance_AddPosition(0.1f);
  assert(Balance_SetPosition(0) && g_bal_seq_state==BAL_SEQ_HOLD);
  near(g_bal_position_rev,0.1f); /* switching out of sequence does not re-zero */
  Balance_Stop(BAL_REASON_STOP);
  assert(!g_bal_seq_state && !g_bal_vel_target_rpm && !g_bal_vel_command_rpm);
  near(g_bal_position_rev,0.1f);assert(!Balance_SetPosition(0));
  activate(3000);near(g_bal_position_rev,0);
  assert(g_bal_seq_state==BAL_SEQ_HOLD && !g_bal_seq_goal_rev);
  for(i=0;i<12;i++) Balance_AddPosition(0.5f);
  assert(Balance_HoldPosition(0));near(g_bal_seq_goal_rev,6);tick();
  assert(g_bal_enable && g_bal_seq_state==BAL_SEQ_HOLD);
  assert(Balance_HoldPosition(1));near(g_bal_seq_goal_rev,0);near(g_bal_position_rev,0);
  Balance_BeginZero();assert(g_bal_state==BAL_ZEROING && !g_bal_seq_state && !g_bal_vel_target_rpm);

  activate(5000);assert(Balance_SetPosition(0));
  g_bal_pos_kp=NAN;tick();assert(!g_bal_enable && g_bal_reason==BAL_REASON_CONFIG);
  g_bal_pos_kp=20;activate(6000);assert(Balance_SetPosition(0));
  g_bal_pos_max_rpm=NAN;tick();assert(!g_bal_enable && g_bal_reason==BAL_REASON_CONFIG);
  g_bal_pos_max_rpm=6;activate(7000);assert(Balance_SetPosition(0));
  g_bal_seq_goal_rev=NAN;tick();assert(!g_bal_enable && g_bal_reason==BAL_REASON_CONFIG);

  activate(UINT32_MAX-600U);assert(Balance_SetPosition(0));
  Balance_AddPosition(0.1f);for(i=0;i<200;i++) tick();
  assert(g_bal_enable && g_bal_seq_state==BAL_SEQ_HOLD && g_bal_seq_ms==1000U);
  g_bal_max_run_ms=500;activate(8000);active=now;
  for(i=0;i<60;i++) tick();assert(Balance_SetPosition(0));
  while((uint32_t)(now-active)<500U) tick();
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_RUN_TIME && !g_bal_seq_state);
  puts("PASS: auto-hold after capture, signed position correction/clamp/no-deadband; explicit target preserves origin/ramp/I; hold/zero here; turn/speed/sequence switching; range/NaN/stop/zero/timeout/tick-wrap");
  return 0;
}
