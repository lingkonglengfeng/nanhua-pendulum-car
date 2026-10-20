/* Production state-machine tests; wheel movement is injected, not simulated. */
#include "balance_control.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
static uint32_t now;
static void tick(void)
{
  float wheels[4]={0,0,0,0};
  now+=5U;
  Balance_Update(0,0,0,1,1,0,now);
  Balance_CheckMotion(wheels,now);
}
static void ticks(unsigned count) { while(count--) tick(); }
static void move_to(float goal)
{
  float delta=goal-g_bal_position_rev;
  while(fabsf(delta)>0.5f)
  {
    Balance_AddPosition((delta<0) ? -0.5f : 0.5f);
    delta=goal-g_bal_position_rev;
  }
  Balance_AddPosition(delta);
}
static void activate(uint32_t start)
{
  Balance_Stop(BAL_REASON_STOP);
  assert(Balance_Arm(1));now=start;
  Balance_Update(0,0,0,1,1,0,now);ticks(80);
  assert(g_bal_state==BAL_ACTIVE && g_bal_enable);
}
int main(void)
{
  float origin,goal,previous,integral;
  unsigned i;
  g_bal_wheel_diameter_mm=0;
  assert(!Balance_StartStep(1,8));
  activate(1000);
  assert(!Balance_StartStep(1,8)); /* No guessed diameter. */
  g_bal_wheel_diameter_mm=65;
  assert(!Balance_StartStep(NAN,8) && !Balance_StartStep(1,NAN));
  assert(!Balance_StartStep(0,8) && !Balance_StartStep(2.01f,8));
  assert(!Balance_StartStep(1,0) && !Balance_StartStep(1,21));
  move_to(1.2f);origin=g_bal_position_rev;
  assert(Balance_SetTurn(1) && !Balance_StartStep(1,8));
  assert(Balance_SetTurn(0));
  g_bal_vel_integral_deg=0.04f;integral=g_bal_vel_integral_deg;
  assert(Balance_StartStep(1,8));
  assert(g_bal_seq_state==BAL_SEQ_STEP_WAIT && g_bal_seq_goal_rev==origin);
  assert(g_bal_position_rev==origin && g_bal_vel_integral_deg==integral);
  assert(!Balance_StartStep(1,8)); /* Repeated command cannot move the anchor. */
  tick();ticks(998);assert(g_bal_seq_state==BAL_SEQ_STEP_WAIT);
  tick();assert(g_bal_seq_state==BAL_SEQ_STEP_MOVE);
  goal=origin+1000.0f/(3.14159265359f*65.0f);
  assert(fabsf(g_bal_seq_goal_rev-goal)<1e-5f && goal>5.0f);
  assert(g_bal_vel_target_rpm==8 && g_bal_position_rev==origin);
  for(i=0;i<20;i++)
  {
    previous=g_bal_vel_command_rpm;tick();
    assert(fabsf(g_bal_vel_command_rpm-previous)<=0.05001f && g_bal_enable);
  }
  move_to(goal);tick();
  ticks(79);assert(g_bal_seq_state==BAL_SEQ_STEP_MOVE);
  tick();assert(g_bal_seq_state==BAL_SEQ_STEP_END);
  ticks(500);
  Balance_AddPosition(0.03f);tick(); /* Endpoint correction does not restart the timer. */
  ticks(498);
  assert(g_bal_seq_state==BAL_SEQ_STEP_END);
  tick();assert(g_bal_seq_state==BAL_SEQ_HOLD && g_bal_enable);
  assert(fabsf(g_bal_seq_goal_rev-goal)<1e-5f); /* No return to the origin. */
  Balance_AddPosition(0.03f);tick();assert(g_bal_vel_target_rpm<0);
  assert(Balance_SetSpeed(0) && g_bal_seq_state==BAL_SEQ_NONE);

  activate(20000);assert(Balance_StartStep(-1,8));tick();ticks(500);
  Balance_AddPosition(0.03f);tick();g_bal_vel_rpm=3.0f;tick();
  ticks(496);assert(g_bal_seq_state==BAL_SEQ_STEP_WAIT);
  tick();assert(g_bal_seq_state==BAL_SEQ_STEP_MOVE && g_bal_seq_goal_rev<0);
  assert(g_bal_vel_target_rpm<0);
  assert(Balance_SetTurn(1) && g_bal_seq_state==BAL_SEQ_NONE);
  Balance_Stop(BAL_REASON_STOP);assert(!g_bal_seq_state && !g_bal_enable);

  /* Dwell subtraction remains correct across the millisecond tick wrap. */
  activate(UINT32_MAX-2000U);assert(Balance_StartStep(1,8));tick();ticks(1000);
  assert(g_bal_seq_state==BAL_SEQ_STEP_MOVE && g_bal_enable);
  g_bal_wheel_diameter_mm=NAN;tick();
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_CONFIG);
  g_bal_wheel_diameter_mm=65;
  activate(30000);g_bal_max_run_ms=500;
  assert(Balance_StartStep(1,8));ticks(100);
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_RUN_TIME && !g_bal_seq_state);
  g_bal_max_run_ms=0;
  activate(40000);assert(Balance_StartStep(1,8));
  now+=5;Balance_Update(0,0,0,0,1,0,now);
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_SENSOR && !g_bal_seq_state);
  puts("PASS: metre conversion, preserved origin/PI, fixed 5s waits despite drift, ramp/settled arrival, final hold/no return, reverse/cancel, wrap and existing faults.");
  return 0;
}
