/* Production swing-up logic/energy-sign tests; not an identified cart model. */
#include "balance_control.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#define RAD 0.017453292519943295f
static uint32_t now;
static void tick(float theta,float rate,float rpm)
{
  float wheels[4]={rpm,rpm,rpm,rpm};
  now+=5;Balance_Update(theta,rate,theta,1,1,0,now);Balance_CheckMotion(wheels,now);
}
static void start(void)
{
  BalanceExitEvent e;
  Balance_Stop(BAL_REASON_STOP);
  if(Balance_PeekExit(&e))Balance_AckExit(e.ms);
  now+=100;Balance_Update(3.1415926f,0,3.1415926f,1,1,0,now);
  assert(Balance_StartSwing(1,1,0));
  assert(g_bal_state==BAL_SWING && g_bal_enable && g_swing_phase==1);
  assert(!g_bal_position_rev && !g_bal_seq_state && !g_bal_turn_target_rpm);
}
int main(void)
{
  unsigned i;float previous;BalanceExitEvent e;
  assert(g_swing_gain==0.65f && g_swing_omega2==63.5f && g_swing_max_ms==15000);
  assert(g_swing_limit==0.37f && g_swing_travel_rev==2.0f && g_swing_capture_rate==2.0f);
  assert(g_swing_capture_deg==2 && g_bal_entry_deg==8);
  /* Energy-sign fixtures retain their original model/gain explicitly. */
  g_swing_limit=0.35f;g_swing_gain=0.18f;g_swing_omega2=55;g_swing_max_ms=10000;
  assert(!Balance_StartSwing(0,1,0) && !Balance_StartSwing(1,0,0));
  now=100;Balance_Update(0,0,0,1,1,0,now);
  assert(!Balance_StartSwing(1,1,0)); /* upright is not a bottom start */
  Balance_Update(3.1415926f,0,3.1415926f,1,1,0,now+5);now+=5;
  assert(!Balance_StartSwing(1,1,6) && !Balance_StartSwing(1,1,NAN));
  start();
  assert(!Balance_SetSpeed(3) && !Balance_SetTurn(3) && !Balance_SetPosition(0) && !Balance_StartSpin(3));
  for(i=0;i<29;i++)
  {
    previous=g_bal_output;tick(3.1415926f,0,0);
    assert(g_bal_state==BAL_SWING && g_swing_phase==1);
    assert(g_bal_output>=0 && g_bal_output<=g_swing_limit);
    assert(fabsf(g_bal_output-previous)<=0.01501f);
    assert(Balance_MotorOutput(0)==Balance_MotorOutput(3));
  }
  tick(2.94f,-1,0);assert(g_swing_phase==2);
  for(i=0;i<50;i++)tick(2.94f,-1,0);
  assert(g_swing_energy<0 && g_bal_output>0.3f); /* deficient E: inject energy */
  for(i=0;i<50;i++)tick(2.94f,1,0);
  assert(g_bal_output<-0.3f);
  for(i=0;i<50;i++)tick(2.94f,18,0);
  assert(g_swing_energy>1.02f && g_bal_output>0); /* excess E: remove energy */
  g_bal_sign=1;
  for(i=0;i<50;i++)tick(2.94f,-1,0);
  assert(g_bal_output<0);g_bal_sign=-1;
  Balance_AddPosition(0.5f);
  for(i=0;i<50;i++)tick(3.1415926f,0,20);
  assert(g_bal_output<0); /* cart centering and damping */
  Balance_AddPosition(-1);
  for(i=0;i<50;i++)tick(3.1415926f,0,-20);
  assert(g_bal_output>0);

  start();Balance_AddPosition(1);Balance_AddPosition(1);tick(3.1415926f,0,0);
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_TRAVEL && g_swing_phase==4);
  assert(Balance_PeekExit(&e) && e.reason==BAL_REASON_TRAVEL && !Balance_MotorOutput(0));
  start();tick(1*RAD,3,0);assert(g_bal_state==BAL_SWING); /* rate too fast */
  /* Replay v39 capture: raw just inside manual 8deg, filtered angle 8.086deg.
   * Automatic controller must keep pumping, rather than jump to -80% duty. */
  now+=5;Balance_Update(8.086f*RAD,-0.815f,7.998f*RAD,1,1,0,now);
  assert(g_bal_state==BAL_SWING && fabsf(g_bal_output)<=g_swing_limit);
  tick(7*RAD,-0.815f,0);assert(g_bal_state==BAL_SWING);
  tick(2.01f*RAD,0,0);assert(g_bal_state==BAL_SWING);
  now+=5;Balance_Update(2.01f*RAD,0,1.99f*RAD,1,1,0,now);
  assert(g_bal_state==BAL_SWING); /* filtered PID angle also bounded */
  {
    float wheels[4]={100,100,100,100};
    Balance_CheckMotion(wheels,now);tick(1.9f*RAD,-1.2f,100);
    assert(g_bal_state==BAL_SWING); /* cart too fast */
    wheels[0]=wheels[1]=wheels[2]=wheels[3]=80;Balance_CheckMotion(wheels,now);
    tick(1.9f*RAD,1.2f,80);assert(g_bal_state==BAL_SWING); /* predicted outside window */
    tick(1.9f*RAD,-1.2f,80);
    assert(g_bal_state==BAL_ACTIVE && g_swing_phase==3 && g_bal_enable);
    assert(g_bal_seq_state==BAL_SEQ_HOLD && !g_bal_position_rev && !g_bal_seq_goal_rev);
    assert(g_bal_vel_rpm>79 && !g_bal_run_ms && fabsf(g_bal_output)<=g_bal_limit);
  }
  /* A tighter manual window remains an upper bound for automatic transfer. */
  start();g_bal_entry_deg=0.5f;tick(0.6f*RAD,0,0);assert(g_bal_state==BAL_SWING);
  tick(0.4f*RAD,0,0);assert(g_bal_state==BAL_ACTIVE);g_bal_entry_deg=8;
  start();g_swing_capture_deg=NAN;tick(3.14f,0,0);
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_CONFIG);g_swing_capture_deg=2;
  /* With the deployed PID, a stationary cart near upright transfers without
   * the logged full-duty discontinuity. Repeat for each approach direction. */
  g_bal_kp=8.4f;g_bal_ki=0;g_bal_kd=0.17f;
  start();tick(1.9f*RAD,-0.5f,0);
  assert(g_bal_state==BAL_ACTIVE && fabsf(g_bal_output)<0.35f);
  start();tick(-1.9f*RAD,0.5f,0);
  assert(g_bal_state==BAL_ACTIVE && fabsf(g_bal_output)<0.35f);
  start();now+=16;Balance_Update(3.14f,0,3.14f,1,1,0,now);
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_TIMING);
  start();now+=5;Balance_Update(3.14f,0,3.14f,0,1,0,now);
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_SENSOR);
  start();now+=5;Balance_Update(3.14f,0,3.14f,1,1,1,now);
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_CONFLICT);
  start();g_swing_gain=NAN;tick(3.14f,0,0);
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_CONFIG);g_swing_gain=0.18f;
  g_swing_max_ms=500;start();
  for(i=0;i<99;i++){tick(3.14f,0,0);assert(g_bal_enable);}
  tick(3.14f,0,0);assert(!g_bal_enable && g_bal_reason==BAL_REASON_RUN_TIME);
  g_swing_max_ms=10000;start();
  for(i=0;i<41;i++)tick(3.14f,0,201);
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_SPEED);
  now=UINT32_MAX-150U;start();
  for(i=0;i<50;i++)tick(3.14f,-1,0);
  assert(g_bal_state==BAL_SWING && g_bal_run_ms==250);
  Balance_Stop(BAL_REASON_STOP);
  assert(!g_bal_enable && !g_bal_output && g_swing_phase==4 && !Balance_MotorOutput(0));
  puts("PASS: explicit bottom start/kick/slew; energy injection/removal and polarity; mean cart damping; no angle trip while swinging; predicted rate/speed capture -> existing auto-hold; travel/timing/sensor/conflict/NaN/timeout/overspeed/STOP and tick wrap");
  return 0;
}
