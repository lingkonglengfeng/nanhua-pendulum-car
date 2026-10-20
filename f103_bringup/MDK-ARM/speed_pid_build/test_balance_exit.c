#include "balance_control.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#define RAD 0.017453292519943295f
static uint32_t now;
static void sample(float raw_deg,float rpm)
{
  float raw[4]={rpm,0,0,0},feedback[4]={0,0,0,0};
  now+=5;Balance_Update(0,0,raw_deg*RAD,1,1,0,now);
  Balance_CheckMotionFeedback(raw,feedback,now);
}
static void activate(void)
{
  unsigned i;
  assert(Balance_Arm(1));sample(0,0);
  for(i=0;i<80;i++)sample(0,0);
  assert(g_bal_enable && g_bal_state==BAL_ACTIVE);
}
static void consume(void)
{
  BalanceExitEvent e;
  assert(Balance_PeekExit(&e));Balance_AckExit(e.ms);
  assert(!Balance_PeekExit(&e));
}
int main(void)
{
  unsigned i;
  uint32_t captured;
  BalanceExitEvent e,again;
  assert(sizeof(BalanceExitEvent)==28 && BAL_OVERSPEED_DWELL_MS==200);
  assert(g_bal_max_rpm==200 && !Balance_PeekExit(&e));
  activate();
  for(i=0;i<40;i++){sample(2.25f,201);assert(g_bal_enable && !Balance_PeekExit(&e));}
  assert(g_bal_overspeed_ms==195);
  sample(2.25f,200);assert(g_bal_enable && !g_bal_overspeed_ms); /* equality resets */
  for(i=0;i<40;i++){sample(2.25f,-201);assert(g_bal_enable);}
  sample(2.25f,-201);
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_SPEED);
  assert(Balance_PeekExit(&e));captured=e.ms;
  assert(e.reason==BAL_REASON_SPEED && e.overspeed_ms==200 && e.peak_rpm==201);
  assert(e.max_rpm==200 && e.ms==now && !e.coalesced);
  assert(fabsf(e.angle_deg-2.25f)<0.001f && g_bal_theta==0); /* raw, not filtered */
  assert(e.run_ms==410);
  Balance_Stop(BAL_REASON_STOP);activate();sample(-0.4f,0);
  assert(Balance_PeekExit(&again) && again.ms==captured && again.overspeed_ms==200);
  assert(again.reason==BAL_REASON_SPEED && fabsf(again.angle_deg-2.25f)<0.001f);
  sample(31,0);assert(!g_bal_enable && g_bal_reason==BAL_REASON_ANGLE);
  assert(Balance_PeekExit(&again) && again.ms==captured && again.coalesced==1);
  Balance_AckExit(captured+1);assert(Balance_PeekExit(&again));
  consume();
  activate();sample(-30.1f,0);
  assert(Balance_PeekExit(&e) && e.reason==BAL_REASON_ANGLE && e.ms==now);
  assert(fabsf(e.angle_deg+30.1f)<0.001f);consume();
  activate();now+=16;Balance_Update(0,0,0,1,1,0,now);
  assert(Balance_PeekExit(&e) && e.reason==BAL_REASON_TIMING && e.ms==now);consume();
  activate();now+=5;Balance_Update(0,0,0,0,1,0,now);
  assert(Balance_PeekExit(&e) && e.reason==BAL_REASON_SENSOR);consume();
  activate();Balance_Stop(BAL_REASON_STOP);
  assert(Balance_PeekExit(&e) && e.reason==BAL_REASON_STOP);consume();
  Balance_Stop(BAL_REASON_STOP);Balance_BeginZero();Balance_ZeroFailed();
  assert(!Balance_PeekExit(&e)); /* No fake ACTIVE-exit event from OFF/ZERO. */
  /* Disabling a partly accumulated overspeed dwell clears it immediately. */
  activate();for(i=0;i<40;i++)sample(0,201);
  assert(g_bal_overspeed_ms==195 && g_bal_enable);
  g_bal_max_rpm=0;sample(0,300);
  assert(g_bal_enable && !g_bal_overspeed_ms && g_bal_guard_rpm==300);
  for(i=0;i<100;i++)sample(0,(i&1U)?-300.0f:300.0f);
  assert(g_bal_enable && !g_bal_overspeed_ms && !Balance_PeekExit(&e));
  assert(Balance_SetSpeed(30) && Balance_SetSpeed(0)); /* Headroom not zero. */
  assert(Balance_StartStep(1,20));sample(0,300);assert(g_bal_enable);
  g_bal_max_rpm=200;
  for(i=0;i<40;i++){sample(0,201);assert(g_bal_enable);}
  sample(0,201);assert(!g_bal_enable && g_bal_reason==BAL_REASON_SPEED);consume();
  /* With no speed cutoff, raw angle and finite run-time exits still work. */
  g_bal_max_rpm=0;activate();sample(31,300);
  assert(!g_bal_enable && g_bal_reason==BAL_REASON_ANGLE);consume();
  g_bal_max_run_ms=500;activate();
  while(g_bal_enable)sample(0,300);
  assert(g_bal_reason==BAL_REASON_RUN_TIME);consume();g_bal_max_run_ms=0;
  /* Zero threshold must not make the independent 90RPM swing capture gate
   * reject a cart with a valid nonzero measured speed. */
  {
    float wheels[4]={20,20,20,20};
    now+=5;Balance_Update(3.1415926f,0,3.1415926f,1,1,0,now);
    assert(Balance_StartSwing(1,1,0));
    Balance_CheckMotionFeedback(wheels,wheels,now);
    now+=5;Balance_Update(0,0,0,1,1,0,now);
    assert(g_bal_state==BAL_ACTIVE && g_bal_enable);
    Balance_Stop(BAL_REASON_STOP);consume();
  }
  puts("PASS: zero threshold disables speed exit and clears dwell; motion headroom, re-enable dwell, angle/time exits and swing capture retained.");
  puts("PASS: >200RPM for 200ms (195ms not enough/equality resets), raw exit snapshot before reset, survives rearm, single pending/coalescing/ack, manual/angle/sensor/timing codes, no boot/off spam");
  return 0;
}
