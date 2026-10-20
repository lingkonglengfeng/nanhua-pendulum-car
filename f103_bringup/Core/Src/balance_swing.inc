/* ControlTask-owned swing-up. theta=0 at upright, +/-pi at bottom.
 * E=0.5*w^2/omega2+cos(theta); target 1.02 slightly exceeds upright energy.
 * Positive cart drive has opposite upright feedback polarity to bal_sign. */
/* Restore the successful UART baseline after the AS5600 replacement.
 * The earlier 35% / omega2=66 baseline passed the top too fast on this setup. */
volatile float g_swing_limit=0.37f,g_swing_travel_rev=2.0f,g_swing_gain=0.65f;
volatile float g_swing_omega2=63.5f,g_swing_energy=-1.0f,g_swing_capture_rate=2.0f;
volatile float g_swing_capture_deg=2.0f;
volatile uint32_t g_swing_max_ms=15000U;
volatile int8_t g_swing_kick_sign=1;
volatile uint8_t g_swing_phase=0U;

static uint8_t swing_configuration_valid(void)
{
  return (g_swing_limit>=0.10f && g_swing_limit<=0.60f &&
          g_swing_travel_rev>=0.5f && g_swing_travel_rev<=8.0f &&
          g_swing_gain>=0.01f && g_swing_gain<=0.8f &&
          g_swing_omega2>=10.0f && g_swing_omega2<=200.0f &&
          g_swing_capture_rate>=0.2f && g_swing_capture_rate<=4.0f &&
          g_swing_capture_deg>=0.5f && g_swing_capture_deg<=4.0f &&
          g_swing_max_ms>=500U && g_swing_max_ms<=15000U &&
          (g_swing_kick_sign==1 || g_swing_kick_sign==-1));
}

uint8_t Balance_StartSwing(uint8_t calibrated,uint8_t valid,float wheel_peak_rpm)
{
  if(!calibrated || !valid || !configuration_valid() ||
     !(motion_abs(g_bal_theta)>=2.618f && motion_abs(g_bal_theta)<=3.142f) ||
     !(motion_abs(g_bal_theta_dot)<=0.5f) ||
     !(wheel_peak_rpm>=0.0f && wheel_peak_rpm<=5.0f)) return 0U;
  Balance_Stop(BAL_REASON_STOP);
  g_bal_position_rev=0.0f;g_bal_run_ms=0U;
  g_bal_guard_rpm=0.0f;g_bal_overspeed_ms=0U;
  s_active_started=s_previous_ms;s_has_previous=1U;
  g_swing_phase=1U;g_swing_energy=-1.0f;
  g_bal_state=BAL_SWING;g_bal_enable=1U;
  return 1U;
}

static void swing_update(float theta,float rate,float dt)
{
  float cosine=cosf(theta),demand,phase,limit=g_swing_limit;
  if(limit>g_bal_limit) limit=g_bal_limit;
  g_swing_energy=0.5f*rate*rate/g_swing_omega2+cosine;
  if(g_bal_run_ms<150U)
    demand=(float)g_swing_kick_sign*limit*0.65f;
  else
  {
    g_swing_phase=2U;
    phase=clamp_symmetric(rate*cosine,1.0f);
    demand=-(float)g_bal_sign*g_swing_gain*(1.02f-g_swing_energy)*phase;
    /* Cart centering/damping: encoder turns and fresh signed mean wheel RPM.
     * This approximates acceleration control through PWM; needs real tuning. */
    demand-=0.08f*g_bal_position_rev+0.002f*g_bal_robust_rpm;
  }
  demand=clamp_symmetric(demand,limit);
  g_bal_output+=clamp_symmetric(demand-g_bal_output,3.0f*dt);
  g_bal_output=clamp_symmetric(g_bal_output,limit);
}
