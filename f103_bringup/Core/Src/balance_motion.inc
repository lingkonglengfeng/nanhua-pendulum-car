/* ControlTask-owned, nonblocking motion requests while BAL_ACTIVE only.
 * Position is signed average output-shaft revolutions, relative to capture or
 * the start of a sequence; metre requests use a configured rolling diameter.
 * Encoder travel does not measure an absolute-world pose or tyre slip. */
#define BAL_MOTION_ACCEL_RPM_S 10.0f
#define BAL_SEQ_POSITION_KP 20.0f
#define BAL_SEQ_POSITION_TOL 0.02f
#define BAL_SEQ_STOP_RPM 2.0f
#define BAL_SEQ_SETTLE_MS 400U
#define BAL_TURN_ACCEL_RPM_S 10.0f
#define BAL_TURN_MAX_RPM 10.0f
#define BAL_TURN_MAX_DUTY BAL_TURN_DUTY_LIMIT
#define BAL_TURN_TAU_S 0.050f

volatile float g_bal_vel_target_rpm=0.0f, g_bal_vel_command_rpm=0.0f;
volatile float g_bal_position_rev=0.0f, g_bal_seq_goal_rev=0.0f;
volatile float g_bal_pos_kp=20.0f, g_bal_pos_max_rpm=6.0f;
volatile uint8_t g_bal_seq_state=BAL_SEQ_NONE;
volatile uint32_t g_bal_seq_ms=0U;
static float s_seq_speed;
static uint32_t s_seq_pause_ms, s_seq_stage_started, s_seq_settle_started;
static uint8_t s_seq_settling;
/* User-measured installed wheel diameter; BALWHEEL can update it at runtime. */
volatile float g_bal_wheel_diameter_mm=65.0f;
static float s_step_goal_rev;
volatile float g_bal_turn_target_rpm=0.0f, g_bal_turn_command_rpm=0.0f;
volatile float g_bal_turn_rpm=0.0f, g_bal_turn_output=0.0f, g_bal_turn_kp=0.008f;
volatile float g_bal_turn_ki=0.010f, g_bal_turn_integral=0.0f;
static uint32_t s_turn_previous_ms;

static float motion_abs(float v) { return (v<0.0f) ? -v : v; }
static uint8_t motion_step_state(void)
{
  return (g_bal_seq_state>=BAL_SEQ_STEP_WAIT && g_bal_seq_state<=BAL_SEQ_STEP_END);
}
static float motion_limit(void)
{
  /* Disabling the exit threshold must not collapse all motion headroom to 0. */
  if(g_bal_max_rpm==0.0f) return 30.0f;
  float limit=g_bal_max_rpm*0.5f;
  return (limit<30.0f) ? limit : 30.0f;
}

static void motion_reset(void)
{
  g_bal_vel_target_rpm=0.0f; g_bal_vel_command_rpm=0.0f;
  g_bal_seq_state=BAL_SEQ_NONE; g_bal_seq_goal_rev=0.0f; g_bal_seq_ms=0U;
  s_seq_settling=0U;
  s_step_goal_rev=0.0f;
  g_bal_turn_target_rpm=0.0f; g_bal_turn_command_rpm=0.0f;
  g_bal_turn_rpm=0.0f; g_bal_turn_output=0.0f; s_turn_previous_ms=0U;
  g_bal_turn_integral=0.0f;
  /* Preserve measured position after a trip for Watch; ARM resets the origin. */
}

static uint8_t motion_configuration_valid(void)
{
  float limit=motion_limit();
  return (g_bal_vel_target_rpm>=-limit && g_bal_vel_target_rpm<=limit &&
          g_bal_vel_command_rpm>=-limit && g_bal_vel_command_rpm<=limit &&
          g_bal_position_rev>=-1000.0f && g_bal_position_rev<=1000.0f &&
          g_bal_seq_goal_rev>=-1000.0f && g_bal_seq_goal_rev<=1000.0f &&
          (g_bal_seq_state==BAL_SEQ_HOLD || g_bal_seq_state==BAL_SEQ_SPIN || motion_step_state() ||
            (g_bal_seq_goal_rev>=-0.5f && g_bal_seq_goal_rev<=0.5f)) &&
          g_bal_seq_state<=BAL_SEQ_STEP_END &&
          (!motion_step_state() ||
            (g_bal_wheel_diameter_mm>=20.0f && g_bal_wheel_diameter_mm<=300.0f &&
             s_step_goal_rev>=-1000.0f && s_step_goal_rev<=1000.0f)) &&
          g_bal_pos_kp>=0.1f && g_bal_pos_kp<=60.0f &&
          g_bal_pos_max_rpm>=1.0f && g_bal_pos_max_rpm<=15.0f &&
          g_bal_turn_target_rpm>=-BAL_TURN_MAX_RPM && g_bal_turn_target_rpm<=BAL_TURN_MAX_RPM &&
          g_bal_turn_command_rpm>=-BAL_TURN_MAX_RPM && g_bal_turn_command_rpm<=BAL_TURN_MAX_RPM &&
          g_bal_turn_rpm>=-100000.0f && g_bal_turn_rpm<=100000.0f &&
          g_bal_turn_output>=-BAL_TURN_MAX_DUTY && g_bal_turn_output<=BAL_TURN_MAX_DUTY &&
          g_bal_turn_kp>=0.0f && g_bal_turn_kp<=0.02f &&
          g_bal_turn_ki>=0.0f && g_bal_turn_ki<=0.05f &&
          g_bal_turn_integral>=-BAL_TURN_MAX_DUTY && g_bal_turn_integral<=BAL_TURN_MAX_DUTY &&
          motion_abs(g_bal_vel_target_rpm)+motion_abs(g_bal_turn_target_rpm)<=limit &&
          (g_bal_seq_state==BAL_SEQ_NONE || g_bal_seq_state==BAL_SEQ_SPIN ||
            g_bal_turn_target_rpm==0.0f) &&
          (g_bal_vel_enable || (g_bal_vel_target_rpm==0.0f &&
            g_bal_vel_command_rpm==0.0f && g_bal_seq_state==BAL_SEQ_NONE)) &&
          (g_bal_seq_state==BAL_SEQ_NONE || g_bal_seq_state==BAL_SEQ_HOLD || g_bal_seq_state==BAL_SEQ_SPIN ||
            (s_seq_speed>=1.0f && s_seq_speed<=20.0f && s_seq_speed<=limit &&
             s_seq_pause_ms>=400U && s_seq_pause_ms<=5000U)));
}

uint8_t Balance_SetSpeed(float rpm)
{
  float limit=motion_limit();
  if(g_bal_state!=BAL_ACTIVE || !g_bal_enable || !g_bal_vel_enable ||
     !(rpm>=-limit && rpm<=limit) ||
     motion_abs(rpm)+motion_abs(g_bal_turn_target_rpm)>limit) return 0U;
  g_bal_seq_state=BAL_SEQ_NONE; g_bal_seq_ms=0U; g_bal_seq_goal_rev=0.0f;
  s_seq_settling=0U; g_bal_vel_target_rpm=rpm;
  /* Keep current ramp and PI history, including a zero-speed bias. */
  return 1U;
}

static uint8_t position_ready(void)
{
  return (g_bal_state==BAL_ACTIVE && g_bal_enable && g_bal_vel_enable &&
          g_bal_turn_target_rpm==0.0f && g_bal_turn_command_rpm==0.0f &&
          g_bal_position_rev>=-1000.0f && g_bal_position_rev<=1000.0f);
}

static void position_begin(float goal)
{
  g_bal_seq_goal_rev=goal; g_bal_seq_state=BAL_SEQ_HOLD; g_bal_seq_ms=0U;
  s_seq_stage_started=s_previous_ms; s_seq_settling=0U;
  /* Preserve ramp, velocity PI bias, encoder counts, and ACTIVE run time. */
}

uint8_t Balance_SetPosition(float turns)
{
  if(!position_ready() || !(turns>=-5.0f && turns<=5.0f)) return 0U;
  position_begin(turns);
  return 1U;
}

uint8_t Balance_HoldPosition(uint8_t zero_origin)
{
  if(!position_ready() || zero_origin>1U) return 0U;
  if(zero_origin) g_bal_position_rev=0.0f;
  position_begin(g_bal_position_rev);
  return 1U;
}

static void turn_set_target(float rpm)
{
  if(rpm==0.0f || rpm*g_bal_turn_target_rpm<0.0f) g_bal_turn_integral=0.0f;
  g_bal_turn_target_rpm=rpm;
}

uint8_t Balance_StartSpin(float rpm)
{
  float room=motion_limit()-motion_abs(rpm);
  if(g_bal_state!=BAL_ACTIVE || !g_bal_enable || !g_bal_vel_enable ||
     !(rpm>=-BAL_TURN_MAX_RPM && rpm<=BAL_TURN_MAX_RPM) || !(room>=0.0f) ||
     !(g_bal_position_rev>=-1000.0f && g_bal_position_rev<=1000.0f)) return 0U;
  if(g_bal_seq_state!=BAL_SEQ_SPIN)
  {
    position_begin(g_bal_position_rev);
    g_bal_seq_state=BAL_SEQ_SPIN; g_bal_vel_target_rpm=0.0f;
  }
  /* Preserve origin, speed/yaw ramps and established speed PI bias. Repeated
   * remote updates or SPIN 0 must not silently move the position anchor. */
  g_bal_vel_target_rpm=clamp_symmetric(g_bal_vel_target_rpm,room);
  turn_set_target(rpm);
  return 1U;
}

uint8_t Balance_StartSequence(float rpm, float turns, uint32_t pause_ms)
{
  float speed=motion_abs(rpm);
  if(g_bal_state!=BAL_ACTIVE || !g_bal_enable || !g_bal_vel_enable ||
     !(speed>=1.0f && speed<=20.0f && speed<=motion_limit()) ||
     g_bal_turn_target_rpm!=0.0f || g_bal_turn_command_rpm!=0.0f ||
     !(turns>=0.05f && turns<=0.5f) || pause_ms<400U || pause_ms>5000U)
    return 0U;
  s_seq_speed=speed; s_seq_pause_ms=pause_ms;
  g_bal_position_rev=0.0f;
  g_bal_seq_goal_rev=(rpm<0.0f) ? -turns : turns;
  g_bal_seq_state=BAL_SEQ_OUT; g_bal_seq_ms=0U;
  s_seq_stage_started=s_previous_ms; s_seq_settling=0U;
  /* Do not reset the independent ACTIVE timeout or PI history. */
  return 1U;
}

uint8_t Balance_StartStep(float metres, float max_rpm)
{
  float goal;
  if(!position_ready() || motion_step_state() ||
     !(g_bal_wheel_diameter_mm>=20.0f && g_bal_wheel_diameter_mm<=300.0f) ||
     !(metres>=-2.0f && metres<=2.0f) || motion_abs(metres)<0.05f ||
     !(max_rpm>=1.0f && max_rpm<=20.0f && max_rpm<=motion_limit())) return 0U;
  goal=g_bal_position_rev+metres*1000.0f/(3.14159265359f*g_bal_wheel_diameter_mm);
  if(!(goal>=-1000.0f && goal<=1000.0f)) return 0U;
  /* Preserve encoder origin, velocity PI history and the ACTIVE time budget. */
  position_begin(g_bal_position_rev);
  s_step_goal_rev=goal; s_seq_speed=max_rpm; s_seq_pause_ms=5000U;
  g_bal_seq_state=BAL_SEQ_STEP_WAIT;
  return 1U;
}

uint8_t Balance_SetTurn(float rpm)
{
  if(g_bal_state!=BAL_ACTIVE || !g_bal_enable || !g_bal_vel_enable ||
     !(rpm>=-BAL_TURN_MAX_RPM && rpm<=BAL_TURN_MAX_RPM) ||
     motion_abs(g_bal_vel_target_rpm)+motion_abs(rpm)>motion_limit()) return 0U;
  if(rpm!=0.0f && g_bal_seq_state!=BAL_SEQ_NONE)
  {
    /* An encoder-only straight return cannot survive a heading change. */
    g_bal_seq_state=BAL_SEQ_NONE; g_bal_seq_ms=0U; g_bal_seq_goal_rev=0.0f;
    g_bal_vel_target_rpm=0.0f; s_seq_settling=0U;
  }
  /* No accumulated torque survives neutral or a direction reversal. */
  turn_set_target(rpm);
  return 1U;
}

static void turn_update(const volatile float *wheels, uint32_t now)
{
  uint32_t elapsed=now-s_turn_previous_ms;
  float dt, measured, room, error, increment, candidate, demand;
  if(!g_bal_vel_enable) { g_bal_turn_output=0.0f; g_bal_turn_integral=0.0f; return; }
  if(s_turn_previous_ms==0U || elapsed==0U || elapsed>15U) elapsed=5U;
  s_turn_previous_ms=now; dt=(float)elapsed*0.001f;
  /* Physical motor order: 0 RR, 1 RF, 2 LR, 3 LF. */
  measured=(wheels[2]+wheels[3]-wheels[0]-wheels[1])*0.25f;
  g_bal_turn_rpm+=dt/(BAL_TURN_TAU_S+dt)*(measured-g_bal_turn_rpm);
  g_bal_turn_command_rpm+=clamp_symmetric(g_bal_turn_target_rpm-g_bal_turn_command_rpm,
                                         BAL_TURN_ACCEL_RPM_S*dt);
  /* Reserve the common balance output first. Integral tracks available room,
   * and outward integration freezes at either yaw or balance saturation. */
  room=g_bal_limit-motion_abs(g_bal_output);
  if(room<0.0f) room=0.0f;
  if(room>BAL_TURN_MAX_DUTY) room=BAL_TURN_MAX_DUTY;
  error=g_bal_turn_command_rpm-g_bal_turn_rpm;
  g_bal_turn_integral=clamp_symmetric(g_bal_turn_integral,room);
  if(g_bal_turn_ki==0.0f || g_bal_turn_target_rpm==0.0f ||
     g_bal_turn_command_rpm*g_bal_turn_target_rpm<=0.0f)
    g_bal_turn_integral=0.0f;
  else
  {
    increment=g_bal_turn_ki*error*dt;
    candidate=clamp_symmetric(g_bal_turn_integral+increment,room);
    demand=g_bal_turn_kp*error+candidate;
    if(!((demand>room && increment>0.0f) || (demand<-room && increment<0.0f)))
      g_bal_turn_integral=candidate;
  }
  g_bal_turn_output=clamp_symmetric(g_bal_turn_kp*error+g_bal_turn_integral,room);
}

float Balance_MotorOutput(uint8_t wheel)
{
  float turn, room;
  if(wheel>=4U || !g_bal_enable) return 0.0f;
  if(g_bal_state==BAL_SWING) return g_bal_output; /* straight pumping only */
  if(g_bal_state!=BAL_ACTIVE) return 0.0f;
  room=g_bal_limit-motion_abs(g_bal_output);
  if(room<0.0f) room=0.0f;
  turn=clamp_symmetric(g_bal_turn_output,room);
  /* Existing per-motor hardware polarity is applied downstream. */
  return g_bal_output+((wheel>=2U) ? turn : -turn);
}

void Balance_AddPosition(float delta_rev)
{
  float position;
  if((g_bal_state!=BAL_ACTIVE && g_bal_state!=BAL_SWING) || !g_bal_enable) return;
  position=g_bal_position_rev+delta_rev;
  if(!(delta_rev>=-1.0f && delta_rev<=1.0f &&
       position>=-1000.0f && position<=1000.0f))
  {
    /* Rejected by the existing configuration gate before computing output. */
    g_bal_position_rev=1001.0f;
    return;
  }
  g_bal_position_rev=position;
}

static uint8_t motion_settled(float error, uint32_t now, uint32_t required)
{
  if(motion_abs(error)>BAL_SEQ_POSITION_TOL ||
     motion_abs(g_bal_vel_rpm)>BAL_SEQ_STOP_RPM)
  {
    s_seq_settling=0U; return 0U;
  }
  if(!s_seq_settling)
  {
    s_seq_settling=1U; s_seq_settle_started=now;
  }
  return ((uint32_t)(now-s_seq_settle_started)>=required) ? 1U : 0U;
}

static void motion_update(float dt, uint32_t now)
{
  float step=BAL_MOTION_ACCEL_RPM_S*dt;
  if(motion_step_state())
  {
    float error=g_bal_seq_goal_rev-g_bal_position_rev;
    float cap;
    uint8_t advance;
    g_bal_seq_ms=now-s_seq_stage_started;
    /* Scheduled waits are elapsed time, not continuous zero-speed dwell:
     * real balance corrections must not prevent the cart ever departing.
     * Arrival still requires the original position/speed settle check. */
    advance=(g_bal_seq_state==BAL_SEQ_STEP_MOVE) ?
      motion_settled(error,now,BAL_SEQ_SETTLE_MS) : (g_bal_seq_ms>=5000U);
    if(advance)
    {
      if(g_bal_seq_state==BAL_SEQ_STEP_WAIT)
      {
        g_bal_seq_goal_rev=s_step_goal_rev; g_bal_seq_state=BAL_SEQ_STEP_MOVE;
      }
      else if(g_bal_seq_state==BAL_SEQ_STEP_MOVE) g_bal_seq_state=BAL_SEQ_STEP_END;
      else g_bal_seq_state=BAL_SEQ_HOLD;
      s_seq_stage_started=now; g_bal_seq_ms=0U; s_seq_settling=0U;
    }
    cap=(g_bal_seq_state==BAL_SEQ_STEP_MOVE) ? s_seq_speed : g_bal_pos_max_rpm;
    if(cap>motion_limit()) cap=motion_limit();
    error=g_bal_seq_goal_rev-g_bal_position_rev;
    g_bal_vel_target_rpm=clamp_symmetric(g_bal_pos_kp*error,cap);
  }
  else if(g_bal_seq_state==BAL_SEQ_HOLD || g_bal_seq_state==BAL_SEQ_SPIN)
  {
    float cap=g_bal_pos_max_rpm;
    float room=motion_limit();
    if(g_bal_seq_state==BAL_SEQ_SPIN) room-=motion_abs(g_bal_turn_target_rpm);
    if(room<0.0f) room=0.0f;
    if(cap>room) cap=room;
    g_bal_seq_ms=now-s_seq_stage_started;
    g_bal_vel_target_rpm=clamp_symmetric(g_bal_pos_kp*(g_bal_seq_goal_rev-g_bal_position_rev),cap);
  }
  else if(g_bal_seq_state!=BAL_SEQ_NONE)
  {
    float error=g_bal_seq_goal_rev-g_bal_position_rev;
    uint32_t settle=(g_bal_seq_state==BAL_SEQ_PAUSE) ? s_seq_pause_ms : BAL_SEQ_SETTLE_MS;
    g_bal_seq_ms=now-s_seq_stage_started;
    if(g_bal_seq_state!=BAL_SEQ_HOME && motion_settled(error,now,settle))
    {
      if(g_bal_seq_state==BAL_SEQ_OUT) g_bal_seq_state=BAL_SEQ_PAUSE;
      else if(g_bal_seq_state==BAL_SEQ_PAUSE)
      {
        g_bal_seq_state=BAL_SEQ_RETURN; g_bal_seq_goal_rev=0.0f;
      }
      else g_bal_seq_state=BAL_SEQ_HOME;
      s_seq_stage_started=now; g_bal_seq_ms=0U; s_seq_settling=0U;
    }
    error=g_bal_seq_goal_rev-g_bal_position_rev;
    g_bal_vel_target_rpm=clamp_symmetric(BAL_SEQ_POSITION_KP*error,s_seq_speed);
  }
  g_bal_vel_command_rpm+=clamp_symmetric(g_bal_vel_target_rpm-g_bal_vel_command_rpm,step);
}
