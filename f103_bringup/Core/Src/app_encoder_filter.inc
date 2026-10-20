/* F103 A-edge qualifier. Caller serializes task/ISR access. No waits or heap.
 * A candidate is counted only after its sampled level is stable for >=100us.
 * Short out-and-back pulses produce no counts; B is captured at the edge. */
volatile uint32_t g_enc_irq[APP_WHEEL_COUNT] = {0U};
volatile uint32_t g_enc_accepted[APP_WHEEL_COUNT] = {0U};
volatile uint32_t g_enc_glitch[APP_WHEEL_COUNT] = {0U};
volatile uint32_t g_enc_duplicate[APP_WHEEL_COUNT] = {0U};
static volatile uint8_t s_enc_stable_a[APP_WHEEL_COUNT];
static volatile int8_t s_enc_pending_step[APP_WHEEL_COUNT];
static volatile uint32_t s_enc_pending_since[APP_WHEEL_COUNT];
static volatile uint8_t s_enc_pending_mask;
static volatile uint32_t s_enc_qualify_cycles;
/* Accepted-edge timestamps only: raw IRQ glitches never set speed periods.
 * 36 bytes shared by task/ISR; no division or floating point in the ISR. */
static volatile uint32_t s_enc_speed_edge[APP_WHEEL_COUNT];
static volatile uint32_t s_enc_speed_period[APP_WHEEL_COUNT];
static volatile int8_t s_enc_speed_direction[APP_WHEEL_COUNT];

static void s_enc_filter_init(const uint8_t *levels, uint32_t cycles)
{
  uint8_t i;
  s_enc_pending_mask=0U;
  s_enc_qualify_cycles=cycles ? cycles : 1U;
  for(i=0U;i<APP_WHEEL_COUNT;i++)
  {
    s_enc_stable_a[i]=levels[i];
    s_enc_pending_step[i]=0;
    s_enc_pending_since[i]=0U;
    g_enc_irq[i]=0U; g_enc_accepted[i]=0U;
    g_enc_glitch[i]=0U; g_enc_duplicate[i]=0U;
    s_enc_speed_edge[i]=0U; s_enc_speed_period[i]=0U;
    s_enc_speed_direction[i]=0;
  }
}

static void s_enc_filter_commit(uint8_t wheel)
{
  int8_t direction=(int8_t)(s_enc_pending_step[wheel]*g_enc_sign[wheel]);
  uint32_t stamp=s_enc_pending_since[wheel];
  /* A first edge or a reversal cannot establish a velocity period. */
  s_enc_speed_period[wheel]=(direction==s_enc_speed_direction[wheel]) ?
                            stamp-s_enc_speed_edge[wheel] : 0U;
  s_enc_speed_edge[wheel]=stamp; s_enc_speed_direction[wheel]=direction;
  g_enc_count[wheel]+=(int32_t)direction;
  g_enc_accepted[wheel]++;
  s_enc_stable_a[wheel]^=1U;
  s_enc_pending_mask&=(uint8_t)~(1U<<wheel);
}

/* Caller holds the same encoder-IRQ critical section and samples now inside
 * it. Expiry is latched, so a 32-bit cycle-counter wrap cannot revive speed. */
static void s_enc_speed_snapshot(uint8_t wheel, uint32_t now, uint32_t timeout,
                                 uint32_t *period, uint32_t *age, int8_t *direction)
{
  *age=now-s_enc_speed_edge[wheel];
  if(s_enc_speed_direction[wheel] && *age>=timeout)
  {
    s_enc_speed_direction[wheel]=0; s_enc_speed_period[wheel]=0U;
  }
  *period=s_enc_speed_period[wheel]; *direction=s_enc_speed_direction[wheel];
}

/* Task-only arithmetic. Below 100 accepted edges/s use period measurement;
 * 100..300 edges/s blend continuously into per-cycle counts.  At standstill
 * stretch the last period by its age rather than holding a stale speed. */
static float s_enc_speed_hybrid(float count_rate, uint32_t period, uint32_t age,
                                int8_t direction, uint32_t clock_hz)
{
  float frequency, weight, period_rate;
  uint32_t effective;
  if(!direction || !period || !clock_hz || period>=clock_hz/4U) return count_rate;
  effective=(age>period) ? age : period;
  frequency=(float)clock_hz/(float)effective;
  period_rate=(float)direction*frequency;
  weight=(frequency-100.0f)/200.0f;
  if(weight<0.0f) weight=0.0f;
  if(weight>1.0f) weight=1.0f;
  return period_rate+(count_rate-period_rate)*weight;
}

static void s_enc_filter_edge(uint8_t wheel, uint8_t a, uint8_t b, uint32_t now)
{
  uint8_t mask=(uint8_t)(1U<<wheel);
  g_enc_irq[wheel]++;
  if(s_enc_pending_mask&mask)
  {
    uint32_t elapsed=now-s_enc_pending_since[wheel];
    if(a!=s_enc_stable_a[wheel])
    {
      /* Repeated IRQ sampled at the same A level is not another edge. */
      g_enc_duplicate[wheel]++;
      if(elapsed>=s_enc_qualify_cycles) s_enc_filter_commit(wheel);
      return;
    }
    if(elapsed<s_enc_qualify_cycles)
    {
      /* Return to the original level: reject both sides of this short pulse. */
      s_enc_pending_mask&=(uint8_t)~mask;
      g_enc_glitch[wheel]++;
      return;
    }
    s_enc_filter_commit(wheel);
  }
  if(a==s_enc_stable_a[wheel])
  {
    g_enc_duplicate[wheel]++;
    return;
  }
  s_enc_pending_since[wheel]=now;
  s_enc_pending_step[wheel]=(a!=b) ? 1 : -1;
  s_enc_pending_mask|=mask;
}

/* Called before the 5ms count snapshot, with encoder IRQs masked. The caller
 * skips any wheel with a pending EXTI flag so a real queued edge is not lost. */
static void s_enc_filter_flush(uint8_t wheel, uint8_t a, uint32_t now)
{
  uint8_t mask=(uint8_t)(1U<<wheel);
  if(!(s_enc_pending_mask&mask)) return;
  if(a==s_enc_stable_a[wheel])
  {
    s_enc_pending_mask&=(uint8_t)~mask;
    g_enc_glitch[wheel]++;
  }
  else if((uint32_t)(now-s_enc_pending_since[wheel])>=s_enc_qualify_cycles)
    s_enc_filter_commit(wheel);
}
