/* F103 private implementation, included by app_vehicle.c. Fixed-pin PWM using
 * TIM1_UP -> DMA1 CH5 -> GPIOA and TIM4_UP -> DMA1 CH7 -> GPIOB.
 * TIM4 resets from TIM1 ITR0, so both streams advance on the same time grid.
 * USART3 keeps DMA1 channels 2/3; HAL tick keeps TIM2. */
#define MOTOR_SLOTS APP_MOTOR_PWM_SLOTS
#define MOTOR_DMA_WORDS (2U * MOTOR_SLOTS)
#define MOTOR_GPIO_A_MASK 0x0F0CU
#define MOTOR_GPIO_B_MASK 0xC000U
volatile uint8_t g_motor_on_ticks[4],g_motor_in1_ticks[4],g_motor_in2_ticks[4];
volatile uint32_t g_motor_pwm_hz,g_motor_dma_periods,g_motor_dma_errors;
volatile uint32_t g_motor_dma_late,g_motor_dma_max_cycles;
static volatile uint32_t s_motor_dma_a[MOTOR_DMA_WORDS];
static volatile uint32_t s_motor_dma_b[MOTOR_DMA_WORDS];
static volatile uint32_t s_motor_wave_a[2][MOTOR_SLOTS];
static volatile uint32_t s_motor_wave_b[2][MOTOR_SLOTS];
static volatile uint32_t s_motor_wave_generation;
static volatile uint8_t s_motor_wave_bank;
static volatile uint32_t s_motor_half_generation[2];
static float s_motor_requested[4],s_motor_fraction[4]; /* ControlTask only */
static int8_t s_motor_previous_sign[4];
static uint8_t s_motor_previous_ticks[4],s_motor_previous_dir[4];

static uint32_t s_motor_bsrr(uint32_t high, uint32_t mask)
{
  return high | ((mask & ~high) << 16);
}

static void s_motor_wave_build(uint8_t bank, const uint8_t *ticks, const uint8_t *reverse)
{
  uint32_t slot;
  uint8_t w;
  for (slot = 0U; slot < MOTOR_SLOTS; slot++)
  {
    uint32_t high_a = 0U, high_b = 0U;
    for (w = 0U; w < 4U; w++)
    {
      uint32_t in1 = 0U, in2 = 0U;
      if (ticks[w] != 0U)
      {
        /* Off-time=11 brake in BOTH directions. Zero command=00 coast. */
        in1 = ((slot >= ticks[w]) || (reverse[w] == 0U)) ? 1U : 0U;
        in2 = ((slot >= ticks[w]) || (reverse[w] != 0U)) ? 1U : 0U;
      }
      if (in1 != 0U) { high_a |= 1UL << (8U + w); }
      if (in2 != 0U)
      {
        if (w < 2U) { high_a |= 1UL << (2U + w); }
        else { high_b |= 1UL << (12U + w); }
      }
    }
    s_motor_wave_a[bank][slot] = s_motor_bsrr(high_a, MOTOR_GPIO_A_MASK);
    s_motor_wave_b[bank][slot] = s_motor_bsrr(high_b, MOTOR_GPIO_B_MASK);
  }
}

static uint8_t s_motor_quantize(uint8_t w, float u)
{
  float magnitude, scaled;
  uint8_t ticks;
  int8_t sign = (u < 0.0f) ? -1 : ((u > 0.0f) ? 1 : 0);
  if (sign != s_motor_previous_sign[w]) { s_motor_fraction[w] = 0.0f; }
  s_motor_previous_sign[w] = sign;
  if (sign == 0) { s_motor_fraction[w] = 0.0f; return 0U; }
  magnitude = (u < 0.0f) ? -u : u;
  if (magnitude >= 1.0f) { s_motor_fraction[w] = 0.0f; return MOTOR_SLOTS; }
  /* Carry fractional slots between 5ms commands, preserving average duty. */
  scaled = magnitude * (float)MOTOR_SLOTS + s_motor_fraction[w];
  ticks = (uint8_t)scaled;
  s_motor_fraction[w] = scaled - (float)ticks;
  return ticks;
}

static void s_motor_dma_commit(void)
{
  uint8_t ticks[4],reverse[4],changed = 0U,w;
  uint8_t bank = s_motor_wave_bank ^ 1U;
  for (w = 0U; w < 4U; w++)
  {
    ticks[w] = s_motor_quantize(w, s_motor_requested[w]);
    reverse[w] = (s_motor_requested[w] < 0.0f) ? 1U : 0U;
    if (ticks[w] != s_motor_previous_ticks[w] || reverse[w] != s_motor_previous_dir[w]) { changed = 1U; }
    s_motor_previous_ticks[w] = ticks[w]; s_motor_previous_dir[w] = reverse[w];
    g_motor_on_ticks[w] = ticks[w];
    g_motor_in1_ticks[w] = (ticks[w] == 0U) ? 0U :
                          ((reverse[w] == 0U) ? MOTOR_SLOTS : MOTOR_SLOTS - ticks[w]);
    g_motor_in2_ticks[w] = (ticks[w] == 0U) ? 0U :
                          ((reverse[w] != 0U) ? MOTOR_SLOTS : MOTOR_SLOTS - ticks[w]);
    /* Legacy observations, not TIM1 compare registers in this implementation. */
    g_bal_ccr[w] = (uint16_t)(((uint32_t)g_motor_in1_ticks[w] * 3599U + 32U) / MOTOR_SLOTS);
    g_bal_dir[w] = reverse[w];
  }
  if (changed != 0U)
  {
    s_motor_wave_build(bank, ticks, reverse);
    {
      uint32_t saved = __get_PRIMASK();
      __disable_irq();
      __DMB(); s_motor_wave_bank = bank; s_motor_wave_generation++;
      __set_PRIMASK(saved);
    }
  }
}

static uint8_t s_motor_half_safe(uint8_t half, uint32_t ndtr_a, uint32_t ndtr_b)
{
  /* Leave at least 32 slots (~25us) before either DMA reader reaches this half. */
  if (half == 0U) { return (ndtr_a > 32U && ndtr_a <= MOTOR_SLOTS &&
                            ndtr_b > 32U && ndtr_b <= MOTOR_SLOTS) ? 1U : 0U; }
  return (ndtr_a > MOTOR_SLOTS + 32U && ndtr_a <= MOTOR_DMA_WORDS &&
          ndtr_b > MOTOR_SLOTS + 32U && ndtr_b <= MOTOR_DMA_WORDS) ? 1U : 0U;
}

static void s_motor_refill(uint8_t half, uint32_t ndtr_a, uint32_t ndtr_b)
{
  uint32_t i, offset = (uint32_t)half * MOTOR_SLOTS;
  uint32_t generation = s_motor_wave_generation;
  uint8_t bank = s_motor_wave_bank;
  if (s_motor_half_generation[half] == generation) { return; }
  if (s_motor_half_safe(half, ndtr_a, ndtr_b) == 0U) { g_motor_dma_late++; return; }
  for (i = 0U; i < MOTOR_SLOTS; i++)
  {
    s_motor_dma_a[offset+i] = s_motor_wave_a[bank][i];
    s_motor_dma_b[offset+i] = s_motor_wave_b[bank][i];
  }
  __DMB();
  s_motor_half_generation[half] = generation;
}
/* MOTOR_WAVE_HOST_END */

static void s_motor_dma_fault(void)
{
  TIM1->DIER = 0U; TIM4->DIER = 0U;
  TIM1->CR1 = 0U; TIM4->CR1 = 0U;
  DMA1_Channel5->CCR &= ~DMA_CCR_EN; DMA1_Channel7->CCR &= ~DMA_CCR_EN;
  GPIOA->BSRR = MOTOR_GPIO_A_MASK << 16;
  GPIOB->BSRR = MOTOR_GPIO_B_MASK << 16;
  g_motor_dma_errors++;
}

void DMA1_Channel5_IRQHandler(void)
{
  if ((DMA1->ISR & DMA_ISR_TEIF5) != 0U)
  {
    DMA1->IFCR = DMA_IFCR_CGIF5;
    s_motor_dma_fault();
  }
}

void DMA1_Channel7_IRQHandler(void)
{
  uint32_t started = DWT->CYCCNT;
  uint32_t flags = DMA1->ISR;
  if ((flags & (DMA_ISR_TEIF5 | DMA_ISR_TEIF7)) != 0U)
  {
    DMA1->IFCR = DMA_IFCR_CGIF5 | DMA_IFCR_CGIF7;
    s_motor_dma_fault(); return;
  }
  if ((flags & DMA_ISR_HTIF7) != 0U)
  {
    DMA1->IFCR = DMA_IFCR_CHTIF7 | DMA_IFCR_CHTIF5;
    s_motor_refill(0U, DMA1_Channel5->CNDTR, DMA1_Channel7->CNDTR);
  }
  if ((flags & DMA_ISR_TCIF7) != 0U)
  {
    DMA1->IFCR = DMA_IFCR_CTCIF7 | DMA_IFCR_CTCIF5;
    s_motor_refill(1U, DMA1_Channel5->CNDTR, DMA1_Channel7->CNDTR);
    g_motor_dma_periods += 2U;
  }
  started = DWT->CYCCNT - started;
  if (started > g_motor_dma_max_cycles) { g_motor_dma_max_cycles = started; }
}

static void s_motor_dma_init(void)
{
  uint32_t i;
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
  TIM1->CR1 = 0U; TIM1->DIER = 0U; TIM1->CCER = 0U;
  TIM1->PSC = 0U; TIM1->ARR = 55U; TIM1->RCR = 0U;
  TIM1->CR2 = (2U << 4); /* update TRGO -> TIM4 ITR0 */
  TIM1->EGR = TIM_EGR_UG; TIM1->SR = 0U;
  TIM4->CR1 = 0U; TIM4->DIER = 0U; TIM4->SMCR = 0U;
  TIM4->PSC = 0U; TIM4->ARR = 65535U;
  TIM4->EGR = TIM_EGR_UG; TIM4->SR = 0U;
  TIM4->SMCR = 4U; /* ITR0, reset mode; URS=0 permits reset-event DMA */
  for (i = 0U; i < MOTOR_DMA_WORDS; i++)
  {
    s_motor_dma_a[i] = MOTOR_GPIO_A_MASK << 16;
    s_motor_dma_b[i] = MOTOR_GPIO_B_MASK << 16;
  }
  DMA1_Channel5->CCR = 0U; DMA1_Channel7->CCR = 0U;
  DMA1->IFCR = DMA_IFCR_CGIF5 | DMA_IFCR_CGIF7;
  DMA1_Channel5->CPAR = (uint32_t)&GPIOA->BSRR;
  DMA1_Channel7->CPAR = (uint32_t)&GPIOB->BSRR;
  DMA1_Channel5->CMAR = (uint32_t)s_motor_dma_a;
  DMA1_Channel7->CMAR = (uint32_t)s_motor_dma_b;
  DMA1_Channel5->CNDTR = MOTOR_DMA_WORDS;
  DMA1_Channel7->CNDTR = MOTOR_DMA_WORDS;
  DMA1_Channel5->CCR = DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_CIRC |
      DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1 | DMA_CCR_PL_1 | DMA_CCR_TEIE;
  DMA1_Channel7->CCR = DMA1_Channel5->CCR | DMA_CCR_HTIE | DMA_CCR_TCIE;
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 4U, 0U);
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 4U, 0U);
  HAL_NVIC_ClearPendingIRQ(DMA1_Channel5_IRQn); HAL_NVIC_ClearPendingIRQ(DMA1_Channel7_IRQn);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn); HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  g_motor_pwm_hz = HAL_RCC_GetHCLKFreq() / (56U * MOTOR_SLOTS);
  __DMB();
  DMA1_Channel5->CCR |= DMA_CCR_EN; DMA1_Channel7->CCR |= DMA_CCR_EN;
  TIM4->DIER = TIM_DIER_UDE; TIM4->CR1 = TIM_CR1_CEN;
  TIM1->DIER = TIM_DIER_UDE; TIM1->CR1 = TIM_CR1_CEN;
}
