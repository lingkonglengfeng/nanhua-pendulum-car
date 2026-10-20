#include "pendulum_angle.h"
#include <string.h>

#define COUNTS_PER_REV 4096
#define RAD_PER_COUNT (6.2831853071795864769f / 4096.0f)

/* Shortest signed displacement: -2048..2047, also correct at 4095/0. */
static int16_t count_difference(uint16_t current, uint16_t reference)
{
  int32_t difference = (int32_t)current - (int32_t)reference;
  if (difference >= 2048) difference -= COUNTS_PER_REV;
  if (difference < -2048) difference += COUNTS_PER_REV;
  return (int16_t)difference;
}

static void invalidate(PendulumAngle *state)
{
  state->valid = 0U;
  state->rate_rad_s = 0.0f;
  state->derivative_ready = 0U;
}

void PendulumAngle_Init(PendulumAngle *state, uint16_t zero,
                        int8_t sign, uint8_t calibrated)
{
  memset(state, 0, sizeof(*state));
  state->sign = 1;
  if (zero > 4095U || (sign != 1 && sign != -1) || calibrated > 1U) {
    state->zero_result = PENDULUM_ZERO_BAD_CONFIG;
    return;
  }
  state->upright_raw = zero;
  state->bottom_raw = (uint16_t)((zero + 2048U) % 4096U);
  state->sign = sign;
  state->calibrated = calibrated;
}

uint8_t PendulumAngle_SetCalibration(PendulumAngle *state, uint16_t zero,
                                     int8_t sign)
{
  if (zero > 4095U || (sign != 1 && sign != -1)) return 0U;
  state->upright_raw = zero;
  state->bottom_raw = (uint16_t)((zero + 2048U) % 4096U);
  state->sign = sign;
  state->calibrated = 1U;
  state->zero_count = 0U;
  state->zero_result = PENDULUM_ZERO_IDLE;
  invalidate(state);
  return 1U;
}

void PendulumAngle_BeginZero(PendulumAngle *state)
{
  /* ZERO now samples the freely hanging BOTTOM; upright is half a turn away. */
  state->calibrated = 0U;
  state->zero_count = 0U;
  state->zero_sum = 0;
  state->zero_min = 0;
  state->zero_max = 0;
  state->zero_result = PENDULUM_ZERO_COLLECTING;
  invalidate(state);
}

uint8_t PendulumAngle_SetSign(PendulumAngle *state, int8_t sign)
{
  if (sign != 1 && sign != -1) return 0U;
  state->sign = sign;
  invalidate(state);
  return 1U;
}

void PendulumAngle_CheckFreshness(PendulumAngle *state, uint32_t now_ms)
{
  if (state->has_valid_sample != 0U &&
      (uint32_t)(now_ms - state->last_valid_ms) > PENDULUM_STALE_MS) {
    invalidate(state);
    if (state->zero_result == PENDULUM_ZERO_COLLECTING)
      state->zero_result = PENDULUM_ZERO_BAD_SAMPLE;
  }
}

static void collect_zero(PendulumAngle *state, uint16_t raw)
{
  int16_t difference;
  int32_t average;
  int32_t zero;
  if (state->zero_count == 0U) state->zero_reference = raw;
  difference = count_difference(raw, state->zero_reference);
  if (difference < state->zero_min) state->zero_min = difference;
  if (difference > state->zero_max) state->zero_max = difference;
  if (state->zero_max - state->zero_min > PENDULUM_ZERO_MAX_SPAN) {
    state->zero_result = PENDULUM_ZERO_MOVED;
    return;
  }
  /* Samples are a stationary cluster. Unwrap about the first sample before
   * averaging, instead of averaging 4095 and 0 as half a revolution. */
  state->zero_sum += difference;
  state->zero_count++;
  if (state->zero_count >= PENDULUM_ZERO_SAMPLES) {
    average = state->zero_sum >= 0 ?
      (state->zero_sum + (int32_t)state->zero_count / 2) / state->zero_count :
      (state->zero_sum - (int32_t)state->zero_count / 2) / state->zero_count;
    zero = (int32_t)state->zero_reference + average;
    if (zero < 0) zero += COUNTS_PER_REV;
    if (zero >= COUNTS_PER_REV) zero -= COUNTS_PER_REV;
    state->bottom_raw = (uint16_t)zero;
    state->upright_raw = (uint16_t)((zero + 2048) % COUNTS_PER_REV);
    state->calibrated = 1U;
    state->zero_result = PENDULUM_ZERO_SUCCESS;
  }
}

void PendulumAngle_Update(PendulumAngle *state, uint16_t raw, uint8_t status,
                          uint8_t comm_ok, uint32_t now_ms)
{
  int32_t signed_counts;
  uint32_t elapsed_ms;
  float dt;
  float rate;
  float alpha;
  PendulumAngle_CheckFreshness(state, now_ms);
  state->last_sample_ms = now_ms;
  state->comm_ok = comm_ok != 0U;
  state->magnet_status = status;
  if (comm_ok == 0U || raw > 4095U || (status & 0x20U) == 0U ||
      (status & 0x18U) != 0U) {
    invalidate(state);
    if (state->zero_result == PENDULUM_ZERO_COLLECTING)
      state->zero_result = PENDULUM_ZERO_BAD_SAMPLE;
    return;
  }
  /* Never count duplicate timestamps as additional calibration samples or
   * divide by zero. Recovery waits for a sample with a fresh timestamp. */
  if (state->has_valid_sample != 0U && now_ms == state->last_valid_ms) return;
  state->last_valid_ms = now_ms;
  state->has_valid_sample = 1U;
  if (state->zero_result == PENDULUM_ZERO_COLLECTING) {
    collect_zero(state, raw);
    invalidate(state);
    return;
  }
  signed_counts = (int32_t)state->sign * count_difference(raw, state->upright_raw);
  if (signed_counts >= 2048) signed_counts -= COUNTS_PER_REV;
  if (signed_counts < -2048) signed_counts += COUNTS_PER_REV;
  state->angle_rad = signed_counts * RAD_PER_COUNT;
  if (state->calibrated == 0U) {
    invalidate(state);
    return;
  }
  if (state->derivative_ready != 0U) {
    elapsed_ms = (uint32_t)(now_ms - state->previous_ms);
    if (elapsed_ms == 0U) return;
    dt = (float)elapsed_ms * 0.001f;
    rate = (float)state->sign * count_difference(raw, state->previous_raw) *
           RAD_PER_COUNT / dt;
    alpha = dt / (PENDULUM_RATE_TAU_S + dt);
    state->rate_rad_s += alpha * (rate - state->rate_rad_s);
  } else {
    state->rate_rad_s = 0.0f;
  }
  state->previous_raw = raw;
  state->previous_ms = now_ms;
  state->derivative_ready = 1U;
  state->valid = 1U;
}
