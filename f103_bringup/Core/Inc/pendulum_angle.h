/**
  ******************************************************************************
  * @file    pendulum_angle.h
  * @brief   摆杆角度处理：零点/跨零/角速率低通（由车工程 G474 版移植）
  *           两平台（F407 探索者 / F103C8T6 蓝丸）共用同一份实现。
  ******************************************************************************
  */
#ifndef PENDULUM_ANGLE_H
#define PENDULUM_ANGLE_H

#include <stdint.h>
#include "pendulum_angle_config.h"

typedef enum {
  PENDULUM_ZERO_IDLE = 0,
  PENDULUM_ZERO_COLLECTING = 1,
  PENDULUM_ZERO_SUCCESS = 2,
  PENDULUM_ZERO_BAD_SAMPLE = 3,
  PENDULUM_ZERO_MOVED = 4,
  PENDULUM_ZERO_BAD_CONFIG = 5
} PendulumZeroResult;

typedef struct {
  float angle_rad;
  float rate_rad_s;
  uint32_t last_sample_ms;
  uint32_t last_valid_ms;
  uint32_t previous_ms;
  uint16_t upright_raw;
  uint16_t bottom_raw;
  uint16_t previous_raw;
  uint16_t zero_reference;
  uint16_t zero_count;
  int32_t zero_sum;
  int16_t zero_min;
  int16_t zero_max;
  int8_t sign;
  uint8_t calibrated;
  uint8_t valid;
  uint8_t comm_ok;
  uint8_t magnet_status;
  uint8_t has_valid_sample;
  uint8_t derivative_ready;
  PendulumZeroResult zero_result;
} PendulumAngle;

void PendulumAngle_Init(PendulumAngle *state, uint16_t zero,
                        int8_t sign, uint8_t calibrated);
uint8_t PendulumAngle_SetCalibration(PendulumAngle *state, uint16_t zero,
                                     int8_t sign);
uint8_t PendulumAngle_SetSign(PendulumAngle *state, int8_t sign);
void PendulumAngle_BeginZero(PendulumAngle *state);
/* comm_ok=0 on ANY communication failure. status contains AS5600 status bits.
 * Call CheckFreshness regularly; timestamps use wrap-safe uint32 milliseconds.
 * Only one task owns this structure; never modify it from an interrupt. */
void PendulumAngle_Update(PendulumAngle *state, uint16_t raw, uint8_t status,
                          uint8_t comm_ok, uint32_t now_ms);
void PendulumAngle_CheckFreshness(PendulumAngle *state, uint32_t now_ms);

#endif
