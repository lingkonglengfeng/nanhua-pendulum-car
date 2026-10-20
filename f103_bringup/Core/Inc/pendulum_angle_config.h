/**
  ******************************************************************************
  * @file    pendulum_angle_config.h
  * @brief   摆角标定与处理参数（两平台共用）
  *
 * 标定流程（固件15起）：
 *   1. 摆杆自然下垂静止，串口 ZERO；采集200个最低点样本。
 *   2. 顶端原始角 = (最低点原始角 + 2048) % 4096。
 *   3. 标定成功后等待手扶至顶端窗口400ms，自动进入直立环。
 *   4. PENDSIGN +/-1 校正角度方向；BALSIGN +/-1 校正控制极性。
 *   本版每次上电必须重新 ZERO；串口标定/参数保存在 RAM。
  ******************************************************************************
  */
#ifndef PENDULUM_ANGLE_CONFIG_H
#define PENDULUM_ANGLE_CONFIG_H

/* Save measured calibration here and rebuild/download to retain it at reboot.
 * Default 0 means no measured upright zero; NEVER learn zero at power-on. */
#define PENDULUM_ZERO_STORED       0U
#define PENDULUM_UPRIGHT_RAW       0U   /* Runtime ZERO at bottom is required. */
/* Rod top tilting towards the front must produce a positive angle. */
#define PENDULUM_ANGLE_SIGN        1

/* ControlTask 周期：5ms = 200Hz（与 v4 一致） */
#define PENDULUM_SAMPLE_PERIOD_MS  5U
#define PENDULUM_STALE_MS          30U
#define PENDULUM_RATE_TAU_S        0.010f  /* Angular-rate LP time constant: 10ms. */
#define PENDULUM_ZERO_SAMPLES      200U    /* 200 次 × 5ms = 1s 采集窗口 */
#define PENDULUM_ZERO_MAX_SPAN     32

#if (PENDULUM_ZERO_STORED != 0U) && (PENDULUM_ZERO_STORED != 1U)
#error PENDULUM_ZERO_STORED_must_be_zero_or_one
#endif
#if PENDULUM_UPRIGHT_RAW > 4095U
#error PENDULUM_UPRIGHT_RAW_out_of_range
#endif
#if (PENDULUM_ANGLE_SIGN != 1) && (PENDULUM_ANGLE_SIGN != -1)
#error PENDULUM_ANGLE_SIGN_must_be_plus_or_minus_one
#endif

#endif
