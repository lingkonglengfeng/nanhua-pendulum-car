/**
  ******************************************************************************
  * @file    app_vehicle.h
  * @brief   车载倒立摆应用层（两平台共用，引脚见 docs/ 接线文档）：
  *            四路编码器 EXTI 软件正交解码 / 四路 AT8236 PWM+DIR /
  *            AS5600 摆角 / 零点标定 / 直立环 PD（200Hz ControlTask）。
  *
  *          电机通道按PWM编号；原始编码器索引独立，由g_enc_map建立对应。
  *          本车F103最新实测：0=右后，1=右前，2=左后，3=左前。
  *          F407 探索者：PWM TIM3_CH3/CH4=PB0/PB1 + TIM9_CH1/CH2=PE5/PE6；
  *                       DIR PB6/PB7/PB10/PB11；编码器 PA4/5、PE2/3、PD6/7、PC7/12
  *          F103C8T6  ：GPIO DMA IN1=PA8..PA11；IN2=PA2/PA3/PB14/PB15；
  *                       编码器 PA0/1、PA4/5、PA6/7、PB12/13；PC13 速度到达指示灯
  ******************************************************************************
  */
#ifndef APP_VEHICLE_H
#define APP_VEHICLE_H

#include <stdint.h>
#include "main.h"
#include "balance_control.h"

#if defined(STM32F407xx)
#define APP_FW_BUILD 0xF4070005U
#elif defined(STM32F103xB)
#define APP_FW_BUILD 0x103C8048U
#else
#error app_vehicle: 未识别的平台（需要 STM32F407xx 或 STM32F103xB）
#endif

#define APP_WHEEL_COUNT 4U
#define APP_MOTOR_PWM_SLOTS 64U
#define APP_ENCODER_COUNTS_PER_REV (11.0f * 2.0f * 21.3f)
#define APP_ENCODER_QUALIFY_US 100U
/* 已确认：370电机、11线、A相双边沿、21.3减速比；可按实测校准。 */
/* 本车F103电机0=右后、1=右前、2=左后、3=左前；原始编码器索引独立。 */

/* 调度器启动前调用（freertos.c 的 MX_FREERTOS_Init 内 USER CODE Init）。 */
void App_Init(void);
/* ControlTask 实体（200Hz）。 */
void App_ControlTask(void *argument);
/* ControlTask 边界更新单轮增益后调用，仅清该轮 PID 状态。 */
void App_ResetSpeedWheel(uint8_t wheel);

/* ====================== Watch 观察/控制变量 ====================== */

/* 固件与运行健康 */
extern volatile uint32_t g_fw_build;        /* 版本号：Watch 核对烧录的固件 */
extern volatile uint32_t g_hse_fail;        /* 0=HSE 正常，1=回退 HSI（检查晶振） */
extern volatile uint32_t g_clock_hz;        /* 实际 HCLK（72M / 168M） */
extern volatile uint32_t g_rtos_heartbeat;  /* defaultTask 500ms 自增 */
extern volatile uint8_t  g_app_init_done;   /* 1=外设初始化完成 */

/* AS5600 */
extern volatile uint16_t g_as5600_raw_angle;
extern volatile uint16_t g_as5600_raw_filt;   /* 中值滤波（窗口3）后的原始角（送摆角处理的值） */
extern volatile uint32_t g_as5600_angle_mdeg;
extern volatile uint8_t  g_as5600_status;
extern volatile uint8_t  g_as5600_valid;    /* 通信+磁铁状态综合有效 */
extern volatile uint32_t g_as5600_read_ok;
extern volatile uint32_t g_as5600_read_errors;
extern volatile uint32_t g_as5600_timeout_count;
extern volatile uint32_t g_as5600_last_sample_ms;
extern volatile uint32_t g_as5600_last_valid_ms;
extern volatile uint32_t g_as5600_read_time_ms; /* 本周期 AS5600 调用耗时 */

/* 摆角（PendulumAngle 模块输出） */
extern volatile float    g_pendulum_angle_rad;     /* 直立偏差，杆顶向车头为正 */
extern volatile float    g_pendulum_rate_rad_s;    /* 低通后的角速率 */
extern volatile uint8_t  g_pendulum_angle_valid;
extern volatile uint8_t  g_pendulum_calibrated;
extern volatile uint16_t g_pendulum_upright_raw;
extern volatile uint16_t g_pendulum_bottom_raw;
extern volatile float g_pendulum_bottom_deg; /* bottom=0, upright=180, range [0,360) */
extern volatile int8_t   g_pendulum_angle_sign;
extern volatile uint32_t g_pendulum_zero_result;   /* 0=空闲 1=采集中 2=成功 3=样本差 4=移动 5=配置错 */
extern volatile uint16_t g_pendulum_zero_samples;
/* 标定命令（Watch 写入，处理后自动清零）：
 *   g_pendulum_zero_request = 1   → 最低点标零，成功后等待手扶至顶端400ms
 *   g_pendulum_sign_request = ±1  → 校正角度符号 */
extern volatile uint8_t  g_pendulum_zero_request;
extern volatile int8_t   g_pendulum_sign_request;
extern volatile uint32_t g_pendulum_command_errors;

/* 周期统计 */
extern volatile uint32_t g_pendulum_sample_count;
extern volatile uint32_t g_pendulum_cycle_time_ms;
extern volatile uint32_t g_pendulum_max_cycle_time_ms;
extern volatile uint32_t g_pendulum_late_wake_count;
extern volatile uint32_t g_pendulum_missed_periods;
extern volatile uint32_t g_pendulum_delay_errors;
extern volatile uint32_t g_pendulum_stale_events;

/* 四轮编码器（EXTI 软件正交解码，A 相双边沿） */
extern volatile int32_t  g_enc_count[APP_WHEEL_COUNT];  /* 累计计数 */
extern volatile int32_t  g_enc_delta[APP_WHEEL_COUNT];  /* 每 5ms 增量 */
extern volatile int8_t   g_enc_sign[APP_WHEEL_COUNT];   /* 手转标定方向用 ±1 */
#if defined(STM32F103xB)
/* Raw GPIO encoder order, not mapped motor order; cumulative since startup. */
extern volatile uint32_t g_enc_irq[APP_WHEEL_COUNT], g_enc_accepted[APP_WHEEL_COUNT];
extern volatile uint32_t g_enc_glitch[APP_WHEEL_COUNT], g_enc_duplicate[APP_WHEEL_COUNT];
#endif

/* 直立环（PID，200Hz）：先 ZERO，再自动等待顶端400ms；或 ARM 重新准备。
 * 输出为限幅归一化占空比，不与四轮速度PI叠加。增益/状态见 balance_control.h。 */
extern volatile uint8_t  g_bal_enable;      /* 只读：仅 BAL_ACTIVE 为1，不能写1绕过状态机 */
extern volatile float    g_bal_kp;
extern volatile float    g_bal_kd;
extern volatile float    g_bal_theta;
extern volatile float    g_bal_theta_dot;
extern volatile float    g_bal_output;      /* 本周期控制量 */
extern volatile float    g_bal_u[APP_WHEEL_COUNT];
extern volatile uint16_t g_bal_ccr[APP_WHEEL_COUNT];
extern volatile uint8_t  g_bal_dir[APP_WHEEL_COUNT];
extern volatile int8_t   g_mot_sign[APP_WHEEL_COUNT];   /* 单轮转向标定 ±1 */
extern volatile uint32_t g_bal_fault;       /* 直立准备/运行期间的退出事件累计数 */

/* 单电机台架测试（优先级高于直立环；enable=0 时全部滑行）
 *   AT8236 真值表：dir=1 且 pwm=0 → 全速反转（车必须架空！） */
extern volatile uint8_t  g_motor_test_enable;
extern volatile uint8_t  g_motor_test_wheel;    /* F103: 0右后、1右前、2左后、3左前；默认0 */
extern volatile uint8_t  g_motor_test_pwm_pct;  /* 原始命令0..100；F103实际GPIO占比见M行 */
extern volatile uint8_t  g_motor_test_dir;      /* 原始命令方向；F103不表示IN2静态电平 */
extern volatile uint16_t g_motor_test_ccr;      /* F103: IN1高电平等效3599刻度；非TIM比较寄存器 */
#if defined(STM32F103xB)
/* DMA PWM: M reports 64-slot GPIO waveforms, ccr/dir are legacy observations. */
extern volatile uint8_t g_motor_on_ticks[4],g_motor_in1_ticks[4],g_motor_in2_ticks[4];
extern volatile uint32_t g_motor_pwm_hz,g_motor_dma_periods,g_motor_dma_errors;
extern volatile uint32_t g_motor_dma_late,g_motor_dma_max_cycles;
#endif

/* 四轮开环直行测试（无传感器时的动力链测试）：-100..100（正=车前进方向），0=滑行。
 * 优先级：单轮台架测试 > 本项 > 直立环 > 滑行。⚠ 车必须架空或留出直线跑道！ */
extern volatile int16_t  g_drive_test;

/* 四轮速度闭环（编码器测速 → PID 调速，Kd=0 时为 PI）：
 *   目标/当前/误差单位为 RPM（输出轴转/分钟），正=前进方向。
 *   g_enc_map[i] = 第 i 路电机闭环使用的原始编码器索引。
 *   本车 F103 默认 0/2/1/3；F407 保持 0/1/2/3，映射与方向需据实物校准。
 *   优先级：单轮台架测试 > 直行测试 > 速度环 > 直立环 > 滑行。
 *   默认 g_speed_enable = 0（关闭）。⚠ 调试时车必须架空！ */
extern volatile float   g_enc_speed[APP_WHEEL_COUNT];   /* 实测轮速（计数/秒，低通后） */
extern volatile float g_speed_estimated_rpm[APP_WHEEL_COUNT]; /* 低速周期/高速计数，电机通道顺序 */
extern volatile float   g_speed_target[APP_WHEEL_COUNT]; /* 目标轮速（输出轴 RPM） */
extern volatile float   g_enc_counts_per_rev;           /* 输出轴每圈计数，默认 468.6 */
extern volatile uint8_t g_speed_enable;                 /* 1=启用速度闭环 */
extern volatile float   g_spd_kp;                       /* 比例增益，建议先仅调 P */
extern volatile float   g_spd_ki;                       /* 积分增益，内部已乘实际 dt（秒） */
extern volatile float   g_spd_kd;                       /* 微分增益，对反馈微分，内部除 dt */
/* 单轮覆盖：individual[i]=1 时使用 *_w[i]，否则使用上方共用增益。 */
extern volatile uint8_t g_spd_individual[APP_WHEEL_COUNT];
extern volatile float g_spd_kp_w[APP_WHEEL_COUNT];
extern volatile float g_spd_ki_w[APP_WHEEL_COUNT];
extern volatile float g_spd_kd_w[APP_WHEEL_COUNT];
extern volatile float g_spd_kp_used[APP_WHEEL_COUNT]; /* 本周期实际增益，只读 */
extern volatile float g_spd_ki_used[APP_WHEEL_COUNT];
extern volatile float g_spd_kd_used[APP_WHEEL_COUNT];
extern volatile float   g_speed_current[APP_WHEEL_COUNT]; /* 按 g_enc_map 对应的当前 RPM，只读 */
extern volatile float   g_speed_fast[APP_WHEEL_COUNT];    /* 原20ms测速，额外反馈滤波前，RPM */
extern volatile uint32_t g_speed_filter_ms[APP_WHEEL_COUNT]; /* 电机索引；额外低通tau，0旁路，最大200ms */
extern volatile uint32_t g_speed_filter_used_ms[APP_WHEEL_COUNT]; /* 本周期实际采用的tau */
extern volatile float   g_speed_error[APP_WHEEL_COUNT]; /* 目标 - 当前（RPM），只读 */
extern volatile float   g_spd_integral[APP_WHEEL_COUNT]; /* 各轮积分输出，只读，±1 */
extern volatile uint8_t g_speed_reset;                  /* 写 1 清积分和微分历史，自动清零 */
extern volatile uint8_t g_speed_active;                 /* 1=本周期实际由速度环驱动，只读 */
extern volatile uint32_t g_speed_dt_ms;                 /* 本次编码器采样间隔，正常约 5ms */
extern volatile uint8_t g_speed_led_wheel;              /* PC13 观察通道 0..3，默认 0 */
extern volatile float   g_speed_tolerance_pct;          /* 目标速度容差百分比，默认 5% */
extern volatile uint8_t g_speed_reached;                /* 1=非零目标且误差在容差内，PC13 亮 */
extern volatile float   g_spd_u[APP_WHEEL_COUNT];       /* 各轮功放输出量（±1） */
extern volatile int8_t  g_enc_map[APP_WHEEL_COUNT];     /* 编码器索引重映射 */

#endif /* APP_VEHICLE_H */
