/**
  ******************************************************************************
  * @file    as5600.h
  * @brief   AS5600 磁编码器驱动（硬件 I2C1：重映射后 SCL=PB8 / SDA=PB9，100kHz）
  *           2026-10-16 由“软件位带 I2C”切换为硬件 I2C（CubeMX 生成 I2C1）。
  *           阻塞式接口：只在任务里调用，禁止在中断里调用。
  ******************************************************************************
  */
#ifndef AS5600_H
#define AS5600_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define AS5600_I2C_ADDRESS_7BIT       0x36U
#define AS5600_STATUS_MAGNET_DETECTED 0x20U
#define AS5600_STATUS_MAGNET_WEAK     0x10U
#define AS5600_STATUS_MAGNET_STRONG   0x08U

/* 确认总线空闲（被从机摁住则做一次 9 时钟解锁）；I2C 外设由 MX_I2C1_Init 完成。 */
void AS5600_BusInit(void);
uint32_t AS5600_LastBusError(void);
extern volatile uint32_t g_as5600_busy_skips; /* BUSY 快速返回次数，不进入 HAL 的 25ms 等待 */

/* 读原始角度（0..4095，单寄存器事务，调试用）。 */
HAL_StatusTypeDef AS5600_ReadRawAngle(uint16_t *raw_angle);
HAL_StatusTypeDef AS5600_ReadStatus(uint8_t *status);

/* 一次事务连读 STATUS + RAW_ANGLE（车辆主流程用，5ms 一次）。 */
HAL_StatusTypeDef AS5600_ReadSample(uint16_t *raw_angle, uint8_t *status);

/* Raw angle is 0..4095. Result is 0..359912 millidegrees. */
uint32_t AS5600_RawToMilliDegrees(uint16_t raw_angle);

#ifdef __cplusplus
}
#endif

#endif /* AS5600_H */
