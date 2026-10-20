/**
  ******************************************************************************
  * @file    as5600.c
  * @brief   AS5600 磁编码器驱动（硬件 I2C1：重映射后 SCL=PB8 / SDA=PB9，100kHz）
  *
  *   2026-10-16 由“软件位带 I2C”切换为硬件 I2C（CubeMX 生成 I2C1 + 重映射）：
  *     - 读写走 HAL_I2C_Mem_Read（8 位寄存器地址，阻塞式，超时 4ms）
  *     - 错误分类：无应答(AF)/超时/其它总线故障，供 Watch 观察
  *     - 总线被从机摁住（SDA 低 / BUSY / 超时）时：停用外设 → 9 时钟 + STOP
  *       解锁 → 重新初始化 I2C（限频 100ms，避免每周期重初始化）
  *   对外接口与旧版完全兼容（app_vehicle.c 不需要任何改动）。
  ******************************************************************************
  */
#include "as5600.h"
#include "i2c.h"

#define AS5600_REG_STATUS         0x0BU
#define AS5600_REG_RAW_ANGLE_HIGH 0x0CU

/* 总线错误码（AS5600_LastBusError 返回） */
#define AS5600_ERR_NONE    0U
#define AS5600_ERR_AF      1U   /* 从机无应答（接线/器件问题） */
#define AS5600_ERR_TIMEOUT 2U   /* 超时（总线被摁住等） */
#define AS5600_ERR_BUS     3U   /* 其它总线故障（BUSY/BERR/ARLO 等） */

#define AS5600_I2C_TIMEOUT_MS   4U     /* 3 字节事务 @100kHz 约 0.4ms，4ms 裕量足够 */
#define AS5600_RECOVER_MIN_MS   100U   /* 总线解锁的最小间隔（限频） */

#define BB_SCL_PIN    8U
#define BB_SDA_PIN    9U
#define BB_SCL_MASK   (1U << BB_SCL_PIN)
#define BB_SDA_MASK   (1U << BB_SDA_PIN)

static uint32_t s_last_error = AS5600_ERR_NONE;
static uint8_t  s_ready = 0U;
static uint32_t s_last_recover_ms = 0U;
volatile uint32_t g_as5600_busy_skips = 0U;

static void bb_delay(void)
{
  volatile uint32_t n = 60U;   /* ≈5µs @72MHz，解锁时钟用（低速即可） */
  while (n-- != 0U)
  {
    __NOP();
  }
}

/* 总线解锁：SDA 被从机摁住时的标准恢复——9 个时钟 + STOP，然后重新初始化 I2C。
 * 注意：HAL_I2C_DeInit 会把外设置为 RESET 状态，随后的 HAL_I2C_Init 会自动
 * 重新调用 HAL_I2C_MspInit（恢复引脚复用含 I2C1 重映射、使能时钟）。 */
static void as5600_bus_recover(void)
{
  GPIO_InitTypeDef io = {0};
  uint8_t i;

  HAL_I2C_DeInit(&hi2c1);                 /* 停外设并释放 PB8/PB9 复用 */

  __HAL_RCC_GPIOB_CLK_ENABLE();
  io.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  io.Mode = GPIO_MODE_OUTPUT_OD;
  io.Pull = GPIO_PULLUP;
  io.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &io);

  GPIOB->BSRR = (1U << BB_SCL_PIN) | (1U << BB_SDA_PIN);   /* 双双释放（靠外部上拉） */
  bb_delay();
  for (i = 0U; i < 9U; i++)
  {
    GPIOB->BSRR = (1U << (BB_SCL_PIN + 16U));              /* SCL 低 */
    bb_delay();
    GPIOB->BSRR = (1U << BB_SCL_PIN);                      /* SCL 高 */
    bb_delay();
  }
  /* STOP：SCL 高时 SDA 由低变高 */
  GPIOB->BSRR = (1U << (BB_SDA_PIN + 16U));
  bb_delay();
  GPIOB->BSRR = (1U << BB_SDA_PIN);
  bb_delay();

  (void)HAL_I2C_Init(&hi2c1);             /* State==RESET → 重新 MspInit + 初始化 */
  s_ready = 1U;
}

void AS5600_BusInit(void)
{
  s_last_error = AS5600_ERR_NONE;
  s_last_recover_ms = HAL_GetTick();
  /* 总线空闲检查：SCL/SDA 都应被上拉到高；被摁住则解锁一次。
   * （I2C 外设本身由 main.c 的 MX_I2C1_Init 完成初始化） */
  if ((GPIOB->IDR & (BB_SCL_MASK | BB_SDA_MASK)) != (BB_SCL_MASK | BB_SDA_MASK))
  {
    as5600_bus_recover();
  }
  s_ready = 1U;
}

uint32_t AS5600_LastBusError(void)
{
  return s_last_error;
}

/* 失败分类；总线级故障（超时/其它）限频做解锁；普通无应答(AF)不恢复（总线本身没坏） */
static void handle_error(HAL_StatusTypeDef result)
{
  if (result == HAL_TIMEOUT)
  {
    s_last_error = AS5600_ERR_TIMEOUT;
  }
  else if ((hi2c1.ErrorCode & HAL_I2C_ERROR_AF) != 0U)
  {
    s_last_error = AS5600_ERR_AF;
    return;
  }
  else
  {
    s_last_error = AS5600_ERR_BUS;
  }

  if ((uint32_t)(HAL_GetTick() - s_last_recover_ms) >= AS5600_RECOVER_MIN_MS)
  {
    s_last_recover_ms = HAL_GetTick();
    as5600_bus_recover();
  }
}

static HAL_StatusTypeDef as5600_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
  HAL_StatusTypeDef result;

  if (s_ready == 0U)
  {
    AS5600_BusInit();
  }
  /* HAL_I2C_Mem_Read waits a fixed I2C_TIMEOUT_BUSY_FLAG=25ms before it
   * even uses the caller's 4ms timeout. This bus has a single synchronous
   * owner, so BUSY before a new transaction is a stale/broken-bus state.
   * Fail this sample promptly; retain the existing rate-limited recovery. */
  if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET)
  {
    g_as5600_busy_skips++;
    hi2c1.ErrorCode = HAL_I2C_ERROR_NONE; /* don't reuse an earlier AF error */
    handle_error(HAL_BUSY);
    return HAL_BUSY;
  }
  result = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(AS5600_I2C_ADDRESS_7BIT << 1U),
                            reg, I2C_MEMADD_SIZE_8BIT, buf, (uint16_t)len,
                            AS5600_I2C_TIMEOUT_MS);
  if (result == HAL_OK)
  {
    s_last_error = AS5600_ERR_NONE;
  }
  else
  {
    handle_error(result);
  }
  return result;
}

HAL_StatusTypeDef AS5600_ReadSample(uint16_t *raw_angle, uint8_t *status)
{
  uint8_t data[3];
  HAL_StatusTypeDef result;
  if (raw_angle == NULL || status == NULL)
  {
    return HAL_ERROR;
  }
  result = as5600_read_regs(AS5600_REG_STATUS, data, (uint8_t)sizeof(data));
  if (result == HAL_OK)
  {
    *status = data[0];
    *raw_angle = (uint16_t)((((uint16_t)data[1] & 0x0FU) << 8U) | data[2]);
  }
  return result;
}

HAL_StatusTypeDef AS5600_ReadRawAngle(uint16_t *raw_angle)
{
  uint8_t data[2];
  HAL_StatusTypeDef result;

  if (raw_angle == NULL)
  {
    return HAL_ERROR;
  }

  result = as5600_read_regs(AS5600_REG_RAW_ANGLE_HIGH, data, (uint8_t)sizeof(data));
  if (result == HAL_OK)
  {
    *raw_angle = (uint16_t)((((uint16_t)data[0] & 0x0FU) << 8U) | data[1]);
  }

  return result;
}

HAL_StatusTypeDef AS5600_ReadStatus(uint8_t *status)
{
  if (status == NULL)
  {
    return HAL_ERROR;
  }

  return as5600_read_regs(AS5600_REG_STATUS, status, 1U);
}

uint32_t AS5600_RawToMilliDegrees(uint16_t raw_angle)
{
  return ((uint32_t)(raw_angle & 0x0FFFU) * 360000U) / 4096U;
}
