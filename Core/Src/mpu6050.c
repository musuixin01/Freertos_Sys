#include "mpu6050.h"
#include "i2c.h"

#define MPU6050_ADDR       (0x68U << 1)
#define MPU6050_REG_PWR    0x6B
#define MPU6050_REG_ACCEL  0x3B
#define MPU6050_REG_ACCEL_CFG 0x1C

static uint8_t write_reg(uint8_t reg, uint8_t value)
{
  return (HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                            &value, 1, 50) == HAL_OK) ? 1U : 0U;
}

uint8_t MPU6050_Init(void)
{
  uint8_t who = 0;
  if (HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, 0x75, I2C_MEMADD_SIZE_8BIT,
                       &who, 1, 50) != HAL_OK)
  {
    return 0U;
  }
  if ((who != 0x68U) && (who != 0x70U))
  {
    return 0U;
  }
  if (write_reg(MPU6050_REG_PWR, 0x00U) == 0U) { return 0U; }
  return write_reg(MPU6050_REG_ACCEL_CFG, 0x00U); /* +/- 2 g */
}

uint8_t MPU6050_ReadAcceleration(int16_t *x_mg, int16_t *y_mg, int16_t *z_mg)
{
  uint8_t raw[6];
  int16_t ax, ay, az;
  if ((x_mg == NULL) || (y_mg == NULL) || (z_mg == NULL))
  {
    return 0U;
  }
  if (HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, MPU6050_REG_ACCEL,
                       I2C_MEMADD_SIZE_8BIT, raw, sizeof(raw), 50) != HAL_OK)
  {
    return 0U;
  }
  ax = (int16_t)((raw[0] << 8) | raw[1]);
  ay = (int16_t)((raw[2] << 8) | raw[3]);
  az = (int16_t)((raw[4] << 8) | raw[5]);
  *x_mg = (int16_t)((int32_t)ax * 1000 / 16384);
  *y_mg = (int16_t)((int32_t)ay * 1000 / 16384);
  *z_mg = (int16_t)((int32_t)az * 1000 / 16384);
  return 1U;
}
