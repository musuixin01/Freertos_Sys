#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "main.h"

/* Adjustable optional vibration threshold (0.3 g = 300 mg). */
#define MPU6050_VIB_THRESHOLD_MG 300U

uint8_t MPU6050_Init(void);
uint8_t MPU6050_ReadAcceleration(int16_t *x_mg, int16_t *y_mg, int16_t *z_mg);

#endif
