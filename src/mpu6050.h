#pragma once
#include <stdbool.h>
#include <stdint.h>

bool mpu6050Init(void);
bool mpu6050ReadData(int16_t accel[3], int16_t gyro[3], int16_t *temp);
