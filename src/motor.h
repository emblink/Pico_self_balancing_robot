#pragma once

#include <stdint.h>

typedef enum {
    MOTOR_A = 0U,
    MOTOR_B = 1,
    MOTOR_COUNT
} Motor_id;

typedef enum {
    MOTOR_DIRECTION_CW = 0U,
    MOTOR_DIRECTION_CCW = 1,
    MOTOR_DIRECTION_COUNT,
} Motor_direction;

void motorInit(void);
void motorDriverEnable(void);
void morotDriverDisable(void);
void motorSetSpeed(Motor_id motor, Motor_direction dir, uint8_t speed);
void motorStop(Motor_id motor);
void motorBrake(Motor_id motor);
