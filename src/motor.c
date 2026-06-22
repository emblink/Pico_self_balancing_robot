#include <assert.h>
#include "motor.h"
#include "gpio.h"
 
void motorInit(void)
{
    morotDriverDisable();
    motorBrake(MOTOR_A);
    motorBrake(MOTOR_B);
    motorDriverEnable();
}

void motorDriverEnable(void)
{
    gpioWrite(GPIO_MOTOR_STBY, true);
}

void morotDriverDisable(void)
{
    gpioWrite(GPIO_MOTOR_STBY, false);
}

void motorSetSpeed(Motor_id motor, Motor_direction dir, uint8_t speed)
{
    assert(motor < MOTOR_COUNT);
    assert(speed <= 255);
    assert(dir < MOTOR_DIRECTION_COUNT);

    if (motor == MOTOR_A) {
        if (dir == MOTOR_DIRECTION_CW) {
            gpioWrite(GPIO_MOTOR_AIN1, true);
            gpioWrite(GPIO_MOTOR_AIN2, false);
        } else {
            gpioWrite(GPIO_MOTOR_AIN1, false);
            gpioWrite(GPIO_MOTOR_AIN2, true);
        }
        gpioSetPWM(GPIO_MOTOR_PWMA, speed);
    } else if (motor == MOTOR_B) {
        if (dir == MOTOR_DIRECTION_CW) {
            gpioWrite(GPIO_MOTOR_BIN1, true);
            gpioWrite(GPIO_MOTOR_BIN2, false);
        } else {
            gpioWrite(GPIO_MOTOR_BIN1, false);
            gpioWrite(GPIO_MOTOR_BIN2, true);
        }
        gpioSetPWM(GPIO_MOTOR_PWMB, speed);
    }
}

void motorStop(Motor_id motor)
{
    assert(motor < MOTOR_COUNT);

    if (motor == MOTOR_A) {
        gpioWrite(GPIO_MOTOR_AIN1, false);
        gpioWrite(GPIO_MOTOR_AIN2, false);
        gpioSetPWM(GPIO_MOTOR_PWMA, 255);
    } else if (motor == MOTOR_B) {
        gpioWrite(GPIO_MOTOR_BIN1, false);
        gpioWrite(GPIO_MOTOR_BIN2, false);
        gpioSetPWM(GPIO_MOTOR_PWMB, 255);
    }
}

void motorBrake(Motor_id motor)
{
    assert(motor < MOTOR_COUNT);

    if (motor == MOTOR_A) {
        gpioWrite(GPIO_MOTOR_AIN1, true);
        gpioWrite(GPIO_MOTOR_AIN2, true);
        gpioSetPWM(GPIO_MOTOR_PWMA, 0);
    } else if (motor == MOTOR_B) {
        gpioWrite(GPIO_MOTOR_BIN1, true);
        gpioWrite(GPIO_MOTOR_BIN2, true);
        gpioSetPWM(GPIO_MOTOR_PWMB, 0);
    }
}