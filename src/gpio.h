#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    GPIO_LED = 0U,
    GPIO_MOTOR_AIN1,
    GPIO_MOTOR_AIN2,
    GPIO_MOTOR_BIN1,
    GPIO_MOTOR_BIN2,
    GPIO_MOTOR_STBY,
    GPIO_MOTOR_PIO_ENCODER_A1,
    GPIO_MOTOR_PIO_ENCODER_A2,
    GPIO_MOTOR_PIO_ENCODER_B1,
    GPIO_MOTOR_PIO_ENCODER_B2,
    GPIO_MOTOR_PWMA,
    GPIO_MOTOR_PWMB,
    GPIO_MPU_I2C_SCL,
    GPIO_MPU_I2C_SDA,
    GPIO_BATTERY_VOLTAGE,
    GPIO_COUNT
} GPIO_pin;

typedef enum {
    PIN_TYPE_GPIO = 0U,
    PIN_TYPE_PWM = 1U,
    PIN_TYPE_PERIPHERAL = 2U,
} Pin_type;

void gpioInit(void);
bool gpioRead(GPIO_pin pin);
void gpioWrite(GPIO_pin pin, bool value);
void gpioSetPWM(GPIO_pin pin, uint8_t dutyCycle);


