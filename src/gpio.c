#include "gpio.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include <assert.h>

typedef struct {
    Pin_type type; // PIN_TYPE_GPIO, PIN_TYPE_PWM, or PIN_TYPE_PERIPHERAL
    int pinNumber;
    bool isOutput;
    uint8_t initialValue; // 0 or 1 for digital pins, or initial duty cycle for PWM pins
} GpioPinConfig;

#define PIN_UNUSED -1

static const GpioPinConfig gpioConfigs[GPIO_COUNT] = {
    [GPIO_LED] =                  { .type = PIN_TYPE_GPIO,   .pinNumber = 25,           .isOutput = true,    .initialValue = 0 },
    [GPIO_MOTOR_AIN1] =           { .type = PIN_TYPE_GPIO,   .pinNumber = 19,           .isOutput = true,    .initialValue = 0 },
    [GPIO_MOTOR_AIN2] =           { .type = PIN_TYPE_GPIO,   .pinNumber = 20,           .isOutput = true,    .initialValue = 0 },
    [GPIO_MOTOR_BIN1] =           { .type = PIN_TYPE_GPIO,   .pinNumber = 17,           .isOutput = true,    .initialValue = 0 },
    [GPIO_MOTOR_BIN2] =           { .type = PIN_TYPE_GPIO,   .pinNumber = 16,           .isOutput = true,    .initialValue = 0 },
    [GPIO_MOTOR_STBY] =           { .type = PIN_TYPE_GPIO,   .pinNumber = 18,           .isOutput = true,    .initialValue = 0 },
    [GPIO_MOTOR_PIO_ENCODER_A1] = { .type = PIN_TYPE_GPIO,   .pinNumber = 28,           .isOutput = false,   .initialValue = 0 },
    [GPIO_MOTOR_PIO_ENCODER_A2] = { .type = PIN_TYPE_GPIO,   .pinNumber = 29,           .isOutput = false,   .initialValue = 0 },
    [GPIO_MOTOR_PIO_ENCODER_B1] = { .type = PIN_TYPE_GPIO,   .pinNumber = PIN_UNUSED,   .isOutput = false,   .initialValue = 0 },
    [GPIO_MOTOR_PIO_ENCODER_B2] = { .type = PIN_TYPE_GPIO,   .pinNumber = PIN_UNUSED,   .isOutput = false,   .initialValue = 0 },
    [GPIO_MPU_I2C_SCL] =          { .type = PIN_TYPE_GPIO,   .pinNumber = PIN_UNUSED,   .isOutput = true,    .initialValue = 1 },
    [GPIO_MPU_I2C_SDA] =          { .type = PIN_TYPE_GPIO,   .pinNumber = PIN_UNUSED,   .isOutput = true,    .initialValue = 1 },
    [GPIO_BATTERY_VOLTAGE] =      { .type = PIN_TYPE_GPIO,   .pinNumber = PIN_UNUSED,   .isOutput = false,   .initialValue = 0 },
    [GPIO_MOTOR_PWMA] =           { .type = PIN_TYPE_PWM,    .pinNumber = 21,           .isOutput = true,    .initialValue = 0 },
    [GPIO_MOTOR_PWMB] =           { .type = PIN_TYPE_PWM,    .pinNumber = 22,           .isOutput = true,    .initialValue = 0 },
};

void gpioInit(void) {
    for (int i = 0; i < GPIO_COUNT; i++) {
        int pin = gpioConfigs[i].pinNumber;
        if (pin == PIN_UNUSED) {
            continue;
        }

        if (gpioConfigs[i].type == PIN_TYPE_GPIO) {
            gpio_init(pin);
            gpio_set_dir(pin, gpioConfigs[i].isOutput);
            if (gpioConfigs[i].isOutput) {
                gpio_put(pin, gpioConfigs[i].initialValue);
            }
        } else if (gpioConfigs[i].type == PIN_TYPE_PWM) {
            gpio_set_function(pin, GPIO_FUNC_PWM);
            uint slice_num = pwm_gpio_to_slice_num(pin);
            pwm_set_wrap(slice_num, 255);
            pwm_set_gpio_level(pin, gpioConfigs[i].initialValue);
            pwm_set_clkdiv(slice_num, 20.0f); // 125 MHz / 20 / 256 = ~24.4 kHz — good pwm frequency for motor driver
            pwm_set_enabled(slice_num, true);
        } else if (gpioConfigs[i].type == PIN_TYPE_PERIPHERAL) {
            // For peripheral pins, we might need to set them up differently
            // This is just a placeholder and should be implemented based on the specific peripheral requirements
        }
    }
}

bool gpioRead(GPIO_pin pin) {
    assert(pin < GPIO_COUNT);
    assert(gpioConfigs[pin].type == PIN_TYPE_GPIO);
    assert(!gpioConfigs[pin].isOutput);

    bool state = gpio_get(gpioConfigs[pin].pinNumber);
    return state;
}

void gpioWrite(GPIO_pin pin, bool value) {
    assert(pin < GPIO_COUNT);
    assert(gpioConfigs[pin].type == PIN_TYPE_GPIO);
    assert(gpioConfigs[pin].isOutput);

    gpio_put(gpioConfigs[pin].pinNumber, value);
}

void gpioSetPWM(GPIO_pin pin, uint8_t dutyCycle) {
    assert(gpioConfigs[pin].type == PIN_TYPE_PWM);

    pwm_set_gpio_level(gpioConfigs[pin].pinNumber, dutyCycle);
}

int gpioGetPinNumber(GPIO_pin pin) {
    assert(pin < GPIO_COUNT);
    return gpioConfigs[pin].pinNumber;
}