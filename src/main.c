#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

/* multicore */
#include "pico/multicore.h"
#include "pico/sync.h"  // For spinlocks

#include "gpio.h"
#include "motor.h"

#include "hardware/pio.h"
#include "hardware/timer.h"
#include "quadrature_encoder.pio.h"

#include "mpu6050.h"

TaskHandle_t blinkTaskHandle = NULL;
TaskHandle_t motorTaskHandle = NULL;
TaskHandle_t encoderTaskHandle = NULL;
TaskHandle_t mpu6050TaskHandle = NULL;

void mpu6050Task(void *params) {
    printf("MPU6050 init...\n");
    mpu6050Init();

    for (;;) {
        int16_t acceleration[3] = {0};
        int16_t gyro[3] = {0};
        int16_t temp = 0;
        mpu6050ReadData(acceleration, gyro, &temp);
        /*
        // These are the raw numbers from the chip, so will need tweaking to be really useful.
        // See the datasheet for more information
        printf("Acc. X = %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1], acceleration[2]);
        printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
        // Temperature is simple so use the datasheet calculation to get deg C.
        // Note this is chip temperature.
        printf("Temp. = %f\n", (temp / 340.0) + 36.53);
        */

        float ax = (float)acceleration[0] / 16384.0f;
        float ay = (float)acceleration[1] / 16384.0f;
        float az = (float)acceleration[2] / 16384.0f;

        float gx = (float)gyro[0] / 131.0f;
        float gy = (float)gyro[1] / 131.0f;
        float gz = (float)gyro[2] / 131.0f;

        float t_c = ((float)temp / 340.0f) + 36.53f;

        printf("Acc:  X %6.2f, Y %6.2f, Z %6.2f\n", ax, ay, az);
        printf("Gyro: X %6.2f, Y %6.2f, Z %6.2f\n", gx, gy, gz);
        printf("Temp: %5.1f°C\n", t_c);
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void encoderTask(void *params) {
    printf("Executing encoderTask\n");
    int new_value, delta, old_value = 0;
    int last_value = -1, last_delta = -1;

    const uint PIN_AB = gpioGetPinNumber(GPIO_MOTOR_PIO_ENCODER_A1); // Assuming A1 and A2 are consecutive pins

    printf("Hello from quadrature encoder\n");

    PIO pio = pio0;
    const uint sm = 0;

    // we don't really need to keep the offset, as this program must be loaded
    // at offset 0
    pio_add_program(pio, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio, sm, PIN_AB, 0);

    for (;;) {
        // note: thanks to two's complement arithmetic delta will always
        // be correct even when new_value wraps around MAXINT / MININT
        new_value = quadrature_encoder_get_count(pio, sm);
        delta = new_value - old_value;
        old_value = new_value;

        if (new_value != last_value || delta != last_delta ) {
            printf("position %8d, delta %6d\n", new_value, delta);
            last_value = new_value;
            last_delta = delta;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


void motorTask(void *params) {
    printf("Executing motorTask\n");
    motorInit();

    for (;;) {
        motorSetSpeed(MOTOR_A, MOTOR_DIRECTION_CW, 255);
        vTaskDelay(pdMS_TO_TICKS(1000));
        motorStop(MOTOR_A);
        vTaskDelay(pdMS_TO_TICKS(2000));
        motorSetSpeed(MOTOR_A, MOTOR_DIRECTION_CCW, 255);
        vTaskDelay(pdMS_TO_TICKS(1000));
        motorStop(MOTOR_A);
        vTaskDelay(pdMS_TO_TICKS(2000));

        motorSetSpeed(MOTOR_A, MOTOR_DIRECTION_CW, 50);
        vTaskDelay(pdMS_TO_TICKS(1000));
        motorBrake(MOTOR_A);
        vTaskDelay(pdMS_TO_TICKS(2000));
        motorSetSpeed(MOTOR_A, MOTOR_DIRECTION_CCW, 50);
        vTaskDelay(pdMS_TO_TICKS(1000));
        motorBrake(MOTOR_A);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void blinkTask(void *params) {
    printf("Executing blinkTask\n");

    for (;;) {
        gpioWrite(GPIO_LED, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpioWrite(GPIO_LED, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

int main() {
    stdio_init_all();

    gpioInit();

    printf("Hello, world!\n");

    BaseType_t status = xTaskCreate(
        blinkTask,
        "blinkTask",
        1024,
        NULL,
        2,
        &blinkTaskHandle
    );

    configASSERT(status == pdPASS);

    status = xTaskCreate(
        motorTask,
        "motorTask",
        1024,
        NULL,
        2,
        &motorTaskHandle
    );

    configASSERT(status == pdPASS);

    status = xTaskCreate(
        encoderTask,
        "encoderTask",
        1024,
        NULL,
        2,
        &encoderTaskHandle
    );

    configASSERT(status == pdPASS);

    status = xTaskCreate(
        mpu6050Task,
        "mpu6050Task",
        1024,
        NULL,
        2,
        &mpu6050TaskHandle
    );

    configASSERT(status == pdPASS);

    vTaskStartScheduler();

    // code will never reach here if everything goes well.
    for(;;);

    return 0;
}
