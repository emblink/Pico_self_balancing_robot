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

TaskHandle_t blinkTaskHandle = NULL;
TaskHandle_t motorTaskHandle = NULL;

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

    vTaskStartScheduler();

    // code will never reach here if everything goes well.
    for(;;);

    return 0;
}
