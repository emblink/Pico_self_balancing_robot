#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

/* multicore */
#include "pico/multicore.h"
#include "pico/sync.h"  // For spinlocks

#define LED_PIN 25

TaskHandle_t blinkTaskHandle = NULL;

void blinkTask(void *params) {
    printf("Executing blinkTask\n");

    for (;;) {
        gpio_put(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_put(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

int main() {
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

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

    vTaskStartScheduler();

    // code will never reach here if everything goes well.
    for(;;);

    return 0;
}
