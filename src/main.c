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
#include <math.h>

#include "protocol.h"
#include "pid.h"

#include "storage.h"
#include <hardware/watchdog.h>

#define MIN_MOTOR_PWM 40.0f
#define DEAD_BAND 10.0f

#define CORE_0 (1 << 0)
#define CORE_1 (1 << 1)

TaskHandle_t blinkTaskHandle = NULL;
TaskHandle_t motorTaskHandle = NULL;
TaskHandle_t encoderTaskHandle = NULL;
TaskHandle_t mpu6050TaskHandle = NULL;

static float measureGyroBiasX(void) {
    printf("Calibration IMU... Do not move the robot!\n");
    int32_t gyro_sum = 0;
    int samples = 200;
    float gyro_bias_x = 0.0f;

    for (int i = 0; i < samples; i++) {
        int16_t acc_raw[3], gyro_raw[3], temp_raw;
        if (mpu6050ReadData(acc_raw, gyro_raw, &temp_raw)) {
            gyro_sum += gyro_raw[0];
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    gyro_bias_x = ((float)gyro_sum / samples) / 131.0f;
    printf("Calibration finished! Bias X: %f\n", gyro_bias_x);
    return gyro_bias_x;
}

static void mpuDataToTeleplot(float ax, float ay, float az, float gx, float gy, float gz, float angle_acc, float robot_angle, float motor_PWM, float error) {
    printf(">Motor_PWM:%3.2f\n", motor_PWM);
    printf(">Error:%6.2f\n", error);
    printf(">Angle_Filtered:%6.2f\n", robot_angle);
    
    printf(">kP:%6.2f\n", pidGetKp());
    printf(">kD:%6.2f\n", pidGetKd());
    printf(">kI:%6.2f\n", pidGetKi());

    printf(">AccX:%6.2f\n", ax);
    printf(">AccY:%6.2f\n", ay);
    printf(">AccZ:%6.2f\n", az);

    printf(">GyroX:%6.2f\n", gx);
    printf(">GyroY:%6.2f\n", gy);
    printf(">GyroZ:%6.2f\n", gz);

    printf(">Angle_Acc:%6.2f\n", angle_acc);
}

void mpu6050Task(void *params) {
    printf("MPU6050 init...\n");
    mpu6050Init();
    bool res = storageInit();
    StorageData data = {0};
    if (!res) {
        printf("Storage not initialized, initializing with default values...\n");

        data.pidConfig.kp = 5.0f;
        data.pidConfig.ki = 0.0f;
        data.pidConfig.kd = 1.0f;
        res = storageWrite(&data);
        if (!res) {
            printf("Failed to write default storage, rebooting...\n");
            watchdog_reboot(0, 0, 500);
            for (;;) {
                tight_loop_contents();
            }
        }
    }
    storageRead(&data);
    pidInit(data.pidConfig.kp, data.pidConfig.ki, data.pidConfig.kd);

    #define GYRO_BIAS_X 10.121f // Pre-calibrated bias for gyro X axis, in degrees per second
    float gyro_bias_x = GYRO_BIAS_X;
    // float gyro_bias_x = mesureGyroBiasX(); // Uncomment this line to perform live calibration on startup instead of using pre-calibrated bias

    float robot_angle = 0.0f; // Final angle after complementary filter
    const float alpha = 0.98f; // Complementary filter coefficient
    const float dt = 0.01f;   // 10ms time step

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

        // printf("Acc:  X %6.2f, Y %6.2f, Z %6.2f\n", ax, ay, az);
        // printf("Gyro: X %6.2f, Y %6.2f, Z %6.2f\n", gx, gy, gz);
        // printf("Temp: %5.1f°C\n", t_c);
        
        float angle_acc = atan2f(ax, sqrtf(ay * ay + az * az)) * 57.2957f;
        
        float gx_final = gx - gyro_bias_x;
        
        // Complementary filter to combine accelerometer and gyroscope data
        robot_angle = alpha * (robot_angle + gx_final * dt) + (1.0f - alpha) * angle_acc;
        
        float speed = pidCalculate(robot_angle, 0.1f);
        float error = pidGetError();

        Motor_direction dir = speed > 0 ? MOTOR_DIRECTION_CW : MOTOR_DIRECTION_CCW;
        if (speed > -DEAD_BAND && speed < DEAD_BAND) {
            speed = 0.0f; // Deadband to prevent jitter
        } else if (speed > -MIN_MOTOR_PWM && speed < MIN_MOTOR_PWM) {
            speed = MIN_MOTOR_PWM; // Minimum PWM to overcome motor deadzone
        }
        uint8_t pwm =  (uint8_t)fminf(fabsf(speed), 255.0f);
        motorSetSpeed(MOTOR_A, dir, pwm);
        motorSetSpeed(MOTOR_B, dir, pwm);
        mpuDataToTeleplot(ax, ay, az, gx, gy, gz, angle_acc, robot_angle, pwm, error);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void encoderTask(void *params) {
    printf("Executing encoderTask\n");
    int new_value_enc_A = 0;
    int new_value_enc_B = 0;
    int delta_enc_A = 0;
    int delta_enc_B = 0;
    int old_value_enc_A = 0;
    int old_value_enc_B = 0;
    int last_value_enc_A = -1;
    int last_value_enc_B = -1;
    int last_delta_enc_A = -1;
    int last_delta_enc_B = -1;

    const uint PIN_A = gpioGetPinNumber(GPIO_MOTOR_PIO_ENCODER_A1); // Assuming A1 and A2 are consecutive pins
    const uint PIN_B = gpioGetPinNumber(GPIO_MOTOR_PIO_ENCODER_B1);

    printf("Hello from quadrature encoder\n");

    PIO pio= pio0;
    const uint sm_A = 0;
    const uint sm_B = 1;

    // we don't really need to keep the offset, as this program must be loaded
    // at offset 0
    pio_add_program(pio, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio, sm_A, PIN_A, 0);
    quadrature_encoder_program_init(pio, sm_B, PIN_B, 0);
    for (;;) {
        // note: thanks to two's complement arithmetic delta will always
        // be correct even when new_value wraps around MAXINT / MININT
        new_value_enc_A = quadrature_encoder_get_count(pio, sm_A);
        delta_enc_A = new_value_enc_A - old_value_enc_A;
        old_value_enc_A = new_value_enc_A;

        new_value_enc_B = quadrature_encoder_get_count(pio, sm_B);
        delta_enc_B = new_value_enc_B - old_value_enc_B;
        old_value_enc_B = new_value_enc_B;

        if (new_value_enc_A != last_value_enc_A || delta_enc_A != last_delta_enc_A ) {
            printf("Encoder A: position %8d, delta %6d\n", new_value_enc_A, delta_enc_A);
            last_value_enc_A = new_value_enc_A;
            last_delta_enc_A = delta_enc_A;
        }

        if (new_value_enc_B != last_value_enc_B || delta_enc_B != last_delta_enc_B ) {
            printf("Encoder B: position %8d, delta %6d\n", new_value_enc_B, delta_enc_B);
            last_value_enc_B = new_value_enc_B;
            last_delta_enc_B = delta_enc_B;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void motorTest(void) {
    motorSetSpeed(MOTOR_A, MOTOR_DIRECTION_CW, 255);
    motorSetSpeed(MOTOR_B, MOTOR_DIRECTION_CW, 255);
    vTaskDelay(pdMS_TO_TICKS(1000));
    motorStop(MOTOR_A);
    motorStop(MOTOR_B);
    vTaskDelay(pdMS_TO_TICKS(2000));
    motorSetSpeed(MOTOR_A, MOTOR_DIRECTION_CCW, 255);
    motorSetSpeed(MOTOR_B, MOTOR_DIRECTION_CCW, 255);
    vTaskDelay(pdMS_TO_TICKS(1000));
    motorStop(MOTOR_A);
    motorStop(MOTOR_B);
    vTaskDelay(pdMS_TO_TICKS(2000));

    motorSetSpeed(MOTOR_A, MOTOR_DIRECTION_CW, 50);
    motorSetSpeed(MOTOR_B, MOTOR_DIRECTION_CW, 50);
    vTaskDelay(pdMS_TO_TICKS(1000));
    motorBrake(MOTOR_A);
    motorBrake(MOTOR_B);
    vTaskDelay(pdMS_TO_TICKS(2000));
    motorSetSpeed(MOTOR_A, MOTOR_DIRECTION_CCW, 50);
    motorSetSpeed(MOTOR_B, MOTOR_DIRECTION_CCW, 50);
    vTaskDelay(pdMS_TO_TICKS(1000));
    motorBrake(MOTOR_A);
    motorBrake(MOTOR_B);
    vTaskDelay(pdMS_TO_TICKS(2000));
}

void motorTask(void *params) {
    printf("Executing motorTask\n");
    motorInit();
    // motorTest();

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
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

    BaseType_t status = xTaskCreateAffinitySet(
        blinkTask,
        "blinkTask",
        1024,
        NULL,
        2,
        CORE_1,
        &blinkTaskHandle
    );

    configASSERT(status == pdPASS);

    status = xTaskCreateAffinitySet(
        motorTask,
        "motorTask",
        1024,
        NULL,
        2,
        CORE_1,
        &motorTaskHandle
    );

    configASSERT(status == pdPASS);

    status = xTaskCreateAffinitySet(
        encoderTask,
        "encoderTask",
        1024,
        NULL,
        2,
        CORE_1,
        &encoderTaskHandle
    );

    configASSERT(status == pdPASS);

    status = xTaskCreateAffinitySet(
        mpu6050Task,
        "mpu6050Task",
        1024,
        NULL,
        2,
        CORE_0,
        &mpu6050TaskHandle
    );

    configASSERT(status == pdPASS);

    protocolInit();
    vTaskStartScheduler();

    // code will never reach here if everything goes well.
    for(;;);

    return 0;
}
