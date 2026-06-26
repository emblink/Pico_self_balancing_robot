#include "pid.h"

static float Kp = 0.0f; 
static float Kd = 0.00f; 
static float Ki = 0.00f;
static float target = 0.0f;
static float error = 0.0f;
static float prevError = 0.0f;
static float integral = 0.0f;

void pidInit(float kp, float ki, float kd) {
    Kp = kp;
    Kd = kd;
    Ki = ki;
    target = 0.0f;
    error = 0.0f;
    prevError = 0.0f;
    integral = 0.0f;
}

void pidSetTunings(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

void pidSetKp(float kp) {
    Kp = kp;
}

void pidSetKi(float ki) {
    Ki = ki;
}

void pidSetKd(float kd) {
    Kd = kd;
}

float pidGetKp() {
    return Kp;
}

float pidGetKi() {
    return Ki;
}

float pidGetKd() {
    return Kd;
}

void pidSetTarget(float targetVal) {
    target = targetVal;
}

float pidGetError(void) {
    return error;
}

float pidCalculate(float currentVal, float dt) {
    error = target - currentVal;
    float proportional = error;

    integral += error * dt;

    float derivative = (error - prevError) / dt;
    prevError = error;

    float output = (Kp * proportional) + (Ki * integral) + (Kd * derivative);

    return output;
}