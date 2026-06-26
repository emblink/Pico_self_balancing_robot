#pragma once

void pidInit(float kp, float ki, float kd);
void pidSetTunings(float kp, float ki, float kd);
void pidSetKp(float kp);
void pidSetKi(float ki);
void pidSetKd(float kd);
void pidSetTarget(float targetVal);
float pidGetKp(void);
float pidGetKi(void);
float pidGetKd(void);
float pidGetError(void);
float pidCalculate(float currentVal, float dt);

