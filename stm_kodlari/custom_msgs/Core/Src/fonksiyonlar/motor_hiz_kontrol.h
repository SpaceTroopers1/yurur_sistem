#ifndef MOTOR_HIZ_KONTROL_H
#define MOTOR_HIZ_KONTROL_H

int speedToPWM(float speed);
void setPWM(TIM_HandleTypeDef *htim, uint32_t channel, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, float pwmvalue);
float calculatePID(float *integral, float *previousError, float ramped, float current, float P, float I, float D, int max_pwm, int min_pwm);
float applyRamp(float current, float target ,float RAMP_RATE);
float limitSpeedToPWM(float speed);
void applySlipControl(Wheel *wheel, float referenceSpeed);
void controlWheel(TIM_HandleTypeDef *htim, uint32_t channel, Wheel *wheel, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void controlRover(TIM_HandleTypeDef *htim, float rotation_speedL, float rotation_speedR, float throttle);
void controlDirection(float throttle, float *rotation_speedL, float *rotation_speedR);

#endif
