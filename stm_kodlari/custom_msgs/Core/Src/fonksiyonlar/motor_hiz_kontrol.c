#include "main.h"
#include "tanimlamalar.h"
#include <math.h>

#define KP 0.5f
#define KI 0.1f
#define KD 0.05f
#define Direction_KP 0.01f
#define Direction_KI 0.02f
#define Direction_KD 0.001f
#define MAX_SPEED_PWM  1000
#define MIN_SPEED_PWM -1000


#define SPEED_RAMP_RATE 1.0f // Her döngüde maksimum hız değişimi
#define DIRECTION_RAMP_RATE 30.0f // Her döngüde maksimum yön değişimi

// Motor hızı sınırlayıcı
#define MAX_SPEED  10.0
#define MIN_SPEED -10.0
#define MAX_DIRECTION_SPEED  3
#define MIN_DIRECTION_SPEED -3

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< MOTOR KONTROL >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//--------------------------------------------------------------------------------------------------------------------
/* --- Hız-PWM Uyumluluğu --- */
int speedToPWM(float speed) {
    return (int)((speed / MAX_SPEED) * MAX_SPEED_PWM);
}

/* --- PWM Ayarı --- */
void setPWM(TIM_HandleTypeDef *htim, uint32_t channel, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, float pwmvalue) {
    if (pwmvalue > 10) {
        __HAL_TIM_SET_COMPARE(htim, channel, pwmvalue);
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);  // Motor ileri yönde
    }
    else if (pwmvalue < -10) {
        __HAL_TIM_SET_COMPARE(htim, channel, -pwmvalue);
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET); // Motor geri yönde
    }
    else {
        __HAL_TIM_SET_COMPARE(htim, channel, 0);
    }
}

/* --- PID Hesaplama --- */
float calculatePID(float *integral, float *previousError, float ramped, float current, float P, float I, float D, int max_pwm, int min_pwm) {
    float error = ramped - current;
    *integral += error;
    float derivative = error - *previousError;
    *previousError = error;

    float output = P * error + I * *integral + D * derivative;

    if (output > max_pwm) output = max_pwm;
    if (output < min_pwm) output = min_pwm;

    return output;
}

/* --- Rampalı Hız Kontrolü --- */
float applyRamp(float current, float target ,float RAMP_RATE) {
    if (fabsf(target - current) > RAMP_RATE) {
        if (target > current) {
            return current + RAMP_RATE;
        } else {
            return current - RAMP_RATE;
        }
    }
    return target;
}

/* --- Hız Sınırlandırma --- */
float limitSpeedToPWM(float speed) {
	if (speed > MAX_SPEED) {speed = MAX_SPEED;}/*rgb yak*/
	if (speed < MIN_SPEED) {speed = MIN_SPEED;}/*rgb yak*/


	switch (currentGround) {
        case GROUND_SLIPPERY:
            return speedToPWM(speed)*0.5f; // Kaygan zemin: %50 hız
        case GROUND_ROUGH:
            return speedToPWM(speed)*0.8f; // Engebeli zemin: %80 hız
        default:
            return speedToPWM(speed);        // Normal zemin: Tam hız
    }
}

/* --- Patinaj Kontrolü --- */
void applySlipControl(Wheel *wheel, float referenceSpeed) {
    if (wheel->currentSpeed > referenceSpeed * 1.1f || wheel->currentSpeed < referenceSpeed * 0.9f  ) //%10 sapma
    {
        wheel->targetSpeed = referenceSpeed; // Hedef hızı ayarla

    }
}

/* --- Tekerlek Kontrolü --- */
void controlWheel(TIM_HandleTypeDef *htim, uint32_t channel, Wheel *wheel, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    wheel->rampedSpeed = applyRamp(wheel->currentSpeed, wheel->targetSpeed, SPEED_RAMP_RATE);
    float pwmValue = calculatePID(&wheel->integralSpeed, &wheel->previousErrorSpeed, wheel->rampedSpeed, wheel->currentSpeed, KP, KI, KD, MAX_SPEED_PWM, MIN_SPEED_PWM);
    setPWM(htim, channel, GPIOx, GPIO_Pin, pwmValue);

}

/* --- Rover Kontrol Fonksiyonu --- */
void controlRover(TIM_HandleTypeDef *htim, float rotation_speedL, float rotation_speedR, float throttle) {

	throttle = rcjoystick.throttle;
	// Hedef hızları joystick'e göre ayarla
    leftFrontWheel.targetSpeed = limitSpeedToPWM(throttle + rotation_speedL);
    leftBackWheel.targetSpeed = limitSpeedToPWM(throttle + rotation_speedL);
    rightFrontWheel.targetSpeed = limitSpeedToPWM(throttle + rotation_speedR);
    rightBackWheel.targetSpeed = limitSpeedToPWM(throttle + rotation_speedR);

    // Patinaj kontrolü uygula (örneğin, ön tekerlekleri referans al)
    applySlipControl(&leftFrontWheel, leftBackWheel.currentSpeed);
    applySlipControl(&rightFrontWheel, rightBackWheel.currentSpeed);
    // Tekerlekleri kontrol et
    controlWheel(htim, TIM_CHANNEL_1, &leftFrontWheel, GPIOB, GPIO_PIN_1);
    controlWheel(htim, TIM_CHANNEL_2, &rightFrontWheel, GPIOB, GPIO_PIN_2);
    controlWheel(htim, TIM_CHANNEL_3, &leftBackWheel, GPIOC, GPIO_PIN_4);
    controlWheel(htim, TIM_CHANNEL_4, &rightBackWheel, GPIOA, GPIO_PIN_6);
}

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<YÖN KONTROL>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//-----------------------------------------------------------------------------------------------------------------------
void controlDirection(float throttle, float *rotation_speedL, float *rotation_speedR){

	roverDirection.targetDirection = atan2(rcjoystick.y,rcjoystick.x) * (180 / M_PI);

    float error = 	roverDirection.targetDirection - roverDirection.currentDirection;
    while (error > 180) error -= 360;
    while (error < -180) error += 360;

    roverDirection.integralDirection += error;
    float derivative = error - roverDirection.previousErrorDirection;
    roverDirection.previousErrorDirection = error;

    float output = Direction_KP * error + Direction_KI * roverDirection.integralDirection + Direction_KD * derivative;

    if (output > MAX_DIRECTION_SPEED) output = MAX_DIRECTION_SPEED;
    if (output < MIN_DIRECTION_SPEED) output = MIN_DIRECTION_SPEED;

    *rotation_speedL=output;
    *rotation_speedR=-output;

}
