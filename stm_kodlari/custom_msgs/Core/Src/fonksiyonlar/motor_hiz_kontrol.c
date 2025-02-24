#include "main.h"
#include "tanimlamalar.h"
#include <math.h>

#define KP 0.5f
#define KI 0.1f
#define KD 0.05f
#define Direction_KP 0.4f
#define Direction_KI 0.1f
#define Direction_KD 0.03f
#define MAX_SPEED_PWM  1000
#define MIN_SPEED_PWM  0


#define D_RAMP_RATE 50// Her döngüde maksimum hız değişimi D
#define S_RAMP_RATE 100// Her döngüde maksimum hız değişimi S
#define DIRECTION_RAMP_RATE 30 // Her döngüde maksimum yön değişimi yönü 0-3600 arasında derece cinsinden

// Motor hızı sınırlayıcı
#define MAX_SPEED_D  1000
#define MAX_SPEED_S  5000
#define MIN_SPEED_D  -1000
#define MAX_DIRECTION_SPEED  30
#define MIN_DIRECTION_SPEED -30

#define integralLimit MAX_SPEED_PWM/KI


//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< MOTOR KONTROL >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//--------------------------------------------------------------------------------------------------------------------
/* --- Hız-PWM Uyumluluğu --- */
int speedToPWM(uint16_t speed, int16_t max_speed) {
    return (uint16_t)((speed / max_speed) * MAX_SPEED_PWM);
}

/* --- PWM Ayarı --- */
void setPWM(TIM_HandleTypeDef *htim, uint32_t channel, int16_t pwmvalue, int16_t imu_speed) {

	pwmvalue=fabs(pwmvalue);

	if (pwmvalue >= 0 && pwmvalue <= 1000) {
        __HAL_TIM_SET_COMPARE(htim, channel, pwmvalue);
    }

    else if(pwmvalue < 0 ) {
        __HAL_TIM_SET_COMPARE(htim, channel, 0);
    }

    else if(pwmvalue > 1000 ) {
        __HAL_TIM_SET_COMPARE(htim, channel, 1000);
    }

    else {

    }
}

/* --- PID Hesaplama --- */
int16_t calculatePID(int *integral, int16_t *previousError, int16_t ramped, int16_t current) {

	if (*integral > integralLimit) * integral = integralLimit;
	if (*integral < -integralLimit) *integral = -integralLimit;


	int16_t error = ramped - current;
    *integral += error;
    int16_t derivative = error - *previousError;
    *previousError = error;

    int16_t output = KP * error + KI * *integral + KD * derivative;

    if (output > MAX_SPEED_PWM) output = MAX_SPEED_PWM;
    if (output < MIN_SPEED_PWM) output = MIN_SPEED_PWM;

    return output;
}

/* --- Rampalı Hız Kontrolü --- */
int16_t applyRamp(uint16_t current, uint16_t target ,uint8_t RAMP_RATE) {
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
int16_t limitSpeedToPWM(int16_t speed, GroundType groundType, int16_t max_speed) {
	if (speed > max_speed) {speed = max_speed;}/*rgb yak*/
	if (speed < MIN_SPEED_D) {speed = MIN_SPEED_D;}/*rgb yak*/


	switch (groundType) {
        case GROUND_SLIPPERY:
            return speedToPWM(speed, max_speed)*0.5f; // Kaygan zemin: %50 hız
        case GROUND_ROUGH:
            return speedToPWM(speed, max_speed)*0.8f; // Engebeli zemin: %80 hız
        default:
            return speedToPWM(speed, max_speed);        // Normal zemin: Tam hız
		}
}

/* --- Patinaj Kontrolü --- */
void applySlipControl(Wheel *wheel, int16_t referenceSpeed) {
    if (wheel->currentSpeed > referenceSpeed * 1.1f || wheel->currentSpeed < referenceSpeed * 0.9f  ) //%10 sapma
    {
        wheel->targetSpeed = referenceSpeed; // Hedef hızı ayarla
    }
}

/* --- Tekerlek Kontrolü --- */
void controlWheel(TIM_HandleTypeDef *htim, uint32_t channel, Wheel *wheel, uint8_t Ramp_Rate, int16_t imu_speed) {
    wheel->rampedSpeed = applyRamp(wheel->currentSpeed, wheel->targetSpeed, Ramp_Rate);
    int16_t pwmValue = calculatePID(&wheel->integralSpeed, &wheel->previousErrorSpeed, wheel->rampedSpeed, wheel->currentSpeed);
    setPWM(htim, channel, pwmValue, imu_speed);

}

/* --- Yon Kontrolü --- */
void controlDirection(int16_t Y, int16_t X, Direction *direction, int16_t *rotation_speedL, int16_t *rotation_speedR, uint8_t Gear){

	direction->targetDirection = atan2(Y,X) * (180 / M_PI) + 180;

    int16_t error = 	direction->targetDirection - direction->currentDirection;
    while (error > 180) error -= 360;
    while (error < -180) error += 360;

    direction->integralDirection += error;
    int16_t derivative = (error - direction->previousErrorDirection);
    direction->previousErrorDirection = error;

    int16_t output = Direction_KP * error + Direction_KI * direction->integralDirection + Direction_KD * derivative;


    if (output > MAX_DIRECTION_SPEED) output = MAX_DIRECTION_SPEED;
    if (output < MIN_DIRECTION_SPEED) output = MIN_DIRECTION_SPEED;
switch(Gear){

	case 3://D
		*rotation_speedL=+output;
		*rotation_speedR=-output;
		break;
	case 6://T
		*rotation_speedL=+output;
		*rotation_speedR=-output;
		break;
	case 2://R
		*rotation_speedL=-output;
		*rotation_speedR=+output;
		break;
	case 4://S
		*rotation_speedL=+output;
		*rotation_speedR=-output;
		break;
	}


}


/* --- Rover Kontrol Fonksiyonu --- */
void controlRover(TIM_HandleTypeDef *htim1,TIM_HandleTypeDef *htim3 , int16_t rotation_speedL, int16_t rotation_speedR, uint16_t throttle, int16_t imu_speed, uint8_t Gear) {

	switch (Gear)
	{
        case 5://P

        	setPWM(htim1, TIM_CHANNEL_1, 0, imu_speed);
            setPWM(htim1, TIM_CHANNEL_2, 0, imu_speed);
            setPWM(htim1, TIM_CHANNEL_3, 0, imu_speed);
            setPWM(htim1, TIM_CHANNEL_4, 0, imu_speed);

            setPWM(htim3, TIM_CHANNEL_1, 0, imu_speed);
            setPWM(htim3, TIM_CHANNEL_2, 0, imu_speed);
            setPWM(htim3, TIM_CHANNEL_3, 0, imu_speed);
            setPWM(htim3, TIM_CHANNEL_4, 0, imu_speed);

            /*


            Fren yapınca buraya eklenecek


            */

            break;
        case 1://N

        	setPWM(htim1, TIM_CHANNEL_1, 0, imu_speed);
            setPWM(htim1, TIM_CHANNEL_2, 0, imu_speed);
            setPWM(htim1, TIM_CHANNEL_3, 0, imu_speed);
            setPWM(htim1, TIM_CHANNEL_4, 0, imu_speed);

            setPWM(htim3, TIM_CHANNEL_1, 0, imu_speed);
            setPWM(htim3, TIM_CHANNEL_2, 0, imu_speed);
            setPWM(htim3, TIM_CHANNEL_3, 0, imu_speed);
            setPWM(htim3, TIM_CHANNEL_4, 0, imu_speed);


            break;
        case 3://D

            leftFrontWheel.targetSpeed = limitSpeedToPWM(throttle + rotation_speedL, currentGround, MAX_SPEED_D);
            leftBackWheel.targetSpeed = limitSpeedToPWM(throttle + rotation_speedL, currentGround, MAX_SPEED_D);
            rightFrontWheel.targetSpeed = limitSpeedToPWM(throttle + rotation_speedR, currentGround, MAX_SPEED_D);
            rightBackWheel.targetSpeed = limitSpeedToPWM(throttle + rotation_speedR, currentGround, MAX_SPEED_D);

            // Patinaj kontrolü uygula (örneğin, ön tekerlekleri referans al)
            applySlipControl(&leftFrontWheel, imu_speed);
            applySlipControl(&rightFrontWheel, imu_speed);
            applySlipControl(&leftBackWheel, imu_speed);
            applySlipControl(&rightBackWheel, imu_speed);

            // Tekerlekleri kontrol et

            controlWheel(htim1, TIM_CHANNEL_1, &leftFrontWheel, D_RAMP_RATE, imu_speed);
            controlWheel(htim1, TIM_CHANNEL_2, &leftBackWheel, D_RAMP_RATE, imu_speed);
            controlWheel(htim1, TIM_CHANNEL_3, &rightFrontWheel, D_RAMP_RATE, imu_speed);
            controlWheel(htim1, TIM_CHANNEL_4, &rightBackWheel, D_RAMP_RATE, imu_speed);

            setPWM(htim3, TIM_CHANNEL_1, 0, imu_speed);
            setPWM(htim3, TIM_CHANNEL_2, 0, imu_speed);
            setPWM(htim3, TIM_CHANNEL_3, 0, imu_speed);
            setPWM(htim3, TIM_CHANNEL_4, 0, imu_speed);

            break;
        case 2://R

            leftFrontWheel.targetSpeed = limitSpeedToPWM(-throttle + rotation_speedL, currentGround, MAX_SPEED_D);
            leftBackWheel.targetSpeed = limitSpeedToPWM(-throttle + rotation_speedL, currentGround, MAX_SPEED_D);
            rightFrontWheel.targetSpeed = limitSpeedToPWM(-throttle + rotation_speedR, currentGround, MAX_SPEED_D);
            rightBackWheel.targetSpeed = limitSpeedToPWM(-throttle + rotation_speedR, currentGround, MAX_SPEED_D);

            // Patinaj kontrolü uygula (örneğin, ön tekerlekleri referans al)
            applySlipControl(&leftFrontWheel, imu_speed);
            applySlipControl(&rightFrontWheel, imu_speed);
            applySlipControl(&leftBackWheel, imu_speed);
            applySlipControl(&rightBackWheel, imu_speed);
            // Tekerlekleri kontrol et

            controlWheel(htim3, TIM_CHANNEL_1, &leftFrontWheel, D_RAMP_RATE, imu_speed);
            controlWheel(htim3, TIM_CHANNEL_2, &rightFrontWheel, D_RAMP_RATE, imu_speed);
            controlWheel(htim3, TIM_CHANNEL_3, &leftBackWheel, D_RAMP_RATE, imu_speed);
            controlWheel(htim3, TIM_CHANNEL_4, &rightBackWheel, D_RAMP_RATE, imu_speed);

            setPWM(htim1, TIM_CHANNEL_1, 0, imu_speed);
            setPWM(htim1, TIM_CHANNEL_2, 0, imu_speed);
            setPWM(htim1, TIM_CHANNEL_3, 0, imu_speed);
            setPWM(htim1, TIM_CHANNEL_4, 0, imu_speed);

            break;
        case 4://S

            leftFrontWheel.targetSpeed = limitSpeedToPWM(throttle + rotation_speedL, currentGround, MAX_SPEED_S);
            leftBackWheel.targetSpeed = limitSpeedToPWM(throttle + rotation_speedL, currentGround, MAX_SPEED_S);
            rightFrontWheel.targetSpeed = limitSpeedToPWM(throttle + rotation_speedR, currentGround, MAX_SPEED_S);
            rightBackWheel.targetSpeed = limitSpeedToPWM(throttle + rotation_speedR, currentGround, MAX_SPEED_S);

            // Patinaj kontrolü uygula (örneğin, ön tekerlekleri referans al)
            applySlipControl(&leftFrontWheel, imu_speed);
            applySlipControl(&rightFrontWheel, imu_speed);
            applySlipControl(&leftBackWheel, imu_speed);
            applySlipControl(&rightBackWheel, imu_speed);
            // Tekerlekleri kontrol et

            controlWheel(htim1, TIM_CHANNEL_1, &leftFrontWheel, S_RAMP_RATE, imu_speed);
            controlWheel(htim1, TIM_CHANNEL_2, &rightFrontWheel, S_RAMP_RATE, imu_speed);
            controlWheel(htim1, TIM_CHANNEL_3, &leftBackWheel, S_RAMP_RATE, imu_speed);
            controlWheel(htim1, TIM_CHANNEL_4, &rightBackWheel, S_RAMP_RATE, imu_speed);

            setPWM(htim3, TIM_CHANNEL_1, 0, imu_speed);
            setPWM(htim3, TIM_CHANNEL_2, 0, imu_speed);
            setPWM(htim3, TIM_CHANNEL_3, 0, imu_speed);
            setPWM(htim3, TIM_CHANNEL_4, 0, imu_speed);

            break;

        case 6://T

            leftFrontWheel.targetSpeed = limitSpeedToPWM(rotation_speedL, currentGround, MAX_SPEED_D);
            leftBackWheel.targetSpeed = limitSpeedToPWM(rotation_speedL, currentGround, MAX_SPEED_D);
            rightFrontWheel.targetSpeed = limitSpeedToPWM(rotation_speedR, currentGround, MAX_SPEED_D);
            rightBackWheel.targetSpeed = limitSpeedToPWM(rotation_speedR, currentGround, MAX_SPEED_D);

            // Patinaj kontrolü uygula (örneğin, ön tekerlekleri referans al)

            applySlipControl(&leftBackWheel, leftFrontWheel.currentSpeed);
            applySlipControl(&rightBackWheel, rightFrontWheel.currentSpeed);

            // Tekerlekleri kontrol et
           if(rotation_speedL>0 && rotation_speedR<0)
            {
                controlWheel(htim1, TIM_CHANNEL_1, &leftFrontWheel, D_RAMP_RATE, imu_speed);
                controlWheel(htim3, TIM_CHANNEL_2, &rightFrontWheel, D_RAMP_RATE, imu_speed);
                controlWheel(htim1, TIM_CHANNEL_3, &leftBackWheel, D_RAMP_RATE, imu_speed);
                controlWheel(htim3, TIM_CHANNEL_4, &rightBackWheel, D_RAMP_RATE, imu_speed);

                setPWM(htim3, TIM_CHANNEL_1, 0, imu_speed);
                setPWM(htim1, TIM_CHANNEL_2, 0, imu_speed);
                setPWM(htim3, TIM_CHANNEL_3, 0, imu_speed);
                setPWM(htim1, TIM_CHANNEL_4, 0, imu_speed);

            }
            else if(rotation_speedL<0 && rotation_speedR>0)
            {
                controlWheel(htim3, TIM_CHANNEL_1, &leftFrontWheel, D_RAMP_RATE, imu_speed);
                controlWheel(htim1, TIM_CHANNEL_2, &rightFrontWheel, D_RAMP_RATE, imu_speed);
                controlWheel(htim3, TIM_CHANNEL_3, &leftBackWheel, D_RAMP_RATE, imu_speed);
                controlWheel(htim1, TIM_CHANNEL_4, &rightBackWheel, D_RAMP_RATE, imu_speed);

                setPWM(htim1, TIM_CHANNEL_1, 0, imu_speed);
                setPWM(htim3, TIM_CHANNEL_2, 0, imu_speed);
                setPWM(htim1, TIM_CHANNEL_3, 0, imu_speed);
                setPWM(htim3, TIM_CHANNEL_4, 0, imu_speed);

            }

            else
            {
            	setPWM(htim1, TIM_CHANNEL_1, 0, imu_speed);
                setPWM(htim1, TIM_CHANNEL_2, 0, imu_speed);
                setPWM(htim1, TIM_CHANNEL_3, 0, imu_speed);
                setPWM(htim1, TIM_CHANNEL_4, 0, imu_speed);

                setPWM(htim3, TIM_CHANNEL_1, 0, imu_speed);
                setPWM(htim3, TIM_CHANNEL_2, 0, imu_speed);
                setPWM(htim3, TIM_CHANNEL_3, 0, imu_speed);
                setPWM(htim3, TIM_CHANNEL_4, 0, imu_speed);
            }

            break;

	}

	// Hedef hızları joystick'e göre ayarla

}
