#ifndef MOTOR_HIZ_KONTROL_H
#define MOTOR_HIZ_KONTROL_H

int speedToPWM(uint16_t speed, int16_t max_speed);
void setPWM(TIM_HandleTypeDef *htim, uint32_t channel, int16_t pwmvalue, int16_t imu_speed);
int16_t calculatePID(int *integral, int16_t *previousError, int16_t ramped, int16_t current);
int16_t applyRamp(int16_t current, int16_t target ,uint8_t RAMP_RATE);
int16_t limitSpeedToPWM(int16_t speed, GroundType groundType, int16_t max_speed);
void applySlipControl(Wheel *wheel, int16_t referenceSpeed);
void controlWheel(TIM_HandleTypeDef *htim, uint32_t channel, Wheel *wheel, uint8_t Ramp_Rate, int16_t imu_speed);
void controlRover(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim3, int16_t rotation_speedL, int16_t rotation_speedR, uint16_t throttle, int16_t imu_speed, uint8_t Gear);
void controlDirection(int16_t Y, int16_t X, Direction *direction, int16_t *rotation_speedL, int16_t *rotation_speedR, uint8_t Gear);

#endif
