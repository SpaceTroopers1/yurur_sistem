#ifndef CAMERA_KONTROL_H
#define CAMERA_KONTROL_H

void SetServoPosition(TIM_HandleTypeDef* htim, uint32_t channel, uint16_t* position, int step, int servoDirection);
void KameraKontrol();

#endif
