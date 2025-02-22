#include "main.h"
#include "tanimlamalar.h"
//Camera Kontrol
#define DEGREE_MIN 400
#define DEGREE_MAX 2600
#define DEGREE_STEP 10
extern TIM_HandleTypeDef htim2;

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< CAMERA KONTROL >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//--------------------------------------------------------------------------------------------------------------------
void SetServoPosition(TIM_HandleTypeDef* htim, uint32_t channel, uint16_t* position, int step, int servoDirection) {
    if (servoDirection > 0) {
        *position = (*position + step <= DEGREE_MAX) ? *position + step : DEGREE_MAX;
    } else if (servoDirection < 0) {
        *position = (*position - step >= DEGREE_MIN) ? *position - step : DEGREE_MIN;
    }
    __HAL_TIM_SET_COMPARE(htim, channel, *position);
}

void KameraKontrol()
{
    static uint16_t degreey = 2200;
    static uint16_t degreex = 1500;

    degreex = (degreex > DEGREE_MAX) ? DEGREE_MAX : (degreex < DEGREE_MIN) ? DEGREE_MIN : degreex;
    degreey = (degreey > DEGREE_MAX) ? DEGREE_MAX : (degreey < DEGREE_MIN) ? DEGREE_MIN : degreey;


    switch (rcjoystick.cameray)
    {
        case 0: // Dur
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, degreey);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
            break;

        case 1: // Yukarı
            SetServoPosition(&htim2, TIM_CHANNEL_2, &degreey, DEGREE_STEP, 1);
            break;

        case -1: // Aşağı
            SetServoPosition(&htim2, TIM_CHANNEL_2, &degreey, DEGREE_STEP, -1);
            break;

        default:
            // Geçersiz joystick z değeri
            break;
    }

    switch (rcjoystick.camerax)
    {
    	case 0: // Dur
    		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, degreex);
    		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
    		break;

        case 1: // Sağa
            SetServoPosition(&htim2, TIM_CHANNEL_1, &degreex, DEGREE_STEP, 1);
            break;

        case -1: // Sola
            SetServoPosition(&htim2, TIM_CHANNEL_1, &degreex, DEGREE_STEP, -1);
            break;

        default:
            // Geçersiz joystick z değeri
            break;
    }
}

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< LED KONTROL >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//-----------------------------------------------------------------------------------------------------------------------
void LedKontrol(){

	if (rcjoystick.light > 0 )
	{
 	 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
	}
}
