#ifndef TANIMLAMALAR_H
#define TANIMLAMALAR_H

#include"main.h"

typedef struct joystick {
    float x;
    float y;
    int16_t throttle;
    int16_t cameray;  // İleri/Geri ekseni 1 yada -1 yada 0
    int8_t light;
    int8_t camerax;  // Sağ/Sol ekseni 1 yada -1 yada 0
    int16_t imu_speed;// -1000 ile 1000 arasında rover hızı
    uint8_t gear;
} joystick;

extern joystick rcjoystick;  // Declare instead of define

typedef enum {
    GROUND_DEFAULT,
    GROUND_SLIPPERY,
    GROUND_ROUGH
} GroundType;

extern GroundType currentGround;  // Declare

typedef struct {
    int16_t targetSpeed;
    int16_t currentSpeed;
    int16_t rampedSpeed;
    int16_t previousErrorSpeed;
    int integralSpeed;
} Wheel;

extern Wheel leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel;

typedef struct {
    int16_t targetDirection;
    int16_t currentDirection;
    int16_t rampedDirection;
    int16_t previousErrorDirection;
    int integralDirection;
} Direction;

extern Direction roverDirection;  // Declare

#endif
