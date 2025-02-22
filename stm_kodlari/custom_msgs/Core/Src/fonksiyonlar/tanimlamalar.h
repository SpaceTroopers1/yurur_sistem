#ifndef TANIMLAMALAR_H
#define TANIMLAMALAR_H

typedef struct joystick {
    float x;
    float y;
    float throttle;
    int cameray;  // İleri/Geri ekseni 1 yada -1 yada 0
    int light;
    int camerax;  // Sağ/Sol ekseni 1 yada -1 yada 0
} joystick;

extern joystick rcjoystick;  // Declare instead of define

typedef enum {
    GROUND_DEFAULT,
    GROUND_SLIPPERY,
    GROUND_ROUGH
} GroundType;

extern GroundType currentGround;  // Declare

typedef struct {
    float targetSpeed;
    float currentSpeed;
    float rampedSpeed;
    float previousErrorSpeed;
    float integralSpeed;
} Wheel;

extern Wheel leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel;

typedef struct {
    float targetDirection;
    float currentDirection;
    float rampedDirection;
    float previousErrorDirection;
    float integralDirection;
} Direction;

extern Direction roverDirection;  // Declare

#endif
