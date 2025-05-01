#ifndef MOTOR_H
#define MOTOR_H

#include "../utils/DRV8833.h"

enum WhichMotor {
    A = 1,
    B = 2,
};

struct Motor {
    int pin1;
    int pin2;
    WhichMotor which;
};

void initMotor(DRV8833 driver, Motor *motor);

#endif // MOTOR_H
