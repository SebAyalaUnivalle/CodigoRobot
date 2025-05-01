#include "motor.h"
#include "../utils/DRV8833.h"

// [Inicio del motor]
void initMotor(DRV8833 driver, Motor *motor) {
    switch (motor->which) {
    case A:
        driver.attachMotorA(motor->pin1, motor->pin2);
        break;
    case B:
        driver.attachMotorB(motor->pin1, motor->pin2);
        break;
    }
}
