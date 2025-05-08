#define MOTOR_H
#include <Arduino.h>

class motor{
    public:
    motor();
    void Detener();
    void Adelante();
    void GirarDerecha();
    void GirarIzquierda();
};