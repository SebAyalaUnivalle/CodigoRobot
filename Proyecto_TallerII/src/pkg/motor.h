#define MOTOR_H
#include <Arduino.h>

class motor{
    public:
    motor();
    ~motor();
    void init();
    void Detener();
    void Adelante();
    void GirarDerecha();
    void GirarIzquierda();
};