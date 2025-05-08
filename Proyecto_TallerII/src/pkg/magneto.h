#define MAGNETO_H
#include <SoftwareSerial.h>
#include "../utils/QMC5883LCompass.h"

class magneto {
    private:
    QMC5883LCompass magnetometro;
    int magnX, magnY;
    float Offset;
    public:
    magneto();
    ~magneto();
    void init();
    float DireccionActual();
    void SetOffsetMagnetometro(float RotacionInicial);
};

