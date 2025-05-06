#define MAGNETO_H
#include "Wire.h"
#include "../utils/I2Cdev.h"
#include "../utils/HMC5883L.h"

class magneto {
    private:
    HMC5883L magnetometro;
    int16_t magnX, magnY, magnZ;
    float Offset;
    public:
    magneto();
    ~magneto();
    float DireccionActual();
    void SetOffsetMagnetometro(float RotacionInicial);
};

