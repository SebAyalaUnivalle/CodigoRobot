#include <math.h>

#include "Wire.h"
#include "I2Cdev.h"
#include "HMC5883L.h"
#include "magneto.h"

//Inicializa el magnetometro y los 3 componentes de la fuerza magnetica que detecta el componente.
magneto::magneto(){
    HMC5883L magnetometro;
    int16_t magnX, magnY, magnZ;
    Wire.begin();
    magnetometro.initialize();
}

magneto::~magneto(){}

//Indica la direccion actual, dada en una distancia desde 0 hasta 2pi radianes.
//Medida con respecto del Norte magnetico, en el sentido del reloj.
float magneto::DireccionActual(){ //En Radianes
    float angulo;
    magnetometro.getHeading(&magnX, &magnY, &magnZ);
    angulo = atan2(magnY, magnX);
    if (angulo < 0){angulo = angulo + 6.28319;}
    return angulo;
}