#include <math.h>

#include "Wire.h"
#include "I2Cdev.h"
#include "HMC5883L.h"
#include "magneto.h"

//Inicializa el magnetometro y los 3 componentes de la fuerza magnetica que detecta el componente.
magneto::magneto(){
    magnX = 0;
    magnY = 0;
    magnZ = 0;
    Offset = 0;
    Serial.begin(9600);
    Wire.begin();
    magnetometro.initialize();
}

magneto::~magneto(){}

//Retorna el angulo entre la direccion hacia el frente del robot, y el eje X positivo del sistema global.
float magneto::DireccionActual(){ //En Radianes
    float angulo;
    magnetometro.getHeading(&magnX, &magnY, &magnZ);
    angulo = atan2(magnY, magnX);
    if (angulo < 0){angulo = angulo + 6.28319;}
    angulo = angulo - Offset;
    if (angulo < 0){angulo = angulo + 6.28319;}
    else if (angulo > 6.28319){angulo = angulo - 6.28319;}
    return angulo;
}

void magneto::SetOffsetMagnetometro(float AnguloInicial){
    float AnguloMagnetico;
    magnetometro.getHeading(&magnX, &magnY, &magnZ);
    AnguloMagnetico = atan2(magnY, magnX);
    if (AnguloMagnetico < 0){AnguloMagnetico = AnguloMagnetico + 6.28319;}
    Offset = AnguloMagnetico + AnguloInicial - 1.5708;
}